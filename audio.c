/*
 * MyPod - dsPIC Based MP3 Player
 * Copyright (C) 2014 Fernando Rodriguez (support@fernansoft.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 3 as 
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#define SAMPLE_SPEED_16KSPS		/* sample at 16 khz */
#define USE_DOUBLE_BUFFERS

#if defined(__dsPIC33E__)
	#define SUPPORT_MP3
	#define USE_DMA_MP3
	//#define MP3_OUTPUT_TO_FILE
#endif

#include <dspic_hal/common.h>
#include <dspic_hal/dma.h>
#include <dspic_hal/rtc.h>
#include <dspic_hal/spi.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "audio.h"
#include <ili9341/ili9341.h>
#include <sdlib/sd.h>

#if defined(SUPPORT_MP3)
	#include <mp3.h>
#endif
/*
// these are used for configuring the PWM
// module and ADC
*/
#if defined(__dsPIC33E__)
	#define FCY		70000000UL
	#define FPWM 	274509UL
#else
	#define FCY  	40000000UL
	#define FPWM 	156862UL
#endif
#define FSMP 	32000UL
#define PREC	8
#define TPWM 	((unsigned int)(FCY / FPWM))
#define TSMP	((unsigned int)(FCY / FSMP))


/*
// when sampling at 16ksps we can use smaller buffers. In this mode this
// sample can run on devices with 8KB RAM
*/
#if defined(SAMPLE_SPEED_16KSPS)
	#define AUDIO_BUFFER_SIZE_MP3	(1152 * 2)
	#define AUDIO_BUFFER_SIZE		512
	#define DMA_BUFFER_SIZE			16
#else
	#define AUDIO_BUFFER_SIZE		6144	/* size of each buffer must be a multiple of sector size */
	#define DMA_BUFFER_SIZE			16		/* size of DMA buffer must be a power of two >= 4 
												with double buffering smaller values give better performance
												when double buffering is disabled it must be a multiple of sector size */
#endif


/*
// audio buffers
*/
static unsigned char __attribute__((section(".audio_buffer"), space(xmemory))) audio_buffer[(AUDIO_BUFFER_SIZE * 2)];
static unsigned char audio_buffer_ready[2];
#define audio_buffer_a 		(audio_buffer)
#define audio_buffer_b		(audio_buffer + AUDIO_BUFFER_SIZE)

/*
// dma buffer for ADC samples
*/
#if defined(__dsPIC33E__)
	#define ADC_BUFFER __attribute__((section(".adc_dma_buffer")))
#else
	#define ADC_BUFFER __attribute__((space(dma), section(".adc_dma_buffer")))
#endif
static unsigned char ADC_BUFFER adc_buffer[DMA_BUFFER_SIZE];

static SM_FILE* audio_file;
static uint16_t audio_error;
static AUDIO_PLAYBACK_COMPLETED audio_callback;
static char audio_invoke_callback;
static int audio_volume_level = 0x1;

/*
// variables used for recording
*/
char audio_waiting_for_data;
char __attribute__((near)) audio_recording = 0;
static unsigned char __attribute__((near)) audio_active_buffer;
static unsigned char __attribute__((near)) audio_dma_active_buffer;
static unsigned int  __attribute__((__near__)) audio_isr_buffer_index = AUDIO_BUFFER_SIZE / 2;
#define WAIT_FOR_DATA
#define COUNT_LOST_CHUNKS
#if !defined(NDEBUG)
	//#define HALT_ON_LOST_CHUNK
#endif

/*
// these are used by the ADC interrupt
*/
unsigned char* __attribute__((near)) p_adc_buffer = adc_buffer;
unsigned char* __attribute__((near)) p_audio_buffer = audio_buffer;

/*
// variables used for playback
*/
uint32_t bytes_read;
char __attribute__((near)) need_more_audio = 0;
char __attribute__((near)) audio_playback = 0;
char __attribute__((near)) audio_io_in_progress = 0;

/*
// volume level ratios
*/
static const int audio_volume_levels[] =
{
	0,
	32768*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L,
	32768*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L,
	32768*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L,
	32768*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L,
	32768*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L,
	32768*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L,
	32768*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L,
	32768*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L,
	32768*0.71L*0.71L*0.71L*0.71L*0.71L*0.71L,
	32768*0.71L*0.71L*0.71L*0.71L*0.71L,
	32768*0.71L*0.71L*0.71L*0.71L,
	32768*0.71L*0.71L*0.71L,
	32768*0.71L*0.71L, 
	32768*0.71L,
	32767
};

/*
// dither for truncating 16-bit samples
*/

#if defined(SUPPORT_MP3)
	static unsigned int __attribute__((section(".mp3_output"))) mp3_output[2][1152];
	static __eds__ unsigned char __attribute__((section(".mp3_audio_buffer_b"), space(ymemory), eds)) audio_buffer_b_mp3[(AUDIO_BUFFER_SIZE_MP3) + 32];
	static __eds__ unsigned char __attribute__((section(".mp3_audio_buffer_a"), space(ymemory), eds)) audio_buffer_a_mp3[(AUDIO_BUFFER_SIZE_MP3) + 32];
	//static __eds__ unsigned char __attribute__((section(".mp3_buffer_guard"), space(ymemory), eds)) mp3_guard[(AUDIO_BUFFER_SIZE_MP3) + 32];
	unsigned char __attribute__((section(".mp3"))) mp3_buffer[512];
	static char mp3_data_ready = 0;
	static unsigned int audio_buffer_index = 0;
	static char mp3_playback_started = 0;
	static unsigned int mp3_samplerate;
#endif

/*
// used for counting lost chunkds while sampling
*/
#if defined(COUNT_LOST_CHUNKS)
	uint16_t __attribute__((near)) audio_lost_chunks;
	uint32_t __attribute__((near)) audio_good_chunks;
#endif

/*
// prototypes
*/
static void init_adc(void);
static void init_pwm(void);
static void init_dsp(void);
static void audio_stream_callback(SM_FILE* f, uint16_t* result, unsigned char** buffer, uint16_t* response);
static void audio_read_callback(SM_FILE* f, uint16_t* result);
static void audio_play_file_raw(SM_FILE* file, AUDIO_PLAYBACK_COMPLETED callback);

/*
// mp3 
*/
#if defined(SUPPORT_MP3)
	static void audio_play_file_mp3(SM_FILE* file, AUDIO_PLAYBACK_COMPLETED callback);
	static void audio_play_mp3_callback(void* context, uint16_t* result);
#endif

/*
// imports
*/
extern void truncate(void* audio_data,  int volume_level, unsigned int n);
extern void truncate_eds(__eds__ void* audio_data, int volume_level, unsigned int n);
/*
// initialize audio driver
*/
void audio_init(void)
{
	init_adc();	 
	init_pwm();	 
	init_dsp();
	/*
	// initialize mp3 decoder state machine
	*/
	#if defined(SUPPORT_MP3)
	mp3_sm_init(2);
	mp3_set_output_buffer(mp3_output);
	#endif
}

/*
// initialize DSP engine
*/
static void init_dsp(void)
{
	/*
	// set DSP engine for signed integer
	// super saturation mode and convergent
	// rounding
	*/
	//CORCON = 0x00F5;

	CORCONbits.SATA = 1;
	CORCONbits.SATB = 1;
	CORCONbits.SATDW = 1;
	CORCONbits.ACCSAT = 1;	// 0 = 1.31
	CORCONbits.RND = 1;
	CORCONbits.IF = 1;
}

/*
// initialized ADC module
*/
static void init_adc(void)
{
	AD1CON1bits.ADON = 0;
	AD1CON1bits.ADSIDL = 1;
	AD1CON1bits.ADDMABM = 1;
	AD1CON1bits.AD12B = 1;		/* 12-bit operation */
	AD1CON1bits.FORM = 1;		/* signed integer */
	AD1CON1bits.SSRC = 7;		/* automatic sampling */
	AD1CON1bits.ASAM = 1;		/* begin sampling after conversion */
	AD1CON2bits.VCFG = 0;		/* use VDD and VSS as reference */
	AD1CON2bits.CSCNA = 0;	
	AD1CON2bits.CHPS = 0;		/* channel 0 */
	AD1CON2bits.SMPI = 0;		/* increment DMA address after each sample */
	AD1CON2bits.BUFM = 0;
	AD1CON2bits.ALTS = 0;
	/*
	// Sample Time = Sampling Time (TSamp) + Conversion Time (TConv)
	//
	// 32 Ksps = (FCY / 32000) * TCY = (40000000 / 32000) * TCY = 1250 * TCY
	//
	// TAD = (ADCS + 1) * TCY = 50 * TCY
	// TCONV = 14 * TAD	= 14 * 50 * TCY =  700 * TCY
	// TSAMP = SAMC * TAD = 11 * 50 * TCY = 550 * TCY
	// T = TSAMP + TCONV = 1250 * TCY
	//
	*/
	#if defined(SAMPLE_SPEED_16KSPS)
		#if defined(__dsPIC33F__)
			AD1CON3bits.ADRC = 0; 		/* use system clock */
			AD1CON3bits.SAMC = 11;		/* 11 TAD */
			AD1CON3bits.ADCS = 50;		/* TAD = (ADCS + 1) * Tcy */
			AD1CON4bits.DMABL = 7;		/* 256 bytes DMA buffer */
		#else
			//#error !!
		#endif
	#else
		#if defined(__dsPIc33F__)
			AD1CON3bits.ADRC = 0; 		/* use system clock */
			AD1CON3bits.SAMC = 11;		/* 11 TAD */
			AD1CON3bits.ADCS = 5;		/* TAD = (ADCS + 1) * Tcy */
			AD1CON4bits.DMABL = 7;		/* 256 bytes DMA buffer */
		#else
			#error !!
		#endif
	#endif	
	/*
	// set AN9/RB15 as analog input. everything 
	// else is digital
	*/
	#if defined(EXPLORER16)
		AD1PCFGH = 0xffff;
		AD2PCFGL = 0Xffff;	
		AD1PCFGL = 0b1111111111011111;
		/*
		// AN5 is analog input
		*/	
		AD1CHS0bits.CH0NB = 0;		/* negative input is VSS */
		AD1CHS0bits.CH0SB = 5;		/* positive input is AN5*/
		AD1CHS0bits.CH0NA = 0;		/* negative input is VSS */
		AD1CHS0bits.CH0SA = 5;		/* positive input is AN5 */
		IO_PIN_SET_AS_INPUT(B, 5);
	#else
		#if defined(__dsPIC33E__)
			ANSELAbits.ANSA0 = 1;
		#else
			AD1PCFGL = 0b1111111111111110;
		#endif
		/*
		// AN9 is analog input
		*/	
		AD1CHS0bits.CH0NB = 0;		/* negative input is VSS */
		AD1CHS0bits.CH0SB = 0;		/* positive input is AN9 */
		AD1CHS0bits.CH0NA = 0;		/* negative input is VSS */
		AD1CHS0bits.CH0SA = 0;		/* positive input is AN9 */
		IO_PIN_SET_AS_INPUT(A, 0);
	#endif

	/*
	// configure dma channel
	*/
	DMA_CHANNEL_DISABLE(DMA_GET_CHANNEL(2));
	DMA_SET_CHANNEL_WIDTH(DMA_GET_CHANNEL(2), DMA_CHANNEL_WIDTH_WORD);
	DMA_SET_INTERRUPT_MODE(DMA_GET_CHANNEL(2), DMA_INTERRUPT_MODE_FULL);
	DMA_SET_NULL_DATA_WRITE_MODE(DMA_GET_CHANNEL(2), 0);
	DMA_SET_ADDRESSING_MODE(DMA_GET_CHANNEL(2), DMA_ADDR_REG_IND_W_POST_INCREMENT);
	DMA_SET_MODE(DMA_GET_CHANNEL(2), DMA_MODE_CONTINUOUS_W_PING_PONG);

	DMA_CHANNEL_CLEAR_INTERRUPT_FLAG(DMA_GET_CHANNEL(2));
	DMA_CHANNEL_ENABLE_INTERRUPT(DMA_GET_CHANNEL(2));
	DMA_CHANNEL_SET_INT_PRIORITY(DMA_GET_CHANNEL(2), 0x7);
	/*
	// configure modulo addressing
	*/
	#if defined(USE_DOUBLE_BUFFERS)
		XMODSRT = (int) audio_buffer;
		XMODEND = (int) (audio_buffer + ((AUDIO_BUFFER_SIZE * 2) - 1));
		YMODSRT = (int) adc_buffer;
		YMODEND = (int) (adc_buffer + ((DMA_BUFFER_SIZE) - 1));
		MODCONbits.XWM = 4;		// w4
		MODCONbits.YWM = 11;		// w11
	#endif
}

/*
//
*/
void audio_set_volume(unsigned char volume_level)
{
	_ASSERT(volume_level >= 0 && volume_level < 16);
	/*
	// convert volume level to attenuation level
	*/
	audio_volume_level = audio_volume_levels[volume_level];
}


/*
// initialize OC1 module for PWM
*/
static void init_pwm(void)
{
	/*
	// this timer is the PWM rate
	*/
	T2CONbits.TON = 0;			/* Stop any 16/32-bit Timer2 operation */
	T2CONbits.T32 = 0;			/* Enable 32-bit Timer mode */
	T2CONbits.TCS = 0;			/* Select internal instruction cycle clock */
	T2CONbits.TGATE = 0;		/* Disable Gated Timer mode */
	T2CONbits.TCKPS = 0x0;		/* Select 1:1 Prescaler */
	TMR2 = 0x00;				/* Clear 32-bit Timer (msw) */
	#if defined(__dsPIC33E__)
		PR2 = TPWM;	// 2 mhz
	#else
		PR2 = TPWM;					/* 600K @ 40 mips */
	#endif
	//PR2 += 128;					/* add some headroom */
	/*
	// this one is the sample rate
	*/
	T3CONbits.TON = 0;			/* Stop any 16/32-bit Timer2 operation */
	T3CONbits.TCS = 0;			/* Select internal instruction cycle clock */
	T3CONbits.TGATE = 0;		/* Disable Gated Timer mode */
	T3CONbits.TCKPS = 0x0;		/* Select 1:1 Prescaler */
	TMR3 = 0x00;				/* Clear 32-bit Timer (msw) */
	PR3 = TSMP;					/* 50K @ 40 mips */
	/*
	// configure OC2 for PWM
	*/
	OC1R = 0; //fpwm >> 1;			/* 50% duty cycle */
	#if defined(__dsPIC33E__)
		OC1CON1bits.OCSIDL = 0;
		OC1CON1bits.OCTSEL = 0;	/* timer2 */
		OC1CON1bits.ENFLTA = 0;
		OC1CON1bits.ENFLTB = 0;
		OC1CON1bits.TRIGMODE = 0;
		OC1CON2bits.OC32 = 0; 
		OC1CON2bits.OCTRIG = 0;
		OC1CON2bits.SYNCSEL = 0b1100;
		OC1CON2bits.TRIGSTAT = 0;
		OC1CON2bits.OCTRIS = 0;
		OC1CON1bits.OCM = 0b110;
		OC1RS = TPWM;
	#else
		OC1CONbits.OCSIDL = 0;
		OC1CONbits.OCFLT = 0;
		OC1CONbits.OCTSEL = 0;		/* timer2 */
		OC1CONbits.OCM = 0b110;		/* PWM no fault */	
		OC1RS = 0;
	#endif	
	/*
	// enable tmr3 interrupts
	*/
	IFS0bits.T3IF = 0;
	IEC0bits.T3IE = 0;
	/*
	// start PWM
	*/
	T2CONbits.TON = 1;			/* Stop any 16/32-bit Timer2 operation */
	T3CONbits.TON = 0;			/* Stop any 16/32-bit Timer2 operation */
}	

/*
// creates a file for sampled data and starts sampling
*/
void start_sampling()
{
	/*
	// open file for stream io
	*/
	audio_error = sm_file_open(audio_file, "x:\\audio.pcm", 
		SM_FILE_ACCESS_CREATE | SM_FILE_ACCESS_OVERWRITE | SM_FILE_FLAG_NO_BUFFERING | SM_FILE_FLAG_OPTIMIZE_FOR_FLASH);
	if (audio_error)
	{
		#if defined(EXPLORER16)
			printf("Error opening file: 0x%x", audio_error);
		#endif
		
		_ASSERT(0);
		return;
	}	
	
	audio_record(audio_file);
}

/*
// record audio from ADC and save to file
*/
void audio_record(SM_FILE* file)
{
	uint16_t ipl;
	/*
	// allocate 256 MB for file
	*/
	audio_error = sm_file_alloc(file, 1024L * 1024 * 5);
	if (audio_error)
	{
		_ASSERT(0);
		sm_file_close(file);
		return;
	}
	
	audio_recording = 0;
	audio_active_buffer = 0;
	audio_dma_active_buffer = 0;
	#if defined(COUNT_LOST_CHUNKS)
		audio_lost_chunks = 0;
		audio_good_chunks = 0;
	#endif
	#if defined(USE_DOUBLE_BUFFERS)
		p_adc_buffer = adc_buffer;
		p_audio_buffer = audio_buffer;
	#endif
	audio_waiting_for_data = 1;
	/*
	// enable DMA channel and ADC
	*/
	DMA_SET_CHANNEL_DIRECTION(DMA_GET_CHANNEL(2), DMA_DIR_PERIPHERAL_TO_RAM);
	DMA_SET_PERIPHERAL(DMA_GET_CHANNEL(2), 0xD);
	DMA_SET_PERIPHERAL_ADDRESS(DMA_GET_CHANNEL(2), &ADC1BUF0);
	DMA_SET_TRANSFER_LENGTH(DMA_GET_CHANNEL(2), (DMA_BUFFER_SIZE / 4) - 1);
	DMA_SET_BUFFER_A(DMA_GET_CHANNEL(2), adc_buffer);
	DMA_SET_BUFFER_B(DMA_GET_CHANNEL(2), adc_buffer + (DMA_BUFFER_SIZE / 2));
	DMA_CHANNEL_ENABLE(DMA_GET_CHANNEL(2));
	AD1CON1bits.ADON = 1;

	audio_recording = 10;
	/*
	// wait for the first round of data
	*/
	while (1)
	{
		SET_AND_SAVE_CPU_IPL(ipl, 7);
		if (audio_active_buffer != audio_dma_active_buffer)
		{
			audio_waiting_for_data = 0;
			audio_active_buffer ^= 1;
			RESTORE_CPU_IPL(ipl);
			break;
		}
		RESTORE_CPU_IPL(ipl);
	}

	/*
	// begin writing. This will continue to write in chunks of 4 KB until the
	// response argument of the callback function (file_write_stream_callback) is set
	// to stop
	*/
	audio_error = sm_file_write_stream(file, audio_buffer_a, AUDIO_BUFFER_SIZE, &audio_error, (SM_STREAM_CALLBACK) &audio_stream_callback, file);
	if (audio_error != FILESYSTEM_OP_IN_PROGRESS)
	{
		#if defined(EXPLORER16)
			printf("Error starting stream: 0x%x", audio_error);
		#else
			_ASSERT(0);
		#endif
		DMA_CHANNEL_DISABLE(DMA_GET_CHANNEL(2));
		AD1CON1bits.ADON = 0;
		audio_recording = 0;
		#if defined(EXPLORER16)
			IO_PIN_WRITE(A, 4, 0);
		#else
			//IO_PIN_WRITE(B, 14, 0);
		#endif
		/*
		// close file
		*/
		sm_file_close(file);
		return;
	}
}

/*
// this function is called by smlib when it is ready to 
// write more data to the file, we just set the *response
// parameter to READY when the buffer is loaded or if the
// data is not ready we set it to SKIP and the file system
// will call back as soon as it is ready to write again
// which is almost immediately unless there are more IO requests
// pending.
*/
void audio_stream_callback(SM_FILE* f, uint16_t* result, unsigned char** buffer, uint16_t* response)
{
	
	static int i;
	
	if (*result == FILESYSTEM_AWAITING_DATA)
	{
		if (audio_recording)
		{
			audio_waiting_for_data = 1;
			/*
			// wait for data
			*/
			#if defined(WAIT_FOR_DATA)
			uint16_t ipl;
			char cmp;
			do
			{
				ili9341_do_processing();
				SET_AND_SAVE_CPU_IPL(ipl, 7);
				cmp = (audio_active_buffer == audio_dma_active_buffer);
				RESTORE_CPU_IPL(ipl);
			}	
			while (cmp);
			#else
			if (audio_active_buffer == audio_dma_active_buffer)
			{
				*response = FAT_STREAMING_RESPONSE_SKIP;
			}	
			else					
			#endif
			{
				/*
				// set the stream buffer
				*/
				#if defined(USE_DOUBLE_BUFFERS)
					*buffer = (audio_active_buffer == 0) ? audio_buffer_a : audio_buffer_b;
					/*
					// scale up to 16 bits
					*/
					for (i = 0; i < AUDIO_BUFFER_SIZE / 2; i++)
					{
						/*
						long sample = (long)((int*) *buffer)[i];
						sample *= 56;
						if (sample > 32767)
						{
							sample = 32767;
						}
						else if (sample < -32767)
						{
							sample = -32767;
						}
						*/
						((int*) *buffer)[i] *= 48; //(int) sample; // 56; 
					}	
				#else
				*buffer = (audio_active_buffer == 0) ? adc_buffer : adc_buffer + (DMA_BUFFER_SIZE / 2);
				/*
				// scale up to 16 bits
				*/
				for (i = 0; i < DMA_BUFFER_SIZE / 2; i++)
				{
					buffer[i] = buffer[i] * 0x10;	
				}	
				#endif
				/*
				// toggle the active buffer and set response to READY
				*/				
				audio_active_buffer ^= 1;
				#if defined(COUNT_LOST_CHUNKS)
				audio_good_chunks++;
				#endif
				audio_waiting_for_data = 0;
				*response = FAT_STREAMING_RESPONSE_READY;
			}	
		}
		else
		{
			AD1CON1bits.ADON = 0;
			#if defined(EXPLORER16)
				IO_PIN_WRITE(A, 4, 0);
			#else
				//IO_PIN_WRITE(B, 14, 0);
			#endif
			DMA_CHANNEL_DISABLE(DMA_GET_CHANNEL(2));
			*response = FAT_STREAMING_RESPONSE_STOP;
		}	
	}
	else
	{
		if (*result != FILESYSTEM_SUCCESS)
		{
			_ASSERT(0);
			AD1CON1bits.ADON = 0;
			DMA_CHANNEL_DISABLE(DMA_GET_CHANNEL(2));
			audio_recording = 0;
		}
		sm_file_close(f);
		
		//start_playback();
	}	
}

/*
// stop audio recording
*/
void audio_stop_recording(void)
{
	audio_recording = 0;
}

/*
// start playing an audio file
*/
void audio_play_file(SM_FILE* file, enum AUDIO_FILE_TYPE type, AUDIO_PLAYBACK_COMPLETED callback)
{
	switch (type)
	{
		case AUDIO_FILE_RAW: audio_play_file_raw(file, callback); break;
		case AUDIO_FILE_MP3: 
			#if defined(SUPPORT_MP3)
			audio_play_file_mp3(file, callback); 
			break;
			#endif
		case AUDIO_FILE_UNKNOWN:
			audio_file = file;
			need_more_audio = 1;
			audio_invoke_callback = 1;
			break;		
	}
}

/*
// plays an audio file
*/
static void audio_play_file_raw(SM_FILE* file, AUDIO_PLAYBACK_COMPLETED callback)
{
	/*
	// save file handle and callback function
	*/
	audio_file = file;
	audio_callback = callback;
	/*
	// initialize playback variables
	*/
	need_more_audio = 0;
	audio_active_buffer = 0;
	audio_playback = 1;
	audio_invoke_callback = 0;
	audio_buffer_ready[0] = 1;
	audio_buffer_ready[1] = 0;
	audio_isr_buffer_index = AUDIO_BUFFER_SIZE / 2;
	
	#if defined(SUPPORT_MP3)
	mp3_playback_started = 0;
	#endif
	
	/*
	// fill 1st buffer with audio data
	*/ 
	audio_error = sm_file_read(file, audio_buffer_a, AUDIO_BUFFER_SIZE, &bytes_read);
	if (audio_error != SM_SUCCESS)
	{
		_ASSERT(0);
		audio_playback = 0;
		need_more_audio = 1;
		audio_invoke_callback = 1;
		sm_file_close(file);
		return;	
	}
	/*
	// truncate
	*/
	truncate(audio_buffer_a, audio_volume_level, bytes_read >> 1);
	/*
	// begin filling 2nd buffer
	*/
	audio_error = sm_file_read_async(file, audio_buffer_b, AUDIO_BUFFER_SIZE, &bytes_read, &audio_error, (SM_ASYNC_CALLBACK) &audio_read_callback, file);
	if (audio_error != FILESYSTEM_OP_IN_PROGRESS)
	{
		_ASSERT(0);
		audio_playback = 0;
		need_more_audio = 1;
		audio_invoke_callback = 1;
		/*
		// close file
		*/
		sm_file_close(file);
		return;
	}
	/*
	// set the IO in progress flag
	*/
	audio_io_in_progress = 1;
	/*
	// PWM module
	*/
	#if defined(__dsPIC33E__)
		OC1R = 0;
	#else
		OC1R = 0;
		OC1RS = 0;
	#endif
	/*
	// enable tmr3 interrupts
	*/
	IFS0bits.T3IF = 0;
	IEC0bits.T3IE = 1;
	/*
	// start playback
	*/	
	PR3 = TSMP;
	T2CONbits.TON = 1;
	T3CONbits.TON = 1;
}

/*
// this function is called by the FAT driver when an
// asynchronous read completes
*/
void audio_read_callback(SM_FILE* f, uint16_t* result)
{
	/*
	// clear IO in progress flag
	*/
	audio_io_in_progress = 0;
	
	if (bytes_read)
	{
		if (audio_active_buffer == 0)
		{
			truncate
			(
				audio_buffer_a , 
				audio_volume_level, 
				bytes_read >> 1
			);
			audio_buffer_ready[0] = 1;
		}
		else
		{
			truncate
			(
				audio_buffer_b, 
				audio_volume_level, 
				bytes_read >> 1 
			);
			
			audio_buffer_ready[1] = 1;
		}
	}
	else
	{
		/*
		// clear the playback flag and close file
		*/
		audio_stop_playback();
		audio_invoke_callback = 1;
	}	
}		
extern SD_DRIVER sd_card;	

/*
// play an mp3 file
*/
#if defined(SUPPORT_MP3)
#if defined(MP3_OUTPUT_TO_FILE)
SM_FILE mp3_output;
#endif
static void audio_play_file_mp3(SM_FILE* file, AUDIO_PLAYBACK_COMPLETED callback)
{
	#if defined(MP3_OUTPUT_TO_FILE)
	int i;
	
	i = sm_file_open(&mp3_output, "x:\\output.pcm", 
				SM_FILE_ACCESS_OVERWRITE | SM_FILE_ACCESS_CREATE);
	#endif
	
	audio_file = file;
	audio_callback = callback;
	mp3_data_ready = 0;
	audio_buffer_index = 0;
	audio_playback = 1;
	audio_active_buffer = 0;
	need_more_audio = 1;
	mp3_playback_started = 1;
	audio_io_in_progress = 1;
	//mp3_data_len = 0;
	audio_buffer_ready[0] = 1;
	audio_buffer_ready[1] = 0;
	audio_isr_buffer_index = AUDIO_BUFFER_SIZE_MP3 / 2;
	/*
	// configure DMA channel
	*/
	#if defined(__dsPIC33E__) && defined(USE_DMA_MP3)
	{
		DMA_SET_CHANNEL_DIRECTION(DMA_GET_CHANNEL(2), DMA_DIR_RAM_TO_PERIPHERAL);
		DMA_SET_PERIPHERAL(DMA_GET_CHANNEL(2), 0b1000);
		DMA_SET_PERIPHERAL_ADDRESS(DMA_GET_CHANNEL(2), &OC1R);
		DMA_SET_TRANSFER_LENGTH(DMA_GET_CHANNEL(2), (AUDIO_BUFFER_SIZE_MP3 / 2) - 1);
		DMA_SET_BUFFER_A_EDS(DMA_GET_CHANNEL(2), audio_buffer_a_mp3);
		DMA_SET_BUFFER_B_EDS(DMA_GET_CHANNEL(2), audio_buffer_b_mp3);
		#if !defined(MP3_OUTPUT_TO_FILE)
		DMA_CHANNEL_ENABLE(DMA_GET_CHANNEL(2));
		#endif
	}
	#endif
	/*
	// begin reading mp3 file
	*/
	#if !defined(MP3_OUTPUT_TO_FILE)
	audio_error = sm_file_read_async(audio_file, mp3_buffer, 512, &bytes_read, &audio_error, &audio_play_mp3_callback, 0);
	if (audio_error != FILESYSTEM_OP_IN_PROGRESS)
	{
		_ASSERT(0);
		mp3_playback_started = 0;
		audio_playback = 0;
		audio_invoke_callback = 1;
		return;
	}
	#endif
	/*
	// start decoding
	*/
	mp3_sm_start();
	/*
	// PWM module
	*/
	#if defined(__dsPIC33E__)
		OC1R = 0;
	#else
		OC1R = 0;
		OC1RS = 0;
	#endif
	/*
	// clear the samplerate
	*/
	mp3_samplerate = 0;	
	/*
	// wait for the 1st frame to be decoded
	*/
	while (!mp3_samplerate)
	{
		sd_idle_processing(&sd_card);
		mp3_sm_tasks();	
	}
	/*
	// setup to decode next frame
	*/
	need_more_audio = 1;
	mp3_samplerate = 0;
	audio_active_buffer = 1;
	/*
	// wait for the 2nd frame to be decoded
	*/	
	while (!mp3_samplerate)
	{
		sd_idle_processing(&sd_card);
		mp3_sm_tasks();	
	}
	/*
	// enable tmr3 interrupts unless this is a
	// dsPIC33E part and DMA is used
	*/
	#if defined(__dsPIC33E__) && defined(USE_DMA_MP3)
		IFS0bits.T3IF = 0;
		IEC0bits.T3IE = 0;
	#else
		IFS0bits.T3IF = 0;
		IEC0bits.T3IE = 1;
	#endif
	/*
	// set the sample rate as close as possible to
	// that of the mp3 file and start playback
	*/	
	PR3 = ((unsigned int)(FCY / mp3_samplerate));
	#if !defined(MP3_OUTPUT_TO_FILE)
	T2CONbits.TON = 1;
	T3CONbits.TON = 1;
	#endif
	#if defined(__dsPIC33E__) && defined(USE_DMA_MP3)
	need_more_audio = 0;
	audio_active_buffer = 1;
	#else
	while (!audio_active_buffer);
	#endif
}

#if !defined(MP3_OUTPUT_TO_FILE)
static void audio_play_mp3_callback(void* context, uint16_t* result)
{
	mp3_data_ready = 1;
}
#endif

unsigned int mp3_sm_input(void)
{
	/*
	// wait for mp3 data to arrive
	*/
	#if !defined(MP3_OUTPUT_TO_FILE)
	if (!mp3_data_ready) 
		return MP3_RSP_IGNORE;
	#else
	audio_error = sm_file_read(audio_file, mp3_buffer, 512, &bytes_read);
	if (audio_error != FILESYSTEM_SUCCESS)
	{
		bytes_read = 0;
	}
	#endif
	/*
	// if no bytes where read...
	*/
	if (!bytes_read || !audio_playback)
	{
		#if defined(MP3_OUTPUT_TO_FILE)
		sm_file_close(&mp3_output);
		#endif
		return MP3_RSP_STOP;
	}
	/*
	// write data to decode bitstream
	*/
	mp3_bitstream_write(mp3_buffer, bytes_read);
	/*
	// clear mp3 data ready flag. we must do this before
	// calling sm_file_read_async because if there's no data
	// to read it will call the callback function before
	// returning
	*/
	mp3_data_ready = 0;
	/*
	// start reading the next chunk
	*/
	#if !defined(MP3_OUTPUT_TO_FILE)
	audio_error = sm_file_read_async(audio_file, mp3_buffer, 512, &bytes_read, &audio_error, &audio_play_mp3_callback, 0);
	if (audio_error != FILESYSTEM_OP_IN_PROGRESS)
	{
		_ASSERT(0);
		return MP3_RSP_STOP;
	}
	#endif
	/*
	// continue decoding
	*/
  	return MP3_RSP_CONTINUE;
}

/*
// process decoder output
*/
unsigned int mp3_sm_output
(
	mp3_header_t const * header, 
	mp3_frac samples[2][1152]
)
{
  	static unsigned int __attribute__((__near__)) nsamples;
    unsigned int* buffer;
    unsigned int dswpag_value, dsrpag_value;
  	mp3_frac *left_ch, *right_ch;

	/*
	// if there's no audio ready IGNORE
	*/
	#if !defined(MP3_OUTPUT_TO_FILE)
	if (!need_more_audio)
		return MP3_RSP_IGNORE;
	#endif
	
	dswpag_value = DSWPAG;
	dsrpag_value = DSRPAG;

	mp3_samplerate = header->samplerate;
  	nsamples = header->no_of_pcm_samples;
  	left_ch = samples[0];
  	right_ch = samples[1];
		
  	if (audio_active_buffer == 0)
  	{
		DSWPAG = __builtin_edspage(audio_buffer_a_mp3);
		DSRPAG = __builtin_edspage(audio_buffer_a_mp3);
		buffer = (unsigned int*) __builtin_edsoffset(audio_buffer_a_mp3);
	   	audio_buffer_index = 0;
	}
	else
	{
		DSWPAG = __builtin_edspage(audio_buffer_b_mp3);
		DSRPAG = __builtin_edspage(audio_buffer_b_mp3);
		buffer = (unsigned int*) __builtin_edsoffset(audio_buffer_b_mp3);
	   	audio_buffer_index = 0;
	}
		
	/*
	// copy to playback buffer
	*/
  	while (nsamples--) 
	{
		#if defined(MP3_OUTPUT_TO_FILE)
		sm_file_write(&mp3_output, (unsigned char*) left_ch, 2);
		if (header->no_of_channels == 2)
			sm_file_write(&mp3_output, (unsigned char*) right_ch++, 2);
		#endif
    	buffer[audio_buffer_index++] = *left_ch++;
  	}
	/*
	// adjust volume and convert to 8-bit
	*/
	#if !defined(MP3_OUTPUT_TO_FILE)
	truncate(buffer, audio_volume_level, AUDIO_BUFFER_SIZE_MP3 / 2);
	#endif
   	need_more_audio = 0;
   	
	DSWPAG = dswpag_value;
	DSRPAG = dsrpag_value;
		
	return MP3_RSP_CONTINUE;
}

unsigned int mp3_sm_error(unsigned int error)
{
	return MP3_RSP_CONTINUE;
}

/*
// invoke when the decoder is finished decoding
// for whatever reason
*/
void mp3_sm_finished(int result)
{
	audio_stop_playback();
	mp3_playback_started = 0;
	audio_invoke_callback = (bytes_read == 0);
	need_more_audio = 1;
}
#endif // SUPPORT_MP3


/*
// stops ongoing playback
*/
void audio_stop_playback(void)
{
	audio_playback = 0;	
}

/*
// audio tasks
*/
void audio_tasks(void)
{
	/*
	// run mp3 decoder tasks
	*/
	#if defined(SUPPORT_MP3)
	mp3_sm_tasks();
	#endif
	/*
	// if more samples are needed then get them
	// and if playback is complete cclose file
	*/
	#if defined(SUPPORT_MP3)
	if (need_more_audio && !mp3_playback_started)
	#else
	if (need_more_audio)
	#endif
	{
		/*
		// stop playback
		*/
		if (!audio_playback)
		{
			//while (1);
			//if (!audio_io_in_progress)
			//{
				/*
				// close audio file
				*/
				if (sm_file_close(audio_file) != SM_SUCCESS)
				{
					//while (1);	
				}
				T2CONbits.TON = 0;
				T3CONbits.TON = 0;
				IFS0bits.T3IF = 0;
				#if defined(__dsPIC33E__) && defined(USE_DMA_MP3)
				DMA_CHANNEL_DISABLE(DMA_GET_CHANNEL(2));
				#endif
				need_more_audio = 0;
				#if defined(__dsPIC33E__)
					OC1R = 0;
				#else
					OC1RS = 0;
				#endif
				
				if (audio_invoke_callback)
				{
					if (audio_callback)
					{
						audio_callback(audio_file);
					}
				}
			//	audio_io_in_progress = 1;
			//}
		}
		else
		{
			if (!audio_io_in_progress)
			{
				audio_error = sm_file_read_async(audio_file, (audio_active_buffer == 0) ? audio_buffer_a : audio_buffer_b, AUDIO_BUFFER_SIZE, &bytes_read, &audio_error, (SM_ASYNC_CALLBACK) &audio_read_callback, audio_file);
				if (audio_error != FILESYSTEM_OP_IN_PROGRESS)
				{
					_ASSERT(0);
					/*
					// close file
					*/
					sm_file_close(audio_file);
					return;
				}
				
				need_more_audio = 0;
				audio_io_in_progress = 1;
			}
		}
	}	
}

/*
// interrupt for ADC conversions
*/
void __attribute__((__interrupt__, __auto_psv__, __shadow__)) _DMA2Interrupt(void) 
{
	#if defined(SUPPORT_MP3) && defined(__dsPIC33E__) && defined(USE_DMA_MP3)
	if (mp3_playback_started)
	{
		audio_active_buffer ^= 1;
		need_more_audio = 1;
	}
	else
	#endif
	{
		/*
		// count lost chunks
		*/
		if (p_audio_buffer == audio_buffer_a || p_audio_buffer == audio_buffer_b)
		{
			if (audio_active_buffer != audio_dma_active_buffer && !audio_waiting_for_data && audio_good_chunks)
			{
				audio_lost_chunks++;
				#if defined(HALT_ON_LOST_CHUNK)
						_ASSERT(0);
				#endif
			}
		}	
		/*
		// if this is the end of the buffer toggle the active buffer bit
		*/
		if (p_audio_buffer == (audio_buffer + (AUDIO_BUFFER_SIZE - (DMA_BUFFER_SIZE / 2))) ||
			p_audio_buffer == (audio_buffer + ((AUDIO_BUFFER_SIZE * 2) - (DMA_BUFFER_SIZE / 2))))
		{
			audio_dma_active_buffer ^= 1;
		}	
		/*
		// copy data to buffer
		*/
		#if (DMA_BUFFER_SIZE >= 8)
			__asm__
			(
				"mov %0, w4\n"
				"mov %1, w11\n"
				"bset MODCON, #14\n"
				"bset MODCON, #15\n"
				"push RCOUNT\n"
				"repeat #%2\n"
				"mov [w11++], [w4++]\n"
				"mov [w11], [w4++]\n"
				"mov w4, %0\n"
				"movsac A, [w11] += 2, w4\n"
				"mov w11, %1\n"
				"pop RCOUNT\n"
				"bclr MODCON, #14\n"
				"bclr MODCON, #15"
				: "+g" (p_audio_buffer), "+g" (p_adc_buffer)
				: "i" ((DMA_BUFFER_SIZE / 4) - 2)
				: "w11", "w4"
			);
		#else
			__asm__
			(
				"mov %0, w4\n"
				"mov %1, w11\n"
				"bset MODCON, #14\n"
				"bset MODCON, #15\n"
				"mov [w11], [w4++]\n"
				"mov w4, %0\n"
				"movsac A, [w11] += 2, w4\n"
				"mov w11, %1\n"
				"bclr MODCON, #14\n"
				"bclr MODCON, #15"
				: "+g" (p_audio_buffer), "+g" (p_adc_buffer)
				: 
				: "w11", "w4"
			);
		#endif
	}
	/*
	// clear interrupt flag
	*/
	IFS1bits.DMA2IF = 0;
}

/*
// sample rate timer interrupt
*/
void __attribute__((__interrupt__, __auto_psv__)) _T3Interrupt(void)
{
	static unsigned int* __attribute__((__near__)) audio_buffer_ptr;
	#if defined(SUPPORT_MP3) && defined(__dsPIC33E__) && defined(USE_DMA_MP3)
	if (mp3_playback_started)
	{
		while (1);
	}
	#elif defined(SUPPORT_MP3)
	static __eds__ unsigned int* __attribute__((__near__)) audio_buffer_ptr_mp3;
	if (mp3_playback_started)
	{
		/*
		// if we're done playing this buffer switch buffers
		*/
		if (audio_isr_buffer_index >= AUDIO_BUFFER_SIZE_MP3 / 2)
		{
			/*
			// switch buffer and update index
			*/
			if (audio_active_buffer == 0)
			{
				audio_buffer_ptr_mp3 = (__eds__ unsigned int*) audio_buffer_a_mp3;
				audio_active_buffer = 1;
				audio_buffer_ready[1] = 0;
			}
			else
			{
				audio_buffer_ptr_mp3 = (__eds__ unsigned int*) audio_buffer_b_mp3;
				audio_active_buffer = 0;
				audio_buffer_ready[0] = 0;
			}
			audio_isr_buffer_index = 0;
			/*
			// set flag to indicate that last buffer
			// should be filled with fresh data
			*/
			need_more_audio = 1;
		}	
		/*
		// get the sample from the buffer and write
		// it to OC register
		*/
		#if defined(__dsPIC33E__)
			OC1R = (unsigned char) audio_buffer_ptr_mp3[audio_isr_buffer_index++];
		#else	
			OC1RS = (unsigned char) audio_buffer_ptr_mp3[audio_isr_buffer_index++];
		#endif
		
	}
	else
	#endif
	{
		/*
		// if we're done playing this buffer switch buffers
		*/
		if (audio_isr_buffer_index >= AUDIO_BUFFER_SIZE / 2)
		{
			/*
			// switch buffer and update index
			*/
			if (audio_active_buffer == 0)
			{
				if (!audio_buffer_ready[0])
				{
					/*
					// if there's no samples ready set output
					// to mid point voltage
					*/
					#if defined(__dsPIC33E__)
						OC1R = 128;
					#else	
						OC1RS = 128;
					#endif
					/*
					// clear interrupt flag
					*/
					IFS0bits.T3IF = 0;
					return;
				}
				audio_buffer_ptr = (unsigned int*) audio_buffer_a;
				audio_active_buffer = 1;
				audio_buffer_ready[1] = 0;
			}
			else
			{
				if (!audio_buffer_ready[1])
				{
					/*
					// if there's no samples ready set output
					// to mid point voltage
					*/
					#if defined(__dsPIC33E__)
						OC1R = 128;
					#else	
						OC1RS = 128;
					#endif
					/*
					// clear interrupt flag
					*/
					IFS0bits.T3IF = 0;
					return;
				}
				audio_buffer_ptr = (unsigned int*) audio_buffer_b;
				audio_active_buffer = 0;
				audio_buffer_ready[0] = 0;
			}
			audio_isr_buffer_index = 0;
			/*
			// set flag to indicate that last buffer
			// should be filled with fresh data
			*/
			need_more_audio = 1;
		}	
		/*
		// get the sample from the buffer and write
		// it to OC register
		*/
		#if defined(__dsPIC33E__)
			OC1R = audio_buffer_ptr[audio_isr_buffer_index++];
		#else	
			OC1RS = audio_buffer_ptr[audio_isr_buffer_index++];
		#endif
	}
	/*
	// clear interrupt flag
	*/
	IFS0bits.T3IF = 0;
}
