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

#define USE_ILI9341
//#define EXPLORER16

/*
// pin configuration
*/
#if defined(EXPLORER16)
	#define SD_CARD_SPI_MODULE					2			/* SPI module for SD card */
	#define SD_CARD_CS_LINE_PORT				PORTG		/* SD card CS line port */
	#define SD_CARD_CS_LINE_TRIS				TRISG		/* SD card CS line tris */
	#define SD_CARD_CS_LINE_LAT					LATG		/* SD card CS line lat */
	#define SD_CARD_CS_LINE_PIN					3			/* SD card CS line pin # */
	
	#define SD_CARD_CD_LINE_PORT				PORTF		/* SD card C/D (card detect) line port */
	#define SD_CARD_CD_LINE_TRIS				TRISF		/* SD card C/D (card detect) line tris */
	#define SD_CARD_CD_LINE_LAT					LATF		/* SD card C/D (card detect) line lat */
	#define SD_CARD_CD_LINE_PIN					2			/* SD card C/D (card detect) line pin # */
	
	#define SD_CARD_ACTIVITY_LED_PORT 			PORTA 		/* SD card activity LED port */
	#define SD_CARD_ACTIVITY_LED_TRIS			TRISA		/* SD card activity LED tris */
	#define SD_CARD_ACTIVITY_LED_LAT			LATA		/* SD card activity LED lat */
	#define SD_CARD_ACTIVITY_LED_PIN			5			/* SD card activity LED pin # */
	/*
	// heartbeat LED
	*/
	#define HEARTBEAT 							LATAbits.LATA7
	#define HEARTBEAT_TRIS						TRISAbits.TRISA7
	
#else
	/*
	// SD card pins and SPI module
	*/
	#define SD_CARD_SPI_MODULE					1
	#define SD_CARD_CS_LINE_PORT				PORTB
	#define SD_CARD_CS_LINE_TRIS				TRISB
	#define SD_CARD_CS_LINE_LAT					LATB
	#define SD_CARD_CS_LINE_PIN					11
	#define SD_CARD_CD_LINE_PORT				PORTA
	#define SD_CARD_CD_LINE_TRIS				TRISA
	#define SD_CARD_CD_LINE_LAT					LATA
	#define SD_CARD_CD_LINE_PIN					3
	#if defined(__dsPIC33E__)
		#define SD_CARD_CD_LINE_CNEN			CNENAbits.CNIEA3
	#else
		#define SD_CARD_CD_LINE_CNEN			CNEN2bits.CN29IE	
	#endif
	#define SD_CARD_ACTIVITY_LED_PORT 			virtual_pins
	#define SD_CARD_ACTIVITY_LED_TRIS			virtual_tris
	#define SD_CARD_ACTIVITY_LED_LAT			virtual_pins
	#define SD_CARD_ACTIVITY_LED_PIN			0
	/*
	// ILI9341 backlight unit
	*/
	#define ILI9341_BLU							LATBbits.LATB2
	#define ILI9341_BLU_TRIS					TRISBbits.TRISB2
	/*
	// heartbeat LED
	*/
	#define HEARTBEAT 							LATBbits.LATB12
	#define HEARTBEAT_TRIS						TRISBbits.TRISB12
	/*
	// button 1
	*/
	#define BUTTON1								PORTBbits.RB13
	#define BUTTON1_LAT							LATBbits.LATB13
	#define BUTTON1_TRIS						TRISBbits.TRISB13
	#if defined(__dsPIC33E__)
		#define BUTTON1_CNEN					CNENBbits.CNIEB13
	#else
		#define BUTTON1_CNEN					CNEN1bits.CN13IE
	#endif
	/*
	// button 2
	*/
	#define BUTTON2								PORTBbits.RB14
	#define BUTTON2_LAT							LATBbits.LATB14
	#define BUTTON2_TRIS						TRISBbits.TRISB14
	#if defined(__dsPIC33E__)
		#define BUTTON2_CNEN					CNENBbits.CNIEB14
	#else
		#define BUTTON2_CNEN					CNEN1bits.CN12IE
	#endif
	/*
	// button 3
	*/
	#define BUTTON3								PORTAbits.RA1
	#define BUTTON3_LAT							LATAbits.LATA1
	#define BUTTON3_TRIS						TRISAbits.TRISA1
	#if defined(__dsPIC33E__)
		#define BUTTON3_CNEN					CNENAbits.CNIEA1
	#else
		#define BUTTON3_CNEN					CNEN1bits.CN3IE
	#endif
	/*
	// button 4
	*/
	#define BUTTON4								PORTBbits.RB15
	#define BUTTON4_LAT							LATBbits.LATB15
	#define BUTTON4_TRIS						TRISBbits.TRISB15
	#if defined(__dsPIC33E__)
		#define BUTTON4_CNEN					CNENBbits.CNIEB15
	#else
		#define BUTTON4_CNEN					CNEN1bits.CN11IE
	#endif
#endif
 
/*
// LCD headers
*/ 
#if defined(USE_ILI9341)
	#include <ili9341/ili9341.h>
	#include <lglib/lg.h>
#elif defined(EXPLORER16)
	#include <lcd.h>
#endif

#define SLEEP_TIMER			5
#define NO_LP_OSC

#include <common.h>
#include <dma.h>
#include <rtc.h>
#include <spi.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/*
// Fat32Lib includes
*/
#include <sdlib/sd.h>
#include <fat32lib/storage_device.h>
#include <fat32lib/fat.h>
#include <fat32lib/filesystem_interface.h>
#include <smlib/sm.h>

#include "audio.h"

/*
// configuration bits
*/
#if defined(EXPLORER16)
	_FOSCSEL(FNOSC_PRIPLL & IESO_OFF);
	_FOSC(POSCMD_XT & OSCIOFNC_ON & FCKSM_CSDCMD);
	_FWDT(FWDTEN_OFF);
	_FPOR(FPWRT_PWR128);
#else
	_FOSCSEL(FNOSC_FRCPLL & IESO_OFF);
	_FOSC(POSCMD_NONE & OSCIOFNC_ON & FCKSM_CSDCMD & IOL1WAY_OFF);
	_FWDT(FWDTEN_OFF & WDTPRE_PR128 & WDTPOST_PS256);
	#if defined(__dsPIC33E__)
		_FPOR(ALTI2C1_OFF & ALTI2C2_OFF & WDTWIN_WIN25);
		_FICD(ICS_PGD3 & JTAGEN_OFF);
	#else
		_FPOR(FPWRT_PWR128);
	#endif
#endif

#if defined(USE_ILI9341)
	/*
	// GUI elements
	*/
	int16_t time_label;
	int16_t date_label;
	int16_t volume_icon;
	int16_t song_name;
	int16_t button1_icon;
	int16_t button2_icon;
	int16_t button3_icon;
	int16_t button4_icon;
	int16_t button5_icon;
	int16_t volume_bar;
	uint16_t virtual_pins;
	uint16_t virtual_tris;
	/*
	//
	*/
	const unsigned char ui_icon_sdcard[] = "\1";
	const unsigned char ui_icon_record[] = "\2";
	const unsigned char ui_icon_rewind[] = "\17";
	const unsigned char ui_icon_forward[] = "\16";
	const unsigned char ui_icon_play[] = "\14";
	const unsigned char ui_icon_stop[] = { 0x5B, 0x00};
	const unsigned char ui_icon_left_arrow[] = "\5";
	const unsigned char ui_icon_right_arrow[] = "\6";
	/*
	//
	*/
	#define ICON_SDCARD					((unsigned char*) ui_icon_sdcard)
	#define ICON_RECORD					((unsigned char*) ui_icon_record)
	#define ICON_REWIND					((unsigned char*) ui_icon_rewind)
	#define ICON_FORWARD				((unsigned char*) ui_icon_forward)
	#define ICON_PLAY					((unsigned char*) ui_icon_play)
	#define ICON_STOP					((unsigned char*) ui_icon_stop)
	#define ICON_LEFT_ARROW				((unsigned char*) ui_icon_left_arrow)
	#define ICON_RIGHT_ARROW			((unsigned char*) ui_icon_right_arrow)
	
#elif defined(EXPLORER16)
	/*
	// text lcd driver for explorer 16.
	*/
	LCD_CONFIG lcd_driver;
#endif																// lcd driver

/*
// SD card driver and DMA buffers
*/
SD_DRIVER __attribute__((section(".sd_driver"))) sd_card;	
#if defined(__dsPIC33E__)
	unsigned char __attribute__((section(".sd_driver_buffer"))) dma_buffer[512];	// DMA buffer for async/stream io
	char __attribute__((section(".sd_driver_byte"))) dma_byte;					// sd driver needs 1 byte of dma memory
#else
	unsigned char __attribute__((space(dma), section(".sd_driver_dma_buffer"))) dma_buffer[512];	// DMA buffer for async/stream io
	char __attribute__((space(dma), section(".sd_driver_dma_byte"))) dma_byte;					// sd driver needs 1 byte of dma memory
#endif
/*
// file handle for audio recording/playback
*/
SM_FILE audio_file_handle;

/*
// this is used to keep track of idle time
*/
static unsigned char sleep_timer;

/*
// these are the details of the currently
// "loaded" file and the count of files on the
// SD card.
*/
static unsigned int file_count = 0;
static unsigned int file_index = 0;
static unsigned char s_song_name[33];
static unsigned char s_volume_bar[17] = "|||||||||||||||";
static unsigned char volume_level = 14;

static char mp3_playback_in_progress = 0;

/*
// function prototypes
*/
static void init_cpu(void);
static void init_uart(void);
static void init_lcd(void);
static void init_fs(void);
static void idle_processing(void);
static void volume_mounted(char* volume_label);
static void volume_dismounted(char* volume_label);
static void init_pins(void);
static void record_pressed(void);
static void load_file(char play);
static void start_sampling(void);
static void button_tasks(void);
static void play_next_file(void);
static void play_prev_file(void);
static void playback_completed(SM_FILE* f);
static void update_song_name(char* filename);
static void volume_down(void);
static void volume_up(void);
static void update_volume_bar(void);
static enum AUDIO_FILE_TYPE get_file_type(char* filename);
static void stop_playback(void);

/*
// entry point
*/
int main()
{
	/*
	// clock th/e cpu and mount the filesystem
	*/
	init_cpu();
	/*
	// initialize pins
	*/
	init_pins();
	/*
	// initialize real-time clock
	*/
	#if defined(__dsPIC33F__)
		rtc_init(40000000);
	#elif defined(__dsPIC33E__)
		rtc_init(70000000);
	#else
		#error !!
	#endif
	/*
	// initialize lcd
	*/
	#if defined(EXPLORER16) || defined(USE_ILI9341)
		init_lcd();
	#endif
	/*
	// initialize audio drivers
	*/
	audio_init();
	audio_set_volume(15);
	/*
	// initialize filesystem	
	*/
	init_fs();
	/*
	// initialize uart (for printf)
	*/
	init_uart();
	/*
	// do background processing
	*/
	while (1) 
		idle_processing();
}

//
// initialize cpu
//
static void init_cpu(void)
{
	/*
	// disable nested interrupts. we need either this or raise
	// the priority of the ADC interrupt as it uses modulo addressing
	// and will corrupt memory if interrupted
	*/
	//INTCON1bits.NSTDIS = 1;
	/*
	// enable overflow traps
	*/
	//INTCON1bits.OVATE = 1;
	//INTCON1bits.OVBTE = 1;
	/*
	// disable all peripherals except timers,
	// SPI1, SPI2, AD1, and OC1
	*/ 	
	PMD1 = 0x07E6;
	PMD2 = 0xFFFE;	
	/*
	// initialize sleep timer counter. This is
	// decreased every second while the CPU is idle
	// when it reaches 0 the cpu is put to sleep
	*/
	sleep_timer = SLEEP_TIMER;

	#if defined(__dsPIC33F__)
		//
		// Configure oscillator to operate the device at 40Mhz
		// using the primary oscillator with PLL and a 8 MHz crystal
		// on the explorer 16
		//
		// Fosc = Fin * M / ( N1 * N2 )
		// Fosc = 8M * 32 / ( 2 * 2 ) = 80 MHz
		// Fcy = Fosc / 2 
		// Fcy = 80 MHz / 2
		// Fcy = 40 MHz
		//
		PLLFBDbits.PLLDIV = 38;
		CLKDIVbits.PLLPOST = 0;		
		CLKDIVbits.PLLPRE = 0;	
		//
		// switch to primary oscillator
		//	
		//clock_sw();
		//
		// wait for PLL to lock
		//
		while (OSCCONbits.LOCK != 0x1);
		
	#elif defined(__dsPIC33E__)
		//
		// Fosc = Fin * ((PLLDIV + 2) / ((PLLPRE + 2) * 2 * (PLLPOST + 1)))
		// Fosc = 8 * ((68 + 2) / (2 * 2 * 1)) = 140
		// Fcy = Fosc / 2
		// Fcy = 140 / 2
		// Fcy = 70 MHz
		//
		PLLFBDbits.PLLDIV = 68;
		CLKDIVbits.PLLPRE = 0;
		CLKDIVbits.PLLPOST = 0;
		
		while (OSCCONbits.LOCK != 0x1);
	#endif
}

/*
// configure pps
*/
#if !defined(EXPLORER16)
static void init_pins(void)
{
	/*
	// enable change notification interrupts
	*/
	IEC1bits.CNIE = 1;

	#if defined(EXPLORER16)
		/*
		// configure all IO pins as digital
		*/
		AD1PCFGH = 0xffff;
		AD2PCFGL = 0Xffff;	
		AD1PCFGL = 0Xffff;
		/*
		// configure io ports
		*/	
		IO_PORT_SET_AS_OUTPUT(A);	// LEDs
		IO_PIN_SET_AS_INPUT(D, 6);	/* this will be our dismount button */
		IO_PIN_WRITE(D, 6, 1);	
		
		IO_PIN_WRITE(A, 4, 0);			// sampling indicator
		IO_PIN_WRITE(A, 6, 0);			// volume mounted indicator
		
		IO_PIN_SET_AS_INPUT(D, 6);	/* this will be our dismount button */
		IO_PIN_WRITE(D, 6, 1);	
		IO_PIN_SET_AS_INPUT(D, 7);	/* this will be our record button */
		IO_PIN_WRITE(D, 7, 1);
		
	#else /* custom board */
	
		/*
		// configure all IO pins as digital
		*/
		#if defined(__dsPIC33F__)
			AD1PCFGL = 0xFFFF;
		#elif defined(__dsPIC33E__)
			ANSELA = 0;
			ANSELB = 0;
		#else
			#error !!
		#endif
		/*
		// configure push buttons
		*/
		BUTTON1_LAT = 1;
		BUTTON3_LAT = 1;
		BUTTON4_LAT = 1;
		BUTTON1_TRIS = 1;
		BUTTON3_TRIS = 1;
		BUTTON4_TRIS = 1;
		BUTTON1_CNEN = 1;
		BUTTON2_CNEN = 1;
		BUTTON3_CNEN = 1;
		BUTTON4_CNEN = 1;
		/*
		// backlight control
		*/
		#if defined(USE_ILI9341)
		ILI9341_BLU = 1;
		ILI9341_BLU_TRIS = 0;
		#endif
		/*
		// configure PPS
		*/
		#if defined(__dsPIC33E__)
			/*
			// configure SPI2 for ILI9341
			*/		
			#if defined(USE_ILI9341)
				RPOR0bits.RP35R = 0b001000; // SDO
				RPOR1bits.RP37R = 0b001001; // SCK
				IO_PIN_SET_AS_OUTPUT(B, 5);
				IO_PIN_SET_AS_OUTPUT(B, 3);
			#endif
			/*
			// configure OC1 on RB2
			*/
			RPOR2bits.RP38R = 0b010000; // OC1
			IO_PIN_WRITE(B, 6, 0);
			IO_PIN_SET_AS_OUTPUT(B, 6);
			/*
			// lock pin configuration
			*/	
			OSCCONbits.IOLOCK = 1;
		
		#elif defined(__dsPIC33F__)
		
			/*
			// configure SPI2 for ILI9341
			*/
			#if defined(USE_ILI9341)
				RPOR2bits.RP5R = 0b01011;		/* SCK */
				RPOR1bits.RP3R = 0b01010;		/* SDO */
				IO_PIN_SET_AS_OUTPUT(B, 5);
				IO_PIN_SET_AS_OUTPUT(B, 3);
			#endif
			/*
			// configure OC1 on RB2
			*/
			RPOR3bits.RP6R = 0b10010;		/* OC1 */
			IO_PIN_WRITE(B, 2, 0);
			IO_PIN_SET_AS_OUTPUT(B, 2);
			/*
			// configure SPI1 for SD card
			*/
			RPINR20bits.SDI1R = 0X9;		/* SDI */
			RPOR3bits.RP7R = 0b01000;		/* SCK */
			RPOR4bits.RP8R = 0b00111;		/* SDO */
			IO_PIN_SET_AS_OUTPUT(B, 7);
			IO_PIN_SET_AS_OUTPUT(B, 8);
			IO_PIN_SET_AS_INPUT(B, 9);
			/*
			// lock pin configuration
			*/
			__builtin_write_OSCCONL(0X46);
			__builtin_write_OSCCONH(0X57);
			OSCCONbits.IOLOCK = 1;
		#endif
	#endif
	/*
	// heartbeat LED
	*/
	HEARTBEAT_TRIS = 0;
	HEARTBEAT = 1;
}
#endif

static void init_uart(void)
{
	#if defined(EXPLORER16)
		U2BRG = 259; //4; //(40000000L / (16 * 460800)) - 1;
		U2MODEbits.BRGH = 0;
		U2MODEbits.UARTEN = 1;
		U2MODEbits.RTSMD = 0;
		
		printf("Welcome\r\n");
		/* while (IO_PIN_READ(D, 6)); */
		printf("Starting\r\n");
	#endif
}

//
// initialize LCD
//
#if defined(EXPLORER16) || defined(USE_ILI9341)
static void init_lcd(void) 
{
	#if defined(USE_ILI9341)
	
		ili9341_init();
		lg_init((LG_DISPLAY_PAINT)&ili9341_paint, 
			(LG_DISPLAY_PAINT_PARTIAL) &ili9341_paint_partial);
		/*
		// set the background color
		*/
		//lg_set_background(0b010101000101010001010100);
		lg_set_background(0x000000);
		/*
		// paint screen and enable the display
		*/
		while (ili9341_is_painting())
			ili9341_do_processing();
		ili9341_display_on(); 
		/*
		// add labels for date/time and volume mounted icon
		*/
		time_label = lg_label_add((unsigned char*) "10:30", 0, 2, 1, 0xFFFFFF, 115, 95);
		date_label = lg_label_add((unsigned char*) "00/00/2000 ", 0, 2, 1, 0xFFFFFF, 65, 62);
		volume_icon = lg_label_add(ICON_SDCARD, 0, 1, 1, 0xFFFFFF, 258, 66);
		song_name = lg_label_add((unsigned char*) "", 0, 1, 1, 0xFFFFFF, 30, 140); 

		volume_bar = lg_label_add(s_volume_bar, 0, 2, 0, 0xFFFFFF, 36, 10);
		
		button1_icon = lg_label_add(ICON_RECORD,  0, 2, 1, 0xFF0000, 010, 010);
		button2_icon = lg_label_add(ICON_RECORD,  0, 2, 1, 0xFFFFFF, 294, 010);
		button3_icon = lg_label_add(ICON_REWIND,  0, 2, 1, 0xFFFFFF, 010, 214);
		button4_icon = lg_label_add(ICON_FORWARD, 0, 2, 1, 0xFFFFFF, 294, 214);
		button5_icon = lg_label_add(ICON_PLAY,    0, 2, 1, 0xFFFFFF, 150, 214);
		/*
		// make all button labels invisible
		*/
		lg_label_set_visibility(volume_bar, 0);
		lg_label_set_visibility(volume_icon, 0);
		lg_label_set_visibility(button1_icon, 0);
		lg_label_set_visibility(button2_icon, 0);
		lg_label_set_visibility(button3_icon, 0);
		lg_label_set_visibility(button4_icon, 0);
		lg_label_set_visibility(button5_icon, 0);
		
		
	#else
		// 
		// For Explorer 16 board:
		//
		// RS -> RB15
		// E  -> RD4
		// RW -> RD5
		// DATA -> RE0 - RE7   
		// ---
		//
		// set the pins used by the LCD as outputs
		//
		IO_PORT_SET_DIR_MASK(E, IO_PORT_GET_DIR_MASK(E) & 0xFF00);
		IO_PIN_SET_AS_OUTPUT(D, 5);
		IO_PIN_SET_AS_OUTPUT(B, 15);
		IO_PIN_SET_AS_OUTPUT(D, 4);
		IO_PIN_WRITE(D, 5, 0);
		IO_PIN_WRITE(B, 15, 0);
		//
		// configure the LCD pins
		//
		lcd_driver.DataLine = (unsigned char*) &LATE; 	// DATA line connected to pins 7-0 on port E
		BP_INIT(lcd_driver.RWLine, &LATD, 5);			// RW line connected to pin 5 on port D
		BP_INIT(lcd_driver.RSLine, &LATB, 15);			// RS line connected to pin 15 on port B
		BP_INIT(lcd_driver.ELine, &LATD, 4);			// E line connected to pin 4 on port D
		//
		// set the VDD rise delay (ms)
		//
		lcd_driver.VddRiseDelay = 0x1;
		//
		// initialize the LCD driver
		//
		lcd_init(&lcd_driver);
	#endif
}
#endif

//
// initialize the filesystem drivers
//
static void init_fs(void)
{
	BIT_POINTER cs;
	BIT_POINTER media_ready;
	BIT_POINTER busy_signal;
	STORAGE_DEVICE storage_device;
	FILESYSTEM fat_filesystem;
	//
	// configure SD card pins
	//
	SD_CARD_CS_LINE_TRIS &= ~(1 << SD_CARD_CS_LINE_PIN);			/* set CS line as output */
	SD_CARD_CD_LINE_TRIS |= (1 << SD_CARD_CD_LINE_PIN);				/* set CD line as input */
	SD_CARD_ACTIVITY_LED_TRIS &= ~(1 << SD_CARD_ACTIVITY_LED_PIN);	/* set disk activity led line as output */
	SD_CARD_ACTIVITY_LED_LAT &= ~(1 << SD_CARD_ACTIVITY_LED_PIN);	/* set disk activity led off */
	SD_CARD_CD_LINE_LAT |= (1 << SD_CARD_CD_LINE_PIN);				/* set CD line latch to high */
	/*
	// enable change notification interrupt
	// for SD card card detect
	*/
	SD_CARD_CD_LINE_CNEN = 1;
	/*
	// initialize bit pointers for sd driver
	*/
	BP_INIT(busy_signal, &SD_CARD_ACTIVITY_LED_LAT, SD_CARD_ACTIVITY_LED_PIN);
	BP_INIT(media_ready, &SD_CARD_CD_LINE_PORT, SD_CARD_CD_LINE_PIN);
	BP_INIT(cs, &SD_CARD_CS_LINE_LAT, SD_CARD_CS_LINE_PIN);
	/*
	// set the priority of the driver's DMA channels
	*/
	DMA_CHANNEL_SET_INT_PRIORITY(DMA_GET_CHANNEL(0), 0x6);
	DMA_CHANNEL_SET_INT_PRIORITY(DMA_GET_CHANNEL(1), 0x6);
	
	/*
	// initialize SD card driver
	*/
	sd_init
	(
		&sd_card, 				// pointer to driver handle
		SPI_GET_MODULE(SD_CARD_SPI_MODULE),
		DMA_GET_CHANNEL(0), 	// 1st DMA channel (interrupt must be configured for this channel)
		DMA_GET_CHANNEL(1), 	// 2nd DMA channel (interrupt must be configured for this channel)
		dma_buffer,				// optional async buffer (DMA memory)
		&dma_byte, 				// 1 byte of dma memory
		media_ready, 			// bit-pointer to pin that rises when card is on slot
		cs,						// bit-pointer to pin where chip select line is connected
		busy_signal,			// bit-pointer to IO indicator LED
		34						// device id
	);
	//
	// get the STORAGE_DEVICE interface for the SD card
	// driver and the FILESYSTEM interface for the FAT driver
	//
	sd_get_storage_device_interface(&sd_card, &storage_device);
	fat_get_filesystem_interface(&fat_filesystem);
	//
	// register the FAT driver with smlib
	//
	fat_init();
	sm_register_filesystem(&fat_filesystem);
	//
	// register the SD device with smlib as drive x:
	// anytime a card is inserted it will be automatically
	// mounted as drive x:
	//
	sm_register_storage_device(&storage_device, "x:");
	//
	// register a callback function to receive notifications
	// when a new drivve is mounted.
	//
	sm_register_volume_mounted_callback(&volume_mounted);
	sm_register_volume_dismounted_callback(&volume_dismounted);
}

//
// callback function to receive notifications when
// a new drive is mounted
//
static void volume_mounted(char* volume_label)
{
	SM_QUERY q;
	SM_DIRECTORY_ENTRY entry;
	uint16_t i;
	/*
	// lignt LED to indicate that drive is mounted
	*/
	#if defined(USE_ILI9341)
		lg_label_set_visibility(volume_icon, 1);
	#elif defined(EXPLORER16)
		IO_PIN_WRITE(A, 6, 1);
	#else
		//IO_PIN_WRITE(A, 2, 1);
	#endif
	
	
	file_count = 0;
	/*
	// find the 1st entry in the root folder
	*/
	i = sm_find_first_entry("x:\\", 0, &entry, &q);
	if (i != SM_SUCCESS)
	{
		_ASSERT(0);	
	}
	/*
	//
	*/
	while (*entry.name != 0)
	{
		if ((entry.attributes & SM_ATTR_DIRECTORY) == 0)
		{
			if (get_file_type(entry.name) != AUDIO_FILE_UNKNOWN)
			{
				if (file_count == file_index)
				{
					update_song_name(entry.name);
				}
				/*
				// increase the file count
				*/
				file_count++;
			}
		}
		i = sm_find_next_entry(&entry, &q);
		if (i != SM_SUCCESS)
		{
			_ASSERT(0);
		}
	}
	i = sm_find_close(&q);
	if (i != SM_SUCCESS)
	{
		_ASSERT(0);
	}
	/*
	// if there's any audio files on the card
	// display all button labels
	*/
	if (file_count)
	{
		lg_label_set_visibility(button1_icon, 1);
		lg_label_set_visibility(button2_icon, 1);
		lg_label_set_visibility(button3_icon, 1);
		lg_label_set_visibility(button4_icon, 1);
		lg_label_set_visibility(button5_icon, 1);
	}
}

//static char is_audio_file(unsigned char* filename)
//{
//	return 1;
//}

static void volume_dismounted(char* volume_label)
{
	/*
	// turn off the drive mounted indicator LED
	*/
	#if defined(USE_ILI9341)
		lg_label_set_visibility(volume_icon, 0);
	#elif defined(EXPLORER16)
		IO_PIN_WRITE(A, 6, 0);
	#else
		//IO_PIN_WRITE(A, 2, 0);
	#endif
	
	file_count = 0;
	file_index = 0;
	/*
	// hide all button labels and song name
	*/
	lg_label_set_string(song_name, (unsigned char*)"");	
	lg_label_set_visibility(button1_icon, 0);
	lg_label_set_visibility(button2_icon, 0);
	lg_label_set_visibility(button3_icon, 0);
	lg_label_set_visibility(button4_icon, 0);
	lg_label_set_visibility(button5_icon, 0);
}

static void start_sampling(void)
{
	uint16_t audio_error;
	audio_error = sm_file_open(&audio_file_handle, "x:\\audio.pcm", 
					SM_FILE_ACCESS_READ | SM_FILE_FLAG_NO_BUFFERING);
	if (audio_error)
	{
		_ASSERT(0);
		return;
	}	
	/*
	//
	*/
	audio_record(&audio_file_handle);
	
}

/*
// starts sampling when record button is pressed
*/
static void record_pressed(void)
{
	if (audio_recording)
	{
		audio_stop_recording();
		lg_label_set_string(button1_icon, (unsigned char*) "\2");
	}
	else
	{
		if (!audio_playback)
		{
			static const unsigned char stop[] = { 0x5B, 0x00};
			start_sampling();
			lg_label_set_string(button1_icon, (unsigned char*) stop);
		}
	}
}

static void volume_down(void)
{
	if (volume_level > 0)
	{
		audio_set_volume(--volume_level);
		update_volume_bar();
	}
}

static void volume_up(void)
{
	if (volume_level < 15)
	{
		audio_set_volume(++volume_level);
		update_volume_bar();	
	}
}

static void update_volume_bar(void)
{
	unsigned char c;
	/*
	//
	*/	
	for (c = 0; c < 15; c++)
	{
		if (volume_level > c)
		{
			s_volume_bar[c] = (unsigned char) '|';	
		}	
		else 
		{
			s_volume_bar[c] = 0x20;
		}		
	}
	
	lg_label_set_string(volume_bar, s_volume_bar);	
}

static void play_prev_file(void)
{
	unsigned int i;
	
	if (file_index)
	{
		file_index--;
	}
	else
	{
		file_index = file_count - 1;	
	}
	if (audio_playback)
	{
		stop_playback();
		for (i = 0; i < 0xFFFF; i++)
		{
			//for (temp = 0; temp < 0xF; temp++)
			//{
				audio_tasks();	
			//}	
		}	
		load_file(1);
	}
	else
	{
		load_file(0);	
	}
}

static void play_next_file(void)
{
	unsigned int i;
	file_index++;
	if (file_index == file_count)
	{
		file_index = 0;
	}
	if (audio_playback)
	{
		stop_playback();
		for (i = 0; i < 0xFFFF; i++)
			audio_tasks();	
		load_file(1);
	}
	else
	{
		load_file(0);
	}
}

/*
//
*/
static void load_file(char play)
{
	SM_QUERY q;
	SM_DIRECTORY_ENTRY entry;
	uint16_t i;
	uint16_t f_index = 0;
	unsigned int temp;
	
	if (!file_count)
		return;
	
	char path[54];
	path[0] = 'x';
	path[1] = ':';
	path[2] = '\\';
	path[53] = 0;

	if (audio_playback)
	{
		/*
		// stop playback
		*/
		stop_playback();
		/*
		// update UI
		*/
		lg_label_set_string(button5_icon, ICON_PLAY);
		lg_label_set_color(button1_icon, 0xFF0000);
		lg_label_set_string(button1_icon, ICON_RECORD);
		lg_label_set_string(button2_icon, ICON_RECORD);
		lg_label_set_visibility(volume_bar, 0);
	}
	else
	{
		char found = 0;		
		/*
		// find the 1st entry in the root folder
		*/
		i = sm_find_first_entry("x:\\", 0, &entry, &q);
		if (i != SM_SUCCESS)
		{
			_ASSERT(0);	
		}	
		/*
		//
		*/
		while (*entry.name != 0)
		{
			if ((entry.attributes & SM_ATTR_DIRECTORY) == 0)
			{
				if (get_file_type(entry.name) != AUDIO_FILE_UNKNOWN)
				{
					if (f_index == file_index)
					{
						update_song_name(entry.name);
						
						temp = strlen(entry.name);
						for (i = 0; i < temp; i++)
						{
							path[i + 3] = entry.name[i];					
						}
						path[i + 3] = 0;
						found = 1;
						break;
					}
					/*
					// increase the file count
					*/
					f_index++;
				}
			}
			i = sm_find_next_entry(&entry, &q);
			if (i != SM_SUCCESS)
			{
				_ASSERT(0);
			}
		}
		i = sm_find_close(&q);
		if (i != SM_SUCCESS)
		{
			_ASSERT(0);
		}
		if (found && play)
		{
			enum AUDIO_FILE_TYPE type;
			/*
			// open file for stream io
			*/
			i = sm_file_open(&audio_file_handle, path, 
							SM_FILE_ACCESS_READ | SM_FILE_FLAG_NO_BUFFERING);
			if (i)
			{
				_ASSERT(0);
			}	
			/*
			// get the file type
			*/
			type = get_file_type(path);
			/*
			// if it's an MP3 set the proper flag
			*/
			if (type == AUDIO_FILE_MP3)
				mp3_playback_in_progress = 1;
			/*
			// update buttons
			*/
			lg_label_set_string(button5_icon, ICON_STOP);
			lg_label_set_color(button1_icon, 0xFFFFFF);
			lg_label_set_string(button1_icon, ICON_LEFT_ARROW);
			lg_label_set_string(button2_icon, ICON_RIGHT_ARROW);
			lg_label_set_visibility(volume_bar, 1);
			/*
			// wait for the UI to update before playing
			*/
			while (ili9341_is_painting())
				ili9341_do_processing();
			/*
			// play the file
			*/
			audio_play_file(&audio_file_handle, type, 
				(AUDIO_PLAYBACK_COMPLETED) &playback_completed);
		}
	}
}

/*
//
*/
static void update_song_name(char* filename)
{
	unsigned int i;
	unsigned int temp = 16 - (strlen(filename) / 2);
	s_song_name[32] = 0;
	/*
	//
	*/
	for (i = 0; i < 32; i++)
	{
		if (i < temp)
		{
			s_song_name[i] = 0x20;
		}
		else
		{							
			if ((i - temp) < strlen(filename) - 4)
			{
				s_song_name[i] = filename[i - temp];
			}
			else
			{
				s_song_name[i] = 0x20;
			}
		}
	}
	/*
	// update the song name label
	*/		
	lg_label_set_string(song_name, s_song_name);	
}

static enum AUDIO_FILE_TYPE get_file_type(char* filename)
{
	size_t len = strlen((char*) filename);
	
	if (filename[len - 4] == '.')
	{
		if (filename[len - 3] == 'P' || filename[len - 3] == 'p')
		{
			if (filename[len - 2] == 'C' || filename[len - 2] == 'c')
			{
				if (filename[len - 1] == 'M' || filename[len - 1] == 'm')
				{
					return AUDIO_FILE_RAW;	
				}	
			}				
		}	
		if (filename[len - 3] == 'R' || filename[len - 3] == 'r')
		{
			if (filename[len - 2] == 'A' || filename[len - 2] == 'a')
			{
				if (filename[len - 1] == 'W' || filename[len - 1] == 'w')
				{
					return AUDIO_FILE_RAW;	
				}	
			}				
		}	
		if (filename[len - 3] == 'M' || filename[len - 3] == 'm')
		{
			if (filename[len - 2] == 'P' || filename[len - 2] == 'p')
			{
				if (filename[len - 1] == '3' || filename[len - 1] == '3')
				{
					return AUDIO_FILE_MP3;
				}	
			}				
		}	
	}
	return AUDIO_FILE_UNKNOWN;
}

/*
// plays back the just recorded file
*/
#if defined(OUT)
static void start_playback(void)
{
	uint16_t audio_error;
	/*
	// open file for stream io
	*/
	audio_error = sm_file_open(&audio_file_handle, "x:\\audio.pcm", 
					SM_FILE_ACCESS_READ | SM_FILE_FLAG_NO_BUFFERING);
	if (audio_error)
	{
		_ASSERT(0);
		return;
	}	
	/*
	//
	*/
	audio_play_file(&audio_file_handle, AUDIO_FILE_RAW, (AUDIO_PLAYBACK_COMPLETED) &playback_completed);
}
#endif

/*
// stop playback
*/
static void stop_playback(void)
{
	audio_stop_playback();
	mp3_playback_in_progress = 0;	
}

/*
// called back by audio driver
*/
static void playback_completed(SM_FILE* f)
{
	mp3_playback_in_progress = 0;
	play_next_file();
	load_file(1);
}

/*
// button tasks
*/
static void button_tasks(void)
{
	
	#define DEBOUNCE_DELAY		0x7FF
	#define PRESS_DELAY			0xFFFF

	static unsigned int last_button_pressed = 0;
	static char button1_press_processed = 0;
	static char button2_press_processed = 0;
	static char button3_press_processed = 0;
	static char button4_press_processed = 0;
	static char button5_press_processed = 0;
	static unsigned int button1_press_delay = PRESS_DELAY;
	static unsigned int button2_press_delay = PRESS_DELAY;
	static unsigned int button3_press_delay = PRESS_DELAY;
	static unsigned int button4_press_delay = PRESS_DELAY;
	static unsigned int button5_press_delay = PRESS_DELAY;
	
	

	#if defined(EXPLORER16)
		static time_t unmount_pressed_time = 0;
	#endif

	int debounce_delay;
	int press_delay;
	
	/*
	// todo: we need to use a timer for this but
	// for now this will do
	*/
	if (mp3_playback_in_progress)
	{
		debounce_delay = 0x7FF; //0x7F;
		press_delay = 0xFFFF; //0x1FF;		
	}
	else
	{
		debounce_delay = 0x7FF;
		press_delay = 0xFFFF;
	}
	#undef DEBOUNCE_DELAY
	#undef PRESS_DELAY
	#define DEBOUNCE_DELAY		debounce_delay
	#define PRESS_DELAY			press_delay

	/*
	// check for start/stop sampling button press
	*/
	if (BUTTON1 == 0)
	{
		if (!last_button_pressed)
		{
			if (!button1_press_processed)
			{
				if (!button1_press_delay)
				{
					if (audio_playback)
					{
						volume_down();
						button1_press_delay = PRESS_DELAY / 2;

					}
					else
					{
						record_pressed();
						button1_press_processed = 1;
					}
				}
				else
				{
					button1_press_delay--;
				}
			}
		}	
	}
	else
	{
		if (button1_press_processed)
		{
			button1_press_processed = 0;
			last_button_pressed = 0x7FF;
		}
		button1_press_delay = PRESS_DELAY;
	}
	/*
	// button2
	*/
	if (!BUTTON2)
	{
		if (!last_button_pressed)
		{
			if (!button2_press_processed)
			{
				if (!button2_press_delay)
				{
					if (audio_playback)
					{
						volume_up();
						button2_press_delay = PRESS_DELAY / 2;

					}
					else
					{
						//record_pressed();
						button2_press_processed = 1;
					}
				}
				else
				{
					button2_press_delay--;
				}
			}
		}	
	}
	else
	{
		if (button2_press_processed)
		{
			button2_press_processed = 0;
			last_button_pressed = 0x7FF;
		}
		button2_press_delay = PRESS_DELAY;
	}
	/*
	// check for play/stop button press
	*/
	if (!BUTTON3)
	{
		if (!last_button_pressed)
		{
			if (!button3_press_processed)
			{
				if (!button3_press_delay)
				{
					play_prev_file();
					button3_press_processed = 1;
				}
				else
				{
					button3_press_delay--;
				}
			}
		}
	}
	else
	{
		if (button3_press_processed)
		{
			button3_press_processed = 0;
			last_button_pressed = 0x7FF;
		}
		button3_press_delay = PRESS_DELAY;
	}
	/*
	// check  for button4 press
	*/
	if (!BUTTON4)
	{
		if (!last_button_pressed)
		{
			if (!button4_press_processed)
			{
				if (!button4_press_delay)
				{
					play_next_file();
					button4_press_processed = 1;
				}
				else
				{
					button4_press_delay--;
				}
			}
		}
	}
	else
	{
		if (button4_press_processed)
		{
			button4_press_processed = 0;
			last_button_pressed = 0x7FF;
		}
		button4_press_delay = PRESS_DELAY;
	}
	/*
	// check  for button5 press
	*/
	if (!BUTTON3 && !BUTTON4)
	{
		/*
		// reset the delay counter for the
		// individual buttons
		*/
		button3_press_delay = PRESS_DELAY;
		button4_press_delay = PRESS_DELAY;
		
		if (!last_button_pressed)
		{
			if (!button5_press_processed)
			{
				if (!button5_press_delay)
				{
					/*
					// load and play the next file
					*/
					load_file(1);
					button5_press_processed = 1;
				}
				else
				{
					button5_press_delay--;
				}
			}
		}
	}
	else
	{
		if (button5_press_processed)
		{
			button5_press_processed = 0;
			last_button_pressed = 0x7FF;
		}
		button5_press_delay = PRESS_DELAY;
	}
	/*
	// check for dismount button press
	*/
	#if defined(EXPLORER16)
		if (!IO_PIN_READ(D, 6))
		{
			if (!unmount_pressed_time)
			{
				time(&unmount_pressed_time);
			}
			else if (unmount_pressed_time + 2 < time(0))
			{
				sm_dismount_volume("x:");
			}
		}
		else
		{
			unmount_pressed_time = 0;
		}
	#endif
	/*
	// decrease the debounce counter
	*/
	if (last_button_pressed)
		last_button_pressed--;
	
}

/*
// background processing routine
*/
static void idle_processing(void)
{
	static char last_min = -1;
	static time_t last_time = 0;
	#if defined(EXPLORER16) || defined(USE_ILI9341)
		static struct tm* timeinfo;
		static char time_string[7];
		static char date_string[11];
	#endif
	static unsigned int sd_activity_hist = 0;
	
	/*
	// decrement SD activity indicator hysteresis counter
	*/	
	if (sd_activity_hist)
	{
		sd_activity_hist--;	
	}	
	/*
	// check SD activity
	*/
	if (SD_CARD_ACTIVITY_LED_PORT & (1 << SD_CARD_ACTIVITY_LED_PIN))
	{
		sd_activity_hist = 0x7FF;	
	}
	/*
	// if we're currently doing any work reset
	// the sleep timer
	*/
	if (audio_playback || audio_recording)
	{
		sleep_timer = SLEEP_TIMER;		
	}
	/*
	// this code runs once per second. it toggles the heartbeat LED and 
	// updates the LCD date/time.
	*/	
	if (last_time < time(0))
	{
		/*
		// update lcd
		*/
		time(&last_time);
		/*
		// toggle heartbeat LED
		*/
		HEARTBEAT ^= 1;
		/*
		// decrement the sleep timer and if it
		// reaches 0 put the CPU to sleep
		*/
		if (--sleep_timer == 0)
		{
			/*
			// make sure heartbeat LED is not
			// drawing power
			*/
			HEARTBEAT = 0;
			/*
			// put LCD on sleep mode
			*/
			ili9341_sleep();
			ILI9341_BLU = 0;
			/*
			// sleep_timer will be reset when an
			// interrupt on change is triggered. Until that
			// happens we'll keep going to sleep. We may wake
			// up every second if a software RTC is employed
			// In that case the clock will be updated by the
			// interrupt so we just go back to sleep.
			*/			
			while (!sleep_timer)
			{
				#if defined(NO_LP_OSC)
					/*
					// enable watchdog timer
					*/
					RCONbits.SWDTEN = 1;
					/*
					// put CPU on sleep mode
					*/
					__asm__("pwrsav #0");
					/*
					// disable watchdog timer
					*/
					RCONbits.SWDTEN = 0;
					/*
					// if we've been woken up by the WDT just
					// increment the clock and go back to sleep
					// this is not accurate and should not be used
					// except during development if there's no watch
					// cristal onboard. It also consumes more power
					// than using a 32.768 Khz crystal
					*/			
					if (!sleep_timer)
					{
						rtc_time_increment();
					}
				#else
					/*
					// put CPU on sleep mode
					*/
					__asm__("pwrsav #0");
				#endif
			}
			/*
			// wake up the LCD
			*/
			ILI9341_BLU = 1;
			ili9341_wake();
		}
		/*
		// update the date and time
		*/	
		#if defined(EXPLORER16) || defined(USE_ILI9341)
			timeinfo = localtime(&last_time);
			if (timeinfo->tm_min != last_min)
			{
				sprintf(date_string, "%02d/%02d/%d", timeinfo->tm_mon + 1, timeinfo->tm_mday, timeinfo->tm_year + 1900);
				sprintf(time_string, "%02d:%02d ", timeinfo->tm_hour, timeinfo->tm_min);
				last_min = timeinfo->tm_min;
				/*
				// write time and date to lcd
				*/
				#if defined(USE_ILI9341)
					lg_label_set_string(time_label, (unsigned char*) time_string);
					lg_label_set_string(date_label, (unsigned char*) date_string);
				#elif defined(EXPLORER16)
					lcd_set_pos(&lcd_driver, 0, 3);
					lcd_write_string(&lcd_driver, (unsigned char*) date_string);
					lcd_set_pos(&lcd_driver, 1, 4);
					lcd_write_string(&lcd_driver, (unsigned char*) time_string);
				#endif
			}
			/*
			// flash disk activity indicator
			*/
			#if defined(USE_ILI9341)
				if (sd_activity_hist)
				{
					lg_label_set_color(volume_icon, HEARTBEAT ? 0xFFFFFF : 0xFF0000);
				}	
				else
				{
					lg_label_set_color(volume_icon, 0xFFFFFF);
				}
			#elif defined(EXPLORER16)
				if (sd_activity_hist)
				{
					lcd_set_pos(&lcd_driver, 0, 15);
					lcd_write_char(&lcd_driver, (IO_PIN_READ(A, 7)) ? 0xF3 : ' ');
				}
			#endif		
		#endif		
	}
	/*
	// update media ready LED
	*/
	#if defined(EXPLORER16)
		/*
		// update media ready LCD indicator
		*/
		if (IO_PIN_READ(F, 2) == 0)
		{
			if (!IO_PIN_READ(A, 5))
			{
				lcd_set_pos(&lcd_driver, 0, 15);
				lcd_write_char(&lcd_driver, 0xF3);
			}
		}
		else
		{
			lcd_set_pos(&lcd_driver, 0, 15);
			lcd_write_char(&lcd_driver, 0x20);
		}
	#endif
	/*
	// lcd driver processing
	*/
	#if defined(USE_ILI9341)
		ili9341_do_processing();
	#elif defined(EXPLORER16)
		lcd_idle_processing(&lcd_driver);
	#endif
	/*
	// SD driver tasks
	*/
	sd_idle_processing(&sd_card);
	/*
	// audio driver tasks
	*/
	audio_tasks();
	/*
	// button tasks
	*/
	button_tasks();
}

/*
// DMA interrupts for SD driver
*/
void __attribute__((__interrupt__, __auto_psv__)) _DMA0Interrupt(void) 
{
	SD_DMA_CHANNEL_1_INTERRUPT(&sd_card);
}

void __attribute__((__interrupt__, __auto_psv__)) _DMA1Interrupt(void) 
{
	SD_DMA_CHANNEL_2_INTERRUPT(&sd_card);
}

/*
// this interrupt is used to wake the CPU from sleep mode
*/
void __attribute__((__interrupt__, __auto_psv__)) _CNInterrupt(void)
{
	sleep_timer = SLEEP_TIMER;
	IFS1bits.CNIF = 0;
}

static uint32_t pc;
static char errorTrap[30];
static char errorInfo[20];
unsigned int addressErrorInfo = 0;

/*
// trap for AddressError
*/
void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _AddressError(void)
{
	SPLIM = 0xFFFF;
	/*
	// get the program counnter from stack
	*/
	pc = __PC();	
	/*
	// get the value of the PC before trap
	*/
	sprintf(errorInfo, "xptr: 0x%04x", addressErrorInfo);
	sprintf(errorTrap, "Address Error: 0x%02x%04x", (uint16_t) (pc >> 16), (uint16_t) (pc & 0xFFFF));
	lg_label_set_string(song_name, (unsigned char*) errorTrap);	
	lg_label_set_string(volume_bar, (unsigned char*) errorInfo);
	while (1)
	{
		//for (pc = 0; pc < 0x7FFFF; pc++);
		//HEARTBEAT ^= 1;
		ili9341_do_processing();
	}
	//HALT();
	/*
	// clear interrupt flag
	*/
	INTCON1bits.ADDRERR = 0;
}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _SoftTrapError(void)
{
	SPLIM = 0xFFFF;
	/*
	// get the program counnter from stack
	*/
	pc = __PC();	
	/*
	// get the value of the PC before trap
	*/
	sprintf(errorTrap, "SoftTrap Error: 0x%02x%04x", (uint16_t) (pc >> 16), (uint16_t) (pc & 0xFFFF));
	lg_label_set_string(song_name, (unsigned char*) errorTrap);	
	while (1)
	{
		for (pc = 0; pc < 0x7FFFF; pc++);
		HEARTBEAT ^= 1;
		ili9341_do_processing();
	}
}

#if defined(__dsPIC33E__)
void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _HardTrapError(void)
{
	SPLIM = 0xFFFF;
	/*
	// get the program counnter from stack
	*/
	pc = __PC();	
	/*
	// get the value of the PC before trap
	*/
	sprintf(errorTrap, "Hard Trap: 0x%02x%04x", (uint16_t) (pc >> 16), (uint16_t) (pc & 0xFFFF));
	lg_label_set_string(song_name, (unsigned char*) errorTrap);	
	while (1)
	{
		for (pc = 0; pc < 0x7FFFF; pc++);
		HEARTBEAT ^= 1;
		ili9341_do_processing();
	}
}
#endif

void __attribute__((__interrupt__, __shadow__, __no_auto_psv__)) _StackError(void)
{
	SPLIM = 0xFFFF;
	while (1)
	{
		for (pc = 0; pc < 0x7FFFF; pc++);
		HEARTBEAT ^= 1;
	}
	//HALT();
	/*
	// clear interrupt flag
	*/
	INTCON1bits.STKERR = 0;
}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _MathError(void)
{
	if (INTCON1bits.OVAERR || INTCON1bits.OVBERR)
	{
		INTCON1bits.OVAERR = 0;
		INTCON1bits.OVBERR = 0;
		INTCON1bits.MATHERR = 0;
		return;	
	}
	
	SPLIM = 0xFFFF;
	
	//while (1)
	//{
	//	for (pc = 0; pc < 0x9Ff; pc++);
	//	HEARTBEAT ^= 1;
	//	//ili9341_do_processing();
	//}	
	/*
	// get the program counnter from stack
	*/
	pc = __PC();	
	/*
	// get the value of the PC before trap
	*/
	sprintf(errorTrap, "Math Error: 0x%02x%04x", (uint16_t) (pc >> 16), (uint16_t) (pc & 0xFFFF));
	lg_label_set_string(song_name, (unsigned char*) errorTrap);	
	while (1)
	{
		ili9341_do_processing();
	}
	//HALT();
	/*
	// clear interrupt flag
	*/
	INTCON1bits.MATHERR = 0;
}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _DMACError(void)
{
	SPLIM = 0xFFFF;
	/*
	// get the program counnter from stack
	*/
	pc = __PC();	
	/*
	// get the value of the PC before trap
	*/
	sprintf(errorTrap, "DMAC Error: 0x%02x%04x", (uint16_t) (pc >> 16), (uint16_t) (pc & 0xFFFF));
	lg_label_set_string(song_name, (unsigned char*) errorTrap);	
	while (1)
	{
		ili9341_do_processing();
	}
	//HALT();
	INTCON1bits.DMACERR = 0;
}

void __attribute__((__interrupt__, __shadow__, __no_auto_psv__)) _OscillatorFail(void)
{
	while (1)
	{
		for (pc = 0; pc < 0x9Ff; pc++);
		HEARTBEAT ^= 1;
		//ili9341_do_processing();
	}	
	//HALT();
}

