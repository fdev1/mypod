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

#include <sdlib/sd.h>
#include <fat32lib/storage_device.h>
#include <fat32lib/fat.h>
#include <fat32lib/filesystem_interface.h>
#include <smlib/sm.h>

typedef void (*AUDIO_PLAYBACK_COMPLETED)(SM_FILE* file);

extern char __attribute__((near)) audio_recording;
extern char __attribute__((near)) audio_playback;

enum AUDIO_FILE_TYPE
{
	AUDIO_FILE_RAW = 0x0001,
	AUDIO_FILE_MP3 = 0x0002,	
	AUDIO_FILE_UNKNOWN = 0x0003
};


void audio_init(void);
void audio_record(SM_FILE* file);
void audio_play_file(SM_FILE* file, enum AUDIO_FILE_TYPE type, AUDIO_PLAYBACK_COMPLETED callback);
void audio_stop_recording(void);
void audio_stop_playback(void);
void audio_tasks(void);
void audio_set_volume(unsigned char volume_level);
