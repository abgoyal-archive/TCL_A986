
#include "SDL_config.h"


#ifndef _SDL_mintaudio_dma8_h
#define _SDL_mintaudio_dma8_h

#define DMAAUDIO_IO_BASE (0xffff8900)
struct DMAAUDIO_IO_S {
	unsigned char int_ctrl;
	unsigned char control;

	unsigned char dummy1;
	unsigned char start_high;
	unsigned char dummy2;
	unsigned char start_mid;
	unsigned char dummy3;
	unsigned char start_low;

	unsigned char dummy4;
	unsigned char cur_high;
	unsigned char dummy5;
	unsigned char cur_mid;
	unsigned char dummy6;
	unsigned char cur_low;

	unsigned char dummy7;
	unsigned char end_high;
	unsigned char dummy8;
	unsigned char end_mid;
	unsigned char dummy9;
	unsigned char end_low;

	unsigned char dummy10[12];

	unsigned char track_ctrl; /* CODEC only */
	unsigned char sound_ctrl;
	unsigned short sound_data;
	unsigned short sound_mask;

	unsigned char dummy11[10];
	
	unsigned short dev_ctrl;
	unsigned short dest_ctrl;
	unsigned short sync_div;
	unsigned char track_rec;
	unsigned char adderin_input;
	unsigned char channel_input;
	unsigned char channel_amplification;
	unsigned char channel_reduction;
	
	unsigned char dummy12[6];

	unsigned char data_direction;
	unsigned char dummy13;
	unsigned char dev_data;
};
#define DMAAUDIO_IO ((*(volatile struct DMAAUDIO_IO_S *)DMAAUDIO_IO_BASE))

#endif /* _SDL_mintaudio_dma8_h */
