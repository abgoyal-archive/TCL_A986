

#ifndef _S6000_I2S_H
#define _S6000_I2S_H

extern struct snd_soc_dai s6000_i2s_dai;

struct s6000_snd_platform_data {
	int lines_in;
	int lines_out;
	int channel_in;
	int channel_out;
	int wide;
	int same_rate;
};
#endif
