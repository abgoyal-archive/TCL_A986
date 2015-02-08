

#ifndef _WM8731_H
#define _WM8731_H

/* WM8731 register space */

#define WM8731_LINVOL   0x00
#define WM8731_RINVOL   0x01
#define WM8731_LOUT1V   0x02
#define WM8731_ROUT1V   0x03
#define WM8731_APANA    0x04
#define WM8731_APDIGI   0x05
#define WM8731_PWR      0x06
#define WM8731_IFACE    0x07
#define WM8731_SRATE    0x08
#define WM8731_ACTIVE   0x09
#define WM8731_RESET	0x0f

#define WM8731_CACHEREGNUM 	10

#define WM8731_SYSCLK	0
#define WM8731_DAI		0

extern struct snd_soc_dai wm8731_dai;
extern struct snd_soc_codec_device soc_codec_dev_wm8731;

#endif
