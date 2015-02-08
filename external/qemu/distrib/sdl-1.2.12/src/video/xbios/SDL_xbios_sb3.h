
#include "SDL_config.h"


#ifndef _SDL_xbios_sb3_h_
#define _SDL_xbios_sb3_h_

/*--- Defines ---*/

#ifndef C_SCPN
#define C_SCPN 0x5343504EL
#endif

#define SCPN_DEV_1BPP	0
#define SCPN_DEV_2BPP	1
#define SCPN_DEV_4BPP	2
#define SCPN_DEV_8BPP	3
#define SCPN_DEV_16BPP	4

extern const int SDL_XBIOS_scpn_planes_device[];

/*--- Types ---*/

typedef struct {
	unsigned short	virtual_width;	/* Virtual screen width */
	unsigned short	virtual_height;	/* Virtual screen height */
	unsigned short	visible_width;	/* Visible width */
	unsigned short	visible_height;	/* Visible height */
	unsigned short	h_pos;	/* Horizontal position in virtual screen */
	unsigned short	v_pos;	/* Vertical position in virtual screen */
	unsigned short	dummy;
	unsigned long	size;	/* Size of screen in bytes */
	unsigned short	device;	/* Device number to find planes = getRez() */
							/* = Index in scpn_planes_device[] */
} __attribute__((packed)) scpn_screeninfo_t;

typedef struct {
	unsigned long	magic;	/* just a BRA assembler jump */
	unsigned short	version;
	void			*dummy1;
	unsigned short	ptsout0_1;
	unsigned short	ptsout0_2;
	unsigned short	dummy3;
	unsigned char	date[8];	/* Date of program build */
	unsigned char	asm_string[30];	/* 10 times the 'ASM' string */
	unsigned short	dummy4;
	scpn_screeninfo_t	*screen_info;
	unsigned short	dummy6;
} __attribute__((packed)) scpn_cookie_t;

/*--- Function prototypes ---*/

int SDL_XBIOS_SB3Usable(scpn_cookie_t *cookie_scpn);

void SDL_XBIOS_SB3Init(_THIS, scpn_cookie_t *cookie_scpn);

#endif /* _SDL_xbios_sb3_h_ */
