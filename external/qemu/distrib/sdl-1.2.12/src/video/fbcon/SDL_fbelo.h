
#include "SDL_config.h"

#ifndef SDL_fbelo_h
#define SDL_fbelo_h

#include "SDL_fbvideo.h"

/* ELO */
#define ELO_PACKET_SIZE	10
#define ELO_START_BYTE		'U'

void eloConvertXY(_THIS, int *dx,  int *dy);

int eloInitController(int fd);

int eloParsePacket(unsigned char* mousebuf, int* dx, int* dy, int* button_state);


int eloReadPosition(_THIS, int fd, int* x, int* y, int* button_state, int* realx, int* realy);

#endif	/* SDL_fbelo_h */
