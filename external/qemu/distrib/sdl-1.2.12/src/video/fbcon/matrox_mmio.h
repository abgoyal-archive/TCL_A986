
#include "SDL_config.h"

/* MGA register definitions */

#include "matrox_regs.h"

/* MGA control macros */

#define mga_in8(reg)		*(volatile Uint8  *)(mapped_io + (reg))
#define mga_in32(reg)		*(volatile Uint32 *)(mapped_io + (reg))

#define mga_out8(reg,v)		*(volatile Uint8  *)(mapped_io + (reg)) = v;
#define mga_out32(reg,v)	*(volatile Uint32 *)(mapped_io + (reg)) = v;


/* Wait for fifo space */
#define mga_wait(space)							\
{									\
	while ( mga_in8(MGAREG_FIFOSTATUS) < space )			\
		;							\
}


/* Wait for idle accelerator */
#define mga_waitidle()							\
{									\
	while ( mga_in32(MGAREG_STATUS) & 0x10000 )			\
		;							\
}

