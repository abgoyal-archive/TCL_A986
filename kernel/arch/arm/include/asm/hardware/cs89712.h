
#ifndef __ASM_HARDWARE_CS89712_H
#define __ASM_HARDWARE_CS89712_H

                                  
#define PCDR			0x0002	/* Port C Data register ---------------------------- */
#define PCDDR			0x0042	/* Port C Data Direction register ------------------ */
#define SDCONF			0x2300  /* SDRAM Configuration register ---------------------*/
#define SDRFPR			0x2340  /* SDRAM Refresh period register --------------------*/

#define SDCONF_ACTIVE		(1 << 10)
#define SDCONF_CLKCTL		(1 << 9)
#define SDCONF_WIDTH_4		(0 << 7)
#define SDCONF_WIDTH_8		(1 << 7)
#define SDCONF_WIDTH_16		(2 << 7)
#define SDCONF_WIDTH_32		(3 << 7)
#define SDCONF_SIZE_16		(0 << 5)
#define SDCONF_SIZE_64		(1 << 5)
#define SDCONF_SIZE_128		(2 << 5)
#define SDCONF_SIZE_256		(3 << 5)
#define SDCONF_CASLAT_2		(2)
#define SDCONF_CASLAT_3		(3)

#endif /* __ASM_HARDWARE_CS89712_H */
