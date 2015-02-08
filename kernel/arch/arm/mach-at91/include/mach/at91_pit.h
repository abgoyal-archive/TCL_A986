

#ifndef AT91_PIT_H
#define AT91_PIT_H

#define AT91_PIT_MR		(AT91_PIT + 0x00)	/* Mode Register */
#define		AT91_PIT_PITIEN		(1 << 25)		/* Timer Interrupt Enable */
#define		AT91_PIT_PITEN		(1 << 24)		/* Timer Enabled */
#define		AT91_PIT_PIV		(0xfffff)		/* Periodic Interval Value */

#define AT91_PIT_SR		(AT91_PIT + 0x04)	/* Status Register */
#define		AT91_PIT_PITS		(1 << 0)		/* Timer Status */

#define AT91_PIT_PIVR		(AT91_PIT + 0x08)	/* Periodic Interval Value Register */
#define AT91_PIT_PIIR		(AT91_PIT + 0x0c)	/* Periodic Interval Image Register */
#define		AT91_PIT_PICNT		(0xfff << 20)		/* Interval Counter */
#define		AT91_PIT_CPIV		(0xfffff)		/* Inverval Value */

#endif
