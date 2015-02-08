
#ifdef __ASSEMBLY__


#include <asm/romimage-macros.h>
#include "partner-jet-setup.txt"

	/* execute icbi after enabling cache */
	mov.l	1f, r0
	icbi	@r0

	/* jump to cached area */
	mova	2f, r0
	jmp	@r0
	nop

	.align 2
1 :	.long 0xa8000000
2 :

#else /* __ASSEMBLY__ */

#define HIZCRA		0xa4050158
#define PGDR		0xa405012c

extern inline void mmcif_update_progress(int nr)
{
	/* disable Hi-Z for LED pins */
	__raw_writew(__raw_readw(HIZCRA) & ~(1 << 1), HIZCRA);

	/* update progress on LED4, LED5, LED6 and LED7 */
	__raw_writeb(1 << (nr - 1), PGDR);
}

#endif /* __ASSEMBLY__ */
