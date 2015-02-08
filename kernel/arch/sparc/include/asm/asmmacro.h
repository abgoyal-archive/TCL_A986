

#ifndef _SPARC_ASMMACRO_H
#define _SPARC_ASMMACRO_H

#include <asm/btfixup.h>
#include <asm/asi.h>

#define GET_PROCESSOR4M_ID(reg) \
	rd	%tbr, %reg; \
	srl	%reg, 12, %reg; \
	and	%reg, 3, %reg;

#define GET_PROCESSOR4D_ID(reg) \
	lda	[%g0] ASI_M_VIKING_TMP1, %reg;

#define SAVE_ALL_HEAD \
	sethi	%hi(trap_setup), %l4; \
	jmpl	%l4 + %lo(trap_setup), %l6;
#define SAVE_ALL \
	SAVE_ALL_HEAD \
	 nop;

/* All traps low-level code here must end with this macro. */
#define RESTORE_ALL b ret_trap_entry; clr %l6;


#define lduXa	lduba
#define stXa	stba

#endif /* !(_SPARC_ASMMACRO_H) */
