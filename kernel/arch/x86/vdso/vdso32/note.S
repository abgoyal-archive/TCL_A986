

#include <linux/version.h>
#include <linux/elfnote.h>

ELFNOTE_START(Linux, 0, "a")
	.long LINUX_VERSION_CODE
ELFNOTE_END

#ifdef CONFIG_XEN

#include "../../xen/vdso.h"	/* Defines VDSO_NOTE_NONEGSEG_BIT.  */

ELFNOTE_START(GNU, 2, "a")
	.long 1			/* ncaps */
VDSO32_NOTE_MASK:		/* Symbol used by arch/x86/xen/setup.c */
	.long 0			/* mask */
	.byte VDSO_NOTE_NONEGSEG_BIT; .asciz "nosegneg"	/* bit, name */
ELFNOTE_END
#endif
