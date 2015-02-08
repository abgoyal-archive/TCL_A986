

#include <linux/linkage.h>
#include <asm/dwarf2.h>

ENTRY(__iowrite32_copy)
	CFI_STARTPROC
	movl %edx,%ecx
	rep movsd
	ret
	CFI_ENDPROC
ENDPROC(__iowrite32_copy)
