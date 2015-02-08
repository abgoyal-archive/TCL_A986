

#include <linux/linkage.h>
#include <asm/alternative-asm.h>
#include <asm/frame.h>
#include <asm/dwarf2.h>


.text

ENTRY(cmpxchg8b_emu)
CFI_STARTPROC

#
# Emulate 'cmpxchg8b (%esi)' on UP except we don't
# set the whole ZF thing (caller will just compare
# eax:edx with the expected value)
#
cmpxchg8b_emu:
	pushfl
	cli

	cmpl  (%esi), %eax
	jne not_same
	cmpl 4(%esi), %edx
	jne half_same

	movl %ebx,  (%esi)
	movl %ecx, 4(%esi)

	popfl
	ret

 not_same:
	movl  (%esi), %eax
 half_same:
	movl 4(%esi), %edx

	popfl
	ret

CFI_ENDPROC
ENDPROC(cmpxchg8b_emu)
