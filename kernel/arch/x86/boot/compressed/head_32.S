


	.text

#include <linux/init.h>
#include <linux/linkage.h>
#include <asm/segment.h>
#include <asm/page_types.h>
#include <asm/boot.h>
#include <asm/asm-offsets.h>

	__HEAD
ENTRY(startup_32)
	cld
	/*
	 * Test KEEP_SEGMENTS flag to see if the bootloader is asking
	 * us to not reload segments
	 */
	testb	$(1<<6), BP_loadflags(%esi)
	jnz	1f

	cli
	movl	$__BOOT_DS, %eax
	movl	%eax, %ds
	movl	%eax, %es
	movl	%eax, %fs
	movl	%eax, %gs
	movl	%eax, %ss
1:

	leal	(BP_scratch+4)(%esi), %esp
	call	1f
1:	popl	%ebp
	subl	$1b, %ebp


#ifdef CONFIG_RELOCATABLE
	movl	%ebp, %ebx
	movl	BP_kernel_alignment(%esi), %eax
	decl	%eax
	addl    %eax, %ebx
	notl	%eax
	andl    %eax, %ebx
#else
	movl	$LOAD_PHYSICAL_ADDR, %ebx
#endif

	/* Target address to relocate to for decompression */
	addl	$z_extract_offset, %ebx

	/* Set up the stack */
	leal	boot_stack_end(%ebx), %esp

	/* Zero EFLAGS */
	pushl	$0
	popfl

	pushl	%esi
	leal	(_bss-4)(%ebp), %esi
	leal	(_bss-4)(%ebx), %edi
	movl	$(_bss - startup_32), %ecx
	shrl	$2, %ecx
	std
	rep	movsl
	cld
	popl	%esi

	leal	relocated(%ebx), %eax
	jmp	*%eax
ENDPROC(startup_32)

	.text
relocated:

	xorl	%eax, %eax
	leal	_bss(%ebx), %edi
	leal	_ebss(%ebx), %ecx
	subl	%edi, %ecx
	shrl	$2, %ecx
	rep	stosl

	leal	z_extract_offset_negative(%ebx), %ebp
				/* push arguments for decompress_kernel: */
	pushl	%ebp		/* output address */
	pushl	$z_input_len	/* input_len */
	leal	input_data(%ebx), %eax
	pushl	%eax		/* input_data */
	leal	boot_heap(%ebx), %eax
	pushl	%eax		/* heap area */
	pushl	%esi		/* real mode pointer */
	call	decompress_kernel
	addl	$20, %esp

#if CONFIG_RELOCATABLE
	leal	z_output_len(%ebp), %edi

	movl	%ebp, %ebx
	subl	$LOAD_PHYSICAL_ADDR, %ebx
	jz	2f	/* Nothing to be done if loaded at compiled addr. */

1:	subl	$4, %edi
	movl	(%edi), %ecx
	testl	%ecx, %ecx
	jz	2f
	addl	%ebx, -__PAGE_OFFSET(%ebx, %ecx)
	jmp	1b
2:
#endif

	xorl	%ebx, %ebx
	jmp	*%ebp

	.bss
	.balign 4
boot_heap:
	.fill BOOT_HEAP_SIZE, 1, 0
boot_stack:
	.fill BOOT_STACK_SIZE, 1, 0
boot_stack_end:
