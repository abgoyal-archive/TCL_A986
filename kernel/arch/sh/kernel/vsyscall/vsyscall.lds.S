
#include <asm/asm-offsets.h>

#ifdef CONFIG_CPU_LITTLE_ENDIAN
OUTPUT_FORMAT("elf32-sh-linux", "elf32-sh-linux", "elf32-sh-linux")
#else
OUTPUT_FORMAT("elf32-shbig-linux", "elf32-shbig-linux", "elf32-shbig-linux")
#endif
OUTPUT_ARCH(sh)

/* The ELF entry point can be used to set the AT_SYSINFO value.  */
ENTRY(__kernel_vsyscall);

SECTIONS
{
	. = SIZEOF_HEADERS;

	.hash		: { *(.hash) }			:text
	.gnu.hash	: { *(.gnu.hash) }
	.dynsym		: { *(.dynsym) }
	.dynstr		: { *(.dynstr) }
	.gnu.version	: { *(.gnu.version) }
	.gnu.version_d	: { *(.gnu.version_d) }
	.gnu.version_r	: { *(.gnu.version_r) }

	/*
	 * This linker script is used both with -r and with -shared.
	 * For the layouts to match, we need to skip more than enough
	 * space for the dynamic symbol table et al.  If this amount
	 * is insufficient, ld -shared will barf.  Just increase it here.
	 */
	. = 0x400;

	.text		: { *(.text) } 			:text	=0x90909090
	.note		: { *(.note.*) }		:text	:note
	.eh_frame_hdr	: { *(.eh_frame_hdr ) }		:text	:eh_frame_hdr
	.eh_frame	: {
		KEEP (*(.eh_frame))
		LONG (0)
	}						:text
	.dynamic	: { *(.dynamic) }		:text	:dynamic
	.useless	: {
	      *(.got.plt) *(.got)
	      *(.data .data.* .gnu.linkonce.d.*)
	      *(.dynbss)
	      *(.bss .bss.* .gnu.linkonce.b.*)
	}						:text
}

#define PT_GNU_EH_FRAME	0x6474e550

PHDRS
{
	text		PT_LOAD FILEHDR PHDRS FLAGS(5);	/* PF_R|PF_X */
	dynamic		PT_DYNAMIC FLAGS(4);		/* PF_R */
	note		PT_NOTE FLAGS(4);		/* PF_R */
	eh_frame_hdr	PT_GNU_EH_FRAME;
}

VERSION
{
	LINUX_2.6 {
	global:
		__kernel_vsyscall;
		__kernel_sigreturn;
		__kernel_rt_sigreturn;

	local: *;
	};
}
