
#include <asm-generic/vmlinux.lds.h>
/* needed for the processor specific cache alignment size */	
#include <asm/cache.h>
#include <asm/page.h>
#include <asm/asm-offsets.h>
#include <asm/thread_info.h>
	
/* ld script to make hppa Linux kernel */
#ifndef CONFIG_64BIT
OUTPUT_FORMAT("elf32-hppa-linux")
OUTPUT_ARCH(hppa)
#else
OUTPUT_FORMAT("elf64-hppa-linux")
OUTPUT_ARCH(hppa:hppa2.0w)
#endif

ENTRY(_stext)
#ifndef CONFIG_64BIT
jiffies = jiffies_64 + 4;
#else
jiffies = jiffies_64;
#endif
SECTIONS
{
	. = KERNEL_BINARY_TEXT_START;

	_text = .;		/* Text and read-only data */
	.text ALIGN(16) : {
		HEAD_TEXT
		TEXT_TEXT
		SCHED_TEXT
		LOCK_TEXT
		KPROBES_TEXT
		IRQENTRY_TEXT
		*(.text.do_softirq)
		*(.text.sys_exit)
		*(.text.do_sigaltstack)
		*(.text.do_fork)
		*(.text.*)
		*(.fixup)
		*(.lock.text)		/* out-of-line lock text */
		*(.gnu.warning)
	} = 0
	/* End of text section */
	_etext = .;

	RODATA

	/* writeable */
	/* Make sure this is page aligned so
	 * that we can properly leave these
	 * as writable
	 */
	. = ALIGN(PAGE_SIZE);
	data_start = .;

	/* unwind info */
	.PARISC.unwind : {
		__start___unwind = .;
		*(.PARISC.unwind)
		__stop___unwind = .;
	}

	EXCEPTION_TABLE(16)
	NOTES

	/* Data */
	RW_DATA_SECTION(L1_CACHE_BYTES, PAGE_SIZE, THREAD_SIZE)

	/* PA-RISC locks requires 16-byte alignment */
	. = ALIGN(16);
	.data..lock_aligned : {
		*(.data..lock_aligned)
	}

	/* End of data section */
	_edata = .;

	/* BSS */
	__bss_start = .;
	/* page table entries need to be PAGE_SIZE aligned */
	. = ALIGN(PAGE_SIZE);
	.data..vmpages : {
		*(.data..vm0.pmd)
		*(.data..vm0.pgd)
		*(.data..vm0.pte)
	}
	.bss : {
		*(.bss)
		*(COMMON)
	}
	__bss_stop = .;

#ifdef CONFIG_64BIT
	. = ALIGN(16);
	/* Linkage tables */
	.opd : {
		*(.opd)
	} PROVIDE (__gp = .); 
	.plt : {
		*(.plt)
	} 
	.dlt : {
		*(.dlt)
	}
#endif

	/* reserve space for interrupt stack by aligning __init* to 16k */
	. = ALIGN(16384);
	__init_begin = .;
	INIT_TEXT_SECTION(16384)
	INIT_DATA_SECTION(16)
	/* we have to discard exit text and such at runtime, not link time */
	.exit.text :
	{
		EXIT_TEXT
	}
	.exit.data :
	{
		EXIT_DATA
	}

	PERCPU(PAGE_SIZE)
	. = ALIGN(PAGE_SIZE);
	__init_end = .;
	/* freed after init ends here */
	_end = . ;

	STABS_DEBUG
	.note 0 : { *(.note) }

	/* Sections to be discarded */
	DISCARDS
	/DISCARD/ : {
#ifdef CONFIG_64BIT
		/* temporary hack until binutils is fixed to not emit these
	 	 * for static binaries
		 */
		*(.interp)
		*(.dynsym)
		*(.dynstr)
		*(.dynamic)
		*(.hash)
		*(.gnu.hash)
#endif
	}
}
