#ifndef __ASMARM_ELF_H
#define __ASMARM_ELF_H



#include <asm/ptrace.h>
#include <asm/user.h>
#ifdef __KERNEL
#include <asm/procinfo.h>
#endif

typedef unsigned long elf_greg_t;
typedef unsigned long elf_freg_t[3];

#define EM_ARM	40
#define EF_ARM_APCS26 0x08
#define EF_ARM_SOFT_FLOAT 0x200
#define EF_ARM_EABI_MASK 0xFF000000

#define R_ARM_NONE	0
#define R_ARM_PC24	1
#define R_ARM_ABS32	2
#define R_ARM_CALL	28
#define R_ARM_JUMP24	29

#define ELF_NGREG (sizeof (struct pt_regs) / sizeof(elf_greg_t))
typedef elf_greg_t elf_gregset_t[ELF_NGREG];

typedef struct user_fp elf_fpregset_t;

#define elf_check_arch(x) ( ((x)->e_machine == EM_ARM) && (ELF_PROC_OK((x))) )

#define ELF_CLASS	ELFCLASS32
#ifdef __ARMEB__
#define ELF_DATA	ELFDATA2MSB
#else
#define ELF_DATA	ELFDATA2LSB
#endif
#define ELF_ARCH	EM_ARM

#define USE_ELF_CORE_DUMP
#define ELF_EXEC_PAGESIZE	4096


#define ELF_ET_DYN_BASE	(2 * TASK_SIZE / 3)

#define ELF_PLAT_INIT(_r, load_addr)	(_r)->ARM_r0 = 0


#define ELF_HWCAP	(elf_hwcap)



#define ELF_PLATFORM_SIZE 8
extern char elf_platform[];
#define ELF_PLATFORM	(elf_platform)

#ifdef __KERNEL__

#define ELF_PROC_OK(x)	(ELF_THUMB_OK(x) && ELF_26BIT_OK(x))

#define ELF_THUMB_OK(x) \
	(( (elf_hwcap & HWCAP_THUMB) && ((x)->e_entry & 1) == 1) || \
	 ((x)->e_entry & 3) == 0)

#define ELF_26BIT_OK(x) \
	(( (elf_hwcap & HWCAP_26BIT) && (x)->e_flags & EF_ARM_APCS26) || \
	  ((x)->e_flags & EF_ARM_APCS26) == 0)

#ifndef CONFIG_IWMMXT

#define SET_PERSONALITY(ex,ibcs2) \
	set_personality(((ex).e_flags&EF_ARM_APCS26 ?PER_LINUX :PER_LINUX_32BIT))

#else


#define SET_PERSONALITY(ex,ibcs2) \
do { \
	set_personality(PER_LINUX_32BIT); \
	if (((ex).e_flags & EF_ARM_EABI_MASK) || \
	    ((ex).e_flags & EF_ARM_SOFT_FLOAT)) \
		set_thread_flag(TIF_USING_IWMMXT); \
	else \
		clear_thread_flag(TIF_USING_IWMMXT); \
} while (0)

#endif

struct task_struct;

extern int dump_task_regs (struct task_struct *, elf_gregset_t *);

#define ELF_CORE_COPY_TASK_REGS(tsk, elf_regs) dump_task_regs(tsk, elf_regs)


#endif

#endif
