
#ifndef __ASM_ALPHA_ELF_H
#define __ASM_ALPHA_ELF_H

#include <asm/auxvec.h>

/* Special values for the st_other field in the symbol table.  */

#define STO_ALPHA_NOPV		0x80
#define STO_ALPHA_STD_GPLOAD	0x88

#define R_ALPHA_NONE            0       /* No reloc */
#define R_ALPHA_REFLONG         1       /* Direct 32 bit */
#define R_ALPHA_REFQUAD         2       /* Direct 64 bit */
#define R_ALPHA_GPREL32         3       /* GP relative 32 bit */
#define R_ALPHA_LITERAL         4       /* GP relative 16 bit w/optimization */
#define R_ALPHA_LITUSE          5       /* Optimization hint for LITERAL */
#define R_ALPHA_GPDISP          6       /* Add displacement to GP */
#define R_ALPHA_BRADDR          7       /* PC+4 relative 23 bit shifted */
#define R_ALPHA_HINT            8       /* PC+4 relative 16 bit shifted */
#define R_ALPHA_SREL16          9       /* PC relative 16 bit */
#define R_ALPHA_SREL32          10      /* PC relative 32 bit */
#define R_ALPHA_SREL64          11      /* PC relative 64 bit */
#define R_ALPHA_GPRELHIGH       17      /* GP relative 32 bit, high 16 bits */
#define R_ALPHA_GPRELLOW        18      /* GP relative 32 bit, low 16 bits */
#define R_ALPHA_GPREL16         19      /* GP relative 16 bit */
#define R_ALPHA_COPY            24      /* Copy symbol at runtime */
#define R_ALPHA_GLOB_DAT        25      /* Create GOT entry */
#define R_ALPHA_JMP_SLOT        26      /* Create PLT entry */
#define R_ALPHA_RELATIVE        27      /* Adjust by program base */
#define R_ALPHA_BRSGP		28
#define R_ALPHA_TLSGD           29
#define R_ALPHA_TLS_LDM         30
#define R_ALPHA_DTPMOD64        31
#define R_ALPHA_GOTDTPREL       32
#define R_ALPHA_DTPREL64        33
#define R_ALPHA_DTPRELHI        34
#define R_ALPHA_DTPRELLO        35
#define R_ALPHA_DTPREL16        36
#define R_ALPHA_GOTTPREL        37
#define R_ALPHA_TPREL64         38
#define R_ALPHA_TPRELHI         39
#define R_ALPHA_TPRELLO         40
#define R_ALPHA_TPREL16         41

#define SHF_ALPHA_GPREL		0x10000000

/* Legal values for e_flags field of Elf64_Ehdr.  */

#define EF_ALPHA_32BIT		1	/* All addresses are below 2GB */


#define ELF_NGREG	33
#define ELF_NFPREG	32

typedef unsigned long elf_greg_t;
typedef elf_greg_t elf_gregset_t[ELF_NGREG];

typedef double elf_fpreg_t;
typedef elf_fpreg_t elf_fpregset_t[ELF_NFPREG];

#define elf_check_arch(x) ((x)->e_machine == EM_ALPHA)

#define ELF_CLASS	ELFCLASS64
#define ELF_DATA	ELFDATA2LSB
#define ELF_ARCH	EM_ALPHA

#define ELF_EXEC_PAGESIZE	8192


#define ELF_ET_DYN_BASE		(TASK_UNMAPPED_BASE + 0x1000000)


#define ELF_PLAT_INIT(_r, load_addr)	_r->r0 = 0


struct pt_regs;
struct thread_info;
struct task_struct;
extern void dump_elf_thread(elf_greg_t *dest, struct pt_regs *pt,
			    struct thread_info *ti);
#define ELF_CORE_COPY_REGS(DEST, REGS) \
	dump_elf_thread(DEST, REGS, current_thread_info());

/* Similar, but for a thread other than current.  */

extern int dump_elf_task(elf_greg_t *dest, struct task_struct *task);
#define ELF_CORE_COPY_TASK_REGS(TASK, DEST) \
	dump_elf_task(*(DEST), TASK)

/* Similar, but for the FP registers.  */

extern int dump_elf_task_fp(elf_fpreg_t *dest, struct task_struct *task);
#define ELF_CORE_COPY_FPREGS(TASK, DEST) \
	dump_elf_task_fp(*(DEST), TASK)


#define ELF_HWCAP  (~amask(-1))


#define ELF_PLATFORM				\
({						\
	enum implver_enum i_ = implver();	\
	( i_ == IMPLVER_EV4 ? "ev4"		\
	: i_ == IMPLVER_EV5			\
	  ? (amask(AMASK_BWX) ? "ev5" : "ev56")	\
	: amask (AMASK_CIX) ? "ev6" : "ev67");	\
})

#define SET_PERSONALITY(EX)					\
	set_personality(((EX).e_flags & EF_ALPHA_32BIT)		\
	   ? PER_LINUX_32BIT : PER_LINUX)

extern int alpha_l1i_cacheshape;
extern int alpha_l1d_cacheshape;
extern int alpha_l2_cacheshape;
extern int alpha_l3_cacheshape;

/* update AT_VECTOR_SIZE_ARCH if the number of NEW_AUX_ENT entries changes */
#define ARCH_DLINFO						\
  do {								\
    NEW_AUX_ENT(AT_L1I_CACHESHAPE, alpha_l1i_cacheshape);	\
    NEW_AUX_ENT(AT_L1D_CACHESHAPE, alpha_l1d_cacheshape);	\
    NEW_AUX_ENT(AT_L2_CACHESHAPE, alpha_l2_cacheshape);		\
    NEW_AUX_ENT(AT_L3_CACHESHAPE, alpha_l3_cacheshape);		\
  } while (0)

#endif /* __ASM_ALPHA_ELF_H */
