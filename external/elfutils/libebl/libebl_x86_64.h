
#ifndef _LIBEBL_X86_64_H
#define _LIBEBL_X86_64_H 1

#include <libeblP.h>


/* Constructor.  */
extern int x86_64_init (Elf *elf, GElf_Half machine, Ebl *eh, size_t ehlen);

/* Destructor.  */
extern void x86_64_destr (Ebl *bh);


/* Function to get relocation type name.  */
extern const char *x86_64_reloc_type_name (int type, char *buf, size_t len);

/* Check relocation type.  */
extern bool x86_64_reloc_type_check (int type);

/* Check relocation type use.  */
extern bool x86_64_reloc_valid_use (Elf *elf, int type);

/* Code note handling.  */
extern bool x86_64_core_note (const char *name, uint32_t type, uint32_t descsz,
			      const char *desc);

#endif	/* libebl_x86_64.h */
