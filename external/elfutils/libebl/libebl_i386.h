
#ifndef _LIBEBL_I386_H
#define _LIBEBL_I386_H 1

#include <libeblP.h>


/* Constructor.  */
extern int i386_init (Elf *elf, GElf_Half machine, Ebl *eh, size_t ehlen);

/* Destructor.  */
extern void i386_destr (Ebl *bh);


/* Function to get relocation type name.  */
extern const char *i386_reloc_type_name (int type, char *buf, size_t len);

/* Check relocation type.  */
extern bool i386_reloc_type_check (int type);

/* Check relocation type use.  */
extern bool i386_reloc_valid_use (Elf *elf, int type);

/* Check relocation type use.  */
extern bool i386_gotpc_reloc_check (Elf *elf, int type);

/* Code note handling.  */
extern bool i386_core_note (const char *name, uint32_t type, uint32_t descsz,
			    const char *desc);

#endif	/* libebl_i386.h */
