
#ifndef _LIBEBL_SPARC_H
#define _LIBEBL_SPARC_H 1

#include <libeblP.h>


/* Constructor.  */
extern int sparc_init (Elf *elf, GElf_Half machine, Ebl *eh, size_t ehlen);

/* Destructor.  */
extern void sparc_destr (Ebl *bh);


/* Function to get relocation type name.  */
extern const char *sparc_reloc_type_name (int type, char *buf, size_t len);

/* Check relocation type.  */
extern bool sparc_reloc_type_check (int type);

/* Code note handling.  */
extern bool sparc_core_note (const char *name, uint32_t type, uint32_t descsz,
			     const char *desc);

#endif	/* libebl_sparc.h */
