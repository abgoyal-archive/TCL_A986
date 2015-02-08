
#ifndef _LIBEBL_PPC64_H
#define _LIBEBL_PPC64_H 1

#include <libeblP.h>


/* Constructor.  */
extern int ppc64_init (Elf *elf, GElf_Half machine, Ebl *eh, size_t ehlen);

/* Destructor.  */
extern void ppc64_destr (Ebl *bh);


/* Function to get relocation type name.  */
extern const char *ppc64_reloc_type_name (int type, char *buf, size_t len);

/* Check relocation type.  */
extern bool ppc64_reloc_type_check (int type);

/* Check relocation type use.  */
extern bool ppc64_reloc_valid_use (Elf *elf, int type);

/* Code note handling.  */
extern bool ppc64_core_note (const char *name, uint32_t type, uint32_t descsz,
			      const char *desc);

/* Name of dynamic tag.  */
extern const char *ppc64_dynamic_tag_name (int64_t tag, char *buf, size_t len);

#endif	/* libebl_ppc.h */
