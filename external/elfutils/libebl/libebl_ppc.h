
#ifndef _LIBEBL_PPC_H
#define _LIBEBL_PPC_H 1

#include <libeblP.h>


/* Constructor.  */
extern int ppc_init (Elf *elf, GElf_Half machine, Ebl *eh, size_t ehlen);

/* Destructor.  */
extern void ppc_destr (Ebl *bh);


/* Function to get relocation type name.  */
extern const char *ppc_reloc_type_name (int type, char *buf, size_t len);

/* Check relocation type.  */
extern bool ppc_reloc_type_check (int type);

/* Check relocation type use.  */
extern bool ppc_reloc_valid_use (Elf *elf, int type);

/* Code note handling.  */
extern bool ppc_core_note (const char *name, uint32_t type, uint32_t descsz,
			      const char *desc);

#endif	/* libebl_ppc.h */
