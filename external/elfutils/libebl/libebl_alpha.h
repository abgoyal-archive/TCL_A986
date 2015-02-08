
#ifndef _LIBEBL_ALPHA_H
#define _LIBEBL_ALPHA_H 1

#include <libeblP.h>


/* Constructor.  */
extern int alpha_init (Elf *elf, GElf_Half machine, Ebl *eh, size_t ehlen);

/* Destructor.  */
extern void alpha_destr (Ebl *bh);


/* Function to get relocation type name.  */
extern const char *alpha_reloc_type_name (int type, char *buf, size_t len);

/* Check relocation type.  */
extern bool alpha_reloc_type_check (int type);

#endif	/* libebl_alpha.h */
