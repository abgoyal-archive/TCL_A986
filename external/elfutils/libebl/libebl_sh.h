
#ifndef _LIBEBL_SH_H
#define _LIBEBL_SH_H 1

#include <libeblP.h>


/* Constructor.  */
extern int sh_init (Elf *elf, GElf_Half machine, Ebl *eh, size_t ehlen);

/* Destructor.  */
extern void sh_destr (Ebl *bh);


/* Function to get relocation type name.  */
extern const char *sh_reloc_type_name (int type, char *buf, size_t len);

#endif	/* libebl_sh.h */
