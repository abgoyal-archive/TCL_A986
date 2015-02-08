
#ifndef _LIBEBL_ARM_H
#define _LIBEBL_ARM_H 1

#include <libeblP.h>


/* Constructor.  */
extern int arm_init (Elf *elf, GElf_Half machine, Ebl *eh, size_t ehlen);

/* Destructor.  */
extern void arm_destr (Ebl *bh);


/* Function to get relocation type name.  */
extern const char *arm_reloc_type_name (int type, char *buf, size_t len);

/* Check relocation type.  */
extern bool arm_reloc_type_check (int type);

#endif	/* libebl_arm.h */
