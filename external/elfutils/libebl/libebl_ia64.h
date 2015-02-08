
#ifndef _LIBEBL_IA64_H
#define _LIBEBL_IA64_H 1

#include <libeblP.h>


/* Constructor.  */
extern int ia64_init (Elf *elf, GElf_Half machine, Ebl *eh, size_t ehlen);

/* Destructor.  */
extern void ia64_destr (Ebl *bh);


/* Function to get relocation type name.  */
extern const char *ia64_reloc_type_name (int type, char *buf, size_t len);

/* Check relocation type.  */
extern bool ia64_reloc_type_check (int type);

/* Name of segment type.  */
extern const char *ia64_segment_type_name (int segment, char *buf, size_t len);

/* Name of dynamic tag.  */
extern const char *ia64_dynamic_tag_name (int64_t tag, char *buf, size_t len);

#endif	/* libebl_ia64.h */
