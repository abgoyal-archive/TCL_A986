
#ifndef _LIBEBL_MIPS_H
#define _LIBEBL_MIPS_H 1

#include <libeblP.h>


/* Constructor.  */
extern int mips_init (Elf *elf, GElf_Half machine, Ebl *eh, size_t ehlen);

/* Destructor.  */
extern void mips_destr (Ebl *bh);


/* Function to get relocation type name.  */
extern const char *mips_reloc_type_name (int type, char *buf, size_t len);

/* Function to get segment type name.  */
extern const char *mips_segment_type_name (int type, char *buf, size_t len);

/* Function to get setion type name.  */
extern const char *mips_section_type_name (int type, char *buf, size_t len);

/* Function to get machine flag name.  */
extern const char *mips_machine_flag_name (Elf64_Word *flags);

/* Function to get dynamic tag name.  */
extern const char *mips_dynamic_tag_name (int64_t tag, char *buf, size_t len);

#endif	/* libebl_mips.h */
