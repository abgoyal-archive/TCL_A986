
#ifndef _EXTTYPES_H
#define	_EXTTYPES_H 1

/* Integral types.  */
typedef char Elf32_Ext_Addr[ELF32_FSZ_ADDR];
typedef char Elf32_Ext_Off[ELF32_FSZ_OFF];
typedef char Elf32_Ext_Half[ELF32_FSZ_HALF];
typedef char Elf32_Ext_Sword[ELF32_FSZ_SWORD];
typedef char Elf32_Ext_Word[ELF32_FSZ_WORD];
typedef char Elf32_Ext_Sxword[ELF32_FSZ_SXWORD];
typedef char Elf32_Ext_Xword[ELF32_FSZ_XWORD];

typedef char Elf64_Ext_Addr[ELF64_FSZ_ADDR];
typedef char Elf64_Ext_Off[ELF64_FSZ_OFF];
typedef char Elf64_Ext_Half[ELF64_FSZ_HALF];
typedef char Elf64_Ext_Sword[ELF64_FSZ_SWORD];
typedef char Elf64_Ext_Word[ELF64_FSZ_WORD];
typedef char Elf64_Ext_Sxword[ELF64_FSZ_SXWORD];
typedef char Elf64_Ext_Xword[ELF64_FSZ_XWORD];


/* Define the composed types.  */
#define START(Bits, Name, EName) typedef struct {
#define END(Bits, Name) } ElfW2(Bits, Name)
#define TYPE_NAME(Type, Name) Type Name;
#define TYPE_EXTRA(Text) Text
#define TYPE_XLATE(Text)

/* Get the abstract definitions. */
#include "abstract.h"

/* And define the types.  */
Ehdr32 (Ext_);
Phdr32 (Ext_);
Shdr32 (Ext_);
Sym32 (Ext_);
Rel32 (Ext_);
Rela32 (Ext_);
Note32 (Ext_);
Dyn32 (Ext_);
Verdef32 (Ext_);
Verdaux32 (Ext_);
Verneed32 (Ext_);
Vernaux32 (Ext_);
Syminfo32 (Ext_);
Move32 (Ext_);

Ehdr64 (Ext_);
Phdr64 (Ext_);
Shdr64 (Ext_);
Sym64 (Ext_);
Rel64 (Ext_);
Rela64 (Ext_);
Note64 (Ext_);
Dyn64 (Ext_);
Verdef64 (Ext_);
Verdaux64 (Ext_);
Verneed64 (Ext_);
Vernaux64 (Ext_);
Syminfo64 (Ext_);
Move64 (Ext_);

#undef START
#undef END
#undef TYPE_NAME
#undef TYPE_EXTRA
#undef TYPE_XLATE

#endif	/* exttypes.h */
