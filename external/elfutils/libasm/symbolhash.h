
#ifndef SYMBOLHASH_H
#define SYMBOLHASH_H	1

/* Definitions for the symbol hash table.  */
#define TYPE AsmSym_t *
#define NAME asm_symbol_tab
#define ITERATE 1
#define COMPARE(a, b) \
  strcmp (ebl_string ((a)->strent), ebl_string ((b)->strent))
#include <dynamicsizehash.h>

#endif	/* symbolhash.h */
