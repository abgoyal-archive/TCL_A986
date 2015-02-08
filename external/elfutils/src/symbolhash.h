
#ifndef SYMBOLHASH_H
#define SYMBOLHASH_H	1

/* Definitions for the symbol hash table.  */
#define TYPE struct symbol *
#define NAME ld_symbol_tab
#define ITERATE 1
#define COMPARE(a, b) strcmp ((a)->name, (b)->name)
#include <dynamicsizehash.h>

#endif	/* symbolhash.h */
