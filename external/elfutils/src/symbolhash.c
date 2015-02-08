
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <string.h>

#include <ld.h>

/* Definitions for the symbol hash table.  */
#define TYPE struct symbol *
#define NAME ld_symbol_tab
#define ITERATE 1
#define COMPARE(a, b) strcmp ((a)->name, (b)->name)

#include "../lib/dynamicsizehash.c"
