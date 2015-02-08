
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <string.h>

#include <ld.h>

/* Definitions for the symbol hash table.  */
#define TYPE struct id_list *
#define NAME ld_version_str_tab
#define COMPARE(a, b) strcmp ((a)->id, (b)->id)

#include "../lib/dynamicsizehash.c"
