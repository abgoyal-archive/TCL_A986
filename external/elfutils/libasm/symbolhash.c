
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <string.h>

#include <libasmP.h>
#include <libebl.h>

/* Definitions for the symbol hash table.  */
#define TYPE AsmSym_t *
#define NAME asm_symbol_tab
#define ITERATE 1
#define REVERSE 1
#define COMPARE(a, b) \
  strcmp (ebl_string ((a)->strent), ebl_string ((b)->strent))

#define next_prime __libasm_next_prime
extern size_t next_prime (size_t) attribute_hidden;

#include "../lib/dynamicsizehash.c"

#undef next_prime
#define next_prime attribute_hidden __libasm_next_prime
#include "../lib/next_prime.c"
