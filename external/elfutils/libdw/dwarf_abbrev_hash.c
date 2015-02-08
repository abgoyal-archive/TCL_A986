
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#define NO_UNDEF
#include "libdwP.h"

#define next_prime __libdwarf_next_prime
extern size_t next_prime (size_t) attribute_hidden;

#include <dynamicsizehash.c>

#undef next_prime
#define next_prime attribute_hidden __libdwarf_next_prime
#include "../lib/next_prime.c"
