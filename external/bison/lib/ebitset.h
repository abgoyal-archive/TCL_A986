
#ifndef _EBITSET_H
#define _EBITSET_H

#include "bitset.h"

extern size_t ebitset_bytes (bitset_bindex);

extern bitset ebitset_init (bitset, bitset_bindex);

extern void ebitset_release_memory (void);

#endif
