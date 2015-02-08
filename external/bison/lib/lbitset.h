
#ifndef _LBITSET_H
#define _LBITSET_H

#include "bitset.h"

extern size_t lbitset_bytes (bitset_bindex);

extern bitset lbitset_init (bitset, bitset_bindex);

extern void lbitset_release_memory (void);

#endif
