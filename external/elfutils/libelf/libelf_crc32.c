
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#define crc32 attribute_hidden __libelf_crc32
#define LIB_SYSTEM_H	1
#include <libelf.h>
#include "../lib/crc32.c"
