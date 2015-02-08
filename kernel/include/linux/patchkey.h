

#ifndef _LINUX_PATCHKEY_H_INDIRECT
#error "patchkey.h included directly"
#endif

#ifndef _LINUX_PATCHKEY_H
#define _LINUX_PATCHKEY_H

/* Endian macros. */
#ifdef __KERNEL__
#  include <asm/byteorder.h>
#else
#  include <endian.h>
#endif

#if defined(__KERNEL__)
#  if defined(__BIG_ENDIAN)
#    define _PATCHKEY(id) (0xfd00|id)
#  elif defined(__LITTLE_ENDIAN)
#    define _PATCHKEY(id) ((id<<8)|0x00fd)
#  else
#    error "could not determine byte order"
#  endif
#elif defined(__BYTE_ORDER)
#  if __BYTE_ORDER == __BIG_ENDIAN
#    define _PATCHKEY(id) (0xfd00|id)
#  elif __BYTE_ORDER == __LITTLE_ENDIAN
#    define _PATCHKEY(id) ((id<<8)|0x00fd)
#  else
#    error "could not determine byte order"
#  endif
#endif

#endif /* _LINUX_PATCHKEY_H */
