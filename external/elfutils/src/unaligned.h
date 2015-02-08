
#ifndef _UNALIGNED_H
#define _UNALIGNED_H	1

#include <byteswap.h>
#include <endian.h>


#ifndef UNALIGNED_ACCESS_CLASS
# error "UNALIGNED_ACCESS_CLASS must be defined"
#endif


/* Macros to convert from the host byte order to that of the object file.  */
#if UNALIGNED_ACCESS_CLASS == BYTE_ORDER
# define target_bswap_16(n) (n)
# define target_bswap_32(n) (n)
# define target_bswap_64(n) (n)
#else
# define target_bswap_16(n) bswap_16 (n)
# define target_bswap_32(n) bswap_32 (n)
# define target_bswap_64(n) bswap_64 (n)
#endif


union u_2ubyte_unaligned
{
  uint16_t u;
  char c[2];
} __attribute__((packed));

union u_4ubyte_unaligned
{
  uint32_t u;
  char c[4];
} __attribute__((packed));

union u_8ubyte_unaligned
{
  uint64_t u;
  char c[8];
} __attribute__((packed));


/* Macros to store value at unaligned address.  */
#define store_2ubyte_unaligned(ptr, value) \
  (void) (((union u_2ubyte_unaligned *) (ptr))->u = target_bswap_16 (value))
#define store_4ubyte_unaligned(ptr, value) \
  (void) (((union u_4ubyte_unaligned *) (ptr))->u = target_bswap_32 (value))
#define store_8ubyte_unaligned(ptr, value) \
  (void) (((union u_8ubyte_unaligned *) (ptr))->u = target_bswap_64 (value))


#if UNALIGNED_ACCESS_CLASS == BYTE_ORDER
# define add_2ubyte_unaligned(ptr, value) \
  (void) (((union u_2ubyte_unaligned *) (ptr))->u += value)
# define add_4ubyte_unaligned(ptr, value) \
  (void) (((union u_4ubyte_unaligned *) (ptr))->u += value)
# define add_8ubyte_unaligned(ptr, value) \
  (void) (((union u_8ubyte_unaligned *) (ptr))->u += value)
#else
# define add_2ubyte_unaligned(ptr, value) \
  do {									      \
    union u_2ubyte_unaligned *_ptr = (ptr);				      \
    uint16_t _val = bswap_16 (_ptr->u) + (value);			      \
    _ptr->u = bswap_16 (_val);						      \
  } while (0)
# define add_4ubyte_unaligned(ptr, value) \
  do {									      \
    union u_4ubyte_unaligned *_ptr = (ptr);				      \
    uint32_t _val = bswap_32 (_ptr->u) + (value);			      \
    _ptr->u = bswap_32 (_val);						      \
  } while (0)
# define add_8ubyte_unaligned(ptr, value) \
  do {									      \
    union u_8ubyte_unaligned *_ptr = (ptr);				      \
    uint64_t _val = bswap_64 (_ptr->u) + (value);			      \
    _ptr->u = bswap_64 (_val);						      \
  } while (0)
#endif

#endif /* unaligned.h */
