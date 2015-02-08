


#ifndef ELFF_ELF_DEFS_H_
#define ELFF_ELF_DEFS_H_

#include "elf.h"

//=============================================================================
// Macros.
//=============================================================================

#define INC_PTR(p, n)   (reinterpret_cast<uint8_t*>(p) + (n))

#define INC_CPTR(p, n)  (reinterpret_cast<const uint8_t*>(p) + (n))

#define INC_PTR_T(T, p, n)                              \
    reinterpret_cast<T*>                                \
        (reinterpret_cast<uint8_t*>(p) + (n))

#define INC_CPTR_T(T, p, n)                                 \
    reinterpret_cast<const T*>                              \
        (reinterpret_cast<const uint8_t*>(p) + (n))

#define ELFF_ARRAY_SIZE(a) (sizeof(a) / sizeof(*(a)))

#define ELFF_FIELD_OFFSET(T, f) ((size_t)(size_t*)&(((T *)0)->f))

//=============================================================================
// Inline routines.
//=============================================================================

static inline size_t
diff_ptr(const void* s, const void* e) {
  assert(s <= e);
  return ((size_t)(reinterpret_cast<const uint8_t*>(e) -
         reinterpret_cast<const uint8_t*>(s)));
}

static inline uint8_t
get_byte(const void* ptr, uint32_t bt) {
  return *(reinterpret_cast<const uint8_t*>(ptr) + bt);
}

static inline bool
is_in_section(const void* rp, size_t rsize, const void* ss, size_t ssize) {
  const void* rend = INC_CPTR(rp, rsize);
  /* We also make sure here that increment didn't overflow the pointer. */
  return rp >= ss && ss != NULL && (diff_ptr(ss, rend) <= ssize) && rend >= rp;
}

static inline bool
is_little_endian_cpu(void) {
  uint16_t tmp = 0x00FF;
  /* Lets see if byte has flipped for little-endian. */
  return get_byte(&tmp, 0) == 0xFF;
}

#endif  // ELFF_ELF_DEFS_H_
