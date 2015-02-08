


#ifndef BISON_LIBIBERTY_H_
# define BISON_LIBIBERTY_H_ 1

# ifndef __attribute__
#  if __GNUC__ < 2 || (__GNUC__ == 2 && __GNUC_MINOR__ < 8) || __STRICT_ANSI__
#   define __attribute__(x)
#  endif
# endif

# define ATTRIBUTE_UNUSED __attribute__ ((__unused__))

# include "xalloc.h"

#endif /* ! BISON_LIBIBERTY_H_ */
