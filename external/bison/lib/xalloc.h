
#ifndef XALLOC_H_
# define XALLOC_H_

# include <stddef.h>


# ifdef __cplusplus
extern "C" {
# endif


# ifndef __attribute__
#  if __GNUC__ < 2 || (__GNUC__ == 2 && __GNUC_MINOR__ < 8) || __STRICT_ANSI__
#   define __attribute__(x)
#  endif
# endif

# ifndef ATTRIBUTE_NORETURN
#  define ATTRIBUTE_NORETURN __attribute__ ((__noreturn__))
# endif

extern void xalloc_die (void) ATTRIBUTE_NORETURN;

void *xmalloc (size_t s);
void *xnmalloc (size_t n, size_t s);
void *xzalloc (size_t s);
void *xcalloc (size_t n, size_t s);
void *xrealloc (void *p, size_t s);
void *xnrealloc (void *p, size_t n, size_t s);
void *x2realloc (void *p, size_t *pn);
void *x2nrealloc (void *p, size_t *pn, size_t s);
void *xmemdup (void const *p, size_t s);
char *xstrdup (char const *str);

# define xalloc_oversized(n, s) \
    ((size_t) (sizeof (ptrdiff_t) <= sizeof (size_t) ? -1 : -2) / (s) < (n))

# ifdef __cplusplus
}
# endif


#endif /* !XALLOC_H_ */
