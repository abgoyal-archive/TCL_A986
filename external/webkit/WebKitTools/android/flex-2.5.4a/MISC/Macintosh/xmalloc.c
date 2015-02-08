

#if STDC_HEADERS || THINK_C
#include <stdlib.h>
#else
char *malloc ();
char *realloc ();
void free ();
#endif

#ifdef THINK_C
#define error(x, y, z) perror(z) /* Throw out meaningless arguments */
#else
void error ();
#endif

/* Allocate N bytes of memory dynamically, with error checking.  */

char *
xmalloc (n)
     unsigned n;
{
  char *p;

  p = malloc (n);
  if (p == 0)
    /* Must exit with 2 for `cmp'.  */
    error (2, 0, "virtual memory exhausted");
  return p;
}


char *
xrealloc (p, n)
     char *p;
     unsigned n;
{
  if (p == 0)
    return xmalloc (n);
  if (n == 0)
    {
      free (p);
      return 0;
    }
  p = realloc (p, n);
  if (p == 0)
    /* Must exit with 2 for `cmp'.  */
    error (2, 0, "virtual memory exhausted");
  return p;
}
