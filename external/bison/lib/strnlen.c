
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "strnlen.h"


size_t
strnlen (const char *string, size_t maxlen)
{
  const char *end = memchr (string, '\0', maxlen);
  return end ? (size_t) (end - string) : maxlen;
}
