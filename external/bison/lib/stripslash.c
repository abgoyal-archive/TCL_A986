
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "dirname.h"


bool
strip_trailing_slashes (char *file)
{
  char *base = base_name (file);
  char *base_lim = base + base_len (base);
  bool had_slash = (*base_lim != '\0');
  *base_lim = '\0';
  return had_slash;
}
