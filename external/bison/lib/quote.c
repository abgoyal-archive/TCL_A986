
/* Written by Paul Eggert <eggert@twinsun.com> */

#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "quotearg.h"
#include "quote.h"

char const *
quote_n (int n, char const *name)
{
  return quotearg_n_style (n, locale_quoting_style, name);
}

char const *
quote (char const *name)
{
  return quote_n (0, name);
}
