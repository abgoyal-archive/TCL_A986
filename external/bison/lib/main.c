
#include <config.h>

#if HAVE_LOCALE_H
# include <locale.h>
#endif
#if ! HAVE_SETLOCALE
# define setlocale(Category, Locale)
#endif

int yyparse (void);

int
main (void)
{
  setlocale (LC_ALL, "");
  return yyparse ();
}
