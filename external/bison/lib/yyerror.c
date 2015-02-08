
#include <stdio.h>

int yyerror (char const *);

int
yyerror (char const *message)
{
  return fprintf (stderr, "%s\n", message);
}
