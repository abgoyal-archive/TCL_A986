
#include <config.h>
#include "system.h"

#include "assoc.h"


const char *
assoc_to_string (assoc a)
{
  switch (a)
    {
    default:
      abort ();

    case undef_assoc:
      return "undefined associativity";

    case right_assoc:
      return "%right";

    case left_assoc:
      return "%left";

    case non_assoc:
      return "%nonassoc";
    }
}
