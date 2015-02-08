
/* Written by Paul Eggert.  */

#include <unistd.h>
#include "unistd-safer.h"

#undef dup
#define dup dup_safer

#undef pipe
#define pipe pipe_safer
