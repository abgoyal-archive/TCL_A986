
/* Written by Paul Eggert.  */

#include <stdio.h>
#include "stdio-safer.h"

#undef fopen
#define fopen fopen_safer
