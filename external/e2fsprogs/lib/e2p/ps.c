

#include <stdio.h>

#include "e2p.h"

void print_fs_state (FILE * f, unsigned short state)
{
	if (state & EXT2_VALID_FS)
		fprintf (f, " clean");
	else
		fprintf (f, " not clean");
	if (state & EXT2_ERROR_FS)
		fprintf (f, " with errors");
}
