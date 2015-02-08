

#include <stdio.h>

#include "e2p.h"

void print_fs_errors (FILE * f, unsigned short errors)
{
	switch (errors)
	{
		case EXT2_ERRORS_CONTINUE:
			fprintf (f, "Continue");
			break;
		case EXT2_ERRORS_RO:
			fprintf (f, "Remount read-only");
			break;
		case EXT2_ERRORS_PANIC:
			fprintf (f, "Panic");
			break;
		default:
			fprintf (f, "Unknown (continue)");
	}
}
