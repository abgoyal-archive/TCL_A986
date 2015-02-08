
/* ccl - routines for character classes */


/* $Header: /home/daffy/u0/vern/flex/RCS/ccl.c,v 2.9 93/09/16 20:32:14 vern Exp $ */

#include "flexdef.h"

/* ccladd - add a single character to a ccl */

void ccladd( cclp, ch )
int cclp;
int ch;
	{
	int ind, len, newpos, i;

	check_char( ch );

	len = ccllen[cclp];
	ind = cclmap[cclp];

	/* check to see if the character is already in the ccl */

	for ( i = 0; i < len; ++i )
		if ( ccltbl[ind + i] == ch )
			return;

	newpos = ind + len;

	if ( newpos >= current_max_ccl_tbl_size )
		{
		current_max_ccl_tbl_size += MAX_CCL_TBL_SIZE_INCREMENT;

		++num_reallocs;

		ccltbl = reallocate_Character_array( ccltbl,
						current_max_ccl_tbl_size );
		}

	ccllen[cclp] = len + 1;
	ccltbl[newpos] = ch;
	}


/* cclinit - return an empty ccl */

int cclinit()
	{
	if ( ++lastccl >= current_maxccls )
		{
		current_maxccls += MAX_CCLS_INCREMENT;

		++num_reallocs;

		cclmap = reallocate_integer_array( cclmap, current_maxccls );
		ccllen = reallocate_integer_array( ccllen, current_maxccls );
		cclng = reallocate_integer_array( cclng, current_maxccls );
		}

	if ( lastccl == 1 )
		/* we're making the first ccl */
		cclmap[lastccl] = 0;

	else
		/* The new pointer is just past the end of the last ccl.
		 * Since the cclmap points to the \first/ character of a
		 * ccl, adding the length of the ccl to the cclmap pointer
		 * will produce a cursor to the first free space.
		 */
		cclmap[lastccl] = cclmap[lastccl - 1] + ccllen[lastccl - 1];

	ccllen[lastccl] = 0;
	cclng[lastccl] = 0;	/* ccl's start out life un-negated */

	return lastccl;
	}


/* cclnegate - negate the given ccl */

void cclnegate( cclp )
int cclp;
	{
	cclng[cclp] = 1;
	}



void list_character_set( file, cset )
FILE *file;
int cset[];
	{
	register int i;

	putc( '[', file );

	for ( i = 0; i < csize; ++i )
		{
		if ( cset[i] )
			{
			register int start_char = i;

			putc( ' ', file );

			fputs( readable_form( i ), file );

			while ( ++i < csize && cset[i] )
				;

			if ( i - 1 > start_char )
				/* this was a run */
				fprintf( file, "-%s", readable_form( i - 1 ) );

			putc( ' ', file );
			}
		}

	putc( ']', file );
	}
