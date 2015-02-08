
/* sym - symbol table routines */


/* $Header: /home/daffy/u0/vern/flex/RCS/sym.c,v 2.19 95/03/04 16:11:04 vern Exp $ */

#include "flexdef.h"


/* declare functions that have forward references */

int hashfunct PROTO((register char[], int));


struct hash_entry *ndtbl[NAME_TABLE_HASH_SIZE];
struct hash_entry *sctbl[START_COND_HASH_SIZE];
struct hash_entry *ccltab[CCL_HASH_SIZE];

struct hash_entry *findsym();



int addsym( sym, str_def, int_def, table, table_size )
register char sym[];
char *str_def;
int int_def;
hash_table table;
int table_size;
	{
	int hash_val = hashfunct( sym, table_size );
	register struct hash_entry *sym_entry = table[hash_val];
	register struct hash_entry *new_entry;
	register struct hash_entry *successor;

	while ( sym_entry )
		{
		if ( ! strcmp( sym, sym_entry->name ) )
			{ /* entry already exists */
			return -1;
			}

		sym_entry = sym_entry->next;
		}

	/* create new entry */
	new_entry = (struct hash_entry *)
		flex_alloc( sizeof( struct hash_entry ) );

	if ( new_entry == NULL )
		flexfatal( _( "symbol table memory allocation failed" ) );

	if ( (successor = table[hash_val]) != 0 )
		{
		new_entry->next = successor;
		successor->prev = new_entry;
		}
	else
		new_entry->next = NULL;

	new_entry->prev = NULL;
	new_entry->name = sym;
	new_entry->str_val = str_def;
	new_entry->int_val = int_def;

	table[hash_val] = new_entry;

	return 0;
	}


/* cclinstal - save the text of a character class */

void cclinstal( ccltxt, cclnum )
Char ccltxt[];
int cclnum;
	{
	/* We don't bother checking the return status because we are not
	 * called unless the symbol is new.
	 */
	Char *copy_unsigned_string();

	(void) addsym( (char *) copy_unsigned_string( ccltxt ),
			(char *) 0, cclnum,
			ccltab, CCL_HASH_SIZE );
	}



int ccllookup( ccltxt )
Char ccltxt[];
	{
	return findsym( (char *) ccltxt, ccltab, CCL_HASH_SIZE )->int_val;
	}


/* findsym - find symbol in symbol table */

struct hash_entry *findsym( sym, table, table_size )
register char sym[];
hash_table table;
int table_size;
	{
	static struct hash_entry empty_entry =
		{
		(struct hash_entry *) 0, (struct hash_entry *) 0,
		(char *) 0, (char *) 0, 0,
		} ;
	register struct hash_entry *sym_entry =
		table[hashfunct( sym, table_size )];

	while ( sym_entry )
		{
		if ( ! strcmp( sym, sym_entry->name ) )
			return sym_entry;
		sym_entry = sym_entry->next;
		}

	return &empty_entry;
	}


/* hashfunct - compute the hash value for "str" and hash size "hash_size" */

int hashfunct( str, hash_size )
register char str[];
int hash_size;
	{
	register int hashval;
	register int locstr;

	hashval = 0;
	locstr = 0;

	while ( str[locstr] )
		{
		hashval = (hashval << 1) + (unsigned char) str[locstr++];
		hashval %= hash_size;
		}

	return hashval;
	}


/* ndinstal - install a name definition */

void ndinstal( name, definition )
char name[];
Char definition[];
	{
	char *copy_string();
	Char *copy_unsigned_string();

	if ( addsym( copy_string( name ),
			(char *) copy_unsigned_string( definition ), 0,
			ndtbl, NAME_TABLE_HASH_SIZE ) )
		synerr( _( "name defined twice" ) );
	}



Char *ndlookup( nd )
char nd[];
	{
	return (Char *) findsym( nd, ndtbl, NAME_TABLE_HASH_SIZE )->str_val;
	}


/* scextend - increase the maximum number of start conditions */

void scextend()
	{
	current_max_scs += MAX_SCS_INCREMENT;

	++num_reallocs;

	scset = reallocate_integer_array( scset, current_max_scs );
	scbol = reallocate_integer_array( scbol, current_max_scs );
	scxclu = reallocate_integer_array( scxclu, current_max_scs );
	sceof = reallocate_integer_array( sceof, current_max_scs );
	scname = reallocate_char_ptr_array( scname, current_max_scs );
	}



void scinstal( str, xcluflg )
char str[];
int xcluflg;
	{
	char *copy_string();

	/* Generate start condition definition, for use in BEGIN et al. */
	action_define( str, lastsc );

	if ( ++lastsc >= current_max_scs )
		scextend();

	scname[lastsc] = copy_string( str );

	if ( addsym( scname[lastsc], (char *) 0, lastsc,
			sctbl, START_COND_HASH_SIZE ) )
		format_pinpoint_message(
				_( "start condition %s declared twice" ),
					str );

	scset[lastsc] = mkstate( SYM_EPSILON );
	scbol[lastsc] = mkstate( SYM_EPSILON );
	scxclu[lastsc] = xcluflg;
	sceof[lastsc] = false;
	}



int sclookup( str )
char str[];
	{
	return findsym( str, sctbl, START_COND_HASH_SIZE )->int_val;
	}
