
/* flexdef - definitions file for flex */


/* @(#) $Header: /home/daffy/u0/vern/flex/RCS/flexdef.h,v 2.53 95/04/20 11:17:36 vern Exp $ (LBL) */

#include <stdio.h>
#include <ctype.h>

#include "config.h"

#ifdef __TURBOC__
#define HAVE_STRING_H 1
#define MS_DOS 1
#ifndef __STDC__
#define __STDC__ 1
#endif
 #pragma warn -pro
 #pragma warn -rch
 #pragma warn -use
 #pragma warn -aus
 #pragma warn -par
 #pragma warn -pia
#endif

#ifdef HAVE_STRING_H
#include <string.h>
#else
#include <strings.h>
#endif

#ifdef HAVE_SYS_TYPES_H
#include <sys/types.h>
#endif

#ifdef HAVE_MALLOC_H
#include <malloc.h>
#endif

#ifdef STDC_HEADERS
#include <stdlib.h>
#endif

#define _(String) (String)

/* Always be prepared to generate an 8-bit scanner. */
#define CSIZE 256
#define Char unsigned char

/* Size of input alphabet - should be size of ASCII set. */
#ifndef DEFAULT_CSIZE
#define DEFAULT_CSIZE 128
#endif

#ifndef PROTO
#if __STDC__
#define PROTO(proto) proto
#else
#define PROTO(proto) ()
#endif
#endif

#ifdef VMS
#ifndef __VMS_POSIX
#define unlink remove
#define SHORT_FILE_NAMES
#endif
#endif

#ifdef MS_DOS
#define SHORT_FILE_NAMES
#endif


/* Maximum line length we'll have to deal with. */
#define MAXLINE 2048

#ifndef MIN
#define MIN(x,y) ((x) < (y) ? (x) : (y))
#endif
#ifndef MAX
#define MAX(x,y) ((x) > (y) ? (x) : (y))
#endif
#ifndef ABS
#define ABS(x) ((x) < 0 ? -(x) : (x))
#endif


/* ANSI C does not guarantee that isascii() is defined */
#ifndef isascii
#define isascii(c) ((c) <= 0177)
#endif


#define true 1
#define false 0
#define unspecified -1


#define EOB_POSITION -1
#define ACTION_POSITION -2

/* Number of data items per line for -f output. */
#define NUMDATAITEMS 10

#define NUMDATALINES 10

/* transition_struct_out() definitions. */
#define TRANS_STRUCT_PRINT_LENGTH 14

#define FREE_EPSILON(state) \
	(transchar[state] == SYM_EPSILON && \
	 trans2[state] == NO_TRANSITION && \
	 finalst[state] != state)

#define SUPER_FREE_EPSILON(state) \
	(transchar[state] == SYM_EPSILON && \
	 trans1[state] == NO_TRANSITION) \

#define INITIAL_MAX_DFA_SIZE 750
#define MAX_DFA_SIZE_INCREMENT 750



/* Mask to mark a trailing context accepting number. */
#define YY_TRAILING_MASK 0x2000

#define YY_TRAILING_HEAD_MASK 0x4000

/* Maximum number of rules, as outlined in the above note. */
#define MAX_RULE (YY_TRAILING_MASK - 1)


#define NIL 0

#define JAM -1	/* to mark a missing DFA transition */
#define NO_TRANSITION NIL
#define UNIQUE -1	/* marks a symbol as an e.c. representative */
#define INFINITY -1	/* for x{5,} constructions */

#define INITIAL_MAX_CCLS 100	/* max number of unique character classes */
#define MAX_CCLS_INCREMENT 100

/* Size of table holding members of character classes. */
#define INITIAL_MAX_CCL_TBL_SIZE 500
#define MAX_CCL_TBL_SIZE_INCREMENT 250

#define INITIAL_MAX_RULES 100	/* default maximum number of rules */
#define MAX_RULES_INCREMENT 100

#define INITIAL_MNS 2000	/* default maximum number of nfa states */
#define MNS_INCREMENT 1000	/* amount to bump above by if it's not enough */

#define INITIAL_MAX_DFAS 1000	/* default maximum number of dfa states */
#define MAX_DFAS_INCREMENT 1000

#define JAMSTATE -32766	/* marks a reference to the state that always jams */

/* Maximum number of NFA states. */
#define MAXIMUM_MNS 31999

#define MARKER_DIFFERENCE (MAXIMUM_MNS+2)

/* Maximum number of nxt/chk pairs for non-templates. */
#define INITIAL_MAX_XPAIRS 2000
#define MAX_XPAIRS_INCREMENT 2000

/* Maximum number of nxt/chk pairs needed for templates. */
#define INITIAL_MAX_TEMPLATE_XPAIRS 2500
#define MAX_TEMPLATE_XPAIRS_INCREMENT 2500

#define SYM_EPSILON (CSIZE + 1)	/* to mark transitions on the symbol epsilon */

#define INITIAL_MAX_SCS 40	/* maximum number of start conditions */
#define MAX_SCS_INCREMENT 40	/* amount to bump by if it's not enough */

#define ONE_STACK_SIZE 500	/* stack of states with only one out-transition */
#define SAME_TRANS -1	/* transition is the same as "default" entry for state */

#define PROTO_SIZE_PERCENTAGE 15

#define CHECK_COM_PERCENTAGE 50

#define FIRST_MATCH_DIFF_PERCENTAGE 10

#define ACCEPTABLE_DIFF_PERCENTAGE 50

#define TEMPLATE_SAME_PERCENTAGE 60

#define NEW_PROTO_DIFF_PERCENTAGE 20

#define INTERIOR_FIT_PERCENTAGE 15

#define PROT_SAVE_SIZE 2000

#define MSP 50	/* maximum number of saved protos (protos on the proto queue) */

#define MAX_XTIONS_FULL_INTERIOR_FIT 4

#define MAX_ASSOC_RULES 100

#define BAD_SUBSCRIPT -32767

#define MAX_SHORT 32700


/* Declarations for global variables. */


struct hash_entry
	{
	struct hash_entry *prev, *next;
	char *name;
	char *str_val;
	int int_val;
	} ;

typedef struct hash_entry **hash_table;

#define NAME_TABLE_HASH_SIZE 101
#define START_COND_HASH_SIZE 101
#define CCL_HASH_SIZE 101

extern struct hash_entry *ndtbl[NAME_TABLE_HASH_SIZE]; 
extern struct hash_entry *sctbl[START_COND_HASH_SIZE];
extern struct hash_entry *ccltab[CCL_HASH_SIZE];



extern int printstats, syntaxerror, eofseen, ddebug, trace, nowarn, spprdflt;
extern int interactive, caseins, lex_compat, do_yylineno;
extern int useecs, fulltbl, usemecs, fullspd;
extern int gen_line_dirs, performance_report, backing_up_report;
extern int C_plus_plus, long_align, use_read, yytext_is_array, do_yywrap;
extern int csize;
extern int yymore_used, reject, real_reject, continued_action, in_rule;

extern int yymore_really_used, reject_really_used;



extern int datapos, dataline, linenum, out_linenum;
extern FILE *skelfile, *yyin, *backing_up_file;
extern const char *skel[];
extern int skel_ind;
extern char *infilename, *outfilename;
extern int did_outfilename;
extern char *prefix, *yyclass;
extern int do_stdinit, use_stdout;
extern char **input_files;
extern int num_input_files;
extern char *program_name;

extern char *action_array;
extern int action_size;
extern int defs1_offset, prolog_offset, action_offset, action_index;



extern int onestate[ONE_STACK_SIZE], onesym[ONE_STACK_SIZE];
extern int onenext[ONE_STACK_SIZE], onedef[ONE_STACK_SIZE], onesp;



extern int current_mns, current_max_rules;
extern int num_rules, num_eof_rules, default_rule, lastnfa;
extern int *firstst, *lastst, *finalst, *transchar, *trans1, *trans2;
extern int *accptnum, *assoc_rule, *state_type;
extern int *rule_type, *rule_linenum, *rule_useful;

#define STATE_NORMAL 0x1
#define STATE_TRAILING_CONTEXT 0x2

/* Global holding current type of state we're making. */

extern int current_state_type;

/* Different types of rules. */
#define RULE_NORMAL 0
#define RULE_VARIABLE 1

extern int variable_trailing_context_rules;



extern int numtemps, numprots, protprev[MSP], protnext[MSP], prottbl[MSP];
extern int protcomst[MSP], firstprot, lastprot, protsave[PROT_SAVE_SIZE];



extern int numecs, nextecm[CSIZE + 1], ecgroup[CSIZE + 1], nummecs;

extern int tecfwd[CSIZE + 1], tecbck[CSIZE + 1];



extern int lastsc, *scset, *scbol, *scxclu, *sceof;
extern int current_max_scs;
extern char **scname;



extern int current_max_dfa_size, current_max_xpairs;
extern int current_max_template_xpairs, current_max_dfas;
extern int lastdfa, *nxt, *chk, *tnxt;
extern int *base, *def, *nultrans, NUL_ec, tblend, firstfree, **dss, *dfasiz;
extern union dfaacc_union
	{
	int *dfaacc_set;
	int dfaacc_state;
	} *dfaacc;
extern int *accsiz, *dhash, numas;
extern int numsnpairs, jambase, jamstate;
extern int end_of_buffer_state;


extern int lastccl, *cclmap, *ccllen, *cclng, cclreuse;
extern int current_maxccls, current_max_ccl_tbl_size;
extern Char *ccltbl;



extern char nmstr[MAXLINE];
extern int sectnum, nummt, hshcol, dfaeql, numeps, eps2, num_reallocs;
extern int tmpuses, totnst, peakpairs, numuniq, numdup, hshsave;
extern int num_backing_up, bol_needed;

void *allocate_array PROTO((int, size_t));
void *reallocate_array PROTO((void*, int, size_t));

void *flex_alloc PROTO((size_t));
void *flex_realloc PROTO((void*, size_t));
void flex_free PROTO((void*));

#define allocate_integer_array(size) \
	(int *) allocate_array( size, sizeof( int ) )

#define reallocate_integer_array(array,size) \
	(int *) reallocate_array( (void *) array, size, sizeof( int ) )

#define allocate_int_ptr_array(size) \
	(int **) allocate_array( size, sizeof( int * ) )

#define allocate_char_ptr_array(size) \
	(char **) allocate_array( size, sizeof( char * ) )

#define allocate_dfaacc_union(size) \
	(union dfaacc_union *) \
		allocate_array( size, sizeof( union dfaacc_union ) )

#define reallocate_int_ptr_array(array,size) \
	(int **) reallocate_array( (void *) array, size, sizeof( int * ) )

#define reallocate_char_ptr_array(array,size) \
	(char **) reallocate_array( (void *) array, size, sizeof( char * ) )

#define reallocate_dfaacc_union(array, size) \
	(union dfaacc_union *) \
	reallocate_array( (void *) array, size, sizeof( union dfaacc_union ) )

#define allocate_character_array(size) \
	(char *) allocate_array( size, sizeof( char ) )

#define reallocate_character_array(array,size) \
	(char *) reallocate_array( (void *) array, size, sizeof( char ) )

#define allocate_Character_array(size) \
	(Char *) allocate_array( size, sizeof( Char ) )

#define reallocate_Character_array(array,size) \
	(Char *) reallocate_array( (void *) array, size, sizeof( Char ) )


extern int yylval;


/* External functions that are cross-referenced among the flex source files. */


/* from file ccl.c */

extern void ccladd PROTO((int, int));	/* add a single character to a ccl */
extern int cclinit PROTO((void));	/* make an empty ccl */
extern void cclnegate PROTO((int));	/* negate a ccl */

/* List the members of a set of characters in CCL form. */
extern void list_character_set PROTO((FILE*, int[]));


/* from file dfa.c */

/* Check a DFA state for backing up. */
extern void check_for_backing_up PROTO((int, int[]));

/* Check to see if NFA state set constitutes "dangerous" trailing context. */
extern void check_trailing_context PROTO((int*, int, int*, int));

/* Construct the epsilon closure of a set of ndfa states. */
extern int *epsclosure PROTO((int*, int*, int[], int*, int*));

/* Increase the maximum number of dfas. */
extern void increase_max_dfas PROTO((void));

extern void ntod PROTO((void));	/* convert a ndfa to a dfa */

/* Converts a set of ndfa states into a dfa state. */
extern int snstods PROTO((int[], int, int[], int, int, int*));


/* from file ecs.c */

/* Convert character classes to set of equivalence classes. */
extern void ccl2ecl PROTO((void));

/* Associate equivalence class numbers with class members. */
extern int cre8ecs PROTO((int[], int[], int));

/* Update equivalence classes based on character class transitions. */
extern void mkeccl PROTO((Char[], int, int[], int[], int, int));

/* Create equivalence class for single character. */
extern void mkechar PROTO((int, int[], int[]));


/* from file gen.c */

extern void do_indent PROTO((void));	/* indent to the current level */

/* Generate the code to keep backing-up information. */
extern void gen_backing_up PROTO((void));

/* Generate the code to perform the backing up. */
extern void gen_bu_action PROTO((void));

/* Generate full speed compressed transition table. */
extern void genctbl PROTO((void));

/* Generate the code to find the action number. */
extern void gen_find_action PROTO((void));

extern void genftbl PROTO((void));	/* generate full transition table */

/* Generate the code to find the next compressed-table state. */
extern void gen_next_compressed_state PROTO((char*));

/* Generate the code to find the next match. */
extern void gen_next_match PROTO((void));

/* Generate the code to find the next state. */
extern void gen_next_state PROTO((int));

/* Generate the code to make a NUL transition. */
extern void gen_NUL_trans PROTO((void));

/* Generate the code to find the start state. */
extern void gen_start_state PROTO((void));

/* Generate data statements for the transition tables. */
extern void gentabs PROTO((void));

/* Write out a formatted string at the current indentation level. */
extern void indent_put2s PROTO((char[], char[]));

/* Write out a string + newline at the current indentation level. */
extern void indent_puts PROTO((char[]));

extern void make_tables PROTO((void));	/* generate transition tables */


/* from file main.c */

extern void check_options PROTO((void));
extern void flexend PROTO((int));
extern void usage PROTO((void));


/* from file misc.c */

/* Add a #define to the action file. */
extern void action_define PROTO(( char *defname, int value ));

/* Add the given text to the stored actions. */
extern void add_action PROTO(( char *new_text ));

/* True if a string is all lower case. */
extern int all_lower PROTO((register char *));

/* True if a string is all upper case. */
extern int all_upper PROTO((register char *));

/* Bubble sort an integer array. */
extern void bubble PROTO((int [], int));

/* Check a character to make sure it's in the expected range. */
extern void check_char PROTO((int c));

/* Replace upper-case letter to lower-case. */
extern Char clower PROTO((int));

/* Returns a dynamically allocated copy of a string. */
extern char *copy_string PROTO((register const char *));

/* Returns a dynamically allocated copy of a (potentially) unsigned string. */
extern Char *copy_unsigned_string PROTO((register Char *));

/* Shell sort a character array. */
extern void cshell PROTO((Char [], int, int));

/* Finish up a block of data declarations. */
extern void dataend PROTO((void));

/* Flush generated data statements. */
extern void dataflush PROTO((void));

/* Report an error message and terminate. */
extern void flexerror PROTO((const char[]));

/* Report a fatal error message and terminate. */
extern void flexfatal PROTO((const char[]));

/* Convert a hexadecimal digit string to an integer value. */
extern int htoi PROTO((Char[]));

/* Report an error message formatted with one integer argument. */
extern void lerrif PROTO((const char[], int));

/* Report an error message formatted with one string argument. */
extern void lerrsf PROTO((const char[], const char[]));

/* Spit out a "#line" statement. */
extern void line_directive_out PROTO((FILE*, int));

extern void mark_defs1 PROTO((void));

/* Mark the current position in the action array as the end of the prolog. */
extern void mark_prolog PROTO((void));

/* Generate a data statment for a two-dimensional array. */
extern void mk2data PROTO((int));

extern void mkdata PROTO((int));	/* generate a data statement */

/* Return the integer represented by a string of digits. */
extern int myctoi PROTO((char []));

/* Return character corresponding to escape sequence. */
extern Char myesc PROTO((Char[]));

/* Convert an octal digit string to an integer value. */
extern int otoi PROTO((Char [] ));

/* Output a (possibly-formatted) string to the generated scanner. */
extern void out PROTO((const char []));
extern void out_dec PROTO((const char [], int));
extern void out_dec2 PROTO((const char [], int, int));
extern void out_hex PROTO((const char [], unsigned int));
extern void out_line_count PROTO((const char []));
extern void out_str PROTO((const char [], const char []));
extern void out_str3
	PROTO((const char [], const char [], const char [], const char []));
extern void out_str_dec PROTO((const char [], const char [], int));
extern void outc PROTO((int));
extern void outn PROTO((const char []));

extern char *readable_form PROTO((int));

/* Write out one section of the skeleton file. */
extern void skelout PROTO((void));

/* Output a yy_trans_info structure. */
extern void transition_struct_out PROTO((int, int));

/* Only needed when using certain broken versions of bison to build parse.c. */
extern void *yy_flex_xmalloc PROTO(( int ));

/* Set a region of memory to 0. */
extern void zero_out PROTO((char *, size_t));


/* from file nfa.c */

/* Add an accepting state to a machine. */
extern void add_accept PROTO((int, int));

/* Make a given number of copies of a singleton machine. */
extern int copysingl PROTO((int, int));

/* Debugging routine to write out an nfa. */
extern void dumpnfa PROTO((int));

/* Finish up the processing for a rule. */
extern void finish_rule PROTO((int, int, int, int));

/* Connect two machines together. */
extern int link_machines PROTO((int, int));

extern void mark_beginning_as_normal PROTO((register int));

/* Make a machine that branches to two machines. */
extern int mkbranch PROTO((int, int));

extern int mkclos PROTO((int));	/* convert a machine into a closure */
extern int mkopt PROTO((int));	/* make a machine optional */

/* Make a machine that matches either one of two machines. */
extern int mkor PROTO((int, int));

/* Convert a machine into a positive closure. */
extern int mkposcl PROTO((int));

extern int mkrep PROTO((int, int, int));	/* make a replicated machine */

/* Create a state with a transition on a given symbol. */
extern int mkstate PROTO((int));

extern void new_rule PROTO((void));	/* initialize for a new rule */


/* from file parse.y */

/* Build the "<<EOF>>" action for the active start conditions. */
extern void build_eof_action PROTO((void));

/* Write out a message formatted with one string, pinpointing its location. */
extern void format_pinpoint_message PROTO((char[], char[]));

/* Write out a message, pinpointing its location. */
extern void pinpoint_message PROTO((char[]));

/* Write out a warning, pinpointing it at the given line. */
extern void line_warning PROTO(( char[], int ));

/* Write out a message, pinpointing it at the given line. */
extern void line_pinpoint PROTO(( char[], int ));

/* Report a formatted syntax error. */
extern void format_synerr PROTO((char [], char[]));
extern void synerr PROTO((char []));	/* report a syntax error */
extern void format_warn PROTO((char [], char[]));
extern void warn PROTO((char []));	/* report a warning */
extern void yyerror PROTO((char []));	/* report a parse error */
extern int yyparse PROTO((void));	/* the YACC parser */


/* from file scan.l */

/* The Flex-generated scanner for flex. */
extern int flexscan PROTO((void));

/* Open the given file (if NULL, stdin) for scanning. */
extern void set_input_file PROTO((char*));

/* Wrapup a file in the lexical analyzer. */
extern int yywrap PROTO((void));


/* from file sym.c */

/* Add symbol and definitions to symbol table. */
extern int addsym PROTO((register char[], char*, int, hash_table, int));

/* Save the text of a character class. */
extern void cclinstal PROTO ((Char [], int));

/* Lookup the number associated with character class. */
extern int ccllookup PROTO((Char []));

/* Find symbol in symbol table. */
extern struct hash_entry *findsym PROTO((register char[], hash_table, int ));

extern void ndinstal PROTO((char[], Char[]));	/* install a name definition */
extern Char *ndlookup PROTO((char[]));	/* lookup a name definition */

/* Increase maximum number of SC's. */
extern void scextend PROTO((void));
extern void scinstal PROTO((char[], int));	/* make a start condition */

/* Lookup the number associated with a start condition. */
extern int sclookup PROTO((char[]));


/* from file tblcmp.c */

/* Build table entries for dfa state. */
extern void bldtbl PROTO((int[], int, int, int, int));

extern void cmptmps PROTO((void));	/* compress template table entries */
extern void expand_nxt_chk PROTO((void));	/* increase nxt/chk arrays */
/* Finds a space in the table for a state to be placed. */
extern int find_table_space PROTO((int*, int));
extern void inittbl PROTO((void));	/* initialize transition tables */
/* Make the default, "jam" table entries. */
extern void mkdeftbl PROTO((void));

extern void mk1tbl PROTO((int, int, int, int));

/* Place a state into full speed transition table. */
extern void place_state PROTO((int*, int, int));

/* Save states with only one out-transition to be processed later. */
extern void stack1 PROTO((int, int, int, int));


/* from file yylex.c */

extern int yylex PROTO((void));
