

#ifndef H_POPT
#define H_POPT

#include <stdio.h>			/* for FILE * */

#define POPT_OPTION_DEPTH	10

/*@{*/
#define POPT_ARG_NONE		0	/*!< no arg */
#define POPT_ARG_STRING		1	/*!< arg will be saved as string */
#define POPT_ARG_INT		2	/*!< arg will be converted to int */
#define POPT_ARG_LONG		3	/*!< arg will be converted to long */
#define POPT_ARG_INCLUDE_TABLE	4	/*!< arg points to table */
#define POPT_ARG_CALLBACK	5	/*!< table-wide callback... must be
					   set first in table; arg points 
					   to callback, descrip points to 
					   callback data to pass */
#define POPT_ARG_INTL_DOMAIN    6       /*!< set the translation domain
					   for this table and any
					   included tables; arg points
					   to the domain string */
#define POPT_ARG_VAL		7	/*!< arg should take value val */
#define	POPT_ARG_FLOAT		8	/*!< arg will be converted to float */
#define	POPT_ARG_DOUBLE		9	/*!< arg will be converted to double */

#define POPT_ARG_MASK		0x0000FFFF
/*@}*/

/*@{*/
#define POPT_ARGFLAG_ONEDASH	0x80000000  /*!< allow -longoption */
#define POPT_ARGFLAG_DOC_HIDDEN 0x40000000  /*!< don't show in help/usage */
#define POPT_ARGFLAG_STRIP	0x20000000  /*!< strip this arg from argv(only applies to long args) */
#define	POPT_ARGFLAG_OPTIONAL	0x10000000  /*!< arg may be missing */

#define	POPT_ARGFLAG_OR		0x08000000  /*!< arg will be or'ed */
#define	POPT_ARGFLAG_NOR	0x09000000  /*!< arg will be nor'ed */
#define	POPT_ARGFLAG_AND	0x04000000  /*!< arg will be and'ed */
#define	POPT_ARGFLAG_NAND	0x05000000  /*!< arg will be nand'ed */
#define	POPT_ARGFLAG_XOR	0x02000000  /*!< arg will be xor'ed */
#define	POPT_ARGFLAG_NOT	0x01000000  /*!< arg will be negated */
#define POPT_ARGFLAG_LOGICALOPS \
        (POPT_ARGFLAG_OR|POPT_ARGFLAG_AND|POPT_ARGFLAG_XOR)

#define	POPT_BIT_SET	(POPT_ARG_VAL|POPT_ARGFLAG_OR)
					/*!< set arg bit(s) */
#define	POPT_BIT_CLR	(POPT_ARG_VAL|POPT_ARGFLAG_NAND)
					/*!< clear arg bit(s) */

#define	POPT_ARGFLAG_SHOW_DEFAULT 0x00800000 /*!< show default value in --help */

/*@}*/

/*@{*/
#define POPT_CBFLAG_PRE		0x80000000  /*!< call the callback before parse */
#define POPT_CBFLAG_POST	0x40000000  /*!< call the callback after parse */
#define POPT_CBFLAG_INC_DATA	0x20000000  /*!< use data from the include line,
					       not the subtable */
#define POPT_CBFLAG_SKIPOPTION	0x10000000  /*!< don't callback with option */
#define POPT_CBFLAG_CONTINUE	0x08000000  /*!< continue callbacks with option */
/*@}*/

/*@{*/
#define POPT_ERROR_NOARG	-10	/*!< missing argument */
#define POPT_ERROR_BADOPT	-11	/*!< unknown option */
#define POPT_ERROR_OPTSTOODEEP	-13	/*!< aliases nested too deeply */
#define POPT_ERROR_BADQUOTE	-15	/*!< error in paramter quoting */
#define POPT_ERROR_ERRNO	-16	/*!< errno set, use strerror(errno) */
#define POPT_ERROR_BADNUMBER	-17	/*!< invalid numeric value */
#define POPT_ERROR_OVERFLOW	-18	/*!< number too large or too small */
#define	POPT_ERROR_BADOPERATION	-19	/*!< mutually exclusive logical operations requested */
#define	POPT_ERROR_NULLARG	-20	/*!< opt->arg should not be NULL */
#define	POPT_ERROR_MALLOC	-21	/*!< memory allocation failed */
/*@}*/

/*@{*/
#define POPT_BADOPTION_NOALIAS  (1 << 0)  /*!< don't go into an alias */
/*@}*/

/*@{*/
#define POPT_CONTEXT_NO_EXEC	(1 << 0)  /*!< ignore exec expansions */
#define POPT_CONTEXT_KEEP_FIRST	(1 << 1)  /*!< pay attention to argv[0] */
#define POPT_CONTEXT_POSIXMEHARDER (1 << 2) /*!< options can't follow args */
#define POPT_CONTEXT_ARG_OPTS	(1 << 4) /*!< return args as options with value 0 */
/*@}*/

struct poptOption {
/*@observer@*/ /*@null@*/
    const char * longName;	/*!< may be NULL */
    char shortName;		/*!< may be NUL */
    int argInfo;
/*@shared@*/ /*@null@*/
    void * arg;			/*!< depends on argInfo */
    int val;			/*!< 0 means don't return, just update flag */
/*@observer@*/ /*@null@*/
    const char * descrip;	/*!< description for autohelp -- may be NULL */
/*@observer@*/ /*@null@*/
    const char * argDescrip;	/*!< argument description for autohelp */
};

struct poptAlias {
/*@owned@*/ /*@null@*/
    const char * longName;	/*!< may be NULL */
    char shortName;		/*!< may be NUL */
    int argc;
/*@owned@*/
    const char ** argv;		/*!< must be free()able */
};

/*@-exporttype@*/
typedef struct poptItem_s {
    struct poptOption option;	/*!< alias/exec name(s) and description. */
    int argc;			/*!< (alias) no. of args. */
/*@owned@*/
    const char ** argv;		/*!< (alias) args, must be free()able. */
} * poptItem;
/*@=exporttype@*/

/*@{*/

/*@-exportvar@*/
/*@unchecked@*/ /*@observer@*/
extern struct poptOption poptAliasOptions[];
/*@=exportvar@*/
#define POPT_AUTOALIAS { NULL, '\0', POPT_ARG_INCLUDE_TABLE, poptAliasOptions, \
			0, "Options implemented via popt alias/exec:", NULL },

/*@-exportvar@*/
/*@unchecked@*/ /*@observer@*/
extern struct poptOption poptHelpOptions[];
/*@=exportvar@*/

/*@-exportvar@*/
/*@unchecked@*/ /*@observer@*/
extern struct poptOption * poptHelpOptionsI18N;
/*@=exportvar@*/

#define POPT_AUTOHELP { NULL, '\0', POPT_ARG_INCLUDE_TABLE, poptHelpOptions, \
			0, "Help options:", NULL },

#define POPT_TABLEEND { NULL, '\0', 0, 0, 0, NULL, NULL }
/*@}*/

/*@-exporttype@*/
typedef /*@abstract@*/ struct poptContext_s * poptContext;
/*@=exporttype@*/

#ifndef __cplusplus
/*@-exporttype -typeuse@*/
typedef struct poptOption * poptOption;
/*@=exporttype =typeuse@*/
#endif

/*@-exportconst@*/
enum poptCallbackReason {
    POPT_CALLBACK_REASON_PRE	= 0, 
    POPT_CALLBACK_REASON_POST	= 1,
    POPT_CALLBACK_REASON_OPTION = 2
};
/*@=exportconst@*/

#ifdef __cplusplus
extern "C" {
#endif
/*@-type@*/

typedef void (*poptCallbackType) (poptContext con, 
		enum poptCallbackReason reason,
		/*@null@*/ const struct poptOption * opt,
		/*@null@*/ const char * arg,
		/*@null@*/ const void * data)
	/*@globals internalState @*/
	/*@modifies internalState @*/;

/*@only@*/ /*@null@*/
poptContext poptGetContext(
		/*@dependent@*/ /*@keep@*/ const char * name,
		int argc, /*@dependent@*/ /*@keep@*/ const char ** argv,
		/*@dependent@*/ /*@keep@*/ const struct poptOption * options,
		int flags)
	/*@*/;

/*@unused@*/
void poptResetContext(/*@null@*/poptContext con)
	/*@modifies con @*/;

int poptGetNextOpt(/*@null@*/poptContext con)
	/*@globals fileSystem, internalState @*/
	/*@modifies con, fileSystem, internalState @*/;

/*@observer@*/ /*@null@*/ /*@unused@*/
const char * poptGetOptArg(/*@null@*/poptContext con)
	/*@modifies con @*/;

/*@observer@*/ /*@null@*/ /*@unused@*/
const char * poptGetArg(/*@null@*/poptContext con)
	/*@modifies con @*/;

/*@observer@*/ /*@null@*/ /*@unused@*/
const char * poptPeekArg(/*@null@*/poptContext con)
	/*@*/;

/*@observer@*/ /*@null@*/
const char ** poptGetArgs(/*@null@*/poptContext con)
	/*@modifies con @*/;

/*@observer@*/
const char * poptBadOption(/*@null@*/poptContext con, int flags)
	/*@*/;

/*@null@*/
poptContext poptFreeContext( /*@only@*/ /*@null@*/ poptContext con)
	/*@modifies con @*/;

/*@unused@*/
int poptStuffArgs(poptContext con, /*@keep@*/ const char ** argv)
	/*@modifies con @*/;

/*@unused@*/
int poptAddAlias(poptContext con, struct poptAlias alias, int flags)
	/*@modifies con @*/;

int poptAddItem(poptContext con, poptItem newItem, int flags)
	/*@modifies con @*/;

int poptReadConfigFile(poptContext con, const char * fn)
	/*@globals errno, fileSystem, internalState @*/
	/*@modifies con->execs, con->numExecs,
		errno, fileSystem, internalState @*/;

/*@unused@*/
int poptReadDefaultConfig(poptContext con, /*@unused@*/ int useEnv)
	/*@globals fileSystem, internalState @*/
	/*@modifies con->execs, con->numExecs,
		fileSystem, internalState @*/;

int poptDupArgv(int argc, /*@null@*/ const char **argv,
		/*@null@*/ /*@out@*/ int * argcPtr,
		/*@null@*/ /*@out@*/ const char *** argvPtr)
	/*@modifies *argcPtr, *argvPtr @*/;

int poptParseArgvString(const char * s,
		/*@out@*/ int * argcPtr, /*@out@*/ const char *** argvPtr)
	/*@modifies *argcPtr, *argvPtr @*/;

/*@-fcnuse@*/
int poptConfigFileToString(FILE *fp, /*@out@*/ char ** argstrp, int flags)
	/*@globals fileSystem @*/
	/*@modifies *fp, *argstrp, fileSystem @*/;
/*@=fcnuse@*/

/*@observer@*/
const char * poptStrerror(const int error)
	/*@*/;

/*@unused@*/
void poptSetExecPath(poptContext con, const char * path, int allowAbsolute)
	/*@modifies con @*/;

void poptPrintHelp(poptContext con, FILE * fp, /*@unused@*/ int flags)
	/*@globals fileSystem @*/
	/*@modifies *fp, fileSystem @*/;

void poptPrintUsage(poptContext con, FILE * fp, /*@unused@*/ int flags)
	/*@globals fileSystem @*/
	/*@modifies *fp, fileSystem @*/;

/*@-fcnuse@*/
void poptSetOtherOptionHelp(poptContext con, const char * text)
	/*@modifies con @*/;
/*@=fcnuse@*/

/*@-fcnuse@*/
/*@observer@*/
const char * poptGetInvocationName(poptContext con)
	/*@*/;
/*@=fcnuse@*/

/*@-fcnuse@*/
int poptStrippedArgv(poptContext con, int argc, char ** argv)
	/*@modifies *argv @*/;
/*@=fcnuse@*/

/*@-incondefs@*/
/*@unused@*/
int poptSaveLong(/*@null@*/ long * arg, int argInfo, long aLong)
	/*@modifies *arg @*/
	/*@requires maxSet(arg) >= 0 /\ maxRead(arg) == 0 @*/;
/*@=incondefs@*/

/*@-incondefs@*/
/*@unused@*/
int poptSaveInt(/*@null@*/ int * arg, int argInfo, long aLong)
	/*@modifies *arg @*/
	/*@requires maxSet(arg) >= 0 /\ maxRead(arg) == 0 @*/;
/*@=incondefs@*/

/*@=type@*/
#ifdef  __cplusplus
}
#endif

#endif
