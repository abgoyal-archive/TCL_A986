
#ifndef _PKGTYPES
#define _PKGTYPES

/* headers */
#include "unicode/utypes.h"
#include "filestrm.h"

/* linked list */
struct _CharList;

typedef struct _CharList
{
  const char       *str;
  struct _CharList *next;
} CharList;



const char *pkg_writeCharList(FileStream *s, CharList *l, const char *delim, int32_t quoted);

const char *pkg_writeCharListWrap(FileStream *s, CharList *l, const char *delim, const char *brk, int32_t quoted);


uint32_t pkg_countCharList(CharList *l);

CharList *pkg_prependToList(CharList *l, const char *str);

CharList *pkg_appendToList(CharList *l, CharList** end, const char *str);

CharList *pkg_appendUniqueDirToList(CharList *l, CharList** end, const char *strAlias);

UBool  pkg_listContains(CharList *l, const char *str);

void pkg_deleteList(CharList *l);

struct UPKGOptions_;
typedef   void (UPKGMODE)(struct UPKGOptions_ *, FileStream *s, UErrorCode *status);

void pkg_sttc_writeReadme(struct UPKGOptions_ *opt, const char *libName, UErrorCode *status);


typedef struct UPKGOptions_
{
  CharList   *fileListFiles; /* list of files containing files for inclusion in the package */
  CharList   *filePaths;     /* All the files, with long paths */
  CharList   *files;         /* All the files */
  CharList   *outFiles;      /* output files [full paths] */

  const char *shortName;   /* name of what we're building */
  const char *cShortName;   /* name of what we're building as a C identifier */
  const char *entryName;   /* special entrypoint name */
  const char *targetDir;  /* dir for packaged data to go */
  const char *dataDir;    /* parent of dir for package (default: tmpdir) */
  const char *tmpDir;
  const char *srcDir;
  const char *options;     /* Options arg */
  const char *mode;        /* Mode of building */
  const char *version;     /* Library version */
  const char *comment;     /* comment string */
  const char *install;     /* Where to install to (NULL = don't install) */
  const char *icuroot;     /* where does ICU lives */
  const char *libName;     /* name for library (default: shortName) */
  UBool      rebuild;
  UBool      verbose;
  UBool      quiet;
} UPKGOptions;

char * convertToNativePathSeparators(char *path);


/* set up common defines for library naming */

#ifdef U_WINDOWS
# ifndef UDATA_SO_SUFFIX
#  define UDATA_SO_SUFFIX ".dll"
# endif
# define LIB_PREFIX ""
# define LIB_STATIC_PREFIX ""
# define OBJ_SUFFIX ".obj"
# define UDATA_LIB_SUFFIX ".lib"

#elif defined(U_CYGWIN)
# define LIB_PREFIX "cyg"
# define LIB_STATIC_PREFIX "lib"
# define OBJ_SUFFIX ".o"
# define UDATA_LIB_SUFFIX ".a"

#else  /* POSIX? */
# define LIB_PREFIX "lib"
# define LIB_STATIC_PREFIX "lib"
# define OBJ_SUFFIX ".o"
# define UDATA_LIB_SUFFIX ".a"
#endif

#define ASM_SUFFIX ".s"

/* defines for common file names */
#define UDATA_CMN_PREFIX ""
#define UDATA_CMN_SUFFIX ".dat"
#define UDATA_CMN_INTERMEDIATE_SUFFIX "_dat"

#define ICUDATA_RES_FILE "icudata.res"

#define PKGDATA_DERIVED_PATH '\t'

#endif
