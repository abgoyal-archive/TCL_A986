
#ifndef __UOPTIONS_H__
#define __UOPTIONS_H__

#include "unicode/utypes.h"

/* This should usually be called before calling u_parseArgs */
/*#if defined(OS390) && (U_CHARSET_FAMILY == U_ASCII_FAMILY)*/
    /* translate args from EBCDIC to ASCII */
/*#   define U_MAIN_INIT_ARGS(argc, argv) __argvtoascii_a(argc, argv)*/
/*#elif defined(XP_MAC_CONSOLE)*/
#if defined(XP_MAC_CONSOLE)
#   include <console.h>
    /* Get the arguments from the GUI, since old Macs don't have a console Window. */
#   define U_MAIN_INIT_ARGS(argc, argv) argc = ccommand((char***)&argv)
#else
    /* Normally we do nothing. */
#   define U_MAIN_INIT_ARGS(argc, argv)
#endif



/* forward declarations for the function declaration */
struct UOption;
typedef struct UOption UOption;

/* function to be called for a command line option */
typedef int UOptionFn(void *context, UOption *option);

/* values of UOption.hasArg */
enum { UOPT_NO_ARG, UOPT_REQUIRES_ARG, UOPT_OPTIONAL_ARG };

/* structure describing a command line option */
struct UOption {
    const char *longName;   /* "foo" for --foo */
    const char *value;      /* output placeholder, will point to the argument string, if any */
    UOptionFn *optionFn;    /* function to be called when this option occurs */
    void *context;          /* parameter for the function */
    char shortName;         /* 'f' for -f */
    char hasArg;            /* enum value: option takes no/requires/may have argument */
    char doesOccur;         /* boolean for "this one occured" */
};

/* macro for an entry in a declaration of UOption[] */
#define UOPTION_DEF(longName, shortName, hasArg) \
    { longName, NULL, NULL, NULL, shortName, hasArg, 0 }

/* ICU Tools option definitions */
#define UOPTION_HELP_H              UOPTION_DEF("help", 'h', UOPT_NO_ARG)
#define UOPTION_HELP_QUESTION_MARK  UOPTION_DEF("help", '?', UOPT_NO_ARG)
#define UOPTION_VERBOSE             UOPTION_DEF("verbose", 'v', UOPT_NO_ARG)
#define UOPTION_QUIET               UOPTION_DEF("quiet", 'q', UOPT_NO_ARG)
#define UOPTION_VERSION             UOPTION_DEF("version", 'V', UOPT_NO_ARG)
#define UOPTION_COPYRIGHT           UOPTION_DEF("copyright", 'c', UOPT_NO_ARG)

#define UOPTION_DESTDIR             UOPTION_DEF("destdir", 'd', UOPT_REQUIRES_ARG)
#define UOPTION_SOURCEDIR           UOPTION_DEF("sourcedir", 's', UOPT_REQUIRES_ARG)
#define UOPTION_ENCODING            UOPTION_DEF("encoding", 'e', UOPT_REQUIRES_ARG)
#define UOPTION_ICUDATADIR          UOPTION_DEF("icudatadir", 'i', UOPT_REQUIRES_ARG)
#define UOPTION_WRITE_JAVA          UOPTION_DEF("write-java", 'j', UOPT_OPTIONAL_ARG)
#define UOPTION_PACKAGE_NAME        UOPTION_DEF("package-name", 'p', UOPT_REQUIRES_ARG)
#define UOPTION_BUNDLE_NAME         UOPTION_DEF("bundle-name", 'b', UOPT_REQUIRES_ARG)

U_CAPI int U_EXPORT2
u_parseArgs(int argc, char* argv[],
            int optionCount, UOption options[]);

#endif
