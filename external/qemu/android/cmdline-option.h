
#ifndef _ANDROID_OPTION_H
#define _ANDROID_OPTION_H

typedef struct ParamList {
    char*              param;
    struct ParamList*  next;
} ParamList;

typedef struct {
#define OPT_LIST(n,t,d)    ParamList*  n;
#define OPT_PARAM(n,t,d)   char*  n;
#define OPT_FLAG(n,d)      int    n;
#include "android/cmdline-options.h"
} AndroidOptions;


extern int
android_parse_options( int  *pargc, char**  *pargv, AndroidOptions*  opt );

/* name of default keyset file */
#define  KEYSET_FILE    "default.keyset"

#define  DEFAULT_DEVICE_DPI  165

#endif /* _ANDROID_OPTION_H */
