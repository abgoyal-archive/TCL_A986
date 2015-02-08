
#ifndef _ANDROID_UTILS_DIR_H
#define _ANDROID_UTILS_DIR_H

/* simple utility to parse directories for files            */
/* needed because Unix and Windows don't use the same stuff */

typedef struct DirScanner  DirScanner;

DirScanner*    dirScanner_new ( const char*  rootPath );

void           dirScanner_free( DirScanner*  s );

const char*    dirScanner_next( DirScanner*  s );

const char*    dirScanner_nextFull( DirScanner*  s );

/* */

#endif /* _ANDROID_UTILS_DIR_H */
