
#ifndef _ANDROID_UTILS_PATH_H
#define _ANDROID_UTILS_PATH_H

#include <android/utils/system.h>
#include <stdint.h>  /* for uint64_t */


#if !defined(_WIN32) && !defined(O_BINARY)
#  define  O_BINARY  0
#endif

/* define  PATH_SEP as a string containing the directory separateor */
#ifdef _WIN32
#  define  PATH_SEP   "\\"
#  define  PATH_SEP_C '\\'
#else
#  define  PATH_SEP   "/"
#  define  PATH_SEP_C '/'
#endif

#ifdef _WIN32
#  undef   MAX_PATH
#  define  MAX_PATH    1024
#  undef   PATH_MAX
#  define  PATH_MAX    MAX_PATH
#else
#  include <limits.h>
#  define  MAX_PATH    PATH_MAX
#endif

/* checks that a given file exists */
extern ABool  path_exists( const char*  path );

/* checks that a path points to a regular file */
extern ABool  path_is_regular( const char*  path );

/* checks that a path points to a directory */
extern ABool  path_is_dir( const char*  path );

/* checks that a path is absolute or not */
extern ABool  path_is_absolute( const char*  path );

/* checks that one can read/write a given (regular) file */
extern ABool  path_can_read( const char*  path );
extern ABool  path_can_write( const char*  path );

/* try to make a directory */
extern APosixStatus   path_mkdir( const char*  path, int  mode );

extern APosixStatus   path_mkdir_if_needed( const char*  path, int  mode );

extern APosixStatus   path_get_size( const char*  path, uint64_t  *psize );

extern char*  path_parent( const char*  path, int  levels );

extern int    path_split( const char*  path, char* *pdirname, char* *pbasename );

extern char*  path_dirname( const char*  path );

extern char*  path_basename( const char*  path );


extern APosixStatus   path_empty_file( const char*  path );

extern APosixStatus   path_copy_file( const char*  dest, const char*  source );

extern APosixStatus   path_delete_file( const char*  path );

extern void*          path_load_file( const char*  path, size_t  *pSize );

/* */

#endif /* _ANDROID_UTILS_PATH_H */
