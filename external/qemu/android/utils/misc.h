
#ifndef _ANDROID_UTILS_MISC_H
#define _ANDROID_UTILS_MISC_H

#include <stdint.h>


extern void   print_tabular( const char** strings, int  count,
                             const char*  prefix,  int  width );


extern void   buffer_translate_char( char*        buff,
                                     unsigned     buffLen,
                                     const char*  src,
                                     char         fromChar,
                                     char         toChar );

extern void   string_translate_char( char*  str, char from, char to );


extern char*  tempstr_get( int   size );
extern char*  tempstr_format( const char*  fmt, ... );


extern const char*   quote_bytes( const char*  str, int  len );
extern const char*   quote_str( const char*  str );


extern int    hex2int( const uint8_t*  hex, int  len );

/* encodes an integer 'val' into 'len' hexadecimal charaters into 'hex' */
extern void   int2hex( uint8_t*  hex, int  len, int  val );

#endif /* _ANDROID_UTILS_MISC_H */
