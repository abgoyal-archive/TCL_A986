

#ifndef _ANDROID_UTILS_STRALLOC_H
#define _ANDROID_UTILS_STRALLOC_H

#include <stddef.h>
#include <stdarg.h>


typedef struct {
    char*     s;
    unsigned  n;
    unsigned  a;
} stralloc_t;

#define  STRALLOC_INIT        { NULL, 0, 0 }
#define  STRALLOC_DEFINE(s)   stralloc_t   s[1] = { STRALLOC_INIT }

extern void   stralloc_reset( stralloc_t*  s );
extern void   stralloc_ready( stralloc_t*  s, unsigned  len );
extern void   stralloc_readyplus( stralloc_t*  s, unsigned  len );

extern void   stralloc_copy( stralloc_t*  s, stralloc_t*  from );
extern void   stralloc_append( stralloc_t*  s, stralloc_t*  from );

extern void   stralloc_add_c( stralloc_t*  s, int  c );
extern void   stralloc_add_str( stralloc_t*  s, const char*  str );
extern void   stralloc_add_bytes( stralloc_t*  s, const void*  from, unsigned  len );

extern char*  stralloc_cstr( stralloc_t*  s );

extern void   stralloc_format( stralloc_t*  s, const char*  fmt, ... );
extern void   stralloc_formatv( stralloc_t*  s, const char*  fmt, va_list  args );
extern void   stralloc_add_format( stralloc_t*  s, const char*  fmt, ... );

extern void   stralloc_add_quote_c( stralloc_t*  s, int  c );
extern void   stralloc_add_quote_str( stralloc_t*  s, const char*  str );
extern void   stralloc_add_quote_bytes( stralloc_t*  s, const void*  from, unsigned   len );

extern void   stralloc_add_hex( stralloc_t*  s, unsigned  value, int  num_digits );
extern void   stralloc_add_hexdump( stralloc_t*  s, void*  base, int  size, const char*  prefix );

extern void   stralloc_tabular( stralloc_t*  s, const char** strings, int  count,
                                                const char*  prefix,  int  width );

extern char*  stralloc_to_tempstr( stralloc_t*  s );

#endif /* ANDROID_UTILS_STRALLOC_H */
