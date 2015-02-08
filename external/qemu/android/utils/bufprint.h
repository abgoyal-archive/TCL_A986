

#ifndef _ANDROID_UTILS_BUFPRINT_H
#define _ANDROID_UTILS_BUFPRINT_H

#include <stdarg.h>


extern char*   vbufprint(char*  buffer, char*  buffend, const char*  fmt, va_list  args );
extern char*   bufprint (char*  buffer, char*  buffend, const char*  fmt, ... );


extern char*  bufprint_app_dir    (char*  buffer, char*  buffend);
extern char*  bufprint_config_path(char*  buffer, char*  buffend);
extern char*  bufprint_config_file(char*  buffer, char*  buffend, const char*  suffix);
extern char*  bufprint_temp_dir   (char*  buffer, char*  buffend);
extern char*  bufprint_temp_file  (char*  buffer, char*  buffend, const char*  suffix);

#endif /* _ANDROID_UTILS_BUFPRINT_H */
