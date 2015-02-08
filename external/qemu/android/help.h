
#ifndef _ANDROID_HELP_H
#define _ANDROID_HELP_H

#include "android/utils/stralloc.h"

#define  BOOT_PROPERTY_MAX_NAME    32
#define  BOOT_PROPERTY_MAX_VALUE   92

/* appends the list of options with a small description to a dynamic string */
extern void  android_help_list_options( stralloc_t*  out );

/* output main help screen into a single dynamic string */
extern void  android_help_main( stralloc_t*  out );

/* output all help into a single dynamic string */
extern void  android_help_all( stralloc_t*  out );

extern int  android_help_for_option( const char*  option, stralloc_t*  out );

extern int  android_help_for_topic( const char*  topic, stralloc_t*  out );

#endif /* _ANDROID_HELP_H */
