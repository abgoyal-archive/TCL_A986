
#ifndef _ANDROID_RESOURCE_H
#define _ANDROID_RESOURCE_H

#include <stddef.h>

extern const unsigned char*
android_resource_find( const char*    name,
                       size_t        *psize );

extern const unsigned char*
android_icon_find( const char*   name,
                   size_t       *psize );

#endif /* END */

