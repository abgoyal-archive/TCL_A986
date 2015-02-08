

#ifndef _ANDROID_BOOT_PROPERTIES_H
#define _ANDROID_BOOT_PROPERTIES_H


#define  PROPERTY_MAX_NAME    32
#define  PROPERTY_MAX_VALUE   92

int  boot_property_add( const char*  name, const char*  value );

int  boot_property_add2( const char*  name, int  namelen,
                         const char*  value, int  valuelen );

void  boot_property_init_service( void );

void  boot_property_parse_option( const char*  param );

#endif /* _ANDROID_BOOT_PROPERTIES_H */
