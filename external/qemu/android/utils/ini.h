
#ifndef _ANDROID_UTILS_INI_H
#define _ANDROID_UTILS_INI_H

#include <stdint.h>


/* an opaque structure used to model an .ini configuration file */
typedef struct IniFile   IniFile;

IniFile*  iniFile_newFromMemory( const char*  text, const char*  fileName  );

IniFile*  iniFile_newFromFile( const char*  filePath);

int       iniFile_saveToFile( IniFile*  f, const char*  filePath );

/* free an IniFile object */
void      iniFile_free( IniFile*  f );

/* returns the number of (key.value) pairs in an IniFile */
int       iniFile_getPairCount( IniFile*  f );

void      iniFile_getPair( IniFile*     f,
                           int          index,
                           const char* *pKey,
                           const char* *pValue );

const char*  iniFile_getValue( IniFile*  f, const char*  key );

char*   iniFile_getString( IniFile*  f, const char*  key );

int     iniFile_getInteger( IniFile*  f, const char*  key, int  defaultValue );

int64_t iniFile_getInt64( IniFile*  f, const char*  key, int64_t  defaultValue );

double  iniFile_getDouble( IniFile*  f, const char*  key, double  defaultValue );

char*   iniFile_getString( IniFile*  f, const char*  key );

int     iniFile_getBoolean( IniFile*  f, const char*  key, const char*  defaultValue );

int64_t  iniFile_getDiskSize( IniFile*  f, const char*  key, const char*  defaultValue );

/* */

#endif /* _ANDROID_UTILS_INI_H */
