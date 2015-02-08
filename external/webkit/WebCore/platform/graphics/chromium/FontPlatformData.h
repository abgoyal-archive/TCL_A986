

#ifndef FontPlatformData_h
#define FontPlatformData_h

#if OS(WINDOWS)
#include "FontPlatformDataChromiumWin.h"
#elif OS(LINUX)
#include "FontPlatformDataLinux.h"
#endif

#endif  // FontPlatformData_h
