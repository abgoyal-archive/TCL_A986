

#ifndef ProgIDMacros_h
#define ProgIDMacros_h

#define STRINGIFIED_VERSION(version) STRINGIFY(version)
#define STRINGIFY(s) L###s

#define CURRENT_PROGID_VERSION 526
#define VERSION_INDEPENDENT_PRODUCTION_PROGID(className) L##"WebKit." L###className
#define VERSION_INDEPENDENT_OPENSOURCE_PROGID(className) L##"OpenSource" VERSION_INDEPENDENT_PRODUCTION_PROGID(className)
#define VERSIONED_PROGID(versionIndependentProgID, version) versionIndependentProgID L##"." STRINGIFIED_VERSION(version)

#endif // !defined(ProgIDMacros_h)
