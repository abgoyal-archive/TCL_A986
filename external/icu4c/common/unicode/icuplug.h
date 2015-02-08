


#ifndef ICUPLUG_H
#define ICUPLUG_H

#include "unicode/utypes.h"


/* === Basic types === */


struct UPlugData;
typedef struct UPlugData UPlugData;

/** @} */

#define UPLUG_TOKEN 0x54762486

#define UPLUG_NAME_MAX              100


typedef uint32_t UPlugTokenReturn;

typedef enum {
    UPLUG_REASON_QUERY = 0,     /**< The plugin is being queried for info. **/
    UPLUG_REASON_LOAD = 1,     /**< The plugin is being loaded. **/
    UPLUG_REASON_UNLOAD = 2,   /**< The plugin is being unloaded. **/
    UPLUG_REASON_COUNT         /**< count of known reasons **/
} UPlugReason;


typedef enum {
    UPLUG_LEVEL_INVALID = 0,     /**< The plugin is invalid, hasn't called uplug_setLevel, or can't load. **/
    UPLUG_LEVEL_UNKNOWN = 1,     /**< The plugin is waiting to be installed. **/
    UPLUG_LEVEL_LOW     = 2,     /**< The plugin must be called before u_init completes **/
    UPLUG_LEVEL_HIGH    = 3,     /**< The plugin can run at any time. **/
    UPLUG_LEVEL_COUNT         /**< count of known reasons **/
} UPlugLevel;

typedef UPlugTokenReturn (U_EXPORT2 UPlugEntrypoint) (
                  UPlugData *plug,
                  UPlugReason reason,
                  UErrorCode *status);

/* === Needed for Implementing === */

U_CAPI void U_EXPORT2 
uplug_setPlugNoUnload(UPlugData *plug, UBool dontUnload);

U_CAPI void U_EXPORT2
uplug_setPlugLevel(UPlugData *plug, UPlugLevel level);

U_CAPI UPlugLevel U_EXPORT2
uplug_getPlugLevel(UPlugData *plug);

U_CAPI UPlugLevel U_EXPORT2
uplug_getCurrentLevel(void);


U_CAPI UErrorCode U_EXPORT2
uplug_getPlugLoadStatus(UPlugData *plug); 

U_CAPI void U_EXPORT2
uplug_setPlugName(UPlugData *plug, const char *name);

U_CAPI const char * U_EXPORT2
uplug_getPlugName(UPlugData *plug);

U_CAPI const char * U_EXPORT2
uplug_getSymbolName(UPlugData *plug);

U_CAPI const char * U_EXPORT2
uplug_getLibraryName(UPlugData *plug, UErrorCode *status);

U_CAPI void * U_EXPORT2
uplug_getLibrary(UPlugData *plug);

U_CAPI void * U_EXPORT2
uplug_getContext(UPlugData *plug);

U_CAPI void U_EXPORT2
uplug_setContext(UPlugData *plug, void *context);


U_CAPI const char * U_EXPORT2
uplug_getConfiguration(UPlugData *plug);

U_CAPI UPlugData* U_EXPORT2
uplug_nextPlug(UPlugData *prior);

U_CAPI UPlugData* U_EXPORT2
uplug_loadPlugFromEntrypoint(UPlugEntrypoint *entrypoint, const char *config, UErrorCode *status);


U_CAPI UPlugData* U_EXPORT2
uplug_loadPlugFromLibrary(const char *libName, const char *sym, const char *config, UErrorCode *status);

U_CAPI void U_EXPORT2
uplug_removePlug(UPlugData *plug, UErrorCode *status);


#endif
