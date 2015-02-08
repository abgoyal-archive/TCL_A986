

#ifndef PLUGIN_DEBUG_ANDROID_H__
#define PLUGIN_DEBUG_ANDROID_H__

#include "android_npapi.h"

// Define PLUGIN_DEBUG_LOCAL in an individual C++ file to enable for
// that file only.

// Define PLUGIN_DEBUG_GLOBAL to 1 to turn plug-in debug for all
// Android plug-in code in this directory.
#define PLUGIN_DEBUG_GLOBAL     0

#if PLUGIN_DEBUG_GLOBAL || defined(PLUGIN_DEBUG_LOCAL)
# define PLUGIN_LOG(FORMAT, ARGS...) do { anp_logPlugin(FORMAT, ## ARGS); } while(0)
# define PLUGIN_LOG_EVENT(NPP, EVT, RET, TIME) do { anp_logPluginEvent(NPP, EVT, RET, TIME); } while(0)

void anp_logPlugin(const char format[], ...);
void anp_logPluginEvent(void* npp, const ANPEvent* event, int16 returnVal, int elapsedTime);

#else
# define PLUGIN_LOG(A, B...)    do { } while(0)
# define PLUGIN_LOG_EVENT(NPP, EVT, RET, TIME) do { } while(0)

#endif

#endif // defined(PLUGIN_DEBUG_ANDROID_H__)
