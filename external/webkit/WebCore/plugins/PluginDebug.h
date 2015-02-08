

#ifndef PluginDebug_h
#define PluginDebug_h

#include "Logging.h"
#include "npruntime_internal.h"
#include "CString.h"

#define LOG_NPERROR(err) if (err != NPERR_NO_ERROR) LOG_VERBOSE(Plugins, "%s\n", prettyNameForNPError(err))
#define LOG_PLUGIN_NET_ERROR() LOG_VERBOSE(Plugins, "Stream failed due to problems with network, disk I/O, lack of memory, or other problems.\n")

#if !LOG_DISABLED

namespace WebCore {

const char* prettyNameForNPError(NPError error);

CString prettyNameForNPNVariable(NPNVariable variable);
CString prettyNameForNPPVariable(NPPVariable variable, void* value);

#ifdef XP_MACOSX
const char* prettyNameForDrawingModel(NPDrawingModel drawingModel);
const char* prettyNameForEventModel(NPEventModel eventModel);
#endif

} // namespace WebCore

#endif // !LOG_DISABLED

#endif // PluginDebug_h
