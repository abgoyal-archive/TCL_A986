

#ifndef PluginInfoStore_h
#define PluginInfoStore_h

#include "PlatformString.h"
#include <wtf/Vector.h>

namespace WebCore {

struct PluginInfo;

class PluginInfoStore {
public:
    PluginInfo *createPluginInfoForPluginAtIndex(unsigned);
    unsigned pluginCount() const;
    static String pluginNameForMIMEType(const String& mimeType);
    static bool supportsMIMEType(const String& mimeType);
};

void refreshPlugins(bool reloadOpenPages);

}

#endif
