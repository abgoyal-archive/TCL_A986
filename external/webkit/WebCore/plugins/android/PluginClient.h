

#ifndef PLUGINCLIENT_H_
#define PLUGINCLIENT_H_

#include "PlatformString.h"
#include <wtf/Vector.h>

using namespace WebCore;

namespace android {

    class PluginClient
    {
    public:
        virtual ~PluginClient() {}
        virtual Vector<String> getPluginDirectories() = 0;
        virtual String getPluginSharedDataDirectory() = 0;
    };
}

#endif /* PLUGINCLIENT_H_ */
