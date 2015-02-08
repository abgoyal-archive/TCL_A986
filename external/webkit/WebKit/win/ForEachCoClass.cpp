

#include "config.h"
#include "WebKitDLL.h"
#include "ForEachCoClass.h"

// deprecated - do not use - remove once a registry-free version of Safari has shipped (first major version after 3.1.1)
void setUseOpenSourceWebKit(bool)
{
}

// deprecated - do not use - remove once a registry-free version of Safari has shipped (first major version after 3.1.1)
LPCOLESTR progIDForClass(WebKitClass cls)
{
    ASSERT(cls < WebKitClassSentinel);
    return s_progIDs[cls];
}
