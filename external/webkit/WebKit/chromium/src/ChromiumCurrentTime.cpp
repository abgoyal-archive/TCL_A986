

#include "config.h"
#include <wtf/CurrentTime.h>

#include "WebKit.h"
#include "WebKitClient.h"

namespace WTF {

double currentTime()
{
    return WebKit::webKitClient()->currentTime();
}

}  // namespace WTF
