

#include "config.h"
#include <wtf/chromium/ChromiumThreading.h>

#include "WebKit.h"
#include "WebKitClient.h"

#include <wtf/MainThread.h>

namespace WTF {

void ChromiumThreading::initializeMainThread()
{
}

void ChromiumThreading::scheduleDispatchFunctionsOnMainThread()
{
    WebKit::webKitClient()->callOnMainThread(&WTF::dispatchFunctionsFromMainThread);
}

}  // namespace WTF
