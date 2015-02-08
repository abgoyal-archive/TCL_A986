

#define LOG_TAG "webkitPlugin"

#include "utils/Log.h"
#include "android_npapi.h"
#include <stdarg.h>

static void anp_log(ANPLogType logType, const char format[], ...) {
    va_list args;
    va_start(args, format);

    android_LogPriority priority;
    switch (logType) {
        case kError_ANPLogType:
            priority = ANDROID_LOG_ERROR;
            break;
        case kWarning_ANPLogType:
            priority = ANDROID_LOG_WARN;
            break;
        case kDebug_ANPLogType:
            priority = ANDROID_LOG_DEBUG;
            break;
        default:
            priority = ANDROID_LOG_UNKNOWN;
            break;
    }
    LOG_PRI_VA(priority, "plugin", format, args);

    va_end(args);
}

void ANPLogInterfaceV0_Init(ANPInterface* value) {
    ANPLogInterfaceV0* i = reinterpret_cast<ANPLogInterfaceV0*>(value);

    i->log = anp_log;
}
