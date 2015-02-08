

#ifndef WebConsoleMessage_h
#define WebConsoleMessage_h

#include "WebString.h"

namespace WebKit {

struct WebConsoleMessage {
    enum Level {
        LevelTip,
        LevelLog,
        LevelWarning,
        LevelError
    };

    Level level;
    WebString text;

    WebConsoleMessage()
        : level(LevelLog) { }
    WebConsoleMessage(Level level, const WebString& text)
        : level(LevelLog)
        , text(text) { }
};

} // namespace WebKit

#endif
