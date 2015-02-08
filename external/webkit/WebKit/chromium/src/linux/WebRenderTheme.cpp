

#include "config.h"
#include "WebRenderTheme.h"

#include "RenderThemeChromiumLinux.h"
#include "WebView.h"

using WebCore::RenderTheme;
using WebCore::RenderThemeChromiumLinux;

namespace WebKit {

void setCaretBlinkInterval(double interval)
{
    RenderThemeChromiumLinux::setCaretBlinkInterval(interval);
}

} // namespace WebKit
