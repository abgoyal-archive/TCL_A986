

#include "config.h"
#include "DocumentLoader.h"

#include "FrameWin.h"
#include "PlatformString.h"

namespace WebCore {

void DocumentLoader::setTitle(const String& title)
{
    String text = title;
    text.replace('//', m_frame->backslashAsCurrencySymbol());

    FrameWin* frameWin = static_cast<FrameWin*>(m_frame);
    if (frameWin->client())
        frameWin->client()->setTitle(text);
}

}
