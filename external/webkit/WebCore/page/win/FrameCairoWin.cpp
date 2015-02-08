

#include "config.h"
#include "Frame.h"
#include "FrameWin.h"

#include "EditorClient.h"
#include "NotImplemented.h"

using std::min;

namespace WebCore {

HBITMAP imageFromSelection(Frame* frame, bool forceBlackText)
{
    notImplemented();
    return 0;
}

HBITMAP Frame::nodeImage(Node*) const
{
    notImplemented();
    return 0;
}

} // namespace WebCore
