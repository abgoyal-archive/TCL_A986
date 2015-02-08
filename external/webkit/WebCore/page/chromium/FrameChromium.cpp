

#include "config.h"

#include "Document.h"
#include "FloatRect.h"
#include "RenderView.h"
#include "Settings.h"

using std::min;

namespace WebCore {

DragImageRef Frame::dragImageForSelection()
{    
    if (selection()->isRange())
        return 0;  // FIXME: implement me!

    return 0;
}

} // namespace WebCore
