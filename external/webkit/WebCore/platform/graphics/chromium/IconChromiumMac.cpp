

#include "config.h"
#include "Icon.h"

#include "PassRefPtr.h"

// FIXME: These are temporary stubs, we need real implementations which
// may come in the form of IconChromium.cpp.  The Windows Chromium
// implementation is currently in IconWin.cpp.
 
namespace WebCore {

PassRefPtr<Icon> Icon::createIconForFiles(const Vector<String>&)
{
    return 0;
}

Icon::~Icon()
{
}

void Icon::paint(GraphicsContext*, const IntRect&)
{
}

}
