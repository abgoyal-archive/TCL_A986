

#include "config.h"
#include "PluginView.h"

using namespace WTF;

namespace WebCore {

void PluginView::setFocus()
{
}

void PluginView::show()
{
}

void PluginView::hide()
{
}

void PluginView::paint(GraphicsContext*, const IntRect&)
{
}

void PluginView::handleKeyboardEvent(KeyboardEvent*)
{
}

void PluginView::handleMouseEvent(MouseEvent*)
{
}

void PluginView::setParent(ScrollView*)
{
}

void PluginView::setNPWindowRect(const IntRect&)
{
}

NPError PluginView::handlePostReadFile(Vector<char>&, uint32, const char*)
{
    return 0;
}

NPError PluginView::getValue(NPNVariable, void*)
{
    return 0;
}

#if ENABLE(NETSCAPE_PLUGIN_API)
NPError PluginView::getValueStatic(NPNVariable, void*)
{
    return 0;
}
#endif

void PluginView::invalidateRect(NPRect*)
{
}

void PluginView::invalidateRect(const IntRect&)
{
}

void PluginView::invalidateRegion(NPRegion)
{
}

void PluginView::forceRedraw()
{
}

bool PluginView::platformStart()
{
    return true;
}

void PluginView::platformDestroy()
{
}

void PluginView::setParentVisible(bool)
{
}

void PluginView::updatePluginWidget()
{
}

void PluginView::halt()
{
}

void PluginView::restart()
{
}

void PluginView::keepAlive(NPP)
{
}

} // namespace WebCore
