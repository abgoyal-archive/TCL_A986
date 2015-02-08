
#include "config.h"
#include "StyleBase.h"

#include "Document.h"
#include "Node.h"
#include "StyleSheet.h"

namespace WebCore {

String StyleBase::cssText() const
{
    return "";
}

void StyleBase::checkLoaded()
{
    if (parent())
        parent()->checkLoaded();
}

StyleSheet* StyleBase::stylesheet()
{
    StyleBase *b = this;
    while (b && !b->isStyleSheet())
        b = b->parent();
    return static_cast<StyleSheet*>(b);
}

KURL StyleBase::baseURL() const
{
    // Try to find the style sheet. If found look for its URL.
    // If it has none, get the URL from the parent sheet or the parent node.

    StyleSheet* sheet = const_cast<StyleBase*>(this)->stylesheet();
    if (!sheet)
        return KURL();
    if (!sheet->finalURL().isNull())
        return sheet->finalURL();
    if (sheet->parent())
        return sheet->parent()->baseURL();
    if (!sheet->ownerNode()) 
        return KURL();
    return sheet->ownerNode()->document()->baseURL();
}

#ifdef ANDROID_INSTRUMENT
static size_t styleSize = 0;

void* StyleBase::operator new(size_t size)
{
    styleSize += size;
    return ::operator new(size);
}

void* StyleBase::operator new[](size_t size)
{
    styleSize += size;
    return ::operator new[](size);
}

void StyleBase::operator delete(void* p, size_t size)
{
    styleSize -= size;
    ::operator delete(p);
}

void StyleBase::operator delete[](void* p, size_t size)
{
    styleSize -= size;
    ::operator delete[](p);
}

size_t StyleBase::reportStyleSize()
{
    return styleSize;
}
#endif

}
