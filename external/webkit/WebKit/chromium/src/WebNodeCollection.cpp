

#include "config.h"
#include "WebNodeCollection.h"

#include "HTMLCollection.h"
#include "Node.h"
#include <wtf/PassRefPtr.h>

#include "WebNode.h"

using namespace WebCore;

namespace WebKit {

void WebNodeCollection::reset()
{
    assign(0);
}

void WebNodeCollection::assign(const WebNodeCollection& other)
{
    HTMLCollection* p = const_cast<HTMLCollection*>(other.m_private);
    if (p)
        p->ref();
    assign(p);
}

WebNodeCollection::WebNodeCollection(const PassRefPtr<HTMLCollection>& col)
    : m_private(static_cast<HTMLCollection*>(col.releaseRef()))
{
}

void WebNodeCollection::assign(HTMLCollection* p)
{
    // p is already ref'd for us by the caller
    if (m_private)
        m_private->deref();
    m_private = p;
}

unsigned WebNodeCollection::length() const
{
    return m_private->length();
}

WebNode WebNodeCollection::nextItem() const
{
    return WebNode(m_private->nextItem());
}

WebNode WebNodeCollection::firstItem() const
{
    return WebNode(m_private->firstItem());
}

} // namespace WebKit
