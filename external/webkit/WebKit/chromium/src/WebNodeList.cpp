

#include "config.h"
#include "WebNodeList.h"

#include "Node.h"
#include "NodeList.h"
#include <wtf/PassRefPtr.h>

#include "WebNode.h"

using namespace WebCore;

namespace WebKit {

void WebNodeList::reset()
{
    assign(0);
}

void WebNodeList::assign(const WebNodeList& other)
{
    NodeList* p = const_cast<NodeList*>(other.m_private);
    if (p)
        p->ref();
    assign(p);
}

WebNodeList::WebNodeList(const PassRefPtr<NodeList>& col)
    : m_private(static_cast<NodeList*>(col.releaseRef()))
{
}

void WebNodeList::assign(NodeList* p)
{
    // p is already ref'd for us by the caller
    if (m_private)
        m_private->deref();
    m_private = p;
}

unsigned WebNodeList::length() const
{
    return m_private->length();
}

WebNode WebNodeList::item(size_t index) const
{
    return WebNode(m_private->item(index));
}

} // namespace WebKit
