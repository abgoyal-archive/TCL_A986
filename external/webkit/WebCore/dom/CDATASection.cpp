

#include "config.h"
#include "CDATASection.h"

#include "Document.h"

namespace WebCore {

inline CDATASection::CDATASection(Document* document, const String& data)
    : Text(document, data)
{
}

PassRefPtr<CDATASection> CDATASection::create(Document* document, const String& data)
{
    return adoptRef(new CDATASection(document, data));
}

String CDATASection::nodeName() const
{
    return "#cdata-section";
}

Node::NodeType CDATASection::nodeType() const
{
    return CDATA_SECTION_NODE;
}

PassRefPtr<Node> CDATASection::cloneNode(bool /*deep*/)
{
    return create(document(), data());
}

bool CDATASection::childTypeAllowed(NodeType)
{
    return false;
}

PassRefPtr<Text> CDATASection::virtualCreate(const String& data)
{
    return create(document(), data);
}

} // namespace WebCore
