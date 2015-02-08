

#ifndef CDATASection_h
#define CDATASection_h

#include "Text.h"

namespace WebCore {

class CDATASection : public Text {
public:
    static PassRefPtr<CDATASection> create(Document*, const String&);

private:
    CDATASection(Document*, const String&);

    virtual String nodeName() const;
    virtual NodeType nodeType() const;
    virtual PassRefPtr<Node> cloneNode(bool deep);
    virtual bool childTypeAllowed(NodeType);
    virtual PassRefPtr<Text> virtualCreate(const String&);
};

} // namespace WebCore

#endif // CDATASection_h
