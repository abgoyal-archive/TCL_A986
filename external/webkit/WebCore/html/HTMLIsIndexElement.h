
#ifndef HTMLIsIndexElement_h
#define HTMLIsIndexElement_h

#include "HTMLInputElement.h"

namespace WebCore {

class HTMLIsIndexElement : public HTMLInputElement {
public:
    HTMLIsIndexElement(const QualifiedName&, Document *doc, HTMLFormElement *f = 0);

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusForbidden; }
    virtual int tagPriority() const { return 0; }

    virtual void parseMappedAttribute(MappedAttribute *attr);

    String prompt() const;
    void setPrompt(const String &);

protected:
    String m_prompt;
};

} //namespace

#endif
