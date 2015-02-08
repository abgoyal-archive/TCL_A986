

#ifndef WMLPostfieldElement_h
#define WMLPostfieldElement_h

#if ENABLE(WML)
#include "WMLElement.h"

namespace WebCore {

class WMLPostfieldElement : public WMLElement {
public:
    WMLPostfieldElement(const QualifiedName& tagName, Document*);

    virtual void insertedIntoDocument();
    virtual void removedFromDocument();

    String name() const;
    String value() const;

    // Encode name() and value() in a CString using the passed encoding
    void encodeData(const TextEncoding&, CString& name, CString& value);
};

}

#endif
#endif
