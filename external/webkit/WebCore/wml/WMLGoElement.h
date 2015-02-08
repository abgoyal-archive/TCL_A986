

#ifndef WMLGoElement_h
#define WMLGoElement_h

#if ENABLE(WML)
#include "FormDataBuilder.h"
#include "WMLTaskElement.h"

namespace WebCore {

class FormData;
class ResourceRequest;
class WMLPostfieldElement;

class WMLGoElement : public WMLTaskElement {
public:
    WMLGoElement(const QualifiedName& tagName, Document*);

    void registerPostfieldElement(WMLPostfieldElement*);
    void deregisterPostfieldElement(WMLPostfieldElement*);

    virtual void parseMappedAttribute(MappedAttribute*);
    virtual void executeTask();

private:
    void preparePOSTRequest(ResourceRequest&, bool inSameDeck, const String& cacheControl);
    void prepareGETRequest(ResourceRequest&, const KURL&);

    PassRefPtr<FormData> createFormData(const CString& boundary);

    Vector<WMLPostfieldElement*> m_postfieldElements;
    FormDataBuilder m_formDataBuilder;
};

}

#endif
#endif
