

#ifndef HTMLNameCollection_h
#define HTMLNameCollection_h

#include "HTMLCollection.h"
#include "PlatformString.h"

namespace WebCore {

class Document;

class HTMLNameCollection : public HTMLCollection {
public:
    static PassRefPtr<HTMLNameCollection> create(PassRefPtr<Document> document, CollectionType type, const String& name)
    {
        return adoptRef(new HTMLNameCollection(document, type, name));
    }
    
private:
    HTMLNameCollection(PassRefPtr<Document>, CollectionType, const String& name);

    virtual Element* itemAfter(Element*) const;

    String m_name;
};

}

#endif
