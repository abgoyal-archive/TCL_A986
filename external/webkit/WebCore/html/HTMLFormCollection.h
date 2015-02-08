

#ifndef HTMLFormCollection_h
#define HTMLFormCollection_h

#include "HTMLCollection.h"

namespace WebCore {

class HTMLFormElement;
class QualifiedName;

// This class is just a big hack to find form elements even in malformed HTML elements.
// The famous <table><tr><form><td> problem.

class HTMLFormCollection : public HTMLCollection {
public:
    static PassRefPtr<HTMLFormCollection> create(PassRefPtr<HTMLFormElement>);

    virtual ~HTMLFormCollection();

    virtual Node* item(unsigned index) const;
    virtual Node* nextItem() const;

    virtual Node* namedItem(const AtomicString& name) const;
    virtual Node* nextNamedItem(const AtomicString& name) const;

private:
    HTMLFormCollection(PassRefPtr<HTMLFormElement>);

    virtual void updateNameCache() const;
    virtual unsigned calcLength() const;

    static CollectionCache* formCollectionInfo(HTMLFormElement*);

    Element* getNamedItem(const QualifiedName& attrName, const AtomicString& name) const;
    Element* nextNamedItemInternal(const String& name) const;

    Element* getNamedFormItem(const QualifiedName& attrName, const String& name, int duplicateNumber) const;

    mutable int currentPos;
};

} //namespace

#endif
