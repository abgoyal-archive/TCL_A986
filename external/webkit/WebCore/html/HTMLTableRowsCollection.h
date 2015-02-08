

#ifndef HTMLTableRowsCollection_h
#define HTMLTableRowsCollection_h

#include "HTMLCollection.h"

namespace WebCore {

class HTMLTableElement;
class HTMLTableRowElement;

class HTMLTableRowsCollection : public HTMLCollection {
public:
    static PassRefPtr<HTMLTableRowsCollection> create(PassRefPtr<HTMLTableElement>);

    static HTMLTableRowElement* rowAfter(HTMLTableElement*, HTMLTableRowElement*);
    static HTMLTableRowElement* lastRow(HTMLTableElement*);

private:
    HTMLTableRowsCollection(PassRefPtr<HTMLTableElement>);

    virtual Element* itemAfter(Element*) const;
};

} // namespace

#endif
