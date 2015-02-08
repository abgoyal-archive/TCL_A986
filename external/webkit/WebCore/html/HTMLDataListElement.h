

#ifndef HTMLDataListElement_h
#define HTMLDataListElement_h

#if ENABLE(DATALIST)

#include "HTMLCollection.h"
#include "HTMLElement.h"

namespace WebCore {

    class HTMLDataListElement : public HTMLElement {
    public:
        HTMLDataListElement(const QualifiedName&, Document*);
        virtual ~HTMLDataListElement();
        PassRefPtr<HTMLCollection> options();

    private:
        virtual bool checkDTD(const Node*);
    };

}  // namespace WebCore

#endif  // ENABLE(DATALIST)

#endif  // HTMLDataListElement_h
