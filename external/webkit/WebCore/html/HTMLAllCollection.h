

#ifndef HTMLAllCollection_h
#define HTMLAllCollection_h

#include "HTMLCollection.h"

namespace WebCore {

class HTMLAllCollection : public HTMLCollection {
public:
    static PassRefPtr<HTMLAllCollection> create(PassRefPtr<Node>);
    virtual ~HTMLAllCollection();

private:
    HTMLAllCollection(PassRefPtr<Node>);
};

} // namespace WebCore

#endif // HTMLAllCollection_h
