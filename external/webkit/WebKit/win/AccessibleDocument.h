

#ifndef AccessibleDocument_h
#define AccessibleDocument_h

#include "AccessibleBase.h"
#include "WebKit.h"

using WebCore::Document;

class AccessibleDocument : public AccessibleBase {
public:
    AccessibleDocument(Document*);
    Document* document() const;

protected:
    virtual long role() const;
};

#endif // AccessibleDocument_h
