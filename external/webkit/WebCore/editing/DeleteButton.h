

#ifndef DeleteButton_h
#define DeleteButton_h

#include "HTMLImageElement.h"

namespace WebCore {

class DeleteButton : public HTMLImageElement {
public:
    DeleteButton(Document*);

    virtual void defaultEventHandler(Event*);
};

} // namespace

#endif
