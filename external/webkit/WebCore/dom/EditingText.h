

#ifndef EditingText_h
#define EditingText_h

#include "Text.h"

namespace WebCore {

class EditingText : public Text {
public:
    static PassRefPtr<EditingText> create(Document*, const String&);

private:
    virtual bool rendererIsNeeded(RenderStyle *);

    EditingText(Document*, const String&);
};

} // namespace WebCore

#endif // EditingText_h
