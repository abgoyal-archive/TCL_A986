

#ifndef RemoveCSSPropertyCommand_h
#define RemoveCSSPropertyCommand_h

#include "EditCommand.h"
#include "CSSMutableStyleDeclaration.h"
#include "CSSPropertyNames.h"

namespace WebCore {

class RemoveCSSPropertyCommand : public SimpleEditCommand {
public:
    static PassRefPtr<RemoveCSSPropertyCommand> create(Document* document, PassRefPtr<CSSMutableStyleDeclaration> style, CSSPropertyID property)
    {
        return adoptRef(new RemoveCSSPropertyCommand(document, style, property));
    }

private:
    RemoveCSSPropertyCommand(Document*, PassRefPtr<CSSMutableStyleDeclaration>, CSSPropertyID property);

    virtual void doApply();
    virtual void doUnapply();

    RefPtr<CSSMutableStyleDeclaration> m_style;
    CSSPropertyID m_property;
    String m_oldValue;
    bool m_important;
};

} // namespace WebCore

#endif // RemoveCSSPropertyCommand_h
