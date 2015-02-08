

#ifndef SetNodeAttributeCommand_h
#define SetNodeAttributeCommand_h

#include "EditCommand.h"
#include "QualifiedName.h"

namespace WebCore {

class SetNodeAttributeCommand : public SimpleEditCommand {
public:
    static PassRefPtr<SetNodeAttributeCommand> create(PassRefPtr<Element> element, const QualifiedName& attribute, const AtomicString& value)
    {
        return adoptRef(new SetNodeAttributeCommand(element, attribute, value));
    }

private:
    SetNodeAttributeCommand(PassRefPtr<Element>, const QualifiedName& attribute, const AtomicString& value);

    virtual void doApply();
    virtual void doUnapply();

    RefPtr<Element> m_element;
    QualifiedName m_attribute;
    AtomicString m_value;
    AtomicString m_oldValue;
};

} // namespace WebCore

#endif // SetNodeAttributeCommand_h
