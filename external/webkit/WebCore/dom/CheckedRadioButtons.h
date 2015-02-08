

#ifndef CheckedRadioButtons_h
#define CheckedRadioButtons_h

#include <wtf/HashMap.h>
#include <wtf/OwnPtr.h>

namespace WebCore {

class AtomicString;
class AtomicStringImpl;
class HTMLInputElement;
class HTMLFormControlElement;

class CheckedRadioButtons {
public:
    void addButton(HTMLFormControlElement*);
    void removeButton(HTMLFormControlElement*);
    HTMLInputElement* checkedButtonForGroup(const AtomicString& groupName) const;

private:
    typedef HashMap<AtomicStringImpl*, HTMLInputElement*> NameToInputMap;
    OwnPtr<NameToInputMap> m_nameToCheckedRadioButtonMap;
};

} // namespace WebCore

#endif // CheckedRadioButtons_h
