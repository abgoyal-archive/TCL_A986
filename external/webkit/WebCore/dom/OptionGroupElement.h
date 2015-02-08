

#ifndef OptionGroupElement_h
#define OptionGroupElement_h

namespace WebCore {

class Element;
class String;

class OptionGroupElement {
public:
    virtual ~OptionGroupElement() { }

    virtual String groupLabelText() const = 0;
};

OptionGroupElement* toOptionGroupElement(Element*);
bool isOptionGroupElement(Element*);

}

#endif
