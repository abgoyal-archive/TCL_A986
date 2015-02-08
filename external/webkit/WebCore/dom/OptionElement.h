

#ifndef OptionElement_h
#define OptionElement_h

#include "PlatformString.h"

namespace WebCore {

class Element;
class Document;
class OptionElementData;
class SelectElement;

class OptionElement {
public:
    virtual ~OptionElement() { }

    virtual bool selected() const = 0;
    virtual void setSelectedState(bool) = 0;

    virtual String text() const = 0;
    virtual String textIndentedToRespectGroupLabel() const = 0;
    virtual String value() const = 0;

protected:
    static void setSelectedState(OptionElementData&, Element*, bool selected);
    static int optionIndex(SelectElement*, const Element*);
    static String collectOptionLabelOrText(const OptionElementData&, const Element*);
    static String collectOptionTextRespectingGroupLabel(const OptionElementData&, const Element*);
    static String collectOptionValue(const OptionElementData&, const Element*);
private:
    static String collectOptionInnerText(const Element*);
    static String normalizeText(const Document*, const String&);
};

// HTML/WMLOptionElement hold this struct as member variable
// and pass it to the static helper functions in OptionElement
class OptionElementData {
public:
    OptionElementData();

    String value() const { return m_value; }
    void setValue(const String& value) { m_value = value; }

    String label() const { return m_label; }
    void setLabel(const String& label) { m_label = label; }

    bool selected() const { return m_selected; }
    void setSelected(bool selected) { m_selected = selected; }

private:
    String m_value;
    String m_label;
    bool m_selected;
};

OptionElement* toOptionElement(Element*);
bool isOptionElement(Element*);

}

#endif
