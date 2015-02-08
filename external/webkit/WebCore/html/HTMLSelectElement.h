

#ifndef HTMLSelectElement_h
#define HTMLSelectElement_h

#include "CollectionCache.h"
#include "HTMLFormControlElement.h"
#include "SelectElement.h"

namespace WebCore {

class HTMLOptionElement;
class HTMLOptionsCollection;
class KeyboardEvent;

class HTMLSelectElement : public HTMLFormControlElementWithState, public SelectElement {
public:
    HTMLSelectElement(const QualifiedName&, Document*, HTMLFormElement* = 0);

    virtual int selectedIndex() const;
    virtual void setSelectedIndex(int index, bool deselect = true);
    virtual void setSelectedIndexByUser(int index, bool deselect = true, bool fireOnChangeNow = false);

    unsigned length() const;

    virtual int size() const { return m_data.size(); }
    virtual bool multiple() const { return m_data.multiple(); }

    void add(HTMLElement* element, HTMLElement* before, ExceptionCode&);
    void remove(int index);

    String value();
    void setValue(const String&);

    PassRefPtr<HTMLOptionsCollection> options();

    virtual void childrenChanged(bool changedByParser = false, Node* beforeChange = 0, Node* afterChange = 0, int childCountDelta = 0);

    void setRecalcListItems();
    void recalcListItemsIfNeeded();

    virtual const Vector<Element*>& listItems() const { return m_data.listItems(this); }

    virtual void accessKeyAction(bool sendToAnyElement);
    void accessKeySetSelectedIndex(int);

    void setMultiple(bool);

    void setSize(int);

    void setOption(unsigned index, HTMLOptionElement*, ExceptionCode&);
    void setLength(unsigned, ExceptionCode&);

    Node* namedItem(const AtomicString& name);
    Node* item(unsigned index);

    CollectionCache* collectionInfo() { m_collectionInfo.checkConsistency(); return &m_collectionInfo; }

    void scrollToSelection();

private:
    virtual int tagPriority() const { return 6; }
    virtual bool checkDTD(const Node* newChild);

    virtual const AtomicString& formControlType() const;
    
    virtual bool isKeyboardFocusable(KeyboardEvent*) const;
    virtual bool isMouseFocusable() const;
    virtual bool canSelectAll() const;
    virtual void selectAll();

    virtual void recalcStyle(StyleChange);

    virtual void dispatchFocusEvent();
    virtual void dispatchBlurEvent();
    
    virtual bool canStartSelection() const { return false; }

    virtual bool isEnumeratable() const { return true; }

    virtual bool saveFormControlState(String& value) const;
    virtual void restoreFormControlState(const String&);

    virtual void parseMappedAttribute(MappedAttribute*);

    virtual RenderObject* createRenderer(RenderArena*, RenderStyle *);
    virtual bool appendFormData(FormDataList&, bool);

    virtual int optionToListIndex(int optionIndex) const;
    virtual int listToOptionIndex(int listIndex) const;

    virtual void reset();

    virtual void defaultEventHandler(Event*);

    virtual void setActiveSelectionAnchorIndex(int index);
    virtual void setActiveSelectionEndIndex(int index);
    virtual void updateListBoxSelection(bool deselectOtherOptions);
    virtual void listBoxOnChange();
    virtual void menuListOnChange();
    
    virtual int activeSelectionStartListIndex() const;
    virtual int activeSelectionEndListIndex() const;
    
    void recalcListItems(bool updateSelectedStates = true) const;

    void deselectItems(HTMLOptionElement* excludeElement = 0);
    void typeAheadFind(KeyboardEvent*);
    void saveLastSelection();

    virtual void insertedIntoTree(bool);

    virtual bool isOptionalFormControl() const { return true; }

    SelectElementData m_data;
    CollectionCache m_collectionInfo;
};

} // namespace

#endif
