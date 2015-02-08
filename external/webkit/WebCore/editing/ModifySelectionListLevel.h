

#ifndef ModifySelectionListLevel_h
#define ModifySelectionListLevel_h

#include "CompositeEditCommand.h"

namespace WebCore {

// ModifySelectionListLevelCommand provides functions useful for both increasing and decreasing the list level.
// It is the base class of IncreaseSelectionListLevelCommand and DecreaseSelectionListLevelCommand.
// It is not used on its own.
class ModifySelectionListLevelCommand : public CompositeEditCommand {
protected:
    ModifySelectionListLevelCommand(Document*);
    
    void appendSiblingNodeRange(Node* startNode, Node* endNode, Element* newParent);
    void insertSiblingNodeRangeBefore(Node* startNode, Node* endNode, Node* refNode);
    void insertSiblingNodeRangeAfter(Node* startNode, Node* endNode, Node* refNode);

private:
    virtual bool preservesTypingStyle() const;    
};

// IncreaseSelectionListLevelCommand moves the selected list items one level deeper.
class IncreaseSelectionListLevelCommand : public ModifySelectionListLevelCommand {
public:
    static bool canIncreaseSelectionListLevel(Document*);
    static PassRefPtr<Node> increaseSelectionListLevel(Document*);
    static PassRefPtr<Node> increaseSelectionListLevelOrdered(Document*);
    static PassRefPtr<Node> increaseSelectionListLevelUnordered(Document*);

private:
    enum Type { InheritedListType, OrderedList, UnorderedList };
    static PassRefPtr<Node> increaseSelectionListLevelWithType(Document*, Type listType);

    IncreaseSelectionListLevelCommand(Document*, Type);
    virtual void doApply();

    Type m_listType;
    RefPtr<Node> m_listElement;
};

// DecreaseSelectionListLevelCommand moves the selected list items one level shallower.
class DecreaseSelectionListLevelCommand : public ModifySelectionListLevelCommand {
public:
    static bool canDecreaseSelectionListLevel(Document*);
    static void decreaseSelectionListLevel(Document*);

private:
    DecreaseSelectionListLevelCommand(Document*);
    virtual void doApply();
};

} // namespace WebCore

#endif // ModifySelectionListLevel_h
