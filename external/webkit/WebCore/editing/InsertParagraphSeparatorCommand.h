

#ifndef InsertParagraphSeparatorCommand_h
#define InsertParagraphSeparatorCommand_h

#include "CompositeEditCommand.h"

namespace WebCore {

class InsertParagraphSeparatorCommand : public CompositeEditCommand {
public:
    static PassRefPtr<InsertParagraphSeparatorCommand> create(Document* document, bool useDefaultParagraphElement = false)
    {
        return adoptRef(new InsertParagraphSeparatorCommand(document, useDefaultParagraphElement));
    }

private:
    InsertParagraphSeparatorCommand(Document*, bool useDefaultParagraphElement);

    virtual void doApply();

    void calculateStyleBeforeInsertion(const Position&);
    void applyStyleAfterInsertion(Node* originalEnclosingBlock);
    void getAncestorsInsideBlock(const Node* insertionNode, Element* outerBlock, Vector<Element*>& ancestors);
    PassRefPtr<Element> cloneHierarchyUnderNewBlock(const Vector<Element*>& ancestors, PassRefPtr<Element> blockToInsert);
    
    bool shouldUseDefaultParagraphElement(Node*) const;

    virtual bool preservesTypingStyle() const;

    RefPtr<CSSMutableStyleDeclaration> m_style;
    
    bool m_mustUseDefaultParagraphElement;
};

}

#endif
