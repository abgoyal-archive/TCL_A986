

#ifndef CompositeEditCommand_h
#define CompositeEditCommand_h

#include "EditCommand.h"
#include "CSSPropertyNames.h"
#include <wtf/Vector.h>

namespace WebCore {

class CSSStyleDeclaration;
class HTMLElement;
class Text;

class CompositeEditCommand : public EditCommand {
public:
    bool isFirstCommand(EditCommand* command) { return !m_commands.isEmpty() && m_commands.first() == command; }

protected:
    CompositeEditCommand(Document*);

    //
    // sugary-sweet convenience functions to help create and apply edit commands in composite commands
    //
    void appendNode(PassRefPtr<Node>, PassRefPtr<Element> parent);
    void applyCommandToComposite(PassRefPtr<EditCommand>);
    void applyStyle(CSSStyleDeclaration*, EditAction = EditActionChangeAttributes);
    void applyStyle(CSSStyleDeclaration*, const Position& start, const Position& end, EditAction = EditActionChangeAttributes);
    void applyStyledElement(PassRefPtr<Element>);
    void removeStyledElement(PassRefPtr<Element>);
    void deleteSelection(bool smartDelete = false, bool mergeBlocksAfterDelete = true, bool replace = false, bool expandForSpecialElements = true);
    void deleteSelection(const VisibleSelection&, bool smartDelete = false, bool mergeBlocksAfterDelete = true, bool replace = false, bool expandForSpecialElements = true);
    virtual void deleteTextFromNode(PassRefPtr<Text>, unsigned offset, unsigned count);
    void inputText(const String&, bool selectInsertedText = false);
    void insertNodeAfter(PassRefPtr<Node>, PassRefPtr<Node> refChild);
    void insertNodeAt(PassRefPtr<Node>, const Position&);
    void insertNodeAtTabSpanPosition(PassRefPtr<Node>, const Position&);
    void insertNodeBefore(PassRefPtr<Node>, PassRefPtr<Node> refChild);
    void insertParagraphSeparator(bool useDefaultParagraphElement = false);
    void insertLineBreak();
    void insertTextIntoNode(PassRefPtr<Text>, unsigned offset, const String& text);
    void joinTextNodes(PassRefPtr<Text>, PassRefPtr<Text>);
    void mergeIdenticalElements(PassRefPtr<Element>, PassRefPtr<Element>);
    void rebalanceWhitespace();
    void rebalanceWhitespaceAt(const Position&);
    void prepareWhitespaceAtPositionForSplit(Position&);
    void removeCSSProperty(PassRefPtr<CSSMutableStyleDeclaration>, CSSPropertyID);
    void removeNodeAttribute(PassRefPtr<Element>, const QualifiedName& attribute);
    void removeChildrenInRange(PassRefPtr<Node>, unsigned from, unsigned to);
    virtual void removeNode(PassRefPtr<Node>);
    HTMLElement* replaceNodeWithSpanPreservingChildrenAndAttributes(PassRefPtr<Node>);
    void removeNodePreservingChildren(PassRefPtr<Node>);
    void removeNodeAndPruneAncestors(PassRefPtr<Node>);
    void prune(PassRefPtr<Node>);
    void replaceTextInNode(PassRefPtr<Text>, unsigned offset, unsigned count, const String& replacementText);
    Position positionOutsideTabSpan(const Position&);
    void setNodeAttribute(PassRefPtr<Element>, const QualifiedName& attribute, const AtomicString& value);
    void splitElement(PassRefPtr<Element>, PassRefPtr<Node> atChild);
    void splitTextNode(PassRefPtr<Text>, unsigned offset);
    void splitTextNodeContainingElement(PassRefPtr<Text>, unsigned offset);
    void wrapContentsInDummySpan(PassRefPtr<Element>);

    void deleteInsignificantText(PassRefPtr<Text>, unsigned start, unsigned end);
    void deleteInsignificantText(const Position& start, const Position& end);
    void deleteInsignificantTextDownstream(const Position&);

    PassRefPtr<Node> appendBlockPlaceholder(PassRefPtr<Element>);
    PassRefPtr<Node> insertBlockPlaceholder(const Position&);
    PassRefPtr<Node> addBlockPlaceholderIfNeeded(Element*);
    void removePlaceholderAt(const Position&);

    PassRefPtr<Node> insertNewDefaultParagraphElementAt(const Position&);

    PassRefPtr<Node> moveParagraphContentsToNewBlockIfNecessary(const Position&);
    
    void pushAnchorElementDown(Node*);
    void pushPartiallySelectedAnchorElementsDown();
    
    void moveParagraph(const VisiblePosition&, const VisiblePosition&, const VisiblePosition&, bool preserveSelection = false, bool preserveStyle = true);
    void moveParagraphs(const VisiblePosition&, const VisiblePosition&, const VisiblePosition&, bool preserveSelection = false, bool preserveStyle = true);
    void moveParagraphWithClones(const VisiblePosition& startOfParagraphToMove, const VisiblePosition& endOfParagraphToMove, Element* blockElement, Node* outerNode);
    void cloneParagraphUnderNewElement(Position& start, Position& end, Node* outerNode, Element* blockElement);
    void cleanupAfterDeletion();
    
    bool breakOutOfEmptyListItem();
    bool breakOutOfEmptyMailBlockquotedParagraph();
    
    Position positionAvoidingSpecialElementBoundary(const Position&);
    
    PassRefPtr<Node> splitTreeToNode(Node*, Node*, bool splitAncestor = false);

    Vector<RefPtr<EditCommand> > m_commands;

private:
    virtual void doUnapply();
    virtual void doReapply();
};

} // namespace WebCore

#endif // CompositeEditCommand_h
