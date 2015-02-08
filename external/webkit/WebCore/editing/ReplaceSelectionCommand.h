

#ifndef ReplaceSelectionCommand_h
#define ReplaceSelectionCommand_h

#include "CompositeEditCommand.h"

namespace WebCore {

class DocumentFragment;
class ReplacementFragment;

class ReplaceSelectionCommand : public CompositeEditCommand {
public:
    static PassRefPtr<ReplaceSelectionCommand> create(Document* document, PassRefPtr<DocumentFragment> fragment,
        bool selectReplacement = true, bool smartReplace = false, bool matchStyle = false, bool preventNesting = true, bool movingParagraph = false,
        EditAction action = EditActionPaste)
    {
        return adoptRef(new ReplaceSelectionCommand(document, fragment, selectReplacement, smartReplace, matchStyle, preventNesting, movingParagraph, action));
    }

private:
    ReplaceSelectionCommand(Document*, PassRefPtr<DocumentFragment>,
        bool selectReplacement, bool smartReplace, bool matchStyle, bool preventNesting, bool movingParagraph, EditAction);

    virtual void doApply();
    virtual EditAction editingAction() const;

    void completeHTMLReplacement(const Position& lastPositionToSelect);

    void insertNodeAfterAndUpdateNodesInserted(PassRefPtr<Node> insertChild, Node* refChild);
    void insertNodeAtAndUpdateNodesInserted(PassRefPtr<Node>, const Position&);
    void insertNodeBeforeAndUpdateNodesInserted(PassRefPtr<Node> insertChild, Node* refChild);
    Node* insertAsListItems(PassRefPtr<Node>, Node* insertionNode, const Position&);

    void updateNodesInserted(Node*);
    bool shouldRemoveEndBR(Node*, const VisiblePosition&);
    
    bool shouldMergeStart(bool, bool, bool);
    bool shouldMergeEnd(bool selectEndWasEndOfParagraph);
    bool shouldMerge(const VisiblePosition&, const VisiblePosition&);
    
    void mergeEndIfNeeded();
    
    void removeUnrenderedTextNodesAtEnds();
    
    void negateStyleRulesThatAffectAppearance();
    void handleStyleSpans();
    void handlePasteAsQuotationNode();
    
    virtual void removeNodePreservingChildren(Node*);
    virtual void removeNodeAndPruneAncestors(Node*);
    
    VisiblePosition positionAtStartOfInsertedContent();
    VisiblePosition positionAtEndOfInsertedContent();
    
    bool performTrivialReplace(const ReplacementFragment&);

    RefPtr<Node> m_firstNodeInserted;
    RefPtr<Node> m_lastLeafInserted;
    RefPtr<CSSMutableStyleDeclaration> m_insertionStyle;
    bool m_selectReplacement;
    bool m_smartReplace;
    bool m_matchStyle;
    RefPtr<DocumentFragment> m_documentFragment;
    bool m_preventNesting;
    bool m_movingParagraph;
    EditAction m_editAction;
    bool m_shouldMergeEnd;
};

} // namespace WebCore

#endif // ReplaceSelectionCommand_h
