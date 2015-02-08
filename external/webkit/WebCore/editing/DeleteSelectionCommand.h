

#ifndef DeleteSelectionCommand_h
#define DeleteSelectionCommand_h

#include "CompositeEditCommand.h"

namespace WebCore {

class DeleteSelectionCommand : public CompositeEditCommand { 
public:
    static PassRefPtr<DeleteSelectionCommand> create(Document* document, bool smartDelete = false, bool mergeBlocksAfterDelete = true, bool replace = false, bool expandForSpecialElements = false)
    {
        return adoptRef(new DeleteSelectionCommand(document, smartDelete, mergeBlocksAfterDelete, replace, expandForSpecialElements));
    }
    static PassRefPtr<DeleteSelectionCommand> create(const VisibleSelection& selection, bool smartDelete = false, bool mergeBlocksAfterDelete = true, bool replace = false, bool expandForSpecialElements = false)
    {
        return adoptRef(new DeleteSelectionCommand(selection, smartDelete, mergeBlocksAfterDelete, replace, expandForSpecialElements));
    }

private:
    DeleteSelectionCommand(Document*, bool smartDelete, bool mergeBlocksAfterDelete, bool replace, bool expandForSpecialElements);
    DeleteSelectionCommand(const VisibleSelection&, bool smartDelete, bool mergeBlocksAfterDelete, bool replace, bool expandForSpecialElements);

    virtual void doApply();
    virtual EditAction editingAction() const;
    
    virtual bool preservesTypingStyle() const;

    void initializeStartEnd(Position&, Position&);
    void initializePositionData();
    void saveTypingStyleState();
    void insertPlaceholderForAncestorBlockContent();
    bool handleSpecialCaseBRDelete();
    void handleGeneralDelete();
    void fixupWhitespace();
    void mergeParagraphs();
    void removePreviouslySelectedEmptyTableRows();
    void calculateEndingPosition();
    void calculateTypingStyleAfterDelete();
    void clearTransientState();
    virtual void removeNode(PassRefPtr<Node>);
    virtual void deleteTextFromNode(PassRefPtr<Text>, unsigned, unsigned);

    bool m_hasSelectionToDelete;
    bool m_smartDelete;
    bool m_mergeBlocksAfterDelete;
    bool m_needPlaceholder;
    bool m_replace;
    bool m_expandForSpecialElements;
    bool m_pruneStartBlockIfNecessary;
    bool m_startsAtEmptyLine;

    // This data is transient and should be cleared at the end of the doApply function.
    VisibleSelection m_selectionToDelete;
    Position m_upstreamStart;
    Position m_downstreamStart;
    Position m_upstreamEnd;
    Position m_downstreamEnd;
    Position m_endingPosition;
    Position m_leadingWhitespace;
    Position m_trailingWhitespace;
    RefPtr<Node> m_startBlock;
    RefPtr<Node> m_endBlock;
    RefPtr<CSSMutableStyleDeclaration> m_typingStyle;
    RefPtr<CSSMutableStyleDeclaration> m_deleteIntoBlockquoteStyle;
    RefPtr<Node> m_startRoot;
    RefPtr<Node> m_endRoot;
    RefPtr<Node> m_startTableRow;
    RefPtr<Node> m_endTableRow;
    RefPtr<Node> m_temporaryPlaceholder;
};

} // namespace WebCore

#endif // DeleteSelectionCommand_h
