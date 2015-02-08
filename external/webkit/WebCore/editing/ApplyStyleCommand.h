

#ifndef ApplyStyleCommand_h
#define ApplyStyleCommand_h

#include "CompositeEditCommand.h"

namespace WebCore {

class CSSPrimitiveValue;
class HTMLElement;
class StyleChange;

class ApplyStyleCommand : public CompositeEditCommand {
public:
    enum EPropertyLevel { PropertyDefault, ForceBlockProperties };

    static PassRefPtr<ApplyStyleCommand> create(Document* document, CSSStyleDeclaration* style, EditAction action = EditActionChangeAttributes, EPropertyLevel level = PropertyDefault)
    {
        return adoptRef(new ApplyStyleCommand(document, style, action, level));
    }
    static PassRefPtr<ApplyStyleCommand> create(Document* document, CSSStyleDeclaration* style, const Position& start, const Position& end, EditAction action = EditActionChangeAttributes, EPropertyLevel level = PropertyDefault)
    {
        return adoptRef(new ApplyStyleCommand(document, style, start, end, action, level));
    }
    static PassRefPtr<ApplyStyleCommand> create(PassRefPtr<Element> element, bool removeOnly = false, EditAction action = EditActionChangeAttributes)
    {
        return adoptRef(new ApplyStyleCommand(element, removeOnly, action));
    }

private:
    ApplyStyleCommand(Document*, CSSStyleDeclaration*, EditAction, EPropertyLevel);
    ApplyStyleCommand(Document*, CSSStyleDeclaration*, const Position& start, const Position& end, EditAction, EPropertyLevel);
    ApplyStyleCommand(PassRefPtr<Element>, bool removeOnly, EditAction);

    virtual void doApply();
    virtual EditAction editingAction() const;

    CSSMutableStyleDeclaration* style() const { return m_style.get(); }

    // style-removal helpers
    bool shouldRemoveTextDecorationTag(CSSStyleDeclaration* styleToApply, int textDecorationAddedByTag) const;
    bool implicitlyStyledElementShouldBeRemovedWhenApplyingStyle(HTMLElement*, CSSMutableStyleDeclaration*);
    void replaceWithSpanOrRemoveIfWithoutAttributes(HTMLElement*&);
    void removeHTMLFontStyle(CSSMutableStyleDeclaration*, HTMLElement*);
    void removeHTMLBidiEmbeddingStyle(CSSMutableStyleDeclaration*, HTMLElement*);
    void removeCSSStyle(CSSMutableStyleDeclaration*, HTMLElement*);
    void removeInlineStyle(PassRefPtr<CSSMutableStyleDeclaration>, const Position& start, const Position& end);
    bool nodeFullySelected(Node*, const Position& start, const Position& end) const;
    bool nodeFullyUnselected(Node*, const Position& start, const Position& end) const;
    PassRefPtr<CSSMutableStyleDeclaration> extractTextDecorationStyle(Node*);
    PassRefPtr<CSSMutableStyleDeclaration> extractAndNegateTextDecorationStyle(Node*);
    void applyTextDecorationStyle(Node*, CSSMutableStyleDeclaration *style);
    void pushDownTextDecorationStyleAroundNode(Node*, bool forceNegate);
    void pushDownTextDecorationStyleAtBoundaries(const Position& start, const Position& end);
    
    // style-application helpers
    void applyBlockStyle(CSSMutableStyleDeclaration*);
    void applyRelativeFontStyleChange(CSSMutableStyleDeclaration*);
    void applyInlineStyle(CSSMutableStyleDeclaration*);
    void applyInlineStyleToRange(CSSMutableStyleDeclaration*, const Position& start, const Position& end);
    void addBlockStyle(const StyleChange&, HTMLElement*);
    void addInlineStyleIfNeeded(CSSMutableStyleDeclaration*, Node* start, Node* end);
    bool splitTextAtStartIfNeeded(const Position& start, const Position& end);
    bool splitTextAtEndIfNeeded(const Position& start, const Position& end);
    bool splitTextElementAtStartIfNeeded(const Position& start, const Position& end);
    bool splitTextElementAtEndIfNeeded(const Position& start, const Position& end);
    bool mergeStartWithPreviousIfIdentical(const Position& start, const Position& end);
    bool mergeEndWithNextIfIdentical(const Position& start, const Position& end);
    void cleanupUnstyledAppleStyleSpans(Node* dummySpanAncestor);

    void surroundNodeRangeWithElement(Node* start, Node* end, PassRefPtr<Element>);
    float computedFontSize(const Node*);
    void joinChildTextNodes(Node*, const Position& start, const Position& end);

    HTMLElement* splitAncestorsWithUnicodeBidi(Node*, bool before, RefPtr<CSSPrimitiveValue> allowedDirection);
    void removeEmbeddingUpToEnclosingBlock(Node* node, Node* unsplitAncestor);

    void updateStartEnd(const Position& newStart, const Position& newEnd);
    Position startPosition();
    Position endPosition();

    RefPtr<CSSMutableStyleDeclaration> m_style;
    EditAction m_editingAction;
    EPropertyLevel m_propertyLevel;
    Position m_start;
    Position m_end;
    bool m_useEndingSelection;
    RefPtr<Element> m_styledInlineElement;
    bool m_removeOnly;
};

bool isStyleSpan(const Node*);
PassRefPtr<HTMLElement> createStyleSpanElement(Document*);
RefPtr<CSSMutableStyleDeclaration> getPropertiesNotInComputedStyle(CSSStyleDeclaration* style, CSSComputedStyleDeclaration* computedStyle);

enum ShouldIncludeTypingStyle {
    IncludeTypingStyle,
    IgnoreTypingStyle
};

PassRefPtr<CSSMutableStyleDeclaration> editingStyleAtPosition(Position, ShouldIncludeTypingStyle = IgnoreTypingStyle);
void prepareEditingStyleToApplyAt(CSSMutableStyleDeclaration*, Position);
void removeStylesAddedByNode(CSSMutableStyleDeclaration*, Node*);

} // namespace WebCore

#endif
