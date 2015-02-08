

#ifndef visible_units_h
#define visible_units_h

#include "Document.h"
#include "TextAffinity.h"

namespace WebCore {

class VisiblePosition;

enum EWordSide { RightWordIfOnBoundary = false, LeftWordIfOnBoundary = true };

// words
VisiblePosition startOfWord(const VisiblePosition &, EWordSide = RightWordIfOnBoundary);
VisiblePosition endOfWord(const VisiblePosition &, EWordSide = RightWordIfOnBoundary);
VisiblePosition previousWordPosition(const VisiblePosition &);
VisiblePosition nextWordPosition(const VisiblePosition &);

// sentences
VisiblePosition startOfSentence(const VisiblePosition &);
VisiblePosition endOfSentence(const VisiblePosition &);
VisiblePosition previousSentencePosition(const VisiblePosition &);
VisiblePosition nextSentencePosition(const VisiblePosition &);

// lines
VisiblePosition startOfLine(const VisiblePosition &);
VisiblePosition endOfLine(const VisiblePosition &);
VisiblePosition previousLinePosition(const VisiblePosition &, int x);
VisiblePosition nextLinePosition(const VisiblePosition &, int x);
bool inSameLine(const VisiblePosition &, const VisiblePosition &);
bool inSameLogicalLine(const VisiblePosition &, const VisiblePosition &);
bool isStartOfLine(const VisiblePosition &);
bool isEndOfLine(const VisiblePosition &);
VisiblePosition logicalStartOfLine(const VisiblePosition &);
VisiblePosition logicalEndOfLine(const VisiblePosition &);

// paragraphs (perhaps a misnomer, can be divided by line break elements)
VisiblePosition startOfParagraph(const VisiblePosition&);
VisiblePosition endOfParagraph(const VisiblePosition&);
VisiblePosition startOfNextParagraph(const VisiblePosition&);
VisiblePosition previousParagraphPosition(const VisiblePosition &, int x);
VisiblePosition nextParagraphPosition(const VisiblePosition &, int x);
bool inSameParagraph(const VisiblePosition &, const VisiblePosition &);
bool isStartOfParagraph(const VisiblePosition &);
bool isEndOfParagraph(const VisiblePosition &);

// blocks (true paragraphs; line break elements don't break blocks)
VisiblePosition startOfBlock(const VisiblePosition &);
VisiblePosition endOfBlock(const VisiblePosition &);
bool inSameBlock(const VisiblePosition &, const VisiblePosition &);
bool isStartOfBlock(const VisiblePosition &);
bool isEndOfBlock(const VisiblePosition &);

// document
VisiblePosition startOfDocument(const Node*);
VisiblePosition endOfDocument(const Node*);
VisiblePosition startOfDocument(const VisiblePosition &);
VisiblePosition endOfDocument(const VisiblePosition &);
bool inSameDocument(const VisiblePosition &, const VisiblePosition &);
bool isStartOfDocument(const VisiblePosition &);
bool isEndOfDocument(const VisiblePosition &);

// editable content
VisiblePosition startOfEditableContent(const VisiblePosition&);
VisiblePosition endOfEditableContent(const VisiblePosition&);

} // namespace WebCore

#endif // VisiblePosition_h
