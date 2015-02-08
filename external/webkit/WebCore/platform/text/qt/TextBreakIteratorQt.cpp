

#include "config.h"
#include "TextBreakIterator.h"

#include <QtCore/qtextboundaryfinder.h>
#include <qdebug.h>

// #define DEBUG_TEXT_ITERATORS
#ifdef DEBUG_TEXT_ITERATORS
#define DEBUG qDebug
#else
#define DEBUG if (1) {} else qDebug
#endif

namespace WebCore {

    class TextBreakIterator : public QTextBoundaryFinder {
    };
    static QTextBoundaryFinder* iterator = 0;
    static unsigned char buffer[1024];

    TextBreakIterator* wordBreakIterator(const UChar* string, int length)
    {
        if (!string)
            return 0;
        if (!iterator)
            iterator = new QTextBoundaryFinder;

        *iterator = QTextBoundaryFinder(QTextBoundaryFinder::Word, (const QChar *)string, length, buffer, sizeof(buffer));
        return static_cast<TextBreakIterator*>(iterator);
    }

    TextBreakIterator* characterBreakIterator(const UChar* string, int length)
    {
        if (!string)
            return 0;
        if (!iterator)
            iterator = new QTextBoundaryFinder;

        *iterator = QTextBoundaryFinder(QTextBoundaryFinder::Grapheme, (const QChar *)string, length, buffer, sizeof(buffer));
        return static_cast<TextBreakIterator*>(iterator);
    }

    TextBreakIterator* cursorMovementIterator(const UChar* string, int length)
    {
        return characterBreakIterator(string, length);
    }

    TextBreakIterator* lineBreakIterator(const UChar* string, int length)
    {
        static QTextBoundaryFinder *iterator = 0;
        if (!string)
            return 0;
        if (!iterator)
            iterator = new QTextBoundaryFinder;

        *iterator = QTextBoundaryFinder(QTextBoundaryFinder::Line, (const QChar *)string, length, buffer, sizeof(buffer));
        return static_cast<TextBreakIterator*>(iterator);
    }

    TextBreakIterator* sentenceBreakIterator(const UChar* string, int length)
    {
        if (!string)
            return 0;
        if (!iterator)
            iterator = new QTextBoundaryFinder;

        *iterator = QTextBoundaryFinder(QTextBoundaryFinder::Sentence, (const QChar *)string, length, buffer, sizeof(buffer));
        return static_cast<TextBreakIterator*>(iterator);
    }

    int textBreakFirst(TextBreakIterator* bi)
    {
        bi->toStart();
        DEBUG() << "textBreakFirst" << bi->position();
        return bi->position();
    }

    int textBreakNext(TextBreakIterator* bi)
    {
        int pos = bi->toNextBoundary();
        DEBUG() << "textBreakNext" << pos;
        return pos;
    }

    int textBreakPreceding(TextBreakIterator* bi, int pos)
    {
        bi->setPosition(pos);
        int newpos = bi->toPreviousBoundary();
        DEBUG() << "textBreakPreceding" << pos << newpos;
        return newpos;
    }

    int textBreakFollowing(TextBreakIterator* bi, int pos)
    {
        bi->setPosition(pos);
        int newpos = bi->toNextBoundary();
        DEBUG() << "textBreakFollowing" << pos << newpos;
        return newpos;
    }

    int textBreakCurrent(TextBreakIterator* bi)
    {
        return bi->position();
    }

    bool isTextBreak(TextBreakIterator*, int)
    {
        return true;
    }

}
