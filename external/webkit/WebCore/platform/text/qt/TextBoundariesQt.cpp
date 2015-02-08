

#include "config.h"

#include "TextBoundaries.h"
#include "NotImplemented.h"

#include <QString>
#include <QChar>

#include <QDebug>
#include <stdio.h>

#include <qtextboundaryfinder.h>

namespace WebCore {

int findNextWordFromIndex(UChar const* buffer, int len, int position, bool forward)
{
    QString str(reinterpret_cast<QChar const*>(buffer), len);
    QTextBoundaryFinder iterator(QTextBoundaryFinder::Word, str);
    iterator.setPosition(position >= len ? len - 1 : position);
    if (forward) {
        int pos = iterator.toNextBoundary();
        while (pos > 0) {
            if (QChar(buffer[pos-1]).isLetterOrNumber())
                return pos;
            pos = iterator.toNextBoundary();
        }
        return len;
    } else {
        int pos = iterator.toPreviousBoundary();
        while (pos > 0) {
            if (QChar(buffer[pos]).isLetterOrNumber())
                return pos;
            pos = iterator.toPreviousBoundary();
        }
        return 0;
    }
}

void findWordBoundary(UChar const* buffer, int len, int position, int* start, int* end)
{
    QString str(reinterpret_cast<QChar const*>(buffer), len);
    QTextBoundaryFinder iterator(QTextBoundaryFinder::Word, str);
    iterator.setPosition(position);
    *start = position > 0 ? iterator.toPreviousBoundary() : 0;
    *end = position == len ? len : iterator.toNextBoundary();
}

}

