

#ifndef QWEBHISTORY_P_H
#define QWEBHISTORY_P_H

#include "BackForwardList.h"
#include "HistoryItem.h"
#include <QtCore/qglobal.h>
#include <QtCore/qshareddata.h>

class QWebPagePrivate;

class Q_AUTOTEST_EXPORT QWebHistoryItemPrivate : public QSharedData {
public:
    static QExplicitlySharedDataPointer<QWebHistoryItemPrivate> get(QWebHistoryItem* q)
    {
        return q->d;
    }
    QWebHistoryItemPrivate(WebCore::HistoryItem* i)
    {
        if (i)
            i->ref();
        item = i;
    }
    ~QWebHistoryItemPrivate()
    {
        if (item)
            item->deref();
    }

    WebCore::HistoryItem* item;
};

class QWebHistoryPrivate : public QSharedData {
public:
    QWebHistoryPrivate(WebCore::BackForwardList* l)
    {
        l->ref();
        lst = l;
    }
    ~QWebHistoryPrivate()
    {
        lst->deref();
    }

    QWebPagePrivate* page();

    WebCore::BackForwardList* lst;
};


#endif
