

#include "config.h"
#include "qwebhistory.h"
#include "qwebhistory_p.h"
#include "qwebframe_p.h"

#include "PlatformString.h"
#include "Image.h"
#include "KURL.h"
#include "Page.h"
#include "PageGroup.h"

#include <QSharedData>
#include <QDebug>

enum {
    InitialHistoryVersion = 1,
    DefaultHistoryVersion = InitialHistoryVersion
};


QWebHistoryItem::QWebHistoryItem(const QWebHistoryItem &other)
    : d(other.d)
{
}

QWebHistoryItem &QWebHistoryItem::operator=(const QWebHistoryItem &other)
{
    d = other.d;
    return *this;
}

QWebHistoryItem::~QWebHistoryItem()
{
}

QUrl QWebHistoryItem::originalUrl() const
{
    if (d->item)
        return QUrl(d->item->originalURL().string());
    return QUrl();
}


QUrl QWebHistoryItem::url() const
{
    if (d->item)
        return QUrl(d->item->url().string());
    return QUrl();
}


QString QWebHistoryItem::title() const
{
    if (d->item)
        return d->item->title();
    return QString();
}


QDateTime QWebHistoryItem::lastVisited() const
{
    //FIXME : this will be wrong unless we correctly set lastVisitedTime ourselves
    if (d->item)
        return QDateTime::fromTime_t((uint)d->item->lastVisitedTime());
    return QDateTime();
}


QIcon QWebHistoryItem::icon() const
{
    if (d->item)
        return *d->item->icon()->nativeImageForCurrentFrame();
    return QIcon();
}

QVariant QWebHistoryItem::userData() const
{
    if (d->item)
        return d->item->userData();
    return QVariant();
}

void QWebHistoryItem::setUserData(const QVariant& userData)
{
    if (d->item)
        d->item->setUserData(userData);
}

QWebHistoryItem::QWebHistoryItem(QWebHistoryItemPrivate *priv)
{
    d = priv;
}

bool QWebHistoryItem::isValid() const
{
    return d->item;
}



QWebHistory::QWebHistory()
    : d(0)
{
}

QWebHistory::~QWebHistory()
{
    delete d;
}

void QWebHistory::clear()
{
    //shortcut to private BackForwardList
    WebCore::BackForwardList* lst = d->lst;

    //clear visited links
    WebCore::Page* page = lst->page();
    if (page && page->groupPtr())
        page->groupPtr()->removeVisitedLinks();

    //if count() == 0 then just return
    if (!lst->entries().size())
        return;

    RefPtr<WebCore::HistoryItem> current = lst->currentItem();
    int capacity = lst->capacity();
    lst->setCapacity(0);

    lst->setCapacity(capacity);   //revert capacity
    lst->addItem(current.get());  //insert old current item
    lst->goToItem(current.get()); //and set it as current again

    d->page()->updateNavigationActions();
}

QList<QWebHistoryItem> QWebHistory::items() const
{
    const WebCore::HistoryItemVector &items = d->lst->entries();

    QList<QWebHistoryItem> ret;
    for (unsigned i = 0; i < items.size(); ++i) {
        QWebHistoryItemPrivate *priv = new QWebHistoryItemPrivate(items[i].get());
        ret.append(QWebHistoryItem(priv));
    }
    return ret;
}

QList<QWebHistoryItem> QWebHistory::backItems(int maxItems) const
{
    WebCore::HistoryItemVector items(maxItems);
    d->lst->backListWithLimit(maxItems, items);

    QList<QWebHistoryItem> ret;
    for (unsigned i = 0; i < items.size(); ++i) {
        QWebHistoryItemPrivate *priv = new QWebHistoryItemPrivate(items[i].get());
        ret.append(QWebHistoryItem(priv));
    }
    return ret;
}

QList<QWebHistoryItem> QWebHistory::forwardItems(int maxItems) const
{
    WebCore::HistoryItemVector items(maxItems);
    d->lst->forwardListWithLimit(maxItems, items);

    QList<QWebHistoryItem> ret;
    for (unsigned i = 0; i < items.size(); ++i) {
        QWebHistoryItemPrivate *priv = new QWebHistoryItemPrivate(items[i].get());
        ret.append(QWebHistoryItem(priv));
    }
    return ret;
}

bool QWebHistory::canGoBack() const
{
    return d->lst->backListCount() > 0;
}

bool QWebHistory::canGoForward() const
{
    return d->lst->forwardListCount() > 0;
}

void QWebHistory::back()
{
    if (canGoBack()) {
        d->lst->goBack();
        WebCore::Page* page = d->lst->page();
        page->goToItem(currentItem().d->item, WebCore::FrameLoadTypeIndexedBackForward);
    }
}

void QWebHistory::forward()
{
    if (canGoForward()) {
        d->lst->goForward();
        WebCore::Page* page = d->lst->page();
        page->goToItem(currentItem().d->item, WebCore::FrameLoadTypeIndexedBackForward);
    }
}

void QWebHistory::goToItem(const QWebHistoryItem &item)
{
    d->lst->goToItem(item.d->item);
    WebCore::Page* page = d->lst->page();
    page->goToItem(currentItem().d->item, WebCore::FrameLoadTypeIndexedBackForward);
}

QWebHistoryItem QWebHistory::backItem() const
{
    WebCore::HistoryItem *i = d->lst->backItem();
    QWebHistoryItemPrivate *priv = new QWebHistoryItemPrivate(i);
    return QWebHistoryItem(priv);
}

QWebHistoryItem QWebHistory::currentItem() const
{
    WebCore::HistoryItem *i = d->lst->currentItem();
    QWebHistoryItemPrivate *priv = new QWebHistoryItemPrivate(i);
    return QWebHistoryItem(priv);
}

QWebHistoryItem QWebHistory::forwardItem() const
{
    WebCore::HistoryItem *i = d->lst->forwardItem();
    QWebHistoryItemPrivate *priv = new QWebHistoryItemPrivate(i);
    return QWebHistoryItem(priv);
}

int QWebHistory::currentItemIndex() const
{
    return d->lst->backListCount();
}

QWebHistoryItem QWebHistory::itemAt(int i) const
{
    QWebHistoryItemPrivate *priv;
    if (i < 0 || i >= count())
        priv = new QWebHistoryItemPrivate(0);
    else {
        WebCore::HistoryItem *item = d->lst->entries()[i].get();
        priv = new QWebHistoryItemPrivate(item);
    }
    return QWebHistoryItem(priv);
}

int QWebHistory::count() const
{
    return d->lst->entries().size();
}

int QWebHistory::maximumItemCount() const
{
    return d->lst->capacity();
}

void QWebHistory::setMaximumItemCount(int count)
{
    d->lst->setCapacity(count);
}


QDataStream& operator<<(QDataStream& target, const QWebHistory& history)
{
    QWebHistoryPrivate* d = history.d;

    int version = DefaultHistoryVersion;

    target << version;
    target << history.count() << history.currentItemIndex();

    const WebCore::HistoryItemVector &items = d->lst->entries();
    for (unsigned i = 0; i < items.size(); i++)
        items[i].get()->saveState(target, version);

    return target;
}


QDataStream& operator>>(QDataStream& source, QWebHistory& history)
{
    QWebHistoryPrivate* d = history.d;

    int version;

    source >> version;

    if (version == 1) {
        int count;
        int currentIndex;
        source >> count >> currentIndex;

        history.clear();
        // only if there are elements
        if (count) {
            // after clear() is new clear HistoryItem (at the end we had to remove it)
            WebCore::HistoryItem* nullItem = d->lst->currentItem();
            for (int i = 0; i < count; i++) {
                WTF::PassRefPtr<WebCore::HistoryItem> item = WebCore::HistoryItem::create();
                item->restoreState(source, version);
                d->lst->addItem(item);
            }
            d->lst->removeItem(nullItem);
            history.goToItem(history.itemAt(currentIndex));
        }
    }

    d->page()->updateNavigationActions();

    return source;
}

QWebPagePrivate* QWebHistoryPrivate::page()
{
    return QWebFramePrivate::kit(lst->page()->mainFrame())->page()->handle();
}
