

#include "config.h"
#include "qgraphicswebview.h"

#include "qwebframe.h"
#include "qwebframe_p.h"
#include "qwebpage.h"
#include "qwebpage_p.h"
#include "QWebPageClient.h"
#include <FrameView.h>
#include <QtCore/qmetaobject.h>
#include <QtCore/qsharedpointer.h>
#include <QtCore/qtimer.h>
#include <QtGui/qapplication.h>
#include <QtGui/qgraphicsscene.h>
#include <QtGui/qgraphicssceneevent.h>
#include <QtGui/qgraphicsview.h>
#include <QtGui/qpixmapcache.h>
#include <QtGui/qstyleoption.h>
#if defined(Q_WS_X11)
#include <QX11Info>
#endif
#include <Settings.h>

#if USE(ACCELERATED_COMPOSITING)

// the overlay is here for one reason only: to have the scroll-bars and other
// extra UI elements appear on top of any QGraphicsItems created by CSS compositing layers
class QGraphicsWebViewOverlay : public QGraphicsItem {
    public:
    QGraphicsWebViewOverlay(QGraphicsWebView* view)
            :QGraphicsItem(view)
            , q(view)
    {
        setPos(0, 0);
        setFlag(QGraphicsItem::ItemUsesExtendedStyleOption, true);
        setCacheMode(QGraphicsItem::DeviceCoordinateCache);
    }

    QRectF boundingRect() const
    {
        return q->boundingRect();
    }

    void paint(QPainter* painter, const QStyleOptionGraphicsItem* options, QWidget*)
    {
        q->page()->mainFrame()->render(painter, static_cast<QWebFrame::RenderLayer>(QWebFrame::AllLayers&(~QWebFrame::ContentsLayer)), options->exposedRect.toRect());
    }

    friend class QGraphicsWebView;
    QGraphicsWebView* q;
};

#endif

class QGraphicsWebViewPrivate : public QWebPageClient {
public:
    QGraphicsWebViewPrivate(QGraphicsWebView* parent)
        : q(parent)
        , page(0)
#if USE(ACCELERATED_COMPOSITING)
        , rootGraphicsLayer(0)
        , shouldSync(false)
#endif
    {
#if USE(ACCELERATED_COMPOSITING)
        // the overlay and stays alive for the lifetime of
        // this QGraphicsWebView as the scrollbars are needed when there's no compositing
        q->setFlag(QGraphicsItem::ItemUsesExtendedStyleOption);
        syncMetaMethod = q->metaObject()->method(q->metaObject()->indexOfMethod("syncLayers()"));
#endif
    }

    virtual ~QGraphicsWebViewPrivate();
    virtual void scroll(int dx, int dy, const QRect&);
    virtual void update(const QRect& dirtyRect);
    virtual void setInputMethodEnabled(bool enable);
    virtual bool inputMethodEnabled() const;
#if QT_VERSION >= 0x040600
    virtual void setInputMethodHint(Qt::InputMethodHint hint, bool enable);
#endif

#ifndef QT_NO_CURSOR
    virtual QCursor cursor() const;
    virtual void updateCursor(const QCursor& cursor);
#endif

    virtual QPalette palette() const;
    virtual int screenNumber() const;
    virtual QWidget* ownerWidget() const;

    virtual QObject* pluginParent() const;

    virtual QStyle* style() const;

#if USE(ACCELERATED_COMPOSITING)
    virtual void setRootGraphicsLayer(QGraphicsItem* layer);
    virtual void markForSync(bool scheduleSync);
    void updateCompositingScrollPosition();
#endif

    void syncLayers();
    void _q_doLoadFinished(bool success);

    QGraphicsWebView* q;
    QWebPage* page;
#if USE(ACCELERATED_COMPOSITING)
    QGraphicsItem* rootGraphicsLayer;

    // the overlay gets instantiated when the root layer is attached, and get deleted when it's detached
    QSharedPointer<QGraphicsWebViewOverlay> overlay;

    // we need to sync the layers if we get a special call from the WebCore
    // compositor telling us to do so. We'll get that call from ChromeClientQt
    bool shouldSync;

    // we have to flush quite often, so we use a meta-method instead of QTimer::singleShot for putting the event in the queue
    QMetaMethod syncMetaMethod;

    // we need to put the root graphics layer behind the overlay (which contains the scrollbar)
    enum { RootGraphicsLayerZValue, OverlayZValue };
#endif
};

QGraphicsWebViewPrivate::~QGraphicsWebViewPrivate()
{
#if USE(ACCELERATED_COMPOSITING)
    if (rootGraphicsLayer) {
        // we don't need to delete the root graphics layer
        // The lifecycle is managed in GraphicsLayerQt.cpp
        rootGraphicsLayer->setParentItem(0);
        q->scene()->removeItem(rootGraphicsLayer);
    }
#endif
}

#if USE(ACCELERATED_COMPOSITING)
void QGraphicsWebViewPrivate::setRootGraphicsLayer(QGraphicsItem* layer)
{
    if (rootGraphicsLayer) {
        rootGraphicsLayer->setParentItem(0);
        q->scene()->removeItem(rootGraphicsLayer);
        QWebFramePrivate::core(q->page()->mainFrame())->view()->syncCompositingStateRecursive();
    }

    rootGraphicsLayer = layer;

    if (layer) {
        layer->setFlag(QGraphicsItem::ItemClipsChildrenToShape, true);
        layer->setParentItem(q);
        layer->setZValue(RootGraphicsLayerZValue);
        if (!overlay) {
            overlay = QSharedPointer<QGraphicsWebViewOverlay>(new QGraphicsWebViewOverlay(q));
            overlay->setZValue(OverlayZValue);
        }
        updateCompositingScrollPosition();
    } else {
        // we don't have compositing layers, we can render the scrollbars and content in one go
        overlay.clear();
    }
}

void QGraphicsWebViewPrivate::markForSync(bool scheduleSync)
{
    shouldSync = true;
    if (scheduleSync)
        syncMetaMethod.invoke(q, Qt::QueuedConnection);
}

void QGraphicsWebViewPrivate::updateCompositingScrollPosition()
{
    if (rootGraphicsLayer && q->page() && q->page()->mainFrame()) {
        const QPoint scrollPosition = q->page()->mainFrame()->scrollPosition();
        rootGraphicsLayer->setPos(-scrollPosition);
    }
}

#endif

void QGraphicsWebViewPrivate::syncLayers()
{
#if USE(ACCELERATED_COMPOSITING)
    if (shouldSync) {
        QWebFramePrivate::core(q->page()->mainFrame())->view()->syncCompositingStateRecursive();
        shouldSync = false;
    }
#endif
}

void QGraphicsWebViewPrivate::_q_doLoadFinished(bool success)
{
    // If the page had no title, still make sure it gets the signal
    if (q->title().isEmpty())
        emit q->urlChanged(q->url());

    emit q->loadFinished(success);
}

void QGraphicsWebViewPrivate::scroll(int dx, int dy, const QRect& rectToScroll)
{
    q->scroll(qreal(dx), qreal(dy), QRectF(rectToScroll));
#if USE(ACCELERATED_COMPOSITING)
    updateCompositingScrollPosition();
#endif
}

void QGraphicsWebViewPrivate::update(const QRect & dirtyRect)
{
    q->update(QRectF(dirtyRect));
#if USE(ACCELERATED_COMPOSITING)
    if (overlay)
        overlay->update(QRectF(dirtyRect));
    syncLayers();
#endif
}


void QGraphicsWebViewPrivate::setInputMethodEnabled(bool enable)
{
#if QT_VERSION >= QT_VERSION_CHECK(4, 6, 0)
    q->setFlag(QGraphicsItem::ItemAcceptsInputMethod, enable);
#endif
}

bool QGraphicsWebViewPrivate::inputMethodEnabled() const
{
#if QT_VERSION >= QT_VERSION_CHECK(4, 6, 0)
    return q->flags() & QGraphicsItem::ItemAcceptsInputMethod;
#else
    return false;
#endif
}

#if QT_VERSION >= 0x040600
void QGraphicsWebViewPrivate::setInputMethodHint(Qt::InputMethodHint hint, bool enable)
{
    if (enable)
        q->setInputMethodHints(q->inputMethodHints() | hint);
    else
        q->setInputMethodHints(q->inputMethodHints() & ~hint);
}
#endif
#ifndef QT_NO_CURSOR
QCursor QGraphicsWebViewPrivate::cursor() const
{
    return q->cursor();
}

void QGraphicsWebViewPrivate::updateCursor(const QCursor& cursor)
{
    q->setCursor(cursor);
}
#endif

QPalette QGraphicsWebViewPrivate::palette() const
{
    return q->palette();
}

int QGraphicsWebViewPrivate::screenNumber() const
{
#if defined(Q_WS_X11)
    const QList<QGraphicsView*> views = q->scene()->views();

    if (!views.isEmpty())
        return views.at(0)->x11Info().screen();
#endif

    return 0;
}

QWidget* QGraphicsWebViewPrivate::ownerWidget() const
{
    const QList<QGraphicsView*> views = q->scene()->views();
    return views.value(0);
}

QObject* QGraphicsWebViewPrivate::pluginParent() const
{
    return q;
}

QStyle* QGraphicsWebViewPrivate::style() const
{
    return q->style();
}







QGraphicsWebView::QGraphicsWebView(QGraphicsItem* parent)
    : QGraphicsWidget(parent)
    , d(new QGraphicsWebViewPrivate(this))
{
#if QT_VERSION >= 0x040600
    setFlag(QGraphicsItem::ItemUsesExtendedStyleOption, true);
#endif
    setAcceptDrops(true);
    setAcceptHoverEvents(true);
#if QT_VERSION >= QT_VERSION_CHECK(4, 6, 0)
    setAcceptTouchEvents(true);
#endif
    setFocusPolicy(Qt::StrongFocus);
    setFlag(QGraphicsItem::ItemClipsChildrenToShape, true);
}

QGraphicsWebView::~QGraphicsWebView()
{
    if (d->page) {
#if QT_VERSION >= 0x040600
        d->page->d->view.clear();
#else
        d->page->d->view = 0;
#endif
        d->page->d->client = 0; // unset the page client
    }

    if (d->page && d->page->parent() == this)
        delete d->page;

    delete d;
}

QWebPage* QGraphicsWebView::page() const
{
    if (!d->page) {
        QGraphicsWebView* that = const_cast<QGraphicsWebView*>(this);
        QWebPage* page = new QWebPage(that);

        // Default to not having a background, in the case
        // the page doesn't provide one.
        QPalette palette = QApplication::palette();
        palette.setBrush(QPalette::Base, QColor::fromRgbF(0, 0, 0, 0));
        page->setPalette(palette);

        that->setPage(page);
    }

    return d->page;
}

void QGraphicsWebView::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget*)
{
#if USE(ACCELERATED_COMPOSITING)
    page()->mainFrame()->render(painter, d->overlay ? QWebFrame::ContentsLayer : QWebFrame::AllLayers, option->exposedRect.toAlignedRect());
#else
    page()->mainFrame()->render(painter, QWebFrame::AllLayers, option->exposedRect.toRect());
#endif
}

bool QGraphicsWebView::sceneEvent(QEvent* event)
{
    // Re-implemented in order to allows fixing event-related bugs in patch releases.

#if QT_VERSION >= QT_VERSION_CHECK(4, 6, 0)
    if (d->page && (event->type() == QEvent::TouchBegin
                || event->type() == QEvent::TouchEnd
                || event->type() == QEvent::TouchUpdate)) {
        d->page->event(event);
        if (event->isAccepted())
            return true;
    }
#endif

    return QGraphicsWidget::sceneEvent(event);
}

QVariant QGraphicsWebView::itemChange(GraphicsItemChange change, const QVariant& value)
{
    switch (change) {
    // Differently from QWebView, it is interesting to QGraphicsWebView to handle
    // post mouse cursor change notifications. Reason: 'ItemCursorChange' is sent
    // as the first action in QGraphicsItem::setCursor implementation, and at that
    // item widget's cursor has not been effectively changed yet.
    // After cursor is properly set (at 'ItemCursorHasChanged' emission time), we
    // fire 'CursorChange'.
    case ItemCursorChange:
        return value;
    case ItemCursorHasChanged:
        QEvent event(QEvent::CursorChange);
        QApplication::sendEvent(this, &event);
        return value;
    }

    return QGraphicsWidget::itemChange(change, value);
}

QSizeF QGraphicsWebView::sizeHint(Qt::SizeHint which, const QSizeF& constraint) const
{
    if (which == Qt::PreferredSize)
        return QSizeF(800, 600); // ###
    return QGraphicsWidget::sizeHint(which, constraint);
}

QVariant QGraphicsWebView::inputMethodQuery(Qt::InputMethodQuery query) const
{
    if (d->page)
        return d->page->inputMethodQuery(query);
    return QVariant();
}

bool QGraphicsWebView::event(QEvent* event)
{
    // Re-implemented in order to allows fixing event-related bugs in patch releases.

    if (d->page) {
#ifndef QT_NO_CONTEXTMENU
        if (event->type() == QEvent::GraphicsSceneContextMenu) {
            if (!isEnabled())
                return false;

            QGraphicsSceneContextMenuEvent* ev = static_cast<QGraphicsSceneContextMenuEvent*>(event);
            QContextMenuEvent fakeEvent(QContextMenuEvent::Reason(ev->reason()), ev->pos().toPoint());
            if (d->page->swallowContextMenuEvent(&fakeEvent)) {
                event->accept();
                return true;
            }
            d->page->updatePositionDependentActions(fakeEvent.pos());
        } else
#endif // QT_NO_CONTEXTMENU
        {
#ifndef QT_NO_CURSOR
            if (event->type() == QEvent::CursorChange) {
                // An unsetCursor will set the cursor to Qt::ArrowCursor.
                // Thus this cursor change might be a QWidget::unsetCursor()
                // If this is not the case and it came from WebCore, the
                // QWebPageClient already has set its cursor internally
                // to Qt::ArrowCursor, so updating the cursor is always
                // right, as it falls back to the last cursor set by
                // WebCore.
                // FIXME: Add a QEvent::CursorUnset or similar to Qt.
                if (cursor().shape() == Qt::ArrowCursor)
                    d->resetCursor();
            }
#endif
        }
    }
    return QGraphicsWidget::event(event);
}

void QGraphicsWebView::setPage(QWebPage* page)
{
    if (d->page == page)
        return;

    if (d->page) {
        d->page->d->client = 0; // unset the page client
        if (d->page->parent() == this)
            delete d->page;
        else
            d->page->disconnect(this);
    }

    d->page = page;
    if (!d->page)
        return;
#if USE(ACCELERATED_COMPOSITING)
    if (d->overlay)
        d->overlay->prepareGeometryChange();
#endif
    d->page->d->client = d; // set the page client

    QSize size = geometry().size().toSize();
    page->setViewportSize(size);

    QWebFrame* mainFrame = d->page->mainFrame();

    connect(mainFrame, SIGNAL(titleChanged(QString)),
            this, SIGNAL(titleChanged(QString)));
    connect(mainFrame, SIGNAL(iconChanged()),
            this, SIGNAL(iconChanged()));
    connect(mainFrame, SIGNAL(urlChanged(QUrl)),
            this, SIGNAL(urlChanged(QUrl)));
    connect(d->page, SIGNAL(loadStarted()),
            this, SIGNAL(loadStarted()));
    connect(d->page, SIGNAL(loadProgress(int)),
            this, SIGNAL(loadProgress(int)));
    connect(d->page, SIGNAL(loadFinished(bool)),
            this, SLOT(_q_doLoadFinished(bool)));
    connect(d->page, SIGNAL(statusBarMessage(QString)),
            this, SIGNAL(statusBarMessage(QString)));
    connect(d->page, SIGNAL(linkClicked(QUrl)),
            this, SIGNAL(linkClicked(QUrl)));
}


void QGraphicsWebView::setUrl(const QUrl &url)
{
    page()->mainFrame()->setUrl(url);
}

QUrl QGraphicsWebView::url() const
{
    if (d->page)
        return d->page->mainFrame()->url();

    return QUrl();
}

QString QGraphicsWebView::title() const
{
    if (d->page)
        return d->page->mainFrame()->title();

    return QString();
}

QIcon QGraphicsWebView::icon() const
{
    if (d->page)
        return d->page->mainFrame()->icon();

    return QIcon();
}


void QGraphicsWebView::setZoomFactor(qreal factor)
{
    if (factor == page()->mainFrame()->zoomFactor())
        return;

    page()->mainFrame()->setZoomFactor(factor);
}

qreal QGraphicsWebView::zoomFactor() const
{
    return page()->mainFrame()->zoomFactor();
}

void QGraphicsWebView::updateGeometry()
{

#if USE(ACCELERATED_COMPOSITING)
    if (d->overlay)
        d->overlay->prepareGeometryChange();
#endif

    QGraphicsWidget::updateGeometry();

    if (!d->page)
        return;

    QSize size = geometry().size().toSize();
    d->page->setViewportSize(size);
}

void QGraphicsWebView::setGeometry(const QRectF& rect)
{
    QGraphicsWidget::setGeometry(rect);

#if USE(ACCELERATED_COMPOSITING)
    if (d->overlay)
        d->overlay->prepareGeometryChange();
#endif

    if (!d->page)
        return;

    // NOTE: call geometry() as setGeometry ensures that
    // the geometry is within legal bounds (minimumSize, maximumSize)
    QSize size = geometry().size().toSize();
    d->page->setViewportSize(size);
}

void QGraphicsWebView::stop()
{
    if (d->page)
        d->page->triggerAction(QWebPage::Stop);
}

void QGraphicsWebView::back()
{
    if (d->page)
        d->page->triggerAction(QWebPage::Back);
}

void QGraphicsWebView::forward()
{
    if (d->page)
        d->page->triggerAction(QWebPage::Forward);
}

void QGraphicsWebView::reload()
{
    if (d->page)
        d->page->triggerAction(QWebPage::Reload);
}

void QGraphicsWebView::load(const QUrl& url)
{
    page()->mainFrame()->load(url);
}


void QGraphicsWebView::load(const QNetworkRequest& request,
                    QNetworkAccessManager::Operation operation,
                    const QByteArray& body)
{
    page()->mainFrame()->load(request, operation, body);
}

void QGraphicsWebView::setHtml(const QString& html, const QUrl& baseUrl)
{
    page()->mainFrame()->setHtml(html, baseUrl);
}

void QGraphicsWebView::setContent(const QByteArray& data, const QString& mimeType, const QUrl& baseUrl)
{
    page()->mainFrame()->setContent(data, mimeType, baseUrl);
}

QWebHistory* QGraphicsWebView::history() const
{
    return page()->history();
}

bool QGraphicsWebView::isModified() const
{
    if (d->page)
        return d->page->isModified();
    return false;
}

QWebSettings* QGraphicsWebView::settings() const
{
    return page()->settings();
}

QAction *QGraphicsWebView::pageAction(QWebPage::WebAction action) const
{
    return page()->action(action);
}

void QGraphicsWebView::triggerPageAction(QWebPage::WebAction action, bool checked)
{
    page()->triggerAction(action, checked);
}

bool QGraphicsWebView::findText(const QString &subString, QWebPage::FindFlags options)
{
    if (d->page)
        return d->page->findText(subString, options);
    return false;
}

void QGraphicsWebView::hoverMoveEvent(QGraphicsSceneHoverEvent* ev)
{
    if (d->page) {
        const bool accepted = ev->isAccepted();
        QMouseEvent me = QMouseEvent(QEvent::MouseMove,
                ev->pos().toPoint(), Qt::NoButton,
                Qt::NoButton, Qt::NoModifier);
        d->page->event(&me);
        ev->setAccepted(accepted);
    }

    if (!ev->isAccepted())
        QGraphicsItem::hoverMoveEvent(ev);
}

void QGraphicsWebView::hoverLeaveEvent(QGraphicsSceneHoverEvent* ev)
{
    Q_UNUSED(ev);
}

void QGraphicsWebView::mouseMoveEvent(QGraphicsSceneMouseEvent* ev)
{
    if (d->page) {
        const bool accepted = ev->isAccepted();
        d->page->event(ev);
        ev->setAccepted(accepted);
    }

    if (!ev->isAccepted())
        QGraphicsItem::mouseMoveEvent(ev);
}

void QGraphicsWebView::mousePressEvent(QGraphicsSceneMouseEvent* ev)
{
    if (d->page) {
        const bool accepted = ev->isAccepted();
        d->page->event(ev);
        ev->setAccepted(accepted);
    }

    if (!ev->isAccepted())
        QGraphicsItem::mousePressEvent(ev);
}

void QGraphicsWebView::mouseReleaseEvent(QGraphicsSceneMouseEvent* ev)
{
    if (d->page) {
        const bool accepted = ev->isAccepted();
        d->page->event(ev);
        ev->setAccepted(accepted);
    }

    if (!ev->isAccepted())
        QGraphicsItem::mouseReleaseEvent(ev);
}

void QGraphicsWebView::mouseDoubleClickEvent(QGraphicsSceneMouseEvent* ev)
{
    if (d->page) {
        const bool accepted = ev->isAccepted();
        d->page->event(ev);
        ev->setAccepted(accepted);
    }

    if (!ev->isAccepted())
        QGraphicsItem::mouseDoubleClickEvent(ev);
}

void QGraphicsWebView::keyPressEvent(QKeyEvent* ev)
{
    if (d->page)
        d->page->event(ev);

    if (!ev->isAccepted())
        QGraphicsItem::keyPressEvent(ev);
}

void QGraphicsWebView::keyReleaseEvent(QKeyEvent* ev)
{
    if (d->page)
        d->page->event(ev);

    if (!ev->isAccepted())
        QGraphicsItem::keyReleaseEvent(ev);
}

void QGraphicsWebView::focusInEvent(QFocusEvent* ev)
{
    if (d->page)
        d->page->event(ev);
    else
        QGraphicsItem::focusInEvent(ev);
}

void QGraphicsWebView::focusOutEvent(QFocusEvent* ev)
{
    if (d->page)
        d->page->event(ev);
    else
        QGraphicsItem::focusOutEvent(ev);
}

bool QGraphicsWebView::focusNextPrevChild(bool next)
{
    if (d->page)
        return d->page->focusNextPrevChild(next);

    return QGraphicsWidget::focusNextPrevChild(next);
}

void QGraphicsWebView::dragEnterEvent(QGraphicsSceneDragDropEvent* ev)
{
#ifndef QT_NO_DRAGANDDROP
    if (d->page)
        d->page->event(ev);
#else
    Q_UNUSED(ev);
#endif
}

void QGraphicsWebView::dragLeaveEvent(QGraphicsSceneDragDropEvent* ev)
{
#ifndef QT_NO_DRAGANDDROP
    if (d->page) {
        const bool accepted = ev->isAccepted();
        d->page->event(ev);
        ev->setAccepted(accepted);
    }

    if (!ev->isAccepted())
        QGraphicsWidget::dragLeaveEvent(ev);
#else
    Q_UNUSED(ev);
#endif
}

void QGraphicsWebView::dragMoveEvent(QGraphicsSceneDragDropEvent* ev)
{
#ifndef QT_NO_DRAGANDDROP
    if (d->page) {
        const bool accepted = ev->isAccepted();
        d->page->event(ev);
        ev->setAccepted(accepted);
    }

    if (!ev->isAccepted())
        QGraphicsWidget::dragMoveEvent(ev);
#else
    Q_UNUSED(ev);
#endif
}

void QGraphicsWebView::dropEvent(QGraphicsSceneDragDropEvent* ev)
{
#ifndef QT_NO_DRAGANDDROP
    if (d->page) {
        const bool accepted = ev->isAccepted();
        d->page->event(ev);
        ev->setAccepted(accepted);
    }

    if (!ev->isAccepted())
        QGraphicsWidget::dropEvent(ev);
#else
    Q_UNUSED(ev);
#endif
}

#ifndef QT_NO_CONTEXTMENU
void QGraphicsWebView::contextMenuEvent(QGraphicsSceneContextMenuEvent* ev)
{
    if (d->page) {
        const bool accepted = ev->isAccepted();
        d->page->event(ev);
        ev->setAccepted(accepted);
    }
}
#endif // QT_NO_CONTEXTMENU

#ifndef QT_NO_WHEELEVENT
void QGraphicsWebView::wheelEvent(QGraphicsSceneWheelEvent* ev)
{
    if (d->page) {
        const bool accepted = ev->isAccepted();
        d->page->event(ev);
        ev->setAccepted(accepted);
    }

    if (!ev->isAccepted())
        QGraphicsItem::wheelEvent(ev);
}
#endif // QT_NO_WHEELEVENT

void QGraphicsWebView::inputMethodEvent(QInputMethodEvent* ev)
{
    if (d->page)
        d->page->event(ev);

    if (!ev->isAccepted())
        QGraphicsItem::inputMethodEvent(ev);
}




#include "moc_qgraphicswebview.cpp"
