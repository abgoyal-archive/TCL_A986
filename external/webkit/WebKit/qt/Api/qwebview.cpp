

#include "config.h"
#include "qwebview.h"

#include "Page.h"
#include "QWebPageClient.h"
#include "Settings.h"
#include "qwebframe.h"
#include "qwebpage_p.h"
#include "qbitmap.h"
#include "qevent.h"
#include "qpainter.h"
#include "qprinter.h"
#include "qdir.h"
#include "qfile.h"

class QWebViewPrivate {
public:
    QWebViewPrivate(QWebView *view)
        : view(view)
        , page(0)
        , renderHints(QPainter::TextAntialiasing)
    {
        Q_ASSERT(view);
    }

    void _q_pageDestroyed();

    QWebView *view;
    QWebPage *page;

    QPainter::RenderHints renderHints;
};

void QWebViewPrivate::_q_pageDestroyed()
{
    page = 0;
    view->setPage(0);
}

#ifdef Q_WS_MAEMO_5
#include "qabstractkineticscroller.h"
#include "qapplication.h"

// QCoreApplication::sendSpontaneousEvent() is private, hence this friend wrapper
bool qt_sendSpontaneousEvent(QObject* receiver, QEvent* ev)
{
    return QCoreApplication::sendSpontaneousEvent(receiver, ev);
}

class QWebViewKineticScroller : public QObject, public QAbstractKineticScroller {
public:
    QWebViewKineticScroller()
        : QObject()
        , QAbstractKineticScroller()
        , m_view(0)
        , m_ignoreEvents(false)
    {
    }

    void setWidget(QWebView* widget)
    {
        if (m_view) {
            m_view->removeEventFilter(this);
            QWebFrame* frame = m_view->page()->mainFrame();
            frame->setScrollBarPolicy(Qt::Vertical, m_oldVerticalScrollBarPolicy);
            frame->setScrollBarPolicy(Qt::Horizontal, m_oldHorizontalScrollBarPolicy);
        }

        m_view = widget;
        setParent(m_view);
        if (m_view) {
            QWebFrame* frame = m_view->page()->mainFrame();
            m_oldHorizontalScrollBarPolicy = frame->scrollBarPolicy(Qt::Horizontal);
            m_oldVerticalScrollBarPolicy = frame->scrollBarPolicy(Qt::Vertical);
            frame->setScrollBarPolicy(Qt::Vertical, Qt::ScrollBarAlwaysOff);
            frame->setScrollBarPolicy(Qt::Horizontal, Qt::ScrollBarAlwaysOff);
            m_view->installEventFilter(this);
        }
    }

protected:
    bool eventFilter(QObject* o, QEvent* ev)
    {
        if (!o || m_view != o || m_ignoreEvents || !m_view->isEnabled())
            return QObject::eventFilter(o, ev);

        bool res = false;

        switch (ev->type()) {
        case QEvent::MouseButtonPress: {
            // remember the frame where the button was pressed
            QWebFrame* hitFrame = scrollingFrameAt(static_cast<QMouseEvent*>(ev)->pos());
            if (hitFrame)
                m_frame = hitFrame;
            // fall through
        }
        case QEvent::MouseMove:
        case QEvent::MouseButtonRelease:
        case QEvent::MouseButtonDblClick:
            res = handleMouseEvent(static_cast<QMouseEvent*>(ev));
            break;
        default:
            break;
        }
        return res ? true : QObject::eventFilter(o, ev);
    }

    void cancelLeftMouseButtonPress(const QPoint& /* globalPressPos */)
    {
        QMouseEvent cmem(QEvent::MouseMove, QPoint(-INT_MAX, -INT_MAX), Qt::LeftButton, QApplication::mouseButtons() | Qt::LeftButton, QApplication::keyboardModifiers());
        sendEvent(m_view, &cmem);
        QMouseEvent cmer(QEvent::MouseButtonRelease, QPoint(-INT_MAX, -INT_MAX), Qt::LeftButton, QApplication::mouseButtons() & ~Qt::LeftButton, QApplication::keyboardModifiers());
        sendEvent(m_view, &cmer);
    }

    QWebFrame* currentFrame() const
    {
        if (m_frame)
            return m_frame;

        if (m_view)
            return m_view->page()->mainFrame();

        return 0;
    }

    // Returns the innermost frame at the given position that can scroll.
    QWebFrame* scrollingFrameAt(const QPoint& pos) const
    {
        QWebFrame* hitFrame = 0;
        if (m_view) {
            QWebFrame* frame = m_view->page()->mainFrame();
            hitFrame = frame->hitTestContent(pos).frame();
            QSize range = hitFrame->contentsSize() - hitFrame->geometry().size();

            while (hitFrame && range.width() <= 1 && range.height() <= 1)
                hitFrame = hitFrame->parentFrame();

            return hitFrame;
        }
    }

    QPoint maximumScrollPosition() const
    {
        QWebFrame* frame = currentFrame();
        QSize s = frame ? frame->contentsSize() - frame->geometry().size() : QSize(0, 0);
        return QPoint(qMax(0, s.width()), qMax(0, s.height()));
    }

    QPoint scrollPosition() const
    {
        QWebFrame* frame = currentFrame();
        return frame ? frame->scrollPosition() : QPoint();
    }

    QSize viewportSize() const
    {
        return m_view ? m_view->page()->viewportSize() : QSize();
    }

    void setScrollPosition(const QPoint& point, const QPoint& /* overShootDelta */)
    {
        QWebFrame* frame = currentFrame();
        if (frame)
            frame->setScrollPosition(point);
    }

    void sendEvent(QWidget* w, QEvent* ev)
    {
        m_ignoreEvents = true;
        qt_sendSpontaneousEvent(w, ev);
        m_ignoreEvents = false;
    }

    QWebView* m_view;
    bool m_ignoreEvents;
    QPointer<QWebFrame> m_frame;
    Qt::ScrollBarPolicy m_oldVerticalScrollBarPolicy;
    Qt::ScrollBarPolicy m_oldHorizontalScrollBarPolicy;
};

#endif // Q_WS_MAEMO_5



QWebView::QWebView(QWidget *parent)
    : QWidget(parent)
{
    d = new QWebViewPrivate(this);

#if !defined(Q_WS_QWS) && !defined(Q_OS_SYMBIAN)
    setAttribute(Qt::WA_InputMethodEnabled);
#endif

#if QT_VERSION >= QT_VERSION_CHECK(4, 6, 0)
    setAttribute(Qt::WA_AcceptTouchEvents);
#endif
#if defined(Q_WS_MAEMO_5)
    QAbstractKineticScroller* scroller = new QWebViewKineticScroller();
    static_cast<QWebViewKineticScroller*>(scroller)->setWidget(this);
    setProperty("kineticScroller", QVariant::fromValue(scroller));
#endif
    setAcceptDrops(true);

    setMouseTracking(true);
    setFocusPolicy(Qt::WheelFocus);
}

QWebView::~QWebView()
{
    if (d->page) {
#if QT_VERSION >= 0x040600
        d->page->d->view.clear();
#else
        d->page->d->view = 0;
#endif
        delete d->page->d->client;
        d->page->d->client = 0;
    }

    if (d->page && d->page->parent() == this)
        delete d->page;
    delete d;
}

QWebPage *QWebView::page() const
{
    if (!d->page) {
        QWebView *that = const_cast<QWebView *>(this);
        that->setPage(new QWebPage(that));
    }
    return d->page;
}

void QWebView::setPage(QWebPage* page)
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
    if (d->page) {
        d->page->setView(this);
        d->page->setPalette(palette());
        // #### connect signals
        QWebFrame *mainFrame = d->page->mainFrame();
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
                this, SIGNAL(loadFinished(bool)));
        connect(d->page, SIGNAL(statusBarMessage(QString)),
                this, SIGNAL(statusBarMessage(QString)));
        connect(d->page, SIGNAL(linkClicked(QUrl)),
                this, SIGNAL(linkClicked(QUrl)));

        connect(d->page, SIGNAL(microFocusChanged()),
                this, SLOT(updateMicroFocus()));
        connect(d->page, SIGNAL(destroyed()),
                this, SLOT(_q_pageDestroyed()));
    }
    setAttribute(Qt::WA_OpaquePaintEvent, d->page);
#if USE(ACCELERATED_COMPOSITING)
    d->page->d->page->settings()->setAcceleratedCompositingEnabled(false);
#endif
    update();
}

void QWebView::load(const QUrl &url)
{
    page()->mainFrame()->load(url);
}


void QWebView::load(const QNetworkRequest &request,
                    QNetworkAccessManager::Operation operation,
                    const QByteArray &body)
{
    page()->mainFrame()->load(request, operation, body);
}

void QWebView::setHtml(const QString &html, const QUrl &baseUrl)
{
    page()->mainFrame()->setHtml(html, baseUrl);
}

void QWebView::setContent(const QByteArray &data, const QString &mimeType, const QUrl &baseUrl)
{
    page()->mainFrame()->setContent(data, mimeType, baseUrl);
}

QWebHistory *QWebView::history() const
{
    return page()->history();
}

QWebSettings *QWebView::settings() const
{
    return page()->settings();
}

QString QWebView::title() const
{
    if (d->page)
        return d->page->mainFrame()->title();
    return QString();
}


void QWebView::setUrl(const QUrl &url)
{
    page()->mainFrame()->setUrl(url);
}

QUrl QWebView::url() const
{
    if (d->page)
        return d->page->mainFrame()->url();
    return QUrl();
}

QIcon QWebView::icon() const
{
    if (d->page)
        return d->page->mainFrame()->icon();
    return QIcon();
}

QString QWebView::selectedText() const
{
    if (d->page)
        return d->page->selectedText();
    return QString();
}

QAction *QWebView::pageAction(QWebPage::WebAction action) const
{
    return page()->action(action);
}

void QWebView::triggerPageAction(QWebPage::WebAction action, bool checked)
{
    page()->triggerAction(action, checked);
}

bool QWebView::isModified() const
{
    if (d->page)
        return d->page->isModified();
    return false;
}




QSize QWebView::sizeHint() const
{
    return QSize(800, 600); // ####...
}


void QWebView::setZoomFactor(qreal factor)
{
    page()->mainFrame()->setZoomFactor(factor);
}

qreal QWebView::zoomFactor() const
{
    return page()->mainFrame()->zoomFactor();
}


void QWebView::setTextSizeMultiplier(qreal factor)
{
    page()->mainFrame()->setTextSizeMultiplier(factor);
}

qreal QWebView::textSizeMultiplier() const
{
    return page()->mainFrame()->textSizeMultiplier();
}


QPainter::RenderHints QWebView::renderHints() const
{
    return d->renderHints;
}

void QWebView::setRenderHints(QPainter::RenderHints hints)
{
    if (hints == d->renderHints)
        return;
    d->renderHints = hints;
    update();
}

void QWebView::setRenderHint(QPainter::RenderHint hint, bool enabled)
{
    QPainter::RenderHints oldHints = d->renderHints;
    if (enabled)
        d->renderHints |= hint;
    else
        d->renderHints &= ~hint;
    if (oldHints != d->renderHints)
        update();
}


bool QWebView::findText(const QString &subString, QWebPage::FindFlags options)
{
    if (d->page)
        return d->page->findText(subString, options);
    return false;
}

bool QWebView::event(QEvent *e)
{
    if (d->page) {
#ifndef QT_NO_CONTEXTMENU
        if (e->type() == QEvent::ContextMenu) {
            if (!isEnabled())
                return false;
            QContextMenuEvent *event = static_cast<QContextMenuEvent *>(e);
            if (d->page->swallowContextMenuEvent(event)) {
                e->accept();
                return true;
            }
            d->page->updatePositionDependentActions(event->pos());
        } else
#endif // QT_NO_CONTEXTMENU
        if (e->type() == QEvent::ShortcutOverride) {
            d->page->event(e);
#ifndef QT_NO_CURSOR
        } else if (e->type() == QEvent::CursorChange) {
            // An unsetCursor will set the cursor to Qt::ArrowCursor.
            // Thus this cursor change might be a QWidget::unsetCursor()
            // If this is not the case and it came from WebCore, the
            // QWebPageClient already has set its cursor internally
            // to Qt::ArrowCursor, so updating the cursor is always
            // right, as it falls back to the last cursor set by
            // WebCore.
            // FIXME: Add a QEvent::CursorUnset or similar to Qt.
            if (cursor().shape() == Qt::ArrowCursor)
                d->page->d->client->resetCursor();
#endif
#if QT_VERSION >= QT_VERSION_CHECK(4, 6, 0)
        } else if (e->type() == QEvent::TouchBegin 
                   || e->type() == QEvent::TouchEnd 
                   || e->type() == QEvent::TouchUpdate) {
            d->page->event(e);
            if (e->isAccepted())
                return true;
#endif
        } else if (e->type() == QEvent::Leave)
            d->page->event(e);
    }

    return QWidget::event(e);
}

void QWebView::print(QPrinter *printer) const
{
#ifndef QT_NO_PRINTER
    page()->mainFrame()->print(printer);
#endif
}

void QWebView::stop()
{
    if (d->page)
        d->page->triggerAction(QWebPage::Stop);
}

void QWebView::back()
{
    if (d->page)
        d->page->triggerAction(QWebPage::Back);
}

void QWebView::forward()
{
    if (d->page)
        d->page->triggerAction(QWebPage::Forward);
}

void QWebView::reload()
{
    if (d->page)
        d->page->triggerAction(QWebPage::Reload);
}

void QWebView::resizeEvent(QResizeEvent *e)
{
    if (d->page)
        d->page->setViewportSize(e->size());
}

void QWebView::paintEvent(QPaintEvent *ev)
{
    if (!d->page)
        return;
#ifdef QWEBKIT_TIME_RENDERING
    QTime time;
    time.start();
#endif

    QWebFrame *frame = d->page->mainFrame();
    QPainter p(this);
    p.setRenderHints(d->renderHints);

    frame->render(&p, ev->region());

#ifdef    QWEBKIT_TIME_RENDERING
    int elapsed = time.elapsed();
    qDebug() << "paint event on " << ev->region() << ", took to render =  " << elapsed;
#endif
}

QWebView *QWebView::createWindow(QWebPage::WebWindowType type)
{
    Q_UNUSED(type)
    return 0;
}

void QWebView::mouseMoveEvent(QMouseEvent* ev)
{
    if (d->page) {
        const bool accepted = ev->isAccepted();
        d->page->event(ev);
        ev->setAccepted(accepted);
    }
}

void QWebView::mousePressEvent(QMouseEvent* ev)
{
    if (d->page) {
        const bool accepted = ev->isAccepted();
        d->page->event(ev);
        ev->setAccepted(accepted);
    }
}

void QWebView::mouseDoubleClickEvent(QMouseEvent* ev)
{
    if (d->page) {
        const bool accepted = ev->isAccepted();
        d->page->event(ev);
        ev->setAccepted(accepted);
    }
}

void QWebView::mouseReleaseEvent(QMouseEvent* ev)
{
    if (d->page) {
        const bool accepted = ev->isAccepted();
        d->page->event(ev);
        ev->setAccepted(accepted);
    }
}

#ifndef QT_NO_CONTEXTMENU
void QWebView::contextMenuEvent(QContextMenuEvent* ev)
{
    if (d->page) {
        const bool accepted = ev->isAccepted();
        d->page->event(ev);
        ev->setAccepted(accepted);
    }
}
#endif // QT_NO_CONTEXTMENU

#ifndef QT_NO_WHEELEVENT
void QWebView::wheelEvent(QWheelEvent* ev)
{
    if (d->page) {
        const bool accepted = ev->isAccepted();
        d->page->event(ev);
        ev->setAccepted(accepted);
    }
}
#endif // QT_NO_WHEELEVENT

void QWebView::keyPressEvent(QKeyEvent* ev)
{
    if (d->page)
        d->page->event(ev);
    if (!ev->isAccepted())
        QWidget::keyPressEvent(ev);
}

void QWebView::keyReleaseEvent(QKeyEvent* ev)
{
    if (d->page)
        d->page->event(ev);
    if (!ev->isAccepted())
        QWidget::keyReleaseEvent(ev);
}

void QWebView::focusInEvent(QFocusEvent* ev)
{
    if (d->page)
        d->page->event(ev);
    else
        QWidget::focusInEvent(ev);
}

void QWebView::focusOutEvent(QFocusEvent* ev)
{
    if (d->page)
        d->page->event(ev);
    else
        QWidget::focusOutEvent(ev);
}

void QWebView::dragEnterEvent(QDragEnterEvent* ev)
{
#ifndef QT_NO_DRAGANDDROP
    if (d->page)
        d->page->event(ev);
#endif
}

void QWebView::dragLeaveEvent(QDragLeaveEvent* ev)
{
#ifndef QT_NO_DRAGANDDROP
    if (d->page)
        d->page->event(ev);
#endif
}

void QWebView::dragMoveEvent(QDragMoveEvent* ev)
{
#ifndef QT_NO_DRAGANDDROP
    if (d->page)
        d->page->event(ev);
#endif
}

void QWebView::dropEvent(QDropEvent* ev)
{
#ifndef QT_NO_DRAGANDDROP
    if (d->page)
        d->page->event(ev);
#endif
}

bool QWebView::focusNextPrevChild(bool next)
{
    if (d->page && d->page->focusNextPrevChild(next))
        return true;
    return QWidget::focusNextPrevChild(next);
}

QVariant QWebView::inputMethodQuery(Qt::InputMethodQuery property) const
{
    if (d->page)
        return d->page->inputMethodQuery(property);
    return QVariant();
}

void QWebView::inputMethodEvent(QInputMethodEvent *e)
{
    if (d->page)
       d->page->event(e);
}

void QWebView::changeEvent(QEvent *e)
{
    if (d->page && e->type() == QEvent::PaletteChange)
        d->page->setPalette(palette());
    QWidget::changeEvent(e);
}










#include "moc_qwebview.cpp"

