

#include "config.h"
#include "qwebframe.h"

#include "Bridge.h"
#include "CallFrame.h"
#include "Document.h"
#include "DocumentLoader.h"
#include "DragData.h"
#include "Element.h"
#include "FocusController.h"
#include "Frame.h"
#include "FrameLoaderClientQt.h"
#include "FrameTree.h"
#include "FrameView.h"
#include "GCController.h"
#include "GraphicsContext.h"
#include "HTMLMetaElement.h"
#include "HitTestResult.h"
#include "IconDatabase.h"
#include "InspectorController.h"
#include "JSDOMBinding.h"
#include "JSDOMWindowBase.h"
#include "JSLock.h"
#include "JSObject.h"
#include "NodeList.h"
#include "Page.h"
#include "PlatformMouseEvent.h"
#include "PlatformWheelEvent.h"
#include "PrintContext.h"
#include "PutPropertySlot.h"
#include "RenderTreeAsText.h"
#include "RenderView.h"
#include "ResourceRequest.h"
#include "ScriptController.h"
#include "ScriptSourceCode.h"
#include "ScriptValue.h"
#include "Scrollbar.h"
#include "SelectionController.h"
#include "SubstituteData.h"
#include "SVGSMILElement.h"
#include "htmlediting.h"
#include "markup.h"
#include "qt_instance.h"
#include "qt_runtime.h"
#include "qwebelement.h"
#include "qwebframe_p.h"
#include "qwebpage.h"
#include "qwebpage_p.h"
#include "qwebsecurityorigin.h"
#include "qwebsecurityorigin_p.h"
#include "runtime_object.h"
#include "runtime_root.h"
#include "wtf/HashMap.h"
#include <QMultiMap>
#include <qdebug.h>
#include <qevent.h>
#include <qfileinfo.h>
#include <qpainter.h>
#include <qprinter.h>
#include <qregion.h>
#include <qnetworkrequest.h>

using namespace WebCore;

// from text/qfont.cpp
QT_BEGIN_NAMESPACE
extern Q_GUI_EXPORT int qt_defaultDpi();
QT_END_NAMESPACE

bool QWEBKIT_EXPORT qt_drt_hasDocumentElement(QWebFrame* qframe)
{
    return QWebFramePrivate::core(qframe)->document()->documentElement();
}

void QWEBKIT_EXPORT qt_drt_setJavaScriptProfilingEnabled(QWebFrame* qframe, bool enabled)
{
#if ENABLE(JAVASCRIPT_DEBUGGER)
    Frame* frame = QWebFramePrivate::core(qframe);
    InspectorController* controller = frame->page()->inspectorController();
    if (!controller)
        return;
    if (enabled)
        controller->enableProfiler();
    else
        controller->disableProfiler();
#endif
}

// Pause a given CSS animation or transition on the target node at a specific time.
// If the animation or transition is already paused, it will update its pause time.
// This method is only intended to be used for testing the CSS animation and transition system.
bool QWEBKIT_EXPORT qt_drt_pauseAnimation(QWebFrame *qframe, const QString &animationName, double time, const QString &elementId)
{
    Frame* frame = QWebFramePrivate::core(qframe);
    if (!frame)
        return false;

    AnimationController* controller = frame->animation();
    if (!controller)
        return false;

    Document* doc = frame->document();
    Q_ASSERT(doc);

    Node* coreNode = doc->getElementById(elementId);
    if (!coreNode || !coreNode->renderer())
        return false;

    return controller->pauseAnimationAtTime(coreNode->renderer(), animationName, time);
}

bool QWEBKIT_EXPORT qt_drt_pauseTransitionOfProperty(QWebFrame *qframe, const QString &propertyName, double time, const QString &elementId)
{
    Frame* frame = QWebFramePrivate::core(qframe);
    if (!frame)
        return false;

    AnimationController* controller = frame->animation();
    if (!controller)
        return false;

    Document* doc = frame->document();
    Q_ASSERT(doc);

    Node* coreNode = doc->getElementById(elementId);
    if (!coreNode || !coreNode->renderer())
        return false;

    return controller->pauseTransitionAtTime(coreNode->renderer(), propertyName, time);
}

// Pause a given SVG animation on the target node at a specific time.
// This method is only intended to be used for testing the SVG animation system.
bool QWEBKIT_EXPORT qt_drt_pauseSVGAnimation(QWebFrame *qframe, const QString &animationId, double time, const QString &elementId)
{
#if !ENABLE(SVG)
    return false;
#else
    Frame* frame = QWebFramePrivate::core(qframe);
    if (!frame)
        return false;

    Document* doc = frame->document();
    Q_ASSERT(doc);

    if (!doc->svgExtensions())
        return false;

    Node* coreNode = doc->getElementById(animationId);
    if (!coreNode || !SVGSMILElement::isSMILElement(coreNode))
        return false;

    return doc->accessSVGExtensions()->sampleAnimationAtTime(elementId, static_cast<SVGSMILElement*>(coreNode), time);
#endif
}

// Returns the total number of currently running animations (includes both CSS transitions and CSS animations).
int QWEBKIT_EXPORT qt_drt_numberOfActiveAnimations(QWebFrame *qframe)
{
    Frame* frame = QWebFramePrivate::core(qframe);
    if (!frame)
        return false;

    AnimationController* controller = frame->animation();
    if (!controller)
        return false;

    return controller->numberOfActiveAnimations();
}

void QWEBKIT_EXPORT qt_drt_clearFrameName(QWebFrame* qFrame)
{
    Frame* frame = QWebFramePrivate::core(qFrame);
    frame->tree()->clearName();
}

int QWEBKIT_EXPORT qt_drt_javaScriptObjectsCount()
{
    return JSDOMWindowBase::commonJSGlobalData()->heap.globalObjectCount();
}

void QWEBKIT_EXPORT qt_drt_garbageCollector_collect()
{
    gcController().garbageCollectNow();
}

void QWEBKIT_EXPORT qt_drt_garbageCollector_collectOnAlternateThread(bool waitUntilDone)
{
    gcController().garbageCollectOnAlternateThreadForDebugging(waitUntilDone);
}

// Returns the value of counter in the element specified by \a id.
QString QWEBKIT_EXPORT qt_drt_counterValueForElementById(QWebFrame* qFrame, const QString& id)
{
    Frame* frame = QWebFramePrivate::core(qFrame);
    if (Document* document = frame->document()) {
        Element* element = document->getElementById(id);
        return WebCore::counterValueForElement(element);
    }
    return QString();
}

int QWEBKIT_EXPORT qt_drt_pageNumberForElementById(QWebFrame* qFrame, const QString& id, float width, float height)
{
    Frame* frame = QWebFramePrivate::core(qFrame);
    if (!frame)
        return -1;

    Element* element = frame->document()->getElementById(AtomicString(id));
    if (!element)
        return -1;

    return PrintContext::pageNumberForElement(element, FloatSize(width, height));
}

// Suspend active DOM objects in this frame.
void QWEBKIT_EXPORT qt_suspendActiveDOMObjects(QWebFrame* qFrame)
{
    Frame* frame = QWebFramePrivate::core(qFrame);
    if (frame->document())
        frame->document()->suspendActiveDOMObjects();
}
            
// Resume active DOM objects in this frame.
void QWEBKIT_EXPORT qt_resumeActiveDOMObjects(QWebFrame* qFrame)
{
    Frame* frame = QWebFramePrivate::core(qFrame);
    if (frame->document())
        frame->document()->resumeActiveDOMObjects();
}                        

QWebFrameData::QWebFrameData(WebCore::Page* parentPage, WebCore::Frame* parentFrame,
                             WebCore::HTMLFrameOwnerElement* ownerFrameElement,
                             const WebCore::String& frameName)
    : name(frameName)
    , ownerElement(ownerFrameElement)
    , page(parentPage)
    , allowsScrolling(true)
    , marginWidth(0)
    , marginHeight(0)
{
    frameLoaderClient = new FrameLoaderClientQt();
    frame = Frame::create(page, ownerElement, frameLoaderClient);

    // FIXME: All of the below should probably be moved over into WebCore
    frame->tree()->setName(name);
    if (parentFrame)
        parentFrame->tree()->appendChild(frame);
}

void QWebFramePrivate::init(QWebFrame *qframe, QWebFrameData *frameData)
{
    q = qframe;

    allowsScrolling = frameData->allowsScrolling;
    marginWidth = frameData->marginWidth;
    marginHeight = frameData->marginHeight;
    frame = frameData->frame.get();
    frameLoaderClient = frameData->frameLoaderClient;
    frameLoaderClient->setFrame(qframe, frame);

    frame->init();
}

WebCore::Scrollbar* QWebFramePrivate::horizontalScrollBar() const
{
    if (!frame->view())
        return 0;
    return frame->view()->horizontalScrollbar();
}

WebCore::Scrollbar* QWebFramePrivate::verticalScrollBar() const
{
    if (!frame->view())
        return 0;
    return frame->view()->verticalScrollbar();
}

void QWebFramePrivate::renderContentsLayerAbsoluteCoords(GraphicsContext* context, const QRegion& clip)
{
    if (!frame->view() || !frame->contentRenderer())
        return;

    QVector<QRect> vector = clip.rects();
    if (vector.isEmpty())
        return;

    QPainter* painter = context->platformContext();

    WebCore::FrameView* view = frame->view();
    view->layoutIfNeededRecursive();

    for (int i = 0; i < vector.size(); ++i) {
        const QRect& clipRect = vector.at(i);

        painter->save();
        painter->setClipRect(clipRect, Qt::IntersectClip);

        context->save();
        view->paintContents(context, clipRect);
        context->restore();

        painter->restore();
    }
}

void QWebFramePrivate::renderRelativeCoords(GraphicsContext* context, QWebFrame::RenderLayer layer, const QRegion& clip)
{
    if (!frame->view() || !frame->contentRenderer())
        return;

    QVector<QRect> vector = clip.rects();
    if (vector.isEmpty())
        return;

    QPainter* painter = context->platformContext();

    WebCore::FrameView* view = frame->view();
    view->layoutIfNeededRecursive();

    for (int i = 0; i < vector.size(); ++i) {
        const QRect& clipRect = vector.at(i);

        QRect intersectedRect = clipRect.intersected(view->frameRect());

        painter->save();
        painter->setClipRect(clipRect, Qt::IntersectClip);

        int x = view->x();
        int y = view->y();

        if (layer & QWebFrame::ContentsLayer) {
            context->save();

            int scrollX = view->scrollX();
            int scrollY = view->scrollY();

            QRect rect = intersectedRect;
            context->translate(x, y);
            rect.translate(-x, -y);
            context->translate(-scrollX, -scrollY);
            rect.translate(scrollX, scrollY);
            context->clip(view->visibleContentRect());

            view->paintContents(context, rect);

            context->restore();
        }

        if (layer & QWebFrame::ScrollBarLayer
            && !view->scrollbarsSuppressed()
            && (view->horizontalScrollbar() || view->verticalScrollbar())) {
            context->save();

            QRect rect = intersectedRect;
            context->translate(x, y);
            rect.translate(-x, -y);

            view->paintScrollbars(context, rect);

            context->restore();
        }

#if ENABLE(PAN_SCROLLING)
        if (layer & QWebFrame::PanIconLayer)
            view->paintPanScrollIcon(context);
#endif

        painter->restore();
    }
}

bool QWebFramePrivate::scrollOverflow(int dx, int dy)
{
    if (!frame || !frame->document() || !frame->eventHandler())
        return false;

    Node* node = frame->document()->focusedNode();
    if (!node)
        node = frame->document()->elementFromPoint(frame->eventHandler()->currentMousePosition().x(),
                                                   frame->eventHandler()->currentMousePosition().y());
    if (!node)
        return false;

    RenderObject* renderer = node->renderer();
    if (!renderer)
        return false;

    if (renderer->isListBox())
        return false;

    RenderLayer* renderLayer = renderer->enclosingLayer();
    if (!renderLayer)
        return false;

    bool scrolledHorizontal = false;
    bool scrolledVertical = false;

    if (dx > 0)
        scrolledHorizontal = renderLayer->scroll(ScrollRight, ScrollByPixel, dx);
    else if (dx < 0)
        scrolledHorizontal = renderLayer->scroll(ScrollLeft, ScrollByPixel, qAbs(dx));

    if (dy > 0)
        scrolledVertical = renderLayer->scroll(ScrollDown, ScrollByPixel, dy);
    else if (dy < 0)
        scrolledVertical = renderLayer->scroll(ScrollUp, ScrollByPixel, qAbs(dy));

    return (scrolledHorizontal || scrolledVertical);
}



QWebFrame::QWebFrame(QWebPage *parent, QWebFrameData *frameData)
    : QObject(parent)
    , d(new QWebFramePrivate)
{
    d->page = parent;
    d->init(this, frameData);

    if (!frameData->url.isEmpty()) {
        WebCore::ResourceRequest request(frameData->url, frameData->referrer);
        d->frame->loader()->load(request, frameData->name, false);
    }
}

QWebFrame::QWebFrame(QWebFrame *parent, QWebFrameData *frameData)
    : QObject(parent)
    , d(new QWebFramePrivate)
{
    d->page = parent->d->page;
    d->init(this, frameData);
}

QWebFrame::~QWebFrame()
{
    if (d->frame && d->frame->loader() && d->frame->loader()->client())
        static_cast<FrameLoaderClientQt*>(d->frame->loader()->client())->m_webFrame = 0;

    delete d;
}

void QWebFrame::addToJavaScriptWindowObject(const QString &name, QObject *object)
{
    addToJavaScriptWindowObject(name, object, QScriptEngine::QtOwnership);
}

void QWebFrame::addToJavaScriptWindowObject(const QString &name, QObject *object, QScriptEngine::ValueOwnership ownership)
{
    if (!page()->settings()->testAttribute(QWebSettings::JavascriptEnabled))
        return;

    JSC::JSLock lock(JSC::SilenceAssertionsOnly);
    JSDOMWindow* window = toJSDOMWindow(d->frame, mainThreadNormalWorld());
    JSC::Bindings::RootObject* root = d->frame->script()->bindingRootObject();
    if (!window) {
        qDebug() << "Warning: couldn't get window object";
        return;
    }

    JSC::ExecState* exec = window->globalExec();

    JSC::JSObject* runtimeObject =
            JSC::Bindings::QtInstance::getQtInstance(object, root, ownership)->createRuntimeObject(exec);

    JSC::PutPropertySlot slot;
    window->put(exec, JSC::Identifier(exec, (const UChar *) name.constData(), name.length()), runtimeObject, slot);
}

QString QWebFrame::toHtml() const
{
    if (!d->frame->document())
        return QString();
    return createMarkup(d->frame->document());
}

QString QWebFrame::toPlainText() const
{
    if (d->frame->view() && d->frame->view()->layoutPending())
        d->frame->view()->layout();

    Element *documentElement = d->frame->document()->documentElement();
    if (documentElement)
        return documentElement->innerText();
    return QString();
}

QString QWebFrame::renderTreeDump() const
{
    if (d->frame->view() && d->frame->view()->layoutPending())
        d->frame->view()->layout();

    return externalRepresentation(d->frame);
}


QString QWebFrame::title() const
{
    if (d->frame->document())
        return d->frame->loader()->documentLoader()->title();
    return QString();
}

QMultiMap<QString, QString> QWebFrame::metaData() const
{
    if (!d->frame->document())
       return QMap<QString, QString>();

    QMultiMap<QString, QString> map;
    Document* doc = d->frame->document();
    RefPtr<NodeList> list = doc->getElementsByTagName("meta");
    unsigned len = list->length();
    for (unsigned i = 0; i < len; i++) {
        HTMLMetaElement* meta = static_cast<HTMLMetaElement*>(list->item(i));
        map.insert(meta->name(), meta->content());
    }
    return map;
}

static inline QUrl ensureAbsoluteUrl(const QUrl &url)
{
    if (!url.isRelative())
        return url;

    return QUrl::fromLocalFile(QFileInfo(url.toLocalFile()).absoluteFilePath());
}


void QWebFrame::setUrl(const QUrl &url)
{
    d->frame->loader()->begin(ensureAbsoluteUrl(url));
    d->frame->loader()->end();
    load(ensureAbsoluteUrl(url));
}

QUrl QWebFrame::url() const
{
    return d->frame->loader()->url();
}

QUrl QWebFrame::requestedUrl() const
{
    // There are some possible edge cases to be handled here,
    // apart from checking if activeDocumentLoader is valid:
    //
    // * Method can be called while processing an unsucessful load.
    //   In this case, frameLoaderClient will hold the current error
    //   (m_loadError), and we will make use of it to recover the 'failingURL'.
    // * If the 'failingURL' holds a null'ed string though, we fallback
    //   to 'outgoingReferrer' (it yet is safer than originalRequest).
    FrameLoader* loader = d->frame->loader();
    FrameLoaderClientQt* loaderClient = d->frameLoaderClient;

    if (!loader->activeDocumentLoader()
        || !loaderClient->m_loadError.isNull()) {
        if (!loaderClient->m_loadError.failingURL().isNull())
            return QUrl(loaderClient->m_loadError.failingURL());
        else if (!loader->outgoingReferrer().isEmpty())
            return QUrl(loader->outgoingReferrer());
    }

    return loader->originalRequest().url();
}

QUrl QWebFrame::baseUrl() const
{
    return d->frame->loader()->baseURL();
}


QIcon QWebFrame::icon() const
{
    return QWebSettings::iconForUrl(d->frame->loader()->url());
}

QString QWebFrame::frameName() const
{
    return d->frame->tree()->name();
}

QWebPage *QWebFrame::page() const
{
    return d->page;
}

void QWebFrame::load(const QUrl &url)
{
    load(QNetworkRequest(ensureAbsoluteUrl(url)));
}

void QWebFrame::load(const QNetworkRequest &req,
                     QNetworkAccessManager::Operation operation,
                     const QByteArray &body)
{
    if (d->parentFrame())
        d->page->d->insideOpenCall = true;

    QUrl url = ensureAbsoluteUrl(req.url());

    WebCore::ResourceRequest request(url);

    switch (operation) {
        case QNetworkAccessManager::HeadOperation:
            request.setHTTPMethod("HEAD");
            break;
        case QNetworkAccessManager::GetOperation:
            request.setHTTPMethod("GET");
            break;
        case QNetworkAccessManager::PutOperation:
            request.setHTTPMethod("PUT");
            break;
        case QNetworkAccessManager::PostOperation:
            request.setHTTPMethod("POST");
            break;
#if QT_VERSION >= 0x040600
        case QNetworkAccessManager::DeleteOperation:
            request.setHTTPMethod("DELETE");
            break;
#endif
        case QNetworkAccessManager::UnknownOperation:
            // eh?
            break;
    }

    QList<QByteArray> httpHeaders = req.rawHeaderList();
    for (int i = 0; i < httpHeaders.size(); ++i) {
        const QByteArray &headerName = httpHeaders.at(i);
        request.addHTTPHeaderField(QString::fromLatin1(headerName), QString::fromLatin1(req.rawHeader(headerName)));
    }

    if (!body.isEmpty())
        request.setHTTPBody(WebCore::FormData::create(body.constData(), body.size()));

    d->frame->loader()->load(request, false);

    if (d->parentFrame())
        d->page->d->insideOpenCall = false;
}

void QWebFrame::setHtml(const QString &html, const QUrl &baseUrl)
{
    KURL kurl(baseUrl);
    WebCore::ResourceRequest request(kurl);
    const QByteArray utf8 = html.toUtf8();
    WTF::RefPtr<WebCore::SharedBuffer> data = WebCore::SharedBuffer::create(utf8.constData(), utf8.length());
    WebCore::SubstituteData substituteData(data, WebCore::String("text/html"), WebCore::String("utf-8"), KURL());
    d->frame->loader()->load(request, substituteData, false);
}

void QWebFrame::setContent(const QByteArray &data, const QString &mimeType, const QUrl &baseUrl)
{
    KURL kurl(baseUrl);
    WebCore::ResourceRequest request(kurl);
    WTF::RefPtr<WebCore::SharedBuffer> buffer = WebCore::SharedBuffer::create(data.constData(), data.length());
    QString actualMimeType = mimeType;
    if (actualMimeType.isEmpty())
        actualMimeType = QLatin1String("text/html");
    WebCore::SubstituteData substituteData(buffer, WebCore::String(actualMimeType), WebCore::String(), KURL());
    d->frame->loader()->load(request, substituteData, false);
}

QWebFrame *QWebFrame::parentFrame() const
{
    return d->parentFrame();
}

QList<QWebFrame*> QWebFrame::childFrames() const
{
    QList<QWebFrame*> rc;
    if (d->frame) {
        FrameTree *tree = d->frame->tree();
        for (Frame *child = tree->firstChild(); child; child = child->tree()->nextSibling()) {
            FrameLoader *loader = child->loader();
            FrameLoaderClientQt *client = static_cast<FrameLoaderClientQt*>(loader->client());
            if (client)
                rc.append(client->webFrame());
        }

    }
    return rc;
}

Qt::ScrollBarPolicy QWebFrame::scrollBarPolicy(Qt::Orientation orientation) const
{
    if (orientation == Qt::Horizontal)
        return d->horizontalScrollBarPolicy;
    return d->verticalScrollBarPolicy;
}

void QWebFrame::setScrollBarPolicy(Qt::Orientation orientation, Qt::ScrollBarPolicy policy)
{
    Q_ASSERT((int)ScrollbarAuto == (int)Qt::ScrollBarAsNeeded);
    Q_ASSERT((int)ScrollbarAlwaysOff == (int)Qt::ScrollBarAlwaysOff);
    Q_ASSERT((int)ScrollbarAlwaysOn == (int)Qt::ScrollBarAlwaysOn);

    if (orientation == Qt::Horizontal) {
        d->horizontalScrollBarPolicy = policy;
        if (d->frame->view()) {
            d->frame->view()->setHorizontalScrollbarMode((ScrollbarMode)policy);
            d->frame->view()->updateCanHaveScrollbars();
        }
    } else {
        d->verticalScrollBarPolicy = policy;
        if (d->frame->view()) {
            d->frame->view()->setVerticalScrollbarMode((ScrollbarMode)policy);
            d->frame->view()->updateCanHaveScrollbars();
        }
    }
}

void QWebFrame::setScrollBarValue(Qt::Orientation orientation, int value)
{
    Scrollbar *sb;
    sb = (orientation == Qt::Horizontal) ? d->horizontalScrollBar() : d->verticalScrollBar();
    if (sb) {
        if (value < 0)
            value = 0;
        else if (value > scrollBarMaximum(orientation))
            value = scrollBarMaximum(orientation);
        sb->setValue(value);
    }
}

int QWebFrame::scrollBarValue(Qt::Orientation orientation) const
{
    Scrollbar *sb;
    sb = (orientation == Qt::Horizontal) ? d->horizontalScrollBar() : d->verticalScrollBar();
    if (sb)
        return sb->value();
    return 0;
}

int QWebFrame::scrollBarMaximum(Qt::Orientation orientation) const
{
    Scrollbar *sb;
    sb = (orientation == Qt::Horizontal) ? d->horizontalScrollBar() : d->verticalScrollBar();
    if (sb)
        return sb->maximum();
    return 0;
}

int QWebFrame::scrollBarMinimum(Qt::Orientation orientation) const
{
    Q_UNUSED(orientation)
    return 0;
}

QRect QWebFrame::scrollBarGeometry(Qt::Orientation orientation) const
{
    Scrollbar *sb;
    sb = (orientation == Qt::Horizontal) ? d->horizontalScrollBar() : d->verticalScrollBar();
    if (sb)
        return sb->frameRect();
    return QRect();
}


void QWebFrame::scroll(int dx, int dy)
{
    if (!d->frame->view())
        return;

    d->frame->view()->scrollBy(IntSize(dx, dy));
}

bool QWebFrame::scrollRecursively(int dx, int dy)
{
    bool scrolledHorizontal = false;
    bool scrolledVertical = false;
    bool scrolledOverflow = d->scrollOverflow(dx, dy);

    if (!scrolledOverflow) {
        Frame* frame = d->frame;
        if (!frame || !frame->view())
            return false;

        do {
            IntSize scrollOffset = frame->view()->scrollOffset();
            IntPoint maxScrollOffset = frame->view()->maximumScrollPosition();

            if (dx > 0) // scroll right
                scrolledHorizontal = scrollOffset.width() < maxScrollOffset.x();
            else if (dx < 0) // scroll left
                scrolledHorizontal = scrollOffset.width() > 0;

            if (dy > 0) // scroll down
                scrolledVertical = scrollOffset.height() < maxScrollOffset.y();
            else if (dy < 0) //scroll up
                scrolledVertical = scrollOffset.height() > 0;

            if (scrolledHorizontal || scrolledVertical) {
                frame->view()->scrollBy(IntSize(dx, dy));
                return true;
            }
            frame = frame->tree()->parent(); 
        } while (frame && frame->view());
    }
    return (scrolledHorizontal || scrolledVertical || scrolledOverflow);
}

bool QWEBKIT_EXPORT qtwebkit_webframe_scrollRecursively(QWebFrame* qFrame, int dx, int dy)
{
    return qFrame->scrollRecursively(dx, dy);
}


QPoint QWebFrame::scrollPosition() const
{
    if (!d->frame->view())
        return QPoint(0, 0);

    IntSize ofs = d->frame->view()->scrollOffset();
    return QPoint(ofs.width(), ofs.height());
}

void QWebFrame::setScrollPosition(const QPoint &pos)
{
    QPoint current = scrollPosition();
    int dx = pos.x() - current.x();
    int dy = pos.y() - current.y();
    scroll(dx, dy);
}


void QWebFrame::render(QPainter* painter, RenderLayer layer, const QRegion& clip)
{
    GraphicsContext context(painter);
    if (context.paintingDisabled() && !context.updatingControlTints())
        return;

    if (!clip.isEmpty())
        d->renderRelativeCoords(&context, layer, clip);
    else if (d->frame->view())
        d->renderRelativeCoords(&context, layer, QRegion(d->frame->view()->frameRect()));
}

void QWebFrame::render(QPainter* painter, const QRegion& clip)
{
    GraphicsContext context(painter);
    if (context.paintingDisabled() && !context.updatingControlTints())
        return;

    d->renderRelativeCoords(&context, AllLayers, clip);
}

void QWebFrame::render(QPainter* painter)
{
    if (!d->frame->view())
        return;

    GraphicsContext context(painter);
    if (context.paintingDisabled() && !context.updatingControlTints())
        return;

    d->renderRelativeCoords(&context, AllLayers, QRegion(d->frame->view()->frameRect()));
}


void QWebFrame::setTextSizeMultiplier(qreal factor)
{
    d->frame->setZoomFactor(factor, /*isTextOnly*/true);
}

qreal QWebFrame::textSizeMultiplier() const
{
    return d->frame->zoomFactor();
}


void QWebFrame::setZoomFactor(qreal factor)
{
    d->frame->setZoomFactor(factor, d->frame->isZoomFactorTextOnly());
}

qreal QWebFrame::zoomFactor() const
{
    return d->frame->zoomFactor();
}

bool QWebFrame::hasFocus() const
{
    WebCore::Frame* ff = d->frame->page()->focusController()->focusedFrame();
    return ff && QWebFramePrivate::kit(ff) == this;
}

void QWebFrame::setFocus()
{
    QWebFramePrivate::core(this)->page()->focusController()->setFocusedFrame(QWebFramePrivate::core(this));
}

QPoint QWebFrame::pos() const
{
    if (!d->frame->view())
        return QPoint();

    return d->frame->view()->frameRect().topLeft();
}

QRect QWebFrame::geometry() const
{
    if (!d->frame->view())
        return QRect();
    return d->frame->view()->frameRect();
}

QSize QWebFrame::contentsSize() const
{
    FrameView *view = d->frame->view();
    if (!view)
        return QSize();
    return QSize(view->contentsWidth(), view->contentsHeight());
}

QWebElement QWebFrame::documentElement() const
{
    WebCore::Document *doc = d->frame->document();
    if (!doc)
        return QWebElement();
    return QWebElement(doc->documentElement());
}

QWebElementCollection QWebFrame::findAllElements(const QString &selectorQuery) const
{
    return documentElement().findAll(selectorQuery);
}

QWebElement QWebFrame::findFirstElement(const QString &selectorQuery) const
{
    return documentElement().findFirst(selectorQuery);
}

QWebHitTestResult QWebFrame::hitTestContent(const QPoint &pos) const
{
    if (!d->frame->view() || !d->frame->contentRenderer())
        return QWebHitTestResult();

    HitTestResult result = d->frame->eventHandler()->hitTestResultAtPoint(d->frame->view()->windowToContents(pos), /*allowShadowContent*/ false, /*ignoreClipping*/ true);

    if (result.scrollbar())
        return QWebHitTestResult();

    return QWebHitTestResult(new QWebHitTestResultPrivate(result));
}

bool QWebFrame::event(QEvent *e)
{
    return QObject::event(e);
}

#ifndef QT_NO_PRINTER
void QWebFrame::print(QPrinter *printer) const
{
    QPainter painter;
    if (!painter.begin(printer))
        return;

    const qreal zoomFactorX = printer->logicalDpiX() / qt_defaultDpi();
    const qreal zoomFactorY = printer->logicalDpiY() / qt_defaultDpi();

    PrintContext printContext(d->frame);
    float pageHeight = 0;

    QRect qprinterRect = printer->pageRect();

    IntRect pageRect(0, 0,
                     int(qprinterRect.width() / zoomFactorX),
                     int(qprinterRect.height() / zoomFactorY));

    printContext.begin(pageRect.width());

    printContext.computePageRects(pageRect, /* headerHeight */ 0, /* footerHeight */ 0, /* userScaleFactor */ 1.0, pageHeight);

    int docCopies;
    int pageCopies;
    if (printer->collateCopies()) {
        docCopies = 1;
        pageCopies = printer->numCopies();
    } else {
        docCopies = printer->numCopies();
        pageCopies = 1;
    }

    int fromPage = printer->fromPage();
    int toPage = printer->toPage();
    bool ascending = true;

    if (fromPage == 0 && toPage == 0) {
        fromPage = 1;
        toPage = printContext.pageCount();
    }
    // paranoia check
    fromPage = qMax(1, fromPage);
    toPage = qMin(printContext.pageCount(), toPage);
    if (toPage < fromPage) {
        // if the user entered a page range outside the actual number
        // of printable pages, just return
        return;
    }

    if (printer->pageOrder() == QPrinter::LastPageFirst) {
        int tmp = fromPage;
        fromPage = toPage;
        toPage = tmp;
        ascending = false;
    }

    painter.scale(zoomFactorX, zoomFactorY);
    GraphicsContext ctx(&painter);

    for (int i = 0; i < docCopies; ++i) {
        int page = fromPage;
        while (true) {
            for (int j = 0; j < pageCopies; ++j) {
                if (printer->printerState() == QPrinter::Aborted
                    || printer->printerState() == QPrinter::Error) {
                    printContext.end();
                    return;
                }
                printContext.spoolPage(ctx, page - 1, pageRect.width());
                if (j < pageCopies - 1)
                    printer->newPage();
            }

            if (page == toPage)
                break;

            if (ascending)
                ++page;
            else
                --page;

            printer->newPage();
        }

        if ( i < docCopies - 1)
            printer->newPage();
    }

    printContext.end();
}
#endif // QT_NO_PRINTER

QVariant QWebFrame::evaluateJavaScript(const QString& scriptSource)
{
    ScriptController *proxy = d->frame->script();
    QVariant rc;
    if (proxy) {
        JSC::JSValue v = d->frame->script()->executeScript(ScriptSourceCode(scriptSource)).jsValue();
        int distance = 0;
        rc = JSC::Bindings::convertValueToQVariant(proxy->globalObject(mainThreadNormalWorld())->globalExec(), v, QMetaType::Void, &distance);
    }
    return rc;
}

QWebSecurityOrigin QWebFrame::securityOrigin() const
{
    QWebFrame* that = const_cast<QWebFrame*>(this);
    QWebSecurityOriginPrivate* priv = new QWebSecurityOriginPrivate(QWebFramePrivate::core(that)->document()->securityOrigin());
    return QWebSecurityOrigin(priv);
}

WebCore::Frame* QWebFramePrivate::core(QWebFrame* webFrame)
{
    return webFrame->d->frame;
}

QWebFrame* QWebFramePrivate::kit(WebCore::Frame* coreFrame)
{
    return static_cast<FrameLoaderClientQt*>(coreFrame->loader()->client())->webFrame();
}












QWebHitTestResult::QWebHitTestResult(QWebHitTestResultPrivate *priv)
    : d(priv)
{
}

QWebHitTestResultPrivate::QWebHitTestResultPrivate(const WebCore::HitTestResult &hitTest)
    : isContentEditable(false)
    , isContentSelected(false)
    , isScrollBar(false)
{
    if (!hitTest.innerNode())
        return;
    pos = hitTest.point();
    WebCore::TextDirection dir;
    title = hitTest.title(dir);
    linkText = hitTest.textContent();
    linkUrl = hitTest.absoluteLinkURL();
    linkTitle = hitTest.titleDisplayString();
    alternateText = hitTest.altDisplayString();
    imageUrl = hitTest.absoluteImageURL();
    innerNode = hitTest.innerNode();
    innerNonSharedNode = hitTest.innerNonSharedNode();
    boundingRect = innerNonSharedNode ? innerNonSharedNode->renderer()->absoluteBoundingBoxRect(true) : IntRect();
    WebCore::Image *img = hitTest.image();
    if (img) {
        QPixmap *pix = img->nativeImageForCurrentFrame();
        if (pix)
            pixmap = *pix;
    }
    WebCore::Frame *wframe = hitTest.targetFrame();
    if (wframe)
        linkTargetFrame = QWebFramePrivate::kit(wframe);
    linkElement = QWebElement(hitTest.URLElement());

    isContentEditable = hitTest.isContentEditable();
    isContentSelected = hitTest.isSelected();
    isScrollBar = hitTest.scrollbar();

    if (innerNonSharedNode && innerNonSharedNode->document()
        && innerNonSharedNode->document()->frame())
        frame = QWebFramePrivate::kit(innerNonSharedNode->document()->frame());

    enclosingBlock = QWebElement(WebCore::enclosingBlock(innerNode.get()));
}

QWebHitTestResult::QWebHitTestResult()
    : d(0)
{
}

QWebHitTestResult::QWebHitTestResult(const QWebHitTestResult &other)
    : d(0)
{
    if (other.d)
        d = new QWebHitTestResultPrivate(*other.d);
}

QWebHitTestResult &QWebHitTestResult::operator=(const QWebHitTestResult &other)
{
    if (this != &other) {
        if (other.d) {
            if (!d)
                d = new QWebHitTestResultPrivate;
            *d = *other.d;
        } else {
            delete d;
            d = 0;
        }
    }
    return *this;
}

QWebHitTestResult::~QWebHitTestResult()
{
    delete d;
}

bool QWebHitTestResult::isNull() const
{
    return !d;
}

QPoint QWebHitTestResult::pos() const
{
    if (!d)
        return QPoint();
    return d->pos;
}

QRect QWebHitTestResult::boundingRect() const
{
    if (!d)
        return QRect();
    return d->boundingRect;
}

QWebElement QWebHitTestResult::enclosingBlockElement() const
{
    if (!d)
        return QWebElement();
    return d->enclosingBlock;
}

QString QWebHitTestResult::title() const
{
    if (!d)
        return QString();
    return d->title;
}

QString QWebHitTestResult::linkText() const
{
    if (!d)
        return QString();
    return d->linkText;
}

QUrl QWebHitTestResult::linkUrl() const
{
    if (!d)
        return QUrl();
    return d->linkUrl;
}

QUrl QWebHitTestResult::linkTitle() const
{
    if (!d)
        return QUrl();
    return d->linkTitle;
}

QWebElement QWebHitTestResult::linkElement() const
{
    if (!d)
        return QWebElement();
    return d->linkElement;
}

QWebFrame *QWebHitTestResult::linkTargetFrame() const
{
    if (!d)
        return 0;
    return d->linkTargetFrame;
}

QString QWebHitTestResult::alternateText() const
{
    if (!d)
        return QString();
    return d->alternateText;
}

QUrl QWebHitTestResult::imageUrl() const
{
    if (!d)
        return QUrl();
    return d->imageUrl;
}

QPixmap QWebHitTestResult::pixmap() const
{
    if (!d)
        return QPixmap();
    return d->pixmap;
}

bool QWebHitTestResult::isContentEditable() const
{
    if (!d)
        return false;
    return d->isContentEditable;
}

bool QWebHitTestResult::isContentSelected() const
{
    if (!d)
        return false;
    return d->isContentSelected;
}

QWebElement QWebHitTestResult::element() const
{
    if (!d || !d->innerNonSharedNode || !d->innerNonSharedNode->isElementNode())
        return QWebElement();

    return QWebElement(static_cast<WebCore::Element*>(d->innerNonSharedNode.get()));
}

QWebFrame *QWebHitTestResult::frame() const
{
    if (!d)
        return 0;
    return d->frame;
}
