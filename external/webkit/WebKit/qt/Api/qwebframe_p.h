

#ifndef QWEBFRAME_P_H
#define QWEBFRAME_P_H

#include "qwebframe.h"
#include "qwebpage_p.h"

#include "EventHandler.h"
#include "GraphicsContext.h"
#include "KURL.h"
#include "PlatformString.h"
#include "qwebelement.h"
#include "wtf/RefPtr.h"
#include "Frame.h"

namespace WebCore {
    class FrameLoaderClientQt;
    class FrameView;
    class HTMLFrameOwnerElement;
    class Scrollbar;
}
class QWebPage;

class QWebFrameData {
public:
    QWebFrameData(WebCore::Page*, WebCore::Frame* parentFrame = 0,
                  WebCore::HTMLFrameOwnerElement* = 0,
                  const WebCore::String& frameName = WebCore::String());

    WebCore::KURL url;
    WebCore::String name;
    WebCore::HTMLFrameOwnerElement* ownerElement;
    WebCore::Page* page;
    RefPtr<WebCore::Frame> frame;
    WebCore::FrameLoaderClientQt* frameLoaderClient;

    WebCore::String referrer;
    bool allowsScrolling;
    int marginWidth;
    int marginHeight;
};

class QWebFramePrivate {
public:
    QWebFramePrivate()
        : q(0)
        , horizontalScrollBarPolicy(Qt::ScrollBarAsNeeded)
        , verticalScrollBarPolicy(Qt::ScrollBarAsNeeded)
        , frameLoaderClient(0)
        , frame(0)
        , page(0)
        , allowsScrolling(true)
        , marginWidth(-1)
        , marginHeight(-1)
        {}
    void init(QWebFrame* qframe, QWebFrameData* frameData);

    inline QWebFrame *parentFrame() { return qobject_cast<QWebFrame*>(q->parent()); }

    WebCore::Scrollbar* horizontalScrollBar() const;
    WebCore::Scrollbar* verticalScrollBar() const;

    static WebCore::Frame* core(QWebFrame*);
    static QWebFrame* kit(WebCore::Frame*);

    void renderRelativeCoords(WebCore::GraphicsContext*, QWebFrame::RenderLayer, const QRegion& clip);
    void renderContentsLayerAbsoluteCoords(WebCore::GraphicsContext*, const QRegion& clip);

    bool scrollOverflow(int dx, int dy);

    QWebFrame *q;
    Qt::ScrollBarPolicy horizontalScrollBarPolicy;
    Qt::ScrollBarPolicy verticalScrollBarPolicy;
    WebCore::FrameLoaderClientQt *frameLoaderClient;
    WebCore::Frame *frame;
    QWebPage *page;

    bool allowsScrolling;
    int marginWidth;
    int marginHeight;
};

class QWebHitTestResultPrivate {
public:
    QWebHitTestResultPrivate() : isContentEditable(false), isContentSelected(false), isScrollBar(false) {}
    QWebHitTestResultPrivate(const WebCore::HitTestResult &hitTest);

    QPoint pos;
    QRect boundingRect;
    QWebElement enclosingBlock;
    QString title;
    QString linkText;
    QUrl linkUrl;
    QString linkTitle;
    QPointer<QWebFrame> linkTargetFrame;
    QWebElement linkElement;
    QString alternateText;
    QUrl imageUrl;
    QPixmap pixmap;
    bool isContentEditable;
    bool isContentSelected;
    bool isScrollBar;
    QPointer<QWebFrame> frame;
    RefPtr<WebCore::Node> innerNode;
    RefPtr<WebCore::Node> innerNonSharedNode;
};

#endif
