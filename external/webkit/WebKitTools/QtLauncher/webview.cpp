

#include "webview.h"

#include <QtGui>
#include <QGraphicsScene>

WebViewGraphicsBased::WebViewGraphicsBased(QWidget* parent)
    : QGraphicsView(parent)
    , m_item(new GraphicsWebView)
    , m_numPaintsTotal(0)
    , m_numPaintsSinceLastMeasure(0)
    , m_measureFps(false)
{
    setScene(new QGraphicsScene(this));
    scene()->addItem(m_item);

    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

}

void WebViewGraphicsBased::resizeEvent(QResizeEvent* event)
{
    QGraphicsView::resizeEvent(event);
    QRectF rect(QPoint(0, 0), event->size());
    m_item->setGeometry(rect);
}

void WebViewGraphicsBased::enableFrameRateMeasurement()
{
    m_measureFps = true;
    m_lastConsultTime = m_startTime = QTime::currentTime();
    QTimer* updateTimer = new QTimer(this);
    updateTimer->setInterval(1000);
    connect(updateTimer, SIGNAL(timeout()), this, SLOT(updateFrameRate()));
    updateTimer->start();
}

void WebViewGraphicsBased::updateFrameRate()
{
    QTime now = QTime::currentTime();

    int interval = m_lastConsultTime.msecsTo(now);
    int total = m_startTime.msecsTo(now);

    int average = total ? m_numPaintsTotal * 1000 / total : 0;
    int current = interval ? m_numPaintsSinceLastMeasure * 1000 / interval : 0;

    qDebug("[FPS] average: %d, current: %d", average, current);

    m_lastConsultTime = now;
    m_numPaintsSinceLastMeasure = 0;
}

void WebViewGraphicsBased::paintEvent(QPaintEvent* event)
{
    QGraphicsView::paintEvent(event);
    if (!m_measureFps)
        return;
    m_numPaintsSinceLastMeasure++;
    m_numPaintsTotal++;
}

static QMenu* createContextMenu(QWebPage* page, QPoint position)
{
    QMenu* menu = page->createStandardContextMenu();

    QWebHitTestResult r = page->mainFrame()->hitTestContent(position);

    if (!r.linkUrl().isEmpty()) {
        WebPage* webPage = qobject_cast<WebPage*>(page);
        QAction* newTabAction = menu->addAction("Open in Default &Browser", webPage, SLOT(openUrlInDefaultBrowser()));
        newTabAction->setData(r.linkUrl());
        menu->insertAction(menu->actions().at(2), newTabAction);
    }
    return menu;
}

void GraphicsWebView::mousePressEvent(QGraphicsSceneMouseEvent* event)
{
    setProperty("mouseButtons", QVariant::fromValue(int(event->buttons())));
    setProperty("keyboardModifiers", QVariant::fromValue(int(event->modifiers())));

    QGraphicsWebView::mousePressEvent(event);
}

void WebViewTraditional::mousePressEvent(QMouseEvent* event)
{
    setProperty("mouseButtons", QVariant::fromValue(int(event->buttons())));
    setProperty("keyboardModifiers", QVariant::fromValue(int(event->modifiers())));

    QWebView::mousePressEvent(event);
}

void GraphicsWebView::contextMenuEvent(QGraphicsSceneContextMenuEvent* event)
{
    QMenu* menu = createContextMenu(page(), event->pos().toPoint());
    menu->exec(mapToScene(event->pos()).toPoint());
    delete menu;
}

void WebViewTraditional::contextMenuEvent(QContextMenuEvent* event)
{
    QMenu* menu = createContextMenu(page(), event->pos());
    menu->exec(mapToGlobal(event->pos()));
    delete menu;
}

