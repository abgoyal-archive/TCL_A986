

#ifndef webview_h
#define webview_h

#include "webpage.h"
#include <qwebview.h>
#include <qgraphicswebview.h>
#include <QGraphicsView>
#include <QGraphicsWidget>
#include <QTime>

class WebViewTraditional : public QWebView {
    Q_OBJECT

public:
    WebViewTraditional(QWidget* parent) : QWebView(parent) {}

protected:
    virtual void contextMenuEvent(QContextMenuEvent*);
    virtual void mousePressEvent(QMouseEvent*);
};


class GraphicsWebView : public QGraphicsWebView {
    Q_OBJECT

public:
    GraphicsWebView(QGraphicsItem* parent = 0) : QGraphicsWebView(parent) {};

protected:
    virtual void contextMenuEvent(QGraphicsSceneContextMenuEvent*);
    virtual void mousePressEvent(QGraphicsSceneMouseEvent*);
};


class WebViewGraphicsBased : public QGraphicsView {
    Q_OBJECT

public:
    WebViewGraphicsBased(QWidget* parent);
    virtual void resizeEvent(QResizeEvent*);
    void setPage(QWebPage* page) { m_item->setPage(page); }
    void setItemCacheMode(QGraphicsItem::CacheMode mode) { m_item->setCacheMode(mode); }

    void enableFrameRateMeasurement();
    virtual void paintEvent(QPaintEvent* event);

public slots:
    void updateFrameRate();

private:
    GraphicsWebView* m_item;
    int m_numPaintsTotal;
    int m_numPaintsSinceLastMeasure;
    QTime m_startTime;
    QTime m_lastConsultTime;
    bool m_measureFps;
};

#endif
