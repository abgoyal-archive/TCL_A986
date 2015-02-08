

#include "../util.h"
#include <QtTest/QtTest>
#include <QGraphicsView>
#include <qgraphicswebview.h>
#include <qwebpage.h>
#include <qwebframe.h>

class tst_QGraphicsWebView : public QObject
{
    Q_OBJECT

private slots:
    void qgraphicswebview();
    void crashOnViewlessWebPages();
};

void tst_QGraphicsWebView::qgraphicswebview()
{
    QGraphicsWebView item;
    item.url();
    item.title();
    item.icon();
    item.zoomFactor();
    item.history();
    item.settings();
    item.page();
    item.setPage(0);
    item.page();
    item.setUrl(QUrl());
    item.setZoomFactor(0);
    item.load(QUrl());
    item.setHtml(QString());
    item.setContent(QByteArray());
    item.isModified();
}

class WebPage : public QWebPage
{
    Q_OBJECT

public:
    WebPage(QObject* parent = 0): QWebPage(parent)
    {
    }

    QGraphicsWebView* webView;

private slots:
    // Force a webview deletion during the load.
    // It should not cause WebPage to crash due to
    // it accessing invalid pageClient pointer.
    void aborting()
    {
        delete webView;
    }
};

void tst_QGraphicsWebView::crashOnViewlessWebPages()
{
    QGraphicsScene scene;
    QGraphicsView view(&scene);

    QGraphicsWebView* webView = new QGraphicsWebView;
    WebPage* page = new WebPage;
    webView->setPage(page);
    page->webView = webView;
    connect(page->mainFrame(), SIGNAL(initialLayoutCompleted()), page, SLOT(aborting()));

    scene.addItem(webView);

    view.setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    view.resize(600, 480);
    webView->resize(view.geometry().size());
    QCoreApplication::processEvents();
    view.show();

    page->mainFrame()->setHtml(QString("data:text/html,"
                                            "<frameset cols=\"25%,75%\">"
                                                "<frame src=\"data:text/html,foo \">"
                                                "<frame src=\"data:text/html,bar\">"
                                            "</frameset>"));

    QVERIFY(waitForSignal(page, SIGNAL(loadFinished(bool))));
}

QTEST_MAIN(tst_QGraphicsWebView)

#include "tst_qgraphicswebview.moc"
