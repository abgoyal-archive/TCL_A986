

#include <QtTest/QtTest>

#include <qwebframe.h>
#include <qwebview.h>
#include <qpainter.h>

static bool waitForSignal(QObject* obj, const char* signal, int timeout = 0)
{
    QEventLoop loop;
    QObject::connect(obj, signal, &loop, SLOT(quit()));
    QTimer timer;
    QSignalSpy timeoutSpy(&timer, SIGNAL(timeout()));
    if (timeout > 0) {
        QObject::connect(&timer, SIGNAL(timeout()), &loop, SLOT(quit()));
        timer.setSingleShot(true);
        timer.start(timeout);
    }
    loop.exec();
    return timeoutSpy.isEmpty();
}

class tst_Painting : public QObject
{
    Q_OBJECT

public:

public Q_SLOTS:
    void init();
    void cleanup();

private Q_SLOTS:
    void paint_data();
    void paint();

private:
    QWebView* m_view;
    QWebPage* m_page;
};

void tst_Painting::init()
{
    m_view = new QWebView;
    m_page = m_view->page();

    QSize viewportSize(1024, 768);
    m_view->setFixedSize(viewportSize);
    m_page->setViewportSize(viewportSize);
}

void tst_Painting::cleanup()
{
    delete m_view;
}

void tst_Painting::paint_data()
{
    QTest::addColumn<QUrl>("url");
    QTest::newRow("amazon") << QUrl("http://www.amazon.com");
}

void tst_Painting::paint()
{
    QFETCH(QUrl, url);

    m_view->load(url);
    ::waitForSignal(m_view, SIGNAL(loadFinished(bool)));

    /* force a layout */
    QWebFrame* mainFrame = m_page->mainFrame();
    mainFrame->toPlainText();

    QPixmap pixmap(m_page->viewportSize());
    QBENCHMARK {
        QPainter painter(&pixmap);
        mainFrame->render(&painter, QRect(QPoint(0, 0), m_page->viewportSize()));
        painter.end();
    }
}

QTEST_MAIN(tst_Painting)
#include "tst_painting.moc"
