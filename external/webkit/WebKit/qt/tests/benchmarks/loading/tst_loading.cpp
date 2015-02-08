

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

class tst_Loading : public QObject
{
    Q_OBJECT

public:

public Q_SLOTS:
    void init();
    void cleanup();

private Q_SLOTS:
    void load_data();
    void load();

private:
    QWebView* m_view;
    QWebPage* m_page;
};

void tst_Loading::init()
{
    m_view = new QWebView;
    m_page = m_view->page();

    QSize viewportSize(1024, 768);
    m_view->setFixedSize(viewportSize);
    m_page->setViewportSize(viewportSize);
}

void tst_Loading::cleanup()
{
    delete m_view;
}

void tst_Loading::load_data()
{
    QTest::addColumn<QUrl>("url");
    QTest::newRow("amazon") << QUrl("http://www.amazon.com");
    QTest::newRow("kde") << QUrl("http://www.kde.org");
    QTest::newRow("apple") << QUrl("http://www.apple.com");
}

void tst_Loading::load()
{
    QFETCH(QUrl, url);


    QBENCHMARK {
        m_view->load(url);

        // really wait for loading, painting is in another test
        ::waitForSignal(m_view, SIGNAL(loadFinished(bool)));
    }
}

QTEST_MAIN(tst_Loading)
#include "tst_loading.moc"
