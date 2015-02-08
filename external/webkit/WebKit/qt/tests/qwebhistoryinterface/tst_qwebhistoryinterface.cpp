


#include <QtTest/QtTest>

#include <qwebpage.h>
#include <qwebview.h>
#include <qwebframe.h>
#include <qwebhistoryinterface.h>
#include <QDebug>

class tst_QWebHistoryInterface : public QObject
{
    Q_OBJECT

public:
    tst_QWebHistoryInterface();
    virtual ~tst_QWebHistoryInterface();

public slots:
    void init();
    void cleanup();

private slots:
    void visitedLinks();

private:


private:
    QWebView* m_view;
    QWebPage* m_page;
};

tst_QWebHistoryInterface::tst_QWebHistoryInterface()
{
}

tst_QWebHistoryInterface::~tst_QWebHistoryInterface()
{
}

void tst_QWebHistoryInterface::init()
{
    m_view = new QWebView();
    m_page = m_view->page();
}

void tst_QWebHistoryInterface::cleanup()
{
    delete m_view;
}

class FakeHistoryImplementation : public QWebHistoryInterface {
public:
    void addHistoryEntry(const QString&) {}
    bool historyContains(const QString& url) const {
        return url == QLatin1String("http://www.trolltech.com/");
    }
};


void tst_QWebHistoryInterface::visitedLinks()
{
    QWebHistoryInterface::setDefaultInterface(new FakeHistoryImplementation);
    m_view->setHtml("<html><body><a href='http://www.trolltech.com'>Trolltech</a></body></html>");
    QCOMPARE(m_page->mainFrame()->evaluateJavaScript("document.querySelectorAll(':visited').length;").toString(),
             QString::fromLatin1("1"));
}

QTEST_MAIN(tst_QWebHistoryInterface)
#include "tst_qwebhistoryinterface.moc"
