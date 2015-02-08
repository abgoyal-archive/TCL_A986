

#include <QtTest/QtTest>

#include <qdir.h>
#include <qwebinspector.h>
#include <qwebpage.h>
#include <qwebsettings.h>

class tst_QWebInspector : public QObject {
    Q_OBJECT

private slots:
    void attachAndDestroy();
};

void tst_QWebInspector::attachAndDestroy()
{
    {   // External inspector + manual destruction of page first
        QWebPage* page = new QWebPage();
        page->settings()->setAttribute(QWebSettings::DeveloperExtrasEnabled, true);
        QWebInspector* inspector = new QWebInspector();
        inspector->setPage(page);
        page->updatePositionDependentActions(QPoint(0, 0));
        page->triggerAction(QWebPage::InspectElement);

        delete page;
        delete inspector;
    }
    {   // External inspector + manual destruction of inspector first
        QWebPage* page = new QWebPage();
        page->settings()->setAttribute(QWebSettings::DeveloperExtrasEnabled, true);
        QWebInspector* inspector = new QWebInspector();
        inspector->setPage(page);
        page->updatePositionDependentActions(QPoint(0, 0));
        page->triggerAction(QWebPage::InspectElement);

        delete inspector;
        delete page;
    }
    {   // Internal inspector
        QWebPage page;
        page.settings()->setAttribute(QWebSettings::DeveloperExtrasEnabled, true);
        page.updatePositionDependentActions(QPoint(0, 0));
        page.triggerAction(QWebPage::InspectElement);
    }
}

QTEST_MAIN(tst_QWebInspector)

#include "tst_qwebinspector.moc"
