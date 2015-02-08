

#include "config.h"
#include "InspectorClientQt.h"

#include "qwebinspector.h"
#include "qwebinspector_p.h"
#include "qwebpage.h"
#include "qwebpage_p.h"
#include "qwebview.h"

#include <QtCore/QSettings>
#include <QtCore/QCoreApplication>

#include "InspectorController.h"
#include "NotImplemented.h"
#include "Page.h"
#include "PlatformString.h"

namespace WebCore {

static const QLatin1String settingStoragePrefix("Qt/QtWebKit/QWebInspector/");
static const QLatin1String settingStorageTypeSuffix(".type");

static String variantToSetting(const QVariant& qvariant);
static QVariant settingToVariant(const String& value);

class InspectorClientWebPage : public QWebPage {
    Q_OBJECT
    friend class InspectorClientQt;
public:
    InspectorClientWebPage(QObject* parent = 0)
        : QWebPage(parent)
    {
    }

    QWebPage* createWindow(QWebPage::WebWindowType)
    {
        QWebView* view = new QWebView;
        QWebPage* page = new QWebPage;
        view->setPage(page);
        view->setAttribute(Qt::WA_DeleteOnClose);
        return page;
    }
};

InspectorClientQt::InspectorClientQt(QWebPage* page)
    : m_inspectedWebPage(page)
{}

void InspectorClientQt::inspectorDestroyed()
{
    delete this;
}

Page* InspectorClientQt::createPage()
{
    QWebView* inspectorView = new QWebView;
    InspectorClientWebPage* inspectorPage = new InspectorClientWebPage(inspectorView);
    inspectorView->setPage(inspectorPage);
    m_inspectorView.set(inspectorView);

    inspectorPage->mainFrame()->load(QString::fromLatin1("qrc:/webkit/inspector/inspector.html"));
    m_inspectedWebPage->d->inspectorFrontend = inspectorView;
    m_inspectedWebPage->d->getOrCreateInspector()->d->setFrontend(inspectorView);

    return m_inspectorView->page()->d->page;
}

String InspectorClientQt::localizedStringsURL()
{
    notImplemented();
    return String();
}

String InspectorClientQt::hiddenPanels()
{
    notImplemented();
    return String();
}

void InspectorClientQt::showWindow()
{
    updateWindowTitle();

#if ENABLE(INSPECTOR)
    m_inspectedWebPage->d->inspectorController()->setWindowVisible(true, true);
#endif
}

void InspectorClientQt::closeWindow()
{
#if ENABLE(INSPECTOR)
    m_inspectedWebPage->d->inspectorController()->setWindowVisible(false);
#endif
}

void InspectorClientQt::attachWindow()
{
    notImplemented();
}

void InspectorClientQt::detachWindow()
{
    notImplemented();
}

void InspectorClientQt::setAttachedWindowHeight(unsigned)
{
    notImplemented();
}

void InspectorClientQt::highlight(Node*)
{
    notImplemented();
}

void InspectorClientQt::hideHighlight()
{
    notImplemented();
}

void InspectorClientQt::inspectedURLChanged(const String& newURL)
{
    m_inspectedURL = newURL;
    updateWindowTitle();
}

void InspectorClientQt::inspectorWindowObjectCleared()
{
    notImplemented();
}

void InspectorClientQt::updateWindowTitle()
{
    if (m_inspectedWebPage->d->inspector) {
        QString caption = QCoreApplication::translate("QWebPage", "Web Inspector - %2").arg(m_inspectedURL);
        m_inspectedWebPage->d->inspector->setWindowTitle(caption);
    }
}

void InspectorClientQt::populateSetting(const String& key, String* setting)
{
    QSettings qsettings;
    if (qsettings.status() == QSettings::AccessError) {
        // QCoreApplication::setOrganizationName and QCoreApplication::setApplicationName haven't been called
        qWarning("QWebInspector: QSettings couldn't read configuration setting [%s].",
                 qPrintable(static_cast<QString>(key)));
        return;
    }

    QString settingKey(settingStoragePrefix + key);
    QString storedValueType = qsettings.value(settingKey + settingStorageTypeSuffix).toString();
    QVariant storedValue = qsettings.value(settingKey);
    storedValue.convert(QVariant::nameToType(storedValueType.toAscii().data()));
    *setting = variantToSetting(storedValue);
}

void InspectorClientQt::storeSetting(const String& key, const String& setting)
{
    QSettings qsettings;
    if (qsettings.status() == QSettings::AccessError) {
        qWarning("QWebInspector: QSettings couldn't persist configuration setting [%s].",
                 qPrintable(static_cast<QString>(key)));
        return;
    }

    QVariant valueToStore = settingToVariant(setting);
    QString settingKey(settingStoragePrefix + key);
    qsettings.setValue(settingKey, valueToStore);
    qsettings.setValue(settingKey + settingStorageTypeSuffix, QVariant::typeToName(valueToStore.type()));
}

static String variantToSetting(const QVariant& qvariant)
{
    String retVal;

    switch (qvariant.type()) {
    case QVariant::Bool:
        retVal = qvariant.toBool() ? "true" : "false";
    case QVariant::String:
        retVal = qvariant.toString();
    }

    return retVal;
}

static QVariant settingToVariant(const String& setting)
{
    QVariant retVal;
    retVal.setValue(static_cast<QString>(setting));
    return retVal;
}

}

#include "InspectorClientQt.moc"
