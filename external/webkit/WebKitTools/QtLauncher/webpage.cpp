

#include "webpage.h"

#include <QDesktopServices>
#include <QtGui>
#include <QtNetwork/QNetworkRequest>
#include <QtNetwork/QNetworkProxy>

WebPage::WebPage(QObject* parent)
    : QWebPage(parent)
{
    applyProxy();
}

void WebPage::applyProxy()
{
    QUrl proxyUrl(qgetenv("http_proxy"));

    if (proxyUrl.isValid() && !proxyUrl.host().isEmpty()) {
        int proxyPort = (proxyUrl.port() > 0) ? proxyUrl.port() : 8080;
        networkAccessManager()->setProxy(QNetworkProxy(QNetworkProxy::HttpProxy, proxyUrl.host(), proxyPort));
    }
}

bool WebPage::supportsExtension(QWebPage::Extension extension) const
{
    if (extension == QWebPage::ErrorPageExtension)
        return true;
    return false;
}

bool WebPage::extension(Extension extension, const ExtensionOption* option, ExtensionReturn* output)
{
    const QWebPage::ErrorPageExtensionOption* info = static_cast<const QWebPage::ErrorPageExtensionOption*>(option);
    QWebPage::ErrorPageExtensionReturn* errorPage = static_cast<QWebPage::ErrorPageExtensionReturn*>(output);

    errorPage->content = QString("<html><head><title>Failed loading page</title></head><body>%1</body></html>")
        .arg(info->errorString).toUtf8();

    return true;
}

bool WebPage::acceptNavigationRequest(QWebFrame* frame, const QNetworkRequest& request, NavigationType type)
{
    QObject* view = parent();

    QVariant value = view->property("keyboardModifiers");

    if (!value.isNull()) {
        Qt::KeyboardModifiers modifiers = Qt::KeyboardModifiers(value.toInt());

        if (modifiers & Qt::ShiftModifier) {
            QWebPage* page = createWindow(QWebPage::WebBrowserWindow);
            page->mainFrame()->load(request);
            return false;
        }

        if (modifiers & Qt::AltModifier) {
            openUrlInDefaultBrowser(request.url());
            return false;
        }
    }

    return QWebPage::acceptNavigationRequest(frame, request, type);
}

void WebPage::openUrlInDefaultBrowser(const QUrl& url)
{
    if (QAction* action = qobject_cast<QAction*>(sender()))
        QDesktopServices::openUrl(action->data().toUrl());
    else
        QDesktopServices::openUrl(url);
}


