

#ifndef webpage_h
#define webpage_h

#include <qwebframe.h>
#include <qwebpage.h>

class WebPage : public QWebPage {
    Q_OBJECT

public:
    WebPage(QObject* parent = 0);

    virtual QWebPage* createWindow(QWebPage::WebWindowType);
    virtual QObject* createPlugin(const QString&, const QUrl&, const QStringList&, const QStringList&);
    virtual bool supportsExtension(QWebPage::Extension extension) const;
    virtual bool extension(Extension extension, const ExtensionOption* option, ExtensionReturn* output);

    virtual bool acceptNavigationRequest(QWebFrame* frame, const QNetworkRequest& request, NavigationType type);

public slots:
    void openUrlInDefaultBrowser(const QUrl& url = QUrl());

private:
    void applyProxy();
};

#endif
