

#include <QApplication>
#include <QUrl>
#include <QWebView>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QWidget *parent = 0;
//! [Using QWebView]
    QWebView *view = new QWebView(parent);
    view->load(QUrl("http://qt.nokia.com/"));
    view->show();
//! [Using QWebView]
    return app.exec();
}
