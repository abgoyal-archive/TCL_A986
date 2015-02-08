

#ifndef widget_h
#define widget_h

#include <QImage>
#include <QPixmap>
#include <QWidget>
#include "qwebview.h"

typedef QWebView WebView;

namespace Ui {
class Widget;
}

class Widget : public QWidget {
    Q_OBJECT
    Q_PROPERTY(QPixmap pixmap READ pixmap WRITE setPixmap)
    Q_PROPERTY(QImage image READ image WRITE setImage)

public:
    Widget(QWidget* parent = 0);
    ~Widget();
    void setPixmap(const QPixmap&);
    QPixmap pixmap() const;
    void setImage(const QImage&);
    QImage image() const;

private slots:
    void refreshJS();

public slots:
    void completeTest();
    void start();
    void compare(const QVariant& a, const QVariant& b);
    void imageSlot(const QImage&);
    void pixmapSlot(const QPixmap&);
    void randomSlot(const QPixmap&);

signals:
    void testComplete();
    void imageSignal(const QImage&);
    void pixmapSignal(const QPixmap&);

protected:
    void changeEvent(QEvent* e);

private:
    Ui::Widget* ui;
};

#endif // widget_h
