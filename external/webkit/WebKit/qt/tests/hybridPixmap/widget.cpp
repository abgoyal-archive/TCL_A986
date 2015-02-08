

#include "widget.h"

#include "qwebelement.h"
#include "qwebframe.h"
#include "ui_widget.h"
#include <QPainter>
#include <QtTest/QtTest>

Widget::Widget(QWidget* parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
}

void Widget::refreshJS()
{
    ui->webView->page()->mainFrame()->addToJavaScriptWindowObject("myWidget", this);
}
void Widget::start()
{
    ui->webView->load(QUrl("qrc:///test.html"));
    connect(ui->webView->page()->mainFrame(), SIGNAL(javaScriptWindowObjectCleared()), this, SLOT(refreshJS()));
    ui->webView->page()->mainFrame()->addToJavaScriptWindowObject("myWidget", this);
}

void Widget::completeTest()
{
    QCOMPARE(ui->lbl1->pixmap()->size(), ui->lbl2->size());
    QCOMPARE(ui->lbl3->size(), ui->lbl4->pixmap()->size());
    QCOMPARE(ui->lbl2->size().width(), ui->webView->page()->mainFrame()->findFirstElement("#img1").evaluateJavaScript("this.width").toInt());
    QCOMPARE(ui->lbl3->size().width(), ui->webView->page()->mainFrame()->findFirstElement("#img2").evaluateJavaScript("this.width").toInt());
    emit testComplete();
}

void Widget::setPixmap(const QPixmap& p)
{
    ui->lbl1->setPixmap(p);
}
QPixmap Widget::pixmap() const
{
    QPixmap px(ui->lbl3->size());
    {
        QPainter p(&px);
        ui->lbl3->render(&p);
    }
    return px;
}
void Widget::setImage(const QImage& img)
{
    ui->lbl4->setPixmap(QPixmap::fromImage(img));
}

QImage Widget::image() const
{
    QImage img(ui->lbl2->size(), QImage::Format_ARGB32);
    {
        QPainter p(&img);
        ui->lbl2->render(&p);
    }
    return img;
}

Widget::~Widget()
{
    delete ui;
}

void Widget::changeEvent(QEvent* e)
{
    QWidget::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}
void Widget::compare(const QVariant& a, const QVariant& b)
{
    QCOMPARE(a, b);
}

void Widget::imageSlot(const QImage& img)
{
    QCOMPARE(img.size(), ui->lbl3->size());
    emit pixmapSignal(QPixmap::fromImage(img));
}

void Widget::pixmapSlot(const QPixmap& pxm)
{
    QCOMPARE(pxm.size(), ui->lbl2->size());
    emit imageSignal(ui->lbl4->pixmap()->toImage());
}

void Widget::randomSlot(const QPixmap& pxm)
{
    QVERIFY(pxm.isNull());
}
