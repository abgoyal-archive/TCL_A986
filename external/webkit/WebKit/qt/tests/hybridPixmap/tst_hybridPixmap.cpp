

#include "../util.h"

#include "widget.h"
#include <QtTest/QtTest>

class tst_hybridPixmap : public QObject {
    Q_OBJECT

public:
    tst_hybridPixmap(QObject* o = 0) : QObject(o) {}

public slots:
    void init()
    {
    }

    void cleanup()
    {
    }

private slots:
    void hybridPixmap()
    {
        Widget widget;
        widget.show();
        widget.start();
        waitForSignal(&widget, SIGNAL(testComplete()));
    }
};

QTEST_MAIN(tst_hybridPixmap)

#include <tst_hybridPixmap.moc>
