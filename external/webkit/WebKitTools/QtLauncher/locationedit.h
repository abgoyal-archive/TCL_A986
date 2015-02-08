

#ifndef locationedit_h
#define locationedit_h

#include <QtGui>

class LocationEdit : public QLineEdit {
    Q_OBJECT

public:
    LocationEdit(QWidget* parent = 0);

public slots:
    void setProgress(int progress);

private slots:
    void reset();

protected:
    virtual void paintEvent(QPaintEvent*);
    virtual void focusInEvent(QFocusEvent*);

private:
    int m_progress;
    QTimer m_clearTimer;
};

#endif
