

#include "locationedit.h"

LocationEdit::LocationEdit(QWidget* parent)
    : QLineEdit(parent)
    , m_progress(0)
{
    m_clearTimer.setSingleShot(true);
    connect(&m_clearTimer, SIGNAL(timeout()), this, SLOT(reset()));
}

void LocationEdit::setProgress(int progress)
{
    m_clearTimer.stop();
    m_progress = progress;
    update();
}

void LocationEdit::reset()
{
    setProgress(0);
}

void LocationEdit::paintEvent(QPaintEvent* ev)
{
    QColor backgroundColor = QApplication::palette().color(QPalette::Base);
    QColor progressColor = QColor(120, 180, 240);
    QPalette p = palette();

    if (!m_progress)
        p.setBrush(QPalette::Base, backgroundColor);
    else {
        QLinearGradient gradient(0, 0, width(), 0);
        gradient.setColorAt(0, progressColor);
        gradient.setColorAt(((double) m_progress) / 100, progressColor);
        if (m_progress != 100)
            gradient.setColorAt((double) m_progress / 100 + 0.001, backgroundColor);
        p.setBrush(QPalette::Base, gradient);
    }
    setPalette(p);

    QLineEdit::paintEvent(ev);

    if (m_progress == 100)
        m_clearTimer.start(100);
}

void LocationEdit::focusInEvent(QFocusEvent* ev)
{
    QLineEdit::focusInEvent(ev);
#ifdef Q_WS_MAEMO_5
    QTimer::singleShot(0, this, SLOT(selectAll()));
#endif
}
