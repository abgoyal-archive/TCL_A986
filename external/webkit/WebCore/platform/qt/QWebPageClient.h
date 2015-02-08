

#ifndef QWebPageClient_h
#define QWebPageClient_h

#ifndef QT_NO_CURSOR
#include <QCursor>
#endif

#include <QRect>

QT_BEGIN_NAMESPACE
class QGraphicsItem;
class QStyle;
QT_END_NAMESPACE

class QWebPageClient {
public:
    virtual ~QWebPageClient() { }
        
    virtual void scroll(int dx, int dy, const QRect&) = 0;
    virtual void update(const QRect&) = 0;
    virtual void setInputMethodEnabled(bool enable) = 0;
    virtual bool inputMethodEnabled() const = 0;
#if USE(ACCELERATED_COMPOSITING)
    // this gets called when we start/stop compositing.
    virtual void setRootGraphicsLayer(QGraphicsItem* layer) {}

    // this gets called when the compositor wants us to sync the layers
    // if scheduleSync is true, we schedule a sync ourselves. otherwise,
    // we wait for the next update and sync the layers then.
    virtual void markForSync(bool scheduleSync = false) {}
#endif

#if QT_VERSION >= 0x040600
    virtual void setInputMethodHint(Qt::InputMethodHint hint, bool enable) = 0;
#endif
    inline void resetCursor()
    {
#ifndef QT_NO_CURSOR
        if (!cursor().bitmap() && cursor().shape() == m_lastCursor.shape())
            return;
        updateCursor(m_lastCursor);
#endif
    }

    inline void setCursor(const QCursor& cursor)
    {
#ifndef QT_NO_CURSOR
        m_lastCursor = cursor;
        if (!cursor.bitmap() && cursor.shape() == this->cursor().shape())
            return;
        updateCursor(cursor);
#endif
    }

    virtual QPalette palette() const = 0;
    virtual int screenNumber() const = 0;
    virtual QWidget* ownerWidget() const = 0;

    virtual QObject* pluginParent() const = 0;

    virtual QStyle* style() const = 0;

protected:
#ifndef QT_NO_CURSOR
    virtual QCursor cursor() const = 0;
    virtual void updateCursor(const QCursor& cursor) = 0;
#endif

private:
#ifndef QT_NO_CURSOR
    QCursor m_lastCursor;
#endif
};

#endif
