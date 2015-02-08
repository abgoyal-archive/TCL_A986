

#ifndef AndroidWebHistoryBridge_h
#define AndroidWebHistoryBridge_h

#include <wtf/RefCounted.h>

namespace WebCore {

class HistoryItem;

class AndroidWebHistoryBridge : public RefCounted<AndroidWebHistoryBridge> {
public:
    AndroidWebHistoryBridge(HistoryItem* item)
        : m_scale(0)
        , m_screenWidthScale(0)
        , m_active(false)
        , m_historyItem(item) { }
    virtual ~AndroidWebHistoryBridge() { }
    virtual void updateHistoryItem(HistoryItem* item) = 0;

    void setScale(int s) { m_scale = s; }
    void setScreenWidthScale(int s) { m_screenWidthScale = s; }
    int scale() const { return m_scale; }
    int screenWidthScale() const { return m_screenWidthScale; }
    void detachHistoryItem() { m_historyItem = 0; }
    HistoryItem* historyItem() const { return m_historyItem; }
    void setActive() { m_active = true; }

protected:
    int m_scale;
    int m_screenWidthScale;
    bool m_active;
    HistoryItem* m_historyItem;
};

} // namespace WebCore

#endif // AndroidWebHistoryBridge_h
