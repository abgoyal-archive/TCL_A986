

#ifndef DashboardRegion_h
#define DashboardRegion_h

#include "Rect.h"

#if ENABLE(DASHBOARD_SUPPORT)

namespace WebCore {

class DashboardRegion : public RectBase, public RefCounted<DashboardRegion> {
public:
    static PassRefPtr<DashboardRegion> create() { return adoptRef(new DashboardRegion); }

    RefPtr<DashboardRegion> m_next;
    String m_label;
    String m_geometryType;
    bool m_isCircle : 1;
    bool m_isRectangle : 1;

private:
    DashboardRegion() : m_isCircle(false), m_isRectangle(false) { }
};

} // namespace

#endif

#endif
