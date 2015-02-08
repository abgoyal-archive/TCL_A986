

#ifndef StyleDashboardRegion_h
#define StyleDashboardRegion_h
#if ENABLE(DASHBOARD_SUPPORT)

#include "LengthBox.h"
#include "PlatformString.h"

namespace WebCore {

// Dashboard region attributes. Not inherited.

struct StyleDashboardRegion {
    String label;
    LengthBox offset;
    int type;

    enum {
        None,
        Circle,
        Rectangle
    };

    bool operator==(const StyleDashboardRegion& o) const
    {
        return type == o.type && offset == o.offset && label == o.label;
    }

    bool operator!=(const StyleDashboardRegion& o) const
    {
        return !(*this == o);
    }
};

} // namespace WebCore

#endif // ENABLE(DASHBOARD_SUPPORT)
#endif // StyleDashboardRegion_h
