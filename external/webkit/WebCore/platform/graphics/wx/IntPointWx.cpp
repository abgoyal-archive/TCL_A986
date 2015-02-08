

#include "config.h"
#include "IntPoint.h"

#include <wx/defs.h>
#include <wx/gdicmn.h>

namespace WebCore {

IntPoint::IntPoint(const wxPoint& p)
    : m_x(p.x)
    , m_y(p.y)
{
}

IntPoint::operator wxPoint() const
{
    return wxPoint(m_x, m_y);
}

}
