

#include "config.h"
#include "IntSize.h"

#include <wx/defs.h>
#include <wx/gdicmn.h>

namespace WebCore {

IntSize::IntSize(const wxSize& s)
    : m_width(s.x)
    , m_height(s.y)
{
}

IntSize::operator wxSize() const
{
    return wxSize(m_width, m_height);
}

}
