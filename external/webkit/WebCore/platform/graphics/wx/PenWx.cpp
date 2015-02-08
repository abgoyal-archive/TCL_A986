

#include "config.h"
#include "Pen.h"

#include <wx/defs.h>
#include <wx/pen.h>
#include <wx/colour.h>
 
namespace WebCore {

// Pen style conversion functions
static int penStyleToWxPenStyle(int p)
{
    if (p == Pen::SolidLine)
        return wxSOLID;
    if (p == Pen::DotLine)
        return wxDOT;
    if (p == Pen::DashLine)
        return wxLONG_DASH;
    if (p == Pen::NoPen)
        return wxTRANSPARENT;
    
    return wxSOLID;
}

static Pen::PenStyle wxPenStyleToPenStyle(int p)
{
    if (p == wxSOLID)
        return Pen::SolidLine;
    if (p == wxDOT)
        return Pen::DotLine;
    if (p == wxLONG_DASH || p == wxSHORT_DASH || p == wxDOT_DASH || p == wxUSER_DASH)
        return Pen::DashLine;
    if (p == wxTRANSPARENT)
        return Pen::NoPen; 
    
    return Pen::SolidLine;
}

Pen::Pen(const wxPen& p)
{
    wxColour color = p.GetColour();
    setColor(Color(color.Red(), color.Green(), color.Blue()));
    setWidth(p.GetWidth());
    setStyle(wxPenStyleToPenStyle(p.GetStyle()));
}

Pen::operator wxPen() const
{
    return wxPen(wxColour(m_color.red(), m_color.blue(), m_color.green()), width(), penStyleToWxPenStyle(style()));
}

}
