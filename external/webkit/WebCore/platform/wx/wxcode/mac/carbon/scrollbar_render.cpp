


#include "config.h"

#include "scrollbar_render.h"

#include <wx/defs.h>
#include <wx/dc.h>
#include <wx/dcgraph.h>
#include <wx/renderer.h>

#include <Carbon/Carbon.h>

static int wxScrollbarPartToHIPressedState(wxScrollbarPart part)
{
    switch (part) {
        case wxSCROLLPART_BACKBTNSTART:
            return kThemeTopOutsideArrowPressed;
        case wxSCROLLPART_BACKBTNEND:
            return kThemeTopOutsideArrowPressed; // This does not make much sense.  For some reason the outside constant is required.
        case wxSCROLLPART_FWDBTNSTART:
            return kThemeTopInsideArrowPressed;
        case wxSCROLLPART_FWDBTNEND:
            return kThemeBottomOutsideArrowPressed;
        case wxSCROLLPART_THUMB:
            return kThemeThumbPressed;
        default:
            return 0;
    }
}

void wxRenderer_DrawScrollbar(wxWindow* WXUNUSED(window), wxDC& dc,
                             const wxRect& rect, wxOrientation orient, int current,
                             wxScrollbarPart focusPart, wxScrollbarPart hoverPart, int max, int step, int flags)
{
    const wxCoord x = rect.x;
    const wxCoord y = rect.y;
    const wxCoord w = rect.width;
    const wxCoord h = rect.height;

    dc.SetBrush( *wxWHITE_BRUSH );
    dc.SetPen( *wxTRANSPARENT_PEN );
    dc.DrawRectangle(rect);
    
    dc.SetBrush( *wxTRANSPARENT_BRUSH );

    HIRect hiRect = CGRectMake( x, y, w, h );

    CGContextRef cgContext = NULL;
    wxGraphicsContext* gc = NULL;
#if wxCHECK_VERSION(2,9,0)
    wxGCDCImpl *impl = dynamic_cast<wxGCDCImpl*> (dc.GetImpl());
    if (impl)
        gc = impl->GetGraphicsContext();
#else
    gc = dc.GetGraphicsContext();
#endif
    if (gc)
        cgContext = (CGContextRef) gc->GetNativeContext();
    
    if (cgContext)
    {
        HIThemeTrackDrawInfo trackInfo;
        trackInfo.version = 0;
        trackInfo.kind = kThemeMediumScrollBar;
        trackInfo.bounds = hiRect;
        trackInfo.min = 0;
        trackInfo.max = max;
        trackInfo.value = current;
        trackInfo.trackInfo.scrollbar.viewsize = step;
        trackInfo.attributes = 0;
        if (orient == wxHORIZONTAL)
            trackInfo.attributes |= kThemeTrackHorizontal;
        trackInfo.enableState = (flags & wxCONTROL_FOCUSED) ? kThemeTrackActive : kThemeTrackInactive;
        trackInfo.trackInfo.scrollbar.pressState = wxScrollbarPartToHIPressedState(focusPart);
        trackInfo.attributes |= kThemeTrackShowThumb;
        
        if (flags & wxCONTROL_DISABLED)
            trackInfo.enableState = kThemeTrackDisabled;
        
        HIThemeDrawTrack(&trackInfo, 0, cgContext, kHIThemeOrientationNormal);
    }
}
