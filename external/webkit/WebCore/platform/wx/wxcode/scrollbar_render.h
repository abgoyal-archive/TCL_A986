

#ifndef scrollbar_render_h
#define scrollbar_render_h

#include <wx/defs.h>
#include <wx/dc.h>
#include <wx/renderer.h>

enum wxScrollbarPart {
    wxSCROLLPART_NONE = 0,
    wxSCROLLPART_BACKBTNSTART = 1,
    wxSCROLLPART_FWDBTNSTART = 1 << 1,
    wxSCROLLPART_BACKTRACK = 1 << 2,
    wxSCROLLPART_THUMB = 1 << 3,
    wxSCROLLPART_FWDTRACK = 1 << 4,
    wxSCROLLPART_BACKBTNEND = 1 << 5,
    wxSCROLLPART_FWDBTNEND = 1 << 6,
    wxSCROLLPART_SCROLLBARBG = 1 << 7,
    wxSCROLLPART_TRACKBG = 1 << 8,
    wxSCROLLPART_ALL = 0xffffffff,
};

void wxRenderer_DrawScrollbar(wxWindow* window, wxDC& dc, 
                                       const wxRect& rect, 
                                       wxOrientation orientation, 
                                       int current, wxScrollbarPart focusPart, wxScrollbarPart hoverPart, 
                                       int max, int step, int flags=0);

inline void calcThumbStartAndLength(int physicalLength, int max, int current, 
                                int step, int *thumbStart, int *thumbLength)
{
        float proportion = ((float)physicalLength - 8)/ (max + step);
        float thumbSize =  proportion * (float)physicalLength;
        if (thumbSize < 20)
            thumbSize = 20;

        float thumbPos = ((float)current / (float)max) * ((float)physicalLength - thumbSize);
        if (thumbStart)
            *thumbStart = thumbPos;
        
        if (thumbLength)
            *thumbLength = thumbSize;
}
#endif
