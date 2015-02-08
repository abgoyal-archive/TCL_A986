
 
#ifndef WXWEBVIEWPRIVATE_H
#define WXWEBVIEWPRIVATE_H

#include "config.h"
#include "EditCommand.h"
#include "EditCommandWx.h"
#include "Page.h"
#include "wtf/Vector.h"

#include <wx/timer.h>

class WebViewPrivate 
{
public:
    WebViewPrivate() :
        page(0)
    {}
    
    WebCore::Page* page;

    wxTimer tripleClickTimer;
    wxPoint tripleClickPos;
    
    WTF::Vector<EditCommandWx> undoStack;
    WTF::Vector<EditCommandWx> redoStack;
};

#endif
