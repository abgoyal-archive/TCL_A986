

#include "config.h"
#include "Cursor.h"
#include "Image.h"

#include <wx/defs.h>
#include <wx/cursor.h>
#include <wx/image.h>

namespace WebCore {

Cursor::Cursor(const Cursor& other)
    : m_impl(other.m_impl)
{
}

Cursor::Cursor(Image* image, const IntPoint&) 
{
    m_impl = 0;
    // FIXME: figure out why the below code causes a crash  
    //m_impl = new wxCursor( image->getWxBitmap()->ConvertToImage() );
}

Cursor::~Cursor()
{
}

Cursor& Cursor::operator=(const Cursor& other)
{
    m_impl = other.m_impl;
    return *this;
}

Cursor::Cursor(wxCursor* c)
    : m_impl(c)
{
}

const Cursor& pointerCursor()
{
    static Cursor c = new wxCursor(wxCURSOR_ARROW);
    return c;
}

const Cursor& crossCursor()
{
    static Cursor c = new wxCursor(wxCURSOR_CROSS);
    return c;
}

const Cursor& handCursor()
{
    static Cursor c = new wxCursor(wxCURSOR_HAND);
    return c;
}

const Cursor& iBeamCursor()
{
    static Cursor c = new wxCursor(wxCURSOR_IBEAM);
    return c;
}

const Cursor& waitCursor()
{
    static Cursor c = new wxCursor(wxCURSOR_WAIT);
    return c;
}

const Cursor& helpCursor()
{
    static Cursor c = new wxCursor(wxCURSOR_QUESTION_ARROW);
    return c;
}

const Cursor& eastResizeCursor()
{
    static Cursor c = new wxCursor(wxCURSOR_SIZEWE);
    return c;
}

const Cursor& northResizeCursor()
{
    static Cursor c = new wxCursor(wxCURSOR_SIZENS);
    return c;
}

const Cursor& northEastResizeCursor()
{
    static Cursor c = new wxCursor(wxCURSOR_SIZENESW);
    return c;
}

const Cursor& northWestResizeCursor()
{
    static Cursor c = new wxCursor(wxCURSOR_SIZENWSE);
    return c;
}

const Cursor& southResizeCursor()
{
    static Cursor c = northResizeCursor();
    return c;
}

const Cursor& southEastResizeCursor()
{
    static Cursor c = northWestResizeCursor();
    return c;
}

const Cursor& southWestResizeCursor()
{
    static Cursor c = northEastResizeCursor();
    return c;
}

const Cursor& westResizeCursor()
{
    static Cursor c = eastResizeCursor();
    return c;
}

const Cursor& northSouthResizeCursor()
{
    static Cursor c = northResizeCursor();
    return c;
}

const Cursor& eastWestResizeCursor()
{
    static Cursor c = eastResizeCursor();
    return c;
}

const Cursor& northEastSouthWestResizeCursor()
{
    static Cursor c = northEastResizeCursor();
    return c;
}

const Cursor& northWestSouthEastResizeCursor()
{
    static Cursor c = northWestResizeCursor();
    return c;
}

const Cursor& columnResizeCursor()
{
    // FIXME: Windows does not have a standard column resize cursor
    static Cursor c = new wxCursor(wxCURSOR_SIZING);
    return c;
}

const Cursor& rowResizeCursor()
{
    // FIXME: Windows does not have a standard row resize cursor
    static Cursor c = new wxCursor(wxCURSOR_SIZING);
    return c;
}
    
const Cursor& middlePanningCursor()
{
    return moveCursor();
}

const Cursor& eastPanningCursor()
{
    return eastResizeCursor();
}

const Cursor& northPanningCursor()
{
    return northResizeCursor();
}

const Cursor& northEastPanningCursor()
{
    return northEastResizeCursor();
}

const Cursor& northWestPanningCursor()
{
    return northWestResizeCursor();
}

const Cursor& southPanningCursor()
{
    return southResizeCursor();
}

const Cursor& southEastPanningCursor()
{
    return southEastResizeCursor();
}

const Cursor& southWestPanningCursor()
{
    return southWestResizeCursor();
}

const Cursor& westPanningCursor()
{
    return westResizeCursor();
}    

const Cursor& verticalTextCursor()
{
    return iBeamCursor();
}

const Cursor& cellCursor()
{
    return pointerCursor();
}

const Cursor& contextMenuCursor()
{
    return handCursor();
}

const Cursor& noDropCursor()
{
    return pointerCursor();
}

const Cursor& progressCursor()
{
    return waitCursor();
}

const Cursor& aliasCursor()
{
    return pointerCursor();
}

const Cursor& copyCursor()
{
    return pointerCursor();
}

const Cursor& noneCursor()
{
    // TODO: Determine if we can find a better cursor for this.
    return pointerCursor();
}

const Cursor& notAllowedCursor()
{
    static Cursor c = new wxCursor(wxCURSOR_NO_ENTRY);
    return c;
}

const Cursor& zoomInCursor()
{
    static Cursor c = new wxCursor(wxCURSOR_MAGNIFIER);
    return c;
}

const Cursor& zoomOutCursor()
{
    // TODO: Find a way to implement in/out magnifiers in wx.
    return zoomInCursor();
}

const Cursor& grabCursor()
{
    // TODO: Determine if we can find a better cursor for this.
    return pointerCursor();
}

const Cursor& grabbingCursor()
{
    // TODO: Determine if we can find a better cursor for this.
    return pointerCursor();
}

const Cursor& moveCursor()
{
    static Cursor c = new wxCursor(wxCURSOR_SIZING);
    return c;
}

}
