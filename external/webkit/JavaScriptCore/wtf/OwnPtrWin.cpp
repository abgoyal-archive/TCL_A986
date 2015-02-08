

#include "config.h"
#include "OwnPtr.h"

#include <windows.h>

namespace WTF {

void deleteOwnedPtr(HBITMAP ptr)
{
    if (ptr)
        DeleteObject(ptr);
}

void deleteOwnedPtr(HBRUSH ptr)
{
    if (ptr)
        DeleteObject(ptr);
}

void deleteOwnedPtr(HDC ptr)
{
    if (ptr)
        DeleteDC(ptr);
}

void deleteOwnedPtr(HFONT ptr)
{
    if (ptr)
        DeleteObject(ptr);
}

void deleteOwnedPtr(HPALETTE ptr)
{
    if (ptr)
        DeleteObject(ptr);
}

void deleteOwnedPtr(HPEN ptr)
{
    if (ptr)
        DeleteObject(ptr);
}

void deleteOwnedPtr(HRGN ptr)
{
    if (ptr)
        DeleteObject(ptr);
}

}
