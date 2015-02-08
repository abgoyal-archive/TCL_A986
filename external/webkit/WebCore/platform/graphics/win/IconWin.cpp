

#include "config.h"
#include "Icon.h"

#include "GraphicsContext.h"
#include "PlatformString.h"
#include <tchar.h>
#include <windows.h>

#if OS(WINCE)
// SHGFI_SHELLICONSIZE is not available on WINCE
#define SHGFI_SHELLICONSIZE         0
#endif

namespace WebCore {

static const int shell32MultipleFileIconIndex = 54;

Icon::Icon(HICON icon)
    : m_hIcon(icon)
{
    ASSERT(icon);
}

Icon::~Icon()
{
    DestroyIcon(m_hIcon);
}

PassRefPtr<Icon> Icon::createIconForFiles(const Vector<String>& filenames)
{
    if (filenames.isEmpty())
        return 0;

    if (filenames.size() == 1) {
        SHFILEINFO sfi;
        memset(&sfi, 0, sizeof(sfi));

        String tmpFilename = filenames[0];
        if (!SHGetFileInfo(tmpFilename.charactersWithNullTermination(), 0, &sfi, sizeof(sfi), SHGFI_ICON | SHGFI_SHELLICONSIZE | SHGFI_SMALLICON))
            return 0;

        return adoptRef(new Icon(sfi.hIcon));
    }

#if OS(WINCE)
    return 0;
#else
    TCHAR buffer[MAX_PATH];    
    UINT length = ::GetSystemDirectory(buffer, ARRAYSIZE(buffer));
    if (!length)
        return 0;
    
    if (_tcscat_s(buffer, TEXT("\\shell32.dll")))
        return 0;

    HICON hIcon;
    if (!::ExtractIconEx(buffer, shell32MultipleFileIconIndex, 0, &hIcon, 1))
        return 0;
    return adoptRef(new Icon(hIcon));
#endif
}

void Icon::paint(GraphicsContext* context, const IntRect& r)
{
    if (context->paintingDisabled())
        return;

#if OS(WINCE)
    context->drawIcon(m_hIcon, r, DI_NORMAL);
#else
    HDC hdc = context->getWindowsContext(r);

    DrawIconEx(hdc, r.x(), r.y(), m_hIcon, r.width(), r.height(), 0, 0, DI_NORMAL);

    context->releaseWindowsContext(hdc, r);
#endif
}

}
