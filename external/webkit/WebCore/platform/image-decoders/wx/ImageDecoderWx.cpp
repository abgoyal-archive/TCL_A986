

#include "config.h"
#include "ImageDecoder.h"

// FIXME: Are all these needed?
#include <wx/defs.h>
#include <wx/bitmap.h>
#if USE(WXGC)
#include <wx/graphics.h>
#endif
#include <wx/image.h>
#include <wx/rawbmp.h>

namespace WebCore {

NativeImagePtr RGBA32Buffer::asNewNativeImage() const
{
    wxBitmap* bmp = new wxBitmap(width(), height(), 32);

    {
        typedef wxPixelData<wxBitmap, wxAlphaPixelFormat> WxPixelData;
        WxPixelData data(*bmp);
        
        // NB: It appears that the data is in BGRA format instead of RGBA format.
        // This code works properly on both ppc and intel, meaning the issue is
        // likely not an issue of byte order getting mixed up on different archs. 
        const unsigned char* bytes = (const unsigned char*)m_bytes.data();
        int rowCounter = 0;
        long pixelCounter = 0;
        WxPixelData::Iterator p(data);
        WxPixelData::Iterator rowStart = p; 
        for (size_t i = 0; i < m_bytes.size() * sizeof(PixelData); i += sizeof(PixelData)) {
                p.Red() = bytes[i + 2];
                p.Green() = bytes[i + 1];
                p.Blue() = bytes[i + 0];
                p.Alpha() = bytes[i + 3];
            
            p++;

            pixelCounter++;
            if ((pixelCounter % width()) == 0) {
                rowCounter++;
                p = rowStart;
                p.MoveTo(data, 0, rowCounter);
            }
        }
#if !wxCHECK_VERSION(2,9,0)
        bmp->UseAlpha();
#endif
    } // ensure that WxPixelData is destroyed as it unlocks the bitmap data in
      // its dtor and we can't access it (notably in CreateBitmap() below)
      // before this is done

    ASSERT(bmp->IsOk());

#if USE(WXGC)
    wxGraphicsBitmap* bitmap = new wxGraphicsBitmap(wxGraphicsRenderer::GetDefaultRenderer()->CreateBitmap(*bmp));
    delete bmp;
    return bitmap;
#else
    return bmp;
#endif
}

} // namespace WebCore
