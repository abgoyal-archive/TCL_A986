

#include "config.h"
#include "FontCustomPlatformData.h"

#include "SkTypeface.h"
#include "SkStream.h"
#include "SharedBuffer.h"
#include "FontPlatformData.h"

namespace WebCore {

FontCustomPlatformData::FontCustomPlatformData(SkTypeface* face)
{
    face->safeRef();
    m_typeface = face;
}

FontCustomPlatformData::~FontCustomPlatformData()
{
    m_typeface->safeUnref();
    // the unref is enough to release the font data...
}

FontPlatformData FontCustomPlatformData::fontPlatformData(int size, bool bold, bool italic,
    FontRenderingMode mode)
{
    // turn bold/italic into fakeBold/fakeItalic
    if (m_typeface != NULL) {
        if (m_typeface->isBold() == bold)
            bold = false;
        if (m_typeface->isItalic() == italic)
            italic = false;
    }
    return FontPlatformData(m_typeface, size, bold, italic);
}

FontCustomPlatformData* createFontCustomPlatformData(SharedBuffer* buffer)
{
    // pass true until we know how we can share the data, and not have to
    // make a copy of it.
    SkStream* stream = new SkMemoryStream(buffer->data(), buffer->size(), true);
    SkTypeface* face = SkTypeface::CreateFromStream(stream);
    if (0 == face) {
        SkDebugf("--------- SkTypeface::CreateFromBuffer failed %d\n",
                 buffer->size());
        return NULL;
    }

    SkAutoUnref aur(face);

    return new FontCustomPlatformData(face);
}

}
