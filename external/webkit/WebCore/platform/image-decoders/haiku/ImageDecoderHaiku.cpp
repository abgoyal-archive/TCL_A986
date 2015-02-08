

#include "config.h"
#include "ImageDecoder.h"

#include <Bitmap.h>

namespace WebCore {

NativeImagePtr RGBA32Buffer::asNewNativeImage() const
{
    int bytesPerRow = width() * sizeof(PixelData);
    OwnPtr<BBitmap> bitmap(new BBitmap(BRect(0, 0, width() - 1, height() - 1), 0, B_RGBA32, bytesPerRow));

    const uint8* source = reinterpret_cast<const uint8*>(m_bytes.data());
    uint8* destination = reinterpret_cast<uint8*>(bitmap->Bits());
    int h = height();
    int w = width();
    for (int y = 0; y < h; y++) {
#if 0
// FIXME: Enable this conversion once Haiku has B_RGBA32P[remultiplied]...
        memcpy(dst, source, bytesPerRow);
#else
        const uint8* sourceHandle = source;
        uint8* destinationHandle = destination;
        for (int x = 0; x < w; x++) {
            if (sourceHandle[3] == 255 || !sourceHandle[3]) {
                destinationHandle[0] = sourceHandle[0];
                destinationHandle[1] = sourceHandle[1];
                destinationHandle[2] = sourceHandle[2];
                destinationHandle[3] = sourceHandle[3];
            } else {
                destinationHandle[0] = static_cast<uint16>(sourceHandle[0]) * 255 / sourceHandle[3];
                destinationHandle[1] = static_cast<uint16>(sourceHandle[1]) * 255 / sourceHandle[3];
                destinationHandle[2] = static_cast<uint16>(sourceHandle[2]) * 255 / sourceHandle[3];
                destinationHandle[3] = sourceHandle[3];
            }
            destinationHandle += 4;
            sourceHandle += 4;
        }
#endif
        destination += bytesPerRow;
        source += bytesPerRow;
    }

    return bitmap.release();
}

} // namespace WebCore

