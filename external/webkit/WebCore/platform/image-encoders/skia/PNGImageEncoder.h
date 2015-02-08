

#ifndef PNGImageEncoder_h
#define PNGImageEncoder_h

#include "Vector.h"

class IntSize;
class SkBitmap;

namespace WebCore {

    // Interface for encoding PNG data. This is a wrapper around libpng.
    class PNGImageEncoder {
    public:
        // Encodes the specific SkBitmap into the supplied vector.
        static bool encode(const SkBitmap&, WTF::Vector<unsigned char>* output);

        // Encodes the specified image data into the supplied vector.
        // w, h give the size of the image and bytes_per_row gives the bytes
        // per row.
        static bool encode(const unsigned char* input, const IntSize& size, int bytesPerRow, WTF::Vector<unsigned char>* output);
    };

}  // namespace WebCore

#endif
