

#ifndef PNGImageDecoder_h
#define PNGImageDecoder_h

#include "ImageDecoder.h"
#include <wtf/OwnPtr.h>

namespace WebCore {

    class PNGImageReader;

    // This class decodes the PNG image format.
    class PNGImageDecoder : public ImageDecoder {
    public:
        PNGImageDecoder();
        ~PNGImageDecoder();

        virtual String filenameExtension() const { return "png"; }

        // Take the data and store it.
        virtual void setData(SharedBuffer* data, bool allDataReceived);

        // Whether or not the size information has been decoded yet.
        virtual bool isSizeAvailable();

        virtual RGBA32Buffer* frameBufferAtIndex(size_t index);

        void decode(bool sizeOnly = false);

        // Callbacks from libpng
        void decodingFailed();
        void headerAvailable();
        void rowAvailable(unsigned char* rowBuffer, unsigned rowIndex, int interlacePass);
        void pngComplete();

    private:
        OwnPtr<PNGImageReader> m_reader;
    };

} // namespace WebCore

#endif
