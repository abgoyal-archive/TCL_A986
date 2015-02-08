

#ifndef JPEGImageDecoder_h
#define JPEGImageDecoder_h

#include "ImageDecoder.h"
#include <wtf/OwnPtr.h>

namespace WebCore {

    class JPEGImageReader;

    // This class decodes the JPEG image format.
    class JPEGImageDecoder : public ImageDecoder {
    public:
        JPEGImageDecoder();
        ~JPEGImageDecoder();

        virtual String filenameExtension() const { return "jpg"; }

        // Take the data and store it.
        virtual void setData(SharedBuffer* data, bool allDataReceived);

        // Whether or not the size information has been decoded yet.
        virtual bool isSizeAvailable();

        virtual bool setSize(unsigned width, unsigned height);

        virtual RGBA32Buffer* frameBufferAtIndex(size_t index);
        
        virtual bool supportsAlpha() const { return false; }

        void decode(bool sizeOnly = false);

        bool outputScanlines();
        void jpegComplete();

    private:
        OwnPtr<JPEGImageReader> m_reader;
    };

} // namespace WebCore

#endif
