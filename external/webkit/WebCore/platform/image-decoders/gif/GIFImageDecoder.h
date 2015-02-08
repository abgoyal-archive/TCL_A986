

#ifndef GIFImageDecoder_h
#define GIFImageDecoder_h

#include "ImageDecoder.h"
#include <wtf/OwnPtr.h>

class GIFImageReader;

namespace WebCore {

    // This class decodes the GIF image format.
    class GIFImageDecoder : public ImageDecoder {
    public:
        GIFImageDecoder();
        ~GIFImageDecoder();

        virtual String filenameExtension() const { return "gif"; }

        // Take the data and store it.
        virtual void setData(SharedBuffer* data, bool allDataReceived);

        // Whether or not the size information has been decoded yet.
        virtual bool isSizeAvailable();

        // The total number of frames for the image.  Will scan the image data for the answer
        // (without necessarily decoding all of the individual frames).
        virtual size_t frameCount();

        // The number of repetitions to perform for an animation loop.
        virtual int repetitionCount() const;

        virtual RGBA32Buffer* frameBufferAtIndex(size_t index);

        virtual void clearFrameBufferCache(size_t clearBeforeFrame);

        virtual unsigned frameDurationAtIndex(size_t index) { return 0; }

        enum GIFQuery { GIFFullQuery, GIFSizeQuery, GIFFrameCountQuery };

        void decode(GIFQuery, unsigned haltAtFrame);

        // Callbacks from the GIF reader.
        bool sizeNowAvailable(unsigned width, unsigned height);
        void decodingHalted(unsigned bytesLeft);
        bool haveDecodedRow(unsigned frameIndex, unsigned char* rowBuffer, unsigned char* rowEnd, unsigned rowNumber, 
                            unsigned repeatCount, bool writeTransparentPixels);
        void frameComplete(unsigned frameIndex, unsigned frameDuration, RGBA32Buffer::FrameDisposalMethod disposalMethod);
        void gifComplete();

    private:
        // Called to initialize the frame buffer with the given index, based on the
        // previous frame's disposal method. Returns true on success. On failure,
        // this will mark the image as failed.
        bool initFrameBuffer(unsigned frameIndex);

        bool m_frameCountValid;
        bool m_currentBufferSawAlpha;
        mutable int m_repetitionCount;
        OwnPtr<GIFImageReader> m_reader;
        unsigned m_readOffset;
    };

} // namespace WebCore

#endif
