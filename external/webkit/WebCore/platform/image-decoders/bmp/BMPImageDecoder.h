

#ifndef BMPImageDecoder_h
#define BMPImageDecoder_h

#include "BMPImageReader.h"
#include <wtf/OwnPtr.h>

namespace WebCore {

    // This class decodes the BMP image format.
    class BMPImageDecoder : public ImageDecoder {
    public:
        BMPImageDecoder();

        // ImageDecoder
        virtual String filenameExtension() const { return "bmp"; }
        virtual void setData(SharedBuffer*, bool allDataReceived);
        virtual bool isSizeAvailable();
        virtual RGBA32Buffer* frameBufferAtIndex(size_t index);

    private:
        inline uint32_t readUint32(int offset) const
        {
            return BMPImageReader::readUint32(m_data.get(),
                                              m_decodedOffset + offset);
        }

        // Decodes the image.  If |onlySize| is true, stops decoding after
        // calculating the image size.  If decoding fails but there is no more
        // data coming, sets the "decode failure" flag.
        void decodeWithCheckForDataEnded(bool onlySize);

        // Decodes the image.  If |onlySize| is true, stops decoding after
        // calculating the image size.  Returns whether decoding succeeded.
        // NOTE: Used internally by decodeWithCheckForDataEnded().  Other people
        // should not call this.
        bool decode(bool onlySize);

        // Processes the file header at the beginning of the data.  Sets
        // |*imgDataOffset| based on the header contents.  Returns true if the
        // file header could be decoded.
        bool processFileHeader(size_t* imgDataOffset);

        // True if we've seen all the data.
        bool m_allDataReceived;

        // An index into |m_data| representing how much we've already decoded.
        // Note that this only tracks data _this_ class decodes; once the
        // BMPImageReader takes over this will not be updated further.
        size_t m_decodedOffset;

        // The reader used to do most of the BMP decoding.
        OwnPtr<BMPImageReader> m_reader;
    };

} // namespace WebCore

#endif
