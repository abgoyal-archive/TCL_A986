

#ifndef ICOImageDecoder_h
#define ICOImageDecoder_h

#include "BMPImageReader.h"

namespace WebCore {

    class PNGImageDecoder;

    // This class decodes the ICO and CUR image formats.
    class ICOImageDecoder : public ImageDecoder {
    public:
        ICOImageDecoder();

        // ImageDecoder
        virtual String filenameExtension() const { return "ico"; }
        virtual void setData(SharedBuffer*, bool allDataReceived);
        virtual bool isSizeAvailable();
        virtual IntSize size() const;
        virtual IntSize frameSizeAtIndex(size_t) const;
        virtual bool setSize(unsigned width, unsigned height);
        virtual size_t frameCount();
        virtual RGBA32Buffer* frameBufferAtIndex(size_t);

    private:
        enum ImageType {
            Unknown,
            BMP,
            PNG,
        };

        struct IconDirectoryEntry {
            IntSize m_size;
            uint16_t m_bitCount;
            uint32_t m_imageOffset;
        };

        // Returns true if |a| is a preferable icon entry to |b|.
        // Larger sizes, or greater bitdepths at the same size, are preferable.
        static bool compareEntries(const IconDirectoryEntry& a,
                                   const IconDirectoryEntry& b);

        inline uint16_t readUint16(int offset) const
        {
            return BMPImageReader::readUint16(m_data.get(),
                                              m_decodedOffset + offset);
        }

        inline uint32_t readUint32(int offset) const
        {
            return BMPImageReader::readUint32(m_data.get(),
                                              m_decodedOffset + offset);
        }

        // If the desired PNGImageDecoder exists, gives it the appropriate data.
        void setDataForPNGDecoderAtIndex(size_t);

        // Decodes the entry at |index|.  If |onlySize| is true, stops decoding
        // after calculating the image size.  If decoding fails but there is no
        // more data coming, sets the "decode failure" flag.
        //
        // NOTE: If the desired entry is a PNG, this doesn't actually trigger
        // decoding, it merely ensures the decoder is created and ready to
        // decode.  The caller will then call a function on the PNGImageDecoder
        // that actually triggers decoding.
        void decodeWithCheckForDataEnded(size_t index, bool onlySize);

        // Decodes the directory and directory entries at the beginning of the
        // data, and initializes members.  Returns true if all decoding
        // succeeded.  Once this returns true, all entries' sizes are known.
        bool decodeDirectory();

        // Decodes the specified entry.
        bool decodeAtIndex(size_t);

        // Processes the ICONDIR at the beginning of the data.  Returns true if
        // the directory could be decoded.
        bool processDirectory();

        // Processes the ICONDIRENTRY records after the directory.  Keeps the
        // "best" entry as the one we'll decode.  Returns true if the entries
        // could be decoded.
        bool processDirectoryEntries();

        // Reads and returns a directory entry from the current offset into
        // |data|.
        IconDirectoryEntry readDirectoryEntry();

        // Determines whether the desired entry is a BMP or PNG.  Returns true
        // if the type could be determined.
        ImageType imageTypeAtIndex(size_t);

        // True if we've seen all the data.
        bool m_allDataReceived;

        // An index into |m_data| representing how much we've already decoded.
        // Note that this only tracks data _this_ class decodes; once the
        // BMPImageReader takes over this will not be updated further.
        size_t m_decodedOffset;

        // The headers for the ICO.
        typedef Vector<IconDirectoryEntry> IconDirectoryEntries;
        IconDirectoryEntries m_dirEntries;

        // The image decoders for the various frames.
        typedef Vector<OwnPtr<BMPImageReader> > BMPReaders;
        BMPReaders m_bmpReaders;
        typedef Vector<OwnPtr<PNGImageDecoder> > PNGDecoders;
        PNGDecoders m_pngDecoders;

        // Valid only while a BMPImageReader is decoding, this holds the size
        // for the particular entry being decoded.
        IntSize m_frameSize;
    };

} // namespace WebCore

#endif
