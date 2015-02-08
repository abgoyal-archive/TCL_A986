

#ifndef TextCodecGTK_h
#define TextCodecGTK_h

#include <glib.h>
#include "TextCodec.h"
#include "TextEncoding.h"

namespace WebCore {

    class TextCodecGtk : public TextCodec {
    public:
        static void registerBaseEncodingNames(EncodingNameRegistrar);
        static void registerBaseCodecs(TextCodecRegistrar);

        static void registerExtendedEncodingNames(EncodingNameRegistrar);
        static void registerExtendedCodecs(TextCodecRegistrar);

        TextCodecGtk(const TextEncoding&);
        virtual ~TextCodecGtk();

        virtual String decode(const char*, size_t length, bool flush, bool stopOnError, bool& sawError);
        virtual CString encode(const UChar*, size_t length, UnencodableHandling);

    private:
        void createIConvDecoder() const;
        void createIConvEncoder() const;

        static void registerEncodingNames(EncodingNameRegistrar registrar, bool extended);        
        static void registerCodecs(TextCodecRegistrar registrar, bool extended);
        static gboolean isEncodingAvailable(const gchar*);

        TextEncoding m_encoding;
        size_t m_numBufferedBytes;
        unsigned char m_bufferedBytes[16]; // bigger than any single multi-byte character        
        mutable GIConv m_iconvDecoder;
        mutable GIConv m_iconvEncoder;

        static const gchar* m_internalEncodingName;

        typedef const gchar* const codecAliasList[];

        // Unicode
        static codecAliasList m_codecAliases_UTF_8;

        // Western
        static codecAliasList m_codecAliases_ISO_8859_1;
        static codecAliasList m_codecAliases_MACROMAN;

        // Japanese
        static codecAliasList m_codecAliases_SHIFT_JIS;
        static codecAliasList m_codecAliases_EUC_JP;
        static codecAliasList m_codecAliases_ISO_2022_JP;

        // Traditional Chinese
        static codecAliasList m_codecAliases_BIG5;
        static codecAliasList m_codecAliases_BIG5_HKSCS;
        static codecAliasList m_codecAliases_CP950;

        // Korean
        static codecAliasList m_codecAliases_ISO_2022_KR;
        static codecAliasList m_codecAliases_CP949;
        static codecAliasList m_codecAliases_EUC_KR;

        // Arabic
        static codecAliasList m_codecAliases_ISO_8859_6;
        static codecAliasList m_codecAliases_CP1256;

        // Hebrew
        static codecAliasList m_codecAliases_ISO_8859_8;
        static codecAliasList m_codecAliases_CP1255;

        // Greek
        static codecAliasList m_codecAliases_ISO_8859_7;
        static codecAliasList m_codecAliases_CP869;
        static codecAliasList m_codecAliases_WINDOWS_1253;

        // Cyrillic
        static codecAliasList m_codecAliases_ISO_8859_5;
        static codecAliasList m_codecAliases_KOI8_R;
        static codecAliasList m_codecAliases_CP866;
        static codecAliasList m_codecAliases_KOI8_U;
        static codecAliasList m_codecAliases_WINDOWS_1251;
        static codecAliasList m_codecAliases_MACCYRILLIC;

        // Thai
        static codecAliasList m_codecAliases_CP874;
        static codecAliasList m_codecAliases_TIS_620;

        // Simplified Chinese
        static codecAliasList m_codecAliases_GBK;
        static codecAliasList m_codecAliases_HZ;
        static codecAliasList m_codecAliases_GB18030;
        static codecAliasList m_codecAliases_EUC_CN;
        static codecAliasList m_codecAliases_2312_80; 

        // Central European
        static codecAliasList m_codecAliases_ISO_8859_2;
        static codecAliasList m_codecAliases_CP1250;
        static codecAliasList m_codecAliases_MACCENTRALEUROPE;

        // Vietnamese
        static codecAliasList m_codecAliases_CP1258;

        // Turkish
        static codecAliasList m_codecAliases_CP1254;
        static codecAliasList m_codecAliases_ISO_8859_9;

        // Baltic
        static codecAliasList m_codecAliases_CP1257;
        static codecAliasList m_codecAliases_ISO_8859_4;

        static gconstpointer const m_iconvBaseCodecList[];
        static gconstpointer const m_iconvExtendedCodecList[];

    };

} // namespace WebCore

#endif // TextCodecGTK_h
