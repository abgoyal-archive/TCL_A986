

namespace WebCore {

    #define kTextEncodingISOLatinThai kCFStringEncodingISOLatinThai

    struct CharsetEntry {
        const char* name;
        ::TextEncoding encoding;
    };

    extern const CharsetEntry CharsetTable[];

}
