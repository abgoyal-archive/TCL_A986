

#ifndef StringTruncator_h
#define StringTruncator_h

namespace WebCore {
    
    class Font;
    class String;
    
    class StringTruncator {
    public:
        static String centerTruncate(const String&, float maxWidth, const Font&, bool disableRoundingHacks = true);
        static String rightTruncate(const String&, float maxWidth, const Font&, bool disableRoundingHacks = true);
        static float width(const String&, const Font&, bool disableRoundingHacks = true);
    };
    
} // namespace WebCore

#endif // !defined(StringTruncator_h)
