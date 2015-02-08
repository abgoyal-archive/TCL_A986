

#ifndef SVGParserUtilities_h
#define SVGParserUtilities_h
#if ENABLE(SVG)

#include "ParserUtilities.h"

namespace WebCore {

    class Path;
    class SVGPointList;
    class SVGPathSegList;

    bool parseNumber(const UChar*& ptr, const UChar* end, float& number, bool skip = true);
    bool parseNumberOptionalNumber(const String& s, float& h, float& v);

    // SVG allows several different whitespace characters:
    // http://www.w3.org/TR/SVG/paths.html#PathDataBNF
    inline bool isWhitespace(const UChar& c)
    {
        return (c == ' ' || c == '\t' || c == '\n' || c == '\r');
    }

    inline bool skipOptionalSpaces(const UChar*& ptr, const UChar* end)
    {
        while (ptr < end && isWhitespace(*ptr))
            ptr++;
        return ptr < end;
    }

    inline bool skipOptionalSpacesOrDelimiter(const UChar*& ptr, const UChar* end, UChar delimiter = ',')
    {
        if (ptr < end && !isWhitespace(*ptr) && *ptr != delimiter)
            return false;
        if (skipOptionalSpaces(ptr, end)) {
            if (ptr < end && *ptr == delimiter) {
                ptr++;
                skipOptionalSpaces(ptr, end);
            }
        }
        return ptr < end;
    }

    bool pointsListFromSVGData(SVGPointList* pointsList, const String& points);
    bool pathFromSVGData(Path& path, const String& d);
    bool pathSegListFromSVGData(SVGPathSegList* pathSegList, const String& d, bool process = false);
    Vector<String> parseDelimitedString(const String& input, const char seperator);

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGParserUtilities_h
