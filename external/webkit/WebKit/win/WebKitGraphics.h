

#ifndef WebKitGraphics_h
#define WebKitGraphics_h

#include <windows.h>

extern "C" {

typedef struct CGColor* CGColorRef;
typedef struct CGContext* CGContextRef;

typedef wchar_t WCHAR;
typedef __nullterminated const WCHAR* LPCWSTR;
typedef LPCWSTR LPCTSTR;

struct WebFontDescription {
    LPCTSTR family;
    unsigned familyLength;
    float size;
    // FIXME: Change to weight.
    bool bold;
    bool italic;
};

struct WebTextRenderInfo
{
    DWORD structSize;
    CGContextRef cgContext;
    LPCTSTR text;
    int length;
    POINT pt;
    const WebFontDescription* description;
    CGColorRef color;
    int underlinedIndex;
    bool drawAsPassword;
    int overrideSmoothingLevel; // pass in -1 if caller does not want to override smoothing level
    SIZE shadowOffset;
    int shadowBlur;
    CGColorRef shadowColor;
};

void WebDrawText(WebTextRenderInfo*);
float TextFloatWidth(LPCTSTR text, int length, const WebFontDescription&);
void FontMetrics(const WebFontDescription&, int* ascent, int* descent, int* lineSpacing);

// buffer must be large enough to hold all of "text", including its null terminator. Returns the number of characters put in buffer (excluding the null terminator).
unsigned CenterTruncateStringToWidth(LPCTSTR text, int length, const WebFontDescription&, float width, WCHAR* buffer);
unsigned RightTruncateStringToWidth(LPCTSTR text, int length, const WebFontDescription&, float width, WCHAR* buffer);

void WebKitSetShouldUseFontSmoothing(bool);
bool WebKitShouldUseFontSmoothing();

}

#endif // !defined(WebKitGraphics_h)
