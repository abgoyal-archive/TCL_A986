

#ifndef HTTPParsers_h
#define HTTPParsers_h

namespace WebCore {

    class String;

    bool parseHTTPRefresh(const String& refresh, bool fromHttpEquivMeta, double& delay, String& url);
    double parseDate(const String&);
    String filenameFromHTTPContentDisposition(const String&); 
    String extractMIMETypeFromMediaType(const String&);
    String extractCharsetFromMediaType(const String&); 
}

#endif
