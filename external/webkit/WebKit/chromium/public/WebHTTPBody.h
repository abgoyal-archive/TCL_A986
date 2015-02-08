

#ifndef WebHTTPBody_h
#define WebHTTPBody_h

#include "WebData.h"
#include "WebFileInfo.h"
#include "WebNonCopyable.h"
#include "WebString.h"

#if WEBKIT_IMPLEMENTATION
namespace WebCore { class FormData; }
namespace WTF { template <typename T> class PassRefPtr; }
#endif

namespace WebKit {

class WebHTTPBodyPrivate;

class WebHTTPBody {
public:
    struct Element {
        enum { TypeData, TypeFile } type;
        WebData data;
        WebString filePath;
        long long fileStart;
        long long fileLength; // -1 means to the end of the file.
        WebFileInfo fileInfo;
    };

    ~WebHTTPBody() { reset(); }

    WebHTTPBody() : m_private(0) { }
    WebHTTPBody(const WebHTTPBody& b) : m_private(0) { assign(b); }
    WebHTTPBody& operator=(const WebHTTPBody& b)
    {
        assign(b);
        return *this;
    }

    WEBKIT_API void initialize();
    WEBKIT_API void reset();
    WEBKIT_API void assign(const WebHTTPBody&);

    bool isNull() const { return !m_private; }

    // Returns the number of elements comprising the http body.
    WEBKIT_API size_t elementCount() const;

    // Sets the values of the element at the given index.  Returns false if
    // index is out of bounds.
    WEBKIT_API bool elementAt(size_t index, Element&) const;

    // Append to the list of elements.
    WEBKIT_API void appendData(const WebData&);
    WEBKIT_API void appendFile(const WebString&); // FIXME: to be removed.
    // Passing -1 to fileLength means to the end of the file.
    WEBKIT_API void appendFile(const WebString&, long long fileStart, long long fileLength, const WebFileInfo&);

    // Identifies a particular form submission instance.  A value of 0 is
    // used to indicate an unspecified identifier.
    WEBKIT_API long long identifier() const;
    WEBKIT_API void setIdentifier(long long);

#if WEBKIT_IMPLEMENTATION
    WebHTTPBody(const WTF::PassRefPtr<WebCore::FormData>&);
    WebHTTPBody& operator=(const WTF::PassRefPtr<WebCore::FormData>&);
    operator WTF::PassRefPtr<WebCore::FormData>() const;
#endif

private:
    void assign(WebHTTPBodyPrivate*);
    void ensureMutable();
    WebHTTPBodyPrivate* m_private;
};

} // namespace WebKit

#endif
