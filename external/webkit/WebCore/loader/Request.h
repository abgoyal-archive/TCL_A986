

#ifndef Request_h
#define Request_h

#include "FrameLoaderTypes.h"
#include <wtf/Vector.h>

namespace WebCore {

    class CachedResource;
    class DocLoader;

    class Request : public Noncopyable {
    public:
        Request(DocLoader*, CachedResource*, bool incremental, SecurityCheckPolicy, bool sendResourceLoadCallbacks);
        ~Request();
        
        Vector<char>& buffer() { return m_buffer; }
        CachedResource* cachedResource() { return m_object; }
        DocLoader* docLoader() { return m_docLoader; }

        bool isIncremental() { return m_incremental; }
        void setIsIncremental(bool b = true) { m_incremental = b; }

        bool isMultipart() { return m_multipart; }
        void setIsMultipart(bool b = true) { m_multipart = b; }

        SecurityCheckPolicy shouldDoSecurityCheck() const { return m_shouldDoSecurityCheck; }
        bool sendResourceLoadCallbacks() const { return m_sendResourceLoadCallbacks; }
        
    private:
        Vector<char> m_buffer;
        CachedResource* m_object;
        DocLoader* m_docLoader;
        bool m_incremental;
        bool m_multipart;
        SecurityCheckPolicy m_shouldDoSecurityCheck;
        bool m_sendResourceLoadCallbacks;
    };

} //namespace WebCore

#endif // Request_h
