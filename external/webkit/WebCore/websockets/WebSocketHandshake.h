

#ifndef WebSocketHandshake_h
#define WebSocketHandshake_h

#if ENABLE(WEB_SOCKETS)

#include "KURL.h"
#include "PlatformString.h"
#include <wtf/Noncopyable.h>

namespace WebCore {

    class ScriptExecutionContext;
    class HTTPHeaderMap;

    class WebSocketHandshake : public Noncopyable {
    public:
        enum Mode {
            Incomplete, Normal, Failed, Connected
        };
        WebSocketHandshake(const KURL&, const String& protocol, ScriptExecutionContext*);
        ~WebSocketHandshake();

        const KURL& url() const;
        void setURL(const KURL&);
        const String host() const;

        const String& clientProtocol() const;
        void setClientProtocol(const String& protocol);

        bool secure() const;
        void setSecure(bool secure);

        String clientOrigin() const;
        String clientLocation() const;

        CString clientHandshakeMessage() const;

        void reset();

        int readServerHandshake(const char* header, size_t len);
        Mode mode() const;

        const String& serverWebSocketOrigin() const;
        void setServerWebSocketOrigin(const String& webSocketOrigin);

        const String& serverWebSocketLocation() const;
        void setServerWebSocketLocation(const String& webSocketLocation);

        const String& serverWebSocketProtocol() const;
        void setServerWebSocketProtocol(const String& webSocketProtocol);

        const String& serverSetCookie() const;
        void setServerSetCookie(const String& setCookie);
        const String& serverSetCookie2() const;
        void setServerSetCookie2(const String& setCookie2);

    private:
        KURL httpURLForAuthenticationAndCookies() const;

        // Reads all headers except for the two predefined ones.
        const char* readHTTPHeaders(const char* start, const char* end, HTTPHeaderMap* headers);
        bool processHeaders(const HTTPHeaderMap& headers);
        void checkResponseHeaders();

        KURL m_url;
        String m_clientProtocol;
        bool m_secure;
        ScriptExecutionContext* m_context;

        Mode m_mode;

        String m_wsOrigin;
        String m_wsLocation;
        String m_wsProtocol;
        String m_setCookie;
        String m_setCookie2;
    };

} // namespace WebCore

#endif // ENABLE(WEB_SOCKETS)

#endif // WebSocketHandshake_h
