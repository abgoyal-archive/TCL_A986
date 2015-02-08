

namespace WebCore {

    class HTTPHeaderMap;
    class ResourceResponse;
    class SecurityOrigin;
    class String;

    bool isSimpleCrossOriginAccessRequest(const String& method, const HTTPHeaderMap&);
    bool isOnAccessControlSimpleRequestMethodWhitelist(const String&);
    bool isOnAccessControlSimpleRequestHeaderWhitelist(const String& name, const String& value);
    bool isOnAccessControlResponseHeaderWhitelist(const String&);

    bool passesAccessControlCheck(const ResourceResponse&, bool includeCredentials, SecurityOrigin*);

} // namespace WebCore
