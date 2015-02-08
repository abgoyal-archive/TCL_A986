

#include "config.h"
#include "CredentialStorage.h"

#include "CString.h"
#include "Credential.h"
#include "KURL.h"
#include "ProtectionSpaceHash.h"
#include "StringHash.h"
#include <wtf/HashMap.h>
#include <wtf/HashSet.h>
#include <wtf/StdLibExtras.h>

namespace WebCore {

typedef HashMap<ProtectionSpace, Credential> ProtectionSpaceToCredentialMap;
static ProtectionSpaceToCredentialMap& protectionSpaceToCredentialMap()
{
    DEFINE_STATIC_LOCAL(ProtectionSpaceToCredentialMap, map, ());
    return map;
}

static HashSet<String>& originsWithCredentials()
{
    DEFINE_STATIC_LOCAL(HashSet<String>, set, ());
    return set;
}

typedef HashMap<String, ProtectionSpace> PathToDefaultProtectionSpaceMap;
static PathToDefaultProtectionSpaceMap& pathToDefaultProtectionSpaceMap()
{
    DEFINE_STATIC_LOCAL(PathToDefaultProtectionSpaceMap, map, ());
    return map;
}

static String originStringFromURL(const KURL& url)
{
    if (url.port())
        return url.protocol() + "://" + url.host() + String::format(":%i/", url.port());
    
    return url.protocol() + "://" + url.host() + "/";
}

static String protectionSpaceMapKeyFromURL(const KURL& url)
{
    ASSERT(url.isValid());

    // Remove the last path component that is not a directory to determine the subtree for which credentials will apply.
    // We keep a leading slash, but remove a trailing one.
    String directoryURL = url.string().substring(0, url.pathEnd());
    unsigned directoryURLPathStart = url.pathStart();
    ASSERT(directoryURL[directoryURLPathStart] == '/');
    if (directoryURL.length() > directoryURLPathStart + 1) {
        int index = directoryURL.reverseFind('/');
        ASSERT(index > 0);
        directoryURL = directoryURL.substring(0, (static_cast<unsigned>(index) != directoryURLPathStart) ? index : directoryURLPathStart + 1);
    }
    ASSERT(directoryURL.length() == directoryURLPathStart + 1 || directoryURL[directoryURL.length() - 1] != '/');

    return directoryURL;
}

void CredentialStorage::set(const Credential& credential, const ProtectionSpace& protectionSpace, const KURL& url)
{
    ASSERT(protectionSpace.isProxy() || url.protocolInHTTPFamily());
    ASSERT(protectionSpace.isProxy() || url.isValid());

    protectionSpaceToCredentialMap().set(protectionSpace, credential);
    if (!protectionSpace.isProxy()) {
        originsWithCredentials().add(originStringFromURL(url));

        ProtectionSpaceAuthenticationScheme scheme = protectionSpace.authenticationScheme();
        if (scheme == ProtectionSpaceAuthenticationSchemeHTTPBasic || scheme == ProtectionSpaceAuthenticationSchemeDefault) {
            // The map can contain both a path and its subpath - while redundant, this makes lookups faster.
            pathToDefaultProtectionSpaceMap().set(protectionSpaceMapKeyFromURL(url), protectionSpace);
        }
    }
}

Credential CredentialStorage::get(const ProtectionSpace& protectionSpace)
{
    return protectionSpaceToCredentialMap().get(protectionSpace);
}

static PathToDefaultProtectionSpaceMap::iterator findDefaultProtectionSpaceForURL(const KURL& url)
{
    ASSERT(url.protocolInHTTPFamily());
    ASSERT(url.isValid());

    PathToDefaultProtectionSpaceMap& map = pathToDefaultProtectionSpaceMap();

    // Don't spend time iterating the path for origins that don't have any credentials.
    if (!originsWithCredentials().contains(originStringFromURL(url)))
        return map.end();

    String directoryURL = protectionSpaceMapKeyFromURL(url);
    unsigned directoryURLPathStart = url.pathStart();
    while (true) {
        PathToDefaultProtectionSpaceMap::iterator iter = map.find(directoryURL);
        if (iter != map.end())
            return iter;

        if (directoryURL.length() == directoryURLPathStart + 1)  // path is "/" already, cannot shorten it any more
            return map.end();

        int index = directoryURL.reverseFind('/', -2);
        ASSERT(index > 0);
        directoryURL = directoryURL.substring(0, (static_cast<unsigned>(index) == directoryURLPathStart) ? index + 1 : index);
        ASSERT(directoryURL.length() > directoryURLPathStart);
        ASSERT(directoryURL.length() == directoryURLPathStart + 1 || directoryURL[directoryURL.length() - 1] != '/');
    }
}

bool CredentialStorage::set(const Credential& credential, const KURL& url)
{
    ASSERT(url.protocolInHTTPFamily());
    ASSERT(url.isValid());
    PathToDefaultProtectionSpaceMap::iterator iter = findDefaultProtectionSpaceForURL(url);
    if (iter == pathToDefaultProtectionSpaceMap().end())
        return false;
    ASSERT(originsWithCredentials().contains(originStringFromURL(url)));
    protectionSpaceToCredentialMap().set(iter->second, credential);
    return true;
}

Credential CredentialStorage::get(const KURL& url)
{
    PathToDefaultProtectionSpaceMap::iterator iter = findDefaultProtectionSpaceForURL(url);
    if (iter == pathToDefaultProtectionSpaceMap().end())
        return Credential();
    return protectionSpaceToCredentialMap().get(iter->second);
}

} // namespace WebCore
