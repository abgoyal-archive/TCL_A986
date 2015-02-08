

#ifndef WebEntities_h
#define WebEntities_h

#include "PlatformString.h"
#include <wtf/HashMap.h>

namespace WebKit {

class WebEntities {
public:
    // &apos;, &percnt;, &nsup;, &supl; are not defined by the HTML standards.
    //  - IE does not support &apos; as an HTML entity (but support it as an XML
    //    entity.)
    //  - Firefox supports &apos; as an HTML entity.
    //  - Both of IE and Firefox don't support &percnt;, &nsup; and &supl;.
    //
    // A web page saved by Chromium should be able to be read by other browsers
    // such as IE and Firefox.  Chromium should produce only the standard entity
    // references which other browsers can recognize.
    // So if standard_html_entities_ is true, we will use a numeric character
    // reference for &apos;, and don't use entity references for &percnt;, &nsup;
    // and &supl; for serialization.
    //
    // If xmlEntities is true, WebEntities will only contain standard XML
    // entities.
    explicit WebEntities(bool xmlEntities);

    // Check whether specified unicode has corresponding html or xml built-in
    // entity name. If yes, return the entity notation. If not, returns an
    // empty string. Parameter isHTML indicates check the code in html entity
    // map or in xml entity map.
    WebCore::String entityNameByCode(int code) const;

    // Returns a new string with corresponding entity names replaced.
    WebCore::String convertEntitiesInString(const WebCore::String&) const;
private:
    typedef HashMap<int, WebCore::String> EntitiesMapType;
    // An internal object that maps the Unicode character to corresponding
    // entity notation.
    EntitiesMapType m_entitiesMap;
};

} // namespace WebKit

#endif
