

#include "config.h"
#include "V8CSSStyleDeclaration.h"

#include "CSSParser.h"
#include "CSSStyleDeclaration.h"
#include "CSSValue.h"
#include "CSSPrimitiveValue.h"
#include "EventTarget.h"

#include "V8Binding.h"
#include "V8Proxy.h"

#include <wtf/ASCIICType.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>
#include <wtf/StdLibExtras.h>
#include <wtf/Vector.h>

namespace WebCore {

// FIXME: Next two functions look lifted verbatim from JSCSSStyleDeclarationCustom. Please remove duplication.

// Check for a CSS prefix.
// Passed prefix is all lowercase.
// First character of the prefix within the property name may be upper or lowercase.
// Other characters in the prefix within the property name must be lowercase.
// The prefix within the property name must be followed by a capital letter.
static bool hasCSSPropertyNamePrefix(const String& propertyName, const char* prefix)
{
#ifndef NDEBUG
    ASSERT(*prefix);
    for (const char* p = prefix; *p; ++p)
        ASSERT(WTF::isASCIILower(*p));
    ASSERT(propertyName.length());
#endif

    if (WTF::toASCIILower(propertyName[0]) != prefix[0])
        return false;

    unsigned length = propertyName.length();
    for (unsigned i = 1; i < length; ++i) {
        if (!prefix[i])
            return WTF::isASCIIUpper(propertyName[i]);
        if (propertyName[i] != prefix[i])
            return false;
    }
    return false;
}

class CSSPropertyInfo {
public:
    int propID;
    bool hadPixelOrPosPrefix;
    bool wasFilter;
};

// When getting properties on CSSStyleDeclarations, the name used from
// Javascript and the actual name of the property are not the same, so
// we have to do the following translation.  The translation turns upper
// case characters into lower case characters and inserts dashes to
// separate words.
//
// Example: 'backgroundPositionY' -> 'background-position-y'
//
// Also, certain prefixes such as 'pos', 'css-' and 'pixel-' are stripped
// and the hadPixelOrPosPrefix out parameter is used to indicate whether or
// not the property name was prefixed with 'pos-' or 'pixel-'.
static CSSPropertyInfo* cssPropertyInfo(v8::Handle<v8::String>v8PropertyName)
{
    String propertyName = toWebCoreString(v8PropertyName);
    typedef HashMap<String, CSSPropertyInfo*> CSSPropertyInfoMap;
    DEFINE_STATIC_LOCAL(CSSPropertyInfoMap, map, ());
    CSSPropertyInfo* propInfo = map.get(propertyName);
    if (!propInfo) {
        unsigned length = propertyName.length();
        bool hadPixelOrPosPrefix = false;
        if (!length)
            return 0;

        Vector<UChar> name;
        name.reserveCapacity(length);

        unsigned i = 0;

        if (hasCSSPropertyNamePrefix(propertyName, "css"))
            i += 3;
        else if (hasCSSPropertyNamePrefix(propertyName, "pixel")) {
            i += 5;
            hadPixelOrPosPrefix = true;
        } else if (hasCSSPropertyNamePrefix(propertyName, "pos")) {
            i += 3;
            hadPixelOrPosPrefix = true;
        } else if (hasCSSPropertyNamePrefix(propertyName, "webkit")
                || hasCSSPropertyNamePrefix(propertyName, "khtml")
                || hasCSSPropertyNamePrefix(propertyName, "apple"))
            name.append('-');
        else if (WTF::isASCIIUpper(propertyName[0]))
            return 0;

        name.append(WTF::toASCIILower(propertyName[i++]));

        for (; i < length; ++i) {
            UChar c = propertyName[i];
            if (!WTF::isASCIIUpper(c))
                name.append(c);
            else {
                name.append('-');
                name.append(WTF::toASCIILower(c));
            }
        }

        String propName = String::adopt(name);
        int propertyID = cssPropertyID(propName);
        if (propertyID) {
            propInfo = new CSSPropertyInfo();
            propInfo->hadPixelOrPosPrefix = hadPixelOrPosPrefix;
            propInfo->wasFilter = (propName == "filter");
            propInfo->propID = propertyID;
            map.add(propertyName, propInfo);
        }
    }
    return propInfo;
}

v8::Handle<v8::Value> V8CSSStyleDeclaration::namedPropertyGetter(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
    INC_STATS("DOM.CSSStyleDeclaration.NamedPropertyGetter");
    // First look for API defined attributes on the style declaration object.
    if (info.Holder()->HasRealNamedCallbackProperty(name))
        return notHandledByInterceptor();

    // Search the style declaration.
    CSSStyleDeclaration* imp = V8CSSStyleDeclaration::toNative(info.Holder());
    CSSPropertyInfo* propInfo = cssPropertyInfo(name);

    // Do not handle non-property names.
    if (!propInfo)
        return notHandledByInterceptor();


    RefPtr<CSSValue> cssValue = imp->getPropertyCSSValue(propInfo->propID);
    if (cssValue) {
        if (propInfo->hadPixelOrPosPrefix &&
            cssValue->cssValueType() == CSSValue::CSS_PRIMITIVE_VALUE) {
            return v8::Number::New(static_cast<CSSPrimitiveValue*>(
                cssValue.get())->getFloatValue(CSSPrimitiveValue::CSS_PX));
        }
        return v8StringOrNull(cssValue->cssText());
    }

    String result = imp->getPropertyValue(propInfo->propID);
    if (result.isNull())
        result = "";  // convert null to empty string.

    // The 'filter' attribute is made undetectable in KJS/WebKit
    // to avoid confusion with IE's filter extension.
    if (propInfo->wasFilter)
        return v8UndetectableString(result);

    return v8String(result);
}

v8::Handle<v8::Value> V8CSSStyleDeclaration::namedPropertySetter(v8::Local<v8::String> name, v8::Local<v8::Value> value, const v8::AccessorInfo& info)
{
    INC_STATS("DOM.CSSStyleDeclaration.NamedPropertySetter");
    CSSStyleDeclaration* imp = V8CSSStyleDeclaration::toNative(info.Holder());
    CSSPropertyInfo* propInfo = cssPropertyInfo(name);
    if (!propInfo)
        return notHandledByInterceptor();

    String propertyValue = toWebCoreStringWithNullCheck(value);
    if (propInfo->hadPixelOrPosPrefix)
        propertyValue.append("px");

    ExceptionCode ec = 0;
    int importantIndex = propertyValue.find("!important", 0, false);
    bool important = false;
    if (importantIndex != -1) {
        important = true;
        propertyValue = propertyValue.left(importantIndex - 1);
    }
    imp->setProperty(propInfo->propID, propertyValue, important, ec);

    if (ec)
        throwError(ec);

    return value;
}

} // namespace WebCore
