

#include "config.h"
#include "HTMLKeygenElement.h"

#include "Document.h"
#include "FormDataList.h"
#include "HTMLNames.h"
#include "HTMLOptionElement.h"
#include "MappedAttribute.h"
#include "SSLKeyGenerator.h"
#include "Text.h"
#include <wtf/StdLibExtras.h>

using namespace WebCore;

namespace WebCore {

using namespace HTMLNames;

HTMLKeygenElement::HTMLKeygenElement(const QualifiedName& tagName, Document* doc, HTMLFormElement* f)
    : HTMLSelectElement(tagName, doc, f)
{
    ASSERT(hasTagName(keygenTag));
    Vector<String> keys;
    getSupportedKeySizes(keys);
        
    Vector<String>::const_iterator end = keys.end();
    for (Vector<String>::const_iterator it = keys.begin(); it != end; ++it) {
        HTMLOptionElement* o = new HTMLOptionElement(optionTag, doc, form());
        addChild(o);
        o->addChild(Text::create(doc, *it));
    }
}

const AtomicString& HTMLKeygenElement::formControlType() const
{
    DEFINE_STATIC_LOCAL(const AtomicString, keygen, ("keygen"));
    return keygen;
}

void HTMLKeygenElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == challengeAttr)
        m_challenge = attr->value();
    else if (attr->name() == keytypeAttr)
        m_keyType = attr->value();
    else
        // skip HTMLSelectElement parsing!
        HTMLFormControlElement::parseMappedAttribute(attr);
}

bool HTMLKeygenElement::appendFormData(FormDataList& encoded_values, bool)
{
    // Only RSA is supported at this time.
    if (!m_keyType.isNull() && !equalIgnoringCase(m_keyType, "rsa"))
        return false;
    String value = signedPublicKeyAndChallengeString(selectedIndex(), m_challenge, document()->baseURL());
    if (value.isNull())
        return false;
    encoded_values.appendData(name(), value.utf8());
    return true;
}

} // namespace
