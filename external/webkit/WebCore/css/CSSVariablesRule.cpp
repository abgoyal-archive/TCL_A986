

#include "config.h"
#include "CSSVariablesRule.h"

#include "MediaList.h"
#include "PlatformString.h"
#include <wtf/HashMap.h>

namespace WebCore {

CSSVariablesRule::CSSVariablesRule(CSSStyleSheet* parent, MediaList* mediaList, bool variablesKeyword)
    : CSSRule(parent)
    , m_lstMedia(mediaList)
    , m_variablesKeyword(variablesKeyword)
{
}

CSSVariablesRule::~CSSVariablesRule()
{
}

String CSSVariablesRule::cssText() const
{
    String result = m_variablesKeyword ? "@-webkit-variables " : "@-webkit-define ";
    if (m_lstMedia) {
        if (!m_variablesKeyword)
            result += "for ";
        result += m_lstMedia->mediaText();
        result += " ";
    }
    if (m_variables)
        result += m_variables->cssText();
    result += ";";
    return result;
}

}
