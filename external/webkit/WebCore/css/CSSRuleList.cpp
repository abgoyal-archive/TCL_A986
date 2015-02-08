

#include "config.h"
#include "CSSRuleList.h"

#include "CSSRule.h"
#include "StyleList.h"

namespace WebCore {

CSSRuleList::CSSRuleList()
{
}

CSSRuleList::CSSRuleList(StyleList* list, bool omitCharsetRules)
{
    m_list = list;
    if (list && omitCharsetRules) {
        m_list = 0;
        unsigned len = list->length();
        for (unsigned i = 0; i < len; ++i) {
            StyleBase* style = list->item(i);
            if (style->isRule() && !style->isCharsetRule())
                append(static_cast<CSSRule*>(style));
        }
    }
}

CSSRuleList::~CSSRuleList()
{
}

unsigned CSSRuleList::length() const
{
    return m_list ? m_list->length() : m_lstCSSRules.size();
}

CSSRule* CSSRuleList::item(unsigned index)
{
    if (m_list) {
        StyleBase* rule = m_list->item(index);
        ASSERT(!rule || rule->isRule());
        return static_cast<CSSRule*>(rule);
    }

    if (index < m_lstCSSRules.size())
        return m_lstCSSRules[index].get();
    return 0;
}

void CSSRuleList::deleteRule(unsigned index)
{
    ASSERT(!m_list);

    if (index >= m_lstCSSRules.size()) {
        // FIXME: Should we throw an INDEX_SIZE_ERR exception here?
        return;
    }

    m_lstCSSRules.remove(index);
}

void CSSRuleList::append(CSSRule* rule)
{
    ASSERT(!m_list);
    if (!rule) {
        // FIXME: Should we throw an exception?
        return;
    }

    m_lstCSSRules.append(rule);
}

unsigned CSSRuleList::insertRule(CSSRule* rule, unsigned index)
{
    ASSERT(!m_list);
    if (!rule) {
        // FIXME: Should we throw an exception?
        return 0;
    }

    if (index > m_lstCSSRules.size()) {
        // FIXME: Should we throw an INDEX_SIZE_ERR exception here?
        return 0;
    }

    m_lstCSSRules.insert(index, rule);
    return index;
}

} // namespace WebCore
