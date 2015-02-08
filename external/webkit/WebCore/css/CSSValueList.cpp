
#include "config.h"
#include "CSSValueList.h"

#include "CSSParserValues.h"
#include "PlatformString.h"

namespace WebCore {

CSSValueList::CSSValueList(bool isSpaceSeparated)
    : m_isSpaceSeparated(isSpaceSeparated)
{
}

CSSValueList::CSSValueList(CSSParserValueList* list)
    : m_isSpaceSeparated(true)
{
    if (list) {
        unsigned s = list->size();
        for (unsigned i = 0; i < s; ++i) {
            CSSParserValue* v = list->valueAt(i);
            append(v->createCSSValue());
        }
    }
}

CSSValueList::~CSSValueList()
{
}

CSSValue* CSSValueList::item(unsigned index)
{
    if (index >= m_values.size())
        return 0;
    return m_values[index].get();
}

unsigned short CSSValueList::cssValueType() const
{
    return CSS_VALUE_LIST;
}

void CSSValueList::append(PassRefPtr<CSSValue> val)
{
    m_values.append(val);
}

void CSSValueList::prepend(PassRefPtr<CSSValue> val)
{
    m_values.prepend(val);
}

bool CSSValueList::removeAll(CSSValue* val)
{
    bool found = false;
    // FIXME: we should be implementing operator== to CSSValue and its derived classes
    // to make comparison more flexible and fast.
    for (size_t index = 0; index < m_values.size(); index++) {
        if (m_values.at(index)->cssText() == val->cssText()) {
            m_values.remove(index);
            found = true;
        }
    }
    
    return found;
}
    
bool CSSValueList::hasValue(CSSValue* val)
{
    // FIXME: we should be implementing operator== to CSSValue and its derived classes
    // to make comparison more flexible and fast.
    for (size_t index = 0; index < m_values.size(); index++) {
        if (m_values.at(index)->cssText() == val->cssText())
            return true;
    }
    return false;
}

PassRefPtr<CSSValueList> CSSValueList::copy()
{
    PassRefPtr<CSSValueList> newList = m_isSpaceSeparated ? createSpaceSeparated() : createCommaSeparated();
    for (size_t index = 0; index < m_values.size(); index++)
        newList->append(item(index));
    return newList;
}

String CSSValueList::cssText() const
{
    String result = "";

    unsigned size = m_values.size();
    for (unsigned i = 0; i < size; i++) {
        if (!result.isEmpty()) {
            if (m_isSpaceSeparated)
                result += " ";
            else
                result += ", ";
        }
        result += m_values[i]->cssText();
    }

    return result;
}

CSSParserValueList* CSSValueList::createParserValueList() const
{
    unsigned s = m_values.size();
    if (!s)
        return 0;
    CSSParserValueList* result = new CSSParserValueList;
    for (unsigned i = 0; i < s; ++i)
        result->addValue(m_values[i]->parserValue());
    return result;
}

void CSSValueList::addSubresourceStyleURLs(ListHashSet<KURL>& urls, const CSSStyleSheet* styleSheet)
{
    size_t size = m_values.size();
    for (size_t i = 0; i < size; ++i)
        m_values[i]->addSubresourceStyleURLs(urls, styleSheet);
}

} // namespace WebCore
