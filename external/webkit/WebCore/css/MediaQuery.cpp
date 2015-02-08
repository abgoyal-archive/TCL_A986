

#include "config.h"
#include "MediaQuery.h"

#include "MediaQueryExp.h"

namespace WebCore {

MediaQuery::MediaQuery(Restrictor r, const String& mediaType, Vector<MediaQueryExp*>* exprs)
    : m_restrictor(r)
    , m_mediaType(mediaType)
    , m_expressions(exprs)
{
    if (!m_expressions)
        m_expressions = new Vector<MediaQueryExp*>;
}

MediaQuery::~MediaQuery()
{
    if (m_expressions) {
        deleteAllValues(*m_expressions);
        delete m_expressions;
    }
}

bool MediaQuery::operator==(const MediaQuery& other) const
{
    if (m_restrictor != other.m_restrictor
        || m_mediaType != other.m_mediaType
        || m_expressions->size() != other.m_expressions->size())
        return false;

    for (size_t i = 0; i < m_expressions->size(); ++i) {
        MediaQueryExp* exp = m_expressions->at(i);
        MediaQueryExp* oexp = other.m_expressions->at(i);
        if (!(*exp == *oexp))
            return false;
    }

    return true;
}

String MediaQuery::cssText() const
{
    String text;
    switch (m_restrictor) {
        case MediaQuery::Only:
            text += "only ";
            break;
        case MediaQuery::Not:
            text += "not ";
            break;
        case MediaQuery::None:
        default:
            break;
    }
    text += m_mediaType;
    for (size_t i = 0; i < m_expressions->size(); ++i) {
        MediaQueryExp* exp = m_expressions->at(i);
        text += " and (";
        text += exp->mediaFeature();
        if (exp->value()) {
            text += ": ";
            text += exp->value()->cssText();
        }
        text += ")";
    }
    return text;
}

} //namespace
