

#include "config.h"
#include "MediaQueryExp.h"

#include "CSSParser.h"
#include "CSSPrimitiveValue.h"
#include "CSSValueList.h"

namespace WebCore {

MediaQueryExp::MediaQueryExp(const AtomicString& mediaFeature, CSSParserValueList* valueList)
    : m_mediaFeature(mediaFeature)
    , m_value(0)
{
    if (valueList) {
        if (valueList->size() == 1) {
            CSSParserValue* value = valueList->current();

            if (value->id != 0)
                m_value = CSSPrimitiveValue::createIdentifier(value->id);
            else if (value->unit == CSSPrimitiveValue::CSS_STRING)
                m_value = CSSPrimitiveValue::create(value->string, (CSSPrimitiveValue::UnitTypes) value->unit);
            else if (value->unit >= CSSPrimitiveValue::CSS_NUMBER &&
                      value->unit <= CSSPrimitiveValue::CSS_KHZ)
                m_value = CSSPrimitiveValue::create(value->fValue, (CSSPrimitiveValue::UnitTypes) value->unit);

            valueList->next();
        } else if (valueList->size() > 1) {
            // create list of values
            // currently accepts only <integer>/<integer>

            RefPtr<CSSValueList> list = CSSValueList::createCommaSeparated();
            CSSParserValue* value = 0;
            bool isValid = true;

            while ((value = valueList->current()) && isValid) {
                if (value->unit == CSSParserValue::Operator && value->iValue == '/')
                    list->append(CSSPrimitiveValue::create("/", CSSPrimitiveValue::CSS_STRING));
                else if (value->unit == CSSPrimitiveValue::CSS_NUMBER)
                    list->append(CSSPrimitiveValue::create(value->fValue, CSSPrimitiveValue::CSS_NUMBER));
                else
                    isValid = false;

                value = valueList->next();
            }

            if (isValid)
                m_value = list.release();
        }
    }
}

MediaQueryExp::~MediaQueryExp()
{
}

} // namespace
