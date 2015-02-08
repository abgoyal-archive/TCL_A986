

#include "config.h"
#include "JSSQLResultSetRowList.h"

#if ENABLE(DATABASE)

#include "ExceptionCode.h"
#include "SQLValue.h"
#include "SQLResultSetRowList.h"

using namespace JSC;

namespace WebCore {

JSValue JSSQLResultSetRowList::item(ExecState* exec, const ArgList& args)
{
    bool indexOk;
    int index = args.at(0).toInt32(exec, indexOk);
    if (!indexOk) {
        setDOMException(exec, TYPE_MISMATCH_ERR);
        return jsUndefined();
    }
    if (index < 0 || (unsigned)index >= m_impl->length()) {
        setDOMException(exec, INDEX_SIZE_ERR);
        return jsUndefined();
    }

    JSObject* object = constructEmptyObject(exec);

    unsigned numColumns = m_impl->columnNames().size();
    unsigned valuesIndex = index * numColumns;
    for (unsigned i = 0; i < numColumns; i++) {
        const SQLValue& value = m_impl->values()[valuesIndex + i];
        JSValue jsValue;

        switch (value.type()) {
            case SQLValue::StringValue:
              jsValue = jsString(exec, value.string());
              break;
          case SQLValue::NullValue:
              jsValue = jsNull();
              break;
          case SQLValue::NumberValue:
              jsValue = jsNumber(exec, value.number());
              break;
          default:
              ASSERT_NOT_REACHED();
        }

        object->putDirect(Identifier(exec, m_impl->columnNames()[i]), jsValue, DontDelete | ReadOnly);
    }

    return object;
}

}

#endif
