

#include "config.h"

#if ENABLE(DATABASE)
#include "V8SQLResultSetRowList.h"

#include "SQLResultSetRowList.h"
#include "V8Binding.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8SQLResultSetRowList::itemCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.SQLResultSetRowList.item()");

    if (args.Length() == 0) {
        V8Proxy::throwError(V8Proxy::SyntaxError, "Item index is required.");
        return v8::Undefined();
    }

    if (!args[0]->IsNumber()) {
        V8Proxy::throwError(V8Proxy::TypeError, "Item index must be a number.");
        return v8::Undefined();
    }

    SQLResultSetRowList* rowList = V8SQLResultSetRowList::toNative(args.Holder());

    unsigned long index = args[0]->IntegerValue();
    if (index < 0 || index >= rowList->length()) {
        V8Proxy::throwError(V8Proxy::RangeError, "Item index is out of range.");
        return v8::Undefined();
    }

    v8::Local<v8::Object> item = v8::Object::New();
    unsigned numColumns = rowList->columnNames().size();
    unsigned valuesIndex = index * numColumns;

    for (unsigned i = 0; i < numColumns; ++i) {
        const SQLValue& sqlValue = rowList->values()[valuesIndex + i];
        v8::Handle<v8::Value> value;
        switch(sqlValue.type()) {
            case SQLValue::StringValue:
                value = v8String(sqlValue.string());
                break;
            case SQLValue::NullValue:
                value = v8::Null();
                break;
            case SQLValue::NumberValue:
                value = v8::Number::New(sqlValue.number());
                break;
            default:
                ASSERT_NOT_REACHED();
        }

        item->Set(v8String(rowList->columnNames()[i]), value, static_cast<v8::PropertyAttribute>(v8::DontDelete|v8::ReadOnly));
    }

    return item;
}

} // namespace WebCore

#endif

