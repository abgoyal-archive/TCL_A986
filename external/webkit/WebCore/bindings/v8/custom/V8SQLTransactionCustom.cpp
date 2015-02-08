

#include "config.h"

#if ENABLE(DATABASE)

#include "V8SQLTransaction.h"

#include "Database.h"
#include "SQLValue.h"
#include "V8Binding.h"
#include "V8CustomSQLStatementCallback.h"
#include "V8CustomSQLStatementErrorCallback.h"
#include "V8Proxy.h"
#include <wtf/Vector.h>

using namespace WTF;

namespace WebCore {

v8::Handle<v8::Value> V8SQLTransaction::executeSqlCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.SQLTransaction.executeSql()");

    if (args.Length() == 0)
        return throwError("SQL statement is required.", V8Proxy::SyntaxError);

    String statement = toWebCoreString(args[0]);

    Vector<SQLValue> sqlValues;

    if (args.Length() > 1 && !isUndefinedOrNull(args[1])) {
        if (args[1]->IsObject()) {
            uint32_t sqlArgsLength = 0;
            v8::Local<v8::Object> sqlArgsObject = args[1]->ToObject();
            v8::Local<v8::Value> lengthGetter;
            {
                v8::TryCatch block;
                lengthGetter = sqlArgsObject->Get(v8::String::New("length"));
                if (block.HasCaught())
                    return throwError(block.Exception());
            }

            if (isUndefinedOrNull(lengthGetter))
                sqlArgsLength = sqlArgsObject->GetPropertyNames()->Length();
            else
                sqlArgsLength = lengthGetter->Uint32Value();

            for (unsigned int i = 0; i < sqlArgsLength; ++i) {
                v8::Local<v8::Integer> key = v8::Integer::New(i);
                v8::Local<v8::Value> value;
                {
                    v8::TryCatch block;
                    value = sqlArgsObject->Get(key);
                    if (block.HasCaught())
                        return throwError(block.Exception());
                }

                if (value.IsEmpty() || value->IsNull())
                    sqlValues.append(SQLValue());
                else if (value->IsNumber())
                    sqlValues.append(SQLValue(value->NumberValue()));
                else
                    sqlValues.append(SQLValue(toWebCoreString(value)));
            }
        } else
            return throwError("sqlArgs should be array or object!", V8Proxy::TypeError);
    }

    SQLTransaction* transaction = V8SQLTransaction::toNative(args.Holder());

    Frame* frame = V8Proxy::retrieveFrameForCurrentContext();

    RefPtr<SQLStatementCallback> callback;
    if (args.Length() > 2 && !isUndefinedOrNull(args[2])) {
        if (!args[2]->IsObject())
            return throwError("Statement callback must be of valid type.", V8Proxy::TypeError);

        if (frame)
            callback = V8CustomSQLStatementCallback::create(args[2], frame);
    }

    RefPtr<SQLStatementErrorCallback> errorCallback;
    if (args.Length() > 3 && !isUndefinedOrNull(args[3])) {
        if (!args[3]->IsObject())
            return throwError("Statement error callback must be of valid type.", V8Proxy::TypeError);

        if (frame)
            errorCallback = V8CustomSQLStatementErrorCallback::create(args[3], frame);
    }

    ExceptionCode ec = 0;
    transaction->executeSQL(statement, sqlValues, callback, errorCallback, ec);
    V8Proxy::setDOMException(ec);

    return v8::Undefined();
}

} // namespace WebCore

#endif

