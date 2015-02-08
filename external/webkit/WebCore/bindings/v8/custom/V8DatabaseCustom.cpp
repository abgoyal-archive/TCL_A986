

#include "config.h"

#if ENABLE(DATABASE)
#include "V8Database.h"

#include "Database.h"
#include "V8Binding.h"
#include "V8CustomSQLTransactionCallback.h"
#include "V8CustomSQLTransactionErrorCallback.h"
#include "V8CustomVoidCallback.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8Database::changeVersionCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.Database.changeVersion()");

    if (args.Length() < 2)
        return throwError("The old and new version strings are required.", V8Proxy::SyntaxError);

    if (!(args[0]->IsString() && args[1]->IsString()))
        return throwError("The old and new versions must be strings.");

    Database* database = V8Database::toNative(args.Holder());

    Frame* frame = V8Proxy::retrieveFrameForCurrentContext();
    if (!frame)
        return v8::Undefined();

    RefPtr<V8CustomSQLTransactionCallback> callback;
    if (args.Length() > 2) {
        if (!args[2]->IsObject())
            return throwError("changeVersion transaction callback must be of valid type.");

        callback = V8CustomSQLTransactionCallback::create(args[2], frame);
    }

    RefPtr<V8CustomSQLTransactionErrorCallback> errorCallback;
    if (args.Length() > 3) {
        if (!args[3]->IsObject())
            return throwError("changeVersion error callback must be of valid type.");

        errorCallback = V8CustomSQLTransactionErrorCallback::create(args[3], frame);
    }

    RefPtr<V8CustomVoidCallback> successCallback;
    if (args.Length() > 4) {
        if (!args[4]->IsObject())
            return throwError("changeVersion success callback must be of valid type.");

        successCallback = V8CustomVoidCallback::create(args[4], frame);
    }

    database->changeVersion(toWebCoreString(args[0]), toWebCoreString(args[1]), callback.release(), errorCallback.release(), successCallback.release());

    return v8::Undefined();
}

static v8::Handle<v8::Value> createTransaction(const v8::Arguments& args, bool readOnly)
{
    if (!args.Length())
        return throwError("Transaction callback is required.", V8Proxy::SyntaxError);

    if (!args[0]->IsObject())
        return throwError("Transaction callback must be of valid type.");

    Database* database = V8Database::toNative(args.Holder());

    Frame* frame = V8Proxy::retrieveFrameForCurrentContext();
    if (!frame)
        return v8::Undefined();

    RefPtr<V8CustomSQLTransactionCallback> callback = V8CustomSQLTransactionCallback::create(args[0], frame);

    RefPtr<V8CustomSQLTransactionErrorCallback> errorCallback;
    if (args.Length() > 1) {
        if (!args[1]->IsObject())
            return throwError("Transaction error callback must be of valid type.");

        errorCallback = V8CustomSQLTransactionErrorCallback::create(args[1], frame);
    }

    RefPtr<V8CustomVoidCallback> successCallback;
    if (args.Length() > 2) {
        if (!args[2]->IsObject())
            return throwError("Transaction success callback must be of valid type.");

        successCallback = V8CustomVoidCallback::create(args[2], frame);
    }

    database->transaction(callback.release(), errorCallback.release(), successCallback.release(), readOnly);
    return v8::Undefined();
}

v8::Handle<v8::Value> V8Database::transactionCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.Database.transaction()");
    return createTransaction(args, false);
}

v8::Handle<v8::Value> V8Database::readTransactionCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.Database.readTransaction()");
    return createTransaction(args, true);
}

} // namespace WebCore

#endif
