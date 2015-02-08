

#include "config.h"

#if ENABLE(INDEXED_DATABASE)
#include "V8IndexedDatabaseRequest.h"

#include "V8Binding.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8IndexedDatabaseRequest::openCallback(const v8::Arguments& args)
{
    IndexedDatabaseRequest* imp = V8IndexedDatabaseRequest::toNative(args.Holder());
    if (args.Length() < 2)
        return throwError(V8Proxy::TypeError);
    V8Parameter<> name = args[0];
    V8Parameter<> description = args[1];
    bool modifyDatabase = true;
    if (args.Length() > 2)
        modifyDatabase = args[2]->BooleanValue();

    ExceptionCode ec = 0;
    imp->open(name, description, modifyDatabase, ec);
    if (ec)
        return throwError(ec);
    return v8::Handle<v8::Value>();
}

} // namespace WebCore

#endif
