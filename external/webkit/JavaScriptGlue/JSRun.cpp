

#include "config.h"
#include "JSRun.h"

#include "UserObjectImp.h"
#include <JavaScriptCore/Completion.h>
#include <JavaScriptCore/SourceCode.h>

JSGlueGlobalObject::JSGlueGlobalObject(PassRefPtr<Structure> structure, JSFlags flags)
    : JSGlobalObject(structure, new Data, this)
{
    d()->flags = flags;
    d()->userObjectStructure = UserObjectImp::createStructure(jsNull());
}

void JSGlueGlobalObject::destroyData(void* data)
{
    delete static_cast<Data*>(data);
}

JSRun::JSRun(CFStringRef source, JSFlags inFlags)
    :   JSBase(kJSRunTypeID),
        fSource(CFStringToUString(source)),
        fGlobalObject(new (&getThreadGlobalExecState()->globalData()) JSGlueGlobalObject(JSGlueGlobalObject::createStructure(jsNull()), inFlags)),
        fFlags(inFlags)
{
}

JSRun::~JSRun()
{
}

JSFlags JSRun::Flags() const
{
    return fFlags;
}

UString JSRun::GetSource() const
{
    return fSource;
}

JSGlobalObject* JSRun::GlobalObject() const
{
    return fGlobalObject;
}

Completion JSRun::Evaluate()
{
    return JSC::evaluate(fGlobalObject->globalExec(), fGlobalObject->globalScopeChain(), makeSource(fSource));
}

bool JSRun::CheckSyntax()
{
    return JSC::checkSyntax(fGlobalObject->globalExec(), makeSource(fSource)).complType() != Throw;
}
