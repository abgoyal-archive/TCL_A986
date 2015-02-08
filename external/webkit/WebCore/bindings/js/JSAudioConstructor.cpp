

#include "config.h"

#if ENABLE(VIDEO)

#include "JSAudioConstructor.h"

#include "HTMLAudioElement.h"
#include "HTMLNames.h"
#include "JSHTMLAudioElement.h"
#include "ScriptExecutionContext.h"
#include "Text.h"
#include <runtime/Error.h>

using namespace JSC;

namespace WebCore {

const ClassInfo JSAudioConstructor::s_info = { "AudioConstructor", 0, 0, 0 };

JSAudioConstructor::JSAudioConstructor(ExecState* exec, JSDOMGlobalObject* globalObject)
    : DOMConstructorWithDocument(JSAudioConstructor::createStructure(globalObject->objectPrototype()), globalObject)
{
    putDirect(exec->propertyNames().prototype, JSHTMLAudioElementPrototype::self(exec, globalObject), None);
    putDirect(exec->propertyNames().length, jsNumber(exec, 1), ReadOnly|DontDelete|DontEnum);
}

static JSObject* constructAudio(ExecState* exec, JSObject* constructor, const ArgList& args)
{
    JSAudioConstructor* jsAudio = static_cast<JSAudioConstructor*>(constructor);
    // FIXME: Why doesn't this need the call toJS on the document like JSImageConstructor?
    Document* document = jsAudio->document();
    if (!document)
        return throwError(exec, ReferenceError, "Audio constructor associated document is unavailable");

    RefPtr<HTMLAudioElement> audio = new HTMLAudioElement(HTMLNames::audioTag, document);
    audio->setAutobuffer(true);
    if (args.size() > 0) {
        audio->setSrc(args.at(0).toString(exec));
        audio->scheduleLoad();
    }
    return asObject(toJS(exec, jsAudio->globalObject(), audio.release()));
}

ConstructType JSAudioConstructor::getConstructData(ConstructData& constructData)
{
    constructData.native.function = constructAudio;
    return ConstructTypeHost;
}

} // namespace WebCore

#endif // ENABLE(VIDEO)
