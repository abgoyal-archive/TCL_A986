

#include "config.h"
#include "JSDOMWindowShell.h"

#include "Frame.h"
#include "JSDOMWindow.h"
#include "DOMWindow.h"
#include "ScriptController.h"
#include <runtime/JSObject.h>

using namespace JSC;

namespace WebCore {

ASSERT_CLASS_FITS_IN_CELL(JSDOMWindowShell);

const ClassInfo JSDOMWindowShell::s_info = { "JSDOMWindowShell", 0, 0, 0 };

JSDOMWindowShell::JSDOMWindowShell(PassRefPtr<DOMWindow> window, DOMWrapperWorld* world)
    : Base(JSDOMWindowShell::createStructure(jsNull()))
    , m_window(0)
    , m_world(world)
{
    setWindow(window);
}

JSDOMWindowShell::~JSDOMWindowShell()
{
}

void JSDOMWindowShell::setWindow(PassRefPtr<DOMWindow> domWindow)
{
    // Explicitly protect the global object's prototype so it isn't collected
    // when we allocate the global object. (Once the global object is fully
    // constructed, it can mark its own prototype.)
    RefPtr<Structure> prototypeStructure = JSDOMWindowPrototype::createStructure(jsNull());
    ProtectedPtr<JSDOMWindowPrototype> prototype = new JSDOMWindowPrototype(prototypeStructure.release());

    RefPtr<Structure> structure = JSDOMWindow::createStructure(prototype);
    JSDOMWindow* jsDOMWindow = new (JSDOMWindow::commonJSGlobalData()) JSDOMWindow(structure.release(), domWindow, this);
    setWindow(jsDOMWindow);
}

// ----
// JSObject methods
// ----

void JSDOMWindowShell::markChildren(MarkStack& markStack)
{
    Base::markChildren(markStack);
    if (m_window)
        markStack.append(m_window);
}

UString JSDOMWindowShell::className() const
{
    return m_window->className();
}

bool JSDOMWindowShell::getOwnPropertySlot(ExecState* exec, const Identifier& propertyName, PropertySlot& slot)
{
    return m_window->getOwnPropertySlot(exec, propertyName, slot);
}

bool JSDOMWindowShell::getOwnPropertyDescriptor(ExecState* exec, const Identifier& propertyName, PropertyDescriptor& descriptor)
{
    return m_window->getOwnPropertyDescriptor(exec, propertyName, descriptor);
}

void JSDOMWindowShell::put(ExecState* exec, const Identifier& propertyName, JSValue value, PutPropertySlot& slot)
{
    m_window->put(exec, propertyName, value, slot);
}

void JSDOMWindowShell::putWithAttributes(ExecState* exec, const Identifier& propertyName, JSValue value, unsigned attributes)
{
    m_window->putWithAttributes(exec, propertyName, value, attributes);
}

bool JSDOMWindowShell::defineOwnProperty(JSC::ExecState* exec, const JSC::Identifier& propertyName, JSC::PropertyDescriptor& descriptor, bool shouldThrow)
{
    return m_window->defineOwnProperty(exec, propertyName, descriptor, shouldThrow);
}

bool JSDOMWindowShell::deleteProperty(ExecState* exec, const Identifier& propertyName)
{
    return m_window->deleteProperty(exec, propertyName);
}

void JSDOMWindowShell::getPropertyNames(ExecState* exec, PropertyNameArray& propertyNames, EnumerationMode mode)
{
    m_window->getPropertyNames(exec, propertyNames, mode);
}

void JSDOMWindowShell::getOwnPropertyNames(ExecState* exec, PropertyNameArray& propertyNames, EnumerationMode mode)
{
    m_window->getOwnPropertyNames(exec, propertyNames, mode);
}

void JSDOMWindowShell::defineGetter(ExecState* exec, const Identifier& propertyName, JSObject* getterFunction, unsigned attributes)
{
    m_window->defineGetter(exec, propertyName, getterFunction, attributes);
}

void JSDOMWindowShell::defineSetter(ExecState* exec, const Identifier& propertyName, JSObject* setterFunction, unsigned attributes)
{
    m_window->defineSetter(exec, propertyName, setterFunction, attributes);
}

JSValue JSDOMWindowShell::lookupGetter(ExecState* exec, const Identifier& propertyName)
{
    return m_window->lookupGetter(exec, propertyName);
}

JSValue JSDOMWindowShell::lookupSetter(ExecState* exec, const Identifier& propertyName)
{
    return m_window->lookupSetter(exec, propertyName);
}

JSObject* JSDOMWindowShell::unwrappedObject()
{
    return m_window;
}

// ----
// JSDOMWindow methods
// ----

DOMWindow* JSDOMWindowShell::impl() const
{
    return m_window->impl();
}

void* JSDOMWindowShell::operator new(size_t size)
{
    return JSDOMWindow::commonJSGlobalData()->heap.allocate(size);
}

// ----
// Conversion methods
// ----

JSValue toJS(ExecState* exec, Frame* frame)
{
    if (!frame)
        return jsNull();
    return frame->script()->windowShell(currentWorld(exec));
}

JSDOMWindowShell* toJSDOMWindowShell(Frame* frame, DOMWrapperWorld* isolatedWorld)
{
    if (!frame)
        return 0;
    return frame->script()->windowShell(isolatedWorld);
}

} // namespace WebCore
