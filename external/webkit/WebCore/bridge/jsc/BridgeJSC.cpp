

#include "config.h"
#include "BridgeJSC.h"

#include "runtime_object.h"
#include "runtime_root.h"
#include <runtime/JSLock.h>

#if PLATFORM(QT)
#include "qt_instance.h"
#endif

namespace JSC {

namespace Bindings {

Array::Array(PassRefPtr<RootObject> rootObject)
    : m_rootObject(rootObject)
{
    ASSERT(m_rootObject);
}

Array::~Array()
{
}

Instance::Instance(PassRefPtr<RootObject> rootObject)
    : m_rootObject(rootObject)
    , m_runtimeObject(0)
{
    ASSERT(m_rootObject);
}

Instance::~Instance()
{
    ASSERT(!m_runtimeObject);
}

static KJSDidExecuteFunctionPtr s_didExecuteFunction;

void Instance::setDidExecuteFunction(KJSDidExecuteFunctionPtr func)
{
    s_didExecuteFunction = func;
}

KJSDidExecuteFunctionPtr Instance::didExecuteFunction()
{
    return s_didExecuteFunction;
}

void Instance::begin()
{
    virtualBegin();
}

void Instance::end()
{
    virtualEnd();
}

RuntimeObjectImp* Instance::createRuntimeObject(ExecState* exec)
{
    ASSERT(m_rootObject);
    ASSERT(m_rootObject->isValid());
    if (m_runtimeObject)
        return m_runtimeObject;
    JSLock lock(SilenceAssertionsOnly);
    m_runtimeObject = newRuntimeObject(exec);
    m_rootObject->addRuntimeObject(m_runtimeObject);
    return m_runtimeObject;
}

RuntimeObjectImp* Instance::newRuntimeObject(ExecState* exec)
{
    JSLock lock(SilenceAssertionsOnly);
    return new (exec)RuntimeObjectImp(exec, this);
}

void Instance::willDestroyRuntimeObject()
{
    ASSERT(m_rootObject);
    ASSERT(m_rootObject->isValid());
    ASSERT(m_runtimeObject);
    m_rootObject->removeRuntimeObject(m_runtimeObject);
    m_runtimeObject = 0;
}

void Instance::willInvalidateRuntimeObject()
{
    ASSERT(m_runtimeObject);
    m_runtimeObject = 0;
}

RootObject* Instance::rootObject() const
{
    return m_rootObject && m_rootObject->isValid() ? m_rootObject.get() : 0;
}

} // namespace Bindings

} // namespace JSC
