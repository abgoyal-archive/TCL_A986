

#include "config.h"

#if ENABLE(DATAGRID)

#include "V8DataGridDataSource.h"

#include "Document.h"
#include "Frame.h"
#include "HTMLDataGridElement.h"
#include "V8HTMLDataGridElement.h"


namespace WebCore {

V8DataGridDataSource::V8DataGridDataSource(v8::Handle<v8::Value> dataSource, Frame* frame)
    : m_dataSource(v8::Persistent<v8::Value>::New(dataSource))
    , m_frame(frame)
{
#ifndef NDEBUG
    V8GCController::registerGlobalHandle(DATASOURCE, this, m_dataSource);
#endif
}

V8DataGridDataSource::~V8DataGridDataSource()
{
#ifndef NDEBUG
    V8GCController::unregisterGlobalHandle(this, m_dataSource);
#endif
    m_dataSource.Dispose();
    m_dataSource.Clear();
}

} // namespace WebCore

#endif // ENABLE(DATAGRID)
