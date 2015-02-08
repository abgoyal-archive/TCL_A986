

#include "config.h"

#if ENABLE(DATAGRID)

#include "JSDataGridDataSource.h"

#include "Document.h"
#include "Frame.h"
#include "HTMLDataGridElement.h"
#include "JSHTMLDataGridElement.h"
#include "JSDOMWindowBase.h"
#include <runtime/JSLock.h>

using namespace JSC;

namespace WebCore {

JSDataGridDataSource::JSDataGridDataSource(JSC::JSValue dataSource, Frame* frame)
    : m_dataSource(dataSource)
    , m_frame(frame)
{
}

JSDataGridDataSource::~JSDataGridDataSource()
{
}

} // namespace WebCore

#endif // ENABLE(DATAGRID)
