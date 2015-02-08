

#include "config.h"

#if ENABLE(DATAGRID)

#include "JSHTMLDataGridElement.h"

#include "Document.h"
#include "HTMLDataGridElement.h"
#include "JSDataGridDataSource.h"

using namespace JSC;

namespace WebCore {

JSValue JSHTMLDataGridElement::dataSource(ExecState*) const
{
    DataGridDataSource* dataSource = static_cast<HTMLDataGridElement*>(impl())->dataSource();
    if (dataSource && dataSource->isJSDataGridDataSource())
        return asJSDataGridDataSource(dataSource)->jsDataSource();
    return jsNull();
}

void JSHTMLDataGridElement::setDataSource(ExecState*, JSValue value)
{
    if (value.isNull()) {
        static_cast<HTMLDataGridElement*>(impl())->setDataSource(0);
        return;
    }

    static_cast<HTMLDataGridElement*>(impl())->setDataSource(JSDataGridDataSource::create(value, impl()->document()->frame()));
}

} // namespace WebCore

#endif // ENABLE(DATAGRID)
