

#include "config.h"
#include "HTMLDataGridElement.h"

#include "Document.h"
#include "V8Binding.h"
#include "V8DataGridDataSource.h"
#include "V8Proxy.h"

#if ENABLE(DATAGRID)

namespace WebCore {

ACCESSOR_GETTER(HTMLDataGridElementDataSource)
{
    INC_STATS("DOM.HTMLDataGridElement.dataSource._get");
    v8::Handle<v8::Object> holder = info.Holder();
    HTMLDataGridElement* imp = V8DOMWrapper::convertDOMWrapperToNode<HTMLDataGridElement>(holder);
    DataGridDataSource* dataSource = imp->dataSource();
    if (dataSource && dataSource->isJSDataGridDataSource())
        return asV8DataGridDataSource(dataSource)->jsDataSource();
    return v8::Null();
}

ACCESSOR_SETTER(HTMLDataGridElementDataSource)
{
    INC_STATS("DOM.HTMLDataGridElement.dataSource._set");
    v8::Handle<v8::Object> holder = info.Holder();
    HTMLDataGridElement* imp = V8DOMWrapper::convertDOMWrapperToNode<HTMLDataGridElement>(holder);
    RefPtr<DataGridDataSource> dataSource;
    if (!value.IsEmpty()) {
        Frame *frame = imp->document()->frame();
        dataSource = V8DataGridDataSource::create(value, frame);
    }
    imp->setDataSource(dataSource.get());
}

} // namespace WebCore

#endif // ENABLE(DATAGRID)
