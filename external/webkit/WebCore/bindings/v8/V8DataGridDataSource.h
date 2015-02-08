


#ifndef V8DataGridDataSource_h
#define V8DataGridDataSource_h

#if ENABLE(DATAGRID)

#include "DataGridDataSource.h"
#include <v8.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class Frame;
class HTMLDataGridElement;

class V8DataGridDataSource : public DataGridDataSource {
public:
    static PassRefPtr<V8DataGridDataSource> create(v8::Handle<v8::Value> dataSource, Frame* frame)
    {
        return adoptRef(new V8DataGridDataSource(dataSource, frame));
    }

    virtual ~V8DataGridDataSource();

    virtual bool isJSDataGridDataSource() const { return true; }
    v8::Handle<v8::Value> jsDataSource() const { return m_dataSource; }

private:
    V8DataGridDataSource(v8::Handle<v8::Value>, Frame*);

    v8::Persistent<v8::Value> m_dataSource;
    RefPtr<Frame> m_frame;
};

inline V8DataGridDataSource* asV8DataGridDataSource(DataGridDataSource* dataSource)
{
    ASSERT(dataSource->isJSDataGridDataSource());
    return static_cast<V8DataGridDataSource*>(dataSource);
}

inline const V8DataGridDataSource* asV8DataGridDataSource(const DataGridDataSource* dataSource)
{
    ASSERT(dataSource->isJSDataGridDataSource());
    return static_cast<const V8DataGridDataSource*>(dataSource);
}

} // namespace WebCore

#endif // ENABLE(DATAGRID)
#endif // V8DataGridDataSource_h
