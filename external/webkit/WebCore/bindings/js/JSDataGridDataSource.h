

#ifndef JSDataGridDataSource_h
#define JSDataGridDataSource_h

#if ENABLE(DATAGRID)

#include "DataGridDataSource.h"
#include <runtime/JSValue.h>
#include <runtime/Protect.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class Frame;
class HTMLDataGridElement;

class JSDataGridDataSource : public DataGridDataSource {
public:
    static PassRefPtr<JSDataGridDataSource> create(JSC::JSValue dataSource, Frame* frame)
    {
        return adoptRef(new JSDataGridDataSource(dataSource, frame));
    }

    virtual ~JSDataGridDataSource();

    virtual bool isJSDataGridDataSource() const { return true; }
    JSC::JSValue jsDataSource() const { return m_dataSource.get(); }

private:
    JSDataGridDataSource(JSC::JSValue, Frame*);

    JSC::ProtectedJSValue m_dataSource;
    RefPtr<Frame> m_frame;
};

inline JSDataGridDataSource* asJSDataGridDataSource(DataGridDataSource* dataSource)
{
    ASSERT(dataSource->isJSDataGridDataSource());
    return static_cast<JSDataGridDataSource*>(dataSource);
}

inline const JSDataGridDataSource* asJSDataGridDataSource(const DataGridDataSource* dataSource)
{
    ASSERT(dataSource->isJSDataGridDataSource());
    return static_cast<const JSDataGridDataSource*>(dataSource);
}

} // namespace WebCore

#endif // ENABLE(DATAGRID)
#endif // JSDataGridDataSource_h
