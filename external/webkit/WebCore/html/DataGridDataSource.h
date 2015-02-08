

#ifndef DataGridDataSource_h
#define DataGridDataSource_h

#if ENABLE(DATAGRID)

#include <wtf/RefCounted.h>

namespace WebCore {

class HTMLDataGridElement;

class DataGridDataSource : public RefCounted<DataGridDataSource> {
public:
    virtual ~DataGridDataSource() { }

    virtual bool isDOMDataGridDataSource() const { return false; }
    virtual bool isJSDataGridDataSource() const { return false; }
};

} // namespace WebCore

#endif

#endif // DataGridDataSource_h
