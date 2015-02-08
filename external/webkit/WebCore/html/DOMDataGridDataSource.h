

#ifndef DOMDataGridDataSource_h
#define DOMDataGridDataSource_h

#if ENABLE(DATAGRID)

#include "DataGridDataSource.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class HTMLDataGridElement;

class DOMDataGridDataSource : public DataGridDataSource {
public:
    static PassRefPtr<DOMDataGridDataSource> create()
    {
        return adoptRef(new DOMDataGridDataSource);
    }

    virtual ~DOMDataGridDataSource();

    virtual bool isDOMDataGridDataSource() const { return true; }

private:
    DOMDataGridDataSource();
};

inline DOMDataGridDataSource* asDOMDataGridDataSource(DataGridDataSource* dataSource)
{
    ASSERT(dataSource->isDOMDataGridDataSource());
    return static_cast<DOMDataGridDataSource*>(dataSource);
}

inline const DOMDataGridDataSource* asDOMDataGridDataSource(const DataGridDataSource* dataSource)
{
    ASSERT(dataSource->isDOMDataGridDataSource());
    return static_cast<const DOMDataGridDataSource*>(dataSource);
}

} // namespace WebCore

#endif // ENABLE(DATAGRID)
#endif // DOMDataGridDataSource_h
