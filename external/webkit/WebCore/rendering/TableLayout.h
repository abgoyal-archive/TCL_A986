

#ifndef TableLayout_h
#define TableLayout_h

#include <wtf/Noncopyable.h>

namespace WebCore {

class RenderTable;

class TableLayout : public Noncopyable {
public:
    TableLayout(RenderTable* table)
        : m_table(table)
    {
    }

    virtual ~TableLayout() { }

    virtual void calcPrefWidths(int& minWidth, int& maxWidth) = 0;
    virtual void layout() = 0;

protected:
    RenderTable* m_table;
};

} // namespace WebCore

#endif // TableLayout_h
