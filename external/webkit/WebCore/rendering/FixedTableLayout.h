

#ifndef FixedTableLayout_h
#define FixedTableLayout_h

#include "Length.h"
#include "TableLayout.h"
#include <wtf/Vector.h>

namespace WebCore {

class RenderTable;

class FixedTableLayout : public TableLayout {
public:
    FixedTableLayout(RenderTable*);

    virtual void calcPrefWidths(int& minWidth, int& maxWidth);
    virtual void layout();

protected:
    int calcWidthArray(int tableWidth);

    Vector<Length> m_width;
};

} // namespace WebCore

#endif // FixedTableLayout_h
