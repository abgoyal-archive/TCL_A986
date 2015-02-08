

#ifndef AutoTableLayout_h
#define AutoTableLayout_h

#include "Length.h"
#include "TableLayout.h"
#include <wtf/Vector.h>

namespace WebCore {

class RenderTable;
class RenderTableCell;

class AutoTableLayout : public TableLayout {
public:
    AutoTableLayout(RenderTable*);
    ~AutoTableLayout();

    virtual void calcPrefWidths(int& minWidth, int& maxWidth);
    virtual void layout();

protected:
    void fullRecalc();
    void recalcColumn(int effCol);

    void calcPercentages() const;
    int totalPercent() const
    {
        if (m_percentagesDirty)
            calcPercentages();
        return m_totalPercent;
    }

    int calcEffectiveWidth();

    void insertSpanCell(RenderTableCell*);

    struct Layout {
        Layout()
            : minWidth(0)
            , maxWidth(0)
            , effMinWidth(0)
            , effMaxWidth(0)
            , calcWidth(0)
            , emptyCellsOnly(true) {}
        Length width;
        Length effWidth;
        int minWidth;
        int maxWidth;
        int effMinWidth;
        int effMaxWidth;
        int calcWidth;
        bool emptyCellsOnly;
    };

    Vector<Layout, 4> m_layoutStruct;
    Vector<RenderTableCell*, 4> m_spanCells;
    bool m_hasPercent : 1;
    mutable bool m_percentagesDirty : 1;
    mutable bool m_effWidthDirty : 1;
    mutable unsigned short m_totalPercent;
};

} // namespace WebCore

#endif // AutoTableLayout_h
