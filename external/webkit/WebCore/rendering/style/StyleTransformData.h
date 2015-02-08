

#ifndef StyleTransformData_h
#define StyleTransformData_h

#include "Length.h"
#include "TransformOperations.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class StyleTransformData : public RefCounted<StyleTransformData> {
public:
    static PassRefPtr<StyleTransformData> create() { return adoptRef(new StyleTransformData); }
    PassRefPtr<StyleTransformData> copy() const { return adoptRef(new StyleTransformData(*this)); }

    bool operator==(const StyleTransformData& o) const;
    bool operator!=(const StyleTransformData& o) const
    {
        return !(*this == o);
    }

    TransformOperations m_operations;
    Length m_x;
    Length m_y;
    float m_z;

private:
    StyleTransformData();
    StyleTransformData(const StyleTransformData&);
};

} // namespace WebCore

#endif // StyleTransformData_h
