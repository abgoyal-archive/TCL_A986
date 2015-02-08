

#ifndef CanvasNumberArray_h
#define CanvasNumberArray_h

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/Vector.h>
#include <limits>

namespace WebCore {

    class String;
    
    class CanvasNumberArray : public RefCounted<CanvasNumberArray> {
    public:
        static PassRefPtr<CanvasNumberArray> create(unsigned length);
        
        Vector<float>& data() { return m_data; }
        const Vector<float>& data() const { return m_data; }

        unsigned length() const { return m_data.size(); }
        float item(unsigned index) const { return (index >= m_data.size()) ? 0 : m_data[index]; }

    private:
        CanvasNumberArray(unsigned length);
        Vector<float> m_data;
    };
    
} // namespace WebCore

#endif // CanvasNumberArray_h
