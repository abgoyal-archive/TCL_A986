

#ifndef CanvasPixelArray_h
#define CanvasPixelArray_h

#include <wtf/ByteArray.h>
#include <wtf/MathExtras.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/Vector.h>

namespace WebCore {
    
    class CanvasPixelArray : public RefCounted<CanvasPixelArray> {
    public:
        static PassRefPtr<CanvasPixelArray> create(unsigned length);
        
        WTF::ByteArray* data() { return m_data.get(); }
        const WTF::ByteArray* data() const { return m_data.get(); }
        unsigned length() const { return m_data->length(); }
        
        void set(unsigned index, double value)
        {
            m_data->set(index, value);
        }

        void set(unsigned index, unsigned char value)
        {
            m_data->set(index, value);
        }
        
        bool get(unsigned index, unsigned char& result) const
        {
            return m_data->get(index, result);
        }

        unsigned char get(unsigned index) const
        {
            return m_data->get(index);
        }

    private:
        CanvasPixelArray(unsigned length);
        RefPtr<WTF::ByteArray> m_data;
    };
    
} // namespace WebCore

#endif // CanvasPixelArray_h
