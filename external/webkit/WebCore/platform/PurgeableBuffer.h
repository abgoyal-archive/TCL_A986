

#ifndef PurgeableBuffer_h
#define PurgeableBuffer_h

#include <wtf/Noncopyable.h>
#include <wtf/Vector.h>

namespace WebCore {
    
    class PurgeableBuffer : public Noncopyable {
    public:
        static PurgeableBuffer* create(const char* data, size_t);
        static PurgeableBuffer* create(const Vector<char>& v) { return create(v.data(), v.size()); }
        
        ~PurgeableBuffer();

        // Call makePurgeable(false) and check the return value before accessing the data.
        const char* data() const;
        size_t size() const { return m_size; }
        
        enum PurgePriority { PurgeLast, PurgeMiddle, PurgeFirst, PurgeDefault = PurgeMiddle };
        PurgePriority purgePriority() const { return m_purgePriority; }
        void setPurgePriority(PurgePriority);
        
        bool isPurgeable() const { return m_state != NonVolatile; }
        bool wasPurged() const;

        bool makePurgeable(bool purgeable);
        
    private:
        PurgeableBuffer(char* data, size_t);
    
        char* m_data;
        size_t m_size;
        PurgePriority m_purgePriority;

        enum State { NonVolatile, Volatile, Purged };
        mutable State m_state;
    };

#if !OS(DARWIN) || defined(BUILDING_ON_TIGER) || PLATFORM(QT) || PLATFORM(GTK)
    inline PurgeableBuffer* PurgeableBuffer::create(const char*, size_t) { return 0; }
    inline PurgeableBuffer::~PurgeableBuffer() { }
    inline const char* PurgeableBuffer::data() const { return 0; }
    inline void PurgeableBuffer::setPurgePriority(PurgePriority) { }
    inline bool PurgeableBuffer::wasPurged() const { return false; }
    inline bool PurgeableBuffer::makePurgeable(bool) { return false; }
#endif
    
}

#endif
