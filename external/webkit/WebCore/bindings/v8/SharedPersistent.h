

#ifndef SharedPersistent_h
#define SharedPersistent_h

#include <v8.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

    // A shareable reference to a v8 persistent handle.  Using a shared
    // persistent any number of objects can share a reference to a v8
    // object and when it should no longer be accessible the object's
    // owner can clear it.
    template <typename T>
    class SharedPersistent : public RefCounted<SharedPersistent<T> > {
    public:
        void set(v8::Persistent<T> value)
        {
            m_value = value;
        }
        v8::Persistent<T> get()
        {
            return m_value;
        }
        void disposeHandle()
        {
            if (!m_value.IsEmpty()) {
              m_value.Dispose();
              m_value.Clear();
            }
        }
        static PassRefPtr<SharedPersistent<T> > create(v8::Persistent<T> value)
        {
            return adoptRef(new SharedPersistent<T>(value));
        }
        static PassRefPtr<SharedPersistent<T> > create()
        {
            return create(v8::Persistent<T>());
        }
    private:
        explicit SharedPersistent(v8::Persistent<T> value) : m_value(value) { }
        v8::Persistent<T> m_value;
    };

} // namespace WebCore

#endif // SharedPersistent_h
