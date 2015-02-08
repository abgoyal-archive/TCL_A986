

#ifndef WTF_GetPtr_h
#define WTF_GetPtr_h

namespace WTF {

    template <typename T> inline T* getPtr(T* p)
    {
        return p;
    }

} // namespace WTF

#endif // WTF_GetPtr_h
