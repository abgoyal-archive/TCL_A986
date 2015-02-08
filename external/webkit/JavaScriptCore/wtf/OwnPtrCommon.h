

#ifndef WTF_OwnPtrCommon_h
#define WTF_OwnPtrCommon_h

#if PLATFORM(WIN)
typedef struct HBITMAP__* HBITMAP;
typedef struct HBRUSH__* HBRUSH;
typedef struct HDC__* HDC;
typedef struct HFONT__* HFONT;
typedef struct HPALETTE__* HPALETTE;
typedef struct HPEN__* HPEN;
typedef struct HRGN__* HRGN;
#endif

namespace WTF {

    template <typename T> inline void deleteOwnedPtr(T* ptr)
    {
        typedef char known[sizeof(T) ? 1 : -1];
        if (sizeof(known))
            delete ptr;
    }

#if PLATFORM(WIN)
    void deleteOwnedPtr(HBITMAP);
    void deleteOwnedPtr(HBRUSH);
    void deleteOwnedPtr(HDC);
    void deleteOwnedPtr(HFONT);
    void deleteOwnedPtr(HPALETTE);
    void deleteOwnedPtr(HPEN);
    void deleteOwnedPtr(HRGN);
#endif

} // namespace WTF

#endif // WTF_OwnPtrCommon_h
