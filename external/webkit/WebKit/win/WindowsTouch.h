

#ifndef WindowsTouch_h
#define WindowsTouch_h


// Value used in WebViewWndProc for Gestures
#define WM_GESTURE 0x0119
#define WM_GESTURENOTIFY 0x011A

// Gesture Information Flags
#define GF_BEGIN 0x00000001
#define GF_INERTIA 0x00000002
#define GF_END 0x00000004

// Gesture IDs
#define GID_BEGIN 1
#define GID_END 2
#define GID_ZOOM 3
#define GID_PAN 4
#define GID_ROTATE 5
#define GID_TWOFINGERTAP 6
#define GID_PRESSANDTAP 7
#define GID_ROLLOVER GID_PRESSANDTAP

// Zoom Gesture Confiration Flags
#define GC_ZOOM 0x00000001

// Pan Gesture Configuration Flags
#define GC_PAN 0x00000001
#define GC_PAN_WITH_SINGLE_FINGER_VERTICALLY 0x00000002
#define GC_PAN_WITH_SINGLE_FINGER_HORIZONTALLY 0x00000004
#define GC_PAN_WITH_GUTTER 0x00000008
#define GC_PAN_WITH_INERTIA 0x00000010

// Rotate Gesture Configuration Flags
#define GC_ROTATE 0x00000001

// Two finger tap configuration flags
#define GC_TWOFINGERTAP 0x00000001

// Press and tap Configuration Flags
#define GC_PRESSANDTAP 0x00000001
#define GC_ROLLOVER GC_PRESSANDTAP

// GESTUREINFO struct definition
typedef struct tagGESTUREINFO {
    UINT cbSize;                    // size, in bytes, of this structure (including variable length Args field)
    DWORD dwFlags;                  // see GF_* flags
    DWORD dwID;                     // gesture ID, see GID_* defines
    HWND hwndTarget;                // handle to window targeted by this gesture
    POINTS ptsLocation;             // current location of this gesture
    DWORD dwInstanceID;             // internally used
    DWORD dwSequenceID;             // internally used
    ULONGLONG ullArguments;         // arguments for gestures whose arguments fit in 8 BYTES
    UINT cbExtraArgs;               // size, in bytes, of extra arguments, if any, that accompany this gesture
} GESTUREINFO, *PGESTUREINFO;
typedef GESTUREINFO const * PCGESTUREINFO;

// GESTURECONFIG struct defintion
typedef struct tagGESTURECONFIG {
    DWORD dwID;                     // gesture ID
    DWORD dwWant;                   // settings related to gesture ID that are to be turned on
    DWORD dwBlock;                  // settings related to gesture ID that are to be turned off
} GESTURECONFIG, *PGESTURECONFIG;

typedef struct tagGESTURENOTIFYSTRUCT {
    UINT cbSize;                    // size, in bytes, of this structure
    DWORD dwFlags;                  // unused
    HWND hwndTarget;                // handle to window targeted by the gesture
    POINTS ptsLocation;             // starting location
    DWORD dwInstanceID;             // internally used
} GESTURENOTIFYSTRUCT, *PGESTURENOTIFYSTRUCT;

DECLARE_HANDLE(HGESTUREINFO);

#endif
