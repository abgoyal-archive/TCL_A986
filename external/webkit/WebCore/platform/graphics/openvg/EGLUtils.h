

#ifndef EGLUtils_h
#define EGLUtils_h

#include <egl.h>
#include <wtf/Assertions.h>

static inline const char* toEGLErrorConstant(EGLint error)
{
    switch (error) {
    case EGL_NOT_INITIALIZED:
        return "EGL_NOT_INITIALIZED";
    case EGL_BAD_ACCESS:
        return "EGL_BAD_ACCESS";
    case EGL_BAD_ALLOC:
        return "EGL_BAD_ALLOC";
    case EGL_BAD_ATTRIBUTE:
        return "EGL_BAD_ATTRIBUTE";
    case EGL_BAD_CONTEXT:
        return "EGL_BAD_CONTEXT";
    case EGL_BAD_CONFIG:
        return "EGL_BAD_CONFIG";
    case EGL_BAD_CURRENT_SURFACE:
        return "EGL_BAD_CURRENT_SURFACE";
    case EGL_BAD_DISPLAY:
        return "EGL_BAD_DISPLAY";
    case EGL_BAD_SURFACE:
        return "EGL_BAD_SURFACE";
    case EGL_BAD_MATCH:
        return "EGL_BAD_MATCH";
    case EGL_BAD_PARAMETER:
        return "EGL_BAD_PARAMETER";
    case EGL_BAD_NATIVE_PIXMAP:
        return "EGL_BAD_NATIVE_PIXMAP";
    case EGL_BAD_NATIVE_WINDOW:
        return "EGL_BAD_NATIVE_WINDOW";
    case EGL_CONTEXT_LOST:
        return "EGL_CONTEXT_LOST";
    default:
        return "UNKNOWN_ERROR";
    }
}

#if ASSERT_DISABLED
#define ASSERT_EGL_NO_ERROR() ((void)0)
#else
#define ASSERT_EGL_NO_ERROR() do { \
    EGLint eglErrorCode = eglGetError(); \
    ASSERT_WITH_MESSAGE(eglErrorCode == EGL_SUCCESS, "Found %s", toEGLErrorConstant(eglErrorCode)); \
} while (0)
#endif

#endif
