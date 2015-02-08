

#ifndef VGUtils_h
#define VGUtils_h

#include <openvg.h>
#include <wtf/Assertions.h>

static inline const char* toVGErrorConstant(VGErrorCode error)
{
    switch (error) {
    case VG_BAD_HANDLE_ERROR:
        return "VG_BAD_HANDLE_ERROR";
    case VG_ILLEGAL_ARGUMENT_ERROR:
        return "VG_ILLEGAL_ARGUMENT_ERROR";
    case VG_OUT_OF_MEMORY_ERROR:
        return "VG_OUT_OF_MEMORY_ERROR";
    case VG_PATH_CAPABILITY_ERROR:
        return "VG_PATH_CAPABILITY_ERROR";
    case VG_UNSUPPORTED_IMAGE_FORMAT_ERROR:
        return "VG_UNSUPPORTED_IMAGE_FORMAT_ERROR";
    case VG_UNSUPPORTED_PATH_FORMAT_ERROR:
        return "VG_UNSUPPORTED_PATH_FORMAT_ERROR";
    case VG_IMAGE_IN_USE_ERROR:
        return "VG_IMAGE_IN_USE_ERROR";
    case VG_NO_CONTEXT_ERROR:
        return "VG_NO_CONTEXT_ERROR";
    default:
        return "UNKNOWN_ERROR";
    }
}

#if ASSERT_DISABLED
#define ASSERT_VG_NO_ERROR() ((void)0)
#else
#define ASSERT_VG_NO_ERROR() do { \
    VGErrorCode vgErrorCode = vgGetError(); \
    ASSERT_WITH_MESSAGE(vgErrorCode == VG_NO_ERROR, "Found %s", toVGErrorConstant(vgErrorCode)); \
} while (0)
#endif


namespace WebCore {

class FloatRect;
class TransformationMatrix;

class VGMatrix {
public:
    VGMatrix(const VGfloat data[9]);
    VGMatrix(const TransformationMatrix&);
    const VGfloat* toVGfloat() const { return m_data; }
    operator TransformationMatrix() const;
private:
    VGfloat m_data[9];
};

class VGRect {
public:
    VGRect(const VGfloat data[4]);
    VGRect(const FloatRect&);
    const VGfloat* toVGfloat() const { return m_data; }
    operator FloatRect() const;
private:
    VGfloat m_data[4];
};

}

#endif
