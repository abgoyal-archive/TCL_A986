

#ifndef WebGLGetInfo_h
#define WebGLGetInfo_h

#include "wtf/PassRefPtr.h"
#include "wtf/RefPtr.h"
#include "PlatformString.h"

#include "WebGLBuffer.h"
#include "WebGLFloatArray.h"
#include "WebGLFramebuffer.h"
#include "WebGLIntArray.h"
// FIXME: implement WebGLObjectArray
//#include "WebGLObjectArray.h"
#include "WebGLProgram.h"
#include "WebGLRenderbuffer.h"
#include "WebGLTexture.h"
#include "WebGLUnsignedByteArray.h"

namespace WebCore {

// A tagged union representing the result of get queries like
// getParameter (encompassing getBooleanv, getIntegerv, getFloatv) and
// similar variants. For reference counted types, increments and
// decrements the reference count of the target object.

class WebGLGetInfo {
public:
    enum Type {
        kTypeBool,
        kTypeFloat,
        kTypeLong,
        kTypeNull,
        kTypeString,
        kTypeUnsignedLong,
        kTypeWebGLBuffer,
        kTypeWebGLFloatArray,
        kTypeWebGLFramebuffer,
        kTypeWebGLIntArray,
        kTypeWebGLObjectArray,
        kTypeWebGLProgram,
        kTypeWebGLRenderbuffer,
        kTypeWebGLTexture,
        kTypeWebGLUnsignedByteArray
    };

    WebGLGetInfo(bool value);
    WebGLGetInfo(float value);
    WebGLGetInfo(long value);
    // Represents the NULL value and type
    WebGLGetInfo();
    WebGLGetInfo(const String& value);
    WebGLGetInfo(unsigned long value);
    WebGLGetInfo(PassRefPtr<WebGLBuffer> value);
    WebGLGetInfo(PassRefPtr<WebGLFloatArray> value);
    WebGLGetInfo(PassRefPtr<WebGLFramebuffer> value);
    WebGLGetInfo(PassRefPtr<WebGLIntArray> value);
    // FIXME: implement WebGLObjectArray
    // WebGLGetInfo(PassRefPtr<WebGLObjectArray> value);
    WebGLGetInfo(PassRefPtr<WebGLProgram> value);
    WebGLGetInfo(PassRefPtr<WebGLRenderbuffer> value);
    WebGLGetInfo(PassRefPtr<WebGLTexture> value);
    WebGLGetInfo(PassRefPtr<WebGLUnsignedByteArray> value);

    virtual ~WebGLGetInfo();

    Type getType() const;

    bool getBool() const;
    float getFloat() const;
    long getLong() const;
    const String& getString() const;
    unsigned long getUnsignedLong() const;
    PassRefPtr<WebGLBuffer> getWebGLBuffer() const;
    PassRefPtr<WebGLFloatArray> getWebGLFloatArray() const;
    PassRefPtr<WebGLFramebuffer> getWebGLFramebuffer() const;
    PassRefPtr<WebGLIntArray> getWebGLIntArray() const;
    // FIXME: implement WebGLObjectArray
    // PassRefPtr<WebGLObjectArray> getWebGLObjectArray() const;
    PassRefPtr<WebGLProgram> getWebGLProgram() const;
    PassRefPtr<WebGLRenderbuffer> getWebGLRenderbuffer() const;
    PassRefPtr<WebGLTexture> getWebGLTexture() const;
    PassRefPtr<WebGLUnsignedByteArray> getWebGLUnsignedByteArray() const;

private:
    Type m_type;
    bool m_bool;
    float m_float;
    long m_long;
    String m_string;
    unsigned long m_unsignedLong;
    RefPtr<WebGLBuffer> m_webglBuffer;
    RefPtr<WebGLFloatArray> m_webglFloatArray;
    RefPtr<WebGLFramebuffer> m_webglFramebuffer;
    RefPtr<WebGLIntArray> m_webglIntArray;
    // FIXME: implement WebGLObjectArray
    // RefPtr<WebGLObjectArray> m_webglObjectArray;
    RefPtr<WebGLProgram> m_webglProgram;
    RefPtr<WebGLRenderbuffer> m_webglRenderbuffer;
    RefPtr<WebGLTexture> m_webglTexture;
    RefPtr<WebGLUnsignedByteArray> m_webglUnsignedByteArray;
};

} // namespace WebCore

#endif  // WebGLGetInfo_h
