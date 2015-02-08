

#ifndef WebGLUniformLocation_h
#define WebGLUniformLocation_h

#include "CanvasObject.h"
#include "WebGLProgram.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class WebGLUniformLocation : public RefCounted<WebGLUniformLocation> {
public:
    virtual ~WebGLUniformLocation() { }

    static PassRefPtr<WebGLUniformLocation> create(WebGLProgram* program, long location);

    WebGLProgram* program() const { return m_program.get(); }

    long location() const { return m_location; }

protected:
    WebGLUniformLocation(WebGLProgram* program, long location);

private:
    RefPtr<WebGLProgram> m_program;
    long m_location;
};

} // namespace WebCore

#endif // WebGLUniformLocation_h
