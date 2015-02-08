

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLUniformLocation.h"

namespace WebCore {
    
PassRefPtr<WebGLUniformLocation> WebGLUniformLocation::create(WebGLProgram* program, long location)
{
    return adoptRef(new WebGLUniformLocation(program, location));
}

WebGLUniformLocation::WebGLUniformLocation(WebGLProgram* program, long location)
    : m_program(program)
    , m_location(location)
{
}

}

#endif // ENABLE(3D_CANVAS)
