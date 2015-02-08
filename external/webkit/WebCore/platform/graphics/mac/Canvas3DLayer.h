

#ifndef Canvas3DLayer_h
#define Canvas3DLayer_h

#if USE(ACCELERATED_COMPOSITING)

#import "WebLayer.h"

namespace WebCore {
    class GraphicsLayer;
}

@interface Canvas3DLayer : CAOpenGLLayer 
{
    WebCore::GraphicsLayer* m_layerOwner;
    CGLContextObj m_contextObj;
    GLuint m_texture;
}

- (id)initWithContext:(CGLContextObj)context texture:(GLuint)texture;

- (CGImageRef)copyImageSnapshotWithColorSpace:(CGColorSpaceRef)colorSpace;

@end

#endif // USE(ACCELERATED_COMPOSITING)

#endif // Canvas3DLayer_h
