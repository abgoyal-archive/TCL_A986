

#ifndef WebLayer_h
#define WebLayer_h

#if USE(ACCELERATED_COMPOSITING)

#import <QuartzCore/QuartzCore.h>

namespace WebCore {
    class GraphicsLayer;
}

// Category implemented by WebLayer and WebTiledLayer.
@interface CALayer(WebLayerAdditions)

- (void)setLayerOwner:(WebCore::GraphicsLayer*)layer;
- (WebCore::GraphicsLayer*)layerOwner;

@end

#if defined(BUILDING_ON_LEOPARD)
@interface CALayer(WebLayerInternal)
- (CGAffineTransform)contentsTransform;
- (void)setContentsTransform:(CGAffineTransform)t;
@end
#endif

@interface WebLayer : CALayer 
{
    WebCore::GraphicsLayer* m_layerOwner;
}

// Class method allows us to share implementation across TiledLayerMac and WebLayer
+ (void)drawContents:(WebCore::GraphicsLayer*)layerContents ofLayer:(CALayer*)layer intoContext:(CGContextRef)context;

@end

#endif // USE(ACCELERATED_COMPOSITING)

#endif // WebLayer_h
