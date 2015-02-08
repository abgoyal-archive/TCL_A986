

#ifndef WebTiledLayer_h
#define WebTiledLayer_h

#if USE(ACCELERATED_COMPOSITING)

#import "WebLayer.h"

@interface WebTiledLayer : CATiledLayer 
{
    WebCore::GraphicsLayer* m_layerOwner;
}

// implements WebLayerAdditions

@end

#endif // USE(ACCELERATED_COMPOSITING)

#endif // WebTiledLayer_h

