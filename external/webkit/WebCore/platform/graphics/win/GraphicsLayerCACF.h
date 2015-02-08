

#ifndef GraphicsLayerCACF_h_
#define GraphicsLayerCACF_h_

#if USE(ACCELERATED_COMPOSITING)

#include "GraphicsLayer.h"
#include "GraphicsContext.h"
#include <wtf/RetainPtr.h>

namespace WebCore {

class WKCACFLayer;

class GraphicsLayerCACF : public GraphicsLayer {
public:

    GraphicsLayerCACF(GraphicsLayerClient*);
    virtual ~GraphicsLayerCACF();

    virtual void setName(const String& inName);

    // for hosting this GraphicsLayer in a native layer hierarchy
    virtual NativeLayer nativeLayer() const;

    virtual bool setChildren(const Vector<GraphicsLayer*>&);
    virtual void addChild(GraphicsLayer *layer);
    virtual void addChildAtIndex(GraphicsLayer *layer, int index);
    virtual void addChildAbove(GraphicsLayer *layer, GraphicsLayer *sibling);
    virtual void addChildBelow(GraphicsLayer *layer, GraphicsLayer *sibling);
    virtual bool replaceChild(GraphicsLayer *oldChild, GraphicsLayer *newChild);

    virtual void removeFromParent();

    virtual void setPosition(const FloatPoint& inPoint);
    virtual void setAnchorPoint(const FloatPoint3D& inPoint);
    virtual void setSize(const FloatSize& inSize);

    virtual void setTransform(const TransformationMatrix&);

    virtual void setChildrenTransform(const TransformationMatrix&);

    virtual void setPreserves3D(bool);
    virtual void setMasksToBounds(bool);
    virtual void setDrawsContent(bool);

    virtual void setBackgroundColor(const Color&);
    virtual void clearBackgroundColor();

    virtual void setContentsOpaque(bool);
    virtual void setBackfaceVisibility(bool);

    virtual void setOpacity(float opacity);

    virtual void setNeedsDisplay();
    virtual void setNeedsDisplayInRect(const FloatRect&);

    virtual void setContentsRect(const IntRect&);

    virtual void setContentsToImage(Image*);
    virtual void setContentsToMedia(PlatformLayer*);
    
    virtual PlatformLayer* platformLayer() const;

    virtual void setDebugBackgroundColor(const Color&);
    virtual void setDebugBorder(const Color&, float borderWidth);

    virtual void setGeometryOrientation(CompositingCoordinatesOrientation);

    void notifySyncRequired() { if (m_client) m_client->notifySyncRequired(this); }

private:
    void updateOpacityOnLayer();

    WKCACFLayer* primaryLayer() const  { return m_transformLayer.get() ? m_transformLayer.get() : m_layer.get(); }
    WKCACFLayer* hostLayerForSublayers() const;
    WKCACFLayer* layerForSuperlayer() const;

    CompositingCoordinatesOrientation defaultContentsOrientation() const;
    void updateSublayerList();
    void updateLayerPosition();
    void updateLayerSize();
    void updateAnchorPoint();
    void updateTransform();
    void updateChildrenTransform();
    void updateMasksToBounds();
    void updateContentsOpaque();
    void updateBackfaceVisibility();
    void updateLayerPreserves3D();
    void updateLayerDrawsContent();
    void updateLayerBackgroundColor();

    void updateContentsImage();
    void updateContentsMedia();
    void updateContentsRect();
    void updateGeometryOrientation();
    
    void setupContentsLayer(WKCACFLayer*);
    WKCACFLayer* contentsLayer() const { return m_contentsLayer.get(); }
    
    RefPtr<WKCACFLayer> m_layer;
    RefPtr<WKCACFLayer> m_transformLayer;
    RefPtr<WKCACFLayer> m_contentsLayer;

    enum ContentsLayerPurpose {
        NoContentsLayer = 0,
        ContentsLayerForImage,
        ContentsLayerForMedia
    };
    
    ContentsLayerPurpose m_contentsLayerPurpose;
    bool m_contentsLayerHasBackgroundColor : 1;
    RetainPtr<CGImageRef> m_pendingContentsImage;
};

} // namespace WebCore

#endif // USE(ACCELERATED_COMPOSITING)

#endif // GraphicsLayerCACF_h_
