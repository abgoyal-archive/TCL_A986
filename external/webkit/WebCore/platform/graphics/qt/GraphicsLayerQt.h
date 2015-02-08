

#ifndef GraphicsLayerQt_h
#define GraphicsLayerQt_h

#include "GraphicsLayer.h"
#include "GraphicsLayerClient.h"

namespace WebCore {

class GraphicsLayerQtImpl;

class GraphicsLayerQt : public GraphicsLayer {
    friend class GraphicsLayerQtImpl;

public:
    GraphicsLayerQt(GraphicsLayerClient*);
    virtual ~GraphicsLayerQt();

    // reimps from GraphicsLayer.h
    virtual NativeLayer nativeLayer() const;
    virtual PlatformLayer* platformLayer() const;
    virtual void setNeedsDisplay();
    virtual void setNeedsDisplayInRect(const FloatRect&);
    virtual void setParent(GraphicsLayer* layer);
    virtual void setName(const String& name);
    virtual bool setChildren(const Vector<GraphicsLayer*>&);
    virtual void addChild(GraphicsLayer*);
    virtual void addChildAtIndex(GraphicsLayer*, int index);
    virtual void addChildAbove(GraphicsLayer* layer, GraphicsLayer* sibling);
    virtual void addChildBelow(GraphicsLayer* layer, GraphicsLayer* sibling);
    virtual bool replaceChild(GraphicsLayer* oldChild, GraphicsLayer* newChild);
    virtual void removeFromParent();
    virtual void setMaskLayer(GraphicsLayer* layer);
    virtual void setPosition(const FloatPoint& p);
    virtual void setAnchorPoint(const FloatPoint3D& p);
    virtual void setSize(const FloatSize& size);
    virtual void setTransform(const TransformationMatrix& t);
    virtual void setChildrenTransform(const TransformationMatrix& t);
    virtual void setPreserves3D(bool b);
    virtual void setMasksToBounds(bool b);
    virtual void setDrawsContent(bool b);
    virtual void setBackgroundColor(const Color&);
    virtual void clearBackgroundColor();
    virtual void setContentsOpaque(bool b);
    virtual void setBackfaceVisibility(bool b);
    virtual void setOpacity(float opacity);
    virtual void setContentsRect(const IntRect& r);
    virtual bool addAnimation(const KeyframeValueList&, const IntSize& boxSize, const Animation*, const String& keyframesName, double timeOffset);
    virtual void removeAnimationsForProperty(AnimatedPropertyID);
    virtual void removeAnimationsForKeyframes(const String& keyframesName);
    virtual void pauseAnimation(const String& keyframesName, double timeOffset);
    virtual void suspendAnimations(double time);
    virtual void resumeAnimations();
    virtual void setContentsToImage(Image*);
    virtual void setContentsBackgroundColor(const Color&);
    virtual void setGeometryOrientation(CompositingCoordinatesOrientation orientation);
    virtual void setContentsOrientation(CompositingCoordinatesOrientation orientation);
    virtual void distributeOpacity(float);
    virtual float accumulatedOpacity() const;
    virtual void syncCompositingState();

private:
    OwnPtr<GraphicsLayerQtImpl> m_impl;
};

}
#endif // GraphicsLayerQt_h
