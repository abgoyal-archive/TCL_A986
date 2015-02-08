

#ifndef WEBCORE_VIEW_BRIDGE_H
#define WEBCORE_VIEW_BRIDGE_H

// TODO: move this outside of jni directory

#include "IntRect.h"
#include "WebCoreRefObject.h"

namespace WebCore
{
    class GraphicsContext;
}

class WebCoreViewBridge : public WebCoreRefObject {
public:
    WebCoreViewBridge() : 
        mBounds(0,0,0,0),
        m_windowBounds(0,0,0,0)
    {}
    virtual ~WebCoreViewBridge() {}

    virtual void draw(WebCore::GraphicsContext* ctx, 
        const WebCore::IntRect& rect) = 0;

    const WebCore::IntRect& getBounds() const 
    {
        return mBounds;
    }
    
    const WebCore::IntRect& getWindowBounds() const
    {
        return m_windowBounds;
    }

    void setSize(int w, int h)
    {
        mBounds.setWidth(w);
        mBounds.setHeight(h);
    }

    void setLocation(int x, int y)
    {
        mBounds.setX(x);
        mBounds.setY(y);
    }

    void setWindowBounds(int x, int y, int h, int v)
    {
        m_windowBounds = WebCore::IntRect(x, y, h, v);
    }

    int width() const     { return mBounds.width(); }
    int height() const    { return mBounds.height(); }
    int locX() const      { return mBounds.x(); }
    int locY() const      { return mBounds.y(); }

    virtual bool forFrameView() const { return false; }
    virtual bool forPluginView() const { return false; }

private:
    WebCore::IntRect    mBounds;
    WebCore::IntRect    m_windowBounds;
};

#endif // WEBCORE_VIEW_BRIDGE_H
