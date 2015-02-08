

#ifndef EmbeddedWidget_h
#define EmbeddedWidget_h

#include <WebCore/COMPtr.h>
#include <WebCore/IntRect.h>
#include <WebCore/PluginView.h>

namespace WebCore {
    class Element;
    class IntSize;
}

interface IWebEmbeddedView;

class EmbeddedWidget : public WebCore::Widget, public WebCore::PluginManualLoader {
public:
    static PassRefPtr<EmbeddedWidget> create(IWebEmbeddedView*, WebCore::Element* element, HWND parentWindow, const WebCore::IntSize&);
    ~EmbeddedWidget();

private:
    EmbeddedWidget(IWebEmbeddedView* view, WebCore::Element* element)
        : m_view(view)
        , m_element(element)
        , m_window(0)
        , m_isVisible(false)
        , m_attachedToWindow(false)
    {
    }

    bool createWindow(HWND parentWindow, const WebCore::IntSize& size);

    virtual void didReceiveResponse(const WebCore::ResourceResponse&);
    virtual void didReceiveData(const char*, int);
    virtual void didFinishLoading();
    virtual void didFail(const WebCore::ResourceError&);

    virtual void invalidateRect(const WebCore::IntRect&);
    virtual void setFrameRect(const WebCore::IntRect&);
    virtual void frameRectsChanged();
    virtual void setFocus();
    virtual void show();
    virtual void hide();
    virtual WebCore::IntRect windowClipRect() const;
    virtual void setParent(WebCore::ScrollView*);

    virtual void attachToWindow();
    virtual void detachFromWindow();

    COMPtr<IWebEmbeddedView> m_view;
    WebCore::Element* m_element;
    HWND m_window;

    bool m_isVisible;
    bool m_attachedToWindow;
        
    WebCore::IntRect m_clipRect; // The clip rect to apply to an embedded view.
    WebCore::IntRect m_windowRect; // Our window rect.
};

#endif // EmbeddedWidget_h
