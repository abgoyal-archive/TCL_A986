
 
#ifndef CachedFrame_h
#define CachedFrame_h

#include "KURL.h"
#include "ScriptCachedFrameData.h"
#include <wtf/RefPtr.h>

namespace WebCore {

class CachedFrame;
class CachedFramePlatformData;
class Document;
class DocumentLoader;
class FrameView;
class Node;

typedef Vector<RefPtr<CachedFrame> > CachedFrameVector;

class CachedFrameBase {
public:
    void restore();

    Document* document() const { return m_document.get(); }
    FrameView* view() const { return m_view.get(); }
    const KURL& url() const { return m_url; }
    DOMWindow* domWindow() const { return m_cachedFrameScriptData->domWindow(); }
    bool isMainFrame() { return m_isMainFrame; }

protected:
    CachedFrameBase(Frame*);
    ~CachedFrameBase();
    
    RefPtr<Document> m_document;
    RefPtr<DocumentLoader> m_documentLoader;
    RefPtr<FrameView> m_view;
    RefPtr<Node> m_mousePressNode;
    KURL m_url;
    OwnPtr<ScriptCachedFrameData> m_cachedFrameScriptData;
    OwnPtr<CachedFramePlatformData> m_cachedFramePlatformData;
    bool m_isMainFrame;
    
    CachedFrameVector m_childFrames;
};

class CachedFrame : public RefCounted<CachedFrame>, private CachedFrameBase {
public:
    static PassRefPtr<CachedFrame> create(Frame* frame) { return adoptRef(new CachedFrame(frame)); }

    void open();
    void clear();
    void destroy();

    void setCachedFramePlatformData(CachedFramePlatformData* data);
    CachedFramePlatformData* cachedFramePlatformData();

    using CachedFrameBase::document;
    using CachedFrameBase::view;
    DocumentLoader* documentLoader() const { return m_documentLoader.get(); }
    Node* mousePressNode() const { return m_mousePressNode.get(); }

    int descendantFrameCount() const;

private:
    CachedFrame(Frame*);
};

} // namespace WebCore

#endif // CachedFrame_h
