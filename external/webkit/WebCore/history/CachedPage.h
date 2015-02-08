

#ifndef CachedPage_h
#define CachedPage_h

#include "CachedFrame.h"

namespace WebCore {
    
class Document;
class DocumentLoader;
class Page;

class CachedPage : public RefCounted<CachedPage> {
public:
    static PassRefPtr<CachedPage> create(Page*);
    ~CachedPage();

    void restore(Page*);
    void clear();
    void destroy();

    Document* document() const { return m_cachedMainFrame->document(); }
    DocumentLoader* documentLoader() const { return m_cachedMainFrame->documentLoader(); }

    double timeStamp() const { return m_timeStamp; }
    
    CachedFrame* cachedMainFrame() { return m_cachedMainFrame.get(); }

private:
    CachedPage(Page*);

    double m_timeStamp;
    RefPtr<CachedFrame> m_cachedMainFrame;
};

} // namespace WebCore

#endif // CachedPage_h

