

#ifndef HTMLFrameElement_h
#define HTMLFrameElement_h

#include "HTMLFrameElementBase.h"

namespace WebCore {

class HTMLFrameElement : public HTMLFrameElementBase {
public:
    static PassRefPtr<HTMLFrameElement> create(const QualifiedName&, Document*);

    bool hasFrameBorder() const { return m_frameBorder; }

    bool noResize() const { return m_noResize; }
    void setNoResize(bool);

private:
    HTMLFrameElement(const QualifiedName&, Document*);

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusForbidden; }
    virtual int tagPriority() const { return 0; }
  
    virtual void attach();

    virtual bool rendererIsNeeded(RenderStyle*);
    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);
    
    virtual void parseMappedAttribute(MappedAttribute*);

    bool m_frameBorder;
    bool m_frameBorderSet;

    bool m_noResize;
};

} // namespace WebCore

#endif // HTMLFrameElement_h
