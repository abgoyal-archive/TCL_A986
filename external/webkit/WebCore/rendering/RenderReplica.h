

#ifndef RenderReplica_h
#define RenderReplica_h

#include "RenderBox.h"

namespace WebCore {

class RenderReplica : public RenderBox {
public:
    RenderReplica(Node*);
    virtual ~RenderReplica();

    virtual const char* renderName() const { return "RenderReplica"; }
    
    virtual bool requiresLayer() const { return true; }

    virtual void layout();
    virtual void calcPrefWidths();
    
    virtual void paint(PaintInfo&, int tx, int ty);

private:
    virtual bool isReplica() const { return true; }

};

} // namespace WebCore

#endif // RenderReplica_h
