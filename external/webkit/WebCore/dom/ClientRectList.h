

#ifndef ClientRectList_h
#define ClientRectList_h

#include "FloatQuad.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/Vector.h>

namespace WebCore {

    class ClientRect;

    class ClientRectList : public RefCounted<ClientRectList> {
    public:
        static PassRefPtr<ClientRectList> create() { return adoptRef(new ClientRectList); }
        static PassRefPtr<ClientRectList> create(const Vector<FloatQuad>& quads) { return adoptRef(new ClientRectList(quads)); }
        ~ClientRectList();

        unsigned length() const;
        ClientRect* item(unsigned index);

    private:
        ClientRectList();
        ClientRectList(const Vector<FloatQuad>&);

        Vector<RefPtr<ClientRect> > m_list;
    }; 

} // namespace WebCore

#endif // ClientRectList_h
