

#ifndef SVGElementInstanceList_h
#define SVGElementInstanceList_h

#if ENABLE(SVG)

#include <wtf/RefCounted.h>
#include "SVGElementInstance.h"

namespace WebCore {

    class SVGElementInstanceList : public RefCounted<SVGElementInstanceList> {
    public:
        static PassRefPtr<SVGElementInstanceList> create(PassRefPtr<SVGElementInstance> rootInstance) { return adoptRef(new SVGElementInstanceList(rootInstance)); }
        virtual ~SVGElementInstanceList();

        unsigned length() const;
        SVGElementInstance* item(unsigned index);

    private:
        SVGElementInstanceList(PassRefPtr<SVGElementInstance> rootInstance);
        RefPtr<SVGElementInstance> m_rootInstance;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif

// vim:ts=4:noet
