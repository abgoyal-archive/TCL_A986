

#ifndef StyleList_h
#define StyleList_h

#include "StyleBase.h"
#include <wtf/Forward.h>
#include <wtf/RefPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

    // a style class which has a list of children (StyleSheets for example)
    class StyleList : public StyleBase {
    public:
        unsigned length() { return m_children.size(); }
        StyleBase* item(unsigned num) { return num < length() ? m_children[num].get() : 0; }

        void append(PassRefPtr<StyleBase>);
        void insert(unsigned position, PassRefPtr<StyleBase>);
        void remove(unsigned position);

    protected:
        StyleList(StyleBase* parent) : StyleBase(parent) { }

        Vector<RefPtr<StyleBase> > m_children;
    };
}

#endif
