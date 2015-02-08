

#ifndef RenderApplet_h
#define RenderApplet_h

#include "RenderWidget.h"
#include "StringHash.h"

namespace WebCore {

    class HTMLAppletElement;

    class RenderApplet : public RenderWidget {
    public:
        RenderApplet(HTMLAppletElement*, const HashMap<String, String>& args);

        void createWidgetIfNecessary();

    private:
        virtual const char* renderName() const { return "RenderApplet"; }

        virtual bool isApplet() const { return true; }

        virtual void layout();
        virtual IntSize intrinsicSize() const;

        HashMap<String, String> m_args;
    };

    inline RenderApplet* toRenderApplet(RenderObject* object)
    {
        ASSERT(!object || object->isApplet());
        return static_cast<RenderApplet*>(object);
    }

    // This will catch anyone doing an unnecessary cast.
    void toRenderApplet(const RenderApplet*);

} // namespace WebCore

#endif // RenderApplet_h
