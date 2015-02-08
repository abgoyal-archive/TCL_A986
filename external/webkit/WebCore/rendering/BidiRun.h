

#ifndef BidiRun_h
#define BidiRun_h

#include <wtf/StdLibExtras.h>
#include "BidiResolver.h"
#include "RenderText.h"

namespace WebCore {

class BidiContext;
class InlineBox;

struct BidiRun : BidiCharacterRun {
    BidiRun(int start, int stop, RenderObject* object, BidiContext* context, WTF::Unicode::Direction dir)
        : BidiCharacterRun(start, stop, context, dir)
        , m_object(object)
        , m_box(0)
    {
    }

    void destroy();

    // Overloaded new operator.
    void* operator new(size_t, RenderArena*) throw();

    // Overridden to prevent the normal delete from being called.
    void operator delete(void*, size_t);

    BidiRun* next() { return static_cast<BidiRun*>(m_next); }

private:
    // The normal operator new is disallowed.
    void* operator new(size_t) throw();

public:
    RenderObject* m_object;
    InlineBox* m_box;
};

}

#endif // BidiRun_h
