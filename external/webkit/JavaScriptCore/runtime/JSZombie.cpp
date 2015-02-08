

#include "config.h"
#include "JSZombie.h"
#include "ClassInfo.h"

#if ENABLE(JSC_ZOMBIES)

namespace JSC {

const ClassInfo JSZombie::s_info = { "Zombie", 0, 0, 0 };

Structure* JSZombie::leakedZombieStructure() {
    static Structure* structure = 0;
    if (!structure) {
        Structure::startIgnoringLeaks();
        structure = Structure::create(jsNull(), TypeInfo(UnspecifiedType)).releaseRef();
        Structure::stopIgnoringLeaks();
    }
    return structure;
}

}

#endif // ENABLE(JSC_ZOMBIES)
