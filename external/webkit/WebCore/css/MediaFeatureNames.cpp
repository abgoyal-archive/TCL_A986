

#include "config.h"

#ifdef SKIP_STATIC_CONSTRUCTORS_ON_GCC
#define CSS_MEDIAQUERY_NAMES_HIDE_GLOBALS 1
#endif

#include "MediaFeatureNames.h"
#include "StaticConstructors.h"

namespace WebCore {
namespace MediaFeatureNames {

#define DEFINE_MEDIAFEATURE_GLOBAL(name, str) \
    DEFINE_GLOBAL(AtomicString, name##MediaFeature, str)
CSS_MEDIAQUERY_NAMES_FOR_EACH_MEDIAFEATURE(DEFINE_MEDIAFEATURE_GLOBAL)
#undef DEFINE_MEDIAFEATURE_GLOBAL

void init()
{
    static bool initialized;
    if (!initialized) {
       // Use placement new to initialize the globals.

        AtomicString::init();
        #define INITIALIZE_GLOBAL(name, str) new ((void*)&name##MediaFeature) AtomicString(str);
        CSS_MEDIAQUERY_NAMES_FOR_EACH_MEDIAFEATURE(INITIALIZE_GLOBAL)
        #undef INITIALIZE_GLOBAL
        initialized = true;
    }
}

} // namespace MediaFeatureNames
} // namespace WebCore
