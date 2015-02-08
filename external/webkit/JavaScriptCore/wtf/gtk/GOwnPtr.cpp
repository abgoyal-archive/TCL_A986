

#include "config.h"
#include "GOwnPtr.h"

#include <glib.h>

namespace WTF {

template <> void freeOwnedGPtr<GError>(GError* ptr)
{
    if (ptr)
        g_error_free(ptr);
}

template <> void freeOwnedGPtr<GList>(GList* ptr)
{
    g_list_free(ptr);
}

template <> void freeOwnedGPtr<GCond>(GCond* ptr)
{
    if (ptr)
        g_cond_free(ptr);
}

template <> void freeOwnedGPtr<GMutex>(GMutex* ptr)
{
    if (ptr)
        g_mutex_free(ptr);
}

template <> void freeOwnedGPtr<GPatternSpec>(GPatternSpec* ptr)
{
    if (ptr)
        g_pattern_spec_free(ptr);
}

template <> void freeOwnedGPtr<GDir>(GDir* ptr)
{
    if (ptr)
        g_dir_close(ptr);
}
} // namespace WTF
