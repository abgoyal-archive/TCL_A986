

#include "config.h"
#include "GRefPtr.h"

#include <glib.h>

namespace WTF {

template <> GHashTable* refGPtr(GHashTable* ptr)
{
    if (ptr)
        g_hash_table_ref(ptr);
    return ptr;
}

template <> void derefGPtr(GHashTable* ptr)
{
    g_hash_table_unref(ptr);
}

} // namespace WTF
