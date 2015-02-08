


#ifndef QEMU_MEMCHECK_MEMCHECK_MMRANGE_MAP_H
#define QEMU_MEMCHECK_MEMCHECK_MMRANGE_MAP_H

#ifndef CONFIG_MEMCHECK
#error CONFIG_MEMCHECK is not defined.
#endif  // CONFIG_MEMCHECK

#include "sys-tree.h"
#include "memcheck_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Memory mapping range map. */
typedef struct MMRangeMap {
    /* Head of the map. */
    struct MMRangeMapEntry* rbh_root;
} MMRangeMap;

// =============================================================================
// Map API
// =============================================================================

void mmrangemap_init(MMRangeMap* map);

RBTMapResult mmrangemap_insert(MMRangeMap* map,
                               const MMRangeDesc* desc,
                               MMRangeDesc* replaced);

MMRangeDesc* mmrangemap_find(const MMRangeMap* map,
                             target_ulong start,
                             target_ulong end);

int mmrangemap_pull(MMRangeMap* map,
                    target_ulong start,
                    target_ulong end,
                    MMRangeDesc* pulled);

int mmrangemap_copy(MMRangeMap* to, const MMRangeMap* from);

int mmrangemap_empty(MMRangeMap* map);

#ifdef __cplusplus
};  /* end of extern "C" */
#endif

#endif  // QEMU_MEMCHECK_MEMCHECK_MMRANGE_MAP_H
