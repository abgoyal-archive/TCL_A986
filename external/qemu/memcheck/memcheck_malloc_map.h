


#ifndef QEMU_MEMCHECK_MEMCHECK_MALLOC_MAP_H
#define QEMU_MEMCHECK_MEMCHECK_MALLOC_MAP_H

#ifndef CONFIG_MEMCHECK
#error CONFIG_MEMCHECK is not defined.
#endif  // CONFIG_MEMCHECK

#include "sys-tree.h"
#include "memcheck_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Allocation descriptors map. */
typedef struct AllocMap {
    /* Head of the map. */
    struct AllocMapEntry*   rbh_root;
} AllocMap;

// =============================================================================
// Map API
// =============================================================================

void allocmap_init(AllocMap* map);

RBTMapResult allocmap_insert(AllocMap* map,
                             const MallocDescEx* desc,
                             MallocDescEx* replaced);

MallocDescEx* allocmap_find(const AllocMap* map,
                            target_ulong address,
                            uint32_t block_size);

int allocmap_pull(AllocMap* map, target_ulong address, MallocDescEx* pulled);

int allocmap_pull_first(AllocMap* map, MallocDescEx* pulled);

int allocmap_copy(AllocMap* to,
                  const AllocMap* from,
                  uint32_t set_flags,
                  uint32_t clear_flags);

int allocmap_empty(AllocMap* map);

#ifdef __cplusplus
};  /* end of extern "C" */
#endif

#endif  // QEMU_MEMCHECK_MEMCHECK_MALLOC_MAP_H
