


#ifndef __VPX_MEM_NDS_H__
#define __VPX_MEM_NDS_H__

#if defined(__cplusplus)
extern "C" {
#endif

#include <nitro.h>
#include <nitro/os.h>

    void *vpx_mem_nds_alloc(osarena_id id, osheap_handle handle, size_t size, size_t align);
    void vpx_mem_nds_free(osarena_id id, osheap_handle handle, void *mem);
    int vpx_nds_alloc_heap(osarena_id id, u32 size);

#if defined(__cplusplus)
}
#endif

#endif /*__VPX_MEM_NDS_H__*/
