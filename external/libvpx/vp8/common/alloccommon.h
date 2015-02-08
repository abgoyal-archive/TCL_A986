


#ifndef __INC_ALLOCCOMMON_H
#define __INC_ALLOCCOMMON_H

#include "onyxc_int.h"

void vp8_create_common(VP8_COMMON *oci);
void vp8_remove_common(VP8_COMMON *oci);
void vp8_de_alloc_frame_buffers(VP8_COMMON *oci);
int vp8_alloc_frame_buffers(VP8_COMMON *oci, int width, int height);
void vp8_setup_version(VP8_COMMON *oci);

#endif
