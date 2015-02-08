


#ifndef __INC_ENCODEMV_H
#define __INC_ENCODEMV_H

#include "onyx_int.h"

void vp8_write_mvprobs(VP8_COMP *);
void vp8_encode_motion_vector(vp8_writer *, const MV *, const MV_CONTEXT *);
void vp8_build_component_cost_table(int *mvcost[2], int *mvsadcost[2], const MV_CONTEXT *mvc, int mvc_flag[2]);

#endif
