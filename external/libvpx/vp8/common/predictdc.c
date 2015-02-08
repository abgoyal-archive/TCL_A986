


#include <stdlib.h>
#include "blockd.h"


void vp8_predict_dc(short *lastdc, short *thisdc, short quant, short *cons)
{
    int diff;
    int sign;
    int last_dc = *lastdc;
    int this_dc = *thisdc;

    if (*cons  > DCPREDCNTTHRESH)
    {
        this_dc += last_dc;
    }

    diff = abs(last_dc - this_dc);
    sign  = (last_dc >> 31) ^(this_dc >> 31);
    sign |= (!last_dc | !this_dc);

    if (sign)
    {
        *cons = 0;
    }
    else
    {
        if (diff <= DCPREDSIMTHRESH * quant)
            (*cons)++ ;
    }

    *thisdc = this_dc;
    *lastdc = this_dc;
}
