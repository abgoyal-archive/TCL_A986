

#if defined(ASMV) && (!defined(NOOLDPENTIUMCODE))
#include "deflate.h"

#ifdef ADD_UNDERLINE_ASMFUNC
#define longest_match_7fff _longest_match_7fff
#define longest_match_686  _longest_match_686
#define cpudetect32        _cpudetect32
#endif


unsigned long cpudetect32();

uInt longest_match_c(
    deflate_state *s,
    IPos cur_match);                             /* current match */


uInt longest_match_7fff(
    deflate_state *s,
    IPos cur_match);                             /* current match */

uInt longest_match_686(
    deflate_state *s,
    IPos cur_match);                             /* current match */


static uInt iIsPPro=2;

void match_init ()
{
    iIsPPro = (((cpudetect32()/0x100)&0xf)>=6) ? 1 : 0;
}

uInt longest_match(
    deflate_state *s,
    IPos cur_match)                             /* current match */
{
    if (iIsPPro!=0)
        return longest_match_686(s,cur_match);

    if (s->w_mask != 0x7fff)
        return longest_match_686(s,cur_match);

    /* now ((s->w_mask == 0x7fff) && (iIsPPro==0)) */
        return longest_match_7fff(s,cur_match);
}


#endif /* defined(ASMV) && (!defined(NOOLDPENTIUMCODE)) */
