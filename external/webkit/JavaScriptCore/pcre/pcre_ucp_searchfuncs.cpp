



#include "config.h"
#include "pcre_internal.h"

#include "ucpinternal.h"       /* Internal table details */
#include "ucptable.cpp"        /* The table itself */



int jsc_pcre_ucp_othercase(unsigned c)
{
    int bot = 0;
    int top = sizeof(ucp_table) / sizeof(cnode);
    int mid;
    
    /* The table is searched using a binary chop. You might think that using
     intermediate variables to hold some of the common expressions would speed
     things up, but tests with gcc 3.4.4 on Linux showed that, on the contrary, it
     makes things a lot slower. */
    
    for (;;) {
        if (top <= bot)
            return -1;
        mid = (bot + top) >> 1;
        if (c == (ucp_table[mid].f0 & f0_charmask))
            break;
        if (c < (ucp_table[mid].f0 & f0_charmask))
            top = mid;
        else {
            if ((ucp_table[mid].f0 & f0_rangeflag) && (c <= (ucp_table[mid].f0 & f0_charmask) + (ucp_table[mid].f1 & f1_rangemask)))
                break;
            bot = mid + 1;
        }
    }
    
    /* Found an entry in the table. Return -1 for a range entry. Otherwise return
     the other case if there is one, else -1. */
    
    if (ucp_table[mid].f0 & f0_rangeflag)
        return -1;
    
    int offset = ucp_table[mid].f1 & f1_casemask;
    if (offset & f1_caseneg)
        offset |= f1_caseneg;
    return !offset ? -1 : c + offset;
}
