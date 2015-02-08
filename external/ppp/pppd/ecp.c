

#define RCSID	"$Id: ecp.c,v 1.4 2004/11/04 10:02:26 paulus Exp $"

static const char rcsid[] = RCSID;

#include <string.h>

#include "pppd.h"
#include "fsm.h"
#include "ecp.h"

static option_t ecp_option_list[] = {
    { "noecp", o_bool, &ecp_protent.enabled_flag,
      "Disable ECP negotiation" },
    { "-ecp", o_bool, &ecp_protent.enabled_flag,
      "Disable ECP negotiation", OPT_ALIAS },

    { NULL }
};

static void ecp_init __P((int unit));
static int  ecp_printpkt __P((u_char *pkt, int len,
			      void (*printer) __P((void *, char *, ...)),
			      void *arg));

struct protent ecp_protent = {
    PPP_ECP,
    ecp_init,
    NULL, /* ecp_input, */
    NULL, /* ecp_protrej, */
    NULL, /* ecp_lowerup, */
    NULL, /* ecp_lowerdown, */
    NULL, /* ecp_open, */
    NULL, /* ecp_close, */
    ecp_printpkt,
    NULL, /* ecp_datainput, */
    0,
    "ECP",
    "Encrypted",
    ecp_option_list,
    NULL,
    NULL,
    NULL
};

fsm ecp_fsm[NUM_PPP];
ecp_options ecp_wantoptions[NUM_PPP];	/* what to request the peer to use */
ecp_options ecp_gotoptions[NUM_PPP];	/* what the peer agreed to do */
ecp_options ecp_allowoptions[NUM_PPP];	/* what we'll agree to do */
ecp_options ecp_hisoptions[NUM_PPP];	/* what we agreed to do */

static fsm_callbacks ecp_callbacks = {
    NULL, /* ecp_resetci, */
    NULL, /* ecp_cilen, */
    NULL, /* ecp_addci, */
    NULL, /* ecp_ackci, */
    NULL, /* ecp_nakci, */
    NULL, /* ecp_rejci, */
    NULL, /* ecp_reqci, */
    NULL, /* ecp_up, */
    NULL, /* ecp_down, */
    NULL,
    NULL,
    NULL,
    NULL,
    NULL, /* ecp_extcode, */
    "ECP"
};

static void
ecp_init(unit)
    int unit;
{
    fsm *f = &ecp_fsm[unit];

    f->unit = unit;
    f->protocol = PPP_ECP;
    f->callbacks = &ecp_callbacks;
    fsm_init(f);

    memset(&ecp_wantoptions[unit],  0, sizeof(ecp_options));
    memset(&ecp_gotoptions[unit],   0, sizeof(ecp_options));
    memset(&ecp_allowoptions[unit], 0, sizeof(ecp_options));
    memset(&ecp_hisoptions[unit],   0, sizeof(ecp_options));

}


static int
ecp_printpkt(p, plen, printer, arg)
    u_char *p;
    int plen;
    void (*printer) __P((void *, char *, ...));
    void *arg;
{
    return 0;
}

