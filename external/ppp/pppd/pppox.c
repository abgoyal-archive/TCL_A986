

#include <unistd.h>
#include "pppd.h"

static int pppox_set(char **);
static int pppox_connect();
static void pppox_disconnect();

static option_t pppox_options[] = {
    {"pppox", o_special, pppox_set, "PPPoX socket", OPT_DEVNAM},
    {NULL},
};

static struct channel pppox_channel = {
    .options = pppox_options,
    .process_extra_options = NULL,
    .check_options = NULL,
    .connect = pppox_connect,
    .disconnect = pppox_disconnect,
    .establish_ppp = generic_establish_ppp,
    .disestablish_ppp = generic_disestablish_ppp,
    .send_config = NULL,
    .recv_config = NULL,
    .cleanup = NULL,
    .close = NULL,
};

static int pppox = -1;

static int pppox_set(char **argv) {
    if (!int_option(*argv, &pppox)) {
        return 0;
    }
    info("Using PPPoX (socket = %d)", pppox);
    the_channel = &pppox_channel;
    return 1;
}

static int pppox_connect() {
    return pppox;
}

static void pppox_disconnect() {
    if (pppox != -1) {
        close(pppox);
        pppox = -1;
    }
}

void pppox_init() {
    add_options(pppox_options);
}
