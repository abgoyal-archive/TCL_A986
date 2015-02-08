


#ifndef __QEMU_VNC_TLS_H__
#define __QEMU_VNC_TLS_H__

#include <gnutls/gnutls.h>
#include <gnutls/x509.h>

#include "acl.h"

enum {
    VNC_WIREMODE_CLEAR,
    VNC_WIREMODE_TLS,
};

typedef struct VncDisplayTLS VncDisplayTLS;
typedef struct VncStateTLS VncStateTLS;

/* Server state */
struct VncDisplayTLS {
    int x509verify; /* Non-zero if server requests & validates client cert */
    qemu_acl *acl;

    /* Paths to x509 certs/keys */
    char *x509cacert;
    char *x509cacrl;
    char *x509cert;
    char *x509key;
};

/* Per client state */
struct VncStateTLS {
    /* Whether data is being TLS encrypted yet */
    int wiremode;
    gnutls_session_t session;

    /* Client's Distinguished Name from the x509 cert */
    char *dname;
};

int vnc_tls_client_setup(VncState *vs, int x509Creds);
void vnc_tls_client_cleanup(VncState *vs);

int vnc_tls_validate_certificate(VncState *vs);

int vnc_tls_set_x509_creds_dir(VncDisplay *vd,
			       const char *path);


#endif /* __QEMU_VNC_TLS_H__ */

