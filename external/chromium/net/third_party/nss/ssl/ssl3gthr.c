/* $Id: ssl3gthr.c,v 1.9 2008/11/20 07:37:25 nelson%bolyard.com Exp $ */

#include "cert.h"
#include "ssl.h"
#include "sslimpl.h"
#include "ssl3prot.h"

static int
ssl3_GatherData(sslSocket *ss, sslGather *gs, int flags)
{
    unsigned char *bp;
    unsigned char *lbp;
    int            nb;
    int            err;
    int            rv		= 1;

    PORT_Assert( ss->opt.noLocks || ssl_HaveRecvBufLock(ss) );
    if (gs->state == GS_INIT) {
	gs->state       = GS_HEADER;
	gs->remainder   = 5;
	gs->offset      = 0;
	gs->writeOffset = 0;
	gs->readOffset  = 0;
	gs->inbuf.len   = 0;
    }
    
    lbp = gs->inbuf.buf;
    for(;;) {
	SSL_TRC(30, ("%d: SSL3[%d]: gather state %d (need %d more)",
		SSL_GETPID(), ss->fd, gs->state, gs->remainder));
	bp = ((gs->state != GS_HEADER) ? lbp : gs->hdr) + gs->offset;
	nb = ssl_DefRecv(ss, bp, gs->remainder, flags);

	if (nb > 0) {
	    PRINT_BUF(60, (ss, "raw gather data:", bp, nb));
	} else if (nb == 0) {
	    /* EOF */
	    SSL_TRC(30, ("%d: SSL3[%d]: EOF", SSL_GETPID(), ss->fd));
	    rv = 0;
	    break;
	} else /* if (nb < 0) */ {
	    SSL_DBG(("%d: SSL3[%d]: recv error %d", SSL_GETPID(), ss->fd,
		     PR_GetError()));
	    rv = SECFailure;
	    break;
	}

	PORT_Assert( nb <= gs->remainder );
	if (nb > gs->remainder) {
	    /* ssl_DefRecv is misbehaving!  this error is fatal to SSL. */
	    gs->state = GS_INIT;         /* so we don't crash next time */
	    rv = SECFailure;
	    break;
	}

	gs->offset    += nb;
	gs->remainder -= nb;
	if (gs->state == GS_DATA)
	    gs->inbuf.len += nb;

	/* if there's more to go, read some more. */
	if (gs->remainder > 0) {
	    continue;
	}

	/* have received entire record header, or entire record. */
	switch (gs->state) {
	case GS_HEADER:
	    /*
	    ** Have received SSL3 record header in gs->hdr.
	    ** Now extract the length of the following encrypted data, 
	    ** and then read in the rest of the SSL3 record into gs->inbuf.
	    */
	    gs->remainder = (gs->hdr[3] << 8) | gs->hdr[4];

	    /* This is the max fragment length for an encrypted fragment
	    ** plus the size of the record header.
	    */
	    if(gs->remainder > (MAX_FRAGMENT_LENGTH + 2048 + 5)) {
		SSL3_SendAlert(ss, alert_fatal, unexpected_message);
		gs->state = GS_INIT;
		PORT_SetError(SSL_ERROR_RX_RECORD_TOO_LONG);
		return SECFailure;
	    }

	    gs->state     = GS_DATA;
	    gs->offset    = 0;
	    gs->inbuf.len = 0;

	    if (gs->remainder > gs->inbuf.space) {
		err = sslBuffer_Grow(&gs->inbuf, gs->remainder);
		if (err) {	/* realloc has set error code to no mem. */
		    return err;
		}
		lbp = gs->inbuf.buf;
	    }
	    break;	/* End this case.  Continue around the loop. */


	case GS_DATA:
	    /* 
	    ** SSL3 record has been completely received.
	    */
	    gs->state = GS_INIT;
	    return 1;
	}
    }

    return rv;
}

int
ssl3_GatherCompleteHandshake(sslSocket *ss, int flags)
{
    SSL3Ciphertext cText;
    int            rv;

    PORT_Assert( ss->opt.noLocks || ssl_HaveRecvBufLock(ss) );
    do {
	/* bring in the next sslv3 record. */
	rv = ssl3_GatherData(ss, &ss->gs, flags);
	if (rv <= 0) {
	    return rv;
	}
	
	/* decipher it, and handle it if it's a handshake. 
	 * If it's application data, ss->gs.buf will not be empty upon return. 
	 */
	cText.type    = (SSL3ContentType)ss->gs.hdr[0];
	cText.version = (ss->gs.hdr[1] << 8) | ss->gs.hdr[2];
	cText.buf     = &ss->gs.inbuf;
	rv = ssl3_HandleRecord(ss, &cText, &ss->gs.buf);
	if (rv < 0) {
	    return ss->recvdCloseNotify ? 0 : rv;
	}
    } while (ss->ssl3.hs.ws != idle_handshake && ss->gs.buf.len == 0);

    ss->gs.readOffset = 0;
    ss->gs.writeOffset = ss->gs.buf.len;
    return 1;
}

int
ssl3_GatherAppDataRecord(sslSocket *ss, int flags)
{
    int            rv;

    PORT_Assert( ss->opt.noLocks || ssl_HaveRecvBufLock(ss) );
    do {
	rv = ssl3_GatherCompleteHandshake(ss, flags);
    } while (rv > 0 && ss->gs.buf.len == 0);

    return rv;
}
