/* $Id: sslsecur.c,v 1.42 2008/10/03 19:20:20 wtc%google.com Exp $ */
#include "cert.h"
#include "secitem.h"
#include "keyhi.h"
#include "ssl.h"
#include "sslimpl.h"
#include "sslproto.h"
#include "secoid.h"	/* for SECOID_GetALgorithmTag */
#include "pk11func.h"	/* for PK11_GenerateRandom */
#include "nss.h"        /* for NSS_RegisterShutdown */
#include "prinit.h"     /* for PR_CallOnceWithArg */

#define MAX_BLOCK_CYPHER_SIZE	32

#define TEST_FOR_FAILURE	/* reminder */
#define SET_ERROR_CODE		/* reminder */

int 
ssl_Do1stHandshake(sslSocket *ss)
{
    int rv        = SECSuccess;
    int loopCount = 0;

    do {
	PORT_Assert(ss->opt.noLocks ||  ssl_Have1stHandshakeLock(ss) );
	PORT_Assert(ss->opt.noLocks || !ssl_HaveRecvBufLock(ss));
	PORT_Assert(ss->opt.noLocks || !ssl_HaveXmitBufLock(ss));

	if (ss->handshake == 0) {
	    /* Previous handshake finished. Switch to next one */
	    ss->handshake = ss->nextHandshake;
	    ss->nextHandshake = 0;
	}
	if (ss->handshake == 0) {
	    /* Previous handshake finished. Switch to security handshake */
	    ss->handshake = ss->securityHandshake;
	    ss->securityHandshake = 0;
	}
	if (ss->handshake == 0) {
	    ssl_GetRecvBufLock(ss);
	    ss->gs.recordLen = 0;
	    ssl_ReleaseRecvBufLock(ss);

	    SSL_TRC(3, ("%d: SSL[%d]: handshake is completed",
			SSL_GETPID(), ss->fd));
            /* call handshake callback for ssl v2 */
	    /* for v3 this is done in ssl3_HandleFinished() */
	    if ((ss->handshakeCallback != NULL) && /* has callback */
		(!ss->firstHsDone) &&              /* only first time */
		(ss->version < SSL_LIBRARY_VERSION_3_0)) {  /* not ssl3 */
		ss->firstHsDone     = PR_TRUE;
		(ss->handshakeCallback)(ss->fd, ss->handshakeCallbackData);
	    }
	    ss->firstHsDone         = PR_TRUE;
	    ss->gs.writeOffset = 0;
	    ss->gs.readOffset  = 0;
	    break;
	}
	rv = (*ss->handshake)(ss);
	++loopCount;
    /* This code must continue to loop on SECWouldBlock, 
     * or any positive value.	See XXX_1 comments.
     */
    } while (rv != SECFailure);  	/* was (rv >= 0); XXX_1 */

    PORT_Assert(ss->opt.noLocks || !ssl_HaveRecvBufLock(ss));
    PORT_Assert(ss->opt.noLocks || !ssl_HaveXmitBufLock(ss));

    if (rv == SECWouldBlock) {
	PORT_SetError(PR_WOULD_BLOCK_ERROR);
	rv = SECFailure;
    }
    return rv;
}

static SECStatus
AlwaysBlock(sslSocket *ss)
{
    PORT_SetError(PR_WOULD_BLOCK_ERROR);	/* perhaps redundant. */
    return SECWouldBlock;
}

void
ssl_SetAlwaysBlock(sslSocket *ss)
{
    if (!ss->firstHsDone) {
	ss->handshake = AlwaysBlock;
	ss->nextHandshake = 0;
    }
}

static SECStatus 
ssl_SetTimeout(PRFileDesc *fd, PRIntervalTime timeout)
{
    sslSocket *ss;

    ss = ssl_FindSocket(fd);
    if (!ss) {
	SSL_DBG(("%d: SSL[%d]: bad socket in SetTimeout", SSL_GETPID(), fd));
	return SECFailure;
    }
    SSL_LOCK_READER(ss);
    ss->rTimeout = timeout;
    if (ss->opt.fdx) {
        SSL_LOCK_WRITER(ss);
    }
    ss->wTimeout = timeout;
    if (ss->opt.fdx) {
        SSL_UNLOCK_WRITER(ss);
    }
    SSL_UNLOCK_READER(ss);
    return SECSuccess;
}

SECStatus
SSL_ResetHandshake(PRFileDesc *s, PRBool asServer)
{
    sslSocket *ss;
    SECStatus status;
    PRNetAddr addr;

    ss = ssl_FindSocket(s);
    if (!ss) {
	SSL_DBG(("%d: SSL[%d]: bad socket in ResetHandshake", SSL_GETPID(), s));
	return SECFailure;
    }

    /* Don't waste my time */
    if (!ss->opt.useSecurity)
	return SECSuccess;

    SSL_LOCK_READER(ss);
    SSL_LOCK_WRITER(ss);

    /* Reset handshake state */
    ssl_Get1stHandshakeLock(ss);
    ssl_GetSSL3HandshakeLock(ss);

    ss->firstHsDone = PR_FALSE;
    if ( asServer ) {
	ss->handshake = ssl2_BeginServerHandshake;
	ss->handshaking = sslHandshakingAsServer;
    } else {
	ss->handshake = ssl2_BeginClientHandshake;
	ss->handshaking = sslHandshakingAsClient;
    }
    ss->nextHandshake       = 0;
    ss->securityHandshake   = 0;

    ssl_GetRecvBufLock(ss);
    status = ssl_InitGather(&ss->gs);
    ssl_ReleaseRecvBufLock(ss);

    /*
    ** Blow away old security state and get a fresh setup.
    */
    ssl_GetXmitBufLock(ss); 
    ssl_ResetSecurityInfo(&ss->sec, PR_TRUE);
    status = ssl_CreateSecurityInfo(ss);
    ssl_ReleaseXmitBufLock(ss); 

    ssl_ReleaseSSL3HandshakeLock(ss);
    ssl_Release1stHandshakeLock(ss);

    if (!ss->TCPconnected)
	ss->TCPconnected = (PR_SUCCESS == ssl_DefGetpeername(ss, &addr));

    SSL_UNLOCK_WRITER(ss);
    SSL_UNLOCK_READER(ss);

    return status;
}

SECStatus
SSL_ReHandshake(PRFileDesc *fd, PRBool flushCache)
{
    sslSocket *ss;
    SECStatus  rv;
    
    ss = ssl_FindSocket(fd);
    if (!ss) {
	SSL_DBG(("%d: SSL[%d]: bad socket in RedoHandshake", SSL_GETPID(), fd));
	return SECFailure;
    }

    if (!ss->opt.useSecurity)
	return SECSuccess;
    
    ssl_Get1stHandshakeLock(ss);

    /* SSL v2 protocol does not support subsequent handshakes. */
    if (ss->version < SSL_LIBRARY_VERSION_3_0) {
	PORT_SetError(SEC_ERROR_INVALID_ARGS);
	rv = SECFailure;
    } else {
	ssl_GetSSL3HandshakeLock(ss);
	rv = ssl3_RedoHandshake(ss, flushCache); /* force full handshake. */
	ssl_ReleaseSSL3HandshakeLock(ss);
    }

    ssl_Release1stHandshakeLock(ss);

    return rv;
}

SSL_IMPORT SECStatus SSL_ReHandshakeWithTimeout(PRFileDesc *fd,
                                                PRBool flushCache,
                                                PRIntervalTime timeout)
{
    if (SECSuccess != ssl_SetTimeout(fd, timeout)) {
        return SECFailure;
    }
    return SSL_ReHandshake(fd, flushCache);
}

SECStatus
SSL_RedoHandshake(PRFileDesc *fd)
{
    return SSL_ReHandshake(fd, PR_TRUE);
}

SECStatus
SSL_HandshakeCallback(PRFileDesc *fd, SSLHandshakeCallback cb,
		      void *client_data)
{
    sslSocket *ss;
    
    ss = ssl_FindSocket(fd);
    if (!ss) {
	SSL_DBG(("%d: SSL[%d]: bad socket in HandshakeCallback",
		 SSL_GETPID(), fd));
	return SECFailure;
    }

    if (!ss->opt.useSecurity) {
	PORT_SetError(SEC_ERROR_INVALID_ARGS);
	return SECFailure;
    }

    ssl_Get1stHandshakeLock(ss);
    ssl_GetSSL3HandshakeLock(ss);

    ss->handshakeCallback     = cb;
    ss->handshakeCallbackData = client_data;

    ssl_ReleaseSSL3HandshakeLock(ss);
    ssl_Release1stHandshakeLock(ss);

    return SECSuccess;
}

SECStatus
SSL_ForceHandshake(PRFileDesc *fd)
{
    sslSocket *ss;
    SECStatus  rv = SECFailure;

    ss = ssl_FindSocket(fd);
    if (!ss) {
	SSL_DBG(("%d: SSL[%d]: bad socket in ForceHandshake",
		 SSL_GETPID(), fd));
	return rv;
    }

    /* Don't waste my time */
    if (!ss->opt.useSecurity) 
    	return SECSuccess;

    ssl_Get1stHandshakeLock(ss);

    if (ss->version >= SSL_LIBRARY_VERSION_3_0) {
	int gatherResult;

    	ssl_GetRecvBufLock(ss);
	gatherResult = ssl3_GatherCompleteHandshake(ss, 0);
	ssl_ReleaseRecvBufLock(ss);
	if (gatherResult > 0) {
	    rv = SECSuccess;
	} else if (gatherResult == 0) {
	    PORT_SetError(PR_END_OF_FILE_ERROR);
	} else if (gatherResult == SECWouldBlock) {
	    PORT_SetError(PR_WOULD_BLOCK_ERROR);
	}
    } else if (!ss->firstHsDone) {
	rv = ssl_Do1stHandshake(ss);
    } else {
	/* tried to force handshake on an SSL 2 socket that has 
	** already completed the handshake. */
    	rv = SECSuccess;	/* just pretend we did it. */
    }

    ssl_Release1stHandshakeLock(ss);

    return rv;
}

SSL_IMPORT SECStatus SSL_ForceHandshakeWithTimeout(PRFileDesc *fd,
                                                   PRIntervalTime timeout)
{
    if (SECSuccess != ssl_SetTimeout(fd, timeout)) {
        return SECFailure;
    }
    return SSL_ForceHandshake(fd);
}


/************************************************************************/

SECStatus
sslBuffer_Grow(sslBuffer *b, unsigned int newLen)
{
    newLen = PR_MAX(newLen, MAX_FRAGMENT_LENGTH + 2048);
    if (newLen > b->space) {
	unsigned char *newBuf;
	if (b->buf) {
	    newBuf = (unsigned char *) PORT_Realloc(b->buf, newLen);
	} else {
	    newBuf = (unsigned char *) PORT_Alloc(newLen);
	}
	if (!newBuf) {
	    return SECFailure;
	}
	SSL_TRC(10, ("%d: SSL: grow buffer from %d to %d",
		     SSL_GETPID(), b->space, newLen));
	b->buf = newBuf;
	b->space = newLen;
    }
    return SECSuccess;
}

SECStatus 
sslBuffer_Append(sslBuffer *b, const void * data, unsigned int len)
{
    unsigned int newLen = b->len + len;
    SECStatus rv;

    rv = sslBuffer_Grow(b, newLen);
    if (rv != SECSuccess)
    	return rv;
    PORT_Memcpy(b->buf + b->len, data, len);
    b->len += len;
    return SECSuccess;
}

SECStatus 
ssl_SaveWriteData(sslSocket *ss, const void *data, unsigned int len)
{
    SECStatus    rv;

    PORT_Assert( ss->opt.noLocks || ssl_HaveXmitBufLock(ss) );
    rv = sslBuffer_Append(&ss->pendingBuf, data, len);
    SSL_TRC(5, ("%d: SSL[%d]: saving %u bytes of data (%u total saved so far)",
		 SSL_GETPID(), ss->fd, len, ss->pendingBuf.len));
    return rv;
}

int 
ssl_SendSavedWriteData(sslSocket *ss)
{
    int rv	= 0;

    PORT_Assert( ss->opt.noLocks || ssl_HaveXmitBufLock(ss) );
    if (ss->pendingBuf.len != 0) {
	SSL_TRC(5, ("%d: SSL[%d]: sending %d bytes of saved data",
		     SSL_GETPID(), ss->fd, ss->pendingBuf.len));
	rv = ssl_DefSend(ss, ss->pendingBuf.buf, ss->pendingBuf.len, 0);
	if (rv < 0) {
	    return rv;
	} 
	ss->pendingBuf.len -= rv;
	if (ss->pendingBuf.len > 0 && rv > 0) {
	    /* UGH !! This shifts the whole buffer down by copying it */
	    PORT_Memmove(ss->pendingBuf.buf, ss->pendingBuf.buf + rv, 
	                 ss->pendingBuf.len);
    	}
    }
    return rv;
}

/************************************************************************/

static int 
DoRecv(sslSocket *ss, unsigned char *out, int len, int flags)
{
    int              rv;
    int              amount;
    int              available;

    ssl_GetRecvBufLock(ss);

    available = ss->gs.writeOffset - ss->gs.readOffset;
    if (available == 0) {
	/* Get some more data */
	if (ss->version >= SSL_LIBRARY_VERSION_3_0) {
	    /* Wait for application data to arrive.  */
	    rv = ssl3_GatherAppDataRecord(ss, 0);
	} else {
	    /* See if we have a complete record */
	    rv = ssl2_GatherRecord(ss, 0);
	}
	if (rv <= 0) {
	    if (rv == 0) {
		/* EOF */
		SSL_TRC(10, ("%d: SSL[%d]: ssl_recv EOF",
			     SSL_GETPID(), ss->fd));
		goto done;
	    }
	    if ((rv != SECWouldBlock) && 
	        (PR_GetError() != PR_WOULD_BLOCK_ERROR)) {
		/* Some random error */
		goto done;
	    }

	    /*
	    ** Gather record is blocked waiting for more record data to
	    ** arrive. Try to process what we have already received
	    */
	} else {
	    /* Gather record has finished getting a complete record */
	}

	/* See if any clear data is now available */
	available = ss->gs.writeOffset - ss->gs.readOffset;
	if (available == 0) {
	    /*
	    ** No partial data is available. Force error code to
	    ** EWOULDBLOCK so that caller will try again later. Note
	    ** that the error code is probably EWOULDBLOCK already,
	    ** but if it isn't (for example, if we received a zero
	    ** length record) then this will force it to be correct.
	    */
	    PORT_SetError(PR_WOULD_BLOCK_ERROR);
	    rv = SECFailure;
	    goto done;
	}
	SSL_TRC(30, ("%d: SSL[%d]: partial data ready, available=%d",
		     SSL_GETPID(), ss->fd, available));
    }

    /* Dole out clear data to reader */
    amount = PR_MIN(len, available);
    PORT_Memcpy(out, ss->gs.buf.buf + ss->gs.readOffset, amount);
    if (!(flags & PR_MSG_PEEK)) {
	ss->gs.readOffset += amount;
    }
    rv = amount;

    SSL_TRC(30, ("%d: SSL[%d]: amount=%d available=%d",
		 SSL_GETPID(), ss->fd, amount, available));
    PRINT_BUF(4, (ss, "DoRecv receiving plaintext:", out, amount));

done:
    ssl_ReleaseRecvBufLock(ss);
    return rv;
}

/************************************************************************/

SSLKEAType
ssl_FindCertKEAType(CERTCertificate * cert)
{
  SSLKEAType keaType = kt_null; 
  int tag;
  
  if (!cert) goto loser;
  
  tag = SECOID_GetAlgorithmTag(&(cert->subjectPublicKeyInfo.algorithm));
  
  switch (tag) {
  case SEC_OID_X500_RSA_ENCRYPTION:
  case SEC_OID_PKCS1_RSA_ENCRYPTION:
    keaType = kt_rsa;
    break;

  case SEC_OID_X942_DIFFIE_HELMAN_KEY:
    keaType = kt_dh;
    break;
#ifdef NSS_ENABLE_ECC
  case SEC_OID_ANSIX962_EC_PUBLIC_KEY:
    keaType = kt_ecdh;
    break;
#endif /* NSS_ENABLE_ECC */
  default:
    keaType = kt_null;
  }
  
 loser:
  
  return keaType;

}

static const PRCallOnceType pristineCallOnce;
static       PRCallOnceType setupServerCAListOnce;

static SECStatus serverCAListShutdown(void* appData, void* nssData)
{
    PORT_Assert(ssl3_server_ca_list);
    if (ssl3_server_ca_list) {
	CERT_FreeDistNames(ssl3_server_ca_list);
	ssl3_server_ca_list = NULL;
    }
    setupServerCAListOnce = pristineCallOnce;
    return SECSuccess;
}

static PRStatus serverCAListSetup(void *arg)
{
    CERTCertDBHandle *dbHandle = (CERTCertDBHandle *)arg;
    SECStatus rv = NSS_RegisterShutdown(serverCAListShutdown, NULL);
    PORT_Assert(SECSuccess == rv);
    if (SECSuccess == rv) {
	ssl3_server_ca_list = CERT_GetSSLCACerts(dbHandle);
	return PR_SUCCESS;
    }
    return PR_FAILURE;
}


/* XXX need to protect the data that gets changed here.!! */

SECStatus
SSL_ConfigSecureServer(PRFileDesc *fd, CERTCertificate *cert,
		       SECKEYPrivateKey *key, SSL3KEAType kea)
{
    SECStatus rv;
    sslSocket *ss;
    sslServerCerts  *sc;
    SECKEYPublicKey * pubKey = NULL;

    ss = ssl_FindSocket(fd);
    if (!ss) {
	return SECFailure;
    }

    /* Both key and cert must have a value or be NULL */
    /* Passing a value of NULL will turn off key exchange algorithms that were
     * previously turned on */
    if (!cert != !key) {
	PORT_SetError(SEC_ERROR_INVALID_ARGS);
	return SECFailure;
    }

    /* make sure the key exchange is recognized */
    if ((kea >= kt_kea_size) || (kea < kt_null)) {
	PORT_SetError(SEC_ERROR_UNSUPPORTED_KEYALG);
	return SECFailure;
    }

    if (kea != ssl_FindCertKEAType(cert)) {
    	PORT_SetError(SSL_ERROR_CERT_KEA_MISMATCH);
	return SECFailure;
    }

    sc = ss->serverCerts + kea;
    /* load the server certificate */
    if (sc->serverCert != NULL) {
	CERT_DestroyCertificate(sc->serverCert);
    	sc->serverCert = NULL;
    }
    if (cert) {
	sc->serverCert = CERT_DupCertificate(cert);
	if (!sc->serverCert)
	    goto loser;
    	/* get the size of the cert's public key, and remember it */
	pubKey = CERT_ExtractPublicKey(cert);
	if (!pubKey) 
	    goto loser;
	sc->serverKeyBits = SECKEY_PublicKeyStrengthInBits(pubKey);
    }


    /* load the server cert chain */
    if (sc->serverCertChain != NULL) {
	CERT_DestroyCertificateList(sc->serverCertChain);
    	sc->serverCertChain = NULL;
    }
    if (cert) {
	sc->serverCertChain = CERT_CertChainFromCert(
			    sc->serverCert, certUsageSSLServer, PR_TRUE);
	if (sc->serverCertChain == NULL)
	     goto loser;
    }

    /* load the private key */
    if (sc->serverKeyPair != NULL) {
        ssl3_FreeKeyPair(sc->serverKeyPair);
        sc->serverKeyPair = NULL;
    }
    if (key) {
	SECKEYPrivateKey * keyCopy	= NULL;
	CK_MECHANISM_TYPE  keyMech	= CKM_INVALID_MECHANISM;

	if (key->pkcs11Slot) {
	    PK11SlotInfo * bestSlot;
	    bestSlot = PK11_ReferenceSlot(key->pkcs11Slot);
	    if (bestSlot) {
		keyCopy = PK11_CopyTokenPrivKeyToSessionPrivKey(bestSlot, key);
		PK11_FreeSlot(bestSlot);
	    }
	}
	if (keyCopy == NULL)
	    keyMech = PK11_MapSignKeyType(key->keyType);
	if (keyMech != CKM_INVALID_MECHANISM) {
	    PK11SlotInfo * bestSlot;
	    /* XXX Maybe should be bestSlotMultiple? */
	    bestSlot = PK11_GetBestSlot(keyMech, NULL /* wincx */);
	    if (bestSlot) {
		keyCopy = PK11_CopyTokenPrivKeyToSessionPrivKey(bestSlot, key);
		PK11_FreeSlot(bestSlot);
	    }
	}
	if (keyCopy == NULL)
	    keyCopy = SECKEY_CopyPrivateKey(key);
	if (keyCopy == NULL)
	    goto loser;
	SECKEY_CacheStaticFlags(keyCopy);
        sc->serverKeyPair = ssl3_NewKeyPair(keyCopy, pubKey);
        if (sc->serverKeyPair == NULL) {
            SECKEY_DestroyPrivateKey(keyCopy);
            goto loser;
        }
	pubKey = NULL; /* adopted by serverKeyPair */
    }

    if (kea == kt_rsa && cert && sc->serverKeyBits > 512) {
	if (ss->opt.noStepDown) {
	    /* disable all export ciphersuites */
	} else {
	    rv = ssl3_CreateRSAStepDownKeys(ss);
	    if (rv != SECSuccess) {
		return SECFailure; /* err set by ssl3_CreateRSAStepDownKeys */
	    }
	}
    }

    /* Only do this once because it's global. */
    if (PR_SUCCESS == PR_CallOnceWithArg(&setupServerCAListOnce, 
                                         &serverCAListSetup,
                                         (void *)(ss->dbHandle))) {
	return SECSuccess;
    }

loser:
    if (pubKey) {
	SECKEY_DestroyPublicKey(pubKey); 
	pubKey = NULL;
    }
    if (sc->serverCert != NULL) {
	CERT_DestroyCertificate(sc->serverCert);
	sc->serverCert = NULL;
    }
    if (sc->serverCertChain != NULL) {
	CERT_DestroyCertificateList(sc->serverCertChain);
	sc->serverCertChain = NULL;
    }
    if (sc->serverKeyPair != NULL) {
	ssl3_FreeKeyPair(sc->serverKeyPair);
	sc->serverKeyPair = NULL;
    }
    return SECFailure;
}

/************************************************************************/

SECStatus
ssl_CreateSecurityInfo(sslSocket *ss)
{
    SECStatus status;

    /* initialize sslv2 socket to send data in the clear. */
    ssl2_UseClearSendFunc(ss);

    ss->sec.blockSize  = 1;
    ss->sec.blockShift = 0;

    ssl_GetXmitBufLock(ss); 
    status = sslBuffer_Grow(&ss->sec.writeBuf, 4096);
    ssl_ReleaseXmitBufLock(ss); 

    return status;
}

SECStatus
ssl_CopySecurityInfo(sslSocket *ss, sslSocket *os)
{
    ss->sec.send 		= os->sec.send;
    ss->sec.isServer 		= os->sec.isServer;
    ss->sec.keyBits    		= os->sec.keyBits;
    ss->sec.secretKeyBits 	= os->sec.secretKeyBits;

    ss->sec.peerCert   		= CERT_DupCertificate(os->sec.peerCert);
    if (os->sec.peerCert && !ss->sec.peerCert)
    	goto loser;

    ss->sec.cache      		= os->sec.cache;
    ss->sec.uncache    		= os->sec.uncache;

    /* we don't dup the connection info. */

    ss->sec.sendSequence 	= os->sec.sendSequence;
    ss->sec.rcvSequence 	= os->sec.rcvSequence;

    if (os->sec.hash && os->sec.hashcx) {
	ss->sec.hash 		= os->sec.hash;
	ss->sec.hashcx 		= os->sec.hash->clone(os->sec.hashcx);
	if (os->sec.hashcx && !ss->sec.hashcx)
	    goto loser;
    } else {
	ss->sec.hash 		= NULL;
	ss->sec.hashcx 		= NULL;
    }

    SECITEM_CopyItem(0, &ss->sec.sendSecret, &os->sec.sendSecret);
    if (os->sec.sendSecret.data && !ss->sec.sendSecret.data)
    	goto loser;
    SECITEM_CopyItem(0, &ss->sec.rcvSecret,  &os->sec.rcvSecret);
    if (os->sec.rcvSecret.data && !ss->sec.rcvSecret.data)
    	goto loser;

    /* XXX following code is wrong if either cx != 0 */
    PORT_Assert(os->sec.readcx  == 0);
    PORT_Assert(os->sec.writecx == 0);
    ss->sec.readcx     		= os->sec.readcx;
    ss->sec.writecx    		= os->sec.writecx;
    ss->sec.destroy    		= 0;	

    ss->sec.enc        		= os->sec.enc;
    ss->sec.dec        		= os->sec.dec;

    ss->sec.blockShift 		= os->sec.blockShift;
    ss->sec.blockSize  		= os->sec.blockSize;

    return SECSuccess;

loser:
    return SECFailure;
}

void 
ssl_ResetSecurityInfo(sslSecurityInfo *sec, PRBool doMemset)
{
    /* Destroy MAC */
    if (sec->hash && sec->hashcx) {
	(*sec->hash->destroy)(sec->hashcx, PR_TRUE);
	sec->hashcx = NULL;
	sec->hash = NULL;
    }
    SECITEM_ZfreeItem(&sec->sendSecret, PR_FALSE);
    SECITEM_ZfreeItem(&sec->rcvSecret, PR_FALSE);

    /* Destroy ciphers */
    if (sec->destroy) {
	(*sec->destroy)(sec->readcx, PR_TRUE);
	(*sec->destroy)(sec->writecx, PR_TRUE);
	sec->readcx = NULL;
	sec->writecx = NULL;
    } else {
	PORT_Assert(sec->readcx == 0);
	PORT_Assert(sec->writecx == 0);
    }
    sec->readcx = 0;
    sec->writecx = 0;

    if (sec->localCert) {
	CERT_DestroyCertificate(sec->localCert);
	sec->localCert = NULL;
    }
    if (sec->peerCert) {
	CERT_DestroyCertificate(sec->peerCert);
	sec->peerCert = NULL;
    }
    if (sec->peerKey) {
	SECKEY_DestroyPublicKey(sec->peerKey);
	sec->peerKey = NULL;
    }

    /* cleanup the ci */
    if (sec->ci.sid != NULL) {
	ssl_FreeSID(sec->ci.sid);
    }
    PORT_ZFree(sec->ci.sendBuf.buf, sec->ci.sendBuf.space);
    if (doMemset) {
        memset(&sec->ci, 0, sizeof sec->ci);
    }
    
}

void 
ssl_DestroySecurityInfo(sslSecurityInfo *sec)
{
    ssl_ResetSecurityInfo(sec, PR_FALSE);

    PORT_ZFree(sec->writeBuf.buf, sec->writeBuf.space);
    sec->writeBuf.buf = 0;

    memset(sec, 0, sizeof *sec);
}

/************************************************************************/

int 
ssl_SecureConnect(sslSocket *ss, const PRNetAddr *sa)
{
    PRFileDesc *osfd = ss->fd->lower;
    int rv;

    if ( ss->opt.handshakeAsServer ) {
	ss->securityHandshake = ssl2_BeginServerHandshake;
	ss->handshaking = sslHandshakingAsServer;
    } else {
	ss->securityHandshake = ssl2_BeginClientHandshake;
	ss->handshaking = sslHandshakingAsClient;
    }

    /* connect to server */
    rv = osfd->methods->connect(osfd, sa, ss->cTimeout);
    if (rv == PR_SUCCESS) {
	ss->TCPconnected = 1;
    } else {
	int err = PR_GetError();
	SSL_DBG(("%d: SSL[%d]: connect failed, errno=%d",
		 SSL_GETPID(), ss->fd, err));
	if (err == PR_IS_CONNECTED_ERROR) {
	    ss->TCPconnected = 1;
	} 
    }

    SSL_TRC(5, ("%d: SSL[%d]: secure connect completed, rv == %d",
		SSL_GETPID(), ss->fd, rv));
    return rv;
}


int
ssl_SecureClose(sslSocket *ss)
{
    int rv;

    if (ss->version >= SSL_LIBRARY_VERSION_3_0 	&&
	!(ss->shutdownHow & ssl_SHUTDOWN_SEND)	&&
    	ss->firstHsDone 			&& 
	!ss->recvdCloseNotify                   &&
	ss->ssl3.initialized) {

	/* We don't want the final alert to be Nagle delayed. */
	if (!ss->delayDisabled) {
	    ssl_EnableNagleDelay(ss, PR_FALSE);
	    ss->delayDisabled = 1;
	}

	(void) SSL3_SendAlert(ss, alert_warning, close_notify);
    }
    rv = ssl_DefClose(ss);
    return rv;
}

/* Caller handles all locking */
int
ssl_SecureShutdown(sslSocket *ss, int nsprHow)
{
    PRFileDesc *osfd = ss->fd->lower;
    int 	rv;
    PRIntn	sslHow	= nsprHow + 1;

    if ((unsigned)nsprHow > PR_SHUTDOWN_BOTH) {
	PORT_SetError(PR_INVALID_ARGUMENT_ERROR);
    	return PR_FAILURE;
    }

    if ((sslHow & ssl_SHUTDOWN_SEND) != 0 		&&
    	ss->version >= SSL_LIBRARY_VERSION_3_0		&&
	!(ss->shutdownHow & ssl_SHUTDOWN_SEND)		&&
	ss->firstHsDone 				&& 
	!ss->recvdCloseNotify                   	&&
	ss->ssl3.initialized) {

	(void) SSL3_SendAlert(ss, alert_warning, close_notify);
    }

    rv = osfd->methods->shutdown(osfd, nsprHow);

    ss->shutdownHow |= sslHow;

    return rv;
}

/************************************************************************/


int
ssl_SecureRecv(sslSocket *ss, unsigned char *buf, int len, int flags)
{
    sslSecurityInfo *sec;
    int              rv   = 0;

    sec = &ss->sec;

    if (ss->shutdownHow & ssl_SHUTDOWN_RCV) {
	PORT_SetError(PR_SOCKET_SHUTDOWN_ERROR);
    	return PR_FAILURE;
    }
    if (flags & ~PR_MSG_PEEK) {
	PORT_SetError(PR_INVALID_ARGUMENT_ERROR);
    	return PR_FAILURE;
    }

    if (!ssl_SocketIsBlocking(ss) && !ss->opt.fdx) {
	ssl_GetXmitBufLock(ss);
	if (ss->pendingBuf.len != 0) {
	    rv = ssl_SendSavedWriteData(ss);
	    if ((rv < 0) && (PORT_GetError() != PR_WOULD_BLOCK_ERROR)) {
		ssl_ReleaseXmitBufLock(ss);
		return SECFailure;
	    }
	    /* XXX short write? */
	}
	ssl_ReleaseXmitBufLock(ss);
    }
    
    rv = 0;
    /* If any of these is non-zero, the initial handshake is not done. */
    if (!ss->firstHsDone) {
	ssl_Get1stHandshakeLock(ss);
	if (ss->handshake || ss->nextHandshake || ss->securityHandshake) {
	    rv = ssl_Do1stHandshake(ss);
	}
	ssl_Release1stHandshakeLock(ss);
    }
    if (rv < 0) {
	return rv;
    }

    if (len == 0) return 0;

    rv = DoRecv(ss, (unsigned char*) buf, len, flags);
    SSL_TRC(2, ("%d: SSL[%d]: recving %d bytes securely (errno=%d)",
		SSL_GETPID(), ss->fd, rv, PORT_GetError()));
    return rv;
}

int
ssl_SecureRead(sslSocket *ss, unsigned char *buf, int len)
{
    return ssl_SecureRecv(ss, buf, len, 0);
}

/* Caller holds the SSL Socket's write lock. SSL_LOCK_WRITER(ss) */
int
ssl_SecureSend(sslSocket *ss, const unsigned char *buf, int len, int flags)
{
    int              rv		= 0;

    SSL_TRC(2, ("%d: SSL[%d]: SecureSend: sending %d bytes",
		SSL_GETPID(), ss->fd, len));

    if (ss->shutdownHow & ssl_SHUTDOWN_SEND) {
	PORT_SetError(PR_SOCKET_SHUTDOWN_ERROR);
    	rv = PR_FAILURE;
	goto done;
    }
    if (flags) {
	PORT_SetError(PR_INVALID_ARGUMENT_ERROR);
    	rv = PR_FAILURE;
	goto done;
    }

    ssl_GetXmitBufLock(ss);
    if (ss->pendingBuf.len != 0) {
	PORT_Assert(ss->pendingBuf.len > 0);
	rv = ssl_SendSavedWriteData(ss);
	if (rv >= 0 && ss->pendingBuf.len != 0) {
	    PORT_Assert(ss->pendingBuf.len > 0);
	    PORT_SetError(PR_WOULD_BLOCK_ERROR);
	    rv = SECFailure;
	}
    }
    ssl_ReleaseXmitBufLock(ss);
    if (rv < 0) {
	goto done;
    }

    if (len > 0) 
    	ss->writerThread = PR_GetCurrentThread();
    /* If any of these is non-zero, the initial handshake is not done. */
    if (!ss->firstHsDone) {
	ssl_Get1stHandshakeLock(ss);
	if (ss->handshake || ss->nextHandshake || ss->securityHandshake) {
	    rv = ssl_Do1stHandshake(ss);
	}
	ssl_Release1stHandshakeLock(ss);
    }
    if (rv < 0) {
    	ss->writerThread = NULL;
	goto done;
    }

    /* Check for zero length writes after we do housekeeping so we make forward
     * progress.
     */
    if (len == 0) {
    	rv = 0;
	goto done;
    }
    PORT_Assert(buf != NULL);
    if (!buf) {
	PORT_SetError(PR_INVALID_ARGUMENT_ERROR);
    	rv = PR_FAILURE;
	goto done;
    }

    /* Send out the data using one of these functions:
     *	ssl2_SendClear, ssl2_SendStream, ssl2_SendBlock, 
     *  ssl3_SendApplicationData
     */
    ssl_GetXmitBufLock(ss);
    rv = (*ss->sec.send)(ss, buf, len, flags);
    ssl_ReleaseXmitBufLock(ss);
    ss->writerThread = NULL;
done:
    if (rv < 0) {
	SSL_TRC(2, ("%d: SSL[%d]: SecureSend: returning %d count, error %d",
		    SSL_GETPID(), ss->fd, rv, PORT_GetError()));
    } else {
	SSL_TRC(2, ("%d: SSL[%d]: SecureSend: returning %d count",
		    SSL_GETPID(), ss->fd, rv));
    }
    return rv;
}

int
ssl_SecureWrite(sslSocket *ss, const unsigned char *buf, int len)
{
    return ssl_SecureSend(ss, buf, len, 0);
}

SECStatus
SSL_BadCertHook(PRFileDesc *fd, SSLBadCertHandler f, void *arg)
{
    sslSocket *ss;
    
    ss = ssl_FindSocket(fd);
    if (!ss) {
	SSL_DBG(("%d: SSL[%d]: bad socket in SSLBadCertHook",
		 SSL_GETPID(), fd));
	return SECFailure;
    }

    ss->handleBadCert = f;
    ss->badCertArg = arg;

    return SECSuccess;
}

SECStatus
SSL_SetURL(PRFileDesc *fd, const char *url)
{
    sslSocket *   ss = ssl_FindSocket(fd);
    SECStatus     rv = SECSuccess;

    if (!ss) {
	SSL_DBG(("%d: SSL[%d]: bad socket in SSLSetURL",
		 SSL_GETPID(), fd));
	return SECFailure;
    }
    ssl_Get1stHandshakeLock(ss);
    ssl_GetSSL3HandshakeLock(ss);

    if ( ss->url ) {
	PORT_Free((void *)ss->url);	/* CONST */
    }

    ss->url = (const char *)PORT_Strdup(url);
    if ( ss->url == NULL ) {
	rv = SECFailure;
    }

    ssl_ReleaseSSL3HandshakeLock(ss);
    ssl_Release1stHandshakeLock(ss);

    return rv;
}

int
SSL_DataPending(PRFileDesc *fd)
{
    sslSocket *ss;
    int        rv  = 0;

    ss = ssl_FindSocket(fd);

    if (ss && ss->opt.useSecurity) {

	ssl_Get1stHandshakeLock(ss);
	ssl_GetSSL3HandshakeLock(ss);

	ssl_GetRecvBufLock(ss);
	rv = ss->gs.writeOffset - ss->gs.readOffset;
	ssl_ReleaseRecvBufLock(ss);

	ssl_ReleaseSSL3HandshakeLock(ss);
	ssl_Release1stHandshakeLock(ss);
    }

    return rv;
}

SECStatus
SSL_InvalidateSession(PRFileDesc *fd)
{
    sslSocket *   ss = ssl_FindSocket(fd);
    SECStatus     rv = SECFailure;

    if (ss) {
	ssl_Get1stHandshakeLock(ss);
	ssl_GetSSL3HandshakeLock(ss);

	if (ss->sec.ci.sid) {
	    ss->sec.uncache(ss->sec.ci.sid);
	    rv = SECSuccess;
	}

	ssl_ReleaseSSL3HandshakeLock(ss);
	ssl_Release1stHandshakeLock(ss);
    }
    return rv;
}

SECItem *
SSL_GetSessionID(PRFileDesc *fd)
{
    sslSocket *    ss;
    SECItem *      item = NULL;

    ss = ssl_FindSocket(fd);
    if (ss) {
	ssl_Get1stHandshakeLock(ss);
	ssl_GetSSL3HandshakeLock(ss);

	if (ss->opt.useSecurity && ss->firstHsDone && ss->sec.ci.sid) {
	    item = (SECItem *)PORT_Alloc(sizeof(SECItem));
	    if (item) {
		sslSessionID * sid = ss->sec.ci.sid;
		if (sid->version < SSL_LIBRARY_VERSION_3_0) {
		    item->len = SSL2_SESSIONID_BYTES;
		    item->data = (unsigned char*)PORT_Alloc(item->len);
		    PORT_Memcpy(item->data, sid->u.ssl2.sessionID, item->len);
		} else {
		    item->len = sid->u.ssl3.sessionIDLength;
		    item->data = (unsigned char*)PORT_Alloc(item->len);
		    PORT_Memcpy(item->data, sid->u.ssl3.sessionID, item->len);
		}
	    }
	}

	ssl_ReleaseSSL3HandshakeLock(ss);
	ssl_Release1stHandshakeLock(ss);
    }
    return item;
}

SECStatus
SSL_CertDBHandleSet(PRFileDesc *fd, CERTCertDBHandle *dbHandle)
{
    sslSocket *    ss;

    ss = ssl_FindSocket(fd);
    if (!ss)
    	return SECFailure;
    if (!dbHandle) {
    	PORT_SetError(SEC_ERROR_INVALID_ARGS);
	return SECFailure;
    }
    ss->dbHandle = dbHandle;
    return SECSuccess;
}

int
SSL_RestartHandshakeAfterCertReq(sslSocket *         ss,
				CERTCertificate *    cert, 
				SECKEYPrivateKey *   key,
				CERTCertificateList *certChain)
{
    int              ret;

    ssl_Get1stHandshakeLock(ss);   /************************************/

    if (ss->version >= SSL_LIBRARY_VERSION_3_0) {
	ret = ssl3_RestartHandshakeAfterCertReq(ss, cert, key, certChain);
    } else {
    	ret = ssl2_RestartHandshakeAfterCertReq(ss, cert, key);
    }

    ssl_Release1stHandshakeLock(ss);  /************************************/
    return ret;
}


int
SSL_RestartHandshakeAfterServerCert(sslSocket *ss)
{
    int rv	= SECSuccess;

    ssl_Get1stHandshakeLock(ss); 

    if (ss->version >= SSL_LIBRARY_VERSION_3_0) {
	rv = ssl3_RestartHandshakeAfterServerCert(ss);
    } else {
	rv = ssl2_RestartHandshakeAfterServerCert(ss);
    }

    ssl_Release1stHandshakeLock(ss);
    return rv;
}
