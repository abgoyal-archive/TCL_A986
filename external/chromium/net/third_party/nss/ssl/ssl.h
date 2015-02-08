/* $Id: ssl.h,v 1.31 2009/11/25 05:24:25 wtc%google.com Exp $ */

#ifndef __ssl_h_
#define __ssl_h_

#include "prtypes.h"
#include "prerror.h"
#include "prio.h"
#include "seccomon.h"
#include "cert.h"
#include "keyt.h"

#include "sslt.h"  /* public ssl data types */

#if defined(_WIN32) && !defined(IN_LIBSSL) && !defined(NSS_USE_STATIC_LIBS)
#define SSL_IMPORT extern __declspec(dllimport)
#else
#define SSL_IMPORT extern
#endif

SEC_BEGIN_PROTOS

/* constant table enumerating all implemented SSL 2 and 3 cipher suites. */
SSL_IMPORT const PRUint16 SSL_ImplementedCiphers[];

/* number of entries in the above table. */
SSL_IMPORT const PRUint16 SSL_NumImplementedCiphers;

/* Macro to tell which ciphers in table are SSL2 vs SSL3/TLS. */
#define SSL_IS_SSL2_CIPHER(which) (((which) & 0xfff0) == 0xff00)

SSL_IMPORT PRFileDesc *SSL_ImportFD(PRFileDesc *model, PRFileDesc *fd);

/* options */
#define SSL_SECURITY			1 /* (on by default) */
#define SSL_SOCKS			2 /* (off by default) */
#define SSL_REQUEST_CERTIFICATE		3 /* (off by default) */
#define SSL_HANDSHAKE_AS_CLIENT		5 /* force accept to hs as client */
                               		  /* (off by default) */
#define SSL_HANDSHAKE_AS_SERVER		6 /* force connect to hs as server */
                               		  /* (off by default) */
#define SSL_ENABLE_SSL2			7 /* enable ssl v2 (on by default) */
#define SSL_ENABLE_SSL3		        8 /* enable ssl v3 (on by default) */
#define SSL_NO_CACHE		        9 /* don't use the session cache */
                    		          /* (off by default) */
#define SSL_REQUIRE_CERTIFICATE        10 /* (SSL_REQUIRE_FIRST_HANDSHAKE */
                                          /* by default) */
#define SSL_ENABLE_FDX                 11 /* permit simultaneous read/write */
                                          /* (off by default) */
#define SSL_V2_COMPATIBLE_HELLO        12 /* send v3 client hello in v2 fmt */
                                          /* (on by default) */
#define SSL_ENABLE_TLS		       13 /* enable TLS (on by default) */
#define SSL_ROLLBACK_DETECTION         14 /* for compatibility, default: on */
#define SSL_NO_STEP_DOWN               15 /* Disable export cipher suites   */
                                          /* if step-down keys are needed.  */
					  /* default: off, generate         */
					  /* step-down keys if needed.      */
#define SSL_BYPASS_PKCS11              16 /* use PKCS#11 for pub key only   */
#define SSL_NO_LOCKS                   17 /* Don't use locks for protection */
#define SSL_ENABLE_SESSION_TICKETS     18 /* Enable TLS SessionTicket       */
                                          /* extension (off by default)     */
#define SSL_ENABLE_DEFLATE             19 /* Enable TLS compression with    */
                                          /* DEFLATE (off by default)       */
#define SSL_ENABLE_RENEGOTIATION       20 /* Values below (default: never)  */
#define SSL_REQUIRE_SAFE_NEGOTIATION   21 /* Peer must use renegotiation    */
                                          /* extension in ALL handshakes.   */
                                          /* default: off                   */
					  /* NOT YET IMPLEMENTED in 3.12.5  */

#ifdef SSL_DEPRECATED_FUNCTION 
/* Old deprecated function names */
SSL_IMPORT SECStatus SSL_Enable(PRFileDesc *fd, int option, PRBool on);
SSL_IMPORT SECStatus SSL_EnableDefault(int option, PRBool on);
#endif

/* New function names */
SSL_IMPORT SECStatus SSL_OptionSet(PRFileDesc *fd, PRInt32 option, PRBool on);
SSL_IMPORT SECStatus SSL_OptionGet(PRFileDesc *fd, PRInt32 option, PRBool *on);
SSL_IMPORT SECStatus SSL_OptionSetDefault(PRInt32 option, PRBool on);
SSL_IMPORT SECStatus SSL_OptionGetDefault(PRInt32 option, PRBool *on);
SSL_IMPORT SECStatus SSL_CertDBHandleSet(PRFileDesc *fd, CERTCertDBHandle *dbHandle);

SSL_IMPORT SECStatus SSL_SetNextProtoNego(PRFileDesc *fd,
					  const unsigned char *data,
					  unsigned short length);
SSL_IMPORT SECStatus SSL_GetNextProto(PRFileDesc *fd,
				      int *state,
				      unsigned char *buf,
				      unsigned *length,
				      unsigned buf_len);
#define SSL_NEXT_PROTO_NO_SUPPORT	0 /* No peer support                */
#define SSL_NEXT_PROTO_NEGOTIATED	1 /* Mutual agreement               */
#define SSL_NEXT_PROTO_NO_OVERLAP	2 /* No protocol overlap found      */

#ifdef SSL_DEPRECATED_FUNCTION 
/* Old deprecated function names */
SSL_IMPORT SECStatus SSL_EnableCipher(long which, PRBool enabled);
SSL_IMPORT SECStatus SSL_SetPolicy(long which, int policy);
#endif

/* New function names */
SSL_IMPORT SECStatus SSL_CipherPrefSet(PRFileDesc *fd, PRInt32 cipher, PRBool enabled);
SSL_IMPORT SECStatus SSL_CipherPrefGet(PRFileDesc *fd, PRInt32 cipher, PRBool *enabled);
SSL_IMPORT SECStatus SSL_CipherPrefSetDefault(PRInt32 cipher, PRBool enabled);
SSL_IMPORT SECStatus SSL_CipherPrefGetDefault(PRInt32 cipher, PRBool *enabled);
SSL_IMPORT SECStatus SSL_CipherPolicySet(PRInt32 cipher, PRInt32 policy);
SSL_IMPORT SECStatus SSL_CipherPolicyGet(PRInt32 cipher, PRInt32 *policy);

/* Values for "policy" argument to SSL_PolicySet */
/* Values returned by SSL_CipherPolicyGet. */
#define SSL_NOT_ALLOWED		 0	      /* or invalid or unimplemented */
#define SSL_ALLOWED		 1
#define SSL_RESTRICTED		 2	      /* only with "Step-Up" certs. */

/* Values for "on" with SSL_REQUIRE_CERTIFICATE. */
#define SSL_REQUIRE_NEVER           ((PRBool)0)
#define SSL_REQUIRE_ALWAYS          ((PRBool)1)
#define SSL_REQUIRE_FIRST_HANDSHAKE ((PRBool)2)
#define SSL_REQUIRE_NO_ERROR        ((PRBool)3)

/* Values for "on" with SSL_ENABLE_RENEGOTIATION */
/* Never renegotiate at all.                                               */
#define SSL_RENEGOTIATE_NEVER        ((PRBool)0)
/* Renegotiate without restriction, whether or not the peer's client hello */
/* bears the renegotiation info extension (like we always did in the past).*/
#define SSL_RENEGOTIATE_UNRESTRICTED ((PRBool)1)
/*  Only renegotiate if the peer's hello bears the TLS renegotiation_info  */
/*  extension.  Cannot renegotiate in SSL 3.0 sessions.                    */
#define SSL_RENEGOTIATE_REQUIRES_XTN ((PRBool)2) /*  (NOT YET IMPLEMENTED) */

SSL_IMPORT SECStatus SSL_ResetHandshake(PRFileDesc *fd, PRBool asServer);

SSL_IMPORT SECStatus SSL_ForceHandshake(PRFileDesc *fd);

SSL_IMPORT SECStatus SSL_ForceHandshakeWithTimeout(PRFileDesc *fd,
                                                   PRIntervalTime timeout);

SSL_IMPORT SECStatus SSL_SecurityStatus(PRFileDesc *fd, int *on, char **cipher,
			                int *keySize, int *secretKeySize,
			                char **issuer, char **subject);

/* Values for "on" */
#define SSL_SECURITY_STATUS_NOOPT	-1
#define SSL_SECURITY_STATUS_OFF		0
#define SSL_SECURITY_STATUS_ON_HIGH	1
#define SSL_SECURITY_STATUS_ON_LOW	2
#define SSL_SECURITY_STATUS_FORTEZZA	3 /* NO LONGER SUPPORTED */

SSL_IMPORT CERTCertificate *SSL_PeerCertificate(PRFileDesc *fd);

typedef SECStatus (PR_CALLBACK *SSLAuthCertificate)(void *arg, PRFileDesc *fd, 
                                                    PRBool checkSig,
                                                    PRBool isServer);

SSL_IMPORT SECStatus SSL_AuthCertificateHook(PRFileDesc *fd, 
					     SSLAuthCertificate f,
				             void *arg);

/* An implementation of the certificate authentication hook */
SSL_IMPORT SECStatus SSL_AuthCertificate(void *arg, PRFileDesc *fd, 
					 PRBool checkSig, PRBool isServer);

typedef SECStatus (PR_CALLBACK *SSLGetClientAuthData)(void *arg,
                                PRFileDesc *fd,
                                CERTDistNames *caNames,
                                CERTCertificate **pRetCert,/*return */
                                SECKEYPrivateKey **pRetKey);/* return */

SSL_IMPORT SECStatus SSL_GetClientAuthDataHook(PRFileDesc *fd, 
			                       SSLGetClientAuthData f, void *a);


SSL_IMPORT SECStatus SSL_SetPKCS11PinArg(PRFileDesc *fd, void *a);

typedef SECStatus (PR_CALLBACK *SSLBadCertHandler)(void *arg, PRFileDesc *fd);
SSL_IMPORT SECStatus SSL_BadCertHook(PRFileDesc *fd, SSLBadCertHandler f, 
				     void *arg);

SSL_IMPORT SECStatus SSL_ConfigSecureServer(
				PRFileDesc *fd, CERTCertificate *cert,
				SECKEYPrivateKey *key, SSLKEAType kea);

SSL_IMPORT SECStatus SSL_ConfigServerSessionIDCache(int      maxCacheEntries,
					            PRUint32 timeout,
					            PRUint32 ssl3_timeout,
				              const char *   directory);
SSL_IMPORT SECStatus SSL_ConfigMPServerSIDCache(int      maxCacheEntries, 
				                PRUint32 timeout,
			       	                PRUint32 ssl3_timeout, 
		                          const char *   directory);

SSL_IMPORT PRUint32  SSL_GetMaxServerCacheLocks(void);
SSL_IMPORT SECStatus SSL_SetMaxServerCacheLocks(PRUint32 maxLocks);

#define SSL_ENV_VAR_NAME            "SSL_INHERITANCE"

SSL_IMPORT SECStatus SSL_InheritMPServerSIDCache(const char * envString);

typedef void (PR_CALLBACK *SSLHandshakeCallback)(PRFileDesc *fd,
                                                 void *client_data);
SSL_IMPORT SECStatus SSL_HandshakeCallback(PRFileDesc *fd, 
			          SSLHandshakeCallback cb, void *client_data);

SSL_IMPORT SECStatus SSL_ReHandshake(PRFileDesc *fd, PRBool flushCache);

SSL_IMPORT SECStatus SSL_ReHandshakeWithTimeout(PRFileDesc *fd,
                                                PRBool flushCache,
                                                PRIntervalTime timeout);


#ifdef SSL_DEPRECATED_FUNCTION 
SSL_IMPORT SECStatus SSL_RedoHandshake(PRFileDesc *fd);
#endif

SSL_IMPORT SECStatus SSL_SetURL(PRFileDesc *fd, const char *url);

SSL_IMPORT int SSL_DataPending(PRFileDesc *fd);

SSL_IMPORT SECStatus SSL_InvalidateSession(PRFileDesc *fd);

SSL_IMPORT SECItem *SSL_GetSessionID(PRFileDesc *fd);

SSL_IMPORT void SSL_ClearSessionCache(void);

SSL_IMPORT SECStatus SSL_ShutdownServerSessionIDCache(void);

SSL_IMPORT SECStatus SSL_SetSockPeerID(PRFileDesc *fd, const char *peerID);

SSL_IMPORT CERTCertificate * SSL_RevealCert(PRFileDesc * socket);
SSL_IMPORT void * SSL_RevealPinArg(PRFileDesc * socket);
SSL_IMPORT char * SSL_RevealURL(PRFileDesc * socket);


SSL_IMPORT SECStatus
NSS_GetClientAuthData(void *                       arg,
                      PRFileDesc *                 socket,
                      struct CERTDistNamesStr *    caNames,
                      struct CERTCertificateStr ** pRetCert,
                      struct SECKEYPrivateKeyStr **pRetKey);

SSL_IMPORT SECStatus NSS_CmpCertChainWCANames(CERTCertificate *cert, 
                                          CERTDistNames *caNames);

SSL_IMPORT SSLKEAType NSS_FindCertKEAType(CERTCertificate * cert);

SSL_IMPORT SECStatus NSS_SetDomesticPolicy(void);

SSL_IMPORT SECStatus NSS_SetExportPolicy(void);

SSL_IMPORT SECStatus NSS_SetFrancePolicy(void);

SSL_IMPORT SSL3Statistics * SSL_GetStatistics(void);

SSL_IMPORT SECStatus SSL_GetChannelInfo(PRFileDesc *fd, SSLChannelInfo *info,
                                        PRUintn len);
SSL_IMPORT SECStatus SSL_GetCipherSuiteInfo(PRUint16 cipherSuite, 
                                        SSLCipherSuiteInfo *info, PRUintn len);

SSL_IMPORT CERTCertificate * SSL_LocalCertificate(PRFileDesc *fd);


/* protocol mask bits */
#define SSL_CBP_SSL3	0x0001	        /* test SSL v3 mechanisms */
#define SSL_CBP_TLS1_0	0x0002		/* test TLS v1.0 mechanisms */

SSL_IMPORT SECStatus SSL_CanBypass(CERTCertificate *cert,
                                   SECKEYPrivateKey *privKey,
				   PRUint32 protocolmask,
				   PRUint16 *ciphers, int nciphers,
                                   PRBool *pcanbypass, void *pwArg);

SEC_END_PROTOS

#endif /* __ssl_h_ */
