/* $Id: sslreveal.c,v 1.4 2004/04/27 23:04:39 gerv%gerv.net Exp $ */

#include "cert.h"
#include "ssl.h"
#include "certt.h"
#include "sslimpl.h"

CERTCertificate * 
SSL_RevealCert(PRFileDesc * fd)
{
  CERTCertificate * cert = NULL;
  sslSocket * sslsocket = NULL;

  sslsocket = ssl_FindSocket(fd);
  
  /* CERT_DupCertificate increases reference count and returns pointer to 
   * the same cert
   */
  if (sslsocket && sslsocket->sec.peerCert)
    cert = CERT_DupCertificate(sslsocket->sec.peerCert);
  
  return cert;
}

void * 
SSL_RevealPinArg(PRFileDesc * fd)
{
  sslSocket * sslsocket = NULL;
  void * PinArg = NULL;
  
  sslsocket = ssl_FindSocket(fd);
  
  /* is pkcs11PinArg part of the sslSocket or sslSecurityInfo ? */
  if (sslsocket)
    PinArg = sslsocket->pkcs11PinArg;
  
  return PinArg;
}


char * 
SSL_RevealURL(PRFileDesc * fd)
{
  sslSocket * sslsocket = NULL;
  char * url = NULL;

  sslsocket = ssl_FindSocket(fd);
  
  if (sslsocket && sslsocket->url)
    url = PL_strdup(sslsocket->url);
  
  return url;
}

