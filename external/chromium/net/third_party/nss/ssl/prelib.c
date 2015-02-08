/* -*- Mode: C; tab-width: 4; indent-tabs-mode: nil -*- */

/* $Id: prelib.c,v 1.7 2005/08/16 03:42:26 nelsonb%netscape.com Exp $ */

#include "cert.h"
#include "ssl.h"
#include "keyhi.h"
#include "secitem.h"
#include "sslimpl.h"
#include "pkcs11t.h"
#include "preenc.h"
#include "pk11func.h"

PEHeader *SSL_PreencryptedStreamToFile(PRFileDesc *fd, PEHeader *inHeader, 
				       int *headerSize)
{
    PORT_SetError(PR_NOT_IMPLEMENTED_ERROR);
    return NULL;
}

PEHeader *SSL_PreencryptedFileToStream(PRFileDesc *fd, PEHeader *header, 
							int *headerSize)
{
    PORT_SetError(PR_NOT_IMPLEMENTED_ERROR);
    return NULL;
}


