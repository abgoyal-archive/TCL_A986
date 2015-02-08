/* $Id: sslerr.c,v 1.4 2004/04/27 23:04:39 gerv%gerv.net Exp $ */

#include "prerror.h"
#include "secerr.h"
#include "sslerr.h"
#include "seccomon.h"

int
ssl_MapLowLevelError(int hiLevelError)
{
    int oldErr	= PORT_GetError();

    switch (oldErr) {

    case 0:
    case PR_IO_ERROR:
    case SEC_ERROR_IO:
    case SEC_ERROR_BAD_DATA:
    case SEC_ERROR_LIBRARY_FAILURE:
    case SEC_ERROR_EXTENSION_NOT_FOUND:
    case SSL_ERROR_BAD_CLIENT:
    case SSL_ERROR_BAD_SERVER:
    case SSL_ERROR_SESSION_NOT_FOUND:
    	PORT_SetError(hiLevelError);
	return hiLevelError;

    default:	/* leave the majority of error codes alone. */
	return oldErr;
    }
}
