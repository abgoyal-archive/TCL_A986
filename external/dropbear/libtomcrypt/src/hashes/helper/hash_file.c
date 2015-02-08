#include "tomcrypt.h"


int hash_file(int hash, const char *fname, unsigned char *out, unsigned long *outlen)
{
#ifdef LTC_NO_FILE
    return CRYPT_NOP;
#else
    FILE *in;
    int err;
    LTC_ARGCHK(fname  != NULL);
    LTC_ARGCHK(out    != NULL);
    LTC_ARGCHK(outlen != NULL);

    if ((err = hash_is_valid(hash)) != CRYPT_OK) {
        return err;
    }

    in = fopen(fname, "rb");
    if (in == NULL) { 
       return CRYPT_FILE_NOTFOUND;
    }

    err = hash_filehandle(hash, in, out, outlen);
    if (fclose(in) != 0) {
       return CRYPT_ERROR;
    }

    return err;
#endif
}


/* $Source: /cvs/libtom/libtomcrypt/src/hashes/helper/hash_file.c,v $ */
/* $Revision: 1.4 $ */
/* $Date: 2006/03/31 14:15:35 $ */
