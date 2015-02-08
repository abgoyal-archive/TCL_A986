

#include <openssl/evp.h>

const EVP_MD *sha1_md;
const EVP_MD *md4_md;
const EVP_MD *md5_md;

void    openssl_hash_init() {
    /* Use the SHA1 functions in openssl to save the flash space.*/
    OpenSSL_add_all_digests();
    sha1_md = EVP_get_digestbyname("sha1");
    if (!sha1_md) {
        dbglog("Error Unknown message digest SHA1\n");
        exit(1);
    }
    md4_md = EVP_get_digestbyname("md4");
    if (!md4_md) {
        dbglog("Error Unknown message digest MD4\n");
        exit(1);
    }
    md5_md = EVP_get_digestbyname("md5");
    if (!md5_md) {
        dbglog("Error Unknown message digest MD5\n");
        exit(1);
    }
}
