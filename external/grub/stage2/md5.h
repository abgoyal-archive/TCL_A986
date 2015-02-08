/* md5.h - an implementation of the MD5 algorithm and MD5 crypt */

extern int md5_password (const char *key, char *crypted, int check);

/* For convenience.  */
#define check_md5_password(key,crypted)	md5_password((key), (crypted), 1)
#define make_md5_password(key,crypted)	md5_password((key), (crypted), 0)
