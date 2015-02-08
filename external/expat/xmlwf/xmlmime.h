#ifdef __cplusplus
extern "C" {
#endif

/* Registered charset names are at most 40 characters long. */

#define CHARSET_MAX 41


void getXMLCharset(const char *buf, char *charset);

#ifdef __cplusplus
}
#endif
