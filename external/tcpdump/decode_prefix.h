

#ifndef tcpdump_decode_prefix_h
#define tcpdump_decode_prefix_h

extern int decode_prefix4(const u_char *pptr, char *buf, u_int buflen);
#ifdef INET6
extern int decode_prefix6(const u_char *pd, char *buf, u_int buflen);
#endif

#endif
