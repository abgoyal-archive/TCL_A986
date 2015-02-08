

/* Name to address translation routines. */

extern const char *linkaddr_string(const u_char *, const unsigned int);
extern const char *etheraddr_string(const u_char *);
extern const char *etherproto_string(u_short);
extern const char *tcpport_string(u_short);
extern const char *udpport_string(u_short);
extern const char *getname(const u_char *);
#ifdef INET6
extern const char *getname6(const u_char *);
#endif
extern const char *intoa(u_int32_t);

extern void init_addrtoname(u_int32_t, u_int32_t);
extern struct hnamemem *newhnamemem(void);
#ifdef INET6
extern struct h6namemem *newh6namemem(void);
#endif

#define ipaddr_string(p) getname((const u_char *)(p))
#ifdef INET6
#define ip6addr_string(p) getname6((const u_char *)(p))
#endif
