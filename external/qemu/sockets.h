
/* headers to use the BSD sockets */
#ifndef QEMU_SOCKET_H
#define QEMU_SOCKET_H

#include <stddef.h>
#include <stdint.h>
#include <errno.h>


#ifdef _WIN32
#  ifndef EINTR
#    define EINTR        10004
#  endif
#  ifndef EAGAIN
#    define EAGAIN       10035
#  endif
#  ifndef EWOULDBLOCK
#    define EWOULDBLOCK  EAGAIN
#  endif
#  ifndef EINPROGRESS
#    define EINPROGRESS  10036
#  endif
#  ifndef EALREADY
#    define EALREADY     10037
#  endif
#  ifndef EDESTADDRREQ
#    define EDESTADDRREQ 10039
#  endif
#  ifndef EMSGSIZE
#    define EMSGSIZE     10040
#  endif
#  ifndef EPROTOTYPE
#    define EPROTOTYPE   10041
#  endif
#  ifndef ENOPROTOOPT
#    define ENOPROTOOPT  10042
#  endif
#  ifndef EAFNOSUPPORT
#    define EAFNOSUPPORT 10047
#  endif
#  ifndef EADDRINUSE
#    define EADDRINUSE   10048
#  endif
#  ifndef EADDRNOTAVAIL
#    define EADDRNOTAVAIL 10049
#  endif
#  ifndef ENETDOWN
#    define ENETDOWN     10050
#  endif
#  ifndef ENETUNREACH
#    define ENETUNREACH  10051
#  endif
#  ifndef ENETRESET
#    define ENETRESET    10052
#  endif
#  ifndef ECONNABORTED
#    define ECONNABORTED 10053
#  endif
#  ifndef ECONNRESET
#    define ECONNRESET   10054
#  endif
#  ifndef ENOBUFS
#    define ENOBUFS      10055
#  endif
#  ifndef EISCONN
#    define EISCONN      10056
#  endif
#  ifndef ENOTCONN
#    define ENOTCONN     10057
#  endif
#  ifndef ESHUTDOWN
#    define ESHUTDOWN     10058
#  endif
#  ifndef ETOOMANYREFS
#    define ETOOMANYREFS  10059
#  endif
#  ifndef ETIMEDOUT
#    define ETIMEDOUT     10060
#  endif
#  ifndef ECONNREFUSED
#    define ECONNREFUSED  10061
#  endif
#  ifndef ELOOP
#    define ELOOP         10062
#  endif
#  ifndef EHOSTDOWN
#    define EHOSTDOWN     10064
#  endif
#  ifndef EHOSTUNREACH
#    define EHOSTUNREACH  10065
#  endif
#endif /* _WIN32 */

#ifdef _WIN32
  extern const char*  _errno_str(void);
#  define  errno_str   _errno_str()
#else
#  define  errno_str   strerror(errno)
#endif

#define  HAVE_IN6_SOCKETS   1

/* Unix sockets are not available on Win32 */
#ifndef _WIN32
#  define  HAVE_UNIX_SOCKETS  1
#endif

int  socket_init( void );

/* return the name of the current host */
char*  host_name( void );

/* supported socket types */
typedef enum {
    SOCKET_DGRAM = 0,
    SOCKET_STREAM
} SocketType;

/* supported socket families */
typedef enum {
    SOCKET_UNSPEC,
    SOCKET_INET,
    SOCKET_IN6,
    SOCKET_UNIX
} SocketFamily;

typedef struct {
    SocketFamily  family;
    union {
        struct {
            uint16_t   port;
            uint32_t   address;
        } inet;
        struct {
            uint16_t   port;
            uint8_t    address[16];
        } in6;
        struct {
            int          owner;
            const char*  path;
        } _unix;
    } u;
} SockAddress;

#define  SOCK_ADDRESS_INET_ANY       0x00000000
#define  SOCK_ADDRESS_INET_LOOPBACK  0x7f000001

void  sock_address_init_inet( SockAddress*  a, uint32_t  ip, uint16_t  port );

#if HAVE_IN6_SOCKETS
void  sock_address_init_in6 ( SockAddress*  a, const uint8_t*  ip6[16], uint16_t  port );
#endif

#if HAVE_UNIX_SOCKETS
void  sock_address_init_unix( SockAddress*  a, const char*  path );
#endif

/* Finalize a socket address, only needed for now for Unix addresses */
void  sock_address_done( SockAddress*  a );

int   sock_address_equal( const SockAddress*  a, const SockAddress*  b );

/* THIS SHOULD DISAPPEAR SOON - TRANSITIONAL HELPER */
int   sock_address_to_bsd( const SockAddress*  a, void*  sa, size_t* salen );
int   sock_address_from_bsd( SockAddress*  a, const void*  sa, size_t  salen );
int   sock_address_to_inet( SockAddress*  a, int  *paddr_ip, int  *paddr_port );

/* return a static string describing the address */
const char*  sock_address_to_string( const SockAddress*  a );

static __inline__
SocketFamily  sock_address_get_family( const SockAddress*  a )
{
    return a->family;
}

/* return the port number of a given socket address, or -1 if it's a Unix one */
int   sock_address_get_port( const SockAddress*  a );

/* set the port number of a given socket address, don't do anything for Unix ones */
void  sock_address_set_port( SockAddress*  a, uint16_t  port );

/* return the path of a given Unix socket, returns NULL for non-Unix ones */
const char*  sock_address_get_path( const SockAddress*  a );

/* return the inet address, or -1 if it's not SOCKET_INET */
int   sock_address_get_ip( const SockAddress*  a );

/* bufprint a socket address into a human-readable string */
char* bufprint_sock_address( char*  p, char*  end, const SockAddress*  a );

int   sock_address_init_resolve( SockAddress*  a,
                                 const char*   hostname,
                                 uint16_t      port,
                                 int           preferIn6 );

int  sock_address_get_numeric_info( SockAddress*  a,
                                    char*         host,
                                    size_t        hostlen,
                                    char*         serv,
                                    size_t        servlen );

/* Support for listing all socket addresses of a given host */
enum {
    SOCKET_LIST_PASSIVE    = (1 << 0),
    SOCKET_LIST_FORCE_INET = (1 << 1),
    SOCKET_LIST_FORCE_IN6  = (1 << 2)
};

SockAddress**  sock_address_list_create( const char*  hostname,
                                         const char*  port,
                                         unsigned     flags );

void sock_address_list_free( SockAddress**  list );

/* create a new socket, return the socket number of -1 on failure */
int  socket_create( SocketFamily  family, SocketType  type );

int   socket_create_inet( SocketType  type );

#if HAVE_IN6_SOCKETS
int   socket_create_in6 ( SocketType  type );
#endif

#if HAVE_UNIX_SOCKETS
int   socket_create_unix( SocketType  type );
#endif

/* return the type of a given socket */
SocketType  socket_get_type(int  fd);

/* set SO_REUSEADDR on Unix, SO_EXCLUSIVEADDR on Windows */
int  socket_set_xreuseaddr(int  fd);

/* set socket in non-blocking mode */
int  socket_set_nonblock(int fd);

/* set socket in blocking mode */
int  socket_set_blocking(int fd);

/* disable the TCP Nagle algorithm for lower latency */
int  socket_set_nodelay(int fd);

/* send OOB data inline for this socket */
int  socket_set_oobinline(int  fd);

/* force listening to IPv6 interfaces only */
int  socket_set_ipv6only(int  fd);

/* retrieve last socket error code */
int  socket_get_error(int  fd);

void  socket_close( int  fd );

int   socket_recv    ( int  fd, void*  buf, int  buflen );
int   socket_recvfrom( int  fd, void*  buf, int  buflen, SockAddress*  from );

int   socket_send  ( int  fd, const void*  buf, int  buflen );
int   socket_send_oob( int  fd, const void*  buf, int  buflen );
int   socket_sendto( int  fd, const void*  buf, int  buflen, const SockAddress*  to );

int   socket_connect( int  fd, const SockAddress*  address );
int   socket_bind( int  fd, const SockAddress*  address );
int   socket_get_address( int  fd, SockAddress*  address );
int   socket_get_peer_address( int  fd, SockAddress*  address );
int   socket_listen( int  fd, int  backlog );
int   socket_accept( int  fd, SockAddress*  address );

/* returns the number of bytes that can be read from a socket */
int   socket_can_read( int  fd );

int  socket_pair(int  *fd1, int *fd2);

/* create a server socket listening on the host's loopback interface */
int  socket_loopback_server( int  port, SocketType  type );

/* connect to a port on the host's loopback interface */
int  socket_loopback_client( int  port, SocketType  type );

/* create a server socket listening to a Unix domain path */
#if HAVE_UNIX_SOCKETS
int  socket_unix_server( const char*  name, SocketType  type );
#endif

/* create a Unix sockets and connects it to a Unix server */
#if HAVE_UNIX_SOCKETS
int  socket_unix_client( const char*  name, SocketType  type );
#endif

/* create an IPv4 client socket and connect it to a given host */
int  socket_network_client( const char*  host, int  port, SocketType  type );

/* create an IPv4 socket and binds it to a given port of the host's interface */
int  socket_anyaddr_server( int  port, SocketType  type );

int  socket_accept_any( int  server_fd );


int  socket_mcast_inet_add_membership( int  s, uint32_t  ip );
int  socket_mcast_inet_drop_membership( int  s, uint32_t  ip );
int  socket_mcast_inet_set_loop( int  s, int  enabled );
int  socket_mcast_inet_set_ttl( int  s, int  ttl );

#endif /* QEMU_SOCKET_H */
