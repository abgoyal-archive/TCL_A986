
#ifndef _SLIRP_HELPER_H
#define _SLIRP_HELPER_H

#ifdef _WIN32
#  include <winsock2.h>  /* for ntohl */
#endif

typedef union {
    u_int32_t   addr;
    u_int8_t    data[4];
} ipaddr_t;

/* return ip address in network order */
static __inline__ uint32_t
ip_getn( ipaddr_t  ip )
{
    return ip.addr;
}

/* return ip address in host order */
static __inline__ uint32_t
ip_geth( ipaddr_t  ip )
{
    return ntohl(ip.addr);
}

/* set ip address in network order */
static __inline__ ipaddr_t
ip_setn( uint32_t  val )
{
    ipaddr_t  ip;
    ip.addr = val;
    return ip;
}

/* set ip address in host order */
static __inline__ ipaddr_t
ip_seth( uint32_t  val )
{
    ipaddr_t  ip;
    ip.addr = htonl(val);
    return ip;
}

static __inline__ ipaddr_t
ip_read( const void*  buf )
{
    ipaddr_t  ip;
    memcpy(ip.data, buf, 4);
    return ip;
}

static __inline__ void
ip_write( ipaddr_t  ip, void*  buf )
{
    memcpy(buf, ip.data, 4);
}

static __inline__ uint32_t
ip_read32h( const void* buf )
{
    ipaddr_t  addr = ip_read(buf);
    return ip_geth(addr);
}

static __inline__ void
ip_write32h( uint32_t  ip, void*  buf )
{
    ipaddr_t  addr = ip_seth(ip);
    ip_write(addr, buf);
}

static __inline__ int
ip_equal( ipaddr_t  ip1, ipaddr_t  ip2 )
{
    return ip1.addr == ip2.addr;
}

typedef union {
    u_int16_t  port;
    u_int8_t   data[2];
} port_t;

static __inline__ uint16_t
port_getn( port_t   p )
{
    return p.port;
}

static __inline__ uint16_t
port_geth( port_t  p )
{
    return ntohs(p.port);
}

static __inline__ port_t
port_setn( uint16_t  val )
{
    port_t  p;
    p.port = val;
    return p;
}

static __inline__ port_t
port_seth( uint16_t  val )
{
    port_t  p;
    p.port = htons(val);
    return p;
}

#endif /* _SLIRP_HELPER_H */
