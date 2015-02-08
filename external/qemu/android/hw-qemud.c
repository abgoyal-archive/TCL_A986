
#include "android/hw-qemud.h"
#include "android/utils/debug.h"
#include "android/utils/misc.h"
#include "android/utils/system.h"
#include "android/utils/bufprint.h"
#include "qemu-char.h"
#include "charpipe.h"
#include "cbuffer.h"

#define  D(...)    VERBOSE_PRINT(qemud,__VA_ARGS__)
#define  D_ACTIVE  VERBOSE_CHECK(qemud)

/* the T(...) macro is used to dump traffic */
#define  T_ACTIVE   0

#if T_ACTIVE
#define  T(...)    VERBOSE_PRINT(qemud,__VA_ARGS__)
#else
#define  T(...)    ((void)0)
#endif

#define  MAX_SERIAL_PAYLOAD        4000

#define  MAX_FRAME_PAYLOAD  65535


#define  SUPPORT_LEGACY_QEMUD  1

#if SUPPORT_LEGACY_QEMUD
#include "telephony/android_modem.h"
#include "telephony/modem_driver.h"
#endif




typedef struct QemudSink {
    int       len;
    int       size;
    uint8_t*  buff;
} QemudSink;

static void
qemud_sink_reset( QemudSink*  ss, int  size, uint8_t*  buffer )
{
    ss->len  = 0;
    ss->size = size;
    ss->buff = buffer;
}

static int
qemud_sink_fill( QemudSink*  ss, const uint8_t* *pmsg, int  *plen)
{
    int  avail = ss->size - ss->len;

    if (avail <= 0)
        return 1;

    if (avail > *plen)
        avail = *plen;

    memcpy(ss->buff + ss->len, *pmsg, avail);
    *pmsg += avail;
    *plen -= avail;
    ss->len += avail;

    return (ss->len == ss->size);
}

static int
qemud_sink_needed( QemudSink*  ss )
{
    return ss->size - ss->len;
}



#define  HEADER_SIZE    6

#define  LENGTH_OFFSET  2
#define  LENGTH_SIZE    4

#define  CHANNEL_OFFSET 0
#define  CHANNEL_SIZE   2

#if SUPPORT_LEGACY_QEMUD
typedef enum {
    QEMUD_VERSION_UNKNOWN,
    QEMUD_VERSION_LEGACY,
    QEMUD_VERSION_NORMAL
} QemudVersion;

#  define  LEGACY_LENGTH_OFFSET   0
#  define  LEGACY_CHANNEL_OFFSET  4
#endif

/* length of the framed header */
#define  FRAME_HEADER_SIZE  4

#define  BUFFER_SIZE    MAX_SERIAL_PAYLOAD

typedef void  (*QemudSerialReceive)( void*  opaque, int  channel, uint8_t*  msg, int  msglen);

typedef struct QemudSerial {
    CharDriverState*  cs;  /* serial charpipe endpoint */

    /* managing incoming packets from the serial port */
    ABool         need_header;
    int           overflow;
    int           in_size;
    int           in_channel;
#if SUPPORT_LEGACY_QEMUD
    QemudVersion  version;
#endif
    QemudSink     header[1];
    QemudSink     payload[1];
    uint8_t       data0[MAX_SERIAL_PAYLOAD+1];

    /* receiver */
    QemudSerialReceive  recv_func;    /* receiver callback */
    void*               recv_opaque;  /* receiver user-specific data */
} QemudSerial;


static int
qemud_serial_can_read( void*  opaque )
{
    QemudSerial*  s = opaque;

    if (s->overflow > 0) {
        return s->overflow;
    }

    /* if in_size is 0, we're reading the header */
    if (s->need_header)
        return qemud_sink_needed(s->header);

    /* otherwise, we're reading the payload */
    return qemud_sink_needed(s->payload);
}

static void
qemud_serial_read( void*  opaque, const uint8_t*  from, int  len )
{
    QemudSerial*  s = opaque;

    T("%s: received %3d bytes: '%s'", __FUNCTION__, len, quote_bytes((const void*)from, len));

    while (len > 0) {
        int  avail;

        /* skip overflow bytes */
        if (s->overflow > 0) {
            avail = s->overflow;
            if (avail > len)
                avail = len;

            from += avail;
            len  -= avail;
            continue;
        }

        /* read header if needed */
        if (s->need_header) {
            if (!qemud_sink_fill(s->header, (const uint8_t**)&from, &len))
                break;

#if SUPPORT_LEGACY_QEMUD
            if (s->version == QEMUD_VERSION_UNKNOWN) {
                /* if we receive "001200" as the first header, then we
                 * detected a legacy qemud daemon. See the comments
                 * in qemud_serial_send_legacy_probe() for details.
                 */
                if ( !memcmp(s->data0, "001200", 6) ) {
                    D("%s: legacy qemud detected.", __FUNCTION__);
                    s->version = QEMUD_VERSION_LEGACY;
                    /* tell the modem to use legacy emulation mode */
                    amodem_set_legacy(android_modem);
                } else {
                    D("%s: normal qemud detected.", __FUNCTION__);
                    s->version = QEMUD_VERSION_NORMAL;
                }
            }

            if (s->version == QEMUD_VERSION_LEGACY) {
                s->in_size     = hex2int( s->data0 + LEGACY_LENGTH_OFFSET,  LENGTH_SIZE );
                s->in_channel  = hex2int( s->data0 + LEGACY_CHANNEL_OFFSET, CHANNEL_SIZE );
            } else {
                s->in_size     = hex2int( s->data0 + LENGTH_OFFSET,  LENGTH_SIZE );
                s->in_channel  = hex2int( s->data0 + CHANNEL_OFFSET, CHANNEL_SIZE );
            }
#else
            /* extract payload length + channel id */
            s->in_size     = hex2int( s->data0 + LENGTH_OFFSET,  LENGTH_SIZE );
            s->in_channel  = hex2int( s->data0 + CHANNEL_OFFSET, CHANNEL_SIZE );
#endif
            s->header->len = 0;

            if (s->in_size <= 0 || s->in_channel < 0) {
                D("%s: bad header: '%.*s'", __FUNCTION__, HEADER_SIZE, s->data0);
                continue;
            }

            if (s->in_size > MAX_SERIAL_PAYLOAD) {
                D("%s: ignoring huge serial packet: length=%d channel=%1",
                  __FUNCTION__, s->in_size, s->in_channel);
                s->overflow = s->in_size;
                continue;
            }

            /* prepare 'in_data' for payload */
            s->need_header = 0;
            qemud_sink_reset(s->payload, s->in_size, s->data0);
        }

        /* read payload bytes */
        if (!qemud_sink_fill(s->payload, &from, &len))
            break;

        /* zero-terminate payload, then send it to receiver */
        s->payload->buff[s->payload->size] = 0;
        D("%s: channel=%2d len=%3d '%s'", __FUNCTION__,
          s->in_channel, s->payload->size,
          quote_bytes((const void*)s->payload->buff, s->payload->size));

        s->recv_func( s->recv_opaque, s->in_channel, s->payload->buff, s->payload->size );

        /* prepare for new header */
        s->need_header = 1;
    }
}


#if SUPPORT_LEGACY_QEMUD
static void
qemud_serial_send_legacy_probe( QemudSerial*  s )
{
    /* we're going to send a specially crafted packet to the qemud
     * daemon, this will help us determine whether we're talking
     * to a legacy or a normal daemon.
     *
     * the trick is to known that a legacy daemon uses the following
     * header:
     *
     *    <length><channel><payload>
     *
     * while the normal one uses:
     *
     *    <channel><length><payload>
     *
     * where <channel> is a 2-hexchar string, and <length> a 4-hexchar
     * string.
     *
     * if we send a header of "000100", it is interpreted:
     *
     * - as the header of a 1-byte payload by the legacy daemon
     * - as the header of a 256-byte payload by the normal one.
     *
     * we're going to send something that looks like:
     *
     *   "000100" + "X" +
     *   "000b00" + "connect:gsm" +
     *   "000b00" + "connect:gps" +
     *   "000f00" + "connect:control" +
     *   "00c210" + "0"*194
     *
     * the normal daemon will interpret this as a 256-byte payload
     * for channel 0, with garbage content ("X000b00conn...") which
     * will be silently ignored.
     *
     * on the other hand, the legacy daemon will see it as a
     * series of packets:
     *
     *   one message "X" on channel 0, which will force the daemon
     *   to send back "001200ko:unknown command" as its first answer.
     *
     *   three "connect:<xxx>" messages used to receive the channel
     *   numbers of the three legacy services implemented by the daemon.
     *
     *   a garbage packet of 194 zeroes for channel 16, which will be
     *   silently ignored.
     */
    uint8_t  tab[194];

    memset(tab, 0, sizeof(tab));
    qemu_chr_write(s->cs, (uint8_t*)"000100X", 7);
    qemu_chr_write(s->cs, (uint8_t*)"000b00connect:gsm", 17);
    qemu_chr_write(s->cs, (uint8_t*)"000b00connect:gps", 17);
    qemu_chr_write(s->cs, (uint8_t*)"000f00connect:control", 21);
    qemu_chr_write(s->cs, (uint8_t*)"00c210", 6);
    qemu_chr_write(s->cs, tab, sizeof(tab));
}
#endif /* SUPPORT_LEGACY_QEMUD */

static void
qemud_serial_init( QemudSerial*        s,
                   CharDriverState*    cs,
                   QemudSerialReceive  recv_func,
                   void*               recv_opaque )
{
    s->cs           = cs;
    s->recv_func    = recv_func;
    s->recv_opaque  = recv_opaque;
    s->need_header  = 1;
    s->overflow     = 0;

    qemud_sink_reset( s->header, HEADER_SIZE, s->data0 );
    s->in_size      = 0;
    s->in_channel   = -1;

#if SUPPORT_LEGACY_QEMUD
    s->version = QEMUD_VERSION_UNKNOWN;
    qemud_serial_send_legacy_probe(s);
#endif

    qemu_chr_add_handlers( cs,
                           qemud_serial_can_read,
                           qemud_serial_read,
                           NULL,
                           s );
}

static void
qemud_serial_send( QemudSerial*    s,
                   int             channel,
                   ABool           framing,
                   const uint8_t*  msg,
                   int             msglen )
{
    uint8_t   header[HEADER_SIZE];
    uint8_t   frame[FRAME_HEADER_SIZE];
    int       avail, len = msglen;

    if (msglen <= 0 || channel < 0)
        return;

    D("%s: channel=%2d len=%3d '%s'",
      __FUNCTION__, channel, msglen,
      quote_bytes((const void*)msg, msglen));

    if (framing) {
        len += FRAME_HEADER_SIZE;
    }

    /* packetize the payload for the serial MTU */
    while (len > 0) 
    {
        avail = len;
        if (avail > MAX_SERIAL_PAYLOAD)
            avail = MAX_SERIAL_PAYLOAD;

        /* write this packet's header */
#if SUPPORT_LEGACY_QEMUD
        if (s->version == QEMUD_VERSION_LEGACY) {
            int2hex(header + LEGACY_LENGTH_OFFSET,  LENGTH_SIZE,  avail);
            int2hex(header + LEGACY_CHANNEL_OFFSET, CHANNEL_SIZE, channel);
        } else {
            int2hex(header + LENGTH_OFFSET,  LENGTH_SIZE,  avail);
            int2hex(header + CHANNEL_OFFSET, CHANNEL_SIZE, channel);
        }
#else
        int2hex(header + LENGTH_OFFSET,  LENGTH_SIZE,  avail);
        int2hex(header + CHANNEL_OFFSET, CHANNEL_SIZE, channel);
#endif
        T("%s: '%.*s'", __FUNCTION__, HEADER_SIZE, header);
        qemu_chr_write(s->cs, header, HEADER_SIZE);

        /* insert frame header when needed */
        if (framing) {
            int2hex(frame, FRAME_HEADER_SIZE, msglen);
            T("%s: '%.*s'", __FUNCTION__, FRAME_HEADER_SIZE, frame);
            qemu_chr_write(s->cs, frame, FRAME_HEADER_SIZE);
            avail  -= FRAME_HEADER_SIZE;
            len    -= FRAME_HEADER_SIZE;
            framing = 0;
        }

        /* write message content */
        T("%s: '%.*s'", __FUNCTION__, avail, msg);
        qemu_chr_write(s->cs, msg, avail);
        msg += avail;
        len -= avail;
    }
}



struct QemudClient {
    int               channel;
    QemudSerial*      serial;
    void*             clie_opaque;
    QemudClientRecv   clie_recv;
    QemudClientClose  clie_close;
    QemudService*     service;
    QemudClient*      next_serv; /* next in same service */
    QemudClient*      next;
    QemudClient**     pref;

    /* framing support */
    int               framing;
    ABool             need_header;
    QemudSink         header[1];
    uint8_t           header0[FRAME_HEADER_SIZE];
    QemudSink         payload[1];
};

static void  qemud_service_remove_client( QemudService*  service,
                                          QemudClient*   client );

/* remove a QemudClient from global list */
static void
qemud_client_remove( QemudClient*  c )
{
    c->pref[0] = c->next;
    if (c->next)
        c->next->pref = c->pref;

    c->next = NULL;
    c->pref = &c->next;
}

/* add a QemudClient to global list */
static void
qemud_client_prepend( QemudClient*  c, QemudClient** plist )
{
    c->next = *plist;
    c->pref = plist;
    *plist  = c;
    if (c->next)
        c->next->pref = &c->next;
}

static void
qemud_client_recv( void*  opaque, uint8_t*  msg, int  msglen )
{
    QemudClient*  c = opaque;

    /* no framing, things are simple */
    if (!c->framing) {
        if (c->clie_recv)
            c->clie_recv( c->clie_opaque, msg, msglen, c );
        return;
    }

    /* framing */

#if 1
    /* special case, in 99% of cases, everything is in
     * the incoming message, and we can do all we need
     * directly without dynamic allocation.
     */
    if (msglen > FRAME_HEADER_SIZE   &&
        c->need_header == 1          &&
        qemud_sink_needed(c->header) == 0)
    {
        int  len = hex2int( msg, FRAME_HEADER_SIZE );

        if (len >= 0 && msglen == len + FRAME_HEADER_SIZE) {
            if (c->clie_recv)
                c->clie_recv( c->clie_opaque, 
                              msg+FRAME_HEADER_SIZE,
                              msglen-FRAME_HEADER_SIZE, c );
            return;
        }
    }
#endif

    while (msglen > 0) {
        uint8_t *data;

        /* read the header */
        if (c->need_header) {
            int       frame_size;
            uint8_t*  data;

            if (!qemud_sink_fill(c->header, (const uint8_t**)&msg, &msglen))
                break;

            frame_size = hex2int(c->header0, 4);
            if (frame_size == 0) {
                D("%s: ignoring empty frame", __FUNCTION__);
                continue;
            }
            if (frame_size < 0) {
                D("%s: ignoring corrupted frame header '.*s'",
                  __FUNCTION__, FRAME_HEADER_SIZE, c->header0 );
                continue;
            }

            AARRAY_NEW(data, frame_size+1);  /* +1 for terminating zero */
            qemud_sink_reset(c->payload, frame_size, data);
            c->need_header = 0;
            c->header->len = 0;
        }

        /* read the payload */
        if (!qemud_sink_fill(c->payload, (const uint8_t**)&msg, &msglen))
            break;

        c->payload->buff[c->payload->size] = 0;
        c->need_header = 1;
        data = c->payload->buff;

        /* Technically, calling 'clie_recv' can destroy client object 'c'
         * if it decides to close the connection, so ensure we don't
         * use/dereference it after the call. */
        if (c->clie_recv)
            c->clie_recv( c->clie_opaque, c->payload->buff, c->payload->size, c );

        AFREE(data);
    }
}

static void
qemud_client_disconnect( void*  opaque )
{
    QemudClient*  c = opaque;

    /* remove from current list */
    qemud_client_remove(c);

    /* send a disconnect command to the daemon */
    if (c->channel > 0) {
        char  tmp[128], *p=tmp, *end=p+sizeof(tmp);
        p = bufprint(tmp, end, "disconnect:%02x", c->channel);
        qemud_serial_send(c->serial, 0, 0, (uint8_t*)tmp, p-tmp);
    }

    /* call the client close callback */
    if (c->clie_close) {
        c->clie_close(c->clie_opaque);
        c->clie_close = NULL;
    }
    c->clie_recv = NULL;

    /* remove from service list, if any */
    if (c->service) {
        qemud_service_remove_client(c->service, c);
        c->service = NULL;
    }

    AFREE(c);
}

/* allocate a new QemudClient object */
static QemudClient*
qemud_client_alloc( int               channel_id,
                    void*             clie_opaque,
                    QemudClientRecv   clie_recv,
                    QemudClientClose  clie_close,
                    QemudSerial*      serial,
                    QemudClient**     pclients )
{
    QemudClient*  c;

    ANEW0(c);

    c->serial      = serial;
    c->channel     = channel_id;
    c->clie_opaque = clie_opaque;
    c->clie_recv   = clie_recv;
    c->clie_close  = clie_close;

    c->framing     = 0;
    c->need_header = 1;
    qemud_sink_reset(c->header, FRAME_HEADER_SIZE, c->header0);

    qemud_client_prepend(c, pclients);

    return c;
}


struct QemudService {
    const char*          name;
    int                  max_clients;
    int                  num_clients;
    QemudClient*         clients;
    QemudServiceConnect  serv_connect;
    void*                serv_opaque;
    QemudService*        next;
};

/* Create a new QemudService object */
static QemudService*
qemud_service_new( const char*          name,
                   int                  max_clients,
                   void*                serv_opaque,
                   QemudServiceConnect  serv_connect,
                   QemudService**       pservices )
{
    QemudService*  s;

    ANEW0(s);
    s->name        = ASTRDUP(name);
    s->max_clients = max_clients;
    s->num_clients = 0;
    s->clients     = NULL;

    s->serv_opaque  = serv_opaque;
    s->serv_connect = serv_connect;

    s->next    = *pservices;
    *pservices = s;

    return s;
}

static void
qemud_service_add_client( QemudService*  s, QemudClient*  c )
{
    c->service      = s;
    c->next_serv    = s->clients;
    s->clients      = c;
    s->num_clients += 1;
}

/* used internally to remove a QemudClient from a QemudService */
static void
qemud_service_remove_client( QemudService*  s, QemudClient*  c )
{
    QemudClient**  pnode = &s->clients;
    QemudClient*   node;

    /* remove from clients linked-list */
    for (;;) {
        node = *pnode;
        if (node == NULL) {
            D("%s: could not find client %d for service '%s'",
              __FUNCTION__, c->channel, s->name);
            return;
        }
        if (node == c)
            break;
        pnode = &node->next_serv;
    }

    *pnode          = node->next_serv;
    s->num_clients -= 1;
}


typedef struct QemudMultiplexer  QemudMultiplexer;

struct QemudMultiplexer {
    QemudSerial    serial[1];
    QemudClient*   clients;
    QemudService*  services;
};

static void
qemud_multiplexer_serial_recv( void*     opaque,
                               int       channel,
                               uint8_t*  msg,
                               int       msglen )
{
    QemudMultiplexer*  m = opaque;
    QemudClient*       c = m->clients;

    /* dispatch to an existing client if possible
     * note that channel 0 is handled by a special
     * QemudClient that is setup in qemud_multiplexer_init()
     */
    for ( ; c != NULL; c = c->next ) {
        if (c->channel == channel) {
            qemud_client_recv(c, msg, msglen);
            return;
        }
    }

    D("%s: ignoring %d bytes for unknown channel %d",
      __FUNCTION__, msglen, channel);
}

static int
qemud_multiplexer_connect( QemudMultiplexer*  m,
                           const char*        service_name,
                           int                channel_id )
{
    QemudService*  sv;
    QemudClient*   client;

    /* find the corresponding registered service by name */
    for (sv = m->services; sv != NULL; sv = sv->next) {
        if (!strcmp(sv->name, service_name)) {
            break;
        }
    }

    if (sv == NULL) {
        D("%s: no registered '%s' service", __FUNCTION__, service_name);
        return -1;
    }

    /* check service's client count */
    if (sv->max_clients > 0 && sv->num_clients >= sv->max_clients) {
        D("%s: registration failed for '%s' service: too many clients (%d)",
          __FUNCTION__, service_name, sv->num_clients);
        return -2;
    }

    /* ask the service to create a new QemudClient. Note that we
     * assume that this calls qemud_client_new() which will add
     * the client to the service's list automatically.
     */
    client = sv->serv_connect( sv->serv_opaque, sv, channel_id );
    if (client == NULL) {
        D("%s: registration failed for '%s' service",
          __FUNCTION__, service_name);
        return -1;
    }

    D("%s: registered client channel %d for '%s' service",
      __FUNCTION__, channel_id, service_name);
    return 0;
}

/* disconnect a given client from its channel id */
static void
qemud_multiplexer_disconnect( QemudMultiplexer*  m,
                              int                channel )
{
    QemudClient*  c;

    /* find the client by its channel id, then disconnect it */
    for (c = m->clients; c; c = c->next) {
        if (c->channel == channel) {
            D("%s: disconnecting client %d",
              __FUNCTION__, channel);
            /* note thatt this removes the client from
             * m->clients automatically.
             */
            c->channel = -1; /* no need to send disconnect:<id> */
            qemud_client_disconnect(c);
            return;
        }
    }
    D("%s: disconnecting unknown channel %d",
      __FUNCTION__, channel);
}

static void
qemud_multiplexer_control_recv( void*         opaque,
                                uint8_t*      msg,
                                int           msglen,
                                QemudClient*  client )
{
    QemudMultiplexer*  mult   = opaque;
    uint8_t*           msgend = msg + msglen;
    char               tmp[64], *p=tmp, *end=p+sizeof(tmp);

    /* handle connection attempts.
     * the client message must be "connect:<service-name>:<id>"
     * where <id> is a 2-char hexadecimal string, which must be > 0
     */
    if (msglen > 8 && !memcmp(msg, "connect:", 8))
    {
        const char*    service_name = (const char*)msg + 8;
        int            channel, ret;
        char*          q;

        q = strchr(service_name, ':');
        if (q == NULL || q+3 != (char*)msgend) {
            D("%s: malformed connect message: '%.*s' (offset=%d)",
              __FUNCTION__, msglen, (const char*)msg, q ? q-(char*)msg : -1);
            return;
        }
        *q++ = 0;  /* zero-terminate service name */
        channel = hex2int((uint8_t*)q, 2);
        if (channel <= 0) {
            D("%s: malformed channel id '%.*s",
              __FUNCTION__, 2, q);
            return;
        }

        ret = qemud_multiplexer_connect(mult, service_name, channel);
        /* the answer can be one of:
         *    ok:connect:<id>
         *    ko:connect:<id>:<reason-for-failure>
         */
        if (ret < 0) {
            if (ret == -1) {
                /* could not connect */
                p = bufprint(tmp, end, "ko:connect:%02x:unknown service", channel);
            } else {
                p = bufprint(tmp, end, "ko:connect:%02x:service busy", channel);
            }
        }
        else {
            p = bufprint(tmp, end, "ok:connect:%02x", channel);
        }
        qemud_serial_send(mult->serial, 0, 0, (uint8_t*)tmp, p-tmp);
        return;
    }

    /* handle client disconnections,
     * this message arrives when the client has closed the connection.
     * format: "disconnect:<id>" where <id> is a 2-hex channel id > 0
     */
    if (msglen == 13 && !memcmp(msg, "disconnect:", 11)) {
        int  channel_id = hex2int(msg+11, 2);
        if (channel_id <= 0) {
            D("%s: malformed disconnect channel id: '%.*s'",
              __FUNCTION__, 2, msg+11);
            return;
        }
        qemud_multiplexer_disconnect(mult, channel_id);
        return;
    }

#if SUPPORT_LEGACY_QEMUD
    /* an ok:connect:<service>:<id> message can be received if we're
     * talking to a legacy qemud daemon, i.e. one running in a 1.0 or
     * 1.1 system image.
     *
     * we should treat is as a normal "connect:" attempt, except that
     * we must not send back any acknowledgment.
     */
    if (msglen > 11 && !memcmp(msg, "ok:connect:", 11)) {
        const char*  service_name = (const char*)msg + 11;
        char*        q            = strchr(service_name, ':');
        int          channel;

        if (q == NULL || q+3 != (char*)msgend) {
            D("%s: malformed legacy connect message: '%.*s' (offset=%d)",
              __FUNCTION__, msglen, (const char*)msg, q ? q-(char*)msg : -1);
            return;
        }
        *q++ = 0;  /* zero-terminate service name */
        channel = hex2int((uint8_t*)q, 2);
        if (channel <= 0) {
            D("%s: malformed legacy channel id '%.*s",
              __FUNCTION__, 2, q);
            return;
        }

        switch (mult->serial->version) {
        case QEMUD_VERSION_UNKNOWN:
            mult->serial->version = QEMUD_VERSION_LEGACY;
            D("%s: legacy qemud daemon detected.", __FUNCTION__);
            break;

        case QEMUD_VERSION_LEGACY:
            /* nothing unusual */
            break;

        default:
            D("%s: weird, ignoring legacy qemud control message: '%.*s'",
              __FUNCTION__, msglen, msg);
            return;
        }

        /* "hw-control" was called "control" in 1.0/1.1 */
        if (!strcmp(service_name,"control"))
            service_name = "hw-control";

        qemud_multiplexer_connect(mult, service_name, channel);
        return;
    }

    /* anything else, don't answer for legacy */
    if (mult->serial->version == QEMUD_VERSION_LEGACY)
        return;
#endif /* SUPPORT_LEGACY_QEMUD */

    /* anything else is a problem */
    p = bufprint(tmp, end, "ko:unknown command");
    qemud_serial_send(mult->serial, 0, 0, (uint8_t*)tmp, p-tmp);
}

static void
qemud_multiplexer_init( QemudMultiplexer*  mult,
                        CharDriverState*   serial_cs )
{
    QemudClient*  control;

    /* initialize serial handler */
    qemud_serial_init( mult->serial,
                       serial_cs,
                       qemud_multiplexer_serial_recv,
                       mult );

    /* setup listener for channel 0 */
    control = qemud_client_alloc( 0,
                                  mult,
                                  qemud_multiplexer_control_recv,
                                  NULL,
                                  mult->serial,
                                  &mult->clients );
}

/* the global multiplexer state */
static QemudMultiplexer  _multiplexer[1];


QemudClient*
qemud_client_new( QemudService*     service,
                  int               channelId,
                  void*             clie_opaque,
                  QemudClientRecv   clie_recv,
                  QemudClientClose  clie_close )
{
    QemudMultiplexer*  m = _multiplexer;
    QemudClient*       c = qemud_client_alloc( channelId,
                                               clie_opaque,
                                               clie_recv,
                                               clie_close,
                                               m->serial,
                                               &m->clients );

    qemud_service_add_client(service, c);
    return c;
}

void
qemud_client_send ( QemudClient*  client, const uint8_t*  msg, int  msglen )
{
    qemud_serial_send(client->serial, client->channel, client->framing != 0, msg, msglen);
}

void
qemud_client_set_framing( QemudClient*  client, int  framing )
{
    /* release dynamic buffer if we're disabling framing */
    if (client->framing) {
        if (!client->need_header) {
            AFREE(client->payload->buff);
            client->need_header = 1;
        }
    }
    client->framing = !!framing;
}

void
qemud_client_close( QemudClient*  client )
{
    qemud_client_disconnect(client);
}



static CharDriverState*  android_qemud_cs;

extern void
android_qemud_init( void )
{
    CharDriverState*    cs;

    if (android_qemud_cs != NULL)
        return;

    if (qemu_chr_open_charpipe( &android_qemud_cs, &cs ) < 0) {
        derror( "%s: can't create charpipe to serial port",
                __FUNCTION__ );
        exit(1);
    }

    qemud_multiplexer_init(_multiplexer, cs);
}

CharDriverState*  android_qemud_get_cs( void )
{
    if (android_qemud_cs == NULL)
        android_qemud_init();

    return android_qemud_cs;
}

QemudService*
qemud_service_register( const char*          service_name,
                        int                  max_clients,
                        void*                serv_opaque,
                        QemudServiceConnect  serv_connect )
{
    QemudMultiplexer*  m  = _multiplexer;
    QemudService*      sv;

    if (android_qemud_cs == NULL)
        android_qemud_init();

    sv = qemud_service_new(service_name,
                             max_clients,
                             serv_opaque,
                             serv_connect,
                             &m->services);

    return sv;
}

extern void
qemud_service_broadcast( QemudService*  sv,
                         const uint8_t*  msg,
                         int             msglen )
{
    QemudClient*  c;

    for (c = sv->clients; c; c = c->next_serv)
        qemud_client_send(c, msg, msglen);
}




typedef struct {
    QemudService*     service;
    CharDriverState*  cs;
} QemudCharService;

static void
_qemud_char_client_recv( void*  opaque, uint8_t*  msg, int  msglen,
                         QemudClient*  client )
{
    CharDriverState*  cs = opaque;
    qemu_chr_write(cs, msg, msglen);
}

static void
_qemud_char_client_close( void*  opaque )
{
    derror("unexpected qemud char. channel close");
}


static int
_qemud_char_service_can_read( void*  opaque )
{
    return 8192;  /* whatever */
}

static void
_qemud_char_service_read( void*  opaque, const uint8_t*  from, int  len )
{
    QemudService*  sv = opaque;
    qemud_service_broadcast( sv, from, len );
}

static QemudClient*
_qemud_char_service_connect( void*  opaque, QemudService*  sv, int  channel )
{
    CharDriverState*   cs = opaque;
    QemudClient*       c  = qemud_client_new( sv, channel,
                                              cs,
                                              _qemud_char_client_recv,
                                              _qemud_char_client_close);

    /* now we can open the gates :-) */
    qemu_chr_add_handlers( cs,
                           _qemud_char_service_can_read,
                           _qemud_char_service_read,
                           NULL,
                           sv );

    return c;
}

int
android_qemud_get_channel( const char*  name, CharDriverState* *pcs )
{
    CharDriverState*   cs;

    if (qemu_chr_open_charpipe(&cs, pcs) < 0) {
        derror("can't open charpipe for '%s' qemud service", name);
        exit(2);
    }
    qemud_service_register(name, 1, cs, _qemud_char_service_connect);
    return 0;
}

int
android_qemud_set_channel( const char*  name, CharDriverState*  peer_cs )
{
    CharDriverState*  char_buffer = qemu_chr_open_buffer(peer_cs);

    if (char_buffer == NULL)
        return -1;

    qemud_service_register(name, 1, char_buffer, _qemud_char_service_connect);
    return 0;
}
