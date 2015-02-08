#ifndef _EVHTTP_H_
#define _EVHTTP_H_

#include <event.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <windows.h>
#undef WIN32_LEAN_AND_MEAN
#endif


/* Response codes */
#define HTTP_OK			200
#define HTTP_NOCONTENT		204
#define HTTP_MOVEPERM		301
#define HTTP_MOVETEMP		302
#define HTTP_NOTMODIFIED	304
#define HTTP_BADREQUEST		400
#define HTTP_NOTFOUND		404
#define HTTP_SERVUNAVAIL	503

struct evhttp;
struct evhttp_request;
struct evkeyvalq;

struct evhttp *evhttp_new(struct event_base *base);

int evhttp_bind_socket(struct evhttp *http, const char *address, u_short port);

int evhttp_accept_socket(struct evhttp *http, int fd);

void evhttp_free(struct evhttp* http);

/** Set a callback for a specified URI */
void evhttp_set_cb(struct evhttp *, const char *,
    void (*)(struct evhttp_request *, void *), void *);

/** Removes the callback for a specified URI */
int evhttp_del_cb(struct evhttp *, const char *);

void evhttp_set_gencb(struct evhttp *,
    void (*)(struct evhttp_request *, void *), void *);

void evhttp_set_timeout(struct evhttp *, int timeout_in_secs);

/* Request/Response functionality */

void evhttp_send_error(struct evhttp_request *req, int error,
    const char *reason);

void evhttp_send_reply(struct evhttp_request *req, int code,
    const char *reason, struct evbuffer *databuf);

/* Low-level response interface, for streaming/chunked replies */
void evhttp_send_reply_start(struct evhttp_request *, int, const char *);
void evhttp_send_reply_chunk(struct evhttp_request *, struct evbuffer *);
void evhttp_send_reply_end(struct evhttp_request *);

struct evhttp *evhttp_start(const char *address, u_short port);

enum evhttp_cmd_type { EVHTTP_REQ_GET, EVHTTP_REQ_POST, EVHTTP_REQ_HEAD };

enum evhttp_request_kind { EVHTTP_REQUEST, EVHTTP_RESPONSE };

struct evhttp_request {
#if defined(TAILQ_ENTRY)
	TAILQ_ENTRY(evhttp_request) next;
#else
struct {
	struct evhttp_request *tqe_next;
	struct evhttp_request **tqe_prev;
}       next;
#endif

	/* the connection object that this request belongs to */
	struct evhttp_connection *evcon;
	int flags;
#define EVHTTP_REQ_OWN_CONNECTION	0x0001
#define EVHTTP_PROXY_REQUEST		0x0002

	struct evkeyvalq *input_headers;
	struct evkeyvalq *output_headers;

	/* address of the remote host and the port connection came from */
	char *remote_host;
	u_short remote_port;

	enum evhttp_request_kind kind;
	enum evhttp_cmd_type type;

	char *uri;			/* uri after HTTP request was parsed */

	char major;			/* HTTP Major number */
	char minor;			/* HTTP Minor number */

	int response_code;		/* HTTP Response code */
	char *response_code_line;	/* Readable response */

	struct evbuffer *input_buffer;	/* read data */
	ev_int64_t ntoread;
	int chunked;

	struct evbuffer *output_buffer;	/* outgoing post or data */

	/* Callback */
	void (*cb)(struct evhttp_request *, void *);
	void *cb_arg;

	/*
	 * Chunked data callback - call for each completed chunk if
	 * specified.  If not specified, all the data is delivered via
	 * the regular callback.
	 */
	void (*chunk_cb)(struct evhttp_request *, void *);
};

struct evhttp_request *evhttp_request_new(
	void (*cb)(struct evhttp_request *, void *), void *arg);

/** enable delivery of chunks to requestor */
void evhttp_request_set_chunked_cb(struct evhttp_request *,
    void (*cb)(struct evhttp_request *, void *));

/** Frees the request object and removes associated events. */
void evhttp_request_free(struct evhttp_request *req);

struct evhttp_connection *evhttp_connection_new(
	const char *address, unsigned short port);

/** Frees an http connection */
void evhttp_connection_free(struct evhttp_connection *evcon);

/** sets the ip address from which http connections are made */
void evhttp_connection_set_local_address(struct evhttp_connection *evcon,
    const char *address);

/** sets the local port from which http connections are made */
void evhttp_connection_set_local_port(struct evhttp_connection *evcon,
    unsigned short port);

/** Sets the timeout for events related to this connection */
void evhttp_connection_set_timeout(struct evhttp_connection *evcon,
    int timeout_in_secs);

/** Sets the retry limit for this connection - -1 repeats indefnitely */
void evhttp_connection_set_retries(struct evhttp_connection *evcon,
    int retry_max);

/** Set a callback for connection close. */
void evhttp_connection_set_closecb(struct evhttp_connection *evcon,
    void (*)(struct evhttp_connection *, void *), void *);

void evhttp_connection_set_base(struct evhttp_connection *evcon,
    struct event_base *base);

/** Get the remote address and port associated with this connection. */
void evhttp_connection_get_peer(struct evhttp_connection *evcon,
    char **address, u_short *port);

/** The connection gets ownership of the request */
int evhttp_make_request(struct evhttp_connection *evcon,
    struct evhttp_request *req,
    enum evhttp_cmd_type type, const char *uri);

const char *evhttp_request_uri(struct evhttp_request *req);

/* Interfaces for dealing with HTTP headers */

const char *evhttp_find_header(const struct evkeyvalq *, const char *);
int evhttp_remove_header(struct evkeyvalq *, const char *);
int evhttp_add_header(struct evkeyvalq *, const char *, const char *);
void evhttp_clear_headers(struct evkeyvalq *);

/* Miscellaneous utility functions */


char *evhttp_encode_uri(const char *uri);


char *evhttp_decode_uri(const char *uri);


void evhttp_parse_query(const char *uri, struct evkeyvalq *headers);


char *evhttp_htmlescape(const char *html);

#ifdef __cplusplus
}
#endif

#endif /* _EVHTTP_H_ */
