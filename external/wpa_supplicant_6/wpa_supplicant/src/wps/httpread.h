

#ifndef HTTPREAD_H
#define HTTPREAD_H

/* event types (passed to callback) */
enum httpread_event {
	HTTPREAD_EVENT_FILE_READY = 1,        /* including reply ready */
	HTTPREAD_EVENT_TIMEOUT = 2,
	HTTPREAD_EVENT_ERROR = 3      /* misc. error, esp malloc error */
};


enum httpread_hdr_type {
	HTTPREAD_HDR_TYPE_UNKNOWN = 0,      /* none of the following */
	HTTPREAD_HDR_TYPE_REPLY = 1,        /* hdr begins w/ HTTP/ */
	HTTPREAD_HDR_TYPE_GET = 2,          /* hdr begins with GET<sp> */
	HTTPREAD_HDR_TYPE_HEAD = 3,         /* hdr begins with HEAD<sp> */
	HTTPREAD_HDR_TYPE_POST = 4,         /* hdr begins with POST<sp> */
	HTTPREAD_HDR_TYPE_PUT = 5,          /* hdr begins with ... */
	HTTPREAD_HDR_TYPE_DELETE = 6,       /* hdr begins with ... */
	HTTPREAD_HDR_TYPE_TRACE = 7,        /* hdr begins with ... */
	HTTPREAD_HDR_TYPE_CONNECT = 8,      /* hdr begins with ... */
	HTTPREAD_HDR_TYPE_NOTIFY = 9,       /* hdr begins with ... */
	HTTPREAD_HDR_TYPE_M_SEARCH = 10,    /* hdr begins with ... */
	HTTPREAD_HDR_TYPE_M_POST = 11,      /* hdr begins with ... */
	HTTPREAD_HDR_TYPE_SUBSCRIBE = 12,   /* hdr begins with ... */
	HTTPREAD_HDR_TYPE_UNSUBSCRIBE = 13, /* hdr begins with ... */

	HTTPREAD_N_HDR_TYPES    /* keep last */
};


struct httpread;


void httpread_destroy(struct httpread *h);

struct httpread * httpread_create(
	int sd,         /* descriptor of TCP socket to read from */
	void (*cb)(struct httpread *handle, void *cookie,
		    enum httpread_event e),  /* call on event */
	void *cookie,    /* pass to callback */
	int max_bytes,          /* maximum file size else abort it */
	int timeout_seconds     /* 0; or total duration timeout period */
	);

enum httpread_hdr_type httpread_hdr_type_get(struct httpread *h);


char *httpread_uri_get(struct httpread *h);

/* httpread_reply_code_get -- When reply is ready, returns reply code */
int httpread_reply_code_get(struct httpread *h);


/* httpread_length_get -- When file is ready, returns file length. */
int httpread_length_get(struct httpread *h);

void * httpread_data_get(struct httpread *h);

char * httpread_hdr_get(struct httpread *h);

char * httpread_hdr_line_get(struct httpread *h, const char *tag);

#endif /* HTTPREAD_H */
