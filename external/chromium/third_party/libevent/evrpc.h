#ifndef _EVRPC_H_
#define _EVRPC_H_

#ifdef __cplusplus
extern "C" {
#endif


struct evbuffer;
struct event_base;
struct evrpc_req_generic;

/* Encapsulates a request */
struct evrpc {
	TAILQ_ENTRY(evrpc) next;

	/* the URI at which the request handler lives */
	const char* uri;

	/* creates a new request structure */
	void *(*request_new)(void);

	/* frees the request structure */
	void (*request_free)(void *);

	/* unmarshals the buffer into the proper request structure */
	int (*request_unmarshal)(void *, struct evbuffer *);

	/* creates a new reply structure */
	void *(*reply_new)(void);

	/* creates a new reply structure */
	void (*reply_free)(void *);

	/* verifies that the reply is valid */
	int (*reply_complete)(void *);
	
	/* marshals the reply into a buffer */
	void (*reply_marshal)(struct evbuffer*, void *);

	/* the callback invoked for each received rpc */
	void (*cb)(struct evrpc_req_generic *, void *);
	void *cb_arg;

	/* reference for further configuration */
	struct evrpc_base *base;
};

#define EVRPC_STRUCT(rpcname) struct evrpc_req__##rpcname

struct evhttp_request;
struct evrpc_status;

/* We alias the RPC specific structs to this voided one */
struct evrpc_req_generic {
	/* the unmarshaled request object */
	void *request;

	/* the empty reply object that needs to be filled in */
	void *reply;

	/* 
	 * the static structure for this rpc; that can be used to
	 * automatically unmarshal and marshal the http buffers.
	 */
	struct evrpc *rpc;

	/*
	 * the http request structure on which we need to answer.
	 */
	struct evhttp_request* http_req;

	/*
	 * callback to reply and finish answering this rpc
	 */
	void (*done)(struct evrpc_req_generic* rpc); 
};

#define EVRPC_HEADER(rpcname, reqstruct, rplystruct) \
EVRPC_STRUCT(rpcname) {	\
	struct reqstruct* request; \
	struct rplystruct* reply; \
	struct evrpc* rpc; \
	struct evhttp_request* http_req; \
	void (*done)(struct evrpc_status *, \
	    struct evrpc* rpc, void *request, void *reply);	     \
};								     \
int evrpc_send_request_##rpcname(struct evrpc_pool *, \
    struct reqstruct *, struct rplystruct *, \
    void (*)(struct evrpc_status *, \
	struct reqstruct *, struct rplystruct *, void *cbarg),	\
    void *);

#define EVRPC_GENERATE(rpcname, reqstruct, rplystruct) \
int evrpc_send_request_##rpcname(struct evrpc_pool *pool, \
    struct reqstruct *request, struct rplystruct *reply, \
    void (*cb)(struct evrpc_status *, \
	struct reqstruct *, struct rplystruct *, void *cbarg),	\
    void *cbarg) { \
	struct evrpc_status status;				    \
	struct evrpc_request_wrapper *ctx;			    \
	ctx = (struct evrpc_request_wrapper *) \
	    malloc(sizeof(struct evrpc_request_wrapper));	    \
	if (ctx == NULL)					    \
		goto error;					    \
	ctx->pool = pool;					    \
	ctx->evcon = NULL;					    \
	ctx->name = strdup(#rpcname);				    \
	if (ctx->name == NULL) {				    \
		free(ctx);					    \
		goto error;					    \
	}							    \
	ctx->cb = (void (*)(struct evrpc_status *, \
		void *, void *, void *))cb;			    \
	ctx->cb_arg = cbarg;					    \
	ctx->request = (void *)request;				    \
	ctx->reply = (void *)reply;				    \
	ctx->request_marshal = (void (*)(struct evbuffer *, void *))reqstruct##_marshal; \
	ctx->reply_clear = (void (*)(void *))rplystruct##_clear;    \
	ctx->reply_unmarshal = (int (*)(void *, struct evbuffer *))rplystruct##_unmarshal; \
	return (evrpc_make_request(ctx));			    \
error:								    \
	memset(&status, 0, sizeof(status));			    \
	status.error = EVRPC_STATUS_ERR_UNSTARTED;		    \
	(*(cb))(&status, request, reply, cbarg);		    \
	return (-1);						    \
}

#define EVRPC_REQUEST_HTTP(rpc_req) (rpc_req)->http_req

#define EVRPC_REQUEST_DONE(rpc_req) do { \
  struct evrpc_req_generic *_req = (struct evrpc_req_generic *)(rpc_req); \
  _req->done(_req); \
} while (0)
  

/* Takes a request object and fills it in with the right magic */
#define EVRPC_REGISTER_OBJECT(rpc, name, request, reply) \
  do { \
    (rpc)->uri = strdup(#name); \
    if ((rpc)->uri == NULL) {			 \
      fprintf(stderr, "failed to register object\n");	\
      exit(1);						\
    } \
    (rpc)->request_new = (void *(*)(void))request##_new; \
    (rpc)->request_free = (void (*)(void *))request##_free; \
    (rpc)->request_unmarshal = (int (*)(void *, struct evbuffer *))request##_unmarshal; \
    (rpc)->reply_new = (void *(*)(void))reply##_new; \
    (rpc)->reply_free = (void (*)(void *))reply##_free; \
    (rpc)->reply_complete = (int (*)(void *))reply##_complete; \
    (rpc)->reply_marshal = (void (*)(struct evbuffer*, void *))reply##_marshal; \
  } while (0)

struct evrpc_base;
struct evhttp;

/* functions to start up the rpc system */

struct evrpc_base *evrpc_init(struct evhttp *server);

void evrpc_free(struct evrpc_base *base);

#define EVRPC_REGISTER(base, name, request, reply, callback, cbarg) \
  do { \
    struct evrpc* rpc = (struct evrpc *)calloc(1, sizeof(struct evrpc)); \
    EVRPC_REGISTER_OBJECT(rpc, name, request, reply); \
    evrpc_register_rpc(base, rpc, \
	(void (*)(struct evrpc_req_generic*, void *))callback, cbarg);	\
  } while (0)

int evrpc_register_rpc(struct evrpc_base *, struct evrpc *,
    void (*)(struct evrpc_req_generic*, void *), void *);

#define EVRPC_UNREGISTER(base, name) evrpc_unregister_rpc(base, #name)

int evrpc_unregister_rpc(struct evrpc_base *base, const char *name);


struct evrpc_pool;
struct evhttp_connection;

struct evrpc_status {
#define EVRPC_STATUS_ERR_NONE		0
#define EVRPC_STATUS_ERR_TIMEOUT	1
#define EVRPC_STATUS_ERR_BADPAYLOAD	2
#define EVRPC_STATUS_ERR_UNSTARTED	3
#define EVRPC_STATUS_ERR_HOOKABORTED	4
	int error;

	/* for looking at headers or other information */
	struct evhttp_request *http_req;
};

struct evrpc_request_wrapper {
	TAILQ_ENTRY(evrpc_request_wrapper) next;

        /* pool on which this rpc request is being made */
        struct evrpc_pool *pool;

        /* connection on which the request is being sent */
	struct evhttp_connection *evcon;

	/* event for implementing request timeouts */
	struct event ev_timeout;

	/* the name of the rpc */
	char *name;

	/* callback */
	void (*cb)(struct evrpc_status*, void *request, void *reply, void *arg);
	void *cb_arg;

	void *request;
	void *reply;

	/* unmarshals the buffer into the proper request structure */
	void (*request_marshal)(struct evbuffer *, void *);

	/* removes all stored state in the reply */
	void (*reply_clear)(void *);

	/* marshals the reply into a buffer */
	int (*reply_unmarshal)(void *, struct evbuffer*);
};

#define EVRPC_MAKE_REQUEST(name, pool, request, reply, cb, cbarg)	\
	evrpc_send_request_##name(pool, request, reply, cb, cbarg)

int evrpc_make_request(struct evrpc_request_wrapper *);

struct evrpc_pool *evrpc_pool_new(struct event_base *base);
void evrpc_pool_free(struct evrpc_pool *pool);
void evrpc_pool_add_connection(struct evrpc_pool *, 
    struct evhttp_connection *);

void evrpc_pool_set_timeout(struct evrpc_pool *pool, int timeout_in_secs);


enum EVRPC_HOOK_TYPE {
	EVRPC_INPUT,		/**< apply the function to an input hook */
	EVRPC_OUTPUT		/**< apply the function to an output hook */
};

#ifndef WIN32
#define INPUT EVRPC_INPUT
#define OUTPUT EVRPC_OUTPUT
#endif

void *evrpc_add_hook(void *vbase,
    enum EVRPC_HOOK_TYPE hook_type,
    int (*cb)(struct evhttp_request *, struct evbuffer *, void *),
    void *cb_arg);

int evrpc_remove_hook(void *vbase,
    enum EVRPC_HOOK_TYPE hook_type,
    void *handle);

#ifdef __cplusplus
}
#endif

#endif /* _EVRPC_H_ */
