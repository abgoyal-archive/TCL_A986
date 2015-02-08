

#define HEADERLEN	4


#define CONFREQ		1	/* Configuration Request */
#define CONFACK		2	/* Configuration Ack */
#define CONFNAK		3	/* Configuration Nak */
#define CONFREJ		4	/* Configuration Reject */
#define TERMREQ		5	/* Termination Request */
#define TERMACK		6	/* Termination Ack */
#define CODEREJ		7	/* Code Reject */


typedef struct fsm {
    int unit;			/* Interface unit number */
    int protocol;		/* Data Link Layer Protocol field value */
    int state;			/* State */
    int flags;			/* Contains option bits */
    u_char id;			/* Current id */
    u_char reqid;		/* Current request id */
    u_char seen_ack;		/* Have received valid Ack/Nak/Rej to Req */
    int timeouttime;		/* Timeout time in milliseconds */
    int maxconfreqtransmits;	/* Maximum Configure-Request transmissions */
    int retransmits;		/* Number of retransmissions left */
    int maxtermtransmits;	/* Maximum Terminate-Request transmissions */
    int nakloops;		/* Number of nak loops since last ack */
    int rnakloops;		/* Number of naks received */
    int maxnakloops;		/* Maximum number of nak loops tolerated */
    struct fsm_callbacks *callbacks;	/* Callback routines */
    char *term_reason;		/* Reason for closing protocol */
    int term_reason_len;	/* Length of term_reason */
} fsm;


typedef struct fsm_callbacks {
    void (*resetci)		/* Reset our Configuration Information */
		__P((fsm *));
    int  (*cilen)		/* Length of our Configuration Information */
		__P((fsm *));
    void (*addci) 		/* Add our Configuration Information */
		__P((fsm *, u_char *, int *));
    int  (*ackci)		/* ACK our Configuration Information */
		__P((fsm *, u_char *, int));
    int  (*nakci)		/* NAK our Configuration Information */
		__P((fsm *, u_char *, int, int));
    int  (*rejci)		/* Reject our Configuration Information */
		__P((fsm *, u_char *, int));
    int  (*reqci)		/* Request peer's Configuration Information */
		__P((fsm *, u_char *, int *, int));
    void (*up)			/* Called when fsm reaches OPENED state */
		__P((fsm *));
    void (*down)		/* Called when fsm leaves OPENED state */
		__P((fsm *));
    void (*starting)		/* Called when we want the lower layer */
		__P((fsm *));
    void (*finished)		/* Called when we don't want the lower layer */
		__P((fsm *));
    void (*protreject)		/* Called when Protocol-Reject received */
		__P((int));
    void (*retransmit)		/* Retransmission is necessary */
		__P((fsm *));
    int  (*extcode)		/* Called when unknown code received */
		__P((fsm *, int, int, u_char *, int));
    char *proto_name;		/* String name for protocol (for messages) */
} fsm_callbacks;


#define INITIAL		0	/* Down, hasn't been opened */
#define STARTING	1	/* Down, been opened */
#define CLOSED		2	/* Up, hasn't been opened */
#define STOPPED		3	/* Open, waiting for down event */
#define CLOSING		4	/* Terminating the connection, not open */
#define STOPPING	5	/* Terminating, but open */
#define REQSENT		6	/* We've sent a Config Request */
#define ACKRCVD		7	/* We've received a Config Ack */
#define ACKSENT		8	/* We've sent a Config Ack */
#define OPENED		9	/* Connection available */


#define OPT_PASSIVE	1	/* Don't die if we don't get a response */
#define OPT_RESTART	2	/* Treat 2nd OPEN as DOWN, UP */
#define OPT_SILENT	4	/* Wait for peer to speak first */


#define DEFTIMEOUT	3	/* Timeout time in seconds */
#define DEFMAXTERMREQS	2	/* Maximum Terminate-Request transmissions */
#define DEFMAXCONFREQS	10	/* Maximum Configure-Request transmissions */
#define DEFMAXNAKLOOPS	5	/* Maximum number of nak loops */


void fsm_init __P((fsm *));
void fsm_lowerup __P((fsm *));
void fsm_lowerdown __P((fsm *));
void fsm_open __P((fsm *));
void fsm_close __P((fsm *, char *));
void fsm_input __P((fsm *, u_char *, int));
void fsm_protreject __P((fsm *));
void fsm_sdata __P((fsm *, int, int, u_char *, int));


extern int peer_mru[];		/* currently negotiated peer MRU (per unit) */
