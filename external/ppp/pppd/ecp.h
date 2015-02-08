

typedef struct ecp_options {
    bool required;		/* Is ECP required? */
    unsigned enctype;		/* Encryption type */
} ecp_options;

extern fsm ecp_fsm[];
extern ecp_options ecp_wantoptions[];
extern ecp_options ecp_gotoptions[];
extern ecp_options ecp_allowoptions[];
extern ecp_options ecp_hisoptions[];

extern struct protent ecp_protent;
