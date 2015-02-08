

#ifndef __rio_cmdblk_h__
#define __rio_cmdblk_h__


struct CmdBlk {
	struct CmdBlk *NextP;	/* Pointer to next command block */
	struct PKT Packet;	/* A packet, to copy to the rup */
	/* The func to call to check if OK */
	int (*PreFuncP) (unsigned long, struct CmdBlk *);
	int PreArg;		/* The arg for the func */
	/* The func to call when completed */
	int (*PostFuncP) (unsigned long, struct CmdBlk *);
	int PostArg;		/* The arg for the func */
};

#define NUM_RIO_CMD_BLKS (3 * (MAX_RUP * 4 + LINKS_PER_UNIT * 4))
#endif
