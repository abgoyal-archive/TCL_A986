

#ifndef __rio_unixrup_h__
#define __rio_unixrup_h__

struct UnixRup {
	struct CmdBlk *CmdsWaitingP;	/* Commands waiting to be done */
	struct CmdBlk *CmdPendingP;	/* The command currently being sent */
	struct RUP __iomem *RupP;	/* the Rup to send it to */
	unsigned int Id;		/* Id number */
	unsigned int BaseSysPort;	/* SysPort of first tty on this RTA */
	unsigned int ModTypes;		/* Modules on this RTA */
	spinlock_t RupLock;	/* Lock structure for MPX */
	/*    struct lockb     RupLock;	*//* Lock structure for MPX */
};

#endif				/* __rio_unixrup_h__ */
