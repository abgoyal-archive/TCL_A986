

#ifndef __riodrvr_h
#define __riodrvr_h

#include <asm/param.h>		/* for HZ */

#define MEMDUMP_SIZE	32
#define	MOD_DISABLE	(RIO_NOREAD|RIO_NOWRITE|RIO_NOXPRINT)


struct rio_info {
	int mode;		/* Intr or polled, word/byte */
	spinlock_t RIOIntrSem;	/* Interrupt thread sem */
	int current_chan;	/* current channel */
	int RIOFailed;		/* Not initialised ? */
	int RIOInstallAttempts;	/* no. of rio-install() calls */
	int RIOLastPCISearch;	/* status of last search */
	int RIONumHosts;	/* Number of RIO Hosts */
	struct Host *RIOHosts;	/* RIO Host values */
	struct Port **RIOPortp;	/* RIO port values */
	int RIOPrintDisabled;	/* RIO printing disabled ? */
	int RIOPrintLogState;	/* RIO printing state ? */
	int RIOPolling;		/* Polling ? */
	int RIOHalted;		/* halted ? */
	int RIORtaDisCons;	/* RTA connections/disconnections */
	unsigned int RIOReadCheck;	/* Rio read check */
	unsigned int RIONoMessage;	/* To display message or not */
	unsigned int RIONumBootPkts;	/* how many packets for an RTA */
	unsigned int RIOBootCount;	/* size of RTA code */
	unsigned int RIOBooting;	/* count of outstanding boots */
	unsigned int RIOSystemUp;	/* Booted ?? */
	unsigned int RIOCounting;	/* for counting interrupts */
	unsigned int RIOIntCount;	/* # of intr since last check */
	unsigned int RIOTxCount;	/* number of xmit intrs  */
	unsigned int RIORxCount;	/* number of rx intrs */
	unsigned int RIORupCount;	/* number of rup intrs */
	int RIXTimer;
	int RIOBufferSize;	/* Buffersize */
	int RIOBufferMask;	/* Buffersize */

	int RIOFirstMajor;	/* First host card's major no */

	unsigned int RIOLastPortsMapped;	/* highest port number known */
	unsigned int RIOFirstPortsMapped;	/* lowest port number known */

	unsigned int RIOLastPortsBooted;	/* highest port number running */
	unsigned int RIOFirstPortsBooted;	/* lowest port number running */

	unsigned int RIOLastPortsOpened;	/* highest port number running */
	unsigned int RIOFirstPortsOpened;	/* lowest port number running */

	/* Flag to say that the topology information has been changed. */
	unsigned int RIOQuickCheck;
	unsigned int CdRegister;	/* ??? */
	int RIOSignalProcess;	/* Signalling process */
	int rio_debug;		/* To debug ... */
	int RIODebugWait;	/* For what ??? */
	int tpri;		/* Thread prio */
	int tid;		/* Thread id */
	unsigned int _RIO_Polled;	/* Counter for polling */
	unsigned int _RIO_Interrupted;	/* Counter for interrupt */
	int intr_tid;		/* iointset return value */
	int TxEnSem;		/* TxEnable Semaphore */


	struct Error RIOError;	/* to Identify what went wrong */
	struct Conf RIOConf;	/* Configuration ??? */
	struct ttystatics channel[RIO_PORTS];	/* channel information */
	char RIOBootPackets[1 + (SIXTY_FOUR_K / RTA_BOOT_DATA_SIZE)]
	    [RTA_BOOT_DATA_SIZE];
	struct Map RIOConnectTable[TOTAL_MAP_ENTRIES];
	struct Map RIOSavedTable[TOTAL_MAP_ENTRIES];

	/* RTA to host binding table for master/slave operation */
	unsigned long RIOBindTab[MAX_RTA_BINDINGS];
	/* RTA memory dump variable */
	unsigned char RIOMemDump[MEMDUMP_SIZE];
	struct ModuleInfo RIOModuleTypes[MAX_MODULE_TYPES];

};


#ifdef linux
#define debug(x)        printk x
#else
#define debug(x)	kkprintf x
#endif



#define RIO_RESET_INT	0x7d80

#endif				/* __riodrvr.h */
