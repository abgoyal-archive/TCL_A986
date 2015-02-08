
#ifndef _ASM_IA64_SN_IOERROR_H
#define _ASM_IA64_SN_IOERROR_H


typedef struct io_error_s {
    /* Bit fields indicating which structure fields are valid */
    union {
	struct {
	    unsigned                ievb_errortype:1;
	    unsigned                ievb_widgetnum:1;
	    unsigned                ievb_widgetdev:1;
	    unsigned                ievb_srccpu:1;
	    unsigned                ievb_srcnode:1;
	    unsigned                ievb_errnode:1;
	    unsigned                ievb_sysioaddr:1;
	    unsigned                ievb_xtalkaddr:1;
	    unsigned                ievb_busspace:1;
	    unsigned                ievb_busaddr:1;
	    unsigned                ievb_vaddr:1;
	    unsigned                ievb_memaddr:1;
	    unsigned		    ievb_epc:1;
	    unsigned		    ievb_ef:1;
	    unsigned		    ievb_tnum:1;
	} iev_b;
	unsigned                iev_a;
    } ie_v;

    short                   ie_errortype;	/* error type: extra info about error */
    short                   ie_widgetnum;	/* Widget number that's in error */
    short                   ie_widgetdev;	/* Device within widget in error */
    cpuid_t                 ie_srccpu;	/* CPU on srcnode generating error */
    cnodeid_t               ie_srcnode;		/* Node which caused the error   */
    cnodeid_t               ie_errnode;		/* Node where error was noticed  */
    iopaddr_t               ie_sysioaddr;	/* Sys specific IO address       */
    iopaddr_t               ie_xtalkaddr;	/* Xtalk (48bit) addr of Error   */
    iopaddr_t               ie_busspace;	/* Bus specific address space    */
    iopaddr_t               ie_busaddr;		/* Bus specific address          */
    caddr_t                 ie_vaddr;	/* Virtual address of error      */
    iopaddr_t               ie_memaddr;		/* Physical memory address       */
    caddr_t		    ie_epc;		/* pc when error reported	 */
    caddr_t		    ie_ef;		/* eframe when error reported	 */
    short		    ie_tnum;		/* Xtalk TNUM field */
} ioerror_t;

#define	IOERROR_INIT(e)		do { (e)->ie_v.iev_a = 0; } while (0)
#define	IOERROR_SETVALUE(e,f,v)	do { (e)->ie_ ## f = (v); (e)->ie_v.iev_b.ievb_ ## f = 1; } while (0)

#endif /* _ASM_IA64_SN_IOERROR_H */
