

#ifndef	__USA26MSG__
#define	__USA26MSG__


struct keyspan_usa26_portControlMessage
{
	/*
		there are three types of "commands" sent in the control message:

		1.	configuration changes which must be requested by setting
			the corresponding "set" flag (and should only be requested
			when necessary, to reduce overhead on the USA26):
	*/
	u8	setClocking,	// BOTH: host requests baud rate be set
		baudLo,		// BOTH: host does baud divisor calculation
		baudHi,		// BOTH: baudHi is only used for first port (gives lower rates)
		externalClock_txClocking,
					// USA26: 0=internal, other=external
					// USA17: 0=internal, other=external/RI
		rxClocking,		// USA17: 0=internal, 1=external/RI, other=external/DSR


		setLcr,			// BOTH: host requests lcr be set
		lcr,			// BOTH: use PARITY, STOPBITS, DATABITS below

		setFlowControl,		// BOTH: host requests flow control be set
		ctsFlowControl,		// BOTH: 1=use CTS flow control, 0=don't
		xonFlowControl,		// BOTH: 1=use XON/XOFF flow control, 0=don't
		xonChar,		// BOTH: specified in current character format
		xoffChar,		// BOTH: specified in current character format

		setTxTriState_setRts,
					// USA26: host requests TX tri-state be set
					// USA17: host requests RTS output be set
		txTriState_rts,		// BOTH: 1=active (normal), 0=tristate (off)

		setHskoa_setDtr,
					// USA26: host requests HSKOA output be set
					// USA17: host requests DTR output be set
		hskoa_dtr,		// BOTH: 1=on, 0=off

		setPrescaler,		// USA26: host requests prescalar be set (default: 13)
		prescaler;		// BOTH: specified as N/8; values 8-ff are valid
					// must be set any time internal baud rate is set;
					// must not be set when external clocking is used
					// note: in USA17, prescaler is applied whenever
					// setClocking is requested

	/*
		3.	configuration data which is simply used as is (no overhead,
			but must be specified correctly in every host message).
	*/
	u8	forwardingLength,  // BOTH: forward when this number of chars available
		reportHskiaChanges_dsrFlowControl,
						// USA26: 1=normal; 0=ignore external clock
						// USA17: 1=use DSR flow control, 0=don't
		txAckThreshold,	// BOTH: 0=not allowed, 1=normal, 2-255 deliver ACK faster
		loopbackMode;	// BOTH: 0=no loopback, 1=loopback enabled

	/*
		4.	commands which are flags only; these are processed in order
			(so that, e.g., if both _txOn and _txOff flags are set, the
			port ends in a TX_OFF state); any non-zero value is respected
	*/
	u8	_txOn,			// BOTH: enable transmitting (and continue if there's data)
		_txOff,			// BOTH: stop transmitting
		txFlush,		// BOTH: toss outbound data
		txBreak,		// BOTH: turn on break (cleared by _txOn)
		rxOn,			// BOTH: turn on receiver
		rxOff,			// BOTH: turn off receiver
		rxFlush,		// BOTH: toss inbound data
		rxForward,		// BOTH: forward all inbound data, NOW (as if fwdLen==1)
		returnStatus,	// BOTH: return current status (even if it hasn't changed)
		resetDataToggle;// BOTH: reset data toggle state to DATA0
	
};

// defines for bits in lcr
#define	USA_DATABITS_5		0x00
#define	USA_DATABITS_6		0x01
#define	USA_DATABITS_7		0x02
#define	USA_DATABITS_8		0x03
#define	STOPBITS_5678_1	0x00	// 1 stop bit for all byte sizes
#define	STOPBITS_5_1p5	0x04	// 1.5 stop bits for 5-bit byte
#define	STOPBITS_678_2	0x04	// 2 stop bits for 6/7/8-bit byte
#define	USA_PARITY_NONE		0x00
#define	USA_PARITY_ODD		0x08
#define	USA_PARITY_EVEN		0x18
#define	PARITY_1		0x28
#define	PARITY_0		0x38

// all things called "StatusMessage" are sent on the status endpoint

struct keyspan_usa26_portStatusMessage	// one for each port
{
	u8	port,			// BOTH: 0=first, 1=second, other=see below
		hskia_cts,		// USA26: reports HSKIA pin
						// USA17: reports CTS pin
		gpia_dcd,		// USA26: reports GPIA pin
						// USA17: reports DCD pin
		dsr,			// USA17: reports DSR pin
		ri,				// USA17: reports RI pin
		_txOff,			// port has been disabled (by host)
		_txXoff,		// port is in XOFF state (either host or RX XOFF)
		rxEnabled,		// as configured by rxOn/rxOff 1=on, 0=off
		controlResponse;// 1=a control message has been processed
};

// bits in RX data message when STAT byte is included
#define	RXERROR_OVERRUN	0x02
#define	RXERROR_PARITY	0x04
#define	RXERROR_FRAMING	0x08
#define	RXERROR_BREAK	0x10

struct keyspan_usa26_globalControlMessage
{
	u8	sendGlobalStatus,	// 2=request for two status responses
		resetStatusToggle,	// 1=reset global status toggle
		resetStatusCount;	// a cycling value
};

struct keyspan_usa26_globalStatusMessage
{
	u8	port,				// 3
		sendGlobalStatus,	// from request, decremented
		resetStatusCount;	// as in request
};

struct keyspan_usa26_globalDebugMessage
{
	u8	port,				// 2
		a,
		b,
		c,
		d;
};

// ie: the maximum length of an EZUSB endpoint buffer
#define	MAX_DATA_LEN			64

// update status approx. 60 times a second (16.6666 ms)
#define	STATUS_UPDATE_INTERVAL	16

// status rationing tuning value (each port gets checked each n ms)
#define	STATUS_RATION	10

#endif


