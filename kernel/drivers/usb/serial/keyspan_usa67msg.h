

#ifndef	__USA67MSG__
#define	__USA67MSG__


// all things called "ControlMessage" are sent on the 'control' endpoint

typedef struct keyspan_usa67_portControlMessage
{
	u8	port;		// 0 or 1 (selects port)
	/*
		there are three types of "commands" sent in the control message:

		1.	configuration changes which must be requested by setting
			the corresponding "set" flag (and should only be requested
			when necessary, to reduce overhead on the device):
	*/
	u8	setClocking,	// host requests baud rate be set
		baudLo,			// host does baud divisor calculation
		baudHi,			// baudHi is only used for first port (gives lower rates)
		externalClock_txClocking,
						// 0=internal, other=external

		setLcr,			// host requests lcr be set
		lcr,			// use PARITY, STOPBITS, DATABITS below

		setFlowControl,	// host requests flow control be set
		ctsFlowControl,	// 1=use CTS flow control, 0=don't
		xonFlowControl,	// 1=use XON/XOFF flow control, 0=don't
		xonChar,		// specified in current character format
		xoffChar,		// specified in current character format

		setTxTriState_setRts,
						// host requests TX tri-state be set
		txTriState_rts,	// 1=active (normal), 0=tristate (off)

		setHskoa_setDtr,
						// host requests HSKOA output be set
		hskoa_dtr,		// 1=on, 0=off

		setPrescaler,	// host requests prescalar be set (default: 13)
		prescaler;		// specified as N/8; values 8-ff are valid
						// must be set any time internal baud rate is set;
						// must not be set when external clocking is used

	/*
		3.	configuration data which is simply used as is (no overhead,
			but must be specified correctly in every host message).
	*/
	u8	forwardingLength,  // forward when this number of chars available
		reportHskiaChanges_dsrFlowControl,
						// 1=normal; 0=ignore external clock
						// 1=use DSR flow control, 0=don't
		txAckThreshold,	// 0=not allowed, 1=normal, 2-255 deliver ACK faster
		loopbackMode;	// 0=no loopback, 1=loopback enabled

	/*
		4.	commands which are flags only; these are processed in order
			(so that, e.g., if both _txOn and _txOff flags are set, the
			port ends in a TX_OFF state); any non-zero value is respected
	*/
	u8	_txOn,			// enable transmitting (and continue if there's data)
		_txOff,			// stop transmitting
		txFlush,		// toss outbound data
		txBreak,		// turn on break (cleared by _txOn)
		rxOn,			// turn on receiver
		rxOff,			// turn off receiver
		rxFlush,		// toss inbound data
		rxForward,		// forward all inbound data, NOW (as if fwdLen==1)
		returnStatus,	// return current status (even if it hasn't changed)
		resetDataToggle;// reset data toggle state to DATA0

} keyspan_usa67_portControlMessage;

// defines for bits in lcr
#define	USA_DATABITS_5		0x00
#define	USA_DATABITS_6		0x01
#define	USA_DATABITS_7		0x02
#define	USA_DATABITS_8		0x03
#define	STOPBITS_5678_1		0x00	// 1 stop bit for all byte sizes
#define	STOPBITS_5_1p5		0x04	// 1.5 stop bits for 5-bit byte
#define	STOPBITS_678_2		0x04	// 2 stop bits for 6/7/8-bit byte
#define	USA_PARITY_NONE		0x00
#define	USA_PARITY_ODD		0x08
#define	USA_PARITY_EVEN		0x18
#define	PARITY_1			0x28
#define	PARITY_0			0x38

// all things called "StatusMessage" are sent on the status endpoint

typedef struct keyspan_usa67_portStatusMessage	// one for each port
{
	u8	port,			// 0=first, 1=second, other=see below
		hskia_cts,		// reports HSKIA pin
		gpia_dcd,		// reports GPIA pin
		_txOff,			// port has been disabled (by host)
		_txXoff,		// port is in XOFF state (either host or RX XOFF)
		txAck,			// indicates a TX message acknowledgement
		rxEnabled,		// as configured by rxOn/rxOff 1=on, 0=off
		controlResponse;// 1=a control message has been processed
} keyspan_usa67_portStatusMessage;

// bits in RX data message when STAT byte is included
#define	RXERROR_OVERRUN	0x02
#define	RXERROR_PARITY	0x04
#define	RXERROR_FRAMING	0x08
#define	RXERROR_BREAK	0x10

typedef struct keyspan_usa67_globalControlMessage
{
	u8	port,	 			// 3
		sendGlobalStatus,	// 2=request for two status responses
		resetStatusToggle,	// 1=reset global status toggle
		resetStatusCount;	// a cycling value
} keyspan_usa67_globalControlMessage;

typedef struct keyspan_usa67_globalStatusMessage
{
	u8	port,				// 3
		sendGlobalStatus,	// from request, decremented
		resetStatusCount;	// as in request
} keyspan_usa67_globalStatusMessage;

typedef struct keyspan_usa67_globalDebugMessage
{
	u8	port,				// 2
		a,
		b,
		c,
		d;
} keyspan_usa67_globalDebugMessage;

// ie: the maximum length of an FX1 endpoint buffer
#define	MAX_DATA_LEN			64

// update status approx. 60 times a second (16.6666 ms)
#define	STATUS_UPDATE_INTERVAL	16

// status rationing tuning value (each port gets checked each n ms)
#define	STATUS_RATION	10

#endif


