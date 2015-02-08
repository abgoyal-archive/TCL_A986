

#ifndef _XEN_PUBLIC_IO_XENBUS_H
#define _XEN_PUBLIC_IO_XENBUS_H

enum xenbus_state
{
	XenbusStateUnknown      = 0,
	XenbusStateInitialising = 1,
	XenbusStateInitWait     = 2,  /* Finished early
					 initialisation, but waiting
					 for information from the peer
					 or hotplug scripts. */
	XenbusStateInitialised  = 3,  /* Initialised and waiting for a
					 connection from the peer. */
	XenbusStateConnected    = 4,
	XenbusStateClosing      = 5,  /* The device is being closed
					 due to an error or an unplug
					 event. */
	XenbusStateClosed       = 6

};

#endif /* _XEN_PUBLIC_IO_XENBUS_H */

