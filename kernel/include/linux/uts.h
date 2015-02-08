
#ifndef _LINUX_UTS_H
#define _LINUX_UTS_H

#ifndef UTS_SYSNAME
#define UTS_SYSNAME "Linux"
#endif

#ifndef UTS_NODENAME
#define UTS_NODENAME "(none)"	/* set by sethostname() */
#endif

#ifndef UTS_DOMAINNAME
#define UTS_DOMAINNAME "(none)"	/* set by setdomainname() */
#endif

#endif
