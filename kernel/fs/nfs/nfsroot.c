

#include <linux/types.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/sunrpc/clnt.h>
#include <linux/sunrpc/xprtsock.h>
#include <linux/nfs.h>
#include <linux/nfs_fs.h>
#include <linux/nfs_mount.h>
#include <linux/in.h>
#include <linux/major.h>
#include <linux/utsname.h>
#include <linux/inet.h>
#include <linux/root_dev.h>
#include <net/ipconfig.h>
#include <linux/parser.h>

#include "internal.h"

/* Define this to allow debugging output */
#undef NFSROOT_DEBUG
#define NFSDBG_FACILITY NFSDBG_ROOT

/* Default port to use if server is not running a portmapper */
#define NFS_MNT_PORT	627

/* Default path we try to mount. "%s" gets replaced by our IP address */
#define NFS_ROOT		"/tftpboot/%s"

/* Parameters passed from the kernel command line */
static char nfs_root_name[256] __initdata = "";

/* Address of NFS server */
static __be32 servaddr __initdata = 0;

/* Name of directory to mount */
static char nfs_export_path[NFS_MAXPATHLEN + 1] __initdata = { 0, };

/* NFS-related data */
static struct nfs_mount_data nfs_data __initdata = { 0, };/* NFS mount info */
static int nfs_port __initdata = 0;		/* Port to connect to for NFS */
static int mount_port __initdata = 0;		/* Mount daemon port number */



enum {
	/* Options that take integer arguments */
	Opt_port, Opt_rsize, Opt_wsize, Opt_timeo, Opt_retrans, Opt_acregmin,
	Opt_acregmax, Opt_acdirmin, Opt_acdirmax,
	/* Options that take no arguments */
	Opt_soft, Opt_hard, Opt_intr,
	Opt_nointr, Opt_posix, Opt_noposix, Opt_cto, Opt_nocto, Opt_ac, 
	Opt_noac, Opt_lock, Opt_nolock, Opt_v2, Opt_v3, Opt_udp, Opt_tcp,
	Opt_acl, Opt_noacl,
	/* Error token */
	Opt_err
};

static const match_table_t tokens __initconst = {
	{Opt_port, "port=%u"},
	{Opt_rsize, "rsize=%u"},
	{Opt_wsize, "wsize=%u"},
	{Opt_timeo, "timeo=%u"},
	{Opt_retrans, "retrans=%u"},
	{Opt_acregmin, "acregmin=%u"},
	{Opt_acregmax, "acregmax=%u"},
	{Opt_acdirmin, "acdirmin=%u"},
	{Opt_acdirmax, "acdirmax=%u"},
	{Opt_soft, "soft"},
	{Opt_hard, "hard"},
	{Opt_intr, "intr"},
	{Opt_nointr, "nointr"},
	{Opt_posix, "posix"},
	{Opt_noposix, "noposix"},
	{Opt_cto, "cto"},
	{Opt_nocto, "nocto"},
	{Opt_ac, "ac"},
	{Opt_noac, "noac"},
	{Opt_lock, "lock"},
	{Opt_nolock, "nolock"},
	{Opt_v2, "nfsvers=2"},
	{Opt_v2, "v2"},
	{Opt_v3, "nfsvers=3"},
	{Opt_v3, "v3"},
	{Opt_udp, "proto=udp"},
	{Opt_udp, "udp"},
	{Opt_tcp, "proto=tcp"},
	{Opt_tcp, "tcp"},
	{Opt_acl, "acl"},
	{Opt_noacl, "noacl"},
	{Opt_err, NULL}
	
};


static int __init root_nfs_parse(char *name, char *buf)
{

	char *p;
	substring_t args[MAX_OPT_ARGS];
	int option;

	if (!name)
		return 1;

	/* Set the NFS remote path */
	p = strsep(&name, ",");
	if (p[0] != '\0' && strcmp(p, "default") != 0)
		strlcpy(buf, p, NFS_MAXPATHLEN);

	while ((p = strsep (&name, ",")) != NULL) {
		int token; 
		if (!*p)
			continue;
		token = match_token(p, tokens, args);

		/* %u tokens only. Beware if you add new tokens! */
		if (token < Opt_soft && match_int(&args[0], &option))
			return 0;
		switch (token) {
			case Opt_port:
				nfs_port = option;
				break;
			case Opt_rsize:
				nfs_data.rsize = option;
				break;
			case Opt_wsize:
				nfs_data.wsize = option;
				break;
			case Opt_timeo:
				nfs_data.timeo = option;
				break;
			case Opt_retrans:
				nfs_data.retrans = option;
				break;
			case Opt_acregmin:
				nfs_data.acregmin = option;
				break;
			case Opt_acregmax:
				nfs_data.acregmax = option;
				break;
			case Opt_acdirmin:
				nfs_data.acdirmin = option;
				break;
			case Opt_acdirmax:
				nfs_data.acdirmax = option;
				break;
			case Opt_soft:
				nfs_data.flags |= NFS_MOUNT_SOFT;
				break;
			case Opt_hard:
				nfs_data.flags &= ~NFS_MOUNT_SOFT;
				break;
			case Opt_intr:
			case Opt_nointr:
				break;
			case Opt_posix:
				nfs_data.flags |= NFS_MOUNT_POSIX;
				break;
			case Opt_noposix:
				nfs_data.flags &= ~NFS_MOUNT_POSIX;
				break;
			case Opt_cto:
				nfs_data.flags &= ~NFS_MOUNT_NOCTO;
				break;
			case Opt_nocto:
				nfs_data.flags |= NFS_MOUNT_NOCTO;
				break;
			case Opt_ac:
				nfs_data.flags &= ~NFS_MOUNT_NOAC;
				break;
			case Opt_noac:
				nfs_data.flags |= NFS_MOUNT_NOAC;
				break;
			case Opt_lock:
				nfs_data.flags &= ~NFS_MOUNT_NONLM;
				break;
			case Opt_nolock:
				nfs_data.flags |= NFS_MOUNT_NONLM;
				break;
			case Opt_v2:
				nfs_data.flags &= ~NFS_MOUNT_VER3;
				break;
			case Opt_v3:
				nfs_data.flags |= NFS_MOUNT_VER3;
				break;
			case Opt_udp:
				nfs_data.flags &= ~NFS_MOUNT_TCP;
				break;
			case Opt_tcp:
				nfs_data.flags |= NFS_MOUNT_TCP;
				break;
			case Opt_acl:
				nfs_data.flags &= ~NFS_MOUNT_NOACL;
				break;
			case Opt_noacl:
				nfs_data.flags |= NFS_MOUNT_NOACL;
				break;
			default:
				printk(KERN_WARNING "Root-NFS: unknown "
					"option: %s\n", p);
				return 0;
		}
	}

	return 1;
}

static int __init root_nfs_name(char *name)
{
	static char buf[NFS_MAXPATHLEN] __initdata;
	char *cp;

	/* Set some default values */
	memset(&nfs_data, 0, sizeof(nfs_data));
	nfs_port          = -1;
	nfs_data.version  = NFS_MOUNT_VERSION;
	nfs_data.flags    = NFS_MOUNT_NONLM;	/* No lockd in nfs root yet */
	nfs_data.rsize    = NFS_DEF_FILE_IO_SIZE;
	nfs_data.wsize    = NFS_DEF_FILE_IO_SIZE;
	nfs_data.acregmin = NFS_DEF_ACREGMIN;
	nfs_data.acregmax = NFS_DEF_ACREGMAX;
	nfs_data.acdirmin = NFS_DEF_ACDIRMIN;
	nfs_data.acdirmax = NFS_DEF_ACDIRMAX;
	strcpy(buf, NFS_ROOT);

	/* Process options received from the remote server */
	root_nfs_parse(root_server_path, buf);

	/* Override them by options set on kernel command-line */
	root_nfs_parse(name, buf);

	cp = utsname()->nodename;
	if (strlen(buf) + strlen(cp) > NFS_MAXPATHLEN) {
		printk(KERN_ERR "Root-NFS: Pathname for remote directory too long.\n");
		return -1;
	}
	sprintf(nfs_export_path, buf, cp);

	return 1;
}


static int __init root_nfs_addr(void)
{
	if ((servaddr = root_server_addr) == htonl(INADDR_NONE)) {
		printk(KERN_ERR "Root-NFS: No NFS server available, giving up.\n");
		return -1;
	}

	snprintf(nfs_data.hostname, sizeof(nfs_data.hostname),
		 "%pI4", &servaddr);
	return 0;
}

#ifdef NFSROOT_DEBUG
static void __init root_nfs_print(void)
{
	printk(KERN_NOTICE "Root-NFS: Mounting %s on server %s as root\n",
		nfs_export_path, nfs_data.hostname);
	printk(KERN_NOTICE "Root-NFS:     rsize = %d, wsize = %d, timeo = %d, retrans = %d\n",
		nfs_data.rsize, nfs_data.wsize, nfs_data.timeo, nfs_data.retrans);
	printk(KERN_NOTICE "Root-NFS:     acreg (min,max) = (%d,%d), acdir (min,max) = (%d,%d)\n",
		nfs_data.acregmin, nfs_data.acregmax,
		nfs_data.acdirmin, nfs_data.acdirmax);
	printk(KERN_NOTICE "Root-NFS:     nfsd port = %d, mountd port = %d, flags = %08x\n",
		nfs_port, mount_port, nfs_data.flags);
}
#endif


static int __init root_nfs_init(void)
{
#ifdef NFSROOT_DEBUG
	nfs_debug |= NFSDBG_ROOT;
#endif

	/*
	 * Decode the root directory path name and NFS options from
	 * the kernel command line. This has to go here in order to
	 * be able to use the client IP address for the remote root
	 * directory (necessary for pure RARP booting).
	 */
	if (root_nfs_name(nfs_root_name) < 0 ||
	    root_nfs_addr() < 0)
		return -1;

#ifdef NFSROOT_DEBUG
	root_nfs_print();
#endif

	return 0;
}


static int __init nfs_root_setup(char *line)
{
	ROOT_DEV = Root_NFS;
	if (line[0] == '/' || line[0] == ',' || (line[0] >= '0' && line[0] <= '9')) {
		strlcpy(nfs_root_name, line, sizeof(nfs_root_name));
	} else {
		int n = strlen(line) + sizeof(NFS_ROOT) - 1;
		if (n >= sizeof(nfs_root_name))
			line[sizeof(nfs_root_name) - sizeof(NFS_ROOT) - 2] = '\0';
		sprintf(nfs_root_name, NFS_ROOT, line);
	}
	root_server_addr = root_nfs_parse_addr(nfs_root_name);
	return 1;
}

__setup("nfsroot=", nfs_root_setup);


static inline void
set_sockaddr(struct sockaddr_in *sin, __be32 addr, __be16 port)
{
	sin->sin_family = AF_INET;
	sin->sin_addr.s_addr = addr;
	sin->sin_port = port;
}

static int __init root_nfs_getport(int program, int version, int proto)
{
	struct sockaddr_in sin;

	printk(KERN_NOTICE "Looking up port of RPC %d/%d on %pI4\n",
		program, version, &servaddr);
	set_sockaddr(&sin, servaddr, 0);
	return rpcb_getport_sync(&sin, program, version, proto);
}


static int __init root_nfs_ports(void)
{
	int port;
	int nfsd_ver, mountd_ver;
	int nfsd_port, mountd_port;
	int proto;

	if (nfs_data.flags & NFS_MOUNT_VER3) {
		nfsd_ver = NFS3_VERSION;
		mountd_ver = NFS_MNT3_VERSION;
		nfsd_port = NFS_PORT;
		mountd_port = NFS_MNT_PORT;
	} else {
		nfsd_ver = NFS2_VERSION;
		mountd_ver = NFS_MNT_VERSION;
		nfsd_port = NFS_PORT;
		mountd_port = NFS_MNT_PORT;
	}

	proto = (nfs_data.flags & NFS_MOUNT_TCP) ? IPPROTO_TCP : IPPROTO_UDP;

	if (nfs_port < 0) {
		if ((port = root_nfs_getport(NFS_PROGRAM, nfsd_ver, proto)) < 0) {
			printk(KERN_ERR "Root-NFS: Unable to get nfsd port "
					"number from server, using default\n");
			port = nfsd_port;
		}
		nfs_port = port;
		dprintk("Root-NFS: Portmapper on server returned %d "
			"as nfsd port\n", port);
	}

	if ((port = root_nfs_getport(NFS_MNT_PROGRAM, mountd_ver, proto)) < 0) {
		printk(KERN_ERR "Root-NFS: Unable to get mountd port "
				"number from server, using default\n");
		port = mountd_port;
	}
	mount_port = port;
	dprintk("Root-NFS: mountd port is %d\n", port);

	return 0;
}


static int __init root_nfs_get_handle(void)
{
	struct sockaddr_in sin;
	unsigned int auth_flav_len = 0;
	struct nfs_mount_request request = {
		.sap		= (struct sockaddr *)&sin,
		.salen		= sizeof(sin),
		.dirpath	= nfs_export_path,
		.version	= (nfs_data.flags & NFS_MOUNT_VER3) ?
					NFS_MNT3_VERSION : NFS_MNT_VERSION,
		.protocol	= (nfs_data.flags & NFS_MOUNT_TCP) ?
					XPRT_TRANSPORT_TCP : XPRT_TRANSPORT_UDP,
		.auth_flav_len	= &auth_flav_len,
	};
	int status = -ENOMEM;

	request.fh = nfs_alloc_fhandle();
	if (!request.fh)
		goto out;
	set_sockaddr(&sin, servaddr, htons(mount_port));
	status = nfs_mount(&request);
	if (status < 0)
		printk(KERN_ERR "Root-NFS: Server returned error %d "
				"while mounting %s\n", status, nfs_export_path);
	else {
		nfs_data.root.size = request.fh->size;
		memcpy(&nfs_data.root.data, request.fh->data, request.fh->size);
	}
	nfs_free_fhandle(request.fh);
out:
	return status;
}

void * __init nfs_root_data(void)
{
	if (root_nfs_init() < 0
	 || root_nfs_ports() < 0
	 || root_nfs_get_handle() < 0)
		return NULL;
	set_sockaddr((struct sockaddr_in *) &nfs_data.addr, servaddr, htons(nfs_port));
	return (void*)&nfs_data;
}
