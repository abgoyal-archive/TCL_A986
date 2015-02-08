

#include <includes.h>
#include <radiusclient.h>


UINT4 rc_get_ipaddr (char *host)
{
	struct hostent *hp;

	if (rc_good_ipaddr (host) == 0)
	{
		return ntohl(inet_addr (host));
	}
	else if ((hp = gethostbyname (host)) == (struct hostent *) NULL)
	{
		error("rc_get_ipaddr: couldn't resolve hostname: %s", host);
		return ((UINT4) 0);
	}
	return ntohl((*(UINT4 *) hp->h_addr));
}


int rc_good_ipaddr (char *addr)
{
	int             dot_count;
	int             digit_count;

	if (addr == NULL)
		return (-1);

	dot_count = 0;
	digit_count = 0;
	while (*addr != '\0' && *addr != ' ')
	{
		if (*addr == '.')
		{
			dot_count++;
			digit_count = 0;
		}
		else if (!isdigit (*addr))
		{
			dot_count = 5;
		}
		else
		{
			digit_count++;
			if (digit_count > 3)
			{
				dot_count = 5;
			}
		}
		addr++;
	}
	if (dot_count != 3)
	{
		return (-1);
	}
	else
	{
		return (0);
	}
}


const char *rc_ip_hostname (UINT4 h_ipaddr)
{
	struct hostent  *hp;
	UINT4           n_ipaddr = htonl (h_ipaddr);

	if ((hp = gethostbyaddr ((char *) &n_ipaddr, sizeof (struct in_addr),
			    AF_INET)) == NULL) {
		error("rc_ip_hostname: couldn't look up host by addr: %08lX", h_ipaddr);
	}

	return ((hp==NULL)?"unknown":hp->h_name);
}


UINT4 rc_own_ipaddress(void)
{
	static UINT4 this_host_ipaddr = 0;

	if (!this_host_ipaddr) {
		if ((this_host_ipaddr = rc_get_ipaddr (hostname)) == 0) {
			error("rc_own_ipaddress: couldn't get own IP address");
			return 0;
		}
	}

	return this_host_ipaddr;
}
