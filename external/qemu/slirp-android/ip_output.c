


#include <slirp.h>

u_int16_t ip_id;

#define IF_THRESH 10

int
ip_output(struct socket *so, struct mbuf *m0)
{
	register struct ip *ip;
	register struct mbuf *m = m0;
	register int hlen = sizeof(struct ip );
	int len, off, error = 0;

	DEBUG_CALL("ip_output");
	DEBUG_ARG("so = %lx", (long)so);
	DEBUG_ARG("m0 = %lx", (long)m0);

	/* We do no options */
	ip = mtod(m, struct ip *);
	/*
	 * Fill in IP header.
	 */
	ip->ip_v = IPVERSION;
	ip->ip_off &= IP_DF;
	ip->ip_id = htons(ip_id++);
	ip->ip_hl = hlen >> 2;
	STAT(ipstat.ips_localout++);

	/*
	 * Verify that we have any chance at all of being able to queue
	 *      the packet or packet fragments
	 */
	/* XXX Hmmm... */

	/*
	 * If small enough for interface, can just send directly.
	 */
	if ((u_int16_t)ip->ip_len <= IF_MTU) {
		ip->ip_len = htons((u_int16_t)ip->ip_len);
		ip->ip_off = htons((u_int16_t)ip->ip_off);
		ip->ip_sum = 0;
		ip->ip_sum = cksum(m, hlen);

		if_output(so, m);
		goto done;
	}

	/*
	 * Too large for interface; fragment if possible.
	 * Must be able to put at least 8 bytes per fragment.
	 */
	if (ip->ip_off & IP_DF) {
		error = -1;
		STAT(ipstat.ips_cantfrag++);
		goto bad;
	}

	len = (IF_MTU - hlen) &~ 7;       /* ip databytes per packet */
	if (len < 8) {
		error = -1;
		goto bad;
	}

    {
	int mhlen, firstlen = len;
	struct mbuf **mnext = &m->m_nextpkt;

	/*
	 * Loop through length of segment after first fragment,
	 * make new header and copy data of each part and link onto chain.
	 */
	m0 = m;
	mhlen = sizeof (struct ip);
	for (off = hlen + len; off < (u_int16_t)ip->ip_len; off += len) {
	  register struct ip *mhip;
	  m = m_get();
          if (m == NULL) {
	    error = -1;
	    STAT(ipstat.ips_odropped++);
	    goto sendorfree;
	  }
	  m->m_data += IF_MAXLINKHDR;
	  mhip = mtod(m, struct ip *);
	  *mhip = *ip;

		/* No options */
	  m->m_len = mhlen;
	  mhip->ip_off = ((off - hlen) >> 3) + (ip->ip_off & ~IP_MF);
	  if (ip->ip_off & IP_MF)
	    mhip->ip_off |= IP_MF;
	  if (off + len >= (u_int16_t)ip->ip_len)
	    len = (u_int16_t)ip->ip_len - off;
	  else
	    mhip->ip_off |= IP_MF;
	  mhip->ip_len = htons((u_int16_t)(len + mhlen));

	  if (m_copy(m, m0, off, len) < 0) {
	    error = -1;
	    goto sendorfree;
	  }

	  mhip->ip_off = htons((u_int16_t)mhip->ip_off);
	  mhip->ip_sum = 0;
	  mhip->ip_sum = cksum(m, mhlen);
	  *mnext = m;
	  mnext = &m->m_nextpkt;
	  STAT(ipstat.ips_ofragments++);
	}
	/*
	 * Update first fragment by trimming what's been copied out
	 * and updating header, then send each fragment (in order).
	 */
	m = m0;
	m_adj(m, hlen + firstlen - (u_int16_t)ip->ip_len);
	ip->ip_len = htons((u_int16_t)m->m_len);
	ip->ip_off = htons((u_int16_t)(ip->ip_off | IP_MF));
	ip->ip_sum = 0;
	ip->ip_sum = cksum(m, hlen);
sendorfree:
	for (m = m0; m; m = m0) {
		m0 = m->m_nextpkt;
                m->m_nextpkt = NULL;
		if (error == 0)
			if_output(so, m);
		else
			m_freem(m);
	}

	if (error == 0)
		STAT(ipstat.ips_fragmented++);
    }

done:
	return (error);

bad:
	m_freem(m0);
	goto done;
}
