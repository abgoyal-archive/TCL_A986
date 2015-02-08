

#include <linux/types.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/sunrpc/gss_spkm3.h>
#include <linux/random.h>
#include <linux/crypto.h>

#ifdef RPC_DEBUG
# define RPCDBG_FACILITY        RPCDBG_AUTH
#endif

void
asn1_bitstring_len(struct xdr_netobj *in, int *enclen, int *zerobits)
{
	int i, zbit = 0,elen = in->len;
	char *ptr;

	ptr = &in->data[in->len -1];

	/* count trailing 0's */
	for(i = in->len; i > 0; i--) {
		if (*ptr == 0) {
			ptr--;
			elen--;
		} else
			break;
	}

	/* count number of 0 bits in final octet */
	ptr = &in->data[elen - 1];
	for(i = 0; i < 8; i++) {
		short mask = 0x01;

		if (!((mask << i) & *ptr))
			zbit++;
		else
			break;
	}
	*enclen = elen;
	*zerobits = zbit;
}

int
decode_asn1_bitstring(struct xdr_netobj *out, char *in, int enclen, int explen)
{
	if (!(out->data = kzalloc(explen,GFP_NOFS)))
		return 0;
	out->len = explen;
	memcpy(out->data, in, enclen);
	return 1;
}


void
spkm3_mic_header(unsigned char **hdrbuf, unsigned int *hdrlen, unsigned char *ctxdata, int elen, int zbit)
{
	char *hptr = *hdrbuf;
	char *top = *hdrbuf;

	*(u8 *)hptr++ = 0x30;
	*(u8 *)hptr++ = elen + 7;  /* on the wire header length */

	/* tokenid */
	*(u8 *)hptr++ = 0x02;
	*(u8 *)hptr++ = 0x02;
	*(u8 *)hptr++ = 0x01;
	*(u8 *)hptr++ = 0x01;

	/* coniextid */
	*(u8 *)hptr++ = 0x03;
	*(u8 *)hptr++ = elen + 1; /* add 1 to include zbit */
	*(u8 *)hptr++ = zbit;
	memcpy(hptr, ctxdata, elen);
	hptr += elen;
	*hdrlen = hptr - top;
}

void
spkm3_make_mic_token(unsigned char **tokp, int toklen, struct xdr_netobj *mic_hdr, struct xdr_netobj *md5cksum, int md5elen, int md5zbit)
{
	unsigned char *ict = *tokp;

	*(u8 *)ict++ = 0xa4;
	*(u8 *)ict++ = toklen;
	memcpy(ict, mic_hdr->data, mic_hdr->len);
	ict += mic_hdr->len;

	*(u8 *)ict++ = 0x03;
	*(u8 *)ict++ = md5elen + 1; /* add 1 to include zbit */
	*(u8 *)ict++ = md5zbit;
	memcpy(ict, md5cksum->data, md5elen);
}

u32
spkm3_verify_mic_token(unsigned char **tokp, int *mic_hdrlen, unsigned char **cksum)
{
	struct xdr_netobj       spkm3_ctx_id = {.len =0, .data = NULL};
	unsigned char 		*ptr = *tokp;
	int 			ctxelen;
	u32     		ret = GSS_S_DEFECTIVE_TOKEN;

	/* spkm3 innercontext token preamble */
	if ((ptr[0] != 0xa4) || (ptr[2] != 0x30)) {
		dprintk("RPC:       BAD SPKM ictoken preamble\n");
		goto out;
	}

	*mic_hdrlen = ptr[3];

	/* token type */
	if ((ptr[4] != 0x02) || (ptr[5] != 0x02)) {
		dprintk("RPC:       BAD asn1 SPKM3 token type\n");
		goto out;
	}

	/* only support SPKM_MIC_TOK */
	if((ptr[6] != 0x01) || (ptr[7] != 0x01)) {
		dprintk("RPC:       ERROR unsupported SPKM3 token\n");
		goto out;
	}

	/* contextid */
	if (ptr[8] != 0x03) {
		dprintk("RPC:       BAD SPKM3 asn1 context-id type\n");
		goto out;
	}

	ctxelen = ptr[9];
	if (ctxelen > 17) {  /* length includes asn1 zbit octet */
		dprintk("RPC:       BAD SPKM3 contextid len %d\n", ctxelen);
		goto out;
	}

	/* ignore ptr[10] */

	if(!decode_asn1_bitstring(&spkm3_ctx_id, &ptr[11], ctxelen - 1, 16))
		goto out;

	/*
	* in the current implementation: the optional int-alg is not present
	* so the default int-alg (md5) is used the optional snd-seq field is
	* also not present
	*/

	if (*mic_hdrlen != 6 + ctxelen) {
		dprintk("RPC:       BAD SPKM_ MIC_TOK header len %d: we only "
				"support default int-alg (should be absent) "
				"and do not support snd-seq\n", *mic_hdrlen);
		goto out;
	}
	/* checksum */
	*cksum = (&ptr[10] + ctxelen); /* ctxelen includes ptr[10] */

	ret = GSS_S_COMPLETE;
out:
	kfree(spkm3_ctx_id.data);
	return ret;
}

