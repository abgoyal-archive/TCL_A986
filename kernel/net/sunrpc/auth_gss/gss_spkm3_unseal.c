

#include <linux/types.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/sunrpc/gss_spkm3.h>
#include <linux/crypto.h>

#ifdef RPC_DEBUG
# define RPCDBG_FACILITY        RPCDBG_AUTH
#endif

u32
spkm3_read_token(struct spkm3_ctx *ctx,
		struct xdr_netobj *read_token,    /* checksum */
		struct xdr_buf *message_buffer, /* signbuf */
		int toktype)
{
	s32			checksum_type;
	s32			code;
	struct xdr_netobj	wire_cksum = {.len =0, .data = NULL};
	char			cksumdata[16];
	struct xdr_netobj	md5cksum = {.len = 0, .data = cksumdata};
	unsigned char		*ptr = (unsigned char *)read_token->data;
	unsigned char		*cksum;
	int			bodysize, md5elen;
	int			mic_hdrlen;
	u32			ret = GSS_S_DEFECTIVE_TOKEN;

	if (g_verify_token_header((struct xdr_netobj *) &ctx->mech_used,
					&bodysize, &ptr, read_token->len))
		goto out;

	/* decode the token */

	if (toktype != SPKM_MIC_TOK) {
		dprintk("RPC:       BAD SPKM3 token type: %d\n", toktype);
		goto out;
	}

	if ((ret = spkm3_verify_mic_token(&ptr, &mic_hdrlen, &cksum)))
		goto out;

	if (*cksum++ != 0x03) {
		dprintk("RPC:       spkm3_read_token BAD checksum type\n");
		goto out;
	}
	md5elen = *cksum++;
	cksum++; 	/* move past the zbit */

	if (!decode_asn1_bitstring(&wire_cksum, cksum, md5elen - 1, 16))
		goto out;

	/* HARD CODED FOR MD5 */

	/* compute the checksum of the message.
	 * ptr + 2 = start of header piece of checksum
	 * mic_hdrlen + 2 = length of header piece of checksum
	 */
	ret = GSS_S_DEFECTIVE_TOKEN;
	if (!g_OID_equal(&ctx->intg_alg, &hmac_md5_oid)) {
		dprintk("RPC:       gss_spkm3_seal: unsupported I-ALG "
				"algorithm\n");
		goto out;
	}

	checksum_type = CKSUMTYPE_HMAC_MD5;

	code = make_spkm3_checksum(checksum_type,
		&ctx->derived_integ_key, ptr + 2, mic_hdrlen + 2,
		message_buffer, 0, &md5cksum);

	if (code)
		goto out;

	ret = GSS_S_BAD_SIG;
	code = memcmp(md5cksum.data, wire_cksum.data, wire_cksum.len);
	if (code) {
		dprintk("RPC:       bad MIC checksum\n");
		goto out;
	}


	/* XXX: need to add expiration and sequencing */
	ret = GSS_S_COMPLETE;
out:
	kfree(wire_cksum.data);
	return ret;
}
