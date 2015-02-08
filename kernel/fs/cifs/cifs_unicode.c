
#include <linux/fs.h>
#include <linux/slab.h>
#include "cifs_unicode.h"
#include "cifs_uniupr.h"
#include "cifspdu.h"
#include "cifsglob.h"
#include "cifs_debug.h"

int
cifs_ucs2_bytes(const __le16 *from, int maxbytes,
		const struct nls_table *codepage)
{
	int i;
	int charlen, outlen = 0;
	int maxwords = maxbytes / 2;
	char tmp[NLS_MAX_CHARSET_SIZE];

	for (i = 0; i < maxwords && from[i]; i++) {
		charlen = codepage->uni2char(le16_to_cpu(from[i]), tmp,
					     NLS_MAX_CHARSET_SIZE);
		if (charlen > 0)
			outlen += charlen;
		else
			outlen++;
	}

	return outlen;
}

static int
cifs_mapchar(char *target, const __le16 src_char, const struct nls_table *cp,
	     bool mapchar)
{
	int len = 1;

	if (!mapchar)
		goto cp_convert;

	/*
	 * BB: Cannot handle remapping UNI_SLASH until all the calls to
	 *     build_path_from_dentry are modified, as they use slash as
	 *     separator.
	 */
	switch (le16_to_cpu(src_char)) {
	case UNI_COLON:
		*target = ':';
		break;
	case UNI_ASTERIK:
		*target = '*';
		break;
	case UNI_QUESTION:
		*target = '?';
		break;
	case UNI_PIPE:
		*target = '|';
		break;
	case UNI_GRTRTHAN:
		*target = '>';
		break;
	case UNI_LESSTHAN:
		*target = '<';
		break;
	default:
		goto cp_convert;
	}

out:
	return len;

cp_convert:
	len = cp->uni2char(le16_to_cpu(src_char), target,
			   NLS_MAX_CHARSET_SIZE);
	if (len <= 0) {
		*target = '?';
		len = 1;
	}
	goto out;
}

int
cifs_from_ucs2(char *to, const __le16 *from, int tolen, int fromlen,
		 const struct nls_table *codepage, bool mapchar)
{
	int i, charlen, safelen;
	int outlen = 0;
	int nullsize = nls_nullsize(codepage);
	int fromwords = fromlen / 2;
	char tmp[NLS_MAX_CHARSET_SIZE];

	/*
	 * because the chars can be of varying widths, we need to take care
	 * not to overflow the destination buffer when we get close to the
	 * end of it. Until we get to this offset, we don't need to check
	 * for overflow however.
	 */
	safelen = tolen - (NLS_MAX_CHARSET_SIZE + nullsize);

	for (i = 0; i < fromwords && from[i]; i++) {
		/*
		 * check to see if converting this character might make the
		 * conversion bleed into the null terminator
		 */
		if (outlen >= safelen) {
			charlen = cifs_mapchar(tmp, from[i], codepage, mapchar);
			if ((outlen + charlen) > (tolen - nullsize))
				break;
		}

		/* put converted char into 'to' buffer */
		charlen = cifs_mapchar(&to[outlen], from[i], codepage, mapchar);
		outlen += charlen;
	}

	/* properly null-terminate string */
	for (i = 0; i < nullsize; i++)
		to[outlen++] = 0;

	return outlen;
}

int
cifs_strtoUCS(__le16 *to, const char *from, int len,
	      const struct nls_table *codepage)
{
	int charlen;
	int i;
	wchar_t *wchar_to = (wchar_t *)to; /* needed to quiet sparse */

	for (i = 0; len && *from; i++, from += charlen, len -= charlen) {

		/* works for 2.4.0 kernel or later */
		charlen = codepage->char2uni(from, len, &wchar_to[i]);
		if (charlen < 1) {
			cERROR(1, "strtoUCS: char2uni of %d returned %d",
				(int)*from, charlen);
			/* A question mark */
			to[i] = cpu_to_le16(0x003f);
			charlen = 1;
		} else
			to[i] = cpu_to_le16(wchar_to[i]);

	}

	to[i] = 0;
	return i;
}

char *
cifs_strndup_from_ucs(const char *src, const int maxlen, const bool is_unicode,
	     const struct nls_table *codepage)
{
	int len;
	char *dst;

	if (is_unicode) {
		len = cifs_ucs2_bytes((__le16 *) src, maxlen, codepage);
		len += nls_nullsize(codepage);
		dst = kmalloc(len, GFP_KERNEL);
		if (!dst)
			return NULL;
		cifs_from_ucs2(dst, (__le16 *) src, len, maxlen, codepage,
			       false);
	} else {
		len = strnlen(src, maxlen);
		len++;
		dst = kmalloc(len, GFP_KERNEL);
		if (!dst)
			return NULL;
		strlcpy(dst, src, len);
	}

	return dst;
}

