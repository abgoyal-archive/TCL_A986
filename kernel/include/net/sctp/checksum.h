

#include <linux/types.h>
#include <net/sctp/sctp.h>
#include <linux/crc32c.h>

static inline __u32 sctp_crc32c(__u32 crc, u8 *buffer, u16 length)
{
	return crc32c(crc, buffer, length);
}

static inline __u32 sctp_start_cksum(__u8 *buffer, __u16 length)
{
	__u32 crc = ~(__u32)0;
	__u8  zero[sizeof(__u32)] = {0};

	/* Optimize this routine to be SCTP specific, knowing how
	 * to skip the checksum field of the SCTP header.
	 */

	/* Calculate CRC up to the checksum. */
	crc = sctp_crc32c(crc, buffer, sizeof(struct sctphdr) - sizeof(__u32));

	/* Skip checksum field of the header. */
	crc = sctp_crc32c(crc, zero, sizeof(__u32));

	/* Calculate the rest of the CRC. */
	crc = sctp_crc32c(crc, &buffer[sizeof(struct sctphdr)],
			    length - sizeof(struct sctphdr));
	return crc;
}

static inline __u32 sctp_update_cksum(__u8 *buffer, __u16 length, __u32 crc32)
{
	return sctp_crc32c(crc32, buffer, length);
}

static inline __le32 sctp_end_cksum(__be32 crc32)
{
	return cpu_to_le32(~crc32);
}
