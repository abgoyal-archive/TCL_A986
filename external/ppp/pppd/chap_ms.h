

#ifndef __CHAPMS_INCLUDE__

#define MD4_SIGNATURE_SIZE	16	/* 16 bytes in a MD4 message digest */
#define MAX_NT_PASSWORD		256	/* Max (Unicode) chars in an NT pass */

#define MS_CHAP_RESPONSE_LEN	49	/* Response length for MS-CHAP */
#define MS_CHAP2_RESPONSE_LEN	49	/* Response length for MS-CHAPv2 */
#define MS_AUTH_RESPONSE_LENGTH	40	/* MS-CHAPv2 authenticator response, */
					/* as ASCII */

/* E=eeeeeeeeee error codes for MS-CHAP failure messages. */
#define MS_CHAP_ERROR_RESTRICTED_LOGON_HOURS	646
#define MS_CHAP_ERROR_ACCT_DISABLED		647
#define MS_CHAP_ERROR_PASSWD_EXPIRED		648
#define MS_CHAP_ERROR_NO_DIALIN_PERMISSION	649
#define MS_CHAP_ERROR_AUTHENTICATION_FAILURE	691
#define MS_CHAP_ERROR_CHANGING_PASSWORD		709

#ifdef __GNUC__
#define PACKED	__attribute__((__packed__))
#else
#define PACKED
#endif

typedef struct {
    u_char LANManResp[24];
    u_char NTResp[24];
    u_char UseNT[1];		/* If 1, ignore the LANMan response field */
} MS_ChapResponse PACKED;

typedef struct {
    u_char PeerChallenge[16];
    u_char Reserved[8];		/* Must be zero */
    u_char NTResp[24];
    u_char Flags[1];		/* Must be zero */
} MS_Chap2Response PACKED;

#ifdef MPPE
#include <net/ppp-comp.h>	/* MPPE_MAX_KEY_LEN */
extern u_char mppe_send_key[MPPE_MAX_KEY_LEN];
extern u_char mppe_recv_key[MPPE_MAX_KEY_LEN];
extern int mppe_keys_set;

/* These values are the RADIUS attribute values--see RFC 2548. */
#define MPPE_ENC_POL_ENC_ALLOWED 1
#define MPPE_ENC_POL_ENC_REQUIRED 2
#define MPPE_ENC_TYPES_RC4_40 2
#define MPPE_ENC_TYPES_RC4_128 4

/* used by plugins (using above values) */
extern void set_mppe_enc_types(int, int);
#endif

/* Are we the authenticator or authenticatee?  For MS-CHAPv2 key derivation. */
#define MS_CHAP2_AUTHENTICATEE 0
#define MS_CHAP2_AUTHENTICATOR 1

void ChapMS __P((u_char *, char *, int, MS_ChapResponse *));
void ChapMS2 __P((u_char *, u_char *, char *, char *, int,
		  MS_Chap2Response *, u_char[MS_AUTH_RESPONSE_LENGTH+1], int));
#ifdef MPPE
void mppe_set_keys __P((u_char *, u_char[MD4_SIGNATURE_SIZE]));
void mppe_set_keys2(u_char PasswordHashHash[MD4_SIGNATURE_SIZE],
		    u_char NTResponse[24], int IsServer);
#endif

void	ChallengeHash __P((u_char[16], u_char *, char *, u_char[8]));

void GenerateAuthenticatorResponse(u_char PasswordHashHash[MD4_SIGNATURE_SIZE],
			u_char NTResponse[24], u_char PeerChallenge[16],
			u_char *rchallenge, char *username,
			u_char authResponse[MS_AUTH_RESPONSE_LENGTH+1]);

void chapms_init(void);

#define __CHAPMS_INCLUDE__
#endif /* __CHAPMS_INCLUDE__ */
