

#ifndef TLS_H
#define TLS_H

struct tls_connection;

struct tls_keys {
	const u8 *master_key; /* TLS master secret */
	size_t master_key_len;
	const u8 *client_random;
	size_t client_random_len;
	const u8 *server_random;
	size_t server_random_len;
	const u8 *inner_secret; /* TLS/IA inner secret */
	size_t inner_secret_len;
};

struct tls_config {
	const char *opensc_engine_path;
	const char *pkcs11_engine_path;
	const char *pkcs11_module_path;
};

#define TLS_CONN_ALLOW_SIGN_RSA_MD5 BIT(0)
#define TLS_CONN_DISABLE_TIME_CHECKS BIT(1)

struct tls_connection_params {
	const char *ca_cert;
	const u8 *ca_cert_blob;
	size_t ca_cert_blob_len;
	const char *ca_path;
	const char *subject_match;
	const char *altsubject_match;
	const char *client_cert;
	const u8 *client_cert_blob;
	size_t client_cert_blob_len;
	const char *private_key;
	const u8 *private_key_blob;
	size_t private_key_blob_len;
	const char *private_key_passwd;
	const char *dh_file;
	const u8 *dh_blob;
	size_t dh_blob_len;
	int tls_ia;

	/* OpenSSL specific variables */
	int engine;
	const char *engine_id;
	const char *pin;
	const char *key_id;
	const char *cert_id;
	const char *ca_cert_id;

	unsigned int flags;
};


void * tls_init(const struct tls_config *conf);

void tls_deinit(void *tls_ctx);

int tls_get_errors(void *tls_ctx);

struct tls_connection * tls_connection_init(void *tls_ctx);

void tls_connection_deinit(void *tls_ctx, struct tls_connection *conn);

int tls_connection_established(void *tls_ctx, struct tls_connection *conn);

int tls_connection_shutdown(void *tls_ctx, struct tls_connection *conn);

enum {
	TLS_SET_PARAMS_ENGINE_PRV_VERIFY_FAILED = -3,
	TLS_SET_PARAMS_ENGINE_PRV_INIT_FAILED = -2
};

int __must_check
tls_connection_set_params(void *tls_ctx, struct tls_connection *conn,
			  const struct tls_connection_params *params);

int __must_check tls_global_set_params(
	void *tls_ctx, const struct tls_connection_params *params);

int __must_check tls_global_set_verify(void *tls_ctx, int check_crl);

int __must_check tls_connection_set_verify(void *tls_ctx,
					   struct tls_connection *conn,
					   int verify_peer);

int __must_check tls_connection_set_ia(void *tls_ctx,
				       struct tls_connection *conn,
				       int tls_ia);

int __must_check tls_connection_get_keys(void *tls_ctx,
					 struct tls_connection *conn,
					 struct tls_keys *keys);

int __must_check  tls_connection_prf(void *tls_ctx,
				     struct tls_connection *conn,
				     const char *label,
				     int server_random_first,
				     u8 *out, size_t out_len);

u8 * tls_connection_handshake(void *tls_ctx, struct tls_connection *conn,
			      const u8 *in_data, size_t in_len,
			      size_t *out_len, u8 **appl_data,
			      size_t *appl_data_len);

u8 * tls_connection_server_handshake(void *tls_ctx,
				     struct tls_connection *conn,
				     const u8 *in_data, size_t in_len,
				     size_t *out_len);

int __must_check tls_connection_encrypt(void *tls_ctx,
					struct tls_connection *conn,
					const u8 *in_data, size_t in_len,
					u8 *out_data, size_t out_len);

int __must_check tls_connection_decrypt(void *tls_ctx,
					struct tls_connection *conn,
					const u8 *in_data, size_t in_len,
					u8 *out_data, size_t out_len);

int tls_connection_resumed(void *tls_ctx, struct tls_connection *conn);

enum {
	TLS_CIPHER_NONE,
	TLS_CIPHER_RC4_SHA /* 0x0005 */,
	TLS_CIPHER_AES128_SHA /* 0x002f */,
	TLS_CIPHER_RSA_DHE_AES128_SHA /* 0x0031 */,
	TLS_CIPHER_ANON_DH_AES128_SHA /* 0x0034 */
};

int __must_check tls_connection_set_cipher_list(void *tls_ctx,
						struct tls_connection *conn,
						u8 *ciphers);

int __must_check tls_get_cipher(void *tls_ctx, struct tls_connection *conn,
				char *buf, size_t buflen);

int __must_check tls_connection_enable_workaround(void *tls_ctx,
						  struct tls_connection *conn);

int __must_check tls_connection_client_hello_ext(void *tls_ctx,
						 struct tls_connection *conn,
						 int ext_type, const u8 *data,
						 size_t data_len);

int tls_connection_get_failed(void *tls_ctx, struct tls_connection *conn);

int tls_connection_get_read_alerts(void *tls_ctx, struct tls_connection *conn);

int tls_connection_get_write_alerts(void *tls_ctx,
				    struct tls_connection *conn);

int tls_connection_get_keyblock_size(void *tls_ctx,
				     struct tls_connection *conn);

#define TLS_CAPABILITY_IA 0x0001 /* TLS Inner Application (TLS/IA) */
unsigned int tls_capabilities(void *tls_ctx);

int __must_check tls_connection_ia_send_phase_finished(
	void *tls_ctx, struct tls_connection *conn, int final,
	u8 *out_data, size_t out_len);

int __must_check tls_connection_ia_final_phase_finished(
	void *tls_ctx, struct tls_connection *conn);

int __must_check tls_connection_ia_permute_inner_secret(
	void *tls_ctx, struct tls_connection *conn,
	const u8 *key, size_t key_len);

typedef int (*tls_session_ticket_cb)
(void *ctx, const u8 *ticket, size_t len, const u8 *client_random,
 const u8 *server_random, u8 *master_secret);

int __must_check  tls_connection_set_session_ticket_cb(
	void *tls_ctx, struct tls_connection *conn,
	tls_session_ticket_cb cb, void *ctx);

#endif /* TLS_H */
