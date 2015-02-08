

#ifndef DH_GROUPS_H
#define DH_GROUPS_H

struct dh_group {
	int id;
	const u8 *generator;
	size_t generator_len;
	const u8 *prime;
	size_t prime_len;
};

const struct dh_group * dh_groups_get(int id);
struct wpabuf * dh_init(const struct dh_group *dh, struct wpabuf **priv);
struct wpabuf * dh_derive_shared(const struct wpabuf *peer_public,
				 const struct wpabuf *own_private,
				 const struct dh_group *dh);

#endif /* DH_GROUPS_H */
