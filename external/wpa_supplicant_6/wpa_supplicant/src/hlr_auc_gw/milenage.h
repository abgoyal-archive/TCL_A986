

#ifndef MILENAGE_H
#define MILENAGE_H

void milenage_generate(const u8 *opc, const u8 *amf, const u8 *k,
		       const u8 *sqn, const u8 *_rand, u8 *autn, u8 *ik,
		       u8 *ck, u8 *res, size_t *res_len);
int milenage_auts(const u8 *opc, const u8 *k, const u8 *_rand, const u8 *auts,
		  u8 *sqn);
int gsm_milenage(const u8 *opc, const u8 *k, const u8 *_rand, u8 *sres,
		 u8 *kc);
int milenage_check(const u8 *opc, const u8 *k, const u8 *sqn, const u8 *_rand,
		   const u8 *autn, u8 *ik, u8 *ck, u8 *res, size_t *res_len,
		   u8 *auts);

#endif /* MILENAGE_H */
