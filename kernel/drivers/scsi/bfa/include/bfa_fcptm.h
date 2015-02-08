

#ifndef __BFA_FCPTM_H__
#define __BFA_FCPTM_H__

#include <bfa.h>
#include <bfa_svc.h>
#include <bfi/bfi_fcptm.h>

struct bfa_tin_s;
struct bfa_iotm_s;
struct bfa_tsktm_s;

void bfa_fcptm_path_tov_set(struct bfa_s *bfa, u16 path_tov);
u16 bfa_fcptm_path_tov_get(struct bfa_s *bfa);
void bfa_fcptm_qdepth_set(struct bfa_s *bfa, u16 q_depth);
u16 bfa_fcptm_qdepth_get(struct bfa_s *bfa);

void bfa_tin_get_stats(struct bfa_tin_s *tin, struct bfa_tin_stats_s *stats);
void bfa_tin_clear_stats(struct bfa_tin_s *tin);

#endif /* __BFA_FCPTM_H__ */

