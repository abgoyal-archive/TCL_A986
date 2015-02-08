


#ifndef __BFA_WC_H__
#define __BFA_WC_H__

typedef void (*bfa_wc_resume_t) (void *cbarg);

struct bfa_wc_s {
	bfa_wc_resume_t wc_resume;
	void		*wc_cbarg;
	int		wc_count;
};

static inline void
bfa_wc_up(struct bfa_wc_s *wc)
{
	wc->wc_count++;
}

static inline void
bfa_wc_down(struct bfa_wc_s *wc)
{
	wc->wc_count--;
	if (wc->wc_count == 0)
		wc->wc_resume(wc->wc_cbarg);
}

static inline void
bfa_wc_init(struct bfa_wc_s *wc, bfa_wc_resume_t wc_resume, void *wc_cbarg)
{
	wc->wc_resume = wc_resume;
	wc->wc_cbarg = wc_cbarg;
	wc->wc_count = 0;
	bfa_wc_up(wc);
}

static inline void
bfa_wc_wait(struct bfa_wc_s *wc)
{
	bfa_wc_down(wc);
}

#endif
