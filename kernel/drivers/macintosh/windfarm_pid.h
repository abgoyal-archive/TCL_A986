


#define WF_PID_MAX_HISTORY	32

struct wf_pid_param {
	int	interval;	/* Interval between samples in seconds */
	int	history_len;	/* Size of history buffer */
	int	additive;	/* 1: target relative to previous value */
	s32	gd, gp, gr;	/* PID gains */
	s32	itarget;	/* PID input target */
	s32	min,max;	/* min and max target values */
};

struct wf_pid_state {
	int	first;				/* first run of the loop */
	int	index; 				/* index of current sample */
	s32	target;				/* current target value */
	s32	samples[WF_PID_MAX_HISTORY];	/* samples history buffer */
	s32	errors[WF_PID_MAX_HISTORY];	/* error history buffer */

	struct wf_pid_param param;
};

extern void wf_pid_init(struct wf_pid_state *st, struct wf_pid_param *param);
extern s32 wf_pid_run(struct wf_pid_state *st, s32 sample);



#define WF_CPU_PID_MAX_HISTORY	32

struct wf_cpu_pid_param {
	int	interval;	/* Interval between samples in seconds */
	int	history_len;	/* Size of history buffer */
	s32	gd, gp, gr;	/* PID gains */
	s32	pmaxadj;	/* PID max power adjust */
	s32	ttarget;	/* PID input target */
	s32	tmax;		/* PID input max */
	s32	min,max;	/* min and max target values */
};

struct wf_cpu_pid_state {
	int	first;				/* first run of the loop */
	int	index; 				/* index of current power */
	int	tindex; 			/* index of current temp */
	s32	target;				/* current target value */
	s32	last_delta;			/* last Tactual - Ttarget */
	s32	powers[WF_PID_MAX_HISTORY];	/* power history buffer */
	s32	errors[WF_PID_MAX_HISTORY];	/* error history buffer */
	s32	temps[2];			/* temp. history buffer */

	struct wf_cpu_pid_param param;
};

extern void wf_cpu_pid_init(struct wf_cpu_pid_state *st,
			    struct wf_cpu_pid_param *param);
extern s32 wf_cpu_pid_run(struct wf_cpu_pid_state *st, s32 power, s32 temp);
