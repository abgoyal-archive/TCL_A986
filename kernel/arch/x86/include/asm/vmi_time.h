

#ifndef _ASM_X86_VMI_TIME_H
#define _ASM_X86_VMI_TIME_H

#define VMI_CALL_GetCycleFrequency	66
#define VMI_CALL_GetCycleCounter	67
#define VMI_CALL_SetAlarm		68
#define VMI_CALL_CancelAlarm		69
#define VMI_CALL_GetWallclockTime	70
#define VMI_CALL_WallclockUpdated	71

/* Cached VMI timer operations */
extern struct vmi_timer_ops {
	u64 (*get_cycle_frequency)(void);
	u64 (*get_cycle_counter)(int);
	u64 (*get_wallclock)(void);
	int (*wallclock_updated)(void);
	void (*set_alarm)(u32 flags, u64 expiry, u64 period);
	void (*cancel_alarm)(u32 flags);
} vmi_timer_ops;

/* Prototypes */
extern void __init vmi_time_init(void);
extern unsigned long vmi_get_wallclock(void);
extern int vmi_set_wallclock(unsigned long now);
extern unsigned long long vmi_sched_clock(void);
extern unsigned long vmi_tsc_khz(void);

#ifdef CONFIG_X86_LOCAL_APIC
extern void __devinit vmi_time_bsp_init(void);
extern void __devinit vmi_time_ap_init(void);
#endif


/* The cycle counters. */
#define VMI_CYCLES_REAL         0
#define VMI_CYCLES_AVAILABLE    1
#define VMI_CYCLES_STOLEN       2

/* The alarm interface 'flags' bits */
#define VMI_ALARM_COUNTERS      2

#define VMI_ALARM_COUNTER_MASK  0x000000ff

#define VMI_ALARM_WIRED_IRQ0    0x00000000
#define VMI_ALARM_WIRED_LVTT    0x00010000

#define VMI_ALARM_IS_ONESHOT    0x00000000
#define VMI_ALARM_IS_PERIODIC   0x00000100

#define CONFIG_VMI_ALARM_HZ	100

#endif /* _ASM_X86_VMI_TIME_H */
