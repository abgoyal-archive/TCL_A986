

static __init int profile_init(void)
{
	u16 tmp;

	if (!prof_buffer)
		return 0;

	/* use timer 11 to drive the profiling interrupts */
	set_intr_stub(EXCEP_IRQ_LEVEL0, profile_handler);

	/* set IRQ priority at which to run */
	set_intr_level(TM11IRQ, GxICR_LEVEL_0);

	/* set up timer 11
	 * - source: (IOCLK 33MHz)*2 = 66MHz
	 * - frequency: (33330000*2) / 8 / 20625 = 202Hz
	 */
	TM11BR = 20625 - 1;
	TM11MD = TM8MD_SRC_IOCLK_8;
	TM11MD |= TM8MD_INIT_COUNTER;
	TM11MD &= ~TM8MD_INIT_COUNTER;
	TM11MD |= TM8MD_COUNT_ENABLE;

	TM11ICR |= GxICR_ENABLE;
	tmp = TM11ICR;

	printk(KERN_INFO "Profiling initiated on timer 11, priority 0, %uHz\n",
	       mn10300_ioclk / 8 / (TM11BR + 1));
	printk(KERN_INFO "Profile histogram stored %p-%p\n",
	       prof_buffer, (u8 *)(prof_buffer + prof_len) - 1);

	return 0;
}

__initcall(profile_init);
