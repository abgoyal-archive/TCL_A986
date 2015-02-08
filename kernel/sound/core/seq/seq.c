

#include <linux/init.h>
#include <linux/moduleparam.h>
#include <sound/core.h>
#include <sound/initval.h>

#include <sound/seq_kernel.h>
#include "seq_clientmgr.h"
#include "seq_memory.h"
#include "seq_queue.h"
#include "seq_lock.h"
#include "seq_timer.h"
#include "seq_system.h"
#include "seq_info.h"
#include <sound/seq_device.h>

#if defined(CONFIG_SND_SEQ_DUMMY_MODULE)
int seq_client_load[15] = {[0] = SNDRV_SEQ_CLIENT_DUMMY, [1 ... 14] = -1};
#else
int seq_client_load[15] = {[0 ... 14] = -1};
#endif
int seq_default_timer_class = SNDRV_TIMER_CLASS_GLOBAL;
int seq_default_timer_sclass = SNDRV_TIMER_SCLASS_NONE;
int seq_default_timer_card = -1;
int seq_default_timer_device =
#ifdef CONFIG_SND_SEQ_HRTIMER_DEFAULT
	SNDRV_TIMER_GLOBAL_HRTIMER
#elif defined(CONFIG_SND_SEQ_RTCTIMER_DEFAULT)
	SNDRV_TIMER_GLOBAL_RTC
#else
	SNDRV_TIMER_GLOBAL_SYSTEM
#endif
	;
int seq_default_timer_subdevice = 0;
int seq_default_timer_resolution = 0;	/* Hz */

MODULE_AUTHOR("Frank van de Pol <fvdpol@coil.demon.nl>, Jaroslav Kysela <perex@perex.cz>");
MODULE_DESCRIPTION("Advanced Linux Sound Architecture sequencer.");
MODULE_LICENSE("GPL");

module_param_array(seq_client_load, int, NULL, 0444);
MODULE_PARM_DESC(seq_client_load, "The numbers of global (system) clients to load through kmod.");
module_param(seq_default_timer_class, int, 0644);
MODULE_PARM_DESC(seq_default_timer_class, "The default timer class.");
module_param(seq_default_timer_sclass, int, 0644);
MODULE_PARM_DESC(seq_default_timer_sclass, "The default timer slave class.");
module_param(seq_default_timer_card, int, 0644);
MODULE_PARM_DESC(seq_default_timer_card, "The default timer card number.");
module_param(seq_default_timer_device, int, 0644);
MODULE_PARM_DESC(seq_default_timer_device, "The default timer device number.");
module_param(seq_default_timer_subdevice, int, 0644);
MODULE_PARM_DESC(seq_default_timer_subdevice, "The default timer subdevice number.");
module_param(seq_default_timer_resolution, int, 0644);
MODULE_PARM_DESC(seq_default_timer_resolution, "The default timer resolution in Hz.");


static int __init alsa_seq_init(void)
{
	int err;

	snd_seq_autoload_lock();
	if ((err = client_init_data()) < 0)
		goto error;

	/* init memory, room for selected events */
	if ((err = snd_sequencer_memory_init()) < 0)
		goto error;

	/* init event queues */
	if ((err = snd_seq_queues_init()) < 0)
		goto error;

	/* register sequencer device */
	if ((err = snd_sequencer_device_init()) < 0)
		goto error;

	/* register proc interface */
	if ((err = snd_seq_info_init()) < 0)
		goto error;

	/* register our internal client */
	if ((err = snd_seq_system_client_init()) < 0)
		goto error;

 error:
	snd_seq_autoload_unlock();
	return err;
}

static void __exit alsa_seq_exit(void)
{
	/* unregister our internal client */
	snd_seq_system_client_done();

	/* unregister proc interface */
	snd_seq_info_done();
	
	/* delete timing queues */
	snd_seq_queues_delete();

	/* unregister sequencer device */
	snd_sequencer_device_done();

	/* release event memory */
	snd_sequencer_memory_done();
}

module_init(alsa_seq_init)
module_exit(alsa_seq_exit)
