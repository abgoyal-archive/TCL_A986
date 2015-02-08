

#include <sound/jack.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

int snd_soc_jack_new(struct snd_soc_card *card, const char *id, int type,
		     struct snd_soc_jack *jack)
{
	jack->card = card;
	INIT_LIST_HEAD(&jack->pins);
	BLOCKING_INIT_NOTIFIER_HEAD(&jack->notifier);

	return snd_jack_new(card->codec->card, id, type, &jack->jack);
}
EXPORT_SYMBOL_GPL(snd_soc_jack_new);

void snd_soc_jack_report(struct snd_soc_jack *jack, int status, int mask)
{
	struct snd_soc_codec *codec;
	struct snd_soc_jack_pin *pin;
	int enable;
	int oldstatus;

	if (!jack)
		return;

	codec = jack->card->codec;

	mutex_lock(&codec->mutex);

	oldstatus = jack->status;

	jack->status &= ~mask;
	jack->status |= status & mask;

	/* The DAPM sync is expensive enough to be worth skipping.
	 * However, empty mask means pin synchronization is desired. */
	if (mask && (jack->status == oldstatus))
		goto out;

	list_for_each_entry(pin, &jack->pins, list) {
		enable = pin->mask & jack->status;

		if (pin->invert)
			enable = !enable;

		if (enable)
			snd_soc_dapm_enable_pin(codec, pin->pin);
		else
			snd_soc_dapm_disable_pin(codec, pin->pin);
	}

	/* Report before the DAPM sync to help users updating micbias status */
	blocking_notifier_call_chain(&jack->notifier, status, NULL);

	snd_soc_dapm_sync(codec);

	snd_jack_report(jack->jack, status);

out:
	mutex_unlock(&codec->mutex);
}
EXPORT_SYMBOL_GPL(snd_soc_jack_report);

int snd_soc_jack_add_pins(struct snd_soc_jack *jack, int count,
			  struct snd_soc_jack_pin *pins)
{
	int i;

	for (i = 0; i < count; i++) {
		if (!pins[i].pin) {
			printk(KERN_ERR "No name for pin %d\n", i);
			return -EINVAL;
		}
		if (!pins[i].mask) {
			printk(KERN_ERR "No mask for pin %d (%s)\n", i,
			       pins[i].pin);
			return -EINVAL;
		}

		INIT_LIST_HEAD(&pins[i].list);
		list_add(&(pins[i].list), &jack->pins);
	}

	/* Update to reflect the last reported status; canned jack
	 * implementations are likely to set their state before the
	 * card has an opportunity to associate pins.
	 */
	snd_soc_jack_report(jack, 0, 0);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_jack_add_pins);

void snd_soc_jack_notifier_register(struct snd_soc_jack *jack,
				    struct notifier_block *nb)
{
	blocking_notifier_chain_register(&jack->notifier, nb);
}
EXPORT_SYMBOL_GPL(snd_soc_jack_notifier_register);

void snd_soc_jack_notifier_unregister(struct snd_soc_jack *jack,
				      struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&jack->notifier, nb);
}
EXPORT_SYMBOL_GPL(snd_soc_jack_notifier_unregister);

#ifdef CONFIG_GPIOLIB
/* gpio detect */
static void snd_soc_jack_gpio_detect(struct snd_soc_jack_gpio *gpio)
{
	struct snd_soc_jack *jack = gpio->jack;
	int enable;
	int report;

	if (gpio->debounce_time > 0)
		mdelay(gpio->debounce_time);

	enable = gpio_get_value(gpio->gpio);
	if (gpio->invert)
		enable = !enable;

	if (enable)
		report = gpio->report;
	else
		report = 0;

	if (gpio->jack_status_check)
		report = gpio->jack_status_check();

	snd_soc_jack_report(jack, report, gpio->report);
}

/* irq handler for gpio pin */
static irqreturn_t gpio_handler(int irq, void *data)
{
	struct snd_soc_jack_gpio *gpio = data;

	schedule_work(&gpio->work);

	return IRQ_HANDLED;
}

/* gpio work */
static void gpio_work(struct work_struct *work)
{
	struct snd_soc_jack_gpio *gpio;

	gpio = container_of(work, struct snd_soc_jack_gpio, work);
	snd_soc_jack_gpio_detect(gpio);
}

int snd_soc_jack_add_gpios(struct snd_soc_jack *jack, int count,
			struct snd_soc_jack_gpio *gpios)
{
	int i, ret;

	for (i = 0; i < count; i++) {
		if (!gpio_is_valid(gpios[i].gpio)) {
			printk(KERN_ERR "Invalid gpio %d\n",
				gpios[i].gpio);
			ret = -EINVAL;
			goto undo;
		}
		if (!gpios[i].name) {
			printk(KERN_ERR "No name for gpio %d\n",
				gpios[i].gpio);
			ret = -EINVAL;
			goto undo;
		}

		ret = gpio_request(gpios[i].gpio, gpios[i].name);
		if (ret)
			goto undo;

		ret = gpio_direction_input(gpios[i].gpio);
		if (ret)
			goto err;

		INIT_WORK(&gpios[i].work, gpio_work);
		gpios[i].jack = jack;

		ret = request_irq(gpio_to_irq(gpios[i].gpio),
				gpio_handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				jack->card->dev->driver->name,
				&gpios[i]);
		if (ret)
			goto err;

#ifdef CONFIG_GPIO_SYSFS
		/* Expose GPIO value over sysfs for diagnostic purposes */
		gpio_export(gpios[i].gpio, false);
#endif

		/* Update initial jack status */
		snd_soc_jack_gpio_detect(&gpios[i]);
	}

	return 0;

err:
	gpio_free(gpios[i].gpio);
undo:
	snd_soc_jack_free_gpios(jack, i, gpios);

	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_jack_add_gpios);

void snd_soc_jack_free_gpios(struct snd_soc_jack *jack, int count,
			struct snd_soc_jack_gpio *gpios)
{
	int i;

	for (i = 0; i < count; i++) {
#ifdef CONFIG_GPIO_SYSFS
		gpio_unexport(gpios[i].gpio);
#endif
		free_irq(gpio_to_irq(gpios[i].gpio), &gpios[i]);
		gpio_free(gpios[i].gpio);
		gpios[i].jack = NULL;
	}
}
EXPORT_SYMBOL_GPL(snd_soc_jack_free_gpios);
#endif	/* CONFIG_GPIOLIB */
