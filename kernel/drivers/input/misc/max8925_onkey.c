

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/mfd/max8925.h>
#include <linux/slab.h>

#define HARDRESET_EN		(1 << 7)
#define PWREN_EN		(1 << 7)

struct max8925_onkey_info {
	struct input_dev	*idev;
	struct i2c_client	*i2c;
	int			irq;
};

static irqreturn_t max8925_onkey_handler(int irq, void *data)
{
	struct max8925_onkey_info *info = data;

	input_report_key(info->idev, KEY_POWER, 1);
	input_sync(info->idev);

	/* Enable hardreset to halt if system isn't shutdown on time */
	max8925_set_bits(info->i2c, MAX8925_SYSENSEL,
			 HARDRESET_EN, HARDRESET_EN);

	return IRQ_HANDLED;
}

static int __devinit max8925_onkey_probe(struct platform_device *pdev)
{
	struct max8925_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct max8925_onkey_info *info;
	int error;

	info = kzalloc(sizeof(struct max8925_onkey_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->i2c = chip->i2c;
	info->irq = chip->irq_base + MAX8925_IRQ_GPM_SW_3SEC;

	info->idev = input_allocate_device();
	if (!info->idev) {
		dev_err(chip->dev, "Failed to allocate input dev\n");
		error = -ENOMEM;
		goto out_input;
	}

	info->idev->name = "max8925_on";
	info->idev->phys = "max8925_on/input0";
	info->idev->id.bustype = BUS_I2C;
	info->idev->dev.parent = &pdev->dev;
	info->idev->evbit[0] = BIT_MASK(EV_KEY);
	info->idev->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);

	error = request_threaded_irq(info->irq, NULL, max8925_onkey_handler,
				     IRQF_ONESHOT, "onkey", info);
	if (error < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq, error);
		goto out_irq;
	}

	error = input_register_device(info->idev);
	if (error) {
		dev_err(chip->dev, "Can't register input device: %d\n", error);
		goto out;
	}

	platform_set_drvdata(pdev, info);

	return 0;

out:
	free_irq(info->irq, info);
out_irq:
	input_free_device(info->idev);
out_input:
	kfree(info);
	return error;
}

static int __devexit max8925_onkey_remove(struct platform_device *pdev)
{
	struct max8925_onkey_info *info = platform_get_drvdata(pdev);

	free_irq(info->irq, info);
	input_unregister_device(info->idev);
	kfree(info);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver max8925_onkey_driver = {
	.driver		= {
		.name	= "max8925-onkey",
		.owner	= THIS_MODULE,
	},
	.probe		= max8925_onkey_probe,
	.remove		= __devexit_p(max8925_onkey_remove),
};

static int __init max8925_onkey_init(void)
{
	return platform_driver_register(&max8925_onkey_driver);
}
module_init(max8925_onkey_init);

static void __exit max8925_onkey_exit(void)
{
	platform_driver_unregister(&max8925_onkey_driver);
}
module_exit(max8925_onkey_exit);

MODULE_DESCRIPTION("Maxim MAX8925 ONKEY driver");
MODULE_AUTHOR("Haojian Zhuang <haojian.zhuang@marvell.com>");
MODULE_LICENSE("GPL");
