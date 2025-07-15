// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/soc/qcom/fsa4480-i2c.h>

#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/soc/qcom/fsa4480-i2c.h>

struct fsa4480_logic_switch_data *fsa4480_logic_data = NULL;


static ssize_t fsa4480_logic_gpio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int tmp = gpio_get_value(fsa4480_logic_data->irq_gpio);
    return sprintf(buf, "%s\n", tmp==0?"0":"1");
}

static DEVICE_ATTR(fsa4480_logic_gpio, 0444, fsa4480_logic_gpio_show, NULL);

static struct attribute *fsa4480_logic_attributes[] = {
    &dev_attr_fsa4480_logic_gpio.attr,
    NULL,
};

static struct attribute_group fsa4480_logic_attr_group = {
    .attrs = fsa4480_logic_attributes,
};

static int fsa4480_logic_probe(struct platform_device *pdev)
{
    int retval = 0;

    int err = 0;
    struct device_node *np = pdev->dev.of_node;
    fsa4480_logic_data = kzalloc(sizeof(struct fsa4480_logic_switch_data), GFP_KERNEL);
    if (!fsa4480_logic_data){
        err = -ENOMEM;
        goto exit;
    }
    fsa4480_logic_data->irq_gpio = of_get_named_gpio(np, "fsa4480_logic-gpio", 0);
    if (fsa4480_logic_data->irq_gpio < 0) {
        pr_err("failed to get fsa4480_logic's \"shenqi,fsa4480_logic-gpio\"\n");
        goto exit_kfree;
    }
	if (0 == gpio_request(fsa4480_logic_data->irq_gpio, "fsa4480-det")) {
		gpio_direction_output(fsa4480_logic_data->irq_gpio, 0);
	}
	else {
		pr_err("failed to get fsa4480_logic: gpio_request fail\n");
		goto exit_free_gpio;
	}
    retval = sysfs_create_group(&pdev->dev.kobj, &fsa4480_logic_attr_group);
    if(retval) {
        printk(KERN_ERR "%s: Failed to create sysfs\n", __FUNCTION__);
		goto exit_free_gpio;

    }
    pr_info("%s sysfs_create_group sucess\n", __func__);
    return retval;
exit_free_gpio:
    gpio_free(fsa4480_logic_data->irq_gpio);
exit_kfree:
    kfree(fsa4480_logic_data);
exit:
    return err;
}


#ifdef CONFIG_OF
static struct of_device_id fsa4480_logic_match_table[] = {
    { .compatible = "shenqi,fsa4480_logic",},
    { },
};
#else
#define fsa4480_logic_match_table NULL
#endif

struct platform_driver fsa4480_logic = {
    .probe = fsa4480_logic_probe,
    .driver = {
        .name = "msm_fsa4480_logic",
        .owner = THIS_MODULE,
        .of_match_table = fsa4480_logic_match_table,
    },
};

#if 0
static int __init fsa4480_logic_init(void)
{
	int rc;
	rc = platform_driver_register(&fsa4480_logic);
	if (rc)
		pr_err("fsa4480_logic: Failed to register logic driver: %d\n", rc);

	return rc;
}
module_init(fsa4480_logic_init);

static void __exit fsa4480_logic_exit(void)
{
	return;
}
module_exit(fsa4480_logic_exit);

MODULE_DESCRIPTION("shenqi FSA4480 logic driver");
MODULE_LICENSE("GPL v2");
#endif
