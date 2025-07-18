/*
 * FPC1020 Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the FPC fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, controlling GPIOs such as sensor reset
 * line, sensor IRQ line.
 *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node.
 *
 * This driver will NOT send any commands to the sensor it only controls the
 * electrical parts.
 *
 * Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/workqueue.h>
#include <drm/drm_panel.h>
#include <linux/soc/qcom/panel_event_notifier.h>

#define FPC_TTW_HOLD_TIME           2000

#define RESET_LOW_SLEEP_MIN_US      5000
#define RESET_LOW_SLEEP_MAX_US      5100
#define RESET_HIGH_SLEEP1_MIN_US    100
#define RESET_HIGH_SLEEP1_MAX_US    200
#define RESET_HIGH_SLEEP2_MIN_US    5000
#define RESET_HIGH_SLEEP2_MAX_US    5100

#define FPC1020_GPIO_EXTERNAL       512
#define FPC1020_PROBE_DEFER_JIFFIES 250
#define FPC1020_PANEL_RETRY_MAX     10

#define NUM_PARAMS_REG_ENABLE_SET   2
#define RELEASE_WAKELOCK_W_V        "release_wakelock_with_verification"
#define RELEASE_WAKELOCK            "release_wakelock"
#define START_IRQS_RECEIVED_CNT     "start_irqs_received_counter"

static const char * const pctl_names[] = {
	"fpc1020_reset_reset",
	"fpc1020_reset_active",
	"fpc1020_irq_active",
	"fpc1020_irq_suspend",
};

struct fpc1020_data {
	struct device          *dev;
	struct pinctrl         *fingerprint_pinctrl;
	struct pinctrl_state   *pinctrl_state[ARRAY_SIZE(pctl_names)];
	struct wakeup_source   *ttw_wl;
	int                     irq_gpio;
	int                     rst_gpio;
	int                     vdd_gpio;
	int                     nbr_irqs_received;
	int                     nbr_irqs_received_counter_start;
	struct mutex            lock;
	bool                    prepared;
	bool                    compatible_enabled;
	u32                     wakeup_enabled;
	u32                     offlock_enabled;
	struct delayed_work     work;
};

static struct drm_panel *active_panel;
static int drm_register_work_times;

static irqreturn_t fpc1020_irq_handler(int irq, void *handle);
static void drm_register_work(struct work_struct *work);
static void fpc_panel_notifier_callback(enum panel_event_notifier_tag tag,
					 struct panel_event_notification *notification,
					 void *client_data);

static int select_pin_ctl(struct fpc1020_data *fpc1020, const char *name)
{
	int idx;
	int rc;

	if (!strncmp("fpc1020_reset_reset", name, 19))
		idx = 0;
	else if (!strncmp("fpc1020_reset_active", name, 20))
		idx = 1;
	else if (!strncmp("fpc1020_irq_active", name, 18))
		idx = 2;
	else if (!strncmp("fpc1020_irq_suspend", name, 19))
		idx = 3;
	else {
		dev_err(fpc1020->dev, "%s:'%s' not found\n", __func__, name);
		return -EINVAL;
	}

	rc = pinctrl_select_state(fpc1020->fingerprint_pinctrl,
				  fpc1020->pinctrl_state[idx]);
	if (rc)
		dev_err(fpc1020->dev, "cannot select '%s'\n", name);

	return rc;
}

static int hw_reset(struct fpc1020_data *fpc1020)
{
	struct gpio_desc *desc;
	int val, rc;

	desc = gpio_to_desc(fpc1020->irq_gpio);
	val = gpiod_get_raw_value(desc);
	dev_info(fpc1020->dev, "IRQ before reset %d\n", val);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");
	if (rc)
		return rc;
	usleep_range(RESET_HIGH_SLEEP1_MIN_US, RESET_HIGH_SLEEP1_MAX_US);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
	if (rc)
		return rc;
	usleep_range(RESET_LOW_SLEEP_MIN_US, RESET_LOW_SLEEP_MAX_US);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");
	if (rc)
		return rc;
	usleep_range(RESET_HIGH_SLEEP2_MIN_US, RESET_HIGH_SLEEP2_MAX_US);

	desc = gpio_to_desc(fpc1020->irq_gpio);
	val = gpiod_get_raw_value(desc);
	dev_info(fpc1020->dev, "IRQ after reset %d\n", val);

	return 0;
}

static inline int fpc_power_setup(struct fpc1020_data *fpc1020, bool enable)
{
	int rc;

	if (fpc1020->vdd_gpio < FPC1020_GPIO_EXTERNAL) {
		struct gpio_desc *desc = gpio_to_desc(fpc1020->vdd_gpio);

		rc = gpiod_direction_output_raw(desc, enable ? 1 : 0);
		if (enable) {
			printk(KERN_INFO "----fpc pwr on result: %d----\n", rc);
			msleep(10);
		} else {
			printk(KERN_INFO "---- power off result: %d----\n", rc);
		}
		return rc;
	} else {
		printk(KERN_INFO "%s: fpc pwr gpio_is_invalid\n", __func__);
		return -EINVAL;
	}
}

static ssize_t clk_enable_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}
static DEVICE_ATTR(clk_enable, S_IWUSR, NULL, clk_enable_set);

static ssize_t pinctl_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int rc;

	mutex_lock(&fpc1020->lock);
	rc = select_pin_ctl(fpc1020, buf);
	mutex_unlock(&fpc1020->lock);

	return rc ? rc : count;
}
static DEVICE_ATTR(pinctl_set, S_IWUSR, NULL, pinctl_set);

static ssize_t regulator_enable_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	char op;
	char name[16];
	int rc = -EINVAL;

	if (NUM_PARAMS_REG_ENABLE_SET != sscanf(buf, "%15[^,],%c", name, &op))
		return -EINVAL;

	if ((op & 0xfe) != 0x64)
		return -EINVAL;

	mutex_lock(&fpc1020->lock);
	if (fpc1020->vdd_gpio < FPC1020_GPIO_EXTERNAL) {
		struct gpio_desc *desc = gpio_to_desc(fpc1020->vdd_gpio);

		if (op == 'e') {
			rc = gpiod_direction_output_raw(desc, 1);
			printk(KERN_INFO "----fpc pwr on result: %d----\n", rc);
			msleep(10);
		} else {
			rc = gpiod_direction_output_raw(desc, 0);
			printk(KERN_INFO "---- power off result: %d----\n", rc);
		}
	} else {
		printk(KERN_INFO "%s: fpc pwr gpio_is_invalid\n", "fpc_power_setup");
		mutex_unlock(&fpc1020->lock);
		return -EINVAL;
	}
	mutex_unlock(&fpc1020->lock);

	return rc ? rc : count;
}
static DEVICE_ATTR(regulator_enable, S_IWUSR, NULL, regulator_enable_set);

static ssize_t hw_reset_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int rc;

	if (strncmp(buf, "reset", 5) != 0)
		return -EINVAL;

	mutex_lock(&fpc1020->lock);
	rc = hw_reset(fpc1020);
	mutex_unlock(&fpc1020->lock);

	return rc ? rc : count;
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, hw_reset_set);

static ssize_t wakeup_enable_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (fpc1020->wakeup_enabled) {
		memcpy(buf, "true", 4);
		buf[4] = '\0';
		return 4;
	} else {
		memcpy(buf, "false", 5);
		buf[5] = '\0';
		return 5;
	}
}

static ssize_t wakeup_enable_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	ssize_t ret = count;

	mutex_lock(&fpc1020->lock);
	if (!strncmp(buf, "enable", 6))
		fpc1020->wakeup_enabled = 1;
	else if (!strncmp(buf, "disable", 7))
		fpc1020->wakeup_enabled = 0;
	else
		ret = -EINVAL;
	mutex_unlock(&fpc1020->lock);

	return ret;
}
static DEVICE_ATTR(wakeup_enable, S_IRUSR | S_IWUSR,
		   wakeup_enable_get, wakeup_enable_set);

static ssize_t offlock_enable_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	buf[0] = fpc1020->offlock_enabled ? '1' : '0';
	buf[1] = '\0';
	return 1;
}

static ssize_t offlock_enable_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	dev_err(dev, "offlock_set %s.\n", buf);
	mutex_lock(&fpc1020->lock);
	fpc1020->offlock_enabled = (buf[0] == '1') ? 1 : 0;
	mutex_unlock(&fpc1020->lock);

	return count;
}
static DEVICE_ATTR(offlock_enable, S_IRUSR | S_IWUSR,
		   offlock_enable_get, offlock_enable_set);

static ssize_t handle_wakelock_cmd(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	ssize_t ret = count;
	size_t cmp_len;

	mutex_lock(&fpc1020->lock);

	cmp_len = min_t(size_t, count, strlen(RELEASE_WAKELOCK_W_V));
	if (!strncmp(buf, RELEASE_WAKELOCK_W_V, cmp_len)) {
		if (fpc1020->nbr_irqs_received_counter_start ==
				fpc1020->nbr_irqs_received)
			__pm_relax(fpc1020->ttw_wl);
	} else {
		cmp_len = min_t(size_t, count, strlen(RELEASE_WAKELOCK));
		if (!strncmp(buf, RELEASE_WAKELOCK, cmp_len)) {
			__pm_relax(fpc1020->ttw_wl);
		} else {
			cmp_len = min_t(size_t, count, strlen(START_IRQS_RECEIVED_CNT));
			if (!strncmp(buf, START_IRQS_RECEIVED_CNT, cmp_len))
				fpc1020->nbr_irqs_received_counter_start =
					fpc1020->nbr_irqs_received;
			else
				ret = -EINVAL;
		}
	}

	mutex_unlock(&fpc1020->lock);
	return ret;
}
static DEVICE_ATTR(handle_wakelock, S_IWUSR, NULL, handle_wakelock_cmd);

static ssize_t irq_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	struct gpio_desc *desc = gpio_to_desc(fpc1020->irq_gpio);
	int irq = gpiod_get_raw_value(desc);

	return scnprintf(buf, PAGE_SIZE, "%i\n", irq);
}

static ssize_t irq_ack(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}
static DEVICE_ATTR(irq, S_IRUSR | S_IWUSR, irq_get, irq_ack);

static ssize_t compatible_all_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	struct gpio_desc *desc;
	int irq, rc;
	size_t i;

	dev_err(dev, "compatible all enter %d\n", fpc1020->compatible_enabled);

	if (!strncmp(buf, "enable", 6)) {
		if (fpc1020->compatible_enabled == 1)
			return count;
	} else if (!strncmp(buf, "disable", 7)) {
		if (fpc1020->compatible_enabled == 0)
			return count;

		if (fpc1020->irq_gpio < FPC1020_GPIO_EXTERNAL) {
			devm_gpio_free(dev, fpc1020->irq_gpio);
			printk(KERN_INFO "remove irq_gpio success\n");
		}
		if (fpc1020->rst_gpio < FPC1020_GPIO_EXTERNAL) {
			devm_gpio_free(dev, fpc1020->rst_gpio);
			printk(KERN_INFO "remove rst_gpio success\n");
		}

		desc = gpio_to_desc(fpc1020->irq_gpio);
		irq = gpiod_to_irq(desc);
		devm_free_irq(dev, irq, fpc1020);
		fpc1020->compatible_enabled = 0;
		return count;
	} else {
		return count;
	}

	rc = of_get_named_gpio_flags(fpc1020->dev->of_node, "fpc,gpio_irq", 0, NULL);
	if (rc < 0) {
		dev_err(fpc1020->dev, "failed to get '%s'\n", "fpc,gpio_irq");
		return -EINVAL;
	}
	fpc1020->irq_gpio = rc;
	rc = devm_gpio_request(fpc1020->dev, fpc1020->irq_gpio, "fpc,gpio_irq");
	if (rc) {
		dev_err(fpc1020->dev, "failed to request %s - gpio %d\n",
			"fpc,gpio_irq", fpc1020->irq_gpio);
		return -EINVAL;
	}

	rc = of_get_named_gpio_flags(fpc1020->dev->of_node, "fpc,gpio_rst", 0, NULL);
	if (rc < 0) {
		dev_err(fpc1020->dev, "failed to get '%s'\n", "fpc,gpio_rst");
		dev_err(dev, "fpc request reset result = %d\n", rc);
		return -EINVAL;
	}
	fpc1020->rst_gpio = rc;
	rc = devm_gpio_request(fpc1020->dev, fpc1020->rst_gpio, "fpc,gpio_rst");
	if (rc) {
		dev_err(fpc1020->dev, "failed to request %s - gpio %d\n",
			"fpc,gpio_rst", fpc1020->rst_gpio);
		dev_err(dev, "fpc request reset result = %d\n", rc);
		return -EINVAL;
	}
	dev_err(dev, "fpc request reset result = %d\n", 0);

	rc = of_get_named_gpio_flags(fpc1020->dev->of_node, "fpc,gpio_pwr", 0, NULL);
	if (rc < 0) {
		dev_err(fpc1020->dev, "failed to get '%s'\n", "fpc,gpio_pwr");
		dev_err(dev, "fpc request pwr_gpio result = %d\n", rc);
		return -EINVAL;
	}
	fpc1020->vdd_gpio = rc;
	rc = devm_gpio_request(fpc1020->dev, fpc1020->vdd_gpio, "fpc,gpio_pwr");
	if (rc) {
		dev_err(fpc1020->dev, "failed to request %s - gpio %d\n",
			"fpc,gpio_pwr", fpc1020->vdd_gpio);
		dev_err(dev, "fpc request pwr_gpio result = %d\n", rc);
		return -EINVAL;
	}
	dev_err(dev, "fpc request pwr_gpio result = %d\n", 0);

	fpc1020->fingerprint_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(fpc1020->fingerprint_pinctrl)) {
		if (PTR_ERR(fpc1020->fingerprint_pinctrl) == -EPROBE_DEFER)
			dev_info(dev, "pinctrl not ready\n");
		else
			dev_err(dev, "Target does not use pinctrl\n");
		fpc1020->fingerprint_pinctrl = NULL;
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(pctl_names); i++) {
		struct pinctrl_state *state =
			pinctrl_lookup_state(fpc1020->fingerprint_pinctrl,
					     pctl_names[i]);
		if (IS_ERR(state)) {
			dev_err(dev, "cannot find '%s'\n", pctl_names[i]);
			return -EINVAL;
		}
		dev_info(dev, "found pin control %s\n", pctl_names[i]);
		fpc1020->pinctrl_state[i] = state;
	}

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
	if (rc)
		return -EINVAL;
	rc = select_pin_ctl(fpc1020, "fpc1020_irq_active");
	if (rc)
		return -EINVAL;

	fpc1020->wakeup_enabled = 1;
	if (of_find_property(dev->of_node, "fpc,enable-wakeup", NULL)) {
		device_init_wakeup(dev, 1);
		dev_info(dev, "fpc enable-wakeup done!\n");
	}

	desc = gpio_to_desc(fpc1020->irq_gpio);
	irq = gpiod_to_irq(desc);
	rc = devm_request_threaded_irq(dev, irq, NULL,
				       fpc1020_irq_handler,
				       IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				       dev_name(dev), fpc1020);
	if (rc) {
		dev_err(dev, "could not request irq %d\n", irq);
		return -EINVAL;
	}

	irq_set_irq_wake(irq, 1);
	fpc1020->compatible_enabled = 1;

	if (of_find_property(dev->of_node, "fpc,enable-on-boot", NULL)) {
		dev_info(dev, "Enabling hardware\n");
		mutex_lock(&fpc1020->lock);
		if (!fpc1020->prepared) {
			fpc1020->prepared = true;
			select_pin_ctl(fpc1020, "fpc1020_reset_reset");
			fpc_power_setup(fpc1020, true);
			usleep_range(100, 1000);
			select_pin_ctl(fpc1020, "fpc1020_reset_active");
		}
		mutex_unlock(&fpc1020->lock);
	}

	hw_reset(fpc1020);
	return count;
}
static DEVICE_ATTR(compatible_all, S_IWUSR, NULL, compatible_all_set);

static ssize_t device_prepare_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int rc = 0;

	if (!strncmp(buf, "enable", 6)) {
		mutex_lock(&fpc1020->lock);
		if (!fpc1020->prepared) {
			fpc1020->prepared = true;
			select_pin_ctl(fpc1020, "fpc1020_reset_reset");
			rc = fpc_power_setup(fpc1020, true);
			usleep_range(100, 1000);
			select_pin_ctl(fpc1020, "fpc1020_reset_active");
		}
		mutex_unlock(&fpc1020->lock);
		return rc ? rc : count;
	} else if (!strncmp(buf, "disable", 7)) {
		mutex_lock(&fpc1020->lock);
		if (fpc1020->prepared) {
			select_pin_ctl(fpc1020, "fpc1020_reset_reset");
			usleep_range(100, 1000);
			fpc_power_setup(fpc1020, false);
			fpc1020->prepared = false;
		}
		mutex_unlock(&fpc1020->lock);
		return count;
	} else {
		return -EINVAL;
	}
}
static DEVICE_ATTR(device_prepare, S_IWUSR, NULL, device_prepare_set);

static struct attribute *attributes[] = {
	&dev_attr_pinctl_set.attr,
	&dev_attr_device_prepare.attr,
	&dev_attr_regulator_enable.attr,
	&dev_attr_hw_reset.attr,
	&dev_attr_wakeup_enable.attr,
	&dev_attr_offlock_enable.attr,
	&dev_attr_handle_wakelock.attr,
	&dev_attr_clk_enable.attr,
	&dev_attr_irq.attr,
	&dev_attr_compatible_all.attr,
	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

static irqreturn_t fpc1020_irq_handler(int irq, void *handle)
{
	struct fpc1020_data *fpc1020 = handle;

	mutex_lock(&fpc1020->lock);
	if (fpc1020->wakeup_enabled) {
		fpc1020->nbr_irqs_received++;
		pm_wakeup_ws_event(fpc1020->ttw_wl, FPC_TTW_HOLD_TIME, false);
	}
	mutex_unlock(&fpc1020->lock);

	sysfs_notify(&fpc1020->dev->kobj, NULL, dev_attr_irq.attr.name);
	return IRQ_HANDLED;
}

static void fpc_panel_notifier_callback(enum panel_event_notifier_tag tag,
					 struct panel_event_notification *notification,
					 void *client_data)
{
	struct fpc1020_data *fpc1020 = client_data;
	struct gpio_desc *desc;
	int irq;

	if (!notification) {
		printk(KERN_ERR "%s: Invalid notification\n", __func__);
		return;
	}

	printk(KERN_INFO "%s: Notification type:%d, early_trigger:%d, offlock=%d",
	       __func__,
	       notification->notif_type,
	       notification->notif_data.early_trigger,
	       fpc1020->offlock_enabled);

	if (fpc1020->offlock_enabled)
		return;

	desc = gpio_to_desc(fpc1020->irq_gpio);
	irq = gpiod_to_irq(desc);

	switch (notification->notif_type) {
	case DRM_PANEL_EVENT_BLANK:
		disable_irq(irq);
		printk(KERN_INFO "fpc: DRM_PANEL_EVENT_BLANK, disable irq \n");
		break;
	case DRM_PANEL_EVENT_UNBLANK:
		enable_irq(irq);
		printk(KERN_INFO "fpc: DRM_PANEL_EVENT_UNBLANK, enable irq \n");
		break;
	default:
		break;
	}
}

static void drm_register_work(struct work_struct *work)
{
	struct fpc1020_data *fpc1020 =
		container_of(work, struct fpc1020_data, work.work);
	struct device_node *of_node = fpc1020->dev->of_node;
	int count, i;
	void *cookie;

	printk(KERN_ERR "fpc: drm_register_work\n");

	count = of_count_phandle_with_args(of_node, "panel", NULL);
	if (count < 1) {
		printk(KERN_INFO "fpc: find drm_panel count(%d) fail", count);
		goto retry;
	}

	for (i = 0; i < count; i++) {
		struct device_node *np =
			of_parse_phandle(of_node, "panel", i);
		struct drm_panel *panel = of_drm_find_panel(np);

		if (!IS_ERR_OR_NULL(panel)) {
			active_panel = panel;
			printk(KERN_INFO "fpc: find drm_panel successfully");
			goto register_notifier;
		}
	}

	printk(KERN_INFO "fpc: no find drm_panel");

retry:
	if (drm_register_work_times <= 9) {
		drm_register_work_times++;
		queue_delayed_work(system_wq, &fpc1020->work,
				   FPC1020_PROBE_DEFER_JIFFIES);
		printk(KERN_INFO "fpc: try register drm after 1s\n");
		return;
	}

register_notifier:
	if (!active_panel)
		return;

	cookie = panel_event_notifier_register(
				PANEL_EVENT_NOTIFICATION_PRIMARY,
				PANEL_EVENT_NOTIFIER_CLIENT_PRIMARY_TOUCH,
				active_panel,
				fpc_panel_notifier_callback,
				fpc1020);
	if (!cookie) {
		printk(KERN_ERR "Failed to register for panel events\n");
	} else {
		printk(KERN_INFO "registered for panel notifications panel: 0x%x\n",
		       (unsigned int)(uintptr_t)active_panel);
	}
}

static int fpc1020_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct fpc1020_data *fpc1020;
	int rc;

	fpc1020 = devm_kzalloc(dev, sizeof(*fpc1020), GFP_KERNEL);
	if (!fpc1020) {
		dev_err(dev, "failed to allocate memory for struct fpc1020_data\n");
		return -ENOMEM;
	}

	fpc1020->dev = dev;
	platform_set_drvdata(pdev, fpc1020);

	if (!np) {
		dev_err(dev, "no of node found\n");
		return -EINVAL;
	}

	mutex_init(&fpc1020->lock);

	fpc1020->ttw_wl = wakeup_source_register(dev, "fpc_ttw_wl");
	if (!fpc1020->ttw_wl)
		return -ENOMEM;

	rc = sysfs_create_group(&dev->kobj, &attribute_group);
	if (rc) {
		dev_err(dev, "could not create sysfs\n");
		return rc;
	}

	INIT_DELAYED_WORK(&fpc1020->work, drm_register_work);
	queue_delayed_work(system_wq, &fpc1020->work,
			   FPC1020_PROBE_DEFER_JIFFIES);

	fpc1020->offlock_enabled = 1;

	dev_info(dev, "%s: ok\n", __func__);
	return 0;
}

static int fpc1020_remove(struct platform_device *pdev)
{
	struct fpc1020_data *fpc1020 = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &attribute_group);
	wakeup_source_unregister(fpc1020->ttw_wl);

	if (fpc1020->vdd_gpio < FPC1020_GPIO_EXTERNAL) {
		struct gpio_desc *desc = gpio_to_desc(fpc1020->vdd_gpio);
		int rc = gpiod_direction_output_raw(desc, 0);

		printk(KERN_INFO "---- power off result: %d----\n", rc);
	} else {
		printk(KERN_INFO "%s: fpc pwr gpio_is_invalid\n", "fpc_power_setup");
	}

	dev_info(&pdev->dev, "%s\n", __func__);
	return 0;
}

static int fpc1020_suspend(struct device *dev)
{
	dev_err(dev, "fpc suspend.\n");
	return 0;
}

static int fpc1020_resume(struct device *dev)
{
	dev_err(dev, "fpc resume.\n");
	return 0;
}

static const struct of_device_id fpc1020_of_match[] = {
	{ .compatible = "fpc,fpc1020", },
	{}
};
MODULE_DEVICE_TABLE(of, fpc1020_of_match);

static const struct dev_pm_ops fpc1020_pm_ops = {
	.suspend = fpc1020_suspend,
	.resume  = fpc1020_resume,
};

static struct platform_driver fpc1020_driver = {
	.driver = {
		.name           = "fpc1020",
		.owner          = THIS_MODULE,
		.of_match_table = fpc1020_of_match,
		.pm             = &fpc1020_pm_ops,
	},
	.probe  = fpc1020_probe,
	.remove = fpc1020_remove,
};

static int __init fpc1020_init(void)
{
	int rc = platform_driver_register(&fpc1020_driver);

	if (!rc)
		pr_info("%s OK\n", __func__);
	else
		pr_err("%s %d\n", __func__, rc);
	return rc;
}

static void __exit fpc1020_exit(void)
{
	pr_info("%s\n", __func__);
	platform_driver_unregister(&fpc1020_driver);
}

module_init(fpc1020_init);
module_exit(fpc1020_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aleksej Makarov");
MODULE_AUTHOR("Henrik Tillman <henrik.tillman@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 Fingerprint sensor device driver.");
