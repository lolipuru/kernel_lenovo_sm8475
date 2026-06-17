/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __LENOVO_THERMAL_H__
#define __LENOVO_THERMAL_H__

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/kobject.h>
#include <linux/workqueue.h>

struct lenovo_thermal_ctx {
	struct device			*hwmon_dev;
	struct mutex			lock;

	u32				display_rate_state;
	u32				display_rate_max;
	u32				*display_rate_table;

	u32				speaker_state;
	u32				speaker_max;
	u32				*speaker_table;

	u32				modem5g_state;
	u32				modem5g_max;
	u32				*modem5g_table;

	u32				camera_state;
	u32				camera_max;
	u32				*camera_table;

	struct thermal_cooling_device	*tcdev;
	struct work_struct		work;
	struct device			*pdev;

	struct blocking_notifier_head	notifier;
	struct notifier_block           nb;
};

#define LENOVO_THERMAL_DT_DISPLAY_RATE	"qcom,thermal-mitigation-display-rate"
#define LENOVO_THERMAL_DT_SPEAKER	"qcom,thermal-mitigation-speaker"
#define LENOVO_THERMAL_DT_MODEM_5G	"qcom,thermal-mitigation-modem-5g"
#define LENOVO_THERMAL_DT_CAMERA	"qcom,thermal-mitigation-camera"

#define LENOVO_CDEV_DISPLAY_RATE	"display-rate"
#define LENOVO_CDEV_SPEAKER		"speaker"
#define LENOVO_CDEV_MODEM_5G		"modem-5g"
#define LENOVO_CDEV_CAMERA		"camera"

#define CTX_FROM_WORK(w) container_of(w, struct lenovo_thermal_ctx, work)

#endif /* __LENOVO_THERMAL_H__ */