/*
 * Copyright (C) 2015, 2018 Motorola Mobility LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/lenovo_sys_temp.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/uaccess.h>
#include <linux/power_supply.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>
#include <linux/compat.h>

#define TEMP_NODE_SENSOR_NAMES "lenovo,temperature-names"
#define SENSOR_LISTENER_NAMES "lenovo,sensor-listener-names"
#define DEFAULT_TEMPERATURE 0

struct lenovo_sys_temp_sensor {
	struct thermal_zone_device *tz_dev;
	const char *name;
	int temp;
};

struct lenovo_sys_temp_dev {
	struct platform_device *pdev;
	int num_sensors;
	int num_sensors_listener;
	struct lenovo_sys_temp_sensor *sensor;
	struct lenovo_sys_temp_sensor *sensor_listener;
	struct notifier_block psy_nb;
	struct work_struct psy_changed_work;
};

static struct lenovo_sys_temp_dev *sys_temp_dev;

static int lenovo_sys_temp_ioctl_open(struct inode *node, struct file *file)
{
	return 0;
}

static int lenovo_sys_temp_ioctl_release(struct inode *node, struct file *file)
{
	return 0;
}

static long lenovo_sys_temp_ioctl(struct file *file, unsigned int cmd,
			       unsigned long arg)
{
	int i;
	int ret = 0;
	struct lenovo_sys_temp_ioctl request;

	if (!sys_temp_dev)
		return -EINVAL;

	if ((_IOC_TYPE(cmd) != LENOVO_SYS_TEMP_MAGIC_NUM) ||
	    (_IOC_NR(cmd) >= LENOVO_SYS_TEMP_MAX_NUM)) {
		return -ENOTTY;
	}

	switch (cmd) {
	case LENOVO_SYS_TEMP_SET_TEMP:
		ret = copy_from_user(&request, (void __user *)arg,
				     sizeof(struct lenovo_sys_temp_ioctl));
		if (ret) {
			dev_err(&sys_temp_dev->pdev->dev,
				"failed to copy_from_user\n");
			return -EACCES;
		}

		//dev_info(&sys_temp_dev->pdev->dev, "name=%s, temperature=%d\n",
			//request.name, request.temperature);

		for (i = 0; i < sys_temp_dev->num_sensors; i++) {
			if (!strncasecmp(sys_temp_dev->sensor[i].name,
					 request.name,
					 LENOVO_SYS_TEMP_NAME_LENGTH)) {
				sys_temp_dev->sensor[i].temp =
					request.temperature;
				break;
			}
		}
		if (i >= sys_temp_dev->num_sensors) {
			dev_info(&sys_temp_dev->pdev->dev,
				"name %s not supported\n",
			request.name);
			ret = -EBADR;
		}
		break;
	default:
		break;
	}
	return ret;
}



#ifdef CONFIG_COMPAT
static long lenovo_sys_temp_compat_ioctl(struct file *file,
				      unsigned int cmd, unsigned long arg)
{
	arg = (unsigned long)compat_ptr(arg);
	return lenovo_sys_temp_ioctl(file, cmd, arg);
}
#endif	/* CONFIG_COMPAT */

static const struct file_operations lenovo_sys_temp_fops = {
	.owner = THIS_MODULE,
	.open = lenovo_sys_temp_ioctl_open,
	.unlocked_ioctl = lenovo_sys_temp_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = lenovo_sys_temp_compat_ioctl,
#endif  /* CONFIG_COMPAT */
	.release = lenovo_sys_temp_ioctl_release,
};

static struct miscdevice lenovo_sys_temp_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lenovo_sys_temp",
	.fops = &lenovo_sys_temp_fops,
};

static int lenovo_sys_temp_get(struct thermal_zone_device *thermal,
			    int *temp)
{
	struct lenovo_sys_temp_sensor *sensor = thermal->devdata;

	if (!sensor)
		return -EINVAL;

	*temp = sensor->temp;
	return 0;
}

static struct thermal_zone_device_ops lenovo_sys_temp_ops = {
	.get_temp = lenovo_sys_temp_get,
};

static void psy_changed_work_func(struct work_struct *work)
{
	int num_sensors_listener, i, desc = 0;
	char buf[1024];

	if (!sys_temp_dev)
		return;

	num_sensors_listener = sys_temp_dev->num_sensors_listener;

		for (i = 0; i < num_sensors_listener; i++) {
			if(sys_temp_dev->sensor_listener[i].tz_dev) {

				thermal_zone_get_temp(sys_temp_dev->sensor_listener[i].tz_dev,
						&sys_temp_dev->sensor_listener[i].temp);
			} else {
				dev_err(&sys_temp_dev->pdev->dev,
					"Invalid thermal zone\n");
				return;
			}
		}

		for (i = 0; i < num_sensors_listener; i++) {
			desc +=
				sprintf(buf + desc, "%s=%s%d.%d, ",
				sys_temp_dev->sensor_listener[i].name,
				sys_temp_dev->sensor_listener[i].temp < 0 ? "-" : "",
				abs(sys_temp_dev->sensor_listener[i].temp / 1000),
				abs(sys_temp_dev->sensor_listener[i].temp % 1000));
		}

		dev_info(&sys_temp_dev->pdev->dev,
				"%s\n", buf);
	return;
}

int lenovo_get_sensor_temp(char *name, int *temp)
{
	int num_sensors_listener, i, desc = 0;
	char buf[1024];

	if (!sys_temp_dev)
		return -1;

	num_sensors_listener = sys_temp_dev->num_sensors_listener;

		for (i = 0; i < num_sensors_listener; i++) {
			if(sys_temp_dev->sensor_listener[i].tz_dev) {

				thermal_zone_get_temp(sys_temp_dev->sensor_listener[i].tz_dev,
						&sys_temp_dev->sensor_listener[i].temp);
			} else {
				dev_err(&sys_temp_dev->pdev->dev,
					"Invalid thermal zone\n");
				return -1;
			}
		}

		for (i = 0; i < num_sensors_listener; i++) {
			desc +=
				sprintf(buf + desc, "get_sensor_temp %s=%s%d.%d, ",
				sys_temp_dev->sensor_listener[i].name,
				sys_temp_dev->sensor_listener[i].temp < 0 ? "-" : "",
				abs(sys_temp_dev->sensor_listener[i].temp / 1000),
				abs(sys_temp_dev->sensor_listener[i].temp % 1000));

			if (!strcmp(sys_temp_dev->sensor_listener[i].name, name))
				*temp = sys_temp_dev->sensor_listener[i].temp;
		}
	return 0;
}

static int psy_changed(struct notifier_block *nb, unsigned long evt, void *ptr)
{
	if (!sys_temp_dev)
		return -EINVAL;

	if (evt == PSY_EVENT_PROP_CHANGED)
		schedule_work(&sys_temp_dev->psy_changed_work);

	return NOTIFY_OK;
}

static int lenovo_sys_temp_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i;
	struct device_node *node;
	int num_sensors;
	int num_registered = 0;
	int num_sensors_listener;

	pr_err("lenovo 2___\n");	
	dev_err(&pdev->dev, "Enter\n");

	if (!pdev)
		return -ENODEV;

	node = pdev->dev.of_node;
	if (!node) {
		dev_err(&pdev->dev, "bad of_node\n");
		return -ENODEV;
	}

	num_sensors = of_property_count_strings(node, TEMP_NODE_SENSOR_NAMES);
	if (num_sensors <= 0) {
		dev_err(&pdev->dev,
			"bad number of sensors: %d\n", num_sensors);
		return -EINVAL;
	}

	num_sensors_listener = of_property_count_strings(node, SENSOR_LISTENER_NAMES);
	if (num_sensors_listener <= 0) {
		dev_err(&pdev->dev,
			"bad number of sensors-listener: %d\n", num_sensors_listener);
	}
	sys_temp_dev = devm_kzalloc(&pdev->dev, sizeof(struct lenovo_sys_temp_dev),
				    GFP_KERNEL);
	if (!sys_temp_dev) {
		dev_err(&pdev->dev,
			"Unable to alloc memory for sys_temp_dev\n");
		return -ENOMEM;
	}

	sys_temp_dev->pdev = pdev;
	sys_temp_dev->num_sensors = num_sensors;

	sys_temp_dev->sensor =
				(struct lenovo_sys_temp_sensor *)devm_kzalloc(&pdev->dev,
				(num_sensors *
				       sizeof(struct lenovo_sys_temp_sensor)),
				       GFP_KERNEL);
	if (!sys_temp_dev->sensor) {
		dev_err(&pdev->dev,
			"Unable to alloc memory for sensor\n");
		return -ENOMEM;
	}

	for (i = 0; i < num_sensors; i++) {
		ret = of_property_read_string_index(node,
						TEMP_NODE_SENSOR_NAMES, i,
						&sys_temp_dev->sensor[i].name);
		if (ret) {
			dev_err(&pdev->dev, "Unable to read of_prop string\n");
			goto err_thermal_unreg;
		}

		sys_temp_dev->sensor[i].temp = DEFAULT_TEMPERATURE;
		sys_temp_dev->sensor[i].tz_dev =
		   thermal_zone_device_register(sys_temp_dev->sensor[i].name,
						0, 0,
						&sys_temp_dev->sensor[i],
						&lenovo_sys_temp_ops,
						NULL, 0, 0);
		if (IS_ERR(sys_temp_dev->sensor[i].tz_dev)) {
			dev_err(&pdev->dev,
				"thermal_zone_device_register() failed.\n");
			ret = -ENODEV;
			goto err_thermal_unreg;
		}
		num_registered = i + 1;
	}

	platform_set_drvdata(pdev, sys_temp_dev);

	ret = misc_register(&lenovo_sys_temp_misc);
	if (ret) {
		dev_err(&pdev->dev, "Error registering device %d\n", ret);
		goto err_thermal_unreg;
	}

	if (num_sensors_listener <= 0) {
		dev_info(&sys_temp_dev->pdev->dev,
				"No configure sensors listener !\n");
		goto err_sensors_listener;
	}

	sys_temp_dev->num_sensors_listener = num_sensors_listener;
	sys_temp_dev->sensor_listener =
				(struct lenovo_sys_temp_sensor *)devm_kzalloc(&pdev->dev,
				(num_sensors_listener *
				       sizeof(struct lenovo_sys_temp_sensor)),
				       GFP_KERNEL);
	if (!sys_temp_dev->sensor_listener) {
		dev_err(&pdev->dev,
			"Unable to alloc memory for sensor_listener\n");
		goto err_sensors_listener;
	}

	for (i = 0; i < num_sensors_listener; i++) {
		ret = of_property_read_string_index(node,
						SENSOR_LISTENER_NAMES, i,
						&sys_temp_dev->sensor_listener[i].name);
		if (ret) {
			dev_err(&pdev->dev, "Unable to read of_prop string\n");
			goto err_sensors_listener;
		}

		sys_temp_dev->sensor_listener[i].temp = DEFAULT_TEMPERATURE;
		if (sys_temp_dev->sensor_listener[i].name) {
			sys_temp_dev->sensor_listener[i].tz_dev =
			thermal_zone_get_zone_by_name(sys_temp_dev->sensor_listener[i].name);
			if (IS_ERR(sys_temp_dev->sensor_listener[i].tz_dev)) {
				dev_err(&pdev->dev,
					"thermal_zone_get_zone_by_name() failed."
					"name %s, i %d\n", sys_temp_dev->sensor_listener[i].name, i);
				goto err_sensors_listener;
			}
		} else {
			dev_err(&pdev->dev,
				"Invalid sensor listener name\n");
			goto err_sensors_listener;
		}
	}

	INIT_WORK(&sys_temp_dev->psy_changed_work, psy_changed_work_func);
	sys_temp_dev->psy_nb.notifier_call = psy_changed;
	power_supply_reg_notifier(&sys_temp_dev->psy_nb);
err_sensors_listener:
	dev_err(&pdev->dev, "Done\n");
	return 0;

err_thermal_unreg:
#ifdef CONFIG_LENOVO_SYS_TEMP
	for (i = 0; i < num_registered; i++)
		thermal_zone_device_unregister(sys_temp_dev->sensor[i].tz_dev);
#endif

	devm_kfree(&pdev->dev, sys_temp_dev);
	return ret;
}

static int lenovo_sys_temp_remove(struct platform_device *pdev)
{
#ifdef CONFIG_LENOVO_SYS_TEMP
	int i;
	struct lenovo_sys_temp_dev *dev =  platform_get_drvdata(pdev);

	for (i = 0; i < dev->num_sensors; i++)
		thermal_zone_device_unregister(dev->sensor[i].tz_dev);
#endif

	misc_deregister(&lenovo_sys_temp_misc);
	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, sys_temp_dev);
	return 0;
}

static const struct of_device_id lenovo_sys_temp_match_table[] = {
	{.compatible = "lenovo,sys-temp"},
	{},
};
MODULE_DEVICE_TABLE(of, lenovo_sys_temp_match_table);

static struct platform_driver lenovo_sys_temp_driver = {
	.probe = lenovo_sys_temp_probe,
	.remove = lenovo_sys_temp_remove,
	.driver = {
		.name = "lenovo_sys_temp",
		.owner = THIS_MODULE,
		.of_match_table = lenovo_sys_temp_match_table,
	},
};

static int __init lenovo_sys_temp_init(void)
{
	pr_err("lenovo 1___\n");	
	return platform_driver_register(&lenovo_sys_temp_driver);
}

static void __exit lenovo_sys_temp_exit(void)
{
	pr_err("lenovo 999___\n");	
	platform_driver_unregister(&lenovo_sys_temp_driver);
}

module_init(lenovo_sys_temp_init);
module_exit(lenovo_sys_temp_exit);

MODULE_ALIAS("platform:lenovo_sys_temp");
MODULE_AUTHOR("Motorola Mobility LLC");
MODULE_DESCRIPTION("Motorola Mobility System Temperatures");
MODULE_LICENSE("GPL");
