#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/thermal.h>
#include <linux/clk.h>

#define MAX_DUTY 			10

struct pwm_fan_ctx {
	struct device           *hwmon_dev;
	struct mutex lock;
	struct pwm_device *pwm;

	unsigned int pwm_fan_state;
	unsigned int pwm_fan_max_state;
	unsigned int *pwm_fan_cooling_levels;
	struct thermal_cooling_device *cdev;

	signed fan0_pwm_gpio;
	signed fan1_pwm_gpio;
	signed fan_en_gpio;
	struct platform_device *fan_pdev;

	int fan0_duty;
	int fan0_enable;
	int fan0_clock;
	struct clk *clk_fan0;

	int fan1_duty;
	int fan1_enable;
	int fan1_clock;
	struct clk *clk_fan1;

	struct pinctrl *pctrl;
	struct pinctrl_state *active;
	struct pinctrl_state *sleep;

	struct work_struct      fan_uevent_work;
};

static int fan0_clk_enable(struct pwm_fan_ctx *ctx)
{
    int ret;

    ret = clk_prepare_enable(ctx->clk_fan0);
    if (ret) {
        pr_err("Failed to enable fan0 clk: %d\n", ret);
        return ret;
    }

    return ret;
}

static void fan0_clk_disable(struct pwm_fan_ctx *ctx)
{
    clk_disable_unprepare(ctx->clk_fan0);
}

static int fan1_clk_enable(struct pwm_fan_ctx *ctx)
{
    int ret;

    ret = clk_prepare_enable(ctx->clk_fan1);
    if (ret) {
        pr_err("Failed to enable fan1 clk: %d\n", ret);
        return ret;
    }

    return ret;
}

static void fan1_clk_disable(struct pwm_fan_ctx *ctx)
{
    clk_disable_unprepare(ctx->clk_fan1);
}

static void fan_open(struct pwm_fan_ctx *ctx, int fan0_en, int fan1_en)
{
	pr_info("fan open fan0 %d %d %d, fan1 %d %d %d",
			fan0_en, ctx->fan0_enable, ctx->fan0_duty,
			fan1_en, ctx->fan1_enable, ctx->fan1_duty);

	if (fan0_en && ctx->fan0_duty) {
		if (ctx->fan0_enable) {
			pr_info("fan0 enable clk, rework");
			fan0_clk_disable(ctx);
			fan0_clk_enable(ctx);
		} else {
			pr_info("fan0 enable clk");
			fan0_clk_enable(ctx);
		}
		ctx->fan0_enable = 1;
	} else {
		if (ctx->fan0_enable) {
			pr_info("fan0 disable clk");
			fan0_clk_disable(ctx);
		} else {
			pr_info("fan0 disable clk, skip");
		}
		ctx->fan0_enable = 0;
	}

	if (fan1_en) {
		if (ctx->fan1_enable) {
			pr_info("fan1 enable clk, rework");
			fan1_clk_disable(ctx);
			fan1_clk_enable(ctx);
		} else {
			pr_info("fan1 enable clk");
			fan1_clk_enable(ctx);
		}
		ctx->fan1_enable = 1;
	} else {
		if (ctx->fan1_enable) {
			pr_info("fan1 disable clk");
			fan1_clk_disable(ctx);
		} else {
			pr_info("fan1 disable clk, skip");
		}
		ctx->fan1_enable = 0;
	}

	if ((ctx->fan0_enable || ctx->fan1_enable)) {
		gpio_direction_output(ctx->fan_en_gpio, 1);
		if (ctx->fan0_enable) {
			pr_info("set fan0 clk duty %d", ctx->fan0_duty);
			clk_set_duty_cycle(ctx->clk_fan0, ctx->fan0_duty, MAX_DUTY);
		}
		if (ctx->fan1_enable) {
			pr_info("set fan1 clk duty %d", ctx->fan1_duty);
			clk_set_duty_cycle(ctx->clk_fan1, ctx->fan1_duty, MAX_DUTY);
		}
	} else
		gpio_direction_output(ctx->fan_en_gpio, 0);

}

static ssize_t fan0_enable_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ctx->fan0_enable);
}

static ssize_t fan0_enable_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);
	unsigned long value;

	if (kstrtoul(buf, 10, &value))
		return -EINVAL;
	pr_info("fan0 enable request %d\n", value);

	fan_open(ctx, value, ctx->fan1_enable);

	return count;
}

static ssize_t fan0_duty_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ctx->fan0_duty);
}

static ssize_t fan0_duty_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);
	unsigned long pwm;

	if (kstrtoul(buf, 10, &pwm) || pwm > MAX_DUTY)
		return -EINVAL;

	pr_info("fan0 duty request %ld %s\n", pwm, buf);
	ctx->fan0_duty = pwm;
	return count;
}

static ssize_t fan0_clock_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ctx->fan0_clock);
}

static ssize_t fan0_clock_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);
	unsigned long clock;

	if (kstrtoul(buf, 10, &clock))
		return -EINVAL;

	pr_info("fan0 clock config %ld %s\n", clock, buf);
	ctx->fan0_clock = clock;
	return count;
}

static ssize_t fan1_enable_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ctx->fan1_enable);
}

static ssize_t fan1_enable_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);
	unsigned long value;

	if (kstrtoul(buf, 10, &value))
		return -EINVAL;
	pr_info("fan1 enable request %d\n", value);

	fan_open(ctx, ctx->fan0_enable, value);

	return count;
}

static ssize_t fan1_duty_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ctx->fan1_duty);
}

static ssize_t fan1_duty_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);
	unsigned long pwm;

	if (kstrtoul(buf, 10, &pwm) || pwm > MAX_DUTY)
		return -EINVAL;

	pr_info("fan1 duty request %ld %s\n", pwm, buf);
	ctx->fan1_duty = pwm;
	return count;
}

static ssize_t fan1_clock_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ctx->fan1_clock);
}

static ssize_t fan1_clock_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);
	unsigned long clock;

	if (kstrtoul(buf, 10, &clock))
		return -EINVAL;

	pr_info("fan1 clock config %ld %s\n", clock, buf);
	ctx->fan1_clock = clock;
	return count;
}


static SENSOR_DEVICE_ATTR_RW(fan0_duty, fan0_duty, 0);
static SENSOR_DEVICE_ATTR_RW(fan0_enable, fan0_enable, 0);
static SENSOR_DEVICE_ATTR_RW(fan0_clock, fan0_clock, 0);
static SENSOR_DEVICE_ATTR_RW(fan1_duty, fan1_duty, 0);
static SENSOR_DEVICE_ATTR_RW(fan1_enable, fan1_enable, 0);
static SENSOR_DEVICE_ATTR_RW(fan1_clock, fan1_clock, 0);

static struct attribute *pwm_fan_attrs[] = {
	&sensor_dev_attr_fan0_duty.dev_attr.attr,
	&sensor_dev_attr_fan0_enable.dev_attr.attr,
	&sensor_dev_attr_fan0_clock.dev_attr.attr,
	&sensor_dev_attr_fan1_duty.dev_attr.attr,
	&sensor_dev_attr_fan1_enable.dev_attr.attr,
	&sensor_dev_attr_fan1_clock.dev_attr.attr,
	NULL,
};

static umode_t pwm_fan_attrs_visible(struct kobject *kobj, struct attribute *a,
				     int n)
{
#if 0
	struct device *dev = container_of(kobj, struct device, kobj);
	struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);
#endif

	return a->mode;
}

static const struct attribute_group pwm_fan_group = {
	.attrs = pwm_fan_attrs,
	.is_visible = pwm_fan_attrs_visible,
};

static const struct attribute_group *pwm_fan_groups[] = {
	&pwm_fan_group,
	NULL,
};

/* thermal cooling device callbacks */
static int pwm_fan_get_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct pwm_fan_ctx *ctx = cdev->devdata;

	if (!ctx)
		return -EINVAL;

	*state = ctx->pwm_fan_max_state;

	return 0;
}

static int pwm_fan_get_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct pwm_fan_ctx *ctx = cdev->devdata;

	if (!ctx)
		return -EINVAL;

	*state = ctx->pwm_fan_state;

	return 0;
}

static void fan_uevent_notify(struct work_struct *ws)
{
	struct pwm_fan_ctx *ctx =
		container_of(ws, struct pwm_fan_ctx, fan_uevent_work);

	kobject_uevent(&ctx->hwmon_dev->kobj, KOBJ_CHANGE);
}

static int
pwm_fan_set_cur_state(struct thermal_cooling_device *cdev, unsigned long state)
{
	struct pwm_fan_ctx *ctx = cdev->devdata;
	int ret = 0;

	if (!ctx || (state > ctx->pwm_fan_max_state))
		return -EINVAL;

	if (state == ctx->pwm_fan_state)
		return 0;

	ctx->pwm_fan_state = state;
	schedule_work(&ctx->fan_uevent_work);

	return ret;
}

static const struct thermal_cooling_device_ops pwm_fan_cooling_ops = {
	.get_max_state = pwm_fan_get_max_state,
	.get_cur_state = pwm_fan_get_cur_state,
	.set_cur_state = pwm_fan_set_cur_state,
};

static int pwm_fan_of_get_cooling_data(struct device *dev,
				       struct pwm_fan_ctx *ctx)
{
	struct device_node *np = dev->of_node;
	int num, i, ret;

	ret = of_property_count_u32_elems(np, "cooling-levels");
	if (ret <= 0) {
		dev_err(dev, "Wrong data!\n");
		return ret ? : -EINVAL;
	}

	num = ret;
	ctx->pwm_fan_cooling_levels = devm_kcalloc(dev, num, sizeof(u32),
						   GFP_KERNEL);
	if (!ctx->pwm_fan_cooling_levels)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "cooling-levels",
					 ctx->pwm_fan_cooling_levels, num);
	if (ret) {
		dev_err(dev, "Property 'cooling-levels' cannot be read!\n");
		return ret;
	}

	for (i = 0; i < num; i++) {
		dev_err(dev, "PWM fan state[%d]:%d\n", i, ctx->pwm_fan_cooling_levels[i]);
#if 0
		if (ctx->pwm_fan_cooling_levels[i] > MAX_DUTY) {
			dev_err(dev, "PWM fan state[%d]:%d > %d\n", i,
				ctx->pwm_fan_cooling_levels[i], MAX_DUTY);
			return -EINVAL;
		}
#endif
	}

	ctx->pwm_fan_max_state = num - 1;

	return 0;
}

static int pwm_fan_probe(struct platform_device *pdev)
{
	struct thermal_cooling_device *cdev;
	struct device *dev = &pdev->dev;
	struct pwm_fan_ctx *ctx;
	int ret;

	dev_err(dev, "%s begin.\n", __func__);

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mutex_init(&ctx->lock);

	platform_set_drvdata(pdev, ctx);

	ctx->fan_pdev = pdev;
	ctx->fan_en_gpio = -EINVAL;

	ctx->fan0_pwm_gpio = -EINVAL;
	ctx->fan0_duty = 0;
	ctx->fan0_clock = 25000;
	ctx->fan0_enable = 0;

	ctx->fan1_pwm_gpio = -EINVAL;
	ctx->fan1_duty = 0;
	ctx->fan1_clock = 25000;
	ctx->fan1_enable = 0;

	ctx->hwmon_dev = devm_hwmon_device_register_with_groups(dev, "pwmfan",
						       ctx, pwm_fan_groups);
	if (IS_ERR(ctx->hwmon_dev)) {
		dev_err(dev, "Failed to register hwmon device\n");
		return PTR_ERR(ctx->hwmon_dev);
	}

	ctx->fan_en_gpio = of_get_named_gpio(pdev->dev.of_node, "fan-en-gpio", 0);
	if (!gpio_is_valid(ctx->fan_en_gpio)) {
		pr_err("fan_en_gpio is invalid.\n");
		return -EPERM;
	}

	ctx->pctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(ctx->pctrl)) {
		ret = PTR_ERR(ctx->pctrl);
		dev_err(&pdev->dev, "Failed to get fan pinctrl %d\n", ret);
		goto err_pctrl;
	}

	ctx->active = pinctrl_lookup_state(ctx->pctrl, "fan_active");
	if (IS_ERR_OR_NULL(ctx->active)) {
		ret = PTR_ERR(ctx->active);
		dev_err(&pdev->dev, "Failed to lookup fan active state %d\n", ret);
		goto err_pctrl;
	}

	ctx->sleep = pinctrl_lookup_state(ctx->pctrl, "fan_sleep");
	if (IS_ERR_OR_NULL(ctx->sleep)) {
		ret = PTR_ERR(ctx->sleep);
		dev_err(&pdev->dev, "Failed to lookup fan sleep state %d\n", ret);
		goto err_pctrl;
	}

	ret = pinctrl_select_state(ctx->pctrl, ctx->active);
	if (ret<0) {
		dev_err(&pdev->dev, "Failed to select fan active state %d\n", ret);
		goto err_pctrl;
	}

	ret = devm_gpio_request_one(&pdev->dev, ctx->fan_en_gpio, GPIOF_OUT_INIT_LOW, "fan_en_gpio");
	if (ret) {
		pr_err("fan_en_gpio request failed\n");
		goto err_pctrl;
	}

	gpio_direction_output(ctx->fan_en_gpio, 0);

	ctx->clk_fan0 = devm_clk_get(&pdev->dev, "fan0_gp_clk");
	if (IS_ERR(ctx->clk_fan0)) {
		dev_err(&pdev->dev, "Failed to get fan0 clock\n");
	}
	clk_set_rate(ctx->clk_fan0, ctx->fan0_clock);

	ctx->clk_fan1 = devm_clk_get(&pdev->dev, "fan1_gp_clk");
	if (IS_ERR(ctx->clk_fan1)) {
		dev_err(&pdev->dev, "Failed to get fan1 clock\n");
	}
	clk_set_rate(ctx->clk_fan1, ctx->fan1_clock);

	ret = pwm_fan_of_get_cooling_data(dev, ctx);
	if (ret)
		return ret;

	//ctx->pwm_fan_state = ctx->pwm_fan_max_state;
	ctx->pwm_fan_state = 0;
	if (IS_ENABLED(CONFIG_THERMAL)) {
		cdev = devm_thermal_of_cooling_device_register(dev,
			dev->of_node, "pwm-fan", ctx, &pwm_fan_cooling_ops);
		if (IS_ERR(cdev)) {
			ret = PTR_ERR(cdev);
			dev_err(dev,
				"Failed to register pwm-fan as cooling device: %d\n",
				ret);
			return ret;
		}
		ctx->cdev = cdev;
		thermal_cdev_update(cdev);
	}

	INIT_WORK(&ctx->fan_uevent_work, fan_uevent_notify);
	dev_err(dev, "%s end.\n", __func__);

	return 0;

err_pctrl:
	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int pwm_fan_suspend(struct device *dev)
{
	//struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);
	return 0;
}

static int pwm_fan_resume(struct device *dev)
{
	//struct pwm_fan_ctx *ctx = dev_get_drvdata(dev);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(pwm_fan_pm, pwm_fan_suspend, pwm_fan_resume);

static const struct of_device_id of_pwm_fan_match[] = {
	{ .compatible = "fan,thermal", },
	{},
};
MODULE_DEVICE_TABLE(of, of_pwm_fan_match);

static struct platform_driver pwm_fan_driver = {
	.probe		= pwm_fan_probe,
	.driver	= {
		.name		= "pwm-fan",
		.pm		= &pwm_fan_pm,
		.of_match_table	= of_pwm_fan_match,
	},
};

module_platform_driver(pwm_fan_driver);

MODULE_AUTHOR("Lenovo.com");
MODULE_ALIAS("platform:pwm-fan");
MODULE_DESCRIPTION("PWM FAN driver");
MODULE_LICENSE("GPL");
