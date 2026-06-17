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
#include <linux/notifier.h>
#include <linux/device.h>
#include <linux/err.h>

#include "lenovo-thermal.h"

static int display_rate_get_max_state(struct thermal_cooling_device *cdev,
                      unsigned long *state);
static int display_rate_get_cur_state(struct thermal_cooling_device *cdev,
                      unsigned long *state);
static int display_rate_set_cur_state(struct thermal_cooling_device *cdev,
                      unsigned long state);

static int speaker_get_max_state(struct thermal_cooling_device *cdev,
                 unsigned long *state);
static int speaker_get_cur_state(struct thermal_cooling_device *cdev,
                 unsigned long *state);
static int speaker_set_cur_state(struct thermal_cooling_device *cdev,
                 unsigned long state);

static int modem_5g_get_max_state(struct thermal_cooling_device *cdev,
                  unsigned long *state);
static int modem_5g_get_cur_state(struct thermal_cooling_device *cdev,
                  unsigned long *state);
static int modem_5g_set_cur_state(struct thermal_cooling_device *cdev,
                  unsigned long state);

static int camera_get_max_state(struct thermal_cooling_device *cdev,
                unsigned long *state);
static int camera_get_cur_state(struct thermal_cooling_device *cdev,
                unsigned long *state);
static int camera_set_cur_state(struct thermal_cooling_device *cdev,
                unsigned long state);

static const struct thermal_cooling_device_ops display_rate_cooling_ops = {
    .get_max_state = display_rate_get_max_state,
    .get_cur_state = display_rate_get_cur_state,
    .set_cur_state = display_rate_set_cur_state,
};

static const struct thermal_cooling_device_ops speaker_cooling_ops = {
    .get_max_state = speaker_get_max_state,
    .get_cur_state = speaker_get_cur_state,
    .set_cur_state = speaker_set_cur_state,
};

static const struct thermal_cooling_device_ops modem_5g_cooling_ops = {
    .get_max_state = modem_5g_get_max_state,
    .get_cur_state = modem_5g_get_cur_state,
    .set_cur_state = modem_5g_set_cur_state,
};

static const struct thermal_cooling_device_ops camera_cooling_ops = {
    .get_max_state = camera_get_max_state,
    .get_cur_state = camera_get_cur_state,
    .set_cur_state = camera_set_cur_state,
};

static int __maybe_unused lenovo_thermal_uevent_notify(struct notifier_block *nb,
                    unsigned long action, void *data)
{
    struct lenovo_thermal_ctx *ctx =
        container_of(nb, struct lenovo_thermal_ctx, nb);

    kobject_uevent(&ctx->pdev->kobj, KOBJ_CHANGE);
    return NOTIFY_OK;
}

static void lenovo_thermal_work(struct work_struct *work)
{
    struct lenovo_thermal_ctx *ctx = CTX_FROM_WORK(work);

    kobject_uevent(&ctx->pdev->kobj, KOBJ_CHANGE);
}

static int display_rate_get_max_state(struct thermal_cooling_device *cdev,
                      unsigned long *state)
{
    struct lenovo_thermal_ctx *ctx = cdev->devdata;

    if (!ctx)
        return -EINVAL;

    *state = ctx->display_rate_max;
    return 0;
}

static int display_rate_get_cur_state(struct thermal_cooling_device *cdev,
                      unsigned long *state)
{
    struct lenovo_thermal_ctx *ctx = cdev->devdata;

    if (!ctx)
        return -EINVAL;

    *state = ctx->display_rate_state;
    return 0;
}

static int display_rate_set_cur_state(struct thermal_cooling_device *cdev,
                      unsigned long state)
{
    struct lenovo_thermal_ctx *ctx = cdev->devdata;

    if (!ctx || state > ctx->display_rate_max)
        return -EINVAL;

    if (ctx->display_rate_state != (u32)state) {
        ctx->display_rate_state = (u32)state;
        queue_work_on(0x20, system_wq, &ctx->work);
    }
    return 0;
}

static int speaker_get_max_state(struct thermal_cooling_device *cdev,
                 unsigned long *state)
{
    struct lenovo_thermal_ctx *ctx = cdev->devdata;

    if (!ctx)
        return -EINVAL;

    *state = ctx->speaker_max;
    return 0;
}

static int speaker_get_cur_state(struct thermal_cooling_device *cdev,
                 unsigned long *state)
{
    struct lenovo_thermal_ctx *ctx = cdev->devdata;

    if (!ctx)
        return -EINVAL;

    *state = ctx->speaker_state;
    return 0;
}

static int speaker_set_cur_state(struct thermal_cooling_device *cdev,
                 unsigned long state)
{
    struct lenovo_thermal_ctx *ctx = cdev->devdata;

    if (!ctx || state > ctx->speaker_max)
        return -EINVAL;

    if (ctx->speaker_state != (u32)state) {
        ctx->speaker_state = (u32)state;
        queue_work_on(0x20, system_wq, &ctx->work);
    }
    return 0;
}

static int modem_5g_get_max_state(struct thermal_cooling_device *cdev,
                  unsigned long *state)
{
    struct lenovo_thermal_ctx *ctx = cdev->devdata;

    if (!ctx)
        return -EINVAL;

    *state = ctx->modem5g_max;
    return 0;
}

static int modem_5g_get_cur_state(struct thermal_cooling_device *cdev,
                  unsigned long *state)
{
    struct lenovo_thermal_ctx *ctx = cdev->devdata;

    if (!ctx)
        return -EINVAL;

    *state = ctx->modem5g_state;
    return 0;
}

static int modem_5g_set_cur_state(struct thermal_cooling_device *cdev,
                  unsigned long state)
{
    struct lenovo_thermal_ctx *ctx = cdev->devdata;

    if (!ctx || state > ctx->modem5g_max)
        return -EINVAL;

    if (ctx->modem5g_state != (u32)state) {
        ctx->modem5g_state = (u32)state;
        queue_work_on(0x20, system_wq, &ctx->work);
    }
    return 0;
}

static int camera_get_max_state(struct thermal_cooling_device *cdev,
                unsigned long *state)
{
    struct lenovo_thermal_ctx *ctx = cdev->devdata;

    if (!ctx)
        return -EINVAL;

    *state = ctx->camera_max;
    return 0;
}

static int camera_get_cur_state(struct thermal_cooling_device *cdev,
                unsigned long *state)
{
    struct lenovo_thermal_ctx *ctx = cdev->devdata;

    if (!ctx)
        return -EINVAL;

    *state = ctx->camera_state;
    return 0;
}

static int camera_set_cur_state(struct thermal_cooling_device *cdev,
                unsigned long state)
{
    struct lenovo_thermal_ctx *ctx = cdev->devdata;

    if (!ctx || state > ctx->camera_max)
        return -EINVAL;

    if (ctx->camera_state != (u32)state) {
        ctx->camera_state = (u32)state;
        queue_work_on(0x20, system_wq, &ctx->work);
    }
    return 0;
}

/* HWMON Sysfs Attribute Show Engines */
static ssize_t display_rate_show(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    struct lenovo_thermal_ctx *ctx = dev_get_drvdata(dev);

    return sprintf(buf, "%u\n", ctx->display_rate_state);
}

static ssize_t speaker_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct lenovo_thermal_ctx *ctx = dev_get_drvdata(dev);

    return sprintf(buf, "%u\n", ctx->speaker_state);
}

static ssize_t modem_5g_show(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    struct lenovo_thermal_ctx *ctx = dev_get_drvdata(dev);

    return sprintf(buf, "%u\n", ctx->modem5g_state);
}

static ssize_t camera_show(struct device *dev,
               struct device_attribute *attr, char *buf)
{
    struct lenovo_thermal_ctx *ctx = dev_get_drvdata(dev);

    return sprintf(buf, "%u\n", ctx->camera_state);
}

static umode_t lenovo_thermal_attrs_visible(struct kobject *kobj,
                        struct attribute *attr, int n)
{
    return attr->mode;
}

static DEVICE_ATTR_RO(display_rate);
static DEVICE_ATTR_RO(speaker);
static DEVICE_ATTR_RO(modem_5g);
static DEVICE_ATTR_RO(camera);

static struct attribute *lenovo_thermal_attrs[] = {
    &dev_attr_display_rate.attr,
    &dev_attr_speaker.attr,
    &dev_attr_modem_5g.attr,
    &dev_attr_camera.attr,
    NULL,
};

static const struct attribute_group lenovo_thermal_group = {
    .attrs      = lenovo_thermal_attrs,
    .is_visible = lenovo_thermal_attrs_visible,
};

static const struct attribute_group *lenovo_thermal_groups[] = {
    &lenovo_thermal_group,
    NULL,
};

static int lenovo_thermal_read_dt_u32_array(struct device *dev,
                        struct device_node *np,
                        const char *propname,
                        u32 **out_table,
                        u32 *out_max)
{
    int count;
    u32 *table;
    int ret;
    int i;

    count = of_property_count_elems_of_size(np, propname, sizeof(u32));
    if (count < 1) {
        dev_err(dev, "Wrong data!\n");
        return (count == 0) ? -EINVAL : count;
    }

    table = devm_kmalloc(dev, count * sizeof(u32), GFP_KERNEL);
    if (!table)
        return -ENOMEM;

    ret = of_property_read_variable_u32_array(np, propname, table,
                          count, 0);
    if (ret < 0) {
        dev_err(dev, "Property '%s' cannot be read!\n", propname);
        return ret;
    }

    for (i = 0; i < count; i++)
        dev_err(dev, "%s state[%d]:%d\n", propname, i, table[i]);

    *out_table = table;
    *out_max   = (u32)(count - 1);
    return count;
}

static int lenovo_thermal_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct device_node *np = pdev->dev.of_node;
    struct lenovo_thermal_ctx *ctx;
    struct device *hwmon_dev;
    struct thermal_cooling_device *tcdev;
    int ret;

    dev_err(dev, "%s begin.\n", __func__);

    ctx = devm_kmalloc(dev, sizeof(*ctx), GFP_KERNEL);
    if (!ctx)
        return -ENOMEM;

    mutex_init(&ctx->lock);
    platform_set_drvdata(pdev, ctx);
    ctx->pdev = dev;

    hwmon_dev = devm_hwmon_device_register_with_groups(dev,
                    "lenovo_thermal", ctx,
                    lenovo_thermal_groups);
    if (IS_ERR(hwmon_dev)) {
        dev_err(dev, "Failed to register lenovo thermal hwmon device\n");
        return PTR_ERR(hwmon_dev);
    }
    ctx->hwmon_dev = hwmon_dev;

    ret = lenovo_thermal_read_dt_u32_array(dev, np,
                           LENOVO_THERMAL_DT_DISPLAY_RATE,
                           &ctx->display_rate_table,
                           &ctx->display_rate_max);
    if (ret < 0)
        return ret;

    ret = lenovo_thermal_read_dt_u32_array(dev, np,
                           LENOVO_THERMAL_DT_SPEAKER,
                           &ctx->speaker_table,
                           &ctx->speaker_max);
    if (ret < 0)
        return ret;

    ret = lenovo_thermal_read_dt_u32_array(dev, np,
                           LENOVO_THERMAL_DT_MODEM_5G,
                           &ctx->modem5g_table,
                           &ctx->modem5g_max);
    if (ret < 0)
        return ret;

    ret = lenovo_thermal_read_dt_u32_array(dev, np,
                           LENOVO_THERMAL_DT_CAMERA,
                           &ctx->camera_table,
                           &ctx->camera_max);
    if (ret < 0)
        return ret;

    ctx->display_rate_state = 0;
    tcdev = devm_thermal_of_cooling_device_register(dev, np,
                LENOVO_CDEV_DISPLAY_RATE, ctx,
                &display_rate_cooling_ops);
    if (IS_ERR(tcdev)) {
        dev_err(dev, "Failed to register display rate as cooling device: %d\n",
            (int)PTR_ERR(tcdev));
        return PTR_ERR(tcdev);
    }
    ctx->tcdev = tcdev;
    thermal_cdev_update(tcdev);

    ctx->speaker_state = 0;
    tcdev = devm_thermal_of_cooling_device_register(dev, np,
                LENOVO_CDEV_SPEAKER, ctx,
                &speaker_cooling_ops);
    if (IS_ERR(tcdev)) {
        dev_err(dev, "Failed to register speaker as cooling device: %d\n",
            (int)PTR_ERR(tcdev));
        return PTR_ERR(tcdev);
    }
    ctx->tcdev = tcdev;
    thermal_cdev_update(tcdev);

    ctx->modem5g_state = 0;
    tcdev = devm_thermal_of_cooling_device_register(dev, np,
                LENOVO_CDEV_MODEM_5G, ctx,
                &modem_5g_cooling_ops);
    if (IS_ERR(tcdev)) {
        dev_err(dev, "Failed to register modem 5g as cooling device: %d\n",
            (int)PTR_ERR(tcdev));
        return PTR_ERR(tcdev);
    }
    ctx->tcdev = tcdev;
    thermal_cdev_update(tcdev);

    ctx->camera_state = 0;
    tcdev = devm_thermal_of_cooling_device_register(dev, np,
                LENOVO_CDEV_CAMERA, ctx,
                &camera_cooling_ops);
    if (IS_ERR(tcdev)) {
        dev_err(dev, "Failed to register camera as cooling device: %d\n",
            (int)PTR_ERR(tcdev));
        return PTR_ERR(tcdev);
    }
    ctx->tcdev = tcdev;
    thermal_cdev_update(tcdev);

    INIT_WORK(&ctx->work, lenovo_thermal_work);
    BLOCKING_INIT_NOTIFIER_HEAD(&ctx->notifier);

    ctx->nb.notifier_call = lenovo_thermal_uevent_notify;
    blocking_notifier_chain_register(&ctx->notifier, &ctx->nb);

    dev_err(dev, "%s end.\n", __func__);
    return 0;
}

static int __maybe_unused lenovo_thermal_suspend(struct device *dev)
{
    return 0;
}

static int __maybe_unused lenovo_thermal_resume(struct device *dev)
{
    return 0;
}

static SIMPLE_DEV_PM_OPS(lenovo_thermal_pm_ops,
              lenovo_thermal_suspend,
              lenovo_thermal_resume);

static const struct of_device_id lenovo_thermal_of_match[] = {
    { .compatible = "lenovo,thermal" },
    { },
};
MODULE_DEVICE_TABLE(of, lenovo_thermal_of_match);

static struct platform_driver lenovo_thermal_driver = {
    .probe  = lenovo_thermal_probe,
    .driver = {
        .name           = "lenovo_thermal",
        .of_match_table = lenovo_thermal_of_match,
        .pm             = &lenovo_thermal_pm_ops,
    },
};

module_platform_driver(lenovo_thermal_driver);

MODULE_DESCRIPTION("Lenovo Thermal Management Driver");
MODULE_LICENSE("GPL v2");