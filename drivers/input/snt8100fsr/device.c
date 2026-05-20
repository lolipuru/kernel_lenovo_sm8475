/*****************************************************************************
* File: device.c
*
* (c) 2016 Sentons Inc. - All Rights Reserved.
*
* All information contained herein is and remains the property of Sentons
* Incorporated and its suppliers if any. The intellectual and technical
* concepts contained herein are proprietary to Sentons Incorporated and its
* suppliers and may be covered by U.S. and Foreign Patents, patents in
* process, and are protected by trade secret or copyright law. Dissemination
* of this information or reproduction of this material is strictly forbidden
* unless prior written permission is obtained from Sentons Incorporated.
*
* SENTONS PROVIDES THIS SOURCE CODE STRICTLY ON AN "AS IS" BASIS,
* WITHOUT ANY WARRANTY WHATSOEVER, AND EXPRESSLY DISCLAIMS ALL
* WARRANTIES, EXPRESS, IMPLIED OR STATUTORY WITH REGARD THERETO, INCLUDING
* THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
* PURPOSE, TITLE OR NON-INFRINGEMENT OF THIRD PARTY RIGHTS. SENTONS SHALL
* NOT BE LIABLE FOR ANY DAMAGES SUFFERED BY YOU AS A RESULT OF USING,
* MODIFYING OR DISTRIBUTING THIS SOFTWARE OR ITS DERIVATIVES.
*****************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pinctrl/consumer.h>
#include <linux/delay.h>

#include "device.h"
#include "utils.h"
#include "hardware.h"
#include "config.h"
#include "debug.h"
#include "sysfs.h"
#include "irq.h"

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/

/*==========================================================================*/
/* CONSTANTS                                                                */
/*==========================================================================*/

/*==========================================================================*/
/* GLOBALS                                                                  */
/*==========================================================================*/
struct snt8100fsr *snt8100fsr_g; // The first and main device and sysFS device
struct snt8100fsr *snt8100fsr_wake_i2c_g; // The i2c wakeup device

/*==========================================================================*/
/* LOCAL PROTOTYPES                                                         */
/*==========================================================================*/
#if USE_DEVICE_TREE_CONFIG
static int snt_request_named_gpio(struct snt8100fsr *snt8100fsr,
                                  const char *label,
                                  int *gpio);
#ifdef PINCTRL_DEVICE_TREE
static int select_pin_ctl(struct snt8100fsr *snt8100fsr,
                          const char *name);
#endif
#endif

/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/

/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/
/*
 * Device initialization using device tree node configuration
 */
#if USE_DEVICE_TREE_CONFIG // config.h =======================================
int snt_spi_device_init(struct spi_device *spi,
                        struct snt8100fsr *snt8100fsr)
{
    int ret = 0;
    u32 data;
#ifdef PINCTRL_DEVICE_TREE
    int i;
#endif
    struct device *dev = &spi->dev;
    struct device_node *np = dev->of_node;

    // Check device tree
    if (!np) {
        PRINT_ERR("no of node found");
        return -EINVAL;
    }

    // Request gpio
    snt_request_named_gpio(snt8100fsr, "snt,gpio_hostirq", &snt8100fsr->hostirq_gpio);

    ret = of_property_read_u32(np, "snt,spi-freq-hz", &data);
    if(ret < 0) {
        PRINT_ERR("snt,spi-freq-hz not found\n");
    } else {
        snt8100fsr->spi_freq_hz = data;
    }
    PRINT_INFO("spi_freq_hz %d\n", snt8100fsr->spi_freq_hz);

#ifdef PINCTRL_DEVICE_TREE
    // Request pinctrl
    snt8100fsr->snt_pinctrl = devm_pinctrl_get(dev);
    if(IS_ERR(snt8100fsr->snt_pinctrl)) {
        if(PTR_ERR(snt8100fsr->snt_pinctrl) == -EPROBE_DEFER)
        {
            PRINT_INFO("pinctrl not ready\n");
            return -EPROBE_DEFER;
        }
        PRINT_ERR("Target does not use pinctrl\n");
        snt8100fsr->snt_pinctrl = NULL;
        return  -EINVAL;
    }

    for(i = 0;i < ARRAY_SIZE(snt8100fsr->pinctrl_state);i++) {
        const char *n = pctl_names[i];
        struct pinctrl_state *state =
            pinctrl_lookup_state(snt8100fsr->snt_pinctrl, n);
        if (IS_ERR(state)) {
            PRINT_ERR("cannot find '%s'\n", n);
            return -EINVAL;
        }
        PRINT_INFO("found pin control %s\n", n);
        snt8100fsr->pinctrl_state[i] = state;
    }

    ret = select_pin_ctl(snt8100fsr, "snt_reset_reset");
    if(ret)
        return ret;

    ret = select_pin_ctl(snt8100fsr, "snt_hostirq_active");
    if(ret)
        return ret;
#else
    snt_request_named_gpio(snt8100fsr,
                           "snt,gpio_rst",
                           &snt8100fsr->rst_gpio);

    gpio_direction_output(snt8100fsr->rst_gpio, 1);
    snt8100fsr->snt_pinctrl = NULL;
#endif

    device_init_wakeup(snt8100fsr->dev, 1);
    mutex_init(&snt8100fsr->track_report_sysfs_lock);
    mutex_init(&snt8100fsr->sb_lock);

    //ret =  devm_request_threaded_irq(dev, gpio_to_irq(snt8100fsr->hostirq_gpio),
    //    NULL, fpc1020_irq_handler, IRQF_TRIGGER_RISING | IRQF_ONESHOT | IRQF_NO_SUSPEND,
    //    dev_name(dev), snt8100fsr);

    // Request that the interrupt should be wakeable
    //enable_irq_wake(gpio_to_irq(snt8100fsr->hostirq_gpio));

    //ret = sysfs_create_group(&dev->kobj, &attribute_group);
    //if(ret)
    //    return ret;

    snt8100fsr->frame_rate = DEFAULT_FRAME_RATE;
    snt8100fsr->suspended_frame_rate = DEFAULT_SUSPENDED_FRAME_RATE;

    //FIXME Remove if no need
#ifdef PINCTRL_DEVICE_TREE
    PRINT_INFO("Enabling hardware\n");
    mutex_lock(&snt8100fsr->sb_lock);
    (void)select_pin_ctl(snt8100fsr, "snt_reset_active");
    usleep_range(100, 900);
    (void)select_pin_ctl(snt8100fsr, "snt_reset_reset");
    usleep_range(100, 100);
    mutex_unlock(&snt8100fsr->sb_lock);
#else
    PRINT_INFO("Enabling hardware\n");
    mutex_lock(&snt8100fsr->sb_lock);
    gpio_direction_output(snt8100fsr->rst_gpio, 0);
    usleep_range(100, 900);
    gpio_direction_output(snt8100fsr->rst_gpio, 1);
    usleep_range(100, 100);
    mutex_unlock(&snt8100fsr->sb_lock);

#endif

    return ret;
}

static int snt_request_named_gpio(struct snt8100fsr *snt8100fsr,
                                  const char *label,
                                  int *gpio)
{
    struct device *dev = snt8100fsr->dev;
    struct device_node *np = dev->of_node;
    int rc = of_get_named_gpio(np, label, 0);
    if (rc < 0) {
        PRINT_ERR("failed to get '%s'", label);
        return rc;
    }
    *gpio = rc;
    rc = devm_gpio_request(dev, *gpio, label);
    if (rc) {
        PRINT_ERR("failed to request gpio %d", *gpio);
        return rc;
    }
    PRINT_DEBUG("%s %d", label, *gpio);
    return 0;
}

#ifdef PINCTRL_DEVICE_TREE
static int select_pin_ctl(struct snt8100fsr *snt8100fsr, const char *name)
{
    size_t i;
    int ret;
    struct device *dev = snt8100fsr->dev;
    for (i = 0; i < ARRAY_SIZE(snt8100fsr->pinctrl_state); i++) {
        const char *n = pctl_names[i];
        if (!strncmp(n, name, strlen(n))) {
            ret = pinctrl_select_state(snt8100fsr->snt_pinctrl,
                    snt8100fsr->pinctrl_state[i]);
            if (ret)
                PRINT_ERR("cannot select '%s'\n", name);
            else
                PRINT_INFO("Selected '%s'\n", name);
            goto exit;
        }
    }
    ret = -EINVAL;
    PRINT_ERR("%s:'%s' not found\n", __func__, name);

exit:
    return ret;
}
#endif

int snt_i2c_device_init(struct i2c_client *i2c,
                        struct snt8100fsr *snt8100fsr)
{
    int ret = 0, data;
    struct device *dev = &i2c->dev;
    struct device_node *np = dev->of_node;
#ifdef PINCTRL_DEVICE_TREE
    int i;
#endif

    // Check device tree
    if (!np) {
        PRINT_ERR("no of node found");
        return -EINVAL;
    }

#ifdef PINCTRL_DEVICE_TREE
    // Request gpio
    snt_request_named_gpio(snt8100fsr,
                           "snt,gpio_hostirq",
                           &snt8100fsr->hostirq_gpio);
    snt_request_named_gpio(snt8100fsr,
                           "snt,gpio_trig0irq",
                           &snt8100fsr->trig0irq_gpio);
    snt_request_named_gpio(snt8100fsr,
                           "snt,gpio_trig1irq",
                           &snt8100fsr->trig1irq_gpio);
    snt_request_named_gpio(snt8100fsr,
                           "snt,gpio_trig2irq",
                           &snt8100fsr->trig2irq_gpio);
    // Request pinctrl
    snt8100fsr->snt_pinctrl = devm_pinctrl_get(dev);
    if(IS_ERR(snt8100fsr->snt_pinctrl)) {
        if(PTR_ERR(snt8100fsr->snt_pinctrl) == -EPROBE_DEFER)
        {
            PRINT_INFO("pinctrl not ready\n");
            return -EPROBE_DEFER;
        }
        PRINT_ERR("Target does not use pinctrl\n");
        snt8100fsr->snt_pinctrl = NULL;
        return  -EINVAL;
    }

    for(i = 0;i < ARRAY_SIZE(snt8100fsr->pinctrl_state);i++) {
        const char *n = pctl_names[i];
        struct pinctrl_state *state =
            pinctrl_lookup_state(snt8100fsr->snt_pinctrl, n);
        if (IS_ERR(state)) {
            PRINT_ERR("cannot find '%s'\n", n);
            return -EINVAL;
        }
        PRINT_INFO("found pin control %s\n", n);
        snt8100fsr->pinctrl_state[i] = state;
    }

    ret = select_pin_ctl(snt8100fsr, "snt_reset_normal");
    if(ret)
        return ret;
    ret = select_pin_ctl(snt8100fsr, "snt_hostirq_active");
    if(ret)
        return ret;
#else
    // Request gpio
    snt_request_named_gpio(snt8100fsr, "snt,gpio_hostirq", &snt8100fsr->hostirq_gpio);
    snt_request_named_gpio(snt8100fsr, "snt,gpio_trig0irq", &snt8100fsr->trig0irq_gpio);
    snt_request_named_gpio(snt8100fsr, "snt,gpio_trig1irq", &snt8100fsr->trig1irq_gpio);
    snt_request_named_gpio(snt8100fsr, "snt,gpio_trig2irq", &snt8100fsr->trig2irq_gpio);
    snt_request_named_gpio(snt8100fsr, "snt,gpio_rst", &snt8100fsr->rst_gpio);
    snt_request_named_gpio(snt8100fsr, "snt,gpio_ldo", &snt8100fsr->ldo_gpio);

    gpio_direction_output(snt8100fsr->ldo_gpio, 1);

    snt8100fsr->snt_pinctrl = NULL;
#endif

    snt8100fsr->raw_report = false;

    ret = of_property_read_u32(np, "snt,i2c-freq-khz", &data);
    if(ret < 0) {
        PRINT_ERR("snt,i2c-freq-khz not found\n");
    } else {
        snt8100fsr->i2c_freq_khz = data;
    }
    PRINT_INFO("i2c_freq_khz %u\n", snt8100fsr->i2c_freq_khz);

    // Set the frame_rate
    snt8100fsr->frame_rate = DEFAULT_FRAME_RATE;
    snt8100fsr->suspended_frame_rate = DEEP_SLEEP_FRAME_RATE;

    device_init_wakeup(snt8100fsr->dev, 1);
    mutex_init(&snt8100fsr->track_report_sysfs_lock);
    mutex_init(&snt8100fsr->sb_lock);

#ifdef PINCTRL_DEVICE_TREE
    PRINT_INFO("Enabling hardware\n");
    mutex_lock(&snt8100fsr->sb_lock);
    (void)select_pin_ctl(snt8100fsr, "snt_reset_normal");
    mutex_unlock(&snt8100fsr->sb_lock);
#else
    PRINT_INFO("Enabling hardware\n");
    mutex_lock(&snt8100fsr->sb_lock);
    gpio_direction_output(snt8100fsr->rst_gpio, 1);
    mutex_unlock(&snt8100fsr->sb_lock);
#endif

    return ret;
}
#else // ---------------------------------------------------------------------
/*
 * Device initialization using the CONFIG.H file
 */
int snt_spi_device_init(struct spi_device *spi,
                        struct snt8100fsr *snt8100fsr) {
    struct device *dev = &spi->dev;

    snt8100fsr->hostirq_gpio = BEAGLEBONE_GPIO49;
    snt8100fsr->rst_gpio = 0;
    snt8100fsr->spi_freq_hz = SPI_MAX_SPEED_HZ;
    snt8100fsr->frame_rate = DEFAULT_FRAME_RATE;
    snt8100fsr->suspended_frame_rate = DEFAULT_SUSPENDED_FRAME_RATE;

    PRINT_INFO("spi_freq_hz %d\n", snt8100fsr->spi_freq_hz);

    // Request pinctrl
    snt8100fsr->snt_pinctrl = NULL;

    device_init_wakeup(snt8100fsr->dev, 1);

    mutex_init(&snt8100fsr->track_report_sysfs_lock);
    mutex_init(&snt8100fsr->sb_lock);
    return 0;
}

int snt_i2c_device_init(struct i2c_client *i2c,
                        struct snt8100fsr *snt8100fsr) {
    snt8100fsr->hostirq_gpio = BEAGLEBONE_GPIO49;
    snt8100fsr->rst_gpio = 0;
    snt8100fsr->frame_rate = DEFAULT_FRAME_RATE;
    snt8100fsr->suspended_frame_rate = DEFAULT_SUSPENDED_FRAME_RATE;

    // Request pinctrl
    snt8100fsr->snt_pinctrl = NULL;

    device_init_wakeup(snt8100fsr->dev, 1);

    mutex_init(&snt8100fsr->track_report_sysfs_lock);
    mutex_init(&snt8100fsr->sb_lock);
    return 0;
}
#endif // ====================================================================


static ssize_t snt_spi_test_set(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    int ret = 0;

    PRINT_INFO("%s\n",__func__);

    return ret? ret:count;
}

static DEVICE_ATTR(snt_spi_test, S_IWUSR, NULL, snt_spi_test_set);

static struct attribute *attributes[] = {
    &dev_attr_snt_spi_test.attr,
    NULL
};

static const struct attribute_group attribute_group = {
    .attrs = attributes,
};

int snt_suspend(struct device *dev)
{
    int ret;

    /*PRINT_FUNC();*/
    if (snt8100fsr_g->sys_suspend == true) {
        PRINT_INFO("Already suspend before!!");
        return 0;
    }

#ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request_force(0 /* No Force */ ) != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return 0;
    }
    VERBOSE("Setting frame rate to %d", snt8100fsr_g->suspended_frame_rate);
    ret = write_register(snt8100fsr_g,
                         REGISTER_FRAME_RATE,
                         &snt8100fsr_g->suspended_frame_rate);
#endif

    if (snt_irq_db.irq_num != 0 && snt8100fsr_g->sys_active_irq == true) {
        disable_irq(snt_irq_db.irq_num);
        snt8100fsr_g->sys_active_irq = false;
    }

    snt8100fsr_g->sys_suspend = true;
    PRINT_FUNC("done");
    return 0;
}

int snt_resume(struct device *dev)
{
    /*PRINT_FUNC();*/
    if (snt_irq_db.irq_num != 0 && snt8100fsr_g->sys_active_irq == false) {
        enable_irq(snt_irq_db.irq_num);
        snt8100fsr_g->sys_active_irq = true;
    }

    if (snt8100fsr_g->sys_suspend == false) {
        PRINT_INFO("Already resume before!!");
        return 0;
    }

#ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request_force(0 /* No Force */ ) != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return 0;
    }
#endif

    snt8100fsr_g->sys_suspend = false;
    PRINT_FUNC("done");
    return 0;
}

int snt_device_hw_reset(struct snt8100fsr *snt8100fsr)
{
    PRINT_FUNC();

    gpio_direction_output(snt8100fsr->rst_gpio, 0);
    msleep(10);
    gpio_direction_output(snt8100fsr->rst_gpio, 1);
    msleep(10);
    PRINT_INFO("Chip reset, wait for interrupt");

    PRINT_DEBUG("done");
    return 0;
}
