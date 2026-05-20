/*****************************************************************************
* File: sysfs.c
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
#include <linux/kernel.h>
#include <linux/module.h>

#include "config.h"
#include "hardware.h"
#include "memory.h"
#include "file.h"
#include "event.h"

#include "sonacomm.h"
#include "utils.h"
#include "device.h"
#include "debug.h"
#include "customize.h"
#include "sysfs.h"

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/
/*#define TAP_OFFSET_TRIGGER      (2)*/
#define TAP_OFFSET_FORCELIMIT   (0)
/*#define TAP_OFFSET_ENABLE       (0)*/
#define TAP0_REG_BASE           (8)
#define TAP1_REG_BASE           (28)
#define TAP2_REG_BASE           (48)
#define TAP3_REG_BASE           (68)

// L1 region
#define L1_REG_FORCE          (TAP3_REG_BASE + TAP_OFFSET_FORCELIMIT)
// L2 region
#define L2_REG_FORCE          (TAP2_REG_BASE + TAP_OFFSET_FORCELIMIT)
// R1 region
#define R1_REG_FORCE          (TAP0_REG_BASE + TAP_OFFSET_FORCELIMIT)
// R2 region
#define R2_REG_FORCE          (TAP1_REG_BASE + TAP_OFFSET_FORCELIMIT)

#define TAIA_REG_ADDR        (0x1a)
#define TAIA_TRIGGER_PIN        (7)
#define TAIA_L1_TAP_VIB         (3)
#define TAIA_L1_FS_VIB          (4)
#define TAIA_L2_TAP_VIB         (2)
#define TAIA_L2_FS_VIB         (13)

#define TAIB_REG_ADDR        (0x1b)
#define TAIB_TRIGGER_PIN        (1)
#define TAIB_R1_TAP_VIB        (12)
#define TAIB_R1_FS_VIB          (5)
#define TAIB_R2_TAP_VIB        (14)
#define TAIB_R2_FS_VIB         (15)
/*==========================================================================*/
/* CONSTANTS                                                                */
/*==========================================================================*/

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/
enum CMD_ID { 
    CMD_SET_TOP_FORCE_THRESHOLD = 1,  // Deprecated, DON'T USE IT.
    CMD_SET_BOT_FORCE_THRESHOLD = 2,  // Deprecated, DON'T USE IT.
    CMD_ENABLE_GAMING_VIB = 3,
    CMD_ENABLE_GAMING_BAR = 4,
    CMD_SET_VOLUME_FORCE_LVL = 5, // Deprecated, DON'T USE IT.
    CMD_SET_PROFILE = 6,
    CMD_MAX
};

uint8_t bar_id_mapper[4] = {0};
/*==========================================================================*/
/* LOCAL PROTOTYPES                                                         */
/*==========================================================================*/
int get_attrs_array_size(struct attribute *list[]);
struct attribute **alloc_sysfs_attrs(void);
void free_sysfs_attrs(struct attribute **attrs);
int dump_deep_trace_to_file(void);

/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/
static struct attribute **sysfs_attrs_g;
static struct attribute_group attr_group_g;
#ifdef SUPPORT_SYSFS_CLASS
static int sntdev_major_num;
static struct class *sntdev_device_class;
static dev_t dev_no;
static struct device *device_ptr;
#else
 static struct   kobject *sysfs_kobj_g;
#endif

volatile int snt8100_flagVerbose = 0;

static uint16_t snt_profile_slider[][6] = {
    {0xc001 , 0 , 0xffff , 0x1827 , 0x0006 , 0}, // R1
    {0      , 0 , 0      , 0      , 0      , 0},
    {0      , 0 , 0      , 0      , 0      , 0},
    {0xc301 , 0 , 0xffff , 0x1827 , 0x0006 , 0}  // L1
};

static uint16_t snt_profile_gas[][6] = {
    {0x900d , 0 , 0xffff , 0x0a00 , 0x0002 , 0}, // R1
    {0x911f , 0 , 0x0035 , 0x0a00 , 0x0002 , 0}, // R2
    {0x921b , 0 , 0x0035 , 0x0a00 , 0x0002 , 0}, // L2
    {0x930b , 0 , 0xffff , 0x0a00 , 0x0002 , 0}  // L1
};
/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/

#ifdef DYNAMIC_PWR_CTL
/*
 * Utility function to perform Activity Request and wait for completion.
 *
 * set Request sema.
 * perform Activity Request. Back out Sema request if fails.
 * wait for Response sema.
 */
int snt_activity_request(void)
{
    return snt_activity_request_force(0 /* No Force */ );
}

ssize_t sysfs_enable_dynamic_pwr_ctl_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count) {
    PRINT_FUNC("%zu bytes", count);
    if (count != 2) {
        PRINT_CRIT("Only single digit number '1' or '0' allowed");
        return -1;
    }
    mutex_lock(&snt8100fsr_g->sb_lock);

    snt8100fsr_g->enable_dpc_flag = (*buf == '0') ? 0 : 1;

    mutex_unlock(&snt8100fsr_g->sb_lock);
    PRINT_INFO("done. enable_dpc = %d", snt8100fsr_g->enable_dpc_flag);
    return count;
}

static ssize_t sysfs_enable_dynamic_pwr_ctl_show(struct device *dev,
                            struct device_attribute *attr,
                            char *buf) {
    int ret;
    PRINT_FUNC();
    ret = snprintf(buf, PAGE_SIZE, "%d\n", snt8100fsr_g->enable_dpc_flag);
    PRINT_DEBUG("done (%d)", ret);
    return ret;
}




#endif /* DYNAMIC_PWR_CTL */

static ssize_t verbose_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    printk("[EDGE] %s\n", __func__);

    return snprintf(buf, 50, "%d\n", snt8100_flagVerbose);
}

static ssize_t verbose_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    sscanf(buf, "%d\n", &snt8100_flagVerbose);
    printk("[EDGE] %s, parsed %d\n", __func__, snt8100_flagVerbose);

    return count;
}

static ssize_t show_raw_report_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
    printk("[EDGE] %s\n", __func__);

    return snprintf(buf, 50, "%d\n", snt8100fsr_g->raw_report);
}

static ssize_t set_raw_report_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int mode, val, gst_enabled = 0;
    sscanf(buf, "%d\n", &mode);
    printk("[EDGE] %s, parsed %d\n", __func__, mode);

    if (snt8100fsr_g->status_fwd != FWUPDATE_SUCCESS) {
        PRINT_CRIT("Sona NOT Ready!!");
        return count;
    }

    if (mode) {
        snt8100fsr_g->raw_report = true;
        val = 0;
    } else {
        snt8100fsr_g->raw_report = false;
        val = 8;
    }

#ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request() != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return count;
    }
#endif
    mutex_lock(&snt8100fsr_g->sb_lock);
    // Disabel gesture processing.
    if (sb_write_register(snt8100fsr_g, 0x01, &gst_enabled)) 
        goto err_handler;

    if (sb_write_register(snt8100fsr_g, 0x40, &val))
        PRINT_CRIT("failed to write 0x40 to %d", val);

    // Enable gesture processing again.
    gst_enabled = 1;
    if (sb_write_register(snt8100fsr_g, 0x01, &gst_enabled)) {
        PRINT_CRIT("Failed to re-enable gesture processing");
    }

err_handler:
    mutex_unlock(&snt8100fsr_g->sb_lock);

    return count;
}

static ssize_t set_tap_force_threshold(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    uint8_t force_threshold, bar_id;
    uint16_t tap_ctrl = 0, addr = 0;

    sscanf(buf, "%d %d\n", &bar_id, &force_threshold);
    PRINT_INFO("parsed %d, %d", bar_id, force_threshold);

    if (snt8100fsr_g->status_fwd != FWUPDATE_SUCCESS) {
        PRINT_CRIT("Sona NOT Ready!!");
        return count;
    }

    if (bar_id > TOP_L_BAR_ID) 
        return count;

    switch (bar_id) {
    case BOT_R_BAR_ID:
        addr = R1_REG_FORCE; break;
    case BOT_L_BAR_ID:
        addr = R2_REG_FORCE; break;
    case TOP_R_BAR_ID:
        addr = L2_REG_FORCE; break;
    case TOP_L_BAR_ID:
        addr = L1_REG_FORCE; break;
    }

#ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request() != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return count;
    }
#endif

    if (!snt8100_read_cfgbank_reg(8, addr, &tap_ctrl)) {
        VERBOSE("OLD: 0x%x -> 0x%x", addr, tap_ctrl);
        tap_ctrl = (tap_ctrl & 0xff) | force_threshold<<8;
        VERBOSE("NEW: 0x%x -> 0x%x", addr, tap_ctrl);

        snt8100_write_cfgbank_reg(8, addr, tap_ctrl);
    }

    return count;
}

static int enable_gamingbar_vib(uint16_t optarg)
{
    uint16_t tap_ctrl = 0;
    int ret = 0;

    mutex_lock(&snt8100fsr_g->sb_lock);
    if (optarg <= 1) {
        // Control TAIA register
        if (!sb_read_register(snt8100fsr_g, TAIA_REG_ADDR, &tap_ctrl)) {
            VERBOSE("top bar old ctrl=0x%x", tap_ctrl);
            if (optarg)
                tap_ctrl |= TAIA_TRIGGER_PIN<<8;
            else
                tap_ctrl &= 0xF0FF; 
            VERBOSE("top bar new ctrl=0x%x", tap_ctrl);
            sb_write_register(snt8100fsr_g, TAIA_REG_ADDR, &tap_ctrl);
        }

        // Control TAIB register
        if (!sb_read_register(snt8100fsr_g, TAIB_REG_ADDR, &tap_ctrl)) {
            VERBOSE("bot bar old ctrl=0x%x", tap_ctrl);
            if (optarg)
                tap_ctrl |= TAIB_TRIGGER_PIN<<8;
            else
                tap_ctrl &= 0xF0FF; 
            VERBOSE("bot bar new ctrl=0x%x", tap_ctrl);
            sb_write_register(snt8100fsr_g, TAIB_REG_ADDR, &tap_ctrl);
        }
    } else {
        uint8_t gesture_id = optarg/100, bar_id = (optarg%100)/10, action = optarg%10;
        uint16_t pos = 32, addr;
        if (ZUI_TAP == gesture_id) {
            switch (bar_id) {
            case BOT_R_BAR_ID: pos = TAIB_R1_TAP_VIB; addr = TAIB_REG_ADDR; break;
            case BOT_L_BAR_ID: pos = TAIB_R2_TAP_VIB; addr = TAIB_REG_ADDR; break;
            case TOP_R_BAR_ID: pos = TAIA_L2_TAP_VIB; addr = TAIA_REG_ADDR; break;
            case TOP_L_BAR_ID: pos = TAIA_L1_TAP_VIB; addr = TAIA_REG_ADDR; break;
            }
        } else if (ZUI_GAS == gesture_id) {
            switch (bar_id) {
            case BOT_R_BAR_ID: pos = TAIB_R1_FS_VIB; addr = TAIB_REG_ADDR; break;
            case BOT_L_BAR_ID: pos = TAIB_R2_FS_VIB; addr = TAIB_REG_ADDR; break;
            case TOP_R_BAR_ID: pos = TAIA_L2_FS_VIB; addr = TAIA_REG_ADDR; break;
            case TOP_L_BAR_ID: pos = TAIA_L1_FS_VIB; addr = TAIA_REG_ADDR; break;
            }
        }
        if (pos < 16) {
            if (!sb_read_register(snt8100fsr_g, addr, &tap_ctrl)) {
                PRINT_INFO("trigger old ctrl=0x%x", tap_ctrl);
                if (action)
                    tap_ctrl |= 1<<pos;
                else
                    tap_ctrl &= ~(1<<pos); 
                PRINT_INFO("trigger new ctrl=0x%x", tap_ctrl);
                sb_write_register(snt8100fsr_g, addr, &tap_ctrl);
            }
        } else {
            PRINT_WARN("Invalid params.");
        }
    }
    mutex_unlock(&snt8100fsr_g->sb_lock);

    return ret;
}

static int set_profile(uint8_t optarg)
{
    int ret = -1;

    uint8_t gesture_id = optarg/10, bar_id = optarg%10;
    PRINT_INFO("Set bar %d to func %d.", bar_id, gesture_id);

    if (bar_id > TOP_L_BAR_ID)
        return ret;

    set_touch_enable(snt8100fsr_g, 0);

    if (bar_id == TOP_L_BAR_ID || bar_id == BOT_R_BAR_ID) {
        switch (gesture_id) {
        case ZUI_TAP: // enable tap gesture.
            set_tap_gesture_enable(snt8100fsr_g, bar_id, 1);
            break;
        case ZUI_SLIDER:
            set_tap_gesture_enable(snt8100fsr_g, bar_id, 0); // disable this for slider performance
            set_slider_gesture(snt8100fsr_g, bar_id, snt_profile_slider[bar_id]);
            if (!is_gas_gesture(bar_id)) {
                ret = 0;
                if (TOP_L_BAR_ID == bar_id)
                    snt8100fsr_g->tap3_slider = true;
                else
                    snt8100fsr_g->tap0_slider = true;
            }
            break;
        case ZUI_GAS:
            set_slider_gesture(snt8100fsr_g, bar_id, snt_profile_gas[bar_id]);
            if (is_gas_gesture(bar_id)) {
                ret = 0;
                if (TOP_L_BAR_ID == bar_id)
                    snt8100fsr_g->tap3_slider = false;
                else
                    snt8100fsr_g->tap0_slider = false;
            }
            break;
        }
    }

    set_touch_enable(snt8100fsr_g, 1);

    return ret;
}

static ssize_t handle_misc_cmd(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    uint16_t arg, cmd_id;

    sscanf(buf, "%d %d\n", &cmd_id, &arg);
    PRINT_INFO("parsed %d, %d", cmd_id, arg);

    if (snt8100fsr_g->status_fwd != FWUPDATE_SUCCESS) {
        PRINT_CRIT("Sona NOT Ready!!");
        return count;
    }

    if (cmd_id >= CMD_MAX) 
        return count;

#ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request() != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return count;
    }
#endif

    switch (cmd_id) {
    case CMD_ENABLE_GAMING_VIB:
        enable_gamingbar_vib(arg); 
        break;
    case CMD_SET_PROFILE:
        set_profile(arg); 
        break;
    default:
        PRINT_INFO("NOT implemented");
    }

    return count;
}

/*
 * SysFS interface for reading and writing a generic 32 bit value
 */
static ssize_t sysfs_uint32_show(struct device *dev,
                                 struct device_attribute *attr,
                                 uint32_t *value,
                                 char *buf) {
    return snprintf(buf, PAGE_SIZE, "%d\n", *value);
}

static ssize_t sysfs_uint32_store(struct device *dev,
                                  struct device_attribute *attr,
                                  uint32_t *result,
                                  const char *buf,
                                  size_t count) {
    long value;

    PRINT_FUNC("%zu bytes", count);

    if (count < 2) {
        return -1;
    }

    if(kstrtol(buf, 10, &value) == 0) {
        PRINT_DEBUG("Storing value %d into result", (uint32_t)value);
        *result = (uint32_t)value;
    } else {
        PRINT_NOTICE("Invalid SysFS Value: %s", buf);
        return -1;
    }

    return count;
}

/*
 * SysFS interface for logging the no touch frame report
 */
ssize_t sysfs_log_no_touch_frame_show(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf) {
    return snprintf(buf, PAGE_SIZE, "%d\n",
                    log_no_touch_frame_file != NULL ? 1 : 0);
}

ssize_t sysfs_log_no_touch_frame_store(struct device *dev,
                                       struct device_attribute *attr,
                                       const char *buf,
                                       size_t count) {
    int ret;
    PRINT_FUNC("%zu bytes", count);

    if (count > 2) {
        return -1;
    }

    if (*buf == '0') {
        PRINT_WARN("Disable no_touch logging not supported. It will auto"
                   "-disable upon completion.");
        return -1;
    } else if (*buf == '1') {

        #ifdef DYNAMIC_PWR_CTL
        if (snt_activity_request() != 0) {
            PRINT_CRIT("snt_activity_request() failed");
            return -1;
        }
        #endif

        ret = enable_no_touch_logging(snt8100fsr_g);
        if (ret) {
            PRINT_CRIT("Enable no_touch logging failed: %d", ret);
            return -1;
        }
    } else {
        PRINT_NOTICE("Invalid SysFS Value: 0x%X", *buf);
        return -1;
    }

    return count;
}

/*
 * SysFS interface to get a copy of the latest track report
 */
ssize_t sysfs_track_report_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf) {
    int i = 0;
    ssize_t ret = 0;

    mutex_lock(&snt8100fsr_g->track_report_sysfs_lock);
    for (i = 0; i < snt8100fsr_g->track_reports_count; i++) {
        ret += snprintf(buf+ret, PAGE_SIZE-ret, "%u %u %u %u %u %u %u\n",
                        snt8100fsr_g->track_reports_frame,
                        snt8100fsr_g->track_reports[i].bar_id,
                        snt8100fsr_g->track_reports[i].trk_id,
                        snt8100fsr_g->track_reports[i].force_lvl,
                        snt8100fsr_g->track_reports[i].top,
                        snt8100fsr_g->track_reports[i].center,
                        snt8100fsr_g->track_reports[i].bottom);
    }

    snt8100fsr_g->track_reports_count = 0;

    mutex_unlock(&snt8100fsr_g->track_report_sysfs_lock);
    return ret;
}

/*
 * SysFS interface for starting and stopping the logging of track reports
 */
ssize_t sysfs_log_track_report_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n",
                    log_track_reports_file != NULL ? 1 : 0);
}

ssize_t sysfs_log_track_report_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
    int ret;

    PRINT_FUNC("%zu bytes", count);

    if (count > 2) {
        return -1;
    }

    #ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request() != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return -1;
    }
    #endif

   if (*buf == '0') {
        ret = enable_track_report_logging(false, 0/*txt*/);
        if (ret) {
            PRINT_CRIT("Unable to disable track report logging: %d", ret);
            return -1;
        }
    } else if (*buf == '1') {
        ret = enable_track_report_logging(true, 0/*txt*/);
        if (ret) {
            PRINT_CRIT("Unable to enable track report logging: %d", ret);
            return -1;
        }
    } else {
        PRINT_NOTICE("Invalid SysFS Value: 0x%X", *buf);
        return -1;
    }

    return count;
}


/*
 * SysFS interface for retrieving the event log
 */
ssize_t sysfs_event_log_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n",
                    snt8100fsr_g->event_log_file != NULL ? 1 : 0);
}

ssize_t sysfs_event_log_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
    int ret;

    PRINT_FUNC("%zu bytes", count);

    if (count > 2) {
        return -1;
    }

    #ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request() != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return -1;
    }
    #endif

    if (*buf == '1') {
        ret = enable_event_logging(snt8100fsr_g);
        if (ret) {
            PRINT_CRIT("Unable to enable event logging: %d", ret);
            return -1;
        }
    } else {
        PRINT_NOTICE("Invalid SysFS Value: 0x%X", *buf);
        return -1;
    }

    // wait for response from driver irpt thread
    PRINT_DEBUG("EventLog Rsp -- wait");
    do {
        ret = down_interruptible(&snt8100fsr_g->sc_wf_rsp);
        PRINT_DEBUG("EventLog Rsp -- acquired %d", ret);
    } while (ret == -EINTR);

    return count;
}


ssize_t sysfs_log_track_report_bin_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n",
                    log_track_reports_bin_file != NULL ? 1 : 0);
}

ssize_t sysfs_log_track_report_bin_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
    int ret;

    PRINT_FUNC("%zu bytes", count);

    if (count > 2) {
        return -1;
    }

    #ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request() != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return -1;
    }
    #endif

    if (*buf == '0') {
        ret = enable_track_report_logging(false, 1/*bin*/);
        if (ret) {
            PRINT_CRIT("Unable to disable track report logging: %d", ret);
            return -1;
        }
    } else if (*buf == '1') {
        ret = enable_track_report_logging(true, 1/*bin*/);
        if (ret) {
            PRINT_CRIT("Unable to enable track report logging: %d", ret);
            return -1;
        }
    } else {
        PRINT_NOTICE("Invalid SysFS Value: 0x%X", *buf);
        return -1;
    }

    return count;
}


ssize_t sysfs_deep_trace_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
    int ret;

    PRINT_FUNC("%zu bytes", count);

    if (count > 2) {
        PRINT_NOTICE("ERROR. Only one byte allowed %zu", count);
        return -1;
    }

    #ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request() != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return -1;
    }
    #endif

    if (*buf == '1') {
        ret = dump_deep_trace_to_file();
        if (ret) {
            PRINT_CRIT("Unable to get deep trace: %d", ret);
            return -1;
        }
    } else {
        PRINT_NOTICE("Invalid SysFS Value: 0x%X", *buf);
        return -1;
    }

    return count;
}


/*
 * SysFS interface for starting and stopping the logging of d1test data
 */
ssize_t sysfs_log_d1test_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf) {
    return snprintf(buf, PAGE_SIZE, "%d %d\n",
                    log_d1test_file != NULL ? 1 : 0,
                    snt8100fsr_g->d1_stream_count);
}

ssize_t sysfs_log_d1test_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf,
                               size_t count) {
    int ret;
    long value;
    PRINT_FUNC("%zu bytes", count);

    if (count > 10) {
        PRINT_CRIT("bad argument size");
        return -1;
    }

    #ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request() != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return -1;
    }
    #endif

    if(kstrtol(buf, 10, &value) == 0) {
        PRINT_INFO("d1test frames %d", (int)value);
        if (value == 1) {
            value = 64*1024;
        }
        ret = enable_d1test_logging(snt8100fsr_g, (int)value);
    } else {
        PRINT_CRIT("Bad Arguement %s", buf);
        return -1;
    }

    PRINT_FUNC("done");
    return count;
}

/*
 * SysFS interface for starting and stopping the logging of frame data
 */
ssize_t sysfs_log_frames_show(struct device *dev,
                             struct device_attribute *attr,
                             char *buf) {
    return snprintf(buf, PAGE_SIZE, "%d %d\n",
                    log_frame_file != NULL ? 1 : 0,
                    snt8100fsr_g->frame_stream_count);
}

ssize_t sysfs_log_frames_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf,
                              size_t count) {
    int ret;
    long value;
    PRINT_FUNC("%zu bytes", count);

    if (count > 10) {
        return -1;
    }

    #ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request() != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return -1;
    }
    #endif

    if(kstrtol(buf, 10, &value) == 0) {
        if (value == 0) {
            ret = enable_frame_logging(snt8100fsr_g, 0);
            if (ret) {
                PRINT_CRIT("Disable frame logging failed: %d", ret);
                return -1;
            }
            return count;
        } else {
            // -1 means log as long as possible.
            if (value == -1) {
                value = 0x7FFFFFFF;
            }

            ret = enable_frame_logging(snt8100fsr_g, (uint32_t)value);
            if (ret) {
                PRINT_CRIT("Enable frame logging failed: %d", ret);
                return -1;
            }
            return count;
        }
    }

    PRINT_NOTICE("Invalid SysFS Value: 0x%X", *buf);
    return -1;
}

/*
 * SysFS interface for product config string
 */
ssize_t sysfs_product_config_show(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf) {
    char *product_string;
    int ret;

    PRINT_FUNC();

    #ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request() != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return -1;
    }
    #endif

    product_string = memory_allocate(PRODUCT_CONFIG_MAX_LEN, 0);
    if (product_string == NULL) {
        PRINT_CRIT("memory_allocate(PRODUCT_CONFIG_MAX_LEN) failed");
        return -1;
    }

    ret = read_product_config(snt8100fsr_g, product_string);
    if (ret) {
        PRINT_WARN("Unable to read product config");
        memory_free(product_string);
        return ret;
    }

    PRINT_FUNC("done");
    ret = snprintf(buf, PAGE_SIZE, "%s\n", product_string);
    memory_free(product_string);
    return ret;
}

/*
 * SysFS interface for version string
 */
ssize_t sysfs_version_show(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf) {

    return snprintf(buf, PAGE_SIZE, "%s\n", SNT_VERSION);
}

/*
 * SysFS interface for firmware update status
 */
ssize_t sysfs_boot_status_show(struct device *dev,
                               struct device_attribute *attr,
                               char *buf) {
    if (snt8100fsr_g->status_fwd == FWUPDATE_YET) {  // Firmware update & retry not finish
        return snprintf(buf, PAGE_SIZE, "-1\n");
    } else {
        if (snt8100fsr_g->status_fwd == FWUPDATE_FAIL) // Firmware update fail
            return snprintf(buf, PAGE_SIZE, "0\n");
        else                                 // Firmware update success
            return snprintf(buf, PAGE_SIZE, "1\n");
    }
}


/*
 * SysFS interface for setting the suspended frame rate.
 * This frame rate will be set when i2c_suspend() or spi_suspend() is called.
 */
ssize_t sysfs_suspended_frame_rate_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf) {
    int ret;
    PRINT_FUNC();
    ret = sysfs_uint32_show(dev, attr,
                            &snt8100fsr_g->suspended_frame_rate, buf);
    PRINT_DEBUG("done");
    return ret;
}

ssize_t sysfs_suspended_frame_rate_store(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf,
                                         size_t count) {
    int ret;
    PRINT_FUNC();
    ret = sysfs_uint32_store(dev, attr,
                             &snt8100fsr_g->suspended_frame_rate,
                             buf, count);
    PRINT_DEBUG("done");
    return ret;
}

/*
 * SysFS interface for waking up the device
 */
ssize_t sysfs_wake_device_store(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf,
                                size_t count) {
    int ret;
    PRINT_FUNC("%zu bytes", count);

    if (count != 2) {
        PRINT_CRIT("Only single digit number '1' or '0' allowed");
        return -1;
    }

    if (*buf == '1') {
        PRINT_DEBUG("Waking device now");

        #ifdef DYNAMIC_PWR_CTL

        ret = snt_activity_request_force(1 /* force */ );
        if (ret) {
            PRINT_CRIT("snt_activity_request() failed %d", ret);
            return -1;
        }

        #else

        mutex_lock(&snt8100fsr_g->sb_lock);
        ret = sb_wake_device(snt8100fsr_g);
        if (ret) {
            PRINT_CRIT("sb_wake_device() failed");
            mutex_unlock(&snt8100fsr_g->sb_lock);
            return -1;
        }
        mutex_unlock(&snt8100fsr_g->sb_lock);

        #endif

    } else if (*buf == '0') {
        PRINT_DEBUG("Do nothing since '0' received");
    } else {
        PRINT_CRIT("Only single digit number '1' or '0' allowed");
        return -1;
    }

    PRINT_DEBUG("done");
    return count;
}

ssize_t do_write_reg_script(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count,
                                           int sc_maj_id) {
    char *p_buf = NULL;
    char *fname = NULL;
    struct file *p_file = NULL;
    int ret;
    int frp_size;
    int bytes;

    PRINT_FUNC("count = %zu", count);

    if (count < 2) {
        PRINT_DEBUG("no file name supplied");
        goto cleanup;
    }
    fname = memory_allocate(count,0);
    if (fname == NULL) {
        PRINT_CRIT("could not allocate memory for fname");
        goto cleanup;
    }
    memcpy(fname, buf, count);
    fname[count-1] = '\0';
    PRINT_INFO("opening frp file %s", fname);
    ret = file_open(fname, O_RDONLY, 0, &p_file);
    if (ret) {
        PRINT_CRIT("Unable to open file %s, error %d", fname, ret);
        goto cleanup;
    }
    ret = file_size(p_file, &frp_size);
    if (ret) {
        PRINT_CRIT("Unable to get file size error %d", ret);
        goto cleanup;
    }
    PRINT_INFO("Frp file size = %d", frp_size);
    p_buf = memory_allocate(frp_size,0);
    if (p_buf == NULL) {
        PRINT_CRIT("Failed to get Frp input buffer of size %d", frp_size);
        goto cleanup;
    }

    bytes = file_read(p_file, 0, p_buf, frp_size);
    if (bytes < 0) {
        PRINT_CRIT("read of Frp file failed %d", bytes);
        goto cleanup;
    }
    PRINT_DEBUG("Finished read of Frp file");

    #ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request() != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        goto cleanup;
    }
    #endif

    enable_write_flash_reg_part_req(snt8100fsr_g, p_buf, frp_size, sc_maj_id);

cleanup:
    if (fname != NULL) {
        memory_free(fname);
    }
    if (p_file != NULL) {
        file_close(p_file);
    }
    if (p_buf != NULL) {
        memory_free(p_buf);
    }
    PRINT_DEBUG("done.");
    return count;
}

#ifdef SUPPORT_FLASH

/*
 * SysFS interface for showing status of fwupdate
 */
ssize_t sysfs_fwupdate_show(struct device *dev,
                            struct device_attribute *attr,
                            char *buf)
{
    int ret;
    PRINT_FUNC("tx_mtu=%d, status=%d", snt8100fsr_g->fwupdate_tx_mtu, snt8100fsr_g->fwupdate_status);
    ret  = snprintf(buf, PAGE_SIZE, "num_pkts = %d\n", snt8100fsr_g->fwupdate_tx_mtu);
    ret += snprintf(buf+ret, PAGE_SIZE, "tot_pkts = %d\n", snt8100fsr_g->fwupdate_tot_mtu);
    ret += snprintf(buf+ret, PAGE_SIZE, "status   = %d\n", snt8100fsr_g->fwupdate_status);
    PRINT_DEBUG("done %d", ret);
    return ret;
}

/*
 * SysFS interface updating the firmware image on system flash
 */
ssize_t sysfs_fwupdate_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count) {
    char *p_buf;
    PRINT_FUNC("%zu bytes", count);

    #ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request() != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return -1;
    }
    #endif

    p_buf = memory_allocate(count, 0);
    memcpy(p_buf, buf, count);
    p_buf[count-1] = '\0'; // remove \n

    enable_fwupdate(snt8100fsr_g, p_buf);
    memory_free(p_buf);

    PRINT_DEBUG("done.");
    return count;
}

ssize_t sysfs_write_flash_reg_part_file_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count) {
    #ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request() != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return -1;
    }
    #endif

    return do_write_reg_script( dev,
                                attr,
                                buf,
                                count,
                                mc_update_regs);
}

ssize_t sysfs_write_flash_reg_part_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count) {
    PRINT_FUNC("%zu bytes", count);

    #ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request() != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return -1;
    }
    #endif

    enable_write_flash_reg_part_req(snt8100fsr_g, buf, count, mc_update_regs);
    PRINT_DEBUG("done.");
    return count;
}

ssize_t sysfs_read_flash_reg_part_store (struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count) {
    int ret;
    struct file *frp_file;
    int frp_file_offset = 0;
    int pcnt;
    int len;
    int i=0;
    int j;
    uint16_t reg_id;
    uint8_t num_reg;
    uint8_t *out_buf=NULL;

    PRINT_FUNC("%zu bytes", count);

    #ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request() != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return -1;
    }
    #endif

    mutex_lock(&snt8100fsr_g->sb_lock);

    PRINT_DEBUG("Creating Frp capture file: %s", FRP_CAPTURE_FILE_LOCATION);

    ret = file_open(FRP_CAPTURE_FILE_LOCATION,
                    O_WRONLY|O_CREAT|O_TRUNC, 0777,
                    &frp_file);
    if(ret) {
        PRINT_DEBUG("Unable to create file '%s', error %d",
                    FRP_CAPTURE_FILE_LOCATION, ret);
        goto errexit;
    }

    // allocate temp buf for writes
    out_buf = (uint8_t*) memory_allocate( PAGE_SIZE, GFP_DMA);
    if (out_buf == NULL) {
        PRINT_CRIT("unable to allocate out_buf");
        goto errexit;
    }

    // allocate buffer to get Flash Register Partition
    if (snt8100fsr_g->reg_part_buf != NULL) {
        memory_free(snt8100fsr_g->reg_part_buf);
    }
    snt8100fsr_g->reg_part_buf = (uint16_t*)memory_allocate(REG_PART_MAX_LEN, GFP_DMA);
    if (snt8100fsr_g->reg_part_buf == NULL) {
        PRINT_CRIT("memory_allocate(%d) failed", REG_PART_MAX_LEN);
        goto errexit;
    }

    /* set up CfgBank regs for read partition 1 */
    snt8100fsr_g->reg_part_buf[0] = 0;      // offset = 0
    snt8100fsr_g->reg_part_buf[1] = 0;      // length = 0
    snt8100fsr_g->reg_part_buf[2] = 0x0101; // READ partition 1
    ret = sb_write_fifo(snt8100fsr_g,
                    REGISTER_BNK_CFG_OFFSET,
                    3*sizeof(uint16_t),
                    snt8100fsr_g->reg_part_buf);
    if (ret) {
        PRINT_CRIT("Unable to write to registers: %d", ret);
        goto errexit;
    }

    // Read FIFO Length and data
    ret = sb_read_fifo(snt8100fsr_g,
                    REGISTER_FIFO_CONFIG,
                    sizeof(uint16_t),
                    snt8100fsr_g->reg_part_buf);

    if (ret) {
        PRINT_CRIT("Unable to read Config FIFO length: %d", ret);
        goto errexit;
    }
    len = snt8100fsr_g->reg_part_buf[0];
    PRINT_DEBUG("Flash Register Partition Length = %d", len);
    if (len > REG_PART_MAX_LEN) {
        PRINT_CRIT("Flash Register Partition length greater than max %d (%d)",
                   REG_PART_MAX_LEN, len);
        goto errexit;
    }
    if (len&1) {
        PRINT_CRIT("Flash Register Partition length must be even number: %d", len);
        goto errexit;
    }
    if (len == 0) {
        PRINT_DEBUG("Flash Register Partition length == 0.");
        // empty partition is not an error
        goto exit;
    }
    ret = sb_read_fifo(snt8100fsr_g,
                    REGISTER_FIFO_CONFIG,
                    len,
                    snt8100fsr_g->reg_part_buf);

    if (ret) {
        PRINT_CRIT("Unable to read Config FIFO length: %d", ret);
        goto errexit;
    }
    // Process output
    pcnt = snprintf(out_buf, PAGE_SIZE, "\n");
    file_write(frp_file, frp_file_offset, out_buf, pcnt);
    frp_file_offset += pcnt;
    reg_id = snt8100fsr_g->reg_part_buf[i]&0x00ff;          // LSB
    num_reg = (snt8100fsr_g->reg_part_buf[i]>>8) & 0x00ff;  // MSB
    PRINT_DEBUG("reg_id=0x%x, num_reg=%d", reg_id, num_reg);
    while (reg_id != 0 && num_reg != 0 && i < len/2) {
        i++;
        if (num_reg == 0) {
            PRINT_CRIT("num_reg in Flash Register Partition is 0 at pos %d", i);
            goto errexit;
        }
        if (num_reg+i >= len/2) {
            PRINT_CRIT("num_reg exceeds register list size: (%d, %d, %d)", num_reg, i, len/2);
            goto errexit;
        }
        if (reg_id == 0x82) reg_id = 0x200;
        pcnt = snprintf(out_buf, PAGE_SIZE, "0x%x %d", reg_id, num_reg);
        file_write(frp_file, frp_file_offset, out_buf, pcnt);
        frp_file_offset += pcnt;
        for (j=0; j < num_reg; j++) {
            if (j % 8 == 0) {
                pcnt = snprintf(out_buf, PAGE_SIZE, "\n");
                file_write(frp_file, frp_file_offset, out_buf, pcnt);
                frp_file_offset += pcnt;
            }
            pcnt = snprintf(out_buf, PAGE_SIZE, " 0x%04x", snt8100fsr_g->reg_part_buf[i+j]);
            file_write(frp_file, frp_file_offset, out_buf, pcnt);
            frp_file_offset += pcnt;
        }
        i += j;
        pcnt = snprintf(out_buf, PAGE_SIZE, "\n");
        file_write(frp_file, frp_file_offset, out_buf, pcnt);
        frp_file_offset += pcnt;
        reg_id = snt8100fsr_g->reg_part_buf[i]&0x00ff;          // LSB
        num_reg = (snt8100fsr_g->reg_part_buf[i]>>8) & 0x00ff;  // MSB
        PRINT_DEBUG("reg_id=0x%x, num_reg=%d", reg_id, num_reg);
   }
exit:
    if (snt8100fsr_g->reg_part_buf != NULL) {
        memory_free(snt8100fsr_g->reg_part_buf);
        snt8100fsr_g->reg_part_buf = NULL;
    }
    if (out_buf != NULL) {
        memory_free(out_buf);
    }
    if (frp_file) {
        file_close(frp_file);
    }
    mutex_unlock(&snt8100fsr_g->sb_lock);
    PRINT_DEBUG("done %d", frp_file_offset);
    return count;

errexit:
    count = -1;
    goto exit;
}


/*
 * SysFS interface for read the Flash Register Partition
 */
ssize_t sysfs_read_flash_reg_part_show(struct device *dev,
                                       struct device_attribute *attr,
                                       char *buf)
{
    int ret;
    int count=0;
    int len;
    int i=0;
    int j;
    uint16_t reg_id;
    uint8_t num_reg;

    PRINT_FUNC("Enter.");

    #ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request() != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return -1;
    }
    #endif

    mutex_lock(&snt8100fsr_g->sb_lock);

    // allocate buffer to get Flash Register Partition
    if (snt8100fsr_g->reg_part_buf != NULL) {
        memory_free(snt8100fsr_g->reg_part_buf);
    }
    snt8100fsr_g->reg_part_buf = (uint16_t*)memory_allocate(REG_PART_MAX_LEN, GFP_DMA);
    if (snt8100fsr_g->reg_part_buf == NULL) {
        PRINT_CRIT("memory_allocate(%d) failed", REG_PART_MAX_LEN);
        goto errexit;
    }

    /* set up CfgBank regs for read partition 1 */
    snt8100fsr_g->reg_part_buf[0] = 0;      // offset = 0
    snt8100fsr_g->reg_part_buf[1] = 0;      // length = 0
    snt8100fsr_g->reg_part_buf[2] = 0x0101; // READ partition 1
    ret = sb_write_fifo(snt8100fsr_g,
                    REGISTER_BNK_CFG_OFFSET,
                    3*sizeof(uint16_t),
                    snt8100fsr_g->reg_part_buf);
    if (ret) {
        PRINT_CRIT("Unable to write to registers: %d", ret);
        goto errexit;
    }

    // Read FIFO Length and data
    ret = sb_read_fifo(snt8100fsr_g,
                    REGISTER_FIFO_CONFIG,
                    sizeof(uint16_t),
                    snt8100fsr_g->reg_part_buf);

    if (ret) {
        PRINT_CRIT("Unable to read Config FIFO length: %d", ret);
        goto errexit;
    }
    len = snt8100fsr_g->reg_part_buf[0];
    PRINT_DEBUG("Flash Register Partition Length = %d", len);
    if (len > REG_PART_MAX_LEN) {
        PRINT_CRIT("Flash Register Partition length greater than max %d (%d)",
                   REG_PART_MAX_LEN, len);
        goto errexit;
    }
    if (len&1) {
        PRINT_CRIT("Flash Register Partition length must be even number: %d", len);
        goto errexit;
    }
    if (len == 0) {
        PRINT_DEBUG("Flash Register Partition length == 0.");
        // empty partition is not an error
        goto exit;
    }
    ret = sb_read_fifo(snt8100fsr_g,
                    REGISTER_FIFO_CONFIG,
                    len,
                    snt8100fsr_g->reg_part_buf);

    if (ret) {
        PRINT_CRIT("Unable to read Config FIFO length: %d", ret);
        goto errexit;
    }
    // Process output
    count += snprintf(buf+count, PAGE_SIZE, "\n");
    reg_id = snt8100fsr_g->reg_part_buf[i]&0x00ff;          // LSB
    num_reg = (snt8100fsr_g->reg_part_buf[i]>>8) & 0x00ff;  // MSB
    PRINT_DEBUG("reg_id=0x%x, num_reg=%d", reg_id, num_reg);
    while (reg_id != 0 && num_reg != 0 && i < len/2) {
        i++;
        if (num_reg == 0) {
            PRINT_CRIT("num_reg in Flash Register Partition is 0 at pos %d", i);
            goto errexit;
        }
        if (num_reg+i >= len/2) {
            PRINT_CRIT("num_reg exceeds register list size: (%d, %d, %d)", num_reg, i, len/2);
            goto errexit;
        }
        if (reg_id == 0x82) reg_id = 0x200;
        count += snprintf(buf+count, PAGE_SIZE, "0x%x %d", reg_id, num_reg);
        for (j=0; j < num_reg; j++) {
            if (j % 8 == 0) count += snprintf(buf+count, PAGE_SIZE, "\n");
            count += snprintf(buf+count, PAGE_SIZE, " 0x%04x", snt8100fsr_g->reg_part_buf[i+j]);
        }
        i += j;
        count += snprintf(buf+count, PAGE_SIZE, "\n");
        reg_id = snt8100fsr_g->reg_part_buf[i]&0x00ff;          // LSB
        num_reg = (snt8100fsr_g->reg_part_buf[i]>>8) & 0x00ff;  // MSB
        PRINT_DEBUG("reg_id=0x%x, num_reg=%d", reg_id, num_reg);
   }
exit:
    if (snt8100fsr_g->reg_part_buf != NULL) {
        memory_free(snt8100fsr_g->reg_part_buf);
        snt8100fsr_g->reg_part_buf = NULL;
    }
    mutex_unlock(&snt8100fsr_g->sb_lock);
    PRINT_DEBUG("done %d", count);
    return count;

errexit:
    count = -1;
    goto exit;
}
#endif
// SUPPORT_FLASH

ssize_t sysfs_reg_script_file_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count) {
    #ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request() != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return -1;
    }
    #endif

    return do_write_reg_script( dev,
                                attr,
                                buf,
                                count,
                                mc_reg_script);
}



/*
 * SysFS interface for --SetSysParam
 */
ssize_t sysfs_set_sys_param_show(struct device *dev,
                            struct device_attribute *attr,
                            char *buf)
{
    int ret;
    PRINT_FUNC("status %d", snt8100fsr_g->set_sys_param_status);
    ret = sysfs_uint32_show(dev, attr,
                            &snt8100fsr_g->set_sys_param_status, buf);
    PRINT_DEBUG("done %d", ret);
    return ret;
}

/*
 *
 */
ssize_t sysfs_set_sys_param_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count) {

    int l = count;
    uint32_t val;
    uint32_t id;
    int status;
    int ret;

    PRINT_FUNC("%zu bytes", count);

    #ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request() != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return -1;
    }
    #endif

    status  = ReadNumFromBuf((const uint8_t**)&buf, &l, &id);
    if (status != 0) {
        PRINT_CRIT("Could not parse param_id %d", status);
        goto errexit;
    }

    status = ReadNumFromBuf((const uint8_t**)&buf, &l, &val);
    if (status != 0) {
        PRINT_CRIT("Could not parse param_val %d", status);
        goto errexit;
    }

    if (enable_set_sys_param(snt8100fsr_g, id, val) != 0) {
        PRINT_DEBUG("send of set sys param failed");
        goto errexit;
    }

    // wait for response from driver irpt thread
    PRINT_DEBUG("SetSysParam Rsp -- wait");
    do {
        ret = down_interruptible(&snt8100fsr_g->sc_wf_rsp);
        PRINT_DEBUG("SetSysParam Rsp -- acquired %d", ret);
    } while (ret == -EINTR);

errexit:
    PRINT_DEBUG("done.");
    return count;
}

/*
 * SysFS Interface for profile
 */


ssize_t sysfs_profile_show(struct device *dev,
                            struct device_attribute *attr,
                            char *buf) {
    int ret = 0;

    PRINT_FUNC();
    mutex_lock(&snt8100fsr_g->sb_lock);

    ret += snprintf(buf+ret, PAGE_SIZE, "%d\n", snt8100fsr_g->op_profile);

    mutex_unlock(&snt8100fsr_g->sb_lock);
    PRINT_DEBUG("done.");
    return ret;
}

ssize_t sysfs_profile_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count) {

    long profile;

    PRINT_FUNC("%zu bytes", count);

    if (count > 10) {
        return -1;
    }

    #ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request() != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return -1;
    }
    #endif

    if(kstrtol(buf, 10, &profile) == 0) {
        snt8100fsr_g->op_profile = (int)profile;
        set_operational_profile(snt8100fsr_g, snt8100fsr_g->op_profile);
    } else {
        PRINT_CRIT("bad parameter");
        return -1;
    }

    PRINT_DEBUG("done.");
    return count;
}


/*
 * SysFS interface for --GetSysParam
 */
ssize_t sysfs_get_sys_param_show(struct device *dev,
                            struct device_attribute *attr,
                            char *buf)
{
    int ret=0;
    int bytes=0;

    PRINT_FUNC();

    mutex_lock(&snt8100fsr_g->sb_lock);

   // deal with gets greater than 4 bytes
    if (snt8100fsr_g->get_sys_param_cmd && snt8100fsr_g->get_sys_param_cmd->length > 8) {
        int len = snt8100fsr_g->get_sys_param_cmd->length-4;
        int i;
        uint8_t *p = (uint8_t*) &snt8100fsr_g->get_sys_param_cmd->data[1];
        PRINT_INFO("id = %d, status=%d, len=%d", snt8100fsr_g->get_sys_param_id,
                                                snt8100fsr_g->get_sys_param_status,
                                                len);
        ret = snprintf(buf, PAGE_SIZE, "%d", snt8100fsr_g->get_sys_param_status);
        for (i=0; i < len; i++) {
            bytes = snprintf(buf+ret, PAGE_SIZE, " %d", p[i]);
            ret += bytes;
        }
        snprintf(buf+ret, PAGE_SIZE, "\n");

        memory_free(snt8100fsr_g->get_sys_param_cmd);
        snt8100fsr_g->get_sys_param_cmd = NULL;

    } else {
        PRINT_DEBUG("id = %d, status=%d, val=%d", snt8100fsr_g->get_sys_param_id,
                                                snt8100fsr_g->get_sys_param_status,
                                                snt8100fsr_g->get_sys_param_val);

        ret= snprintf(buf, PAGE_SIZE, "%d %u\n", snt8100fsr_g->get_sys_param_status, snt8100fsr_g->get_sys_param_val);
    }

    if (snt8100fsr_g->get_sys_param_cmd) {
        memory_free(snt8100fsr_g->get_sys_param_cmd);
        snt8100fsr_g->get_sys_param_cmd = NULL;
    }
    mutex_unlock(&snt8100fsr_g->sb_lock);
    PRINT_DEBUG("done %d", ret);
    return ret;
}

/*
 *
 */
ssize_t sysfs_get_sys_param_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count) {
    int l = count;
    uint32_t val;
    int status;
    int ret;

    PRINT_FUNC("%zu bytes", count);

    #ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request() != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return -1;
    }
    #endif

    status  = ReadNumFromBuf((const uint8_t**)&buf, &l, &val);
    if (status != 0) {
        PRINT_CRIT("Could not parse param_id %d", status);
        goto errexit;
    }

    enable_get_sys_param(snt8100fsr_g, val);

    // wait for response from driver irpt thread
    PRINT_DEBUG("GetSysParam Rsp -- wait");
    do {
        ret = down_interruptible(&snt8100fsr_g->sc_wf_rsp);
        PRINT_DEBUG("GetSysParam Rsp -- acquired %d", ret);
    } while (ret == -EINTR);

errexit:
    PRINT_DEBUG("done");
    return count;
}

/*
 * SysFS interface for read/write of registers
 */
ssize_t sysfs_get_reg_show(struct device *dev,
                            struct device_attribute *attr,
                            char *buf)
{
    int ret=0;
    int bytes=0;

    PRINT_FUNC();
    mutex_lock(&snt8100fsr_g->sb_lock);

    if (NULL != snt8100fsr_g->get_reg_buf) {
        int i;
        PRINT_DEBUG("id = %d, len=%d", snt8100fsr_g->get_reg_id, snt8100fsr_g->get_reg_num);
        for (i=0; i < snt8100fsr_g->get_reg_num; i++) {
            PRINT_INFO("reg_id=%d, val=0x%x, ret=%d", snt8100fsr_g->get_reg_id+i, snt8100fsr_g->get_reg_buf[i], ret );
            bytes = snprintf(buf+ret, PAGE_SIZE, " 0x%04x", snt8100fsr_g->get_reg_buf[i]);
            ret += bytes;
        }

        ret += snprintf(buf+ret, PAGE_SIZE, "\n");;

        if (NULL != snt8100fsr_g->get_reg_buf) memory_free(snt8100fsr_g->get_reg_buf);
        snt8100fsr_g->get_reg_buf = NULL;

    } else {
      PRINT_DEBUG("Empty get_reg_buf");
    }

    mutex_unlock(&snt8100fsr_g->sb_lock);
    PRINT_DEBUG("done %d", ret);
    return ret;
}


ssize_t sysfs_get_reg_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count) {
    int len;
    int ret;
    int l = count;
    uint32_t val;
    int status;

    PRINT_FUNC("%zu bytes", count);

    if (snt8100fsr_g->status_fwd != FWUPDATE_SUCCESS) {
        PRINT_CRIT("Sona NOT Ready!!");
        return count;
    }

    #ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request() != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return -1;
    }
    #endif

    mutex_lock(&snt8100fsr_g->sb_lock);

    status  = ReadNumFromBuf((const uint8_t**)&buf, &l, &val);
    if (status != 0) {
        PRINT_CRIT("Could not parse reg_id %d", status);
        goto errexit;
    }

    if (val > 0x400) {
        PRINT_CRIT("Invalid register address (0x%x)", val);
        goto errexit;
    }

    snt8100fsr_g->get_reg_id  = val;

    status = ReadNumFromBuf((const uint8_t**)&buf, &l, &val);
    if (status != 0) {
        PRINT_CRIT("Could not parse num regs %d", status);
        goto errexit;
    }
    snt8100fsr_g->get_reg_num = val;

    len = snt8100fsr_g->get_reg_num * 2;                                  // cvt from num_reg to byte len
    PRINT_INFO("reg_id=%d, num_reg=%d", snt8100fsr_g->get_reg_id, snt8100fsr_g->get_reg_num);

    if (snt8100fsr_g->get_reg_buf != NULL) {
        memory_free(snt8100fsr_g->get_reg_buf);
    }
    snt8100fsr_g->get_reg_buf = (uint16_t*)memory_allocate(len, GFP_DMA);
    if (snt8100fsr_g->get_reg_buf == NULL) {
        PRINT_CRIT("memory_allocate(%d) failed", len);
        count = -1;
        goto errexit;
    }

    ret = sb_read_fifo(snt8100fsr_g,
                    snt8100fsr_g->get_reg_id,
                    len,
                    snt8100fsr_g->get_reg_buf);

    if (ret) {
        PRINT_CRIT("Unable to read from registers: %d", ret);
        if (snt8100fsr_g->get_reg_buf != NULL) memory_free(snt8100fsr_g->get_reg_buf);
        snt8100fsr_g->get_reg_buf = NULL;
        snt8100fsr_g->get_reg_num = 0;
        count = -1;
    }

errexit:
    mutex_unlock(&snt8100fsr_g->sb_lock);
    PRINT_DEBUG("done");
    return count;
}

/*
 * SysFS interface for read/write of registers
 */
ssize_t sysfs_factory_mode_show(struct device *dev,
                            struct device_attribute *attr,
                            char *buf)
{
    int ret=0;
    int bytes=0;

    PRINT_FUNC();
    PRINT_DEBUG("factory mode = %d", snt8100fsr_g->factory_mode);
    bytes = snprintf(buf+ret, PAGE_SIZE, "%d", snt8100fsr_g->factory_mode);
    ret += bytes;
    PRINT_DEBUG("done %d", ret);
    return ret;
}

ssize_t sysfs_factory_mode_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count) {
    int l = count;
    uint32_t val;
    int status;

    PRINT_FUNC("%zu bytes", count);
    status  = ReadNumFromBuf((const uint8_t**)&buf, &l, &val);
    if (status != 0) {
        PRINT_CRIT("Could not parse the value %d", status);
        return 0;
    }

    if (val < 0 || val > 3) {
        PRINT_CRIT("Invalid input value (%d)", val);
        goto errexit;
    }

    snt8100fsr_g->factory_mode  = val;

errexit:
    PRINT_DEBUG("done");
    return count;
}

ssize_t sysfs_sc_reset_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count) {
    PRINT_FUNC("%zu bytes", count);
    if (count != 2) {
        PRINT_CRIT("Only single digit number '1' or '0' allowed");
        return -1;
    }

    #ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request() != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return -1;
    }
    #endif

    if (*buf == '1') {
        sc_cleanup(snt8100fsr_g);
    } else {
        PRINT_DEBUG("only '1' allowed");
    }

    PRINT_DEBUG("done");
    return count;
}

ssize_t sysfs_hw_reset_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count) {

    int ret;

    PRINT_FUNC("%zu bytes", count);

    if (2 != count) {
        PRINT_CRIT("Only single digit number '1' or '0' allowed");
        return -1;
    }

    if ('1' == *buf) {
        // TBD, to add more check here to make sure the driver can recognize
        // the next interrupt as firmware request IRQ Reg.0x00 == 0x0080

        PRINT_INFO("reset the chip...");
        // mutex lock here in order to make sure no communication will be intrrrupted.
        mutex_lock(&snt8100fsr_g->sb_lock);
        ret = snt_device_hw_reset(snt8100fsr_g);
        mutex_unlock(&snt8100fsr_g->sb_lock);
    } else {
        PRINT_DEBUG("only '1' allowed");
    }

    PRINT_DEBUG("done");
    return count;
}

ssize_t sysfs_sw_reset_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count) {

    PRINT_FUNC("%zu bytes", count);

    if (2 != count) {
        PRINT_CRIT("Only single digit number '1' or '0' allowed");
        return -1;
    }

    if ('1' == *buf) {
        // TBD, to add more check here to make sure the driver can recognize
        // the next interrupt as firmware request IRQ Reg.0x00 == 0x0080

        PRINT_INFO("reset the chip...");
        #ifdef DYNAMIC_PWR_CTL
        if (snt_activity_request() != 0) {
            PRINT_CRIT("snt_activity_request() failed");
            return -1;
        }
        #endif

        sc_cleanup(snt8100fsr_g);
        snt_device_sw_reset(snt8100fsr_g);



    } else {
        PRINT_DEBUG("only '1' allowed");
    }

    PRINT_DEBUG("done");
    return count;
}
ssize_t sysfs_set_reg_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count) {
    int l = count;
    uint32_t val;
    int reg_id;
    uint16_t * reg_buf = NULL;
    int len = 0;
    int ret;
    int status;

    PRINT_FUNC("%zu bytes", count);

    if (snt8100fsr_g->status_fwd != FWUPDATE_SUCCESS) {
        PRINT_CRIT("Sona NOT Ready!!");
        return count;
    }

    #ifdef DYNAMIC_PWR_CTL
    if (snt_activity_request() != 0) {
        PRINT_CRIT("snt_activity_request() failed");
        return -1;
    }
    #endif

    status = ReadNumFromBuf((const uint8_t**)&buf, &l, &val);
    if (status != 0) {
        PRINT_DEBUG("Could not parse reg_id %d", (int)status);
        goto errexit;
    }
    reg_id = val;
    PRINT_DEBUG("reg_id=%d", reg_id);

    if (reg_id > 0x400) {
        PRINT_CRIT("Invalid register address (0x%x)", reg_id);
        return -EPERM;
    }

    reg_buf = (uint16_t*)memory_allocate(count, GFP_DMA);
    if (reg_buf == NULL) {
        PRINT_CRIT("memory_allocate(%zu) failed",count);
        return -1;
    }

    status = ReadNumFromBuf((const uint8_t**)&buf, &l, &val);
    while (status == 0) {
        if (0x80000000 & val) val &= 0x0000ffff;            // to 16 bit neg num
        if ( val > 0x0000ffff) {
             PRINT_DEBUG("invalid register value %x", val);
             goto errexit;
        }
        reg_buf[len++] = val;
        status = ReadNumFromBuf((const uint8_t**)&buf, &l, &val);
    }
    len *= 2; // cvt to from num reg to num bytes

    mutex_lock(&snt8100fsr_g->sb_lock);
    ret = sb_write_fifo(snt8100fsr_g,
                    reg_id,
                    len,
                    reg_buf);
    mutex_unlock(&snt8100fsr_g->sb_lock);
    if (ret) {
        PRINT_CRIT("Unable to write to registers: %d", ret);
        count = 0;
        goto errexit;
    }
    else { //cache register_frame_rate if it was set
        if (reg_id < 3  && (reg_id + count) > 2) {
            snt8100fsr_g-> frame_rate = reg_buf[2-reg_id];
        }
    }

errexit:
    if (reg_buf) {
        memory_free(reg_buf);
    }
    PRINT_DEBUG("done");
    return count;
}


/*==========================================================================*/
/* SysFS Setup                                                              */
/*==========================================================================*/
// SYSFS_PERM_SHOW_STORE
static DEVICE_ATTR(log_track_report,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_log_track_report_show,
                   sysfs_log_track_report_store);

static DEVICE_ATTR(log_track_report_bin,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_log_track_report_bin_show,
                   sysfs_log_track_report_bin_store);

static DEVICE_ATTR(event_log,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_event_log_show,
                   sysfs_event_log_store);

static DEVICE_ATTR(log_d1test,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_log_d1test_show,
                   sysfs_log_d1test_store);

static DEVICE_ATTR(log_frames,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_log_frames_show,
                   sysfs_log_frames_store);

static DEVICE_ATTR(log_no_touch_frame,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_log_no_touch_frame_show,
                   sysfs_log_no_touch_frame_store);

static DEVICE_ATTR(suspended_frame_rate,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_suspended_frame_rate_show,
                   sysfs_suspended_frame_rate_store);

static DEVICE_ATTR(set_sys_param,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_set_sys_param_show,
                   sysfs_set_sys_param_store);

static DEVICE_ATTR(get_sys_param,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_get_sys_param_show,
                   sysfs_get_sys_param_store);

static DEVICE_ATTR(profile,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_profile_show,
                   sysfs_profile_store);

static DEVICE_ATTR(get_reg,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_get_reg_show,
                   sysfs_get_reg_store);

static DEVICE_ATTR(factory_mode,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_factory_mode_show,
                   sysfs_factory_mode_store);

#ifdef SUPPORT_FLASH
static DEVICE_ATTR(fwupdate,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_fwupdate_show,
                   sysfs_fwupdate_store);
static DEVICE_ATTR(read_flash_reg_part,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_read_flash_reg_part_show,
                   sysfs_read_flash_reg_part_store);

#endif

#ifdef DYNAMIC_PWR_CTL
static DEVICE_ATTR(enable_dynamic_pwr_ctl,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_enable_dynamic_pwr_ctl_show,
                   sysfs_enable_dynamic_pwr_ctl_store);
#endif


// SYSFS_PERM_STORE
static DEVICE_ATTR(wake_device,
                   SYSFS_PERM_STORE,
                   NULL,
                   sysfs_wake_device_store);

static DEVICE_ATTR(set_reg,
                   SYSFS_PERM_STORE,
                   NULL,
                   sysfs_set_reg_store);

static DEVICE_ATTR(deep_trace,
                   SYSFS_PERM_STORE,
                   NULL,
                   sysfs_deep_trace_store);

static DEVICE_ATTR(sc_reset,
                   SYSFS_PERM_STORE,
                   NULL,
                   sysfs_sc_reset_store);

static DEVICE_ATTR(hw_reset,
                   SYSFS_PERM_STORE,
                   NULL,
                   sysfs_hw_reset_store);

#ifdef SUPPORT_FLASH
static DEVICE_ATTR(write_flash_reg_part,
                   SYSFS_PERM_STORE,
                   NULL,
                   sysfs_write_flash_reg_part_store);
static DEVICE_ATTR(write_flash_reg_part_file,
                   SYSFS_PERM_STORE,
                   NULL,
                   sysfs_write_flash_reg_part_file_store);
#endif

static DEVICE_ATTR(reg_script_file,
                   SYSFS_PERM_STORE,
                   NULL,
                   sysfs_reg_script_file_store);

// SYSFS_PERM_SHOW
static DEVICE_ATTR(track_report,
                   SYSFS_PERM_SHOW,
                   sysfs_track_report_show,
                   NULL);

static DEVICE_ATTR(product_config,
                   SYSFS_PERM_SHOW,
                   sysfs_product_config_show,
                   NULL);

static DEVICE_ATTR(boot_status,
                   SYSFS_PERM_SHOW,
                   sysfs_boot_status_show,
                   NULL);

static DEVICE_ATTR(version,
                   SYSFS_PERM_SHOW,
                   sysfs_version_show,
                   NULL);


static DEVICE_ATTR(verbose, SYSFS_PERM_SHOW_STORE, verbose_show, verbose_set);
static DEVICE_ATTR(raw_report, SYSFS_PERM_SHOW_STORE, show_raw_report_mode, set_raw_report_mode);
static DEVICE_ATTR(force_threshold, SYSFS_PERM_SHOW_STORE, NULL, set_tap_force_threshold);
static DEVICE_ATTR(misc_cmd, SYSFS_PERM_SHOW_STORE, NULL, handle_misc_cmd);

static struct attribute *sysfs_attrs_static[] = {
    &dev_attr_track_report.attr,
    &dev_attr_log_track_report.attr,
    &dev_attr_log_track_report_bin.attr,
    &dev_attr_event_log.attr,
    &dev_attr_log_no_touch_frame.attr,
    &dev_attr_log_d1test.attr,
    &dev_attr_log_frames.attr,
    &dev_attr_suspended_frame_rate.attr,
    &dev_attr_product_config.attr,
    &dev_attr_boot_status.attr,
    &dev_attr_verbose.attr,
    &dev_attr_raw_report.attr,
    &dev_attr_force_threshold.attr,
    &dev_attr_misc_cmd.attr,
    &dev_attr_wake_device.attr,
    &dev_attr_version.attr,
    &dev_attr_profile.attr,
    &dev_attr_set_sys_param.attr,
    &dev_attr_get_sys_param.attr,
    &dev_attr_get_reg.attr,
    &dev_attr_set_reg.attr,
    &dev_attr_factory_mode.attr,
    &dev_attr_sc_reset.attr,
    &dev_attr_hw_reset.attr,
    &dev_attr_deep_trace.attr,
    &dev_attr_reg_script_file.attr,
#ifdef SUPPORT_FLASH
    &dev_attr_fwupdate.attr,
    &dev_attr_write_flash_reg_part.attr,
    &dev_attr_write_flash_reg_part_file.attr,
    &dev_attr_read_flash_reg_part.attr,
#endif
#ifdef DYNAMIC_PWR_CTL
    &dev_attr_enable_dynamic_pwr_ctl.attr,
#endif
    NULL
};

int snt_sysfs_init(struct snt8100fsr *snt8100fsr, bool enable) {
#ifdef SUPPORT_SYSFS_CLASS
    int ret = 0;
#endif

    PRINT_FUNC("0x%p, %s for \"%s\"", snt8100fsr, enable ? "true" : "false", SYSFS_NAME);

    if (!enable) {
#ifndef SUPPORT_SYSFS_CLASS
        sysfs_remove_link(sysfs_kobj_g, SYSFS_NAME);
        sysfs_remove_group(sysfs_kobj_g, &attr_group_g);
        kobject_del(sysfs_kobj_g);
#else
        if (sntdev_device_class)
            device_destroy(sntdev_device_class, dev_no);

        unregister_chrdev_region(dev_no, 1);

        if (sntdev_device_class != NULL) {
            class_destroy(sntdev_device_class);
            sntdev_device_class = NULL;
        }
#endif

        free_sysfs_attrs(sysfs_attrs_g);

        if (snt8100fsr && snt8100fsr->get_sys_param_cmd) {
            memory_free(snt8100fsr->get_sys_param_cmd);
            snt8100fsr->get_sys_param_cmd = NULL;
        }
        if (snt8100fsr && snt8100fsr->get_reg_buf) {
            memory_free(snt8100fsr->get_reg_buf);
            snt8100fsr->get_reg_buf = NULL;
        }
        snt8100fsr_g = NULL;
        dev_info(snt8100fsr->dev, "%s: sysfs attributes removed\n", __func__);
    } else {

        sysfs_attrs_g = alloc_sysfs_attrs();

        // Save our attributes to the attribute group
        attr_group_g.attrs = sysfs_attrs_g;

#ifndef SUPPORT_SYSFS_CLASS
        sysfs_kobj_g = kobject_create_and_add(SYSFS_NAME, NULL);
        if (sysfs_kobj_g == NULL) {
            dev_err(snt8100fsr->dev,"%s: subsystem_register failed\n", __func__);
            return -ENOMEM;
        }

        if (snt8100fsr->spi) {
            if (sysfs_create_link(sysfs_kobj_g, &snt8100fsr->spi->dev.kobj, "spi") < 0) {
                dev_err(snt8100fsr->dev,
                        "%s: failed to create link\n",
                        __func__);
                return -ENOMEM;
            }
        }

        if (snt8100fsr->i2c) {
            if (sysfs_create_link(sysfs_kobj_g, &snt8100fsr->i2c->dev.kobj, "i2c") < 0) {
                dev_err(snt8100fsr->dev,
                        "%s: failed to create link\n",
                        __func__);
                return -ENOMEM;
            }
        }

        if (sysfs_create_group(sysfs_kobj_g, &attr_group_g) < 0) {
            dev_err(snt8100fsr->dev,
                    "%s: Failed to create sysfs attributes\n",
                    __func__);
            return -ENOMEM;
        }
    }

    PRINT_DEBUG("done");
    return 0;
#else
        if (sntdev_device_class != NULL)
            return 0;

        sntdev_device_class = class_create(THIS_MODULE, SYSFS_DEVICE_CLASS_NAME);
        if (IS_ERR(sntdev_device_class)) {
            PRINT_ERR("%s: Failed to create /sys/class/%s", __func__, SYSFS_DEVICE_CLASS_NAME);
            return -ENODEV;
        }
        else
            PRINT_INFO("%s: Success to create /dev/%s", __func__, SYSFS_DEVICE_CLASS_NAME);

        ret = alloc_chrdev_region(&dev_no, 0, 1, SYSFS_CHAR_DEVICE_NAME);
        if (ret < 0) {
             dev_err(snt8100fsr->dev,
                     "[EDGE] %s: Failed to allocate char device region\n",
                     __func__);
             goto err_device_region;
        }

        sntdev_major_num = MAJOR(dev_no);
        dev_dbg(snt8100fsr->dev,
                "[EDGE] %s: Major number of rmidev = %d\n",
                __func__, sntdev_major_num);

        device_ptr = device_create(sntdev_device_class, NULL, dev_no,
                NULL, SYSFS_CHAR_DEVICE_NAME);
        if (IS_ERR(device_ptr)) {
            dev_err(snt8100fsr->dev,
                    "[EDGE] %s: Failed to create rmi char device\n",
                    __func__);
            ret = -ENODEV;
            goto err_dev_data;
        }


        if (sysfs_create_group(&device_ptr->kobj, &attr_group_g) < 0) {
            dev_err(snt8100fsr->dev,
                    "%s: Failed to create sysfs attributes\n",
                    __func__);
            ret = -ENODEV;
            goto err_char_device;
        }
    }

    PRINT_DEBUG("done");
    return 0;

err_char_device:
    if (sntdev_device_class)
        device_destroy(sntdev_device_class, dev_no);
err_dev_data:
    unregister_chrdev_region(dev_no, 1);
err_device_region:
    if (sntdev_device_class != NULL) {
        class_destroy(sntdev_device_class);
        sntdev_device_class = NULL;
    }

    return ret;
#endif
}

void free_sysfs_attrs(struct attribute **attrs) {

    /*
     * Free memory for our dynamically allocated register sysFS entries,
     * starting from after the static portion of the list which we don't
     * perform memory_free operations on.
     */
    int i = get_attrs_array_size(sysfs_attrs_static);
    struct attribute *attr = attrs[i];
    while (attr) {
        PRINT_DEBUG("sysFS[%d]: 0x%p", i, attr);
        memory_free((void *)attr->name);
        memory_free(attr);
        attr = attrs[++i];
    }
    memory_free(attrs);
}

struct attribute **alloc_sysfs_attrs(void) {
    int sysfs_entries;
    int total_entries;
    int total_size;
    struct attribute **attrs;

    /*
     * Calculate how many sysFS entries we need room for, plus trailing
     * null inside sysfs_attrs_list.
     */
    sysfs_entries = get_attrs_array_size(sysfs_attrs_static);
    total_entries = sysfs_entries;

    // Calcualte the size of these entries in bytes, plus room for null;
    total_size = (total_entries + 1) * sizeof(struct attribute *);

    PRINT_DEBUG("%d static sysFS entries found", sysfs_entries);
    PRINT_DEBUG("Allocating %d bytes for %d sysFS entries", total_size, total_entries);

    attrs = memory_allocate(total_size, 0);
    memset(attrs, 0, total_size);

    // Firstly, copy the static sysfs attributes
    memcpy(attrs, sysfs_attrs_static, sizeof(sysfs_attrs_static));

    return attrs;
}

int get_attrs_array_size(struct attribute *list[]) {
    int i = 0;
    struct attribute *attr = list[i];
    while (attr) {
        attr = list[++i];
    }
    return i;
}



/*==========================================================================*/
/* DEEP TRACE LOGGING                                                       */
/*==========================================================================*/
int dump_deep_trace_to_file(void) {
    int             ret;
    int             err_ret = 0;
    struct file*    deep_trace_file = NULL;
    int             deep_trace_file_offset = 0;
    uint16_t    *   deep_trace_buf = NULL;
    int             len;

    PRINT_FUNC("Enter.");

    mutex_lock(&snt8100fsr_g->sb_lock);

    PRINT_DEBUG("Creating deep trace file: %s",  DEEP_TRACE_LOCATION);
    ret = file_open(DEEP_TRACE_LOCATION,
                    O_WRONLY|O_CREAT|O_TRUNC, 0777,
                    &deep_trace_file);
    if(ret) {
        PRINT_DEBUG("Unable to create file '%s', error %d",
                    DEEP_TRACE_LOCATION, ret);
        goto errexit;
    }

    // allocate buffer to get Deep Trace table
    deep_trace_buf = (uint16_t*)memory_allocate(DEEP_TRACE_BUF_LEN, GFP_DMA);
    if (deep_trace_buf == NULL) {
        PRINT_CRIT("memory_allocate(%d) failed", DEEP_TRACE_BUF_LEN);
        goto errexit;
    }

    /* set up CfgBank regs for read partition 7 */
    deep_trace_buf[0] = 0;      // offset = 0
    deep_trace_buf[1] = 0;      // length = 0
    deep_trace_buf[2] = 0x0701; // READ partition 7
    ret = sb_write_fifo(snt8100fsr_g,
                    REGISTER_BNK_CFG_OFFSET,
                    3*sizeof(uint16_t),
                    deep_trace_buf);
    if (ret) {
        PRINT_CRIT("Unable to write to cfgbank registers: %d", ret);
        goto errexit;
    }

    // Read and write FIFO Length
    ret = sb_read_fifo(snt8100fsr_g,
                    REGISTER_FIFO_CONFIG,
                    sizeof(uint16_t),
                    deep_trace_buf);

    if (ret) {
        PRINT_CRIT("Unable to read Config FIFO length: %d", ret);
        goto errexit;
    }
    len = deep_trace_buf[0];
    PRINT_DEBUG("Deep Trace Length = %d", len);
    if (len&1) {
        PRINT_CRIT("Deep Trace length must be even number: %d", len);
        goto errexit;
    }
    if (len == 0) {
        PRINT_DEBUG("Deep Trace length == 0.");
        // empty partition is not an error
        goto exit;
    }
    file_write(deep_trace_file,
                deep_trace_file_offset,
                (uint8_t*)deep_trace_buf,
                sizeof(uint16_t));
    deep_trace_file_offset += sizeof(uint16_t);

    // Read and write Deep Trace Table
    while (len > 0) {
        int block_len = min(len, DEEP_TRACE_BUF_LEN);
        ret = sb_read_fifo(snt8100fsr_g,
                        REGISTER_FIFO_CONFIG,
                        block_len,
                        deep_trace_buf);

        if (ret) {
            PRINT_CRIT("Unable to read deep trace buffer: %d", ret);
            goto errexit;
        }
        len -= block_len;
        file_write(deep_trace_file,
                    deep_trace_file_offset,
                    (uint8_t*)deep_trace_buf,
                    block_len);
        deep_trace_file_offset += block_len;
    }

exit:
    if (deep_trace_file != NULL) {
        file_close(deep_trace_file);
    }
    if (deep_trace_buf != NULL) {
        memory_free(deep_trace_buf);
    }
    mutex_unlock(&snt8100fsr_g->sb_lock);
    PRINT_DEBUG("done");

    return err_ret;

errexit:
    err_ret = -1;
    goto exit;
}
