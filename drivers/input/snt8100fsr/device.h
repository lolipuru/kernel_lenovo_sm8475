/*****************************************************************************
* File: device.h
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
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/semaphore.h>

#ifndef DEVICE_H
#define DEVICE_H

/*==========================================================================*/
/* CONSTANTS                                                                */
/*==========================================================================*/
static const char * const pctl_names[] = {
    "snt_reset_reset",
    "snt_reset_normal",
    "snt_hostirq_active",
    "snt_trig0irq_active",
};

#define BOOT_INIT_BUFFER_SIZE  4096
/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/
enum {
  BUS_TYPE_NONE = 0,
  BUS_TYPE_SPI  = 1,
  BUS_TYPE_I2C  = 2,
  BUS_TYPE_MAX,
};

struct snt8100fsr {
    struct device           *dev;
    struct spi_device       *spi;
    struct i2c_client       *i2c;
    struct pinctrl          *snt_pinctrl;
    struct pinctrl_state    *pinctrl_state[ARRAY_SIZE(pctl_names)];
    int                     hostirq_gpio;
    int                     trig0irq_gpio;
    int                     trig1irq_gpio;
    int                     trig2irq_gpio;
    int                     ldo_gpio;
    int                     rst_gpio;
    int                     bus_type;

    // Rate to sample frames from the sensor
    int                     frame_rate;

    //Rate when device suspend (e.g. probe) is called
    int                     suspended_frame_rate;

    //// true if this i2c dev is to wake up the chip
    bool                    wake_i2c_device;
    struct class            *class;
    struct device           *device;

    // SysFS lock for sysfs calls
    struct mutex            track_report_sysfs_lock;

    // Serial bus lock (i2c/spi)
    struct mutex            sb_lock;
    u32                     spi_freq_hz;
    u32                     i2c_freq_khz;
    struct track_report     *track_reports;
    uint16_t                track_reports_count;
    uint16_t                track_reports_frame;

    // Frame streams left to receive from the hardware device for logging
    uint32_t                frame_stream_count;

    uint32_t                d1_stream_count;

    uint32_t                get_sys_param_id;
    uint32_t                get_sys_param_val;
    uint32_t                get_sys_param_status;
    struct   sc_command *   get_sys_param_cmd;
    uint32_t                set_sys_param_id;
    uint32_t                set_sys_param_val;
    uint32_t                set_sys_param_status;
    struct semaphore        sc_wf_rsp_req;
    struct semaphore        sc_wf_rsp;

    int                     op_profile;

    // fwupdate state variables
    struct file*    fwupdate_file;
    int             fwupdate_size;
    int             fwupdate_tx_mtu;
    int             fwupdate_tot_mtu;
    int             fwupdate_major;
    int             fwupdate_address;
    int             fwupdate_status;
    bool            context_fwd;
    int             status_fwd;
    int             fwupdate_retry;

    // read_flash_reg_partr variables
    uint16_t    *   reg_part_buf;
    int             frp_max_size;
    int             frp_cur_size;
    int             frp_tx_offset;

    // update_regs/reg_script variables
    int             regs_cmd_id;

    // event log variables
    struct file *   event_log_file;

    // get_reg variables
    uint16_t   *    get_reg_buf;
    int             get_reg_id;
    int             get_reg_num;

    int             active_sc_cmd;

    int		    enable_demo;
    int		    factory_mode;
    // DYNAMIC_PWR_CTL state variables
    struct semaphore wake_req;
    struct semaphore wake_rsp;
    int              wake_rsp_result;
    int              enable_dpc_flag;

    // determine the boot process finish or not
    bool             probe_done;

    struct regulator *vdd;
    struct regulator *vddio;
    bool            raw_report;
    bool            tap0_slider;
    bool            tap3_slider;
    char            suspend_profile[BOOT_INIT_BUFFER_SIZE];
    int             sz_suspend_profile;
    char            resume_profile[BOOT_INIT_BUFFER_SIZE];
    int             sz_resume_profile;

    //Suspend/Resume toggle
    bool             sys_suspend;
    bool             sys_active_irq;

};

/*==========================================================================*/
/* EXTERNS                                                                  */
/*==========================================================================*/
// The currently enabled SysFS device
extern struct snt8100fsr *snt8100fsr_g;

// The i2c device to wake the snt8100fsr
extern struct snt8100fsr *snt8100fsr_wake_i2c_g;

/*==========================================================================*/
/* PROTOTYPES                                                               */
/*==========================================================================*/
int snt_spi_device_init(struct spi_device *spi,
                        struct snt8100fsr *snt8100fsr);

int snt_i2c_device_init(struct i2c_client *i2c,
                        struct snt8100fsr *snt8100fsr);

int snt_suspend(struct device *dev);
int snt_resume(struct device *dev);
int snt_device_hw_reset(struct snt8100fsr *snt8100fsr);

#endif // PROTOTYPE_H
