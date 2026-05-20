/*****************************************************************************
* File: utils.c
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
#include <linux/time.h>
#include <linux/version.h>

#include "config.h"
#include "serial_bus.h"
#include "debug.h"
#include "event.h"
#include "file.h"
#include "memory.h"
#include "sonacomm.h"
#include "sysfs.h"

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/
#define OFFSET_REG_ID   0
#define OFFSET_REG_LEN  1
#define OFFSET_LSB      2
#define OFFSET_MSB      3

/*==========================================================================*/
/* CONSTANTS                                                                */
/*==========================================================================*/

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/

/*==========================================================================*/
/* LOCAL PROTOTYPES                                                         */
/*==========================================================================*/

/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/

/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/
uint32_t get_time_in_ms(void) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 20, 0))
    struct timespec64 tv;
    ktime_get_boottime_ts64(&tv);
    return (tv.tv_sec * 1000) + (tv.tv_nsec / (NSEC_PER_SEC / 1000));
#else
    struct timespec tv;
    get_monotonic_boottime(&tv);
    return (tv.tv_sec * 1000) + (tv.tv_nsec / (NSEC_PER_SEC / 1000));
#endif
}

/*
 * Parses a string in base 10 decimal format or base 16 hexidecimal format
 * and autodetects which format to use based on if the string prefix "0x" is
 * present, then hexidecimal will be used. Returns 0 on success, -1 on error.
 */
int string_to_uint32(const char *str, uint32_t *value) {
    long     tmp;

    if(strncmp(str, "0x", 2) == 0) {
        if(kstrtol(&str[2], 16, &tmp) == 0) {
            *value = (uint32_t)tmp;
            return 0;
        }
    } else {
        if(kstrtol(str, 10, &tmp) == 0) {
            *value = (uint32_t)tmp;
            return 0;
        }
    }

    return -1;
}

int read_register(struct snt8100fsr *snt8100fsr,
                  int reg,
                  void *value) {
    int ret=0;

    // Read the event register
    mutex_lock(&snt8100fsr->sb_lock);
    ret = sb_read_register(snt8100fsr, reg, value);
    if (ret) {
        PRINT_CRIT("sb_read_register() failed");
    } else {
        VERBOSE("0x%04x == 0x%04X", reg, *((uint16_t *)value));
    }
    mutex_unlock(&snt8100fsr->sb_lock);
    return ret;
}

int write_register(struct snt8100fsr *snt8100fsr,
                   int reg,
                   void *value) {
    int ret=0;

    PRINT_INFO("0x%04x = 0x%04x", reg, *((uint16_t *)value));

    // Read the event register
    mutex_lock(&snt8100fsr->sb_lock);
    ret = sb_write_register(snt8100fsr, reg, value);
    if (ret) {
        PRINT_CRIT("sb_write_register() failed");
    }
    mutex_unlock(&snt8100fsr->sb_lock);
    //PRINT_DEBUG("done");
    return ret;
}

int cust_write_registers(void *dev, int reg, int num, void *value) {

    int ret=0;
    struct snt8100fsr *snt8100fsr = (struct snt8100fsr *) dev;

    mutex_lock(&snt8100fsr->sb_lock);
    ret = sb_write_fifo(snt8100fsr, reg, num*2, value);
    if (ret) {
        PRINT_CRIT("cust_write_registers() failed (%d)", ret);
    }
    mutex_unlock(&snt8100fsr->sb_lock);
    return ret;
}

int cust_read_registers(void *dev, int reg, int num, void *value) {

    int ret=0;
    struct snt8100fsr *snt8100fsr = (struct snt8100fsr *) dev;

    mutex_lock(&snt8100fsr->sb_lock);
    ret = sb_read_fifo(snt8100fsr, reg, num*2, value);
    if (ret) {
        PRINT_CRIT("cust_read_registers() failed (%d)", ret);
    } else {
    }
    mutex_unlock(&snt8100fsr->sb_lock);
    return ret;
}

void boot_init_reg_req(struct snt8100fsr *snt8100fsr,
                      const char *buf,
                      size_t count)
{
    int ret;
    uint8_t buf_data[256] = { 0 };
    uint16_t reg_data[128] = { 0 };
    int reg_id;
    int len;
    int dataIdx;
    int next_reg_idx = 0;
    int times=0;

    /*PRINT_FUNC();*/
    ret = ReadRegList(buf, count, (uint8_t*) &buf_data, sizeof(buf_data));
    VERBOSE("buf = %s, count = %zu", buf, count);

    reg_id = buf_data[OFFSET_REG_ID];
    len = buf_data[OFFSET_REG_LEN];

    mutex_lock(&snt8100fsr->sb_lock);

    /* Parsing buf_data to reg_id/len/reg_data only when {reg_id} and {len}
     * are not equal to zero, maybe there is better way to stop the while-loop */
    while ((reg_id != 0x00) && (len != 0x00)) {
        //PRINT_INFO("Enter while: reg_id = %d, len = %d, times = %d", reg_id, len, times);
        for(dataIdx = 0; dataIdx < len; dataIdx++) {
            reg_data[dataIdx] = (buf_data[next_reg_idx+OFFSET_MSB+dataIdx*2] << 8) |
                                 buf_data[next_reg_idx+OFFSET_LSB+dataIdx*2];
            //PRINT_DEBUG("{%d} (%d / %d) [%x]", reg_id, dataIdx, len, reg_data[dataIdx]);
        }

        //PRINT_INFO("Before sb_write_fifo, reg_id = %d, len = %d, times = %d", reg_id, len, times);
        ret = sb_write_fifo(snt8100fsr_g, reg_id, len*2, reg_data);
        if (ret) {
            PRINT_CRIT("Unable to write to registers: %d, reg: 0x%02X", ret, (uint16_t)reg_id);
            break;
        }

        next_reg_idx += (2 + len*2);
        reg_id = buf_data[next_reg_idx+OFFSET_REG_ID];
        len = buf_data[next_reg_idx+OFFSET_REG_LEN];
        //PRINT_DEBUG("End while, reg_id = %d, len = %d, times = %d", reg_id, len, times);
        times++;
    }

    mutex_unlock(&snt8100fsr->sb_lock);
    VERBOSE("done");
    return;
}

void snt8100_read_profile(const char *file_location, char *buf_ptr, int *sz_read)
{
    struct file *ptrFile;
    int numRead;
    int profile_size;

    *sz_read = 0;
    PRINT_INFO("Read profile from %s", file_location);
    numRead = file_open(file_location, O_RDONLY, 0, &ptrFile);
    if(numRead) {
        PRINT_DEBUG("Unable to open file %s, error %d", file_location, numRead);
        return;
    }

    numRead = file_size(ptrFile, &profile_size);
    if (numRead) {
        PRINT_DEBUG("Unable to get file size error %d", numRead);
        return;
    }

    if (profile_size >= BOOT_INIT_BUFFER_SIZE) 
        return;

    numRead = file_read(ptrFile, 0, buf_ptr, profile_size);
    if (numRead > 0) {
        *sz_read = numRead;
        PRINT_INFO("NOTE:\n%s\n", buf_ptr);
    }
}

int snt8100_read_cfgbank_reg(uint16_t cfgbank_id, uint16_t cfgbank_reg, uint16_t* reg_val)
{
    uint16_t cfg_bank_read[3] = { 0, 0, 0x01 | (cfgbank_id << 8)};
    uint16_t ret = 1, gst_enabled = 0;
    uint16_t tap_size = 0;
    uint16_t *snt_profile_tap = NULL;

    if (cfgbank_reg%2) { PRINT_WARN("cfgbank_reg is not even"); return ret; }

    mutex_lock(&snt8100fsr_g->sb_lock);
    // Disabel gesture processing.
    if (sb_write_register(snt8100fsr_g, 0x01, &gst_enabled)) 
        goto err_handler_read;

    // Read config bank as a whole.
    if (sb_write_fifo(snt8100fsr_g, 0x2c, 3*2, (void *)cfg_bank_read)) 
        goto err_read_handler;

    if (sb_read_register(snt8100fsr_g, 0x200, &tap_size)) 
        goto err_read_handler;

    snt_profile_tap = (uint16_t *)memory_allocate(tap_size, GFP_DMA);
    if (sb_read_fifo(snt8100fsr_g, 0x200, tap_size, (void *)snt_profile_tap))
        goto err_read_handler;

    *reg_val = snt_profile_tap[cfgbank_reg/2];
    VERBOSE("Read CfgBankID:%d, 0x%x - 0x%x", cfgbank_id, cfgbank_reg, *reg_val);
    ret = 0;

err_read_handler:
    if (snt_profile_tap != NULL) {
        memory_free(snt_profile_tap);
        snt_profile_tap = NULL;
    }
    // Enable gesture processing again.
    gst_enabled = 1;
    if (sb_write_register(snt8100fsr_g, 0x01, &gst_enabled)) {
        PRINT_CRIT("Failed to re-enable gesture processing");
        ret = 1;
    }

err_handler_read:
    mutex_unlock(&snt8100fsr_g->sb_lock);

    return ret;
}

int snt8100_write_cfgbank_reg(uint16_t cfgbank_id, uint16_t cfgbank_reg, uint16_t reg_val)
{
    uint16_t cfg_bank_write[3] = { cfgbank_reg, 2, 0x02 | (cfgbank_id << 8)};
    uint16_t cfg_bank_commit[3] = { 0, 0, 0x03 | (cfgbank_id << 8) };
    uint16_t ret = 1, gst_enabled = 0;

    mutex_lock(&snt8100fsr_g->sb_lock);
    // Disabel gesture processing.
    if (sb_write_register(snt8100fsr_g, 0x01, &gst_enabled)) 
        goto err_handler;

    VERBOSE("Write CfgBankID:%d, 0x%x - 0x%x", cfgbank_id, cfgbank_reg, reg_val);

    if (sb_write_fifo(snt8100fsr_g, 0x2c, 3*2, (void *)cfg_bank_write))
        goto err_rw_handler;

    if (sb_write_register(snt8100fsr_g, 0x200, &reg_val))
        goto err_rw_handler;

    if (sb_write_fifo(snt8100fsr_g, 0x2c, 3*2, (void *)cfg_bank_commit)) {
        PRINT_CRIT("Failed to commit config bank.");
    }
    ret = 0;

err_rw_handler:
    // Enable gesture processing again.
    gst_enabled = 1;
    if (sb_write_register(snt8100fsr_g, 0x01, &gst_enabled)) {
        PRINT_CRIT("Failed to re-enable gesture processing");
        ret = 1;
    }

err_handler:
    mutex_unlock(&snt8100fsr_g->sb_lock);

    return ret;
}

int do_write_reg_script_file(struct snt8100fsr *snt8100fsr, const char *file_location, bool wake_device)
{

#if 1
    char *buffer;
    int ret;
    int numRead;

    if (!strcmp(SLEEP_PROFILE_LOCATION, file_location)) {
        buffer = snt8100fsr->suspend_profile;
        numRead = snt8100fsr->sz_suspend_profile;
        VERBOSE("Apply prefetched sleep profile.");
    } else if (!strcmp(IDLE_PROFILE_LOCATION, file_location)) {
        buffer = snt8100fsr->resume_profile;
        numRead = snt8100fsr->sz_resume_profile;
        VERBOSE("Apply prefetched idle profile.");
    } else {
        struct file *boot_init_file;
        int boot_init_file_size;
        PRINT_INFO("Initialize the register from %s", file_location);
        ret = file_open(file_location, O_RDONLY, 0, &boot_init_file);
        if(ret) {
            PRINT_DEBUG("Unable to open file %s, error %d",
                    file_location, ret);
            return -1;
        }
        ret = file_size(boot_init_file, &boot_init_file_size);
        if (ret) {
            PRINT_DEBUG("Unable to get file size error %d", ret);
            return -1;
        }

        buffer = (char *)memory_allocate(BOOT_INIT_BUFFER_SIZE, GFP_DMA);
        memset(buffer, 0, BOOT_INIT_BUFFER_SIZE);
        numRead = file_read(boot_init_file, 0, buffer, boot_init_file_size);
    }

    if (numRead <= 0) {
        PRINT_WARN("Bad read from reg script: %s", file_location);
        return -1;
    }

#ifdef DYNAMIC_PWR_CTL
    if (wake_device == true) {
        VERBOSE("wake device");
        if (snt_activity_request()) {
            PRINT_CRIT("snt_activity_request() failed");
            return -1;
        }
    }
#endif
    boot_init_reg_req(snt8100fsr_g, buffer, (size_t)numRead);
    PRINT_DEBUG("done.");
    return 0;

#else
    char *p_buf = NULL;
    struct file *p_file = NULL;
    int ret;
    int frp_size;
    int bytes;

    PRINT_INFO("Initialize the register from %s", file_location);

    PRINT_DEBUG("opening frp file %s", file_location);
    ret = file_open(file_location, O_RDONLY, 0, &p_file);
    if (ret) {
        PRINT_CRIT("Unable to open file %s, error %d", file_location, ret);
        return -1;
    }
    ret = file_size(p_file, &frp_size);
    if (ret) {
        PRINT_CRIT("Unable to get file size error %d", ret);
        goto cleanup;
    }
    PRINT_DEBUG("Frp file size = %d", frp_size);
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
    if (wake_device == true) {
        PRINT_DEBUG("wake device");
        ret = snt_activity_request();
    }

    if (ret) {
        PRINT_CRIT("snt_activity_request() failed %d", ret);
        return -1;
    }
    #endif

    enable_write_flash_reg_part_req(snt8100fsr, p_buf, frp_size, mc_reg_script);

cleanup:
    if (p_file != NULL) {
        file_close(p_file);
    }
    if (p_buf != NULL) {
        memory_free(p_buf);
    }
    PRINT_DEBUG("done.");
    return 0;
#endif
}

#ifdef DYNAMIC_PWR_CTL
/*
 * Utility function to perform Activity Request and wait for completion.
 *
 * set Request sema.
 * perform Activity Request. Back out Sema request if fails.
 * wait for Response sema.
 */

int snt_activity_request_force(int force)
{
    int ret = 0;
    VERBOSE("start:%d", snt8100fsr_g->enable_dpc_flag);

    // protect with sb_lock as  we call sb_wake_device
    mutex_lock(&snt8100fsr_g->sb_lock);

    if (snt8100fsr_g->enable_dpc_flag || force) {

        up(&snt8100fsr_g->wake_req);    // post request

        ret = sb_wake_device(snt8100fsr_g);
        if (ret) {
            PRINT_CRIT("sb_wake_device() failed, ret=%d", ret);

            /* attempt to back out request */
            if (down_trylock(&snt8100fsr_g->wake_req)) {
                VERBOSE("Wake Req already consumed");
                mutex_unlock(&snt8100fsr_g->sb_lock);
                if (down_trylock(&snt8100fsr_g->wake_rsp)) {
                    VERBOSE("Wake Rsp already consumed");
                }
            } else {
                mutex_unlock(&snt8100fsr_g->sb_lock);
            }
            PRINT_CRIT("reset chip...");
            mutex_lock(&snt8100fsr_g->sb_lock);
            ret = snt_device_hw_reset(snt8100fsr_g);
            mutex_unlock(&snt8100fsr_g->sb_lock);
            return ret;
        }
        mutex_unlock(&snt8100fsr_g->sb_lock);

        // wait for response from driver irpt thread
        do {
            ret = down_timeout(&snt8100fsr_g->wake_rsp, msecs_to_jiffies(3 * MSEC_PER_SEC));
            if (ret || snt8100fsr_g->wake_rsp_result)
                PRINT_WARN("recv wake rsp, ret=%d, res=%d", ret, snt8100fsr_g->wake_rsp_result);
            else
                VERBOSE("recv wake rsp, ret=%d, res=%d", ret, snt8100fsr_g->wake_rsp_result);
        } while (ret == -EINTR);

        // check  if wake_rsp succeeded
        if (ret==0)
            ret = snt8100fsr_g->wake_rsp_result;
	else {
            /* attempt to back out request */
            if (down_trylock(&snt8100fsr_g->wake_req)) {
                VERBOSE("Wake Req already consumed");
                if (down_trylock(&snt8100fsr_g->wake_rsp)) {
                    VERBOSE("Wake Rsp already consumed");
                }
            }
            mutex_lock(&snt8100fsr_g->sb_lock);
            snt8100fsr_g->get_reg_id = 0x0;
            snt8100fsr_g->get_reg_num = 0x1;
            PRINT_DEBUG("reg_id=%d, num_reg=%d", snt8100fsr_g->get_reg_id, snt8100fsr_g->get_reg_num);

            if (snt8100fsr_g->get_reg_buf != NULL) {
                memory_free(snt8100fsr_g->get_reg_buf);
            }
            snt8100fsr_g->get_reg_buf = (uint16_t*)memory_allocate(snt8100fsr_g->get_reg_num * 2, GFP_DMA);
            if (snt8100fsr_g->get_reg_buf == NULL) {
                PRINT_CRIT("memory_allocate(%d) failed", snt8100fsr_g->get_reg_num * 2);
            } else {
                ret = sb_read_fifo(snt8100fsr_g,
                                   snt8100fsr_g->get_reg_id,
                                   snt8100fsr_g->get_reg_num * 2,
                                   snt8100fsr_g->get_reg_buf);

                if (ret) {
                    PRINT_CRIT("Unable to read from registers: %d", ret);
                    if (snt8100fsr_g->get_reg_buf != NULL) memory_free(snt8100fsr_g->get_reg_buf);
                        snt8100fsr_g->get_reg_buf = NULL;
                    snt8100fsr_g->get_reg_num = 0;
                }
                else
                    PRINT_DEBUG("reg_id=%d, val=0x%x, ret=%d", snt8100fsr_g->get_reg_id, snt8100fsr_g->get_reg_buf[0], ret );
                mutex_unlock(&snt8100fsr_g->sb_lock);
            }
	}

    } else {
        mutex_unlock(&snt8100fsr_g->sb_lock);
    }

    VERBOSE("done:%d", ret);
    return ret;
}
#endif

void set_tap_gesture_enable(void *dev, uint16_t tap_id, uint16_t enable)
{
    uint16_t cfg_bank_read[3] = { 0, 0, 0x0801};
    uint16_t tap_size = 0;
    uint16_t *snt_profile_tap;
    uint16_t cfg_bank_write[3] = { 8+tap_id*20, 2, 0x0802};
    uint16_t cfg_bank_commit[3] = { 0, 0, 0x0803};
    uint16_t tap_ctrl = 0;


    cust_write_registers(dev, 0x2c, 3, cfg_bank_read);
    cust_read_registers(dev, 0x200, 1, &tap_size);
    snt_profile_tap = (uint16_t *)memory_allocate(tap_size, GFP_DMA);

    cust_read_registers(dev, 0x200, tap_size/2, snt_profile_tap);

    if (enable == 1) {
        tap_ctrl = snt_profile_tap[4+10*tap_id] | 0x1;
        PRINT_DEBUG("Enable: 0x%x -> 0x%x", snt_profile_tap[4+10*tap_id], tap_ctrl);
    } else {
        tap_ctrl = snt_profile_tap[4+10*tap_id] & 0xfffe;
        PRINT_DEBUG("Disable: 0x%x -> 0x%x", snt_profile_tap[4+10*tap_id], tap_ctrl);
    }

    cust_write_registers(dev, 0x2c, 3, cfg_bank_write);
    cust_write_registers(dev, 0x200, 1, &tap_ctrl);
    cust_write_registers(dev, 0x2c, 3, cfg_bank_commit);
}

void set_slider_gesture(void *dev, uint16_t slider_id, uint16_t *reg_val) 
{
    uint16_t cfg_bank_write[3] = { slider_id*12, 12, 0x0b02};
    uint16_t cfg_bank_commit[3] = { 0, 0, 0x0b03};
    /*switch (slider_id) {*/
    /*case 0: cfg_bank_write[0] = 0; break;*/
    /*case 1: cfg_bank_write[0] = 36; break;*/
    /*case 2: cfg_bank_write[0] = 24; break;*/
    /*case 3: cfg_bank_write[0] = 12; break;*/
    /*}*/
    cust_write_registers(dev, 0x2c, 3, cfg_bank_write);
    cust_write_registers(dev, 0x200, 6, reg_val);
    cust_write_registers(dev, 0x2c, 3, cfg_bank_commit);
}

void set_slider_gesture_enable(void *dev, uint16_t slider_id, uint16_t enable)
{
	uint16_t cfg_bank_read[3] = { 0, 0, 0x0b01};
	uint16_t slider_size = 0;
	uint16_t *snt_profile_slider;
	uint16_t cfg_bank_write[3] = { slider_id*12, 2, 0x0b02};
	uint16_t cfg_bank_commit[3] = { 0, 0, 0x0b03};
	uint16_t slider_ctrl = 0;

	cust_write_registers(dev, 0x2c, 3, cfg_bank_read);
	cust_read_registers(dev, 0x200, 1, &slider_size);
	snt_profile_slider = (uint16_t *)memory_allocate(slider_size, GFP_DMA);

	cust_read_registers(dev, 0x200, slider_size/2, snt_profile_slider);

	if (enable == 1) {
		slider_ctrl = snt_profile_slider[8*slider_id] | 0x8000;
		PRINT_DEBUG("Enable: 0x%x -> 0x%x", snt_profile_slider[8*slider_id],
					slider_ctrl);
	} else {
		slider_ctrl = snt_profile_slider[8*slider_id] & 0x7fff;
		PRINT_DEBUG("Disable: 0x%x -> 0x%x", snt_profile_slider[8*slider_id],
					slider_ctrl);
	}

    /*switch (slider_id) {*/
    /*case 0: cfg_bank_write[0] = 0; break;*/
    /*case 1: cfg_bank_write[0] = 36; break;*/
    /*case 2: cfg_bank_write[0] = 24; break;*/
    /*case 3: cfg_bank_write[0] = 12; break;*/
    /*}*/
	cust_write_registers(dev, 0x2c, 3, cfg_bank_write);
	cust_write_registers(dev, 0x200, 1, &slider_ctrl);
	cust_write_registers(dev, 0x2c, 3, cfg_bank_commit);

}

void set_touch_enable(void *dev, uint16_t reg_val) 
{
    cust_write_registers(dev, 1, 1, &reg_val);
}

// check whether the target gesture was set up.
bool is_gas_gesture(uint8_t bar_id)
{
    uint16_t tap_ctrl;
    bool flag = false;
    if (!snt8100_read_cfgbank_reg(11, bar_id*12, &tap_ctrl)) {
        PRINT_INFO("ForceS bit: 0x%x", tap_ctrl);

        if (tap_ctrl & (1<<12)) {
            flag = true;
        }
    }
    return flag;
}
