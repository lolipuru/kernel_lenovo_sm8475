/*****************************************************************************
* File: i2c-bus.c
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
*
*
*****************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#define CONFIG_I2C_MODULE
#include <linux/i2c.h>

#include "config.h"
#include "device.h"
#include "memory.h"
#include "serial_bus.h"
#include "main.h"
#include "event.h"
#include "hardware.h"
#include "sysfs.h"
#include "utils.h"
#include "debug.h"

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/
#define MAX_CMD_FIFO        256
#define CMD_HDR_SIZE        8
#define MAX_CMD_BYTES       (MAX_CMD_FIFO + 2*CMD_HDR_SIZE)

#define DATA_SIZE           (17 * 1024)

/*==========================================================================*/
/* LOCAL PROTOTYPES                                                         */
/*==========================================================================*/
int snt_i2c_read(struct snt8100fsr *snt8100fsr,
                 int num_read,
                 uint8_t *data_in);
int snt_i2c_write(struct snt8100fsr *snt8100fsr,
                  int num_write,
                  uint8_t *data_out);

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/

/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/
static uint32_t *request_out;
static uint8_t  *status_in;
static uint8_t  *data_in;
static uint8_t  *data_out;

/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/
int snt_i2c_open(struct i2c_client *i2c) {
    // Allocate our data buffers in contiguous memory for DMA support
    status_in = memory_allocate(I2C_FIFO_SIZE, GFP_DMA);
    if (status_in == NULL) {
        PRINT_CRIT("status_in = memory_allocate(%d) failed", I2C_FIFO_SIZE);
        return -1;
    }

    request_out = memory_allocate(I2C_FIFO_SIZE, GFP_DMA);
    if (request_out == NULL) {
        PRINT_CRIT("request_out = memory_allocate(%d) failed", I2C_FIFO_SIZE);
        return -1;
    }

    data_in = memory_allocate(DATA_SIZE, GFP_DMA);
    if (data_in == NULL) {
        PRINT_CRIT("data_in = memory_allocate(%d) failed", DATA_SIZE);
        return -1;
    }

    data_out = memory_allocate(DATA_SIZE, GFP_DMA);
    if (data_out == NULL) {
        PRINT_CRIT("data_out = memory_allocate(%d) failed", DATA_SIZE);
        return -1;
    }

    PRINT_INFO("I2C Address: 0x%02X", i2c->addr);

    PRINT_DEBUG("done");
    return 0;
}

void snt_i2c_close(struct snt8100fsr *snt8100fsr) {
    PRINT_FUNC();

    if (request_out) {
        memory_free(request_out);
        request_out = NULL;
    }

    if (status_in) {
        memory_free(status_in);
        status_in = NULL;
    }

    if (data_in) {
        memory_free(data_in);
        data_in = NULL;
    }

    if (data_out) {
        memory_free(data_out);
        data_out = NULL;
    }

    PRINT_DEBUG("done");
}

/*
 * snt_i2c_get_boot_status() can only be called during firmware loading
 * prior to the last interrupt saying the system is ready.
 */
uint8_t snt_i2c_get_boot_status(struct snt8100fsr *snt8100fsr) {
    uint8_t u8Data;
    int ret;

    ret = snt_i2c_read(snt8100fsr, 1, &u8Data);
    if (ret) {
        PRINT_CRIT("snt_i2c_read() failed");
        return 0xFF;
    }
    else {
        PRINT_DEBUG("Download Status %d", u8Data);
        return u8Data;
    }
}


/*
 * Wake the device up over the I2C bus by issuing a write of 4 bytes
 * to the I2C Wake Device which operates on a different I2C address.
 */
int snt_i2c_wake_device(struct snt8100fsr *snt8100fsr) {
    int ret;
    uint8_t buf[4] = {0, 0, 0, 0};

    /*PRINT_FUNC();*/

    // We will use the I2C Wake Device for this command
    if (snt8100fsr_wake_i2c_g == NULL) {
        PRINT_CRIT("Unable to wake device due to missing wake I2C device");
        return -1;
    }

    ret = snt_i2c_write(snt8100fsr_wake_i2c_g, 4, buf);

    if (ret) {
        PRINT_DEBUG("snt_i2c_write(snt8100fsr_wake_i2c, 4, buf) failed!");
        return ret;
    }

    /*PRINT_DEBUG("done");*/
    return 0;
}

int snt_i2c_read(struct snt8100fsr *snt8100fsr,
                 int num_read,
                 uint8_t *data_in) {

    int count;
    //PRINT_FUNC("%d bytes", num_read);

    count = i2c_master_recv(snt8100fsr->i2c, data_in, num_read);
    if (count < 0) {
        PRINT_CRIT("I2C write failed, error = %d\n", count);
        return -1;
    } else if (count != num_read) {
        PRINT_CRIT("I2C read of %d bytes only read %d bytes",
                   num_read, count);
        return -1;
    }

    PRINT_DEBUG("%d bytes read", num_read);
    return 0;
}

int snt_i2c_write(struct snt8100fsr *snt8100fsr,
                  int num_write,
                  uint8_t *data_out) {
    int count;
#ifdef SNT_I2C_WRITE_DEBUG_VERBOSE
    PRINT_FUNC("%d bytes", num_write);
#endif
    count = i2c_master_send(snt8100fsr->i2c, data_out, num_write);
    if (count < 0) {
        PRINT_CRIT("I2C write failed, error = %d\n", count);
        return -1;
    } else if (count != num_write) {
        PRINT_CRIT("I2C write of %d bytes only wrote %d bytes",
                   num_write, count);
        return -1;
    }
#ifdef SNT_I2C_WRITE_DEBUG_VERBOSE
    PRINT_DEBUG("%d bytes written", num_write);
#endif
    return 0;
}

int snt_i2c_read_fifo_pkt(struct snt8100fsr *snt8100fsr,
                      uint16_t reg,
                      uint16_t len,
                      uint8_t *in_val) {
    uint8_t addr_phase[2];
    int count;
#ifdef SNT_I2C_READ_FIFO_PKT_DEBUG_VERBOSE
    PRINT_FUNC("len %d", len);
#endif
    addr_phase[0] = (reg >= 0x100) ? (0x80 | (reg >> 8)) : reg;
    if (len > I2C_MAX_PKT_SIZE_BYTES) {
        PRINT_CRIT("Warning. Max I2C read is %d bytes, truncating.\n", I2C_MAX_PKT_SIZE_BYTES);
        len = I2C_MAX_PKT_SIZE_BYTES;
    }

    if (len == 0) {
        PRINT_CRIT("ERROR: Must read at least 1 word\n");
        return -1;
    }

    addr_phase[1] = (len / 2) - 1;
    count = i2c_master_send(snt8100fsr->i2c, addr_phase, 2);
    if (count != 2) {
        PRINT_CRIT("I2C header write failed len = %d\n", count);
        return -1;
    }

    count = i2c_master_recv(snt8100fsr->i2c, in_val, len);
    if (count < 0) {
        PRINT_CRIT("I2C read failed, error code = %d", count);
        return count;
    } else if (count != len) {
        PRINT_CRIT("I2C Read failed len=%d (expected %d)\n",
                   len, count);
        return -1;
    }
#ifdef SNT_I2C_READ_FIFO_PKT_DEBUG_VERBOSE
    PRINT_DEBUG("done");
#endif
    return 0;
}


int snt_i2c_read_fifo(struct snt8100fsr *snt8100fsr,
                      uint16_t reg,
                      uint16_t len,
                      uint8_t *in_val) {
    int ret = 0;

   //PRINT_FUNC("reg 0x%X", reg);

    while (len != 0 && ret == 0) {
        uint16_t pkt_len = (len > I2C_MAX_PKT_SIZE_BYTES) ? I2C_MAX_PKT_SIZE_BYTES : len;
        len -= pkt_len;
        ret = snt_i2c_read_fifo_pkt(snt8100fsr, reg, pkt_len, in_val);
        if (ret != 0) {
            PRINT_CRIT("i2c pkt read failed at len %d", len);
        }
        in_val += pkt_len;
    }
    //PRINT_DEBUG("done");
    return ret;
}


int snt_i2c_write_fifo_pkt(struct snt8100fsr *snt8100fsr,
                       uint16_t reg,
                       uint16_t len,
                       uint8_t *out_val) {
    int write_len;
    int count;

    //PRINT_FUNC("len %d", len);

    data_out[0] = (reg >= 0x100) ? (0x80 | (reg >> 8)) : reg;
    if (len > I2C_MAX_PKT_SIZE_BYTES) {
      PRINT_CRIT("Warning. Max i2c register write is %d bytes, truncating.\n", I2C_MAX_PKT_SIZE_BYTES);
      len = I2C_MAX_PKT_SIZE_BYTES;
    }
    if (len == 0) {
      PRINT_CRIT("ERROR: Must write at least 1 word\n");
      return -1;
    }

    data_out[1] = (len / 2) - 1;
    memcpy(&data_out[2], out_val, len);
    write_len = len + 2;

    /* Debug output of the hex values of the data
    for(count = 0; count < write_len; count++) {
        PRINT_DEBUG("%02X (%d)", data_out[count], data_out[count]);
    }*/

    count = i2c_master_send(snt8100fsr->i2c, data_out, write_len);
    if (count < 0) {
        PRINT_CRIT("I2C read failed, error code = %d", count);
        return count;
    } else if (count != write_len) {
      PRINT_CRIT("ERROR: I2C Write failed len=%d (expected %d)\n", count, write_len);
      return -1;
    }

    //PRINT_DEBUG("done");
    return 0;
}

int snt_i2c_write_fifo(struct snt8100fsr *snt8100fsr,
                       uint16_t reg,
                       uint16_t len,
                       uint8_t *out_val) {
    int ret = 0;
    VERBOSE("reg 0x%X", reg);
    while (len != 0 && ret == 0) {
        uint16_t pkt_len = (len > I2C_MAX_PKT_SIZE_BYTES) ? I2C_MAX_PKT_SIZE_BYTES : len;
        len -= pkt_len;
        ret = snt_i2c_write_fifo_pkt(snt8100fsr, reg, pkt_len, out_val);
        if (ret != 0) {
            PRINT_CRIT("i2c pkt write failed at len %d", len);
        }
        out_val += pkt_len;
    }
    VERBOSE("done");
    return ret;
}

/*==========================================================================*/
/* Device Probe/Remove/Resume/Suspend                                       */
/*==========================================================================*/
#ifdef USE_I2C_BUS
static int snt_i2c_probe(struct i2c_client *i2c,
                         const struct i2c_device_id *id)
{
    int ret;
    struct device *dev = &i2c->dev;
    struct device_node *np = dev->of_node;
    struct snt8100fsr *snt8100fsr;

    PRINT_FUNC();

    PRINT_INFO("Driver Version %s", SNT_VERSION);

    snt8100fsr = memory_allocate(sizeof(*snt8100fsr),
                                 GFP_KERNEL);

    if(!snt8100fsr) {
        PRINT_CRIT("failed to allocate memory for struct snt8100fsr");
        return -ENOMEM;
    }

    memset(snt8100fsr, 0, sizeof(*snt8100fsr));

    snt8100fsr->bus_type = BUS_TYPE_I2C;
    snt8100fsr->dev = dev;
    i2c_set_clientdata(i2c, snt8100fsr);
    snt8100fsr->i2c = i2c;

    /* We use two I2C devices to communicate with the snt8100fsr chip.
     * The main address is used for general communication, the secondary
     * address is used only to awaken the device when it is in sleep mode.
     * We must detect which I2C device is being initialized, the main one,
     * or the secondary address.
     */
    if (of_property_read_bool(np, "wake-device")) {
        PRINT_INFO("Wake I2C device discovered");
        snt8100fsr->wake_i2c_device = true;
        snt8100fsr_wake_i2c_g = snt8100fsr;

        // We don't do any more init for this device
        PRINT_DEBUG("done");
        return 0;
    }

    PRINT_INFO("Main I2C device discovered");
    snt8100fsr->wake_i2c_device = false;
    #ifdef UPLOAD_FIRMWARE
    snt8100fsr->status_fwd = FWUPDATE_YET;
    #else
    snt8100fsr->status_fwd = FWUPDATE_SUCCESS;
    #endif
    snt8100fsr->fwupdate_retry = FIRMWARE_RETRY_COUNT;
    snt8100fsr->probe_done = false;
    snt8100fsr->factory_mode = 0;
    snt8100fsr->sys_suspend = false;
    snt8100fsr->sys_active_irq = false;

    // Save this as our main device and to be used for sysFS access
    snt8100fsr_g = snt8100fsr;

    #ifdef DYNAMIC_PWR_CTL
    // create wake semaphore
    sema_init(&snt8100fsr_g->wake_rsp, 0);
    sema_init(&snt8100fsr_g->wake_req, 0);
    snt8100fsr_g->enable_dpc_flag = DYNAMIC_PWR_CTL;
    #endif

    // Set up semaphores to control waiting for sonacomm response
    sema_init(&snt8100fsr_g->sc_wf_rsp_req, 0);
    sema_init(&snt8100fsr_g->sc_wf_rsp, 0);

    ret = main_init();
    if (ret) {
        PRINT_CRIT("main_init() failed");
        return ret;
    }

    ret = snt_i2c_device_init(i2c, snt8100fsr);
    if(ret) {
        PRINT_CRIT("snt_i2c_device_init() failed");
        return ret;
    }

    ret = snt_i2c_open(snt8100fsr->i2c);
        if (ret) {
        PRINT_CRIT("snt_i2c_open() failed");
        return ret;
    }

    // Start our sysfs interface
    snt_sysfs_init(snt8100fsr, true);

#if defined (UPLOAD_FIRMWARE_ASYNC_MODE)
    /*
     * This is original method inplemented in the driver.
     * It upload the firmware asynchronously.
     * When finished, it will call start_event_processing() in event.c
     */
    PRINT_INFO("Reset the chip, then do firmware upload asynchronous");

    /* Reset the chip before firmware upload */
    snt8100fsr->probe_done = true;
    ret = snt_device_hw_reset(snt8100fsr);

    /* Start to upload firmware to the chip */
    upload_firmware(snt8100fsr, FIRMWARE_LOCATION);
#elif defined (UPLOAD_FIRMWARE_SYNC_MODE)
    PRINT_INFO("to call start_event_processing...");
    start_event_processing(snt8100fsr);

    PRINT_INFO("Reset the chip after irq is enabled, then firmware upload will be started automatically");

    /* Reset the chip before firmware upload */
    snt8100fsr->probe_done = true;
    ret = snt_device_hw_reset(snt8100fsr);
#else
    PRINT_INFO("to call start_event_processing...");
    start_event_processing(snt8100fsr);
    snt8100fsr->probe_done = true;
#endif


    PRINT_DEBUG("done");
    return 0;
}

static int snt_i2c_remove(struct i2c_client *i2c)
{
    struct snt8100fsr *snt8100fsr;
    PRINT_FUNC();

    /* Check if this is the i2c_wake_device used to awaken a sleeping
     * snt8100fsr chip.
     */
    snt8100fsr = i2c_get_clientdata(i2c);
    if (snt8100fsr && snt8100fsr->wake_i2c_device) {
        PRINT_INFO("removing wake_i2c_device");
        return 0;
    }

    // Else we are on the main i2c device, uninit the sysfs interface
    if (snt8100fsr_g) {
        snt_sysfs_init(snt8100fsr_g, false);
    }

    main_exit();
    return 0;
}

static int snt_i2c_suspend(struct device *dev)
{
    int ret = 0;
    /*PRINT_FUNC();*/
    ret = snt_suspend(dev);
    /*PRINT_DEBUG("done");*/
    return ret;
}

static int snt_i2c_resume(struct device *dev)
{
    int ret = 0;
    /*PRINT_FUNC();*/
    ret = snt_resume(dev);
    /*PRINT_DEBUG("done");*/
    return ret;
}

static struct i2c_device_id snt_i2c_id[] = {
    {SENTONS_DRIVER_NAME, 0},
    {SENTONS_WAKE_DRIVER_NAME, 1},
    {},
};

static const struct dev_pm_ops snt_i2c_pm_ops = {
    .suspend = snt_i2c_suspend,
    .resume = snt_i2c_resume,
};

static struct i2c_driver snt_i2c_driver = {
    .driver = {
           .name = SENTONS_DRIVER_NAME,
           .owner = THIS_MODULE,
           .pm = &snt_i2c_pm_ops,
           },
    .probe = snt_i2c_probe,
    .remove = snt_i2c_remove,
    .id_table = snt_i2c_id,
};

MODULE_DEVICE_TABLE(i2c, snt_i2c_id);

module_i2c_driver(snt_i2c_driver);
#endif
