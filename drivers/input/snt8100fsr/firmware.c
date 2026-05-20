/*****************************************************************************
* File: firmware.c
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
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/delay.h>

#include "config.h"
#include "irq.h"
#include "serial_bus.h"
#include "workqueue.h"
#include "event.h"
#include "file.h"
#include "memory.h"
#include "device.h"
#include "firmware.h"
#include "utils.h"
#include "debug.h"
#include "hardware.h"

/*==========================================================================*/
/* CONSTANTS                                                                */
/*==========================================================================*/
#define MAX_PAYLOAD_NUM     1000 // 1 = Hdr only
#define MAX_PAYLOAD_BYTES   (65 * 1024)

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/
#define OWN_WORK 1
#if OWN_WORK
static struct delayed_work g_fwd_work;
#endif

struct upload_work {
  struct snt8100fsr   *snt8100fsr;

  char                *filename;
  struct file         *f;
  int                 payload_num;
  int                 file_offset;

  int                 num_write;
  uint32_t            size;
  uint32_t            pay_write;

  bool                final_irq;
  bool                waiting_for_irq;

  uint32_t            firmware_upload_time;
  uint32_t            payload_upload_time;
  uint32_t            total_bytes_written;

  uint8_t             *data_in;
  uint8_t             *data_out;
};

/*==========================================================================*/
/* PROTOTYPES                                                               */
/*==========================================================================*/
static void upload_wq_func(struct work_struct *lwork);
static void upload_firmware_internal(struct upload_work *w);
static irqreturn_t irq_fwd_handler_top(int irq, void *dev);

static void download_wq_func(struct work_struct *lwork);
static void download_firmware_internal(struct upload_work *w);

static int open_firmware_file(struct upload_work *w);
static error_t firmware_waiting_irq(struct upload_work *w);
static error_t firmware_transfer_payloads(struct upload_work *w);
static void firmware_cleanup(struct upload_work *w, error_t err);
/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/
static uint32_t bcr_log_level = 0;
static uint32_t bcr_addr      = 0;
static uint32_t bcr_irpt      = 0;

static struct upload_work *work = NULL;


/*==========================================================================*/
/* BOOT FIRMWARE CONFIG                                                     */
/*==========================================================================*/
void set_bcr_addr(uint32_t addr) {
    bcr_addr = (addr & BCR_I2CADDR_MASK) << BCR_I2CADDR_POS;
}

void set_bcr_log_lvl(uint32_t l) {
    bcr_log_level = (l & BCR_LOGLVL_MASK) << BCR_LOGLVL_POS;
}

void set_bcr_irpt_lvl(uint32_t l) {
    bcr_irpt |= (l & BCR_IRPTLVL_MASK) << BCR_IRPTLVL_POS;
}

void set_bcr_irpt_pol(uint32_t p) {
    bcr_irpt |= (p & BCR_IRPTPOL_MASK) << BCR_IRPTPOL_POS;
}

void set_bcr_irpt_dur(uint32_t d) {
    bcr_irpt |= (d & BCR_IRPTDUR_MASK) << BCR_IRPTDUR_POS;
}

/*
 * set_bcr_word()
 *
 * [31:24] - I2C address. valid 0,0x2c,0x2d,0x5c,0x5d (0 means "all 4")
 * [23:11] - Reserved.
 * [10:8]  - Logging Level
 * [7:2]   - Edge Duration in 78MHz tics. "0" means 10 tics (default).
 * [1]     - Interrupt Level. 0 - Level, 1 - Edge
 * [0]     - Interrupt Polarity. 0 - Active High, 1 - Active Low
 */
uint32_t set_bcr_word(void) {

  uint32_t bcr = bcr_addr
               | bcr_log_level
               | bcr_irpt;
  PRINT_DEBUG("Boot Cfg Record = 0x%08x", bcr);
  return bcr;
}

/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/
int upload_firmware_fwd(struct snt8100fsr *snt8100fsr, char *filename) {
    PRINT_FUNC();
    // We can't upload the firmware directly as we need to pace
    // ourselves for how fast the hardware can accept data. So we
    // must setup a background thread with a workerqueue and process
    // the upload a piece at a time.
    work = memory_allocate(sizeof(struct upload_work), GFP_KERNEL);
	if (NULL != work)
        memset(work, 0, sizeof(struct upload_work));
    if (!work) {
        PRINT_CRIT(OOM_STRING);
        return -ENOMEM;
    }

    /* To workaround the kernel panic(alignment fault),
     * use a global delayed_work structure */
    INIT_DELAYED_WORK(&g_fwd_work, download_wq_func);

    work->snt8100fsr = snt8100fsr;
    work->filename = FIRMWARE_LOCATION;

    // Allocate our data buffers in contiguous memory for DMA support
    work->data_in = memory_allocate(SNT_FWDL_BUF_SIZE, GFP_DMA);
    if (work->data_in == NULL) {
        PRINT_CRIT("data_in = memory_allocate(%d) failed", SNT_FWDL_BUF_SIZE);
        return -ENOMEM;
    }

    work->data_out = memory_allocate(SNT_FWDL_BUF_SIZE, GFP_DMA);
    if (work->data_out == NULL) {
        PRINT_CRIT("data_out = memory_allocate(%d) failed", SNT_FWDL_BUF_SIZE);
        return -ENOMEM;
    }

    // Setup our Boot Configuration Record settings
    set_bcr_log_lvl(BCR_LOGGING_LEVEL);
    set_bcr_addr(BCR_ADDRESS_FILTER);
    set_bcr_irpt_lvl(BCR_INTERRUPT_TYPE);
    set_bcr_irpt_pol(BCR_INTERRUPT_POLARITY);

    workqueue_queue_work(&g_fwd_work, 0);
    return 0;
}


int upload_firmware(struct snt8100fsr *snt8100fsr, char *filename) {
    // We can't upload the firmware directly as we need to pace
    // ourselves for how fast the hardware can accept data. So we
    // must setup a background thread with a workerqueue and process
    // the upload a piece at a time.
    work = memory_allocate(sizeof(struct upload_work), GFP_KERNEL);
    if (NULL != work)
        memset(work, 0, sizeof(struct upload_work));
    if (!work) {
        PRINT_CRIT(OOM_STRING);
        return -ENOMEM;
    }

    /* To workaround the kernel panic(alignment fault),
     * use a global delayed_work structure */
    INIT_DELAYED_WORK(&g_fwd_work, upload_wq_func);

    work->snt8100fsr = snt8100fsr;
    work->filename = FIRMWARE_LOCATION;

    // Allocate our data buffers in contiguous memory for DMA support
    work->data_in = memory_allocate(SNT_FWDL_BUF_SIZE, GFP_DMA);
    if (work->data_in == NULL) {
        PRINT_CRIT("data_in = memory_allocate(%d) failed", SNT_FWDL_BUF_SIZE);
        return -ENOMEM;
    }

    work->data_out = memory_allocate(SNT_FWDL_BUF_SIZE, GFP_DMA);
    if (work->data_out == NULL) {
        PRINT_CRIT("data_out = memory_allocate(%d) failed", SNT_FWDL_BUF_SIZE);
        return -ENOMEM;
    }

    // Setup our logging level
    set_bcr_log_lvl(BCR_LOGGING_LEVEL);
    set_bcr_addr(BCR_ADDRESS_FILTER);
    set_bcr_irpt_lvl(BCR_INTERRUPT_TYPE);
    set_bcr_irpt_pol(BCR_INTERRUPT_POLARITY);

    workqueue_queue_work(&g_fwd_work, 0);
    return 0;
}

int irq_handler_fwd( void) {

    int delay;

    // Add a delay in milliseconds if our boot loader is logging output.
    // During it's logging, it can't receive data, so we delay a bit.
    // We have a known amount of delay, so it's always safe.
    if (BCR_LOGGING_LEVEL != BCR_LOGLVL_OFF) {
        delay = FIRMWARE_LOG_DELAY_MS;
    } else {
        delay = 0;
    }

    work->waiting_for_irq = false;
    if (workqueue_mod_work(&g_fwd_work, delay) == false) {
      workqueue_queue_work(&g_fwd_work, delay);
    }
    return 0;
}

static void upload_wq_func(struct work_struct *lwork) {
    upload_firmware_internal(work);
    return;
}

static void download_wq_func(struct work_struct *lwork) {
    download_firmware_internal(work);
    return;
}

static irqreturn_t irq_fwd_handler_top(int irq, void *dev) {
    int delay;

    // Add a delay in milliseconds if our boot loader is logging output.
    // During it's logging, it can't receive data, so we delay a bit.
    // We have a known amount of delay, so it's always safe.
    if (BCR_LOGGING_LEVEL != BCR_LOGLVL_OFF) {
        delay = FIRMWARE_LOG_DELAY_MS;
    } else {
        delay = 0;
    }

    work->waiting_for_irq = false;
    if (workqueue_mod_work(&g_fwd_work, delay) == false) {
      workqueue_queue_work(&g_fwd_work, delay);
    }

    return IRQ_HANDLED;
}

static int open_firmware_file(struct upload_work *w) {
    int ret = 0, retry = 40;
    PRINT_FUNC("0x%p", w);
    // If we haven't opened the firmware file, do so
    if (w->f == 0) {
        PRINT_DEBUG("Opening file: %s", w->filename);
        ret = file_open(w->filename, O_RDONLY, 0, &w->f);
        while (ret < 0 && retry--) {
            ret = file_open(w->filename, O_RDONLY, 0, &w->f);
            if(ret) {
                PRINT_ERR("%d trial, error:%d", retry, ret);
            }
            msleep(1000);
        }
    }
    return ret;
}

static error_t firmware_waiting_irq( struct upload_work *w) {
    int     ret = 0;
    PRINT_FUNC();
    /* If we are here, and are waiting for an IRQ,
     * then we have a timeout
     * condition as the IRQ didn't occurr.
     */
    if(FIRMWARE_UPLOAD_WITH_IRQ) {
        if (w->waiting_for_irq) {
            PRINT_CRIT("Timeout waiting for interrupt. Please ensure hardware "
                       "is correctly wired and firmware image is valid.");
            w->waiting_for_irq = false;
            /* read the status code for last packet */
            ret = sb_get_boot_status(w->snt8100fsr);
            return E_TIMEOUT;
        }

        w->waiting_for_irq = true;

        /* Queue a job to check for an IRQ timeout. The timer includes the
         * time to transfer a payload, so ensure it's long enough.
         */
        workqueue_queue_work(&g_fwd_work, FIRMWARE_IRQ_TIMEOUT_MS);
    }
    return E_SUCCESS;
}

static error_t firmware_transfer_payloads( struct upload_work *w) {
    int         ret;
    int         num_write;
    uint32_t    payload_write;
    uint32_t    payload_size;
    uint32_t    size;
    uint32_t    payload_duration;
    PRINT_FUNC();
    /*
     * Read size of next payload from the file.
     * Actually reading full
     * HW_FIFO_SIZE so will need to write this out
     */
    num_write = file_read(w->f, w->file_offset, (void *)w->data_out, SPI_FIFO_SIZE);
    if (num_write <= 0) {
        PRINT_INFO("EOF Reached. Firmware data uploaded.");
        return E_FINISHED;
    }

    w->file_offset += SPI_FIFO_SIZE;
    w->payload_num++;

    // Size is first long word of buffer read
    payload_size = ((uint32_t*)w->data_out)[0];
    size = payload_size;
    PRINT_INFO("Payload %d = %d Bytes (%d inc 'size' field)", w->payload_num, size, size + 4);

    // If this is first segment, then pad word is boot cfg
    if (w->payload_num == 1) {
        ((uint32_t*)w->data_out)[1] = set_bcr_word();

        // Record the start time of the first payload to measure the total
        // time the firmware upload takes to complete.
        w->firmware_upload_time = get_time_in_ms();
    }

    // Record the start of the transfer of this payload
    w->payload_upload_time = get_time_in_ms();

    // Write the size out to chip
    ret = sb_read_and_write(w->snt8100fsr, num_write, w->data_out, w->data_in);
    if (ret == FIRMWARE_ALREADY_LOADED_RESULT) {
        PRINT_NOTICE("Existing firmware already loaded...");
        return E_ALREADY;
    } else if (ret) {
        PRINT_ERR("sb_write() failed");
        return E_FAILURE;
    }

    w->total_bytes_written += num_write;
    size -= (SPI_FIFO_SIZE - sizeof(size));

    // Fatal if read_size not /8
    if (size % SPI_FIFO_SIZE) {
        PRINT_ERR("Size not multiple of %d", SPI_FIFO_SIZE);
        return E_BADSIZE;
    }

    // Get payload and write it out in SNT_FWDL_BUF_SIZE chunks
    payload_write = 0;
    while (size != 0 && payload_write < MAX_PAYLOAD_BYTES) {
        int read_size = min((unsigned int)SNT_FWDL_BUF_SIZE, size);

        num_write = file_read(w->f, w->file_offset, (void*)w->data_out, read_size);
        if (num_write <= 0) {
            PRINT_DEBUG("EOF Reached. Stopping...");
            return E_BADREAD;
        }

        w->file_offset += read_size;

        /* Write the data to the bus */
        ret = sb_read_and_write(w->snt8100fsr, num_write, w->data_out, w->data_in);
        if (ret == FIRMWARE_ALREADY_LOADED_RESULT) {
            PRINT_NOTICE("Existing firmware already loaded...");
            return E_ALREADY;
        } else if (ret) {
            PRINT_ERR("sb_write() failed");
            return E_BADWRITE;
        }

        w->total_bytes_written += num_write;
        size -= num_write;
        payload_write += num_write;
    }

    // Calculate how long this total payload took
    payload_duration = get_time_in_ms() - w->payload_upload_time;
    if (payload_duration == 0)
        payload_duration = 1;

    PRINT_DEBUG("Payload %d took %dms at %d kbit/s",
                w->payload_num,
                payload_duration,
                ((payload_size * 8 / payload_duration) * 1000) / 1024);

    if (w->payload_num >= MAX_PAYLOAD_NUM) {
        PRINT_DEBUG("Max Payload Reached. Stopping...");
        return E_TOOMANY;
    }

    if (FIRMWARE_UPLOAD_WITH_IRQ) {
        PRINT_INFO("Waiting for IRQ for next payload");
    } else {
        PRINT_INFO("Queue firmware update workqueue");
        workqueue_queue_work(&g_fwd_work, FIRMWARE_UPLOAD_DELAY_MS);
    }
    return E_SUCCESS;
}

static void firmware_cleanup(struct upload_work *w, error_t err) {
    PRINT_FUNC();
    if(err <= E_SUCCESS) {
        int duration;
        duration = get_time_in_ms() - w->firmware_upload_time;
        if (duration == 0)
            duration = 1;

        PRINT_DEBUG("Total %d bytes took %dms at %d kbit/s",
                    w->total_bytes_written,
                    duration,
                    ((w->total_bytes_written * 8 / duration) * 1000) / 1024);
    }

    memory_free(work->data_in);
    memory_free(work->data_out);

    if(FIRMWARE_UPLOAD_WITH_IRQ) {
        // Cancel our irq timeout work queue item
        workqueue_cancel_work(&g_fwd_work);
    }

    if (w->f) {
        filp_close(w->f, NULL);
        w->f = NULL;
        w->file_offset = 0;
        w->payload_num = 0;
        w->total_bytes_written = 0;
    }

    PRINT_DEBUG("Finished");
    return;
}

static void upload_firmware_internal(struct upload_work *w) {
    /*
     * insmod driver causes firmware to be uploaded to chip
     */
    int         ret;
    error_t err = E_SUCCESS;
    PRINT_FUNC("0x%p", w);

    // If we haven't opened the firmware file, do so
    if (w->f == 0) {
        ret = open_firmware_file(w);
        if (ret) {
            err = E_BADOPEN;
            goto cleanup;
        }
        else {
            // Register our interrupt handler
            if(FIRMWARE_UPLOAD_WITH_IRQ) {
                snt_irq_db.top = irq_fwd_handler_top;
                snt_irq_db.gpio_num = snt8100fsr_g->hostirq_gpio;
                snt_irq_db.irq_num = gpio_to_irq(snt_irq_db.gpio_num);
                irq_handler_register(&snt_irq_db);

            }
        }
    }

    err = firmware_waiting_irq(w);
    if (err >= E_FAILURE) {
        PRINT_CRIT("firmware_waiting_irq err %d", (int)err);
        goto cleanup;
    }
    err = firmware_transfer_payloads(w);
    if (err >= E_FAILURE) {
        PRINT_CRIT("firmware_transfer_payloads err %d", (int)err);
    }
    else if (err == E_FINISHED) { goto cleanup; }
    return;
cleanup:
    firmware_cleanup( w, err);
    if(FIRMWARE_UPLOAD_WITH_IRQ) {
        irq_handler_unregister(&snt_irq_db);  //[dy] unique to upload_firmware
    }

    if (err >= E_FAILURE)
        w->snt8100fsr->status_fwd = FWUPDATE_FAIL;
    else
        w->snt8100fsr->status_fwd = FWUPDATE_SUCCESS;

    start_event_processing(w->snt8100fsr);
    return;
}

static void download_firmware_internal(struct upload_work *w) {
    /*
     * Chip reset sets event register bit FWD so driver must
     * download chip firmware.
     * IRQ already set up by start_event_processing
     */
    error_t err = E_SUCCESS;
    int         ret;
    PRINT_FUNC("0x%p", w);

    // If we haven't opened the firmware file, do so
    if (w->f == 0) {
        ret = open_firmware_file(w);
        if(ret) {
            err = E_BADOPEN;
            goto cleanup;
        }
    }

    err = firmware_waiting_irq(w);
    if (err >= E_FAILURE) {
        PRINT_CRIT("firmware_waiting_irq err %d", (int)err);
        goto cleanup;
    }
    err = firmware_transfer_payloads(w);
    if (err >= E_FAILURE) {
        PRINT_CRIT("firmware_transfer_payloads err %d", (int)err);
        goto cleanup;
    }
    else if (err == E_FINISHED) { goto cleanup; }
    return;
cleanup:
    firmware_cleanup( w, err);

    if (err >= E_FAILURE) {
        if (w->snt8100fsr->fwupdate_retry > 0) {
            w->snt8100fsr->fwupdate_retry--;
            PRINT_INFO("reset the chip...retry (%d)", w->snt8100fsr->fwupdate_retry);
            mutex_lock(&w->snt8100fsr->sb_lock);
            msleep(3000);
            w->snt8100fsr->context_fwd = false;
            ret = snt_device_hw_reset(w->snt8100fsr);
            mutex_unlock(&w->snt8100fsr->sb_lock);
            PRINT_INFO("reset the chip done");
        }
        else {
            PRINT_CRIT("firmware update failed");
            w->snt8100fsr->fwupdate_retry = FIRMWARE_RETRY_COUNT;
            w->snt8100fsr->status_fwd = FWUPDATE_FAIL;
        }
        return;
    }
    else {
        w->snt8100fsr->fwupdate_retry = FIRMWARE_RETRY_COUNT;
        w->snt8100fsr->status_fwd = FWUPDATE_SUCCESS;
    }

    set_context_fwd_done(w->snt8100fsr);
    return;
}
