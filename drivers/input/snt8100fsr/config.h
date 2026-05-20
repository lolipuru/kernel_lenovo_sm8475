/*****************************************************************************
* File: config.h
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
#include <linux/interrupt.h>
#include "firmware.h"
#include "serial_bus.h"

#ifndef CONFIG_H
#define CONFIG_H

#define SNT_VERSION     "Rel.3.6.9.15"

#define MAX_TRACK_REPORTS 12
#define MT_MAX_BARID       2
#define MT_MAX_TOUCHBAR    5
#define MT_MAX_SLOTS      (MT_MAX_BARID*MT_MAX_TOUCHBAR)

//Bradly chen new setting for snowbird
#define SNT_BAR0_CENTER_MAX 413
#define SNT_BAR0_CENTER_MIN 59
#define SNT_BAR1_CENTER_MAX 1400
#define SNT_BAR1_CENTER_MIN 0

#define DISPLAY_POSITION_X_MAX 1440

#define BAR0_POSITION_Y_MAX 1147
#define BAR0_POSITION_Y_MIN 530
#define BAR1_POSITION_Y_MAX 820
#define BAR1_POSITION_Y_MIN 50

#define BAR0_Y(x)   ((BAR0_POSITION_Y_MAX-BAR0_POSITION_Y_MIN)*(SNT_BAR0_CENTER_MAX-(x))/ \
		(SNT_BAR0_CENTER_MAX-SNT_BAR0_CENTER_MIN) + BAR0_POSITION_Y_MIN)
#define BAR1_Y(x)   ((BAR1_POSITION_Y_MAX-BAR1_POSITION_Y_MIN)*(SNT_BAR1_CENTER_MAX-(x))/ \
		(SNT_BAR1_CENTER_MAX-SNT_BAR1_CENTER_MIN) + BAR1_POSITION_Y_MIN)
#define PANEL_SIZE 2559
#define MT_MAX_TOUCHBAR    5

/*
 * Select which bus to use. Only one bus can be active at a time.
 */
//#define USE_SPI_BUS
#define USE_I2C_BUS

/* FIRMWARE_IRQ_TIMEOUT_MS: Amount of time to wait for an interrupt during
 * firmware upload in ms. The timer includes the time to transfer a single
 * payload, so ensure it's long enough.
 */

/*
 *  To support boot-from-flash mode:
 * //#define USE_SPI_BUS
 * #define USE_I2C_BUS
 *
 * in #ifdef USE_I2C_BUS
 * //#define UPLOAD_FIRMWARE
 * #define SUPPORT_FLASH
 * // If you want snt_i2c_write DBUG: message every 512 bytes, uncomment,
 * //#define SNT_I2C_WRITE_DEBUG_VERBOSE
 * //#define SNT_I2C_READ_FIFO_PKT_DEBUG_VERBOSE
 */


/*
 * Custom BUS Settings
 */
#ifdef USE_SPI_BUS
    #define SENTONS_DRIVER_NAME "snt8100fsr-spi"
    #define UPLOAD_FIRMWARE
    #define SNT_FWDL_BUF_SIZE         (1024 * 8)
    #define FIRMWARE_IRQ_TIMEOUT_MS   2000
    //#define PINCTRL_DEVICE_TREE
    // uncomment to add feature.
    // #define DYNAMIC_PWR_CTL      1       // 1 - enable by default
    					    // 0 - disable by default
#endif

#ifdef USE_I2C_BUS
    #define SENTONS_DRIVER_NAME "snt8100fsr-i2c"
    #define SENTONS_WAKE_DRIVER_NAME "snt8100fsr-i2c-wake"
    #define UPLOAD_FIRMWARE
    #ifdef UPLOAD_FIRMWARE
	//ASYNC mode is original design, where POR firmware download starts without confirming the 1st irq is received
	//SYNC mode is newly added mode, where POR firmware download only starts after receiving the 1st irq
    //#define UPLOAD_FIRMWARE_ASYNC_MODE
    #define UPLOAD_FIRMWARE_SYNC_MODE
    #endif

    // comment out to remove feature.
    #define DYNAMIC_PWR_CTL           1       // 1 - enable by default
                                              // 0 - disable by default
    //#define SUPPORT_FLASH
    #define CONFIG_I2C_MODULE                 // (used by linux/i2c.h)
    #define SNT_FWDL_BUF_SIZE         512
    #define FIRMWARE_IRQ_TIMEOUT_MS   5000    // supports 100KHz
    //#define PINCTRL_DEVICE_TREE
    //#define SNT_I2C_WRITE_DEBUG_VERBOSE
    //#define SNT_I2C_READ_FIFO_PKT_DEBUG_VERBOSE
#endif


/*
 * if this is commented out then touch reporting will not be enabled by default
 */
#define TOUCH_ENABLED_AT_STARTUP

/*
 * Set to 1 to use the DTS nodes for configuration. Refer to device.c.
 * If set to 0, then use this config.h file for configuration.
 */
#define USE_DEVICE_TREE_CONFIG 1

/*
 * Support customize key setting through profile setting
 */
#define KEY_PROFILE
enum key_profile {
	BOOT_PROFILE = 0,
	IDLE_PROFILE = 1,
	SLEEP_PROFILE = 2,
	PWROFF_PROFILE = 3
};

enum fw_update_state {
	FWUPDATE_YET = -1,
	FWUPDATE_FAIL = 0,
	FWUPDATE_SUCCESS = 1
};

/*
 * Support key event through driver report
 */
#define KEY_REPORT
#ifdef KEY_REPORT
#define TAP0_DOWN 0x1
#define TAP0_UP   0x10
#define TAP1_DOWN 0x2
#define TAP1_UP   0x20
#define TAP2_DOWN 0x4
#define TAP2_UP   0x40
#define TAP3_DOWN 0x8
#define TAP3_UP   0x80
#define SLIDE_MAX_NUM 3
#endif

// SysFS name
#define SUPPORT_SYSFS_CLASS
#define SYSFS_NAME "snt8100fsr"
#define SYSFS_CHAR_DEVICE_NAME "snt8100fsr"
#define SYSFS_DEVICE_CLASS_NAME "edge"

// SySFS permissions for showing, storing, or both
#define SYSFS_PERM_SHOW (S_IRUSR|S_IRUGO)
#define SYSFS_PERM_STORE (S_IWUSR|S_IRUGO|S_IWGRP)
#define SYSFS_PERM_SHOW_STORE (S_IWUSR|S_IRUSR|S_IRUGO|S_IWGRP)

// The default frame rate to sample frames from the sensor hardware in hz
#define DEFAULT_FRAME_RATE 50

// The location on disk of the firmware to upload to the hardware on boot
#define FIRMWARE_LOCATION   "/vendor/firmware/snt8100fsr.bin"

// The location of the profile file
#define BOOT_PROFILE_LOCATION     "/vendor/etc/snt8100/boot_profile_rscript.txt"
#define SLEEP_PROFILE_LOCATION    "/vendor/etc/snt8100/tap0_gas.txt"
#define IDLE_PROFILE_LOCATION     "/vendor/etc/snt8100/tap0_slider.txt"

// The location of the track report log file
#define TRACK_REPORT_LOG_LOCATION "/data/snt8100fsr/track_report.log"

// The location of the binary track report log file
#define TRACK_REPORT_BIN_LOG_LOCATION "/data/snt8100fsr/track_report.bin"

// The location of the deep trace file
#define DEEP_TRACE_LOCATION       "/data/snt8100fsr/deep_trace.bin"

// The location of the event log file
#define EVENT_LOG_LOCATION        "/data/snt8100fsr/event_log.bin"

// The location of the small sensor data log file
#define D1TEST_DATA_LOG_LOCATION "/data/snt8100fsr/d1test_data.log"

// The location of the small sensor data log file
#define FRAME_DATA_LOG_LOCATION "/data/snt8100fsr/frame_data.log"

// The location of the flash register partition read file
#define FRP_CAPTURE_FILE_LOCATION "/data/snt8100fsr/frp_out.txt"

// The location of the no touch data log file
#define NO_TOUCH_FRAME_DATA_LOG_LOCATION "/data/snt8100fsr/no_touch_frame_data.log"

// SPI Bus Settings if not loaded via DeviceTree nodes (bottom of file for chart)
#define SPI_MAX_SPEED_HZ    1500000
#define SPI_BUS             1
#define SPI_CHIPSELECT      0

/*
 * The amount of time in microseconds to keep chip select active to wake up
 * the device.
 */
#define SPI_WAKE_CHIP_SELECT_TIME 100

/*
 * the maximum size of an I2c bus transfer. Adjust this to fit OS xfer
 * requirement for i2c.
 */
 #define I2C_MAX_PKT_SIZE_BYTES     512

/*
 * The frame rate to use when the operating system is suspended in hz.
 * Use 5 for Squeeze to Wake mode in which the user squeezes the sensors
 * in order to wake up the device via interrupt.
 * Use 65535 for Deep Sleep in which the sensors will be disabled and the only
 * way to wake up is via sb_wake_device() call, usually made from dev resume()
 *
 * This value is saved into snt8100fsr->suspended_frame_rate and can be
 * updated dynamically at runtime in code and via sysfs in sysfs.c.
 */
#define DEFAULT_SUSPENDED_FRAME_RATE 5
#define DEEP_SLEEP_FRAME_RATE 0xFF
// Should we use the IRQ to perform the firmware upload?
#define FIRMWARE_UPLOAD_WITH_IRQ    true

/*
 * enable these defined to turn Trigger Interrupt support
 */
#define USE_TRIG0_IRQ               0
#define USE_TRIG1_IRQ               0
#define USE_TRIG2_IRQ               0

#define USE_TRIG_IRQ        (USE_TRIG0_IRQ||USE_TRIG1_IRQ||USE_TRIG2_IRQ)

// Boot Configuration Record (BCR)
// values found in firmware.h
#define BCR_LOGGING_LEVEL   BCR_LOGLVL_OFF // BCR_LOGLVL_DEFAULT

// May set to 0x2c 0x2d 0x5c 0x5d 0x00
#define BCR_ADDRESS_FILTER    0x2c

// Boot Configuration Record
// Interrupt shaping configuration. Note: DEFAULT IS STRONGLY RECOMMENDED
// AS THE FIRST RESET INTERRUPT WILL ALWAYS BE LEVEL, ACTIVE HIGH
#define BCR_INTERRUPT_POLARITY      BCR_IRQ_POLARITY_ACTIVE_HIGH
#define BCR_INTERRUPT_TYPE          BCR_IRQ_TYPE_LEVEL

// Delay in IRQ'less Firmware Uploads due to Boot Rom LOG Glitch
#define FIRMWARE_LOG_DELAY_MS 2

// For firmware uploads not using the IRQ, use this delay
#define FIRMWARE_UPLOAD_DELAY_MS 100

// Firmware download fail and retry
#define FIRMWARE_RETRY_COUNT 2

// Our interrupt name for the GPIO19 pin interrupt
#define IRQ_NAME            "snt_act_int"
#define TRIG0_NAME          "snt_trig0_int"
#define TRIG1_NAME          "snt_trig1_int"
#define TRIG2_NAME          "snt_trig2_int"

// This is used to calculate device pin value from the GPIO pin value
#define BEAGLEBONE_GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

// Our used GPIO pins
#define BEAGLEBONE_GPIO48 BEAGLEBONE_GPIO_TO_PIN(1, 16)
#define BEAGLEBONE_GPIO49 BEAGLEBONE_GPIO_TO_PIN(1, 17)
#define BEAGLEBONE_GPIO20 BEAGLEBONE_GPIO_TO_PIN(0, 20)
#define BEAGLEBONE_GPIO60 BEAGLEBONE_GPIO_TO_PIN(1, 28)

// The GPIO we will use for interrupt handling
#define IRQ_GPIO    BEAGLEBONE_GPIO49
#define TRIG0_GPIO  BEAGLEBONE_GPIO20
#define TRIG1_GPIO  BEAGLEBONE_GPIO48
#define TRIG2_GPIO  BEAGLEBONE_GPIO60

// The Interrupt number
#define IRQ_NUMBER     gpio_to_irq(IRQ_GPIO)
#define TRIG0_IRQ_NUM  gpio_to_irq(TRIG0_GPIO)
#define TRIG1_IRQ_NUM  gpio_to_irq(TRIG1_GPIO)
#define TRIG2_IRQ_NUM  gpio_to_irq(TRIG2_GPIO)

// The triggers for the interrupt handler to execute on
#define IRQ_TRIGGER     IRQF_TRIGGER_RISING
#define TRIG0_TRIGGER   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
#define TRIG1_TRIGGER   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
#define TRIG2_TRIGGER   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING

/*
 * From: https://groups.google.com/forum/#!topic/beagleboard/Lo0GEl1RdbU
 * Beagle Bone Black SPI Clock Speed Chart
 * CLKD Divide by   SPI_CLK [Hz]
 * 0    1       48,000,000
 * 1    2       24,000,000
 * 2    4       12,000,000
 * 3    8       6,000,000
 * 4    16      3,000,000
 * 5    32      1,500,000
 * 6    64      750,000
 * 7    128     375,000
 * 8    256     187,500
 * 9    512     93,750
 * 10   1024    46,875
 * 11   2048    23,438
 * 12   4096    11,719
 * 13   8192    5,859
 * 14   16384   2,930
 * 15   32768   1,465
 */

/*
 * Set up 64 vs 32 bit arch detection. Note this only works for arm right now
 */
#ifdef __aarch64__
#define ENVIRONMENT64
#define PTR_2_UINT uint64_t
#else
#define ENVIRONMENT32
#define PTR_2_UINT  uint32_t
#endif


#define TOP_L_BAR_ID  3
#define TOP_R_BAR_ID  2
#define BOT_L_BAR_ID  1
#define BOT_R_BAR_ID  0

#endif // CONFIG_H

