/*
 * Cypress USB Type-C and PD Controller Driver
 *
 * Author: Dudley Du <dudl@cypress.com>
 *
 * Copyright (C) 2016 Cypress Semiconductor, Inc.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#ifndef _CYCCG_CORE_H
#define _CYCCG_CORE_H

#include <linux/types.h>
#include <asm/unaligned.h>
#include "cyccg.h"

#define MEM_ALIGN_CONST 8
#define MEM_ALLIGNED_SIZE(_size)	\
	((((_size) + (MEM_ALIGN_CONST) - 1) / (MEM_ALIGN_CONST)) *	\
		(MEM_ALIGN_CONST))

#define CYCCG_DEVICE_INIT_RETRIES		10
#define CYCCG_DEVICE_INIT_RETRIES_INTERVAL	500	/* ms */
#define CYCCG_POLLING_TIMER_INTERVAL		20	/* ms */
#define CYCCG_POLLING_TIMER_SLOW_INTERVAL	1000	/* ms */
#define CYCCG_POLLING_IDLE_THRESHOLD		30000	/* ms */

enum ccg_version {
	CCG_UNKNOWN,
	CCG1,
	CCG2,
	CCG3,
	CCG4,
};
#define CCGX_SILICON_ID_MASK	0xff00
#define CCG1_SILICON_ID_MASK	0x0400
#define CCG2_SILICON_ID_MASK	0x1400
#define CCG3_SILICON_ID_MASK	0x1d00
#define CCG4_SILICON_ID_MASK	0x1800

enum ccg_type {
	CCG_NOTEBOOK_MOBILE_MONITOR,
	CCG_POWER_ADAPTER,
	CCG_CABLE,
};

enum ccg_flash_mode {
	CCG_LEGACY_BOOT_MODE,	/* HPI_VERSION_1 */
	CCG_DUAL_FW_MODE,	/* HPI_VERSION_2 */
};

enum cyccg_work_state {
	CYCCG_WORK_STATE_NONE,
	CYCCG_WORK_STATE_QUEUED,
	CYCCG_WORK_STATE_RUNNING,
};

#include "cyccg_hpi.h"

/*
 * ccg_info is the structure used to store the basic information of
 * a CCG chipset device.
 */
struct ccg_info {
	u16 silicon_id;
	u8 uuid[HPI_FW_IMAGE_UUID_SIZE];
	enum ccg_version ccg_ver;
	enum hpi_version hpi_ver;
	u16 flash_row_size;
	u16 num_port;
	enum hpi_dp_mode dp_mode;

	enum ccg_flash_mode flash_mode;
	u16 bl_last_row_num;
	u16 fw1_start_row_num;
	u16 fw2_start_row_num;

	struct ccg_bootloader_type blt_type;
};

struct ccg_state {
	bool is_in_dp_mode;
	enum ccg_fw_mode_type running_mode;
	struct hpi_ccg_fw_version ccg_fw_vers;
};

struct cyccg_platform_data {
	enum hpi_dp_mode dp_mode;
	int reset_gpio;		/* gpio pin for host reset CCGx ship. */
	int intr_gpio;		/* gpio pin for CCGx chip notify host. */

	/*
	 * TODO(Customer):
	 * Add more gpio pin definitions based on the board design.
	 */
};

struct cyccg {
	struct device *dev;
	int irq;
	struct ccg_bus_operations *bus_ops;
	struct cyccg_platform_data *platform_data;
	struct completion done;

	/* The cyccg instance and device global lock. */
	struct mutex mlock;
	spinlock_t slock;
	/*
	 * Indicates the probe routine done, aimed to avoid the CCG power on
	 * with system startup caused RESET_COMPLETED event to queue and
	 * rerun the device init process.
	 */
	bool probe_done;

	struct ccg_info ccg_info;
	struct ccg_state ccg_state;

	struct work_struct device_init_work;
	enum cyccg_work_state device_init_work_state;
	struct work_struct fw_work;
	enum cyccg_work_state fw_work_state;

	/*
	 * The instance operates on Device Specific Registers only
	 * or on mixed with Device Specific and PD Port Specific Registers,
	 * such as CCG1 and CCG2.
	 */
	struct hpi_device hpi_dev;
	/* The instances operate on PD Port Specific Registers only. */
	struct hpi_device *ports[HPI_MAX_PD_PORTS];

	/*
	 * No IRQ polling timer. It's aimed to help working on the IRQ not
	 * not working situation.
	 */
	spinlock_t polling_timer_slock;
	struct delayed_work polling_timer;
	unsigned long last_irq_time;	/* time in jiffies */
	enum cyccg_work_state irq_handler_state;
	unsigned long idle_time;	/* ms */

	struct mutex sysfs_mlock;
	int sysfs_port_index;
	enum ccg_type sysfs_port_ccg_type;
	enum vdm_sop_type sysfs_port_sop_type;
	u32 reg_addr;
	size_t reg_size;
};

void cyccg_device_init_work(struct work_struct *work);
int cyccg_port_init(struct hpi_device *port);
bool cyccg_busying_check_and_set(struct hpi_device *hpidev);
void cyccg_wait_for_idle(struct hpi_device *hpidev);
void cyccg_set_to_busying_state(struct hpi_device *hpidev, bool busying);
enum ccg_version ccg_silicon_id_to_ccg_version(u16 silicon_id);
int cyccg_device_reset_and_init(struct cyccg *cyccg, bool force_reset);
void cyccg_queue_device_init_work(struct cyccg *cyccg);
int cyccg_get_port_list_string(struct cyccg *cyccg, char *buf, size_t size);

void cyccg_start_polling_timer(struct cyccg *cyccg, bool polling_timer_reset);

#include "cyccg_update.h"

int cyccg_sysfs_init(struct cyccg *cyccg);

#endif  /* #ifndef _CYCCG_H */