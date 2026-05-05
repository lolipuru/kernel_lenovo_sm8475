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

#ifndef _CYCCG_HPI_H
#define _CYCCG_HPI_H

struct cyccg;

enum hpi_version {
	HPI_VERSION_1,
	HPI_VERSION_2,
};

/* CCG pre-configured DP mode, can set and parsed from DT. */
enum hpi_dp_mode {
	DP_UNKNOWN	= 0,
	DP_SOURCE	= 1,
	DP_SINK		= 2,
};

enum hpi_rw_mode {
	HPI_READ,
	HPI_WRITE,
};

enum hpi_sync_mode {
	HPI_SYNC,
	HPI_ASYNC,
};

#define HPI_VALUE(_hpi_ver, _hpi_v1_value, _hpi_v2_value)	\
	(((_hpi_ver) == HPI_VERSION_1) ? (_hpi_v1_value) : (_hpi_v2_value))
#define HPI(_NAME, _hpi_ver)	\
	HPI_VALUE((_hpi_ver), (HPI_V1_##_NAME), (HPI_V2_##_NAME))

#define HPI_REG_OFFSET_OF(_REG_NAME, _hpi_ver)		\
	HPI_VALUE((_hpi_ver), (HPI_V1_REG_OFFSET_OF_##_REG_NAME), \
			      (HPI_V2_REG_OFFSET_OF_##_REG_NAME))
#define HPI_REG_SIZE_OF(_REG_NAME, _hpi_ver)		\
	HPI_VALUE((_hpi_ver), (HPI_V1_REG_SIZE_OF_##_REG_NAME), \
			      (HPI_V2_REG_SIZE_OF_##_REG_NAME))
#define HPI_PORT_REG_OFFSET_OF(_REG_NAME, _hpi_ver)		\
	HPI_VALUE((_hpi_ver), (HPI_V1_REG_OFFSET_OF_PORT_##_REG_NAME), \
			      (HPI_V2_REG_OFFSET_OF_PORT_##_REG_NAME))
#define HPI_PORT_REG_SIZE_OF(_REG_NAME, _hpi_ver)		\
	HPI_VALUE((_hpi_ver), (HPI_V1_REG_SIZE_OF_##_REG_NAME), \
			      (HPI_V2_REG_SIZE_OF_##_REG_NAME))
#define HPI_TIME_OF(_REG_NAME, _hpi_ver)		\
	HPI_VALUE((_hpi_ver), (HPI_V1_TIME_OF_##_REG_NAME), \
			      (HPI_V2_TIME_OF_##_REG_NAME))

#define HPI_REG_RW_SIZE(_hpi_ver)	(HPI(MAX_REG_RW_SIZE, (_hpi_ver)))
#define HPI_REG_ADDR_SIZE(_hpi_ver)	(HPI(REG_ADDR_SIZE, (_hpi_ver)))

/*
 * In device_mode, two bits used to indicates PD ports, max value is 4.
 * Indeed, currently, the designed PD ports is 2 in CCG4.
 */
#define HPI_MAX_PD_PORTS	4
/* The _port_id value should start from 0. */
#define HPI_PORT_INTR_BIT_MASK(_port_id)		\
	(0x01 << ((_port_id) + 1))
#define IS_HPI_PORT_INTR_BIT_SET(_val, _port_id)	\
	((_val) & HPI_PORT_INTR_BIT_MASK(_port_id))

/*
 * CCG device configuration and status registers.
 * The offset values are all based on the CCG device base address.
 */
#define HPI_REG_INVALID_NA	0xffff

/* Macros and definitions for HPI Device Information Registers. */
#define HPI_V1_REG_OFFSET_OF_DEVICE_MODE	0x00
#define HPI_V2_REG_OFFSET_OF_DEVICE_MODE	0x0000
#define HPI_V1_REG_SIZE_OF_DEVICE_MODE		1
#define HPI_V2_REG_SIZE_OF_DEVICE_MODE		1
/* Indicates the image area that CCG device current running on. */
enum ccg_fw_mode_type {
	CCG_FW_MODE_TYPE_BOOTLAODER	= 0x00,

	/*
	 * For Single FW
	 */
	CCG_FW_MODE_TYPE_APP		= 0x01,

	/*
	 * For Dual FW
	 */
	/* CCG2 alias: Fail Safe Image. */
	CCG_FW_MODE_TYPE_FW_IMAGE1	= CCG_FW_MODE_TYPE_APP,
	/* CCG2 alias: Main Image. */
	CCG_FW_MODE_TYPE_FW_IMAGE2	= 0x02,
};

struct hpi_device_mode {
	/*
	 * b1-b0: CCG running mode. See enum ccg_fw_mode_type.
	 *   0 - Running in Booloader Mode
	 *   1 - Running in Firmware 1
	 *   2 - Running in Firmware 2
	 */
	u8 running_mode : 2;
	/*
	 * b3-b2: Number of PD ports supported.
	 *   0 - 1 port.
	 *   1 - 2 port.
	 *   Other values resereved.
	 */
	u8 num_port : 2;
	/*
	 * b5-b4: Flash row size.
	 *   0 - 128 bytes.
	 *   1 - 256 bytes.
	 *   Other values reserved.
	 */
	u8 flash_row_size : 2;
	u8 reserved : 1;
	/*
	 * b7: HPI version mode.
	 *   0 - HPI version 1, using single byte HPI addressing, and only
	 *	 bootloader can do flash read/write.
	 *   1 - HPI version 2, using two-byte HPI addressing. Dual firmware
	 *	 mode with two copies of firmware that can do mutual updates.
	 */
	u8 hpi_ver : 1;
} __packed;

#define HPI_V1_REG_OFFSET_OF_BOOT_MODE_REASON	0x01
#define HPI_V2_REG_OFFSET_OF_BOOT_MODE_REASON	0x0001
#define HPI_V1_REG_SIZE_OF_BOOT_MODE_REASON	1
#define HPI_V2_REG_SIZE_OF_BOOT_MODE_REASON	1
struct hpi_boot_mode_reason {
	/*
	 * b0: Boot mode request by FW.
	 *   0 - No boot mode request.
	 *   1 - EC requested through JUMP_TO_BOOT command.
	 */
	u8 boot_mode_request : 1;
	/*
	 * b1: Configuration Table Status, HPI Version 1 only.
	 *   0 - Table valid.
	 *   1 - Table invalid.
	 */
	u8 config_table_status : 1;
	/*
	 * b2: Firmware application 1 status.
	 *   0 - Application image 1 valid.
	 *   1 - Application image 1 invalid.
	 */
	u8 fw_app_1_status : 1;
	/*
	 * b3: Firmware application 2 status.
	 *   0 - Application image 2 valid.
	 *   1 - Application image 2 invalid.
	 */
	u8 fw_app_2_status : 1;
	/* b7-b4: Reserved, should be always 0. */
	u8 reserved : 4;
} __packed;

#define HPI_V1_REG_OFFSET_OF_READ_SILICON_ID	0x02
#define HPI_V2_REG_OFFSET_OF_READ_SILICON_ID	0x0002
#define HPI_V1_REG_SIZE_OF_READ_SILICON_ID	2
#define HPI_V2_REG_SIZE_OF_READ_SILICON_ID	2

#define HPI_V1_REG_OFFSET_OF_BOOT_LOADER_LAST_ROW	0x04
#define HPI_V2_REG_OFFSET_OF_BOOT_LOADER_LAST_ROW	0x0004
#define HPI_V1_REG_SIZE_OF_BOOT_LOADER_LAST_ROW		2
#define HPI_V2_REG_SIZE_OF_BOOT_LOADER_LAST_ROW		2

#define HPI_V1_REG_OFFSET_OF_INTR_REG	0x06
#define HPI_V2_REG_OFFSET_OF_INTR_REG	0x0006
#define HPI_V1_REG_SIZE_OF_INTR_REG	1
#define HPI_V2_REG_SIZE_OF_INTR_REG	1
union hpi_intr_reg {
	u8 val;
	struct {
		/*
		 * b0: DEV_INTR bit, valid for CCG1/CCG2.
		 *   0 - No response in Device specific RESPONSE register.
		 *   1 - A new response available in Device specific RESPONSE
		 *	 register.
		 */
		u8 dev_intr : 1;
		/*
		 * b1: PORT0_INTR.
		 *   0 - No response in PORT_0 specific PD_RESPONSE register.
		 *   1 - A new response available in PORT_0 specific PD_RESPONSE
		 *	 register.
		 */
		u8 port_0_intr : 1;
		/*
		 * b2: PORT1_INTR.
		 *   0 - No response in PORT_1 specific PD_RESPONSE register.
		 *   1 - A new response available in PORT_1 specific PD_RESPONSE
		 *	 register.
		 */
		u8 port_1_intr : 1;
		/* b7-b3: Reserved for future use, always ignores these bits. */
		u8 reserved : 5;	/* These bits should be ignored. */
	};
} __packed;

#define HPI_V1_REG_OFFSET_OF_JUMP_TO_BOOT    0x07
#define HPI_V2_REG_OFFSET_OF_JUMP_TO_BOOT    0x0007
#define HPI_V1_REG_SIZE_OF_JUMP_TO_BOOT	     1
#define HPI_V2_REG_SIZE_OF_JUMP_TO_BOOT	     1
#define HPI_SIGNATURE_JUMP_TO_BOOT	     0x4a	/* 'J' */
#define HPI_V1_REG_OFFSET_OF_JUMP_TO_ALT_FW  (HPI_V1_REG_OFFSET_OF_JUMP_TO_BOOT)
#define HPI_V2_REG_OFFSET_OF_JUMP_TO_ALT_FW  (HPI_V2_REG_OFFSET_OF_JUMP_TO_BOOT)
#define HPI_V1_REG_SIZE_OF_JUMP_TO_ALT_FW    (HPI_V1_REG_SIZE_OF_JUMP_TO_BOOT)
#define HPI_V2_REG_SIZE_OF_JUMP_TO_ALT_FW    (HPI_V2_REG_SIZE_OF_JUMP_TO_BOOT)
#define HPI_SIGNATURE_JUMP_TO_ALT_FW	     0x41	/* 'A' */

#define HPI_V1_REG_OFFSET_OF_RESET	0x08
#define HPI_V2_REG_OFFSET_OF_RESET	0x0008
#define HPI_V1_REG_SIZE_OF_RESET	2
#define HPI_V2_REG_SIZE_OF_RESET	2
#define HPI_SIGNATURE_RESET		0x52	/* 'R' */
enum hpi_reset_type {
	HPI_RESET_TYPE_I2C	= 0x00,
	HPI_RESET_TYPE_DEVICE	= 0x01,
};

#define HPI_V1_REG_OFFSET_OF_ENTER_FLASHING_MODE	0x0a
#define HPI_V2_REG_OFFSET_OF_ENTER_FLASHING_MODE	0x000a
#define HPI_V1_REG_SIZE_OF_ENTER_FLASHING_MODE		1
#define HPI_V2_REG_SIZE_OF_ENTER_FLASHING_MODE		1
#define HPI_SIGNATURE_ENTER_FLASHING_MODE		0x50	/* 'P' */

/* See enum ccg_fw_mode_type for the values of FW mode used in validate FW. */
#define HPI_V1_REG_OFFSET_OF_VALIDATE_FW	0x0b
#define HPI_V2_REG_OFFSET_OF_VALIDATE_FW	0x000b
#define HPI_V1_REG_SIZE_OF_VALIDATE_FW		1
#define HPI_V2_REG_SIZE_OF_VALIDATE_FW		1

/*
 * The firmware update process can be done in:
 * CCG1/CCG2, only supported in bootloader mode.
 * CCG3, supports a seamless fw update with PD contract and functioning normally
 * CCG4, requires PD contracts to be terminated and ports to be disabled
 */
#define HPI_V1_REG_OFFSET_OF_FLASH_ROW_READ_WRITE	0x0c
#define HPI_V2_REG_OFFSET_OF_FLASH_ROW_READ_WRITE	0x000c
#define HPI_V1_REG_SIZE_OF_FLASH_ROW_READ_WRITE		4
#define HPI_V2_REG_SIZE_OF_FLASH_ROW_READ_WRITE		4
#define HPI_SIGNATURE_FLASH_ROW_READ_WRITE		0x46  /* 'F' */
enum hpi_flash_row_command {
	HPI_FLASH_ROW_READ	= 0x00,
	HPI_FLASH_ROW_WRITE	= 0x01,
};
struct hpi_flash_row_rw_reg {
	u8 signature;
	u8 command;
	__le16 row_num;
} __packed;

#define HPI_V1_REG_OFFSET_OF_READ_ALL_VERSION	0x10
#define HPI_V2_REG_OFFSET_OF_READ_ALL_VERSION	0x0010
#define HPI_V1_REG_SIZE_OF_READ_ALL_VERSION	16
#define HPI_V2_REG_SIZE_OF_READ_ALL_VERSION	16
struct hpi_base_version {
	u16 build_number : 16;
	u8 patch_ver;
	u8 minor : 4;
	u8 major : 4;
} __packed;

struct hpi_app_version {
	/*
	 * The two bytes of the name is put in little-endian.
	 * e.g.: ASCII for "nb", 0x6e62, name[1]='n', name[0]='b'.
	 */
	char name[2];
	u8 external_circuit_ver;
	u8 minor : 4;
	u8 major : 4;
} __packed;

struct hpi_image_version {
	struct hpi_base_version base;
	struct hpi_app_version app;
} __packed;

/*
 * Only @fw1_app and @fw2_app can be changed through the firmware update
 * process. Bootloader version @btldr cannot be updated through the driver,
 * only can be updated though the PSoc Programmer.
 * Note, @fw2_app will be reserved as all zero on non dual firmware images'
 * device, such as CCG1 and CCG2 devices.
 * The @fw1_app is also called as Fail Safe Firmware on Power Adapter devices.
 */
struct hpi_ccg_fw_version {
	struct hpi_image_version btldr;
	struct hpi_image_version fw1_app;
	struct hpi_image_version fw2_app;	/* FW2_VERSION register */
} __packed;

#define HPI_V1_REG_OFFSET_OF_FW2_VERSION	(HPI_REG_INVALID_NA)
#define HPI_V2_REG_OFFSET_OF_FW2_VERSION	0x0020
#define HPI_V1_REG_SIZE_OF_FW2_VERSION		(HPI_REG_INVALID_NA)
#define HPI_V2_REG_SIZE_OF_FW2_VERSION		8

#define HPI_V1_REG_OFFSET_OF_FW_BINARY_LOCATION		(HPI_REG_INVALID_NA)
#define HPI_V2_REG_OFFSET_OF_FW_BINARY_LOCATION		0x0028
#define HPI_V1_REG_SIZE_OF_FW_BINARY_LOCATION		(HPI_REG_INVALID_NA)
#define HPI_V2_REG_SIZE_OF_FW_BINARY_LOCATION		4

#define HPI_V1_REG_OFFSET_OF_PDPORT_ENABLE	(HPI_REG_INVALID_NA)
#define HPI_V2_REG_OFFSET_OF_PDPORT_ENABLE	0x002c
#define HPI_V1_REG_SIZE_OF_PDPORT_ENABLE	(HPI_REG_INVALID_NA)
#define HPI_V2_REG_SIZE_OF_PDPORT_ENABLE	1

#define HPI_V1_REG_OFFSET_OF_SLEEP_CTRL		(HPI_REG_INVALID_NA)
#define HPI_V2_REG_OFFSET_OF_SLEEP_CTRL		0x002d
#define HPI_V1_REG_SIZE_OF_SLEEP_CTRL		(HPI_REG_INVALID_NA)
#define HPI_V2_REG_SIZE_OF_SLEEP_CTRL		1
#define HPI_DEEP_SLEEP_MASK			0x01
#define HPI_DEEP_SLEEP_ENABLED			0x00
#define HPI_DEEP_SLEEP_DISABLED			0x01

#define HPI_V1_REG_OFFSET_OF_BATTERY_STAT	(HPI_REG_INVALID_NA)
#define HPI_V2_REG_OFFSET_OF_BATTERY_STAT	0x002e
#define HPI_V1_REG_SIZE_OF_BATTERY_STAT		(HPI_REG_INVALID_NA)
#define HPI_V2_REG_SIZE_OF_BATTERY_STAT		1
#define HPI_BATTERY_STAT_MASK			0x01
#define HPI_DEAD_BATTERY_ENABLED		0x00
#define HPI_DEAD_BATTERY_DISABLED		0x01

#define HPI_V1_REG_OFFSET_OF_SET_APP_PRIORITY	(HPI_REG_INVALID_NA)
#define HPI_V2_REG_OFFSET_OF_SET_APP_PRIORITY	0x002f
#define HPI_V1_REG_SIZE_OF_SET_APP_PRIORITY	(HPI_REG_INVALID_NA)
#define HPI_V2_REG_SIZE_OF_SET_APP_PRIORITY	1
#define HPI_SET_APP_PRIORITY_MASK		0x03
enum hpi_app_priority {
	HPI_APP_PRIORITY_DEFAULT	= 0x00,
	HPI_APP_PRIORITY_FW1		= 0x01,
	HPI_APP_PRIORITY_FW2		= 0x02,
};

#define HPI_V1_REG_OFFSET_OF_READ_CUSTOMER_INFO		(HPI_REG_INVALID_NA)
#define HPI_V2_REG_OFFSET_OF_READ_CUSTOMER_INFO		0x0030
#define HPI_V1_REG_SIZE_OF_READ_CUSTOMER_INFO		(HPI_REG_INVALID_NA)
#define HPI_V2_REG_SIZE_OF_READ_CUSTOMER_INFO		1
#define HPI_SIGNATURE_READ_CUSTOMER_INFO		0x43  /* 'C' */
#define HPI_V1_REG_OFFSET_OF_CUSTOMER_INFO_DATA		(HPI_REG_INVALID_NA)
#define HPI_V2_REG_OFFSET_OF_CUSTOMER_INFO_DATA		0x40
#define HPI_V1_REG_SIZE_OF_CUSTOMER_INFO_DATA		(HPI_REG_INVALID_NA)
#define HPI_V2_REG_SIZE_OF_CUSTOMER_INFO_DATA		32

/*
 * PD policy/status registers.
 * The offset values are all based on the port base address.
 */
#define HPI_V1_REG_OFFSET_OF_PORT_VDM_CONTROL	0x20
#define HPI_V2_REG_OFFSET_OF_PORT_VDM_CONTROL	0x0000
#define HPI_V1_REG_SIZE_OF_PORT_VDM_CONTROL	2
#define HPI_V2_REG_SIZE_OF_PORT_VDM_CONTROL	2
struct hpi_vdm_ctrl {
	u8 vdm_mode;	/* See enum vdm_sop_type */
	u8 length;
} __packed;

#define HPI_V1_REG_OFFSET_OF_PORT_EFFECTIVE_SOURCE_PDO_MASK	0x24
#define HPI_V2_REG_OFFSET_OF_PORT_EFFECTIVE_SOURCE_PDO_MASK	0x0002
#define HPI_V1_REG_SIZE_OF_PORT_EFFECTIVE_SOURCE_PDO_MASK	1
#define HPI_V2_REG_SIZE_OF_PORT_EFFECTIVE_SOURCE_PDO_MASK	1

#define HPI_V1_REG_OFFSET_OF_PORT_EFFECTIVE_SINK_PDO_MASK	0x25
#define HPI_V2_REG_OFFSET_OF_PORT_EFFECTIVE_SINK_PDO_MASK	0x0003
#define HPI_V1_REG_SIZE_OF_PORT_EFFECTIVE_SINK_PDO_MASK		1
#define HPI_V2_REG_SIZE_OF_PORT_EFFECTIVE_SINK_PDO_MASK		1

#define HPI_V1_REG_OFFSET_OF_PORT_SELECT_SOURCE_PDO	0x26
#define HPI_V2_REG_OFFSET_OF_PORT_SELECT_SOURCE_PDO	0x0004
#define HPI_V1_REG_SIZE_OF_PORT_SELECT_SOURCE_PDO	1
#define HPI_V2_REG_SIZE_OF_PORT_SELECT_SOURCE_PDO	1
#define HPI_SIGNATURE_SELECT_SOURCE_PDO			0x53524350  /* 'SRCP' */

#define HPI_V1_REG_OFFSET_OF_PORT_SELECT_SINK_PDO	0x27
#define HPI_V2_REG_OFFSET_OF_PORT_SELECT_SINK_PDO	0x0005
#define HPI_V1_REG_SIZE_OF_PORT_SELECT_SINK_PDO		1
#define HPI_V2_REG_SIZE_OF_PORT_SELECT_SINK_PDO		1
#define HPI_SIGNATURE_SELECT_SINK_PDO			0x534e4b50  /* 'SNKP' */

#define HPI_PDO_LIST_TYPE_MASK
#define HPI_PDO_LIST_TYPE_SOURCE	0x00
#define HPI_PDO_LIST_TYPE_SINK		0x01
struct hpi_pdo_list {
	union {
		/*
		 * Byte 0 of pdo_list_type indicates the type of PDO data.
		 * 0 - Source PDOs list; 1 - Sink PDOs list.
		 * Used in reading Source/Sink PDO list.
		 */
		u32 pdo_list_type;
		/* Used in changing Source/Sink PDO list. */
		u32 signature;
	};
	u32 pdos[7];
} __packed;

#define HPI_V1_REG_OFFSET_OF_PORT_PD_CONTROL	0x28
#define HPI_V2_REG_OFFSET_OF_PORT_PD_CONTROL	0x0006
#define HPI_V1_REG_SIZE_OF_PORT_PD_CONTROL	1
#define HPI_V2_REG_SIZE_OF_PORT_PD_CONTROL	1
enum hpi_pd_ctrl_command {
	HPI_PD_CTRL_CMD_SET_TYPE_C_DEFAULT_PROFILE = 0x00,
	HPI_PD_CTRL_CMD_SET_TYPE_C_1_5_A_PROFILE,
	HPI_PD_CTRL_CMD_SET_TYPE_C_3_A_PROFILE,
	HPI_PD_CTRL_CMD_TRIGGER_DATA_ROLE_SWAP = 0x05,
	HPI_PD_CTRL_CMD_TRIGGER_POWER_ROLE_SWAP,
	HPI_PD_CTRL_CMD_SWITCH_ON_VCONN,
	HPI_PD_CTRL_CMD_SWITCH_OFF_VCONN,
	HPI_PD_CTRL_CMD_TRIGGER_VCONN_ROLE_SWAP,
	HPI_PD_CTRL_CMD_RETRIEVE_SOURCE_CAPABILITY,
	HPI_PD_CTRL_CMD_RETRIEVE_SINK_CAPABILITY,
	HPI_PD_CTRL_CMD_SEND_GOTOMIN_MESSAGE,
	HPI_PD_CTRL_CMD_SEND_HARD_RESET,
	HPI_PD_CTRL_CMD_SEND_SOFT_RESET,
	HPI_PD_CTRL_CMD_SEND_CABLE_RESET,
	HPI_PD_CTRL_CMD_EC_INITIALIZATION_COMPLETE,
	HPI_PD_CTRL_CMD_PORT_DISABLE,
	HPI_PD_CTRL_CMD_SEND_SOFT_RESET_SOP_PRIME,
	HPI_PD_CTRL_CMD_SEND_SOFT_RESET_SOP_DPRIME,
	HPI_PD_CTRL_CMD_CHANGE_PD_PORT_PARAMETERS,	/* HPIv2 only */
	HPI_PD_CTRL_CMD_ABORT_PENDING_PD_COMMAND,	/* HPIv2 only */
	HPI_PD_CTRL_CMD_OVP_OCP_OTP_TRIGGERED,		/* HPIv2 only */
	HPI_PD_CTRL_CMD_READ_SOURCE_PDOS = 0x20,	/* HPIv2 only */
	HPI_PD_CTRL_CMD_READ_SINK_PDOS = 0x21,		/* HPIv2 only */
};

/*
 * This data structure used in Port Configuration Change through PD_CONTROL by
 * the command HPI_PD_CTRL_CMD_CHANGE_PD_PORT_PARAMETERS.
 */
struct hpi_pd_port_change_config {
	/* 0=Sink; 1=Source; 2=Dual Role. */
	u8 desired_port_role;
	/* 0=Sink; 1=Source. */
	u8 default_port_role_in_case_of_dual_role;
	/* Enable DRP toggle in case of Dual Role Port. */
	u8 drp_toggle_enable;
	/* Enable Tyr.SRC in casue of Dual Role Port. */
	u8 try_src_enable;
} __packed;

/*
 * This data structure used in reading current source PDO list through
 * PD_CONTROL by the HPI_PD_CTRL_CMD_READ_SOURCE_PDOS and
 * HPI_PD_CTRL_CMD_READ_SINK_PDOS commands.
 */
struct hpi_pd_src_sink_pdo_list {
	/*
	 * Used in updating Source/Sink PDO list.
	 * Signature "SRCP" for updating Source PDO list.
	 * Signature "SNKP" for udpating Sink PDO list.
	 */
	u32 signature_pdo_data_type;
	u32 pdos[USBPD_VDM_DATA_VDOS];
} __packed;
#define HPI_PDO_LIST_DATA_TYPE_MASK		0x000000ff
#define HPI_PDO_LIST_DATA_TYPE_SOURCE_PDO	0x00
#define HPI_PDO_LIST_DATA_TYPE_SINK_PDO		0x01
#define IS_HPI_SROUCE_PDO_LIST_DATA(_signature_pdo_data_type)		 \
	((((_signature_pdo_data_type) & HPI_PDO_LIST_DATA_TYPE_MASK) == \
	 HPI_PDO_LIST_DATA_TYPE_SOURCE_PDO) ? true : false)
#define IS_HPI_SINK_PDO_LIST_DATA(_signature_pdo_data_type)		 \
	((((_signature_pdo_data_type) & HPI_PDO_LIST_DATA_TYPE_MASK) == \
	 HPI_PDO_LIST_DATA_TYPE_SINK_PDO) ? true : false)


#define HPI_V1_REG_OFFSET_OF_PORT_PD_STATUS	0x2c
#define HPI_V2_REG_OFFSET_OF_PORT_PD_STATUS	0x0008
#define HPI_V1_REG_SIZE_OF_PORT_PD_STATUS	4
#define HPI_V2_REG_SIZE_OF_PORT_PD_STATUS	4
struct hpi_pd_status {
	/*
	 * b1-b0: Default Port Data Role.
	 *   00 - UFP; 01 - DFP; 10 - DRP.
	 */
	u8 default_data_role : 2;
	/*
	 * b2: Default Port Data Role in case of DRP.
	 *   0 - UFP; 1 - DFP.
	 */
	u8 default_data_role_in_drp : 1;
	/*
	 * b4-b3: Default Port Power Role.
	 *   00 - Sink; 01 - Source; 10 - Dual Role.
	 */
	u8 default_power_role : 2;
	/*
	 * b5: Default Port Power Role in case of Dual Role.
	 *   0 - Sink; 1 - Source.
	 */
	u8 default_power_role_in_dual : 1;
	/*
	 * b6: Current Port Data Role.
	 *   0 - UFP; 1 - DFP.
	 *   This bit only valid when the contract_state is set.
	 */
	u8 current_data_role : 1;
	u8 reserved1 : 1;	/* b7 */
	/*
	 * b8: Current Port Power Role.
	 *  0 - Sink; 1 - Source.
	 *  This bit only valid when the contract_state is set.
	 */
	u8 current_power_role : 1;
	u8 reserved2 : 1;	/* b9 */
	/*
	 * b10: Contract State.
	 *  0 - No contract exists with port partner.
	 *  1 - Contract exists with port partner.
	 */
	u8 contract_established : 1;
	/* b11: EMCA Present. 0 - EMCA is not present; 1 - EMCA is present. */
	u8 emca_present : 1;
	/*
	 * b12: VCONN Supplier.
	 *  0 - CCG is not the current supplier of VCONN.
	 *  1 - CCG is the current supplier of VCONN.
	 */
	u8 vconn_supplier : 1;
	/*
	 * b13 - VCONN Status.
	 *  0 - CCG is not providing VCONN supply.
	 *  1 - CCG has VCONN supply enabled.
	 */
	u8 vconn_sourcing : 1;
	u8 reserved3 : 2;	/* b15-b14 */
	u8 reserved4;		/* b23-b16 */
	u8 reserved5;		/* b31-b24 */
} __packed;

enum hpi_pd_port_data_role {
	PD_PORT_DATA_ROLE_UFP	= 0x00,
	PD_PORT_DATA_ROLE_DFP	= 0x01,
	PD_PORT_DATA_ROLE_DRP	= 0x02,
};

enum hpi_pd_port_power_role {
	PD_PORT_POWER_ROLE_SINK	= 0x00,
	PD_PORT_POWER_ROLE_SOURCE	= 0x01,
	PD_PORT_POWER_ROLE_DUAL	= 0x02,
};

#define HPI_V1_REG_OFFSET_OF_PORT_TYPE_C_STATUS	0x30
#define HPI_V2_REG_OFFSET_OF_PORT_TYPE_C_STATUS	0x000c
#define HPI_V1_REG_SIZE_OF_PORT_TYPE_C_STATUS	1
#define HPI_V2_REG_SIZE_OF_PORT_TYPE_C_STATUS	1
struct hpi_type_c_status {
	/*
	 * b0: Port Partner connection status.
	 *   0 - Port not conencted to partner.
	 *   1 - Port connected to partner.
	 */
	u8 type_c_connected : 1;
	/* b1: CC polarity.	0 - CC1; 1 - CC2. */
	u8 cc_polarity : 1;
	/*
	 * b4-b2: Type of device attached.
	 *   000 - Nothing attached
	 *   001 - Sink attached
	 *   010 - Source attached
	 *   011 - Debug Accessory attached
	 *   100 - Audio Accessory attached
	 *   101 - Powered Accessory attached
	 *   110 - Unsupported Accessory attached
	 */
	u8 attached_device_type : 3;
	/*
	 * b5: Ra Status.	0 - If CCG not detects Ra; 1 - CCG detects Ra.
	 *   This bit only valid when CCG is source,
	 */
	u8 ra_detected : 1;
	/*
	 * b7-b6: Type-C current level.
	 *   00 - Default; 01 - 1.5A; 02 - 3A.
	 */
	u8 current_level : 2;
} __packed;

enum hpi_type_c_cc_polarity {
	CC_POLARITY_CC1		= 0x00,
	CC_POLARITY_CC2		= 0x01
};

enum hpi_type_c_attached_device_type {
	ATTACHED_DEV_TYPE_NOTHING		= 0x00,
	ATTACHED_DEV_TYPE_SINK			= 0x01,
	ATTACHED_DEV_TYPE_SOURCE		= 0x02,
	ATTACHED_DEV_TYPE_DEBUG_ACCESSORY	= 0x03,
	ATTACHED_DEV_TYPE_AUDIO_ACCESSORY	= 0x04,
	ATTACHED_DEV_TYPE_POWERED_ACCESSORY	= 0x05,
	ATTACHED_DEV_TYPE_UNSUPPORTED_ACCESSORY	= 0x06,
};

enum hpi_type_c_current_level {
	CURRENT_LEVEL_DEFAULT	= 0x00,
	CURRENT_LEVEL_1_5_A	= 0x01,
	CURRENT_LEVEL_3_A	= 0x02,
};

#define HPI_V1_REG_OFFSET_OF_PORT_CURRENT_PDO	0x34
#define HPI_V2_REG_OFFSET_OF_PORT_CURRENT_PDO	0x0010
#define HPI_V1_REG_SIZE_OF_PORT_CURRENT_PDO	4
#define HPI_V2_REG_SIZE_OF_PORT_CURRENT_PDO	4

#define HPI_V1_REG_OFFSET_OF_PORT_CURRENT_RDO	0x38
#define HPI_V2_REG_OFFSET_OF_PORT_CURRENT_RDO	0x0014
#define HPI_V1_REG_SIZE_OF_PORT_CURRENT_RDO	4
#define HPI_V2_REG_SIZE_OF_PORT_CURRENT_RDO	4

#define HPI_V1_REG_OFFSET_OF_PORT_CURRENT_CABLE_VDO	0x3c
#define HPI_V2_REG_OFFSET_OF_PORT_CURRENT_CABLE_VDO	0x0018
#define HPI_V1_REG_SIZE_OF_PORT_CURRENT_CABLE_VDO	4
#define HPI_V2_REG_SIZE_OF_PORT_CURRENT_CABLE_VDO	4

#define HPI_V1_REG_OFFSET_OF_PORT_EC_DP_HPD_CONTROL	0x44
#define HPI_V2_REG_OFFSET_OF_PORT_EC_DP_HPD_CONTROL	(HPI_REG_INVALID_NA)
#define HPI_V1_REG_SIZE_OF_PORT_EC_DP_HPD_CONTROL	1
#define HPI_V2_REG_SIZE_OF_PORT_EC_DP_HPD_CONTROL	(HPI_REG_INVALID_NA)
struct hpi_hpd_control {
	u8 hpd_control		: 1;
	u8 hpd_initial_state	: 1;
	u8 reserved		: 6;
} __packed;
#define HPI_HPD_CONTROL_DISENABLED	0x00
#define HPI_HPD_CONTROL_ENABLED		0x01
/* Initial HPD State is only valid when HPD control is enabled. */
#define HPI_HPD_INITIAL_STATE_LOW	0x00
#define HPI_HPD_INITIAL_STATE_HIGH	0x01

#define HPI_V1_REG_OFFSET_OF_PORT_EC_DP_MUX_CONTROL	0x45
#define HPI_V2_REG_OFFSET_OF_PORT_EC_DP_MUX_CONTROL	(HPI_REG_INVALID_NA)
#define HPI_V1_REG_SIZE_OF_PORT_EC_DP_MUX_CONTROL	1
#define HPI_V2_REG_SIZE_OF_PORT_EC_DP_MUX_CONTROL	(HPI_REG_INVALID_NA)
enum hpi_dp_mux_config {
	DP_MUX_CONFIG_USB_SAFE_ISOLATE_SUB	= 0x00,
	DP_MUX_CONFIG_USB_SS_ONLY		= 0x01,
	DP_MUX_CONFIG_DP_2_LANE_AND_USB_SS	= 0x02,
	DP_MUX_CONFIG_DP_4_LANE			= 0x03,
};

#define HPI_V1_REG_OFFSET_OF_PORT_TRIGGER_DP_MODE	0x46
#define HPI_V2_REG_OFFSET_OF_PORT_TRIGGER_DP_MODE	(HPI_REG_INVALID_NA)
#define HPI_V1_REG_SIZE_OF_PORT_TRIGGER_DP_MODE		1
#define HPI_V2_REG_SIZE_OF_PORT_TRIGGER_DP_MODE		(HPI_REG_INVALID_NA)
struct hpi_trigger_dp_mode {
	/* 0-Wait for trigger from EC; 1-Initiate DP mode handling. */
	u8 mode : 1;
	u8 reserved : 7;
} __packed;
#define HPI_TRIGGER_DP_WAIT_FROM_EC	0x00
#define HPI_TRIGGER_DP_INITIATE_DP_MODE	0x01

#define HPI_V1_REG_OFFSET_OF_PORT_DP_CONFIGURE_MODE	0x47
#define HPI_V2_REG_OFFSET_OF_PORT_DP_CONFIGURE_MODE	(HPI_REG_INVALID_NA)
#define HPI_V1_REG_SIZE_OF_PORT_DP_CONFIGURE_MODE	1
#define HPI_V2_REG_SIZE_OF_PORT_DP_CONFIGURE_MODE	(HPI_REG_INVALID_NA)
enum hpi_dp_source_config_mode {
	DP_SRC_CONFIG_RFU_PIN_A		= 0x00,
	DP_SRC_CONFIG_RFU_PIN_B		= 0x01,
	DP_SRC_CONFIG_SWITCH_TO_PIN_C	= 0x02,
	DP_SRC_CONFIG_SWITCH_TO_PIN_D	= 0x03,
	DP_SRC_CONFIG_SWITCH_TO_PIN_E	= 0x04,
	DP_SRC_CONFIG_SWITCH_TO_PIN_F	= 0x05,
	DP_SRC_CONFIG_SWITCH_TO_USB	= 0x06,
	DP_SRC_CONFIG_EXIT_DP		= 0x07,
};
struct hpi_dp_sink_config_mode {
	u8 set_status_vdo : 1;
	u8 set_status_vdo_pdo : 1;
	u8 force_send_status_vdo : 1;
	u8 reserved : 5;
} __packed;

#define HPI_V1_REG_OFFSET_OF_PORT_ALT_MODE_CMD	(HPI_REG_INVALID_NA)
#define HPI_V2_REG_OFFSET_OF_PORT_ALT_MODE_CMD	0x001c
#define HPI_V1_REG_SIZE_OF_PORT_ALT_MODE_CMD	(HPI_REG_INVALID_NA)
#define HPI_V2_REG_SIZE_OF_PORT_ALT_MODE_CMD	4
struct hpi_alt_mode_cmd {
	/* CCG Data Role: 0 - UFP; 1 - DFP */
	u8 alt_mode_data_role : 1;
	u8 alt_mode_cmd_id : 7;
	u8 alt_mode_id;	/* 0 - DisplayPort; 1 - Thunderbolt; other - Reserved */
	u16 svid;
} __packed;

enum hpi_alt_mode_cmd_id {	/* Value definition for alt_mode_cmd field */
	ALT_MODE_CMD_RESERVED				= 0x00,
	ALT_MODE_CMD_ENABLE_EC_TRIGGER			= 0x01,
	ALT_MODE_CMD_DISABLE_EC_TRIGGER			= 0x02,
	ALT_MODE_CMD_INITIATE_ALTERNATE_MODE		= 0x03,
	ALT_MODE_CMD_INITIATE_ALTERNATE_MODE_EXIT	= 0x04,
	ALT_MODE_CMD_ALTERNATE_MODE_SPECIFIC		= 0x05,
};

enum hpi_alt_mode_id {
	ALT_MODE_ID_DISOPLAYPORT	= 0x00,
	ALT_MODE_ID_THUNDERBOLT		= 0x01,
};

#define HPI_V1_REG_OFFSET_OF_PORT_APP_HW_CMD	(HPI_REG_INVALID_NA)
#define HPI_V2_REG_OFFSET_OF_PORT_APP_HW_CMD	0x0020
#define HPI_V1_REG_SIZE_OF_PORT_APP_HW_CMD	(HPI_REG_INVALID_NA)
#define HPI_V2_REG_SIZE_OF_PORT_APP_HW_CMD	4
struct hpi_app_hw_cmd {
	/*
	 * The command depending on the @hw_type.
	 * If is MUX, use enum hpi_mux_command defined commands;
	 * if is HPD, use enum hpi_hpd_command defined commands.
	 */
	u16 command;
	u8 hw_type;		/* 1 - MUX; 2  - HPD; Other values resereved. */
	u8 data_role_type;	/* CCG data role: 0=UFP; 1=DFP */
} __packed;
enum hpi_mux_command {
	MUX_CMD_SET_FOR_ISOLATE_MODE		= 0x00,
	MUX_CMD_SET_FOR_USB_3_1			= 0x02,
	MUX_CMD_SET_FOR_DP_2_LANE_AND_USB	= 0x03,
	MUX_CMD_SET_FOR_DP_4_LANE_		= 0x04,
};
enum hpi_hpd_command {
	HPD_CMD_ENABLE_HPD_SIGNAL_OUTPUT	= 0x00,
	HPD_CMD_SIGNAL_HPD_LOW			= 0x01,
	HPD_CMD_SIGNAL_HPD_HIGH			= 0x02,
	HPD_CMD_SIGNAL_HPD_IRQ			= 0x03,
	HPD_CMD_DISABLE_HPD_SIGNAL_OUTPUT	= 0x04,
};

#define HPI_V1_REG_OFFSET_OF_PORT_EVENT_MASK	0x48
#define HPI_V2_REG_OFFSET_OF_PORT_EVENT_MASK	0x0024
#define HPI_V1_REG_SIZE_OF_PORT_EVENT_MASK	4
#define HPI_V2_REG_SIZE_OF_PORT_EVENT_MASK	4
#define HPI_EVENT_MASK_OVER_CURRENT_DETECTED			0x00000002
#define HPI_EVENT_MASK_OVER_VOLTAGE_DETECTED			0x00000004
#define HPI_EVENT_MASK_TYPE_C_PORT_CONNECT_DETECTED		0x00000008
#define HPI_EVENT_MASK_TYPE_C_PORT_DISCONNECT_DETECTED		0x00000010
#define HPI_EVENT_MASK_PD_CONTRACT_NEGOTIATIOIN_COMPLETED	0x00000020
#define HPI_EVENT_MASK_PD_CONTROL_MESSAGE_RECEIVED		0x00000040
#define HPI_EVENT_MASK_VDM_RECEIVED				0x00000080
#define HPI_EVENT_MASK_SOURCE_CAPABILITY_RECEIVED		0x00000100
#define HPI_EVENT_MASK_SINK_CAPABILITY_RECEIVED			0x00000200
#define HPI_EVENT_MASK_DP_ALTERNATE_MODE_EVENTS			0x00000400
#define HPI_EVENT_MASK_ERROR_TIMEOUT_EVENTS			0x00000800
#define HPI_EVENT_MASK_EMCA_EVENTS				0x00001000
#define HPI_EVENT_MASK_MISCELLANEOUS_EVENTS			0x00002000
#define HPI_EVENT_MASK_BILLBOARD_EVENTS				0x00004000
#define HPI_EVENT_MASK_EC_POWER_CONTROL_EVENTS			0x00008000

#define HPI_V1_REG_OFFSET_OF_PORT_SWAP_RESPONSE		0x4c
#define HPI_V2_REG_OFFSET_OF_PORT_SWAP_RESPONSE		0x0028
#define HPI_V1_REG_SIZE_OF_PORT_SWAP_RESPONSE		1
#define HPI_V2_REG_SIZE_OF_PORT_SWAP_RESPONSE		1
struct hpi_swap_status {
	u8 swap_type		: 4;
	u8 swap_resp_code	: 4;
} __packed;

enum hpi_swap_type {
	HPI_DR_SWAP	= 0,
	HPI_PR_SWAP	= 1,
	HPI_VCONN_SWAP	= 2,
};

enum hpi_swap_resp_code {
	SWAP_RESP_ACCEPT		= 0x00,
	SWAP_RESP_REJECT		= 0x01,
	SWAP_RESP_WAIT			= 0x02,
	SWAP_RESP_ILLEGAL_IGNORED	= 0x03,
	SWAP_RESP_HARD_RESET_SENT	= 0x04,
};

struct hpi_swap_response_reg {
	u8 dr_swap : 2;
	u8 pr_swap : 2;
	u8 vconn_swap : 2;
} __packed;

#define HPI_V1_REG_OFFSET_OF_PORT_ACTIVE_EC_MODES	0x4d
#define HPI_V2_REG_OFFSET_OF_PORT_ACTIVE_EC_MODES	0x0029
#define HPI_V1_REG_SIZE_OF_PORT_ACTIVE_EC_MODES		1
#define HPI_V2_REG_SIZE_OF_PORT_ACTIVE_EC_MODES		1
#define IS_EC_HAS_ACTIVE_ALT_MODES(_status)	\
	(((_status) == 0x00) ? true : false)

#define HPI_V1_REG_OFFSET_OF_PORT_VDM_EC_CONTROL	0x4e
#define HPI_V2_REG_OFFSET_OF_PORT_VDM_EC_CONTROL	0x002a
#define HPI_V1_REG_SIZE_OF_PORT_VDM_EC_CONTROL		1
#define HPI_V2_REG_SIZE_OF_PORT_VDM_EC_CONTROL		1
struct hpi_vdm_ec_control {
	u8 ec_ctrl_enabled : 1;
	u8 reserved : 7;	/* Should be zero */
} __packed;

#define HPI_V1_REG_OFFSET_OF_PORT_CMD_TIMEOUT	(HPI_REG_INVALID_NA)
#define HPI_V2_REG_OFFSET_OF_PORT_CMD_TIMEOUT	0x0030
#define HPI_V1_REG_SIZE_OF_PORT_CMD_TIMEOUT	(HPI_REG_INVALID_NA)
#define HPI_V2_REG_SIZE_OF_PORT_CMD_TIMEOUT	1

#define HPI_V1_REG_OFFSET_OF_PORT_PORT_INTR_STATUS	(HPI_REG_INVALID_NA)
#define HPI_V2_REG_OFFSET_OF_PORT_PORT_INTR_STATUS	0x34
#define HPI_V1_REG_SIZE_OF_PORT_PORT_INTR_STATUS	(HPI_REG_INVALID_NA)
#define HPI_V2_REG_SIZE_OF_PORT_PORT_INTR_STATUS	4
/* Bit mask for PORT_INTER_STATUS register */
#define HPI_PORT_INTR_STATUS_MASK_CC_ATTACH		0x00000001
#define HPI_PORT_INTR_STATUS_MASK_CC_DETACH		0x00000002
#define HPI_PORT_INTR_STATUS_MASK_CONTRACT_CMPLT	0x00000004
#define HPI_PORT_INTR_STATUS_MASK_PRSWAP_CMPLT		0x00000008
#define HPI_PORT_INTR_STATUS_MASK_DRSWAP_CMPLT		0x00000010
#define HPI_PORT_INTR_STATUS_MASK_CONNSWAP_CMPLT	0x00000020
#define HPI_PORT_INTR_STATUS_MASK_HARDRESET_RCVD	0x00000040
#define HPI_PORT_INTR_STATUS_MASK_HARDRESET_SENT	0x00000080
#define HPI_PORT_INTR_STATUS_MASK_SOFTRESET_SENT	0x00000100
#define HPI_PORT_INTR_STATUS_MASK_CABLERESET_SENT	0x00000200
#define HPI_PORT_INTR_STATUS_MASK_CC_ERROR_RECVERY	0x00000400
#define HPI_PORT_INTR_STATUS_MASK_SRC_DSIABLED		0x00000800
#define HPI_PORT_INTR_STATUS_MASK_EMCA_DETECT		0x00001000
#define HPI_PORT_INTR_STATUS_MASK_CBLDISC_FAIL		0x00002000
#define HPI_PORT_INTR_STATUS_MASK_ALTMODE_ENTRY		0x00010000
#define HPI_PORT_INTR_STATUS_MASK_ALTMODE_EXIT		0x00020000
#define HPI_PORT_INTR_STATUS_MASK_UNEXP_VOLTAGE		0x20000000
#define HPI_PORT_INTR_STATUS_MASK_OVP_EVT		0x40000000
#define HPI_PORT_INTR_STATUS_MASK_OCP_EVT		0x80000000

#define HPI_V1_REG_OFFSET_OF_PORT_DISABLE_BILLBOARD_RESET	0x50
#define HPI_V2_REG_OFFSET_OF_PORT_DISABLE_BILLBOARD_RESET	0x0050
#define HPI_V1_REG_SIZE_OF_PORT_DISABLE_BILLBOARD_RESET		1
#define HPI_V2_REG_SIZE_OF_PORT_DISABLE_BILLBOARD_RESET		1
#define HPI_SIGNATURE_BILLBOARD_RESET_DISABLED			0xad  /* ~'R' */

#define HPI_V1_REG_OFFSET_OF_PORT_BILLBOARD_ALTMODE_STATUS	0x52
#define HPI_V2_REG_OFFSET_OF_PORT_BILLBOARD_ALTMODE_STATUS	0x0052
#define HPI_V1_REG_SIZE_OF_PORT_BILLBOARD_ALTMODE_STATUS	2
#define HPI_V2_REG_SIZE_OF_PORT_BILLBOARD_ALTMODE_STATUS	2
struct hpi_billboard_altmode_status {
	u8 mode1_status : 2;
	u8 mode2_status : 2;
	u8 mode3_status : 2;
	u8 mode4_status : 2;
	u8 mode5_status : 2;
	u8 mode6_status : 2;
	u8 mode7_status : 2;
	u8 mode8_status : 2;
} __packed;

#define HPI_V1_REG_OFFSET_OF_PORT_BILLBOARD_OPER_MODEL	0x54
#define HPI_V2_REG_OFFSET_OF_PORT_BILLBOARD_OPER_MODEL	0x0054
#define HPI_V1_REG_SIZE_OF_PORT_BILLBOARD_OPER_MODEL	1
#define HPI_V2_REG_SIZE_OF_PORT_BILLBOARD_OPER_MODEL	1
#define HPI_BILLBOARD_OPER_MODEL_EC_PRESENT_MASK	0x10
#define HPI_BILLBOARD_OPER_MODEL_RESET_EVENT_MASK	0x01
struct hpi_billboard_oper_model {
	u8 hpi_based : 1;		/* 0=Reset Based; 1=HPI Based. */
	u8 reserved1 : 3;		/* Should be ignored. */
	u8 ec_present : 1;		/* 0=EC not present; 1=EC present. */
	u8 reserved2: 3;		/* Should be ignored. */
} __packed;
enum hpi_billboard_oper_model_status {
	BILLBOARD_OPER_MODEL_NO_EC_RESET_BASED		= 0x00,
	BILLBOARD_OPER_MODEL_NO_EC_HPI_BASED		= 0x01,
	BILLBOARD_OPER_MODEL_WITH_EC_RESET_BASED	= 0x10,
	BILLBOARD_OPER_MODEL_WITH_EC_HPI_BASED		= 0x11,
};

#define HPI_V1_REG_OFFSET_OF_PORT_EXTERNAL_POWER_CONTROL  0x60
#define HPI_V2_REG_OFFSET_OF_PORT_EXTERNAL_POWER_CONTROL  (HPI_REG_INVALID_NA)
#define HPI_V1_REG_SIZE_OF_PORT_EXTERNAL_POWER_CONTROL    1
#define HPI_V2_REG_SIZE_OF_PORT_EXTERNAL_POWER_CONTROL    1
struct hpi_external_power_control {
	u8 power_ctrl_status;  /* See enum hpi_external_power_control_status. */
} __packed;
enum hpi_external_power_control_status {
	EXTERNAL_POWER_CTRL_STATUS_IDLE				= 0x00,
	EXTERNAL_POWER_CTRL_STATUS_SRC_VBUS_EABLE_REQUEST	= 0x01,
	EXTERNAL_POWER_CTRL_STATUS_SRC_VBUS_ENABLING_CMPLT	= 0x02,
	EXTERNAL_POWER_CTRL_STATUS_SRC_VBUS_ENABLING_ERROR	= 0x03,
	EXTERNAL_POWER_CTRL_STATUS_SRC_VBUS_DISABLING_REQUEST	= 0x04,
	EXTERNAL_POWER_CTRL_STATUS_SRC_VBUS_DISBALING_CMPLT	= 0x05,
	EXTERNAL_POWER_CTRL_STATUS_SRC_VBUS_DISABLING_ERROR	= 0x06,
	EXTERNAL_POWER_CTRL_STATUS_SINK_VBUS_ENABLE_REQUEST	= 0x07,
	EXTERNAL_POWER_CTRL_STATUS_SINK_VBUS_DISABLING_REQUEST	= 0x08,
};

/*
 * Flash Orgination Structure.
 *
 * HPI Version 1 Address Space
 */
#define HPI_V1_REG_OFFSET_OF_DEVICE_BASE	0x00
#define HPI_V1_REG_SIZE_OF_DEVICE_BASE		256
#define HPI_V1_REG_OFFSET_OF_PORT_BASE		0x00
#define HPI_V1_REG_SIZE_OF_PORT_BASE		256

#define HPI_V1_REG_OFFSET_OF_PORT_0_BASE	0x00
#define HPI_V1_REG_SIZE_OF_PORT_0_BASE		256
#define HPI_V1_REG_OFFSET_OF_PORT_1_BASE	(HPI_REG_INVALID_NA)
#define HPI_V1_REG_SIZE_OF_PORT_1_BASE		(HPI_REG_INVALID_NA)
#define HPI_V1_REG_OFFSET_OF_PORT_2_BASE	(HPI_REG_INVALID_NA)
#define HPI_V1_REG_SIZE_OF_PORT_2_BASE		(HPI_REG_INVALID_NA)
#define HPI_V1_REG_OFFSET_OF_PORT_3_BASE	(HPI_REG_INVALID_NA)
#define HPI_V1_REG_SIZE_OF_PORT_3_BASE		(HPI_REG_INVALID_NA)

/* HPI Version 2 Address Space */
#define HPI_V2_REG_OFFSET_OF_DEVICE_BASE	0x0000
#define HPI_V2_REG_SIZE_OF_DEVICE_BASE		4096
#define HPI_V2_REG_OFFSET_OF_PORT_BASE		0x1000
#define HPI_V2_REG_SIZE_OF_PORT_BASE		4096

#define HPI_V2_REG_OFFSET_OF_PORT_0_BASE	0x1000
#define HPI_V2_REG_SIZE_OF_PORT_0_BASE		4096
#define HPI_V2_REG_OFFSET_OF_PORT_1_BASE	0x2000
#define HPI_V2_REG_SIZE_OF_PORT_1_BASE		4096
#define HPI_V2_REG_OFFSET_OF_PORT_2_BASE	(HPI_REG_INVALID_NA)
#define HPI_V2_REG_SIZE_OF_PORT_2_BASE		(HPI_REG_INVALID_NA)
#define HPI_V2_REG_OFFSET_OF_PORT_3_BASE	(HPI_REG_INVALID_NA)
#define HPI_V2_REG_SIZE_OF_PORT_3_BASE		(HPI_REG_INVALID_NA)

/* The real size for flash read/write depending on CCG flash_row_size. */
#define HPI_V1_REG_OFFSET_OF_FLASH_RW		0x80
#define HPI_V1_REG_SIZE_OF_FLASH_RW		128
#define HPI_V2_REG_OFFSET_OF_FLASH_RW		0x0200
#define HPI_V2_REG_SIZE_OF_FLASH_RW		256
#define HPI_MAX_FLASH_RW_REG_SIZE		(HPI_V2_REG_SIZE_OF_FLASH_RW)

/*
 * The read/write data memory offset must be added on the base address of
 * the Device Specific or Port-X Specific registers.
 */
#define HPI_V1_REG_OFFSET_OF_DEVICE_RESPONSE	0x7e
#define HPI_V1_REG_SIZE_OF_DEVICE_RESPONSE	2
#define HPI_V1_REG_OFFSET_OF_DEV_READ_DATA_MEM	0x80
#define HPI_V1_REG_SIZE_OF_DEV_READ_DATA_MEM	64
#define HPI_V1_REG_OFFSET_OF_DEV_WRITE_DATA_MEM	0xc0
#define HPI_V1_REG_SIZE_OF_DEV_WRITE_DATA_MEM	64
#define HPI_V1_REG_OFFSET_OF_PD_RESPONSE	0x7e
#define HPI_V1_REG_SIZE_OF_PD_RESPONSE		2
#define HPI_V1_REG_OFFSET_OF_PD_READ_DATA_MEM	0x80
#define HPI_V1_REG_SIZE_OF_PD_READ_DATA_MEM	64
#define HPI_V1_REG_OFFSET_OF_PD_WRITE_DATA_MEM	0xc0
#define HPI_V1_REG_SIZE_OF_PD_WRITE_DATA_MEM	64

#define HPI_V2_REG_OFFSET_OF_DEVICE_RESPONSE	0x007e
#define HPI_V2_REG_SIZE_OF_DEVICE_RESPONSE	2
#define HPI_V2_REG_OFFSET_OF_DEV_READ_DATA_MEM	(HPI_V2_REG_OFFSET_OF_FLASH_RW)
#define HPI_V2_REG_SIZE_OF_DEV_READ_DATA_MEM	(HPI_V2_REG_SIZE_OF_FLASH_RW)
#define HPI_V2_REG_OFFSET_OF_DEV_WRITE_DATA_MEM	(HPI_V2_REG_OFFSET_OF_FLASH_RW)
#define HPI_V2_REG_SIZE_OF_DEV_WRITE_DATA_MEM	(HPI_V2_REG_SIZE_OF_FLASH_RW)
#define HPI_V2_REG_OFFSET_OF_PD_RESPONSE	0x0400
#define HPI_V2_REG_SIZE_OF_PD_RESPONSE		4
#define HPI_V2_REG_OFFSET_OF_PD_READ_DATA_MEM	0x0404
#define HPI_V2_REG_SIZE_OF_PD_READ_DATA_MEM	508
#define HPI_V2_REG_OFFSET_OF_PD_WRITE_DATA_MEM	0x0800
#define HPI_V2_REG_SIZE_OF_PD_WRITE_DATA_MEM	512

/* Unit of all timeout value is milliseconds. */
#define HPI_V1_TIME_OF_DEFAULT			200
#define HPI_V1_TIME_OF_INIT_COMPLETE		100
#define HPI_V1_TIME_OF_PORT_DISABLE		1000	/* worst case is 650 */
#define HPI_V1_TIME_OF_HANDLER_RESET_CMD	1
#define HPI_V1_TIME_OF_FW_SWITCH_TO_BL		3
#define HPI_V1_TIME_OF_VALIDATE_FLASH		11
#define HPI_V1_TIME_OF_BOOT_WAIT_WINDOWS	32
#define HPI_V1_TIME_OF_BOOT_INTO_FW		150
#define HPI_V1_TIME_OF_JUMP_TO_BOOT		\
	((HPI_V1_TIME_OF_HANDLER_RESET_CMD) + \
	 (HPI_V1_TIME_OF_FW_SWITCH_TO_BL) + \
	 (HPI_V1_TIME_OF_VALIDATE_FLASH))
#define HPI_V1_TIME_OF_DP_RESPONSE		250	/* 25ms + (9 retries) */
#define HPI_V1_TIME_OF_FLASH_ROW_WRITE		200

#define HPI_V2_TIME_OF_DEFAULT			200
#define HPI_V2_TIME_OF_INIT_COMPLETE		100
#define HPI_V2_TIME_OF_PORT_DISABLE		1000
#define HPI_V2_TIME_OF_HANDLER_RESET_CMD	5
#define HPI_V2_TIME_OF_FW_SWITCH_TO_BL		100
#define HPI_V2_TIME_OF_VALIDATE_FLASH		25
#define HPI_V2_TIME_OF_BOOT_WAIT_WINDOWS	50
#define HPI_V2_TIME_OF_BOOT_INTO_FW		250
#define HPI_V2_TIME_OF_JUMP_TO_BOOT		\
	((HPI_V2_TIME_OF_HANDLER_RESET_CMD) + \
	 (HPI_V2_TIME_OF_FW_SWITCH_TO_BL) + \
	 (HPI_V2_TIME_OF_VALIDATE_FLASH))
#define HPI_V2_TIME_OF_DP_RESPONSE		250
#define HPI_V2_TIME_OF_FLASH_ROW_WRITE		250

enum hpi_resp_code {
	/* Responses, 0x00-0x7F */
	HPI_PD_RESP_NO_RESPONSE,
	HPI_PD_RESP_SUCCESS = 0x02,
	HPI_PD_RESP_FLASH_DATA_AVAILABLE,
	HPI_PD_RESP_INVALID_COMMAND = 0x05,
	HPI_PD_RESP_COLLISION_DETECTED,		/* Reserved */
	HPI_PD_RESP_FLASH_UPDATE_FAILED,
	HPI_PD_RESP_INVALID_FW,
	HPI_PD_RESP_INVALID_ARGUMENTS,
	HPI_PD_RESP_NOT_SUPPORTED,
	HPI_PD_RESP_TRANSACTION_FAILED = 0x0C,
	HPI_PD_RESP_PD_COMMAND_FAILED,
	HPI_PD_RESP_UNDEFINED_ERROR,

	HPI_PD_RESP_READ_PDO_DATA = 0x10,	/* HPIv2 only */
	HPI_PD_RESP_CMD_ABORTED,			/* HPIv2 only */
	HPI_PD_RESP_PORT_BUSY,			/* HPIv2 only */

	/* Device Specific Events, 0x80-0x81 */
	HPI_PD_RESP_RESET_COMPLETE = 0x80,
	HPI_PD_RESP_MESSAGE_QUEUE_OVERFLOW,

	/* Type C specific events, 0x82-0x85 */
	HPI_PD_RESP_OVER_CURRENT_DETECTED,
	HPI_PD_RESP_OVER_VOLTAGE_DETECTED,
	HPI_PD_RESP_TYPE_C_CONNECTED,
	HPI_PD_RESP_TYPE_C_DISCONNECTED,

	/* PD Specific events and asynchronous messages, 0x86-0x8F */
	HPI_PD_RESP_PD_CONTRACT_ESTABLISHED = 0x86,
	HPI_PD_RESP_SWAP_COMPLETE,
	/* Follow three code replaced by the HPI_PD_RESP_SWAP_COMPLETE. */
	HPI_PD_RESP_DR_SWAP = HPI_PD_RESP_SWAP_COMPLETE,
	HPI_PD_RESP_PR_SWAP,
	HPI_PD_RESP_VCON_SWAP,

	HPI_PD_RESP_PS_RDY = 0x8A,
	HPI_PD_RESP_GOTOMIN,
	HPI_PD_RESP_ACCEPT_MESSAGE,
	HPI_PD_RESP_REJECT_MESSAGE,
	HPI_PD_RESP_WAIT_MESSAGE,
	HPI_PD_RESP_HARD_RESET,

	/* PD Data Message Specific Events, 0x90 */
	HPI_PD_RESP_VDM_RECEIVED = 0x90,

	/* Capability Message Specific Events, 0x91-0x92*/
	HPI_PD_RESP_SRC_CAP_RCVD,
	HPI_PD_RESP_SINK_CAP_RCVD,

	/* DP and Alternate mode Specific Events, 0x93-0x99*/
	HPI_PD_RESP_DP_ALTERNATE_MODE_ENTER = 0x93,
	HPI_PD_RESP_DP_STATUS_UPDATE,
	/* Following two items replaced by HPI_PD_RESP_DP_STATUS_UPDATE. */
	HPI_PD_RESP_DP_DEVICE_CONNECTED =	/* Connected at UFP_U */
		HPI_PD_RESP_DP_STATUS_UPDATE,
	HPI_PD_RESP_DP_DEVICE_NOT_CONNECTED,	/*Not connected at UFP_U */

	HPI_PD_RESP_DP_SID_NOT_FOUND = 0x96,
	HPI_PD_RESP_MULTIPLE_SVID_DISCOVERED,
	HPI_PD_RESP_DP_FUNC_NOT_SUPPORTED_BY_CABLE, /* Not supported by Cable */
	HPI_PD_RESP_DP_PORT_CONFIG_NOT_SUPPORTED,  /* Not supported by UFP */

	/* Resets and Error Scenario Events, 0x9A-0xA5*/
	HPI_PD_HARD_RESET_SENT = 0x9A,	/* to Port Partner. */
	HPI_PD_SOFT_RESET_SENT,		/* to Port Partner. */
	HPI_PD_CABLE_RESET_SENT,	/* to EMCA. */
	HPI_PD_SOURCE_DISBALED_STATE_ENTERED,
	HPI_PD_SENDER_RESPONSE_TIMER_TIMEOUT,
	HPI_PD_NO_VDM_RESPONSE_RECEIVED,
	HPI_PD_UNEXPECTED_VOLTAGE_VBUS,
	HPI_PD_TYPE_C_ERROR_RECOVERY,

	/* EMCA Related Events, 0xA6-0xA7 */
	HPI_PD_EMCA_DETECTED = 0xA6,
	HPI_PD_CABLE_DISCOVERY_FAILED,	/* HPIv2 only */

	/* Miscellaneous Events */
	HPI_PD_RP_CHANGE_DETECTED = 0xAA,
	HPI_PD_EC_VBUS_CONTROL = 0xAD,	/* HPIv1 only */

	/* Billboard Related Events */
	HPI_PD_BILLBOARD_CONNECT = 0xAB,
	HPI_PD_SEND_VENDOR_DATA,

	/* Alternate Mode Related Events, HPIv2 only */
	HPI_PD_ALTERNATE_MODE_EVENT = 0xB0,
	HPI_PD_ALTERNATE_MODE_HARDWARE_RESET,
};
#define HPI_PD_RESP_EVENT_MASK	0x80
#define IS_HPI_PD_EVENT_CODE(_code)	\
	(((_code) & (HPI_PD_RESP_EVENT_MASK)) == (HPI_PD_RESP_EVENT_MASK))
#define IS_HPI_PD_RESP_CODE(_code)	(!IS_HPI_PD_EVENT_CODE(_code))

/* Event data structure of HPI_PD_ALTERNATE_MODE_EVENT event. */
struct hpi_alternate_mode_event {
	u8 ccg_data_role : 1;	/* 0=UFP; 1=DFP */
	u8 event_type : 7;
	u8 mode_id;	/* 0=DisplayPort; 1=Thunderbolt */
	u16 mode_svid;	/* 0xff01=DisplayPort; 0x080807=Thunderbolt */
	u8 event_data[3]; /* Event specific data determined by alt_event_code */
	/*
	 * Mode specific event type.
	 * 0x01=DisplayPort pin configuration event. evet_data contians a bitmap
	 *	shows which pin configurations are supported by CCG and
	 *	port partner.
	 * 0x02 DisplayPort Status Update event. event_data continas the lower
	 *	24 bits of the status update VDO.
	 * No specific events are defined for Thunderbolt.
	 */
	u8 alt_event_code;
} __packed;
enum hpi_alternate_mode_event_type {
	ALT_EVENT_TYPE_UFP_NOT_SUPPORT_ANY_ALT		= 0x01,
	ALT_EVENT_TYPE_ALT_MODE_ENTERED			= 0x02,
	ALT_EVENT_TYPE_ALT_MODE_EXITED			= 0x03,
	ALT_EVENT_TYPE_MODE_DISCOVERY_COMPLETED		= 0x04,
	ALT_EVENT_TYPE_CCG_NOT_SUPPORT_UFP_SVID		= 0x05,
	ALT_EVENT_TYPE_CCG_SUPPORT_UFP_SVID		= 0x06,
	ALT_EVENT_TYPE_CCG_SUPPORT_UFP_ALT_MODE		= 0x07,
	ALT_EVENT_TYPE_UFP_RESPOND_VDM_FAILED		= 0x08,
	ALT_EVENT_TYPE_CABLE_RESPOND_VDM_FAILED		= 0x09,
	ALT_EVENT_TYPE_CABLE_NOT_SUPPORT_ALT_MODE	= 0x0a,
	ALT_EVENT_TYPE_MISMATCH_CAPABILITIES		= 0x0b,
	ALT_EVENT_TYPE_ALT_MODE_SPECIFIC		= 0x0c,
};

/* Event data format of HPI_PD_ALTERNATE_MODE_HARDWARE_RESET event. */
struct hpi_alternate_mode_hw_event {
	u16 event_type;
	u8 hw_type;	/* 1=Data Mux; 2=HPD signal */
	u8 ccg_data_role;	/* 0=UFP; 1=DFP */
} __packed;
enum hpi_alternate_mode_hw_event_type {
	HPD_UNPLUG_LOW_DETECTED		= 1,	/* DP Sink only */
	HPD_PLUG_HIGH_DETECTED		= 2,	/* DP Sink only */
	HPD_IRQ_DETECTED		= 3,	/* DP Sink only */
	HPD_STATUS_UPDATE_COMPLETED	= 4,	/* DP Source only */
};

struct ccg_bootloader_type {
	/* Set if Boot uses SHA-2 HASH sum for validation. */
	u32 use_sha_2_hash_sum		: 1;
	/* Set if Boot loader has FW update interface. */
	u32 support_fw_update		: 1;
	/* Set if Boot loader supports APP PRIORITY feature. */
	u32 support_app_priority	: 1;
	/* Reserved and set to 0. */
	u32 reserved			: 29;
};

struct hpi_port_status {
	struct hpi_device *port;
	struct hpi_pd_status pd_status;
	struct hpi_type_c_status type_c_status;
};

struct hpi_config_table_header {
	__le16 signature;	/* Signature: 'CY', 0x4359. */
	/*
	 * Indicates the type of table based on solution.
	 *	0 = Reserved
	 *	1 = EMCA
	 *	2 = DFP
	 *	3 = UFP
	 *	4 = DRP
	 *	Rest all values are reserved.
	 */
	u8 table_type;
	/*
	 * Indicates various defined variations in the table type.
	 *	0 = Notebook
	 *	1 = Tablet
	 *	2 = Passive cable
	 *	3 = Active Cable
	 *	4 = Monitor
	 *	5 = Power Adapter
	 *	6 = Type-C Cable Adapter
	 *	Rest all values are reserved.
	 */
	u8 table_subtype;
	__le16 table_ver;	/* See struct hpi_config_table_version. */
	__le16 table_size;
	/*
	 * The checksum is 2's complement of 1-byte sum of values from byte
	 * index 10 to byte index [table_size - 1].
	 * Fisrt 10 bytes of config table are not used in calculating checksum.
	 */
	u8 table_checksum;
	/*
	 * Factor used to ensure that binary sum of all bytes in the config
	 * table is zero.
	 * This field is only valid in CCG3/CCG4 config table,
	 * should reserved as 0 when in CCG1/CCG2 config table.
	 */
	u8 flash_checksum;
} __packed;
#define HPI_CONFIG_TABLE_SIGNATURE	0x4359	/* 'CY' */
#define HPI_V1_CONFIG_TABLE_VERSION	0x0100	/* Minimal version: 0.1.0 */
#define HPI_V2_CONFIG_TABLE_VERSION	0x1000	/* Minimal version: 1.0.0 */
#define HPI_V1_CONFIG_TABLE_SIZE	0x0200
#define HPI_V2_CONFIG_TABLE_SIZE	0x0400
#define HPI_CONFIG_TABLE_CHECKSUM_CALCULATE_START_OFFSET	10

struct hpi_config_table_version {
	u8 patch_num;
	u8 minor	: 4;	/* Minor Version. */
	u8 major	: 4;	/* Major Version. */
} __packed;

enum hpi_config_table_type {
	CONFIG_TABLE_TYPE_RESERVED	= 0,
	CONFIG_TABLE_TYPE_EMCA		= 1,
	CONFIG_TABLE_TYPE_DFP		= 2,
	CONFIG_TABLE_TYPE_UFP		= 3,
	CONFIG_TABLE_TYPE_DRP		= 4,
};

enum hpi_config_table_subtype {
	CONFIG_TABLE_SUBTYPE_NOTEBOOKK			= 0,
	CONFIG_TABLE_SUBTYPE_TABLET			= 1,
	CONFIG_TABLE_SUBTYPE_PASSIVE_CABLE		= 2,
	CONFIG_TABLE_SUBTYPE_ACTIVE_CABLE		= 3,
	CONFIG_TABLE_SUBTYPE_MONITOR			= 4,
	CONFIG_TABLE_SUBTYPE_POWER_ADAPTER		= 5,
	CONFIG_TABLE_SUBTYPE_TYPE_C_CABLE_ADAPTER	= 6,
};

/*
 * VDM relative definitions and data structures.
 */
#define VDM_VDO_OBJ_SIZE	(USBPD_DATA_OBJ_SIZE)
#define VDM_HEADER_SIZE		(VDM_VDO_OBJ_SIZE)
#define VDM_MAX_VDO_NUM		(USBPD_VDM_DATA_VDOS)
#define VDM_MAX_MSG_SIZE	((USBPD_MAX_DATA_OBJS) * (VDM_VDO_OBJ_SIZE))
#define VDM_MAX_DATA_SIZE	((VDM_MAX_VDO_NUM) * (VDM_VDO_OBJ_SIZE))
#define HPI_VDM_MSG_HEADER_SIZE	(USBPD_MSG_HEADER_SIZE)
#define HPI_VDM_SOP_TYPE_SIZE	1
#define HPI_VDM_HEADER_SIZE	\
	((HPI_VDM_MSG_HEADER_SIZE) + (HPI_VDM_SOP_TYPE_SIZE) + 1)
#define HPI_MAX_VDM_MSG_SIZE	((HPI_VDM_HEADER_SIZE) + (VDM_MAX_MSG_SIZE))
#define HPI_VDM_RESP_SIZE_BY_VDO_NUM(vdo_num)	\
	((HPI_VDM_HEADER_SIZE) + (vdo_num) * (VDM_VDO_OBJ_SIZE))

enum vdm_cmd_type {
	VDM_CMD_TYPE_INITIATOR	= 0x00,
	/* The command request was received and handled normally */
	VDM_CMD_TYPE_RESP_ACK	= 0x01,
	/* The command request was invalid or not recognized or run failed */
	VDM_CMD_TYPE_RESP_NACK	= 0x02,
	/* If received, shall wait tVDMBusy=50ms, then retrying the command */
	VDM_CMD_TYPE_RESP_BUSY	= 0x03,
};

enum vdm_sid_command {
	VDM_SID_CMD_RESERVED		= 0x00,
	VDM_SID_CMD_DISCOVER_IDENTITY	= 0x01,
	VDM_SID_CMD_DISCOVER_SVID	= 0x02,
	VDM_SID_CMD_DISCOVER_MODES	= 0x03,
	VDM_SID_CMD_ENTER_MODE		= 0x04,
	VDM_SID_CMD_EXIT_MODE		= 0x05,
	VDM_SID_CMD_ATTENTION		= 0x06,
};

enum vdm_cy_command {
	VDM_CY_CMD_RESERVED		= 0x00,
	VDM_CY_CMD_GET_DEVICE_MODE	= 0x01,
	VDM_CY_CMD_GET_DEVICE_VERSION	= 0x02,
	VDM_CY_CMD_GET_SILICON_ID	= 0x03,
	VDM_CY_CMD_DEVICE_RESET		= 0x04,
	VDM_CY_CMD_JUMP_TO_BOOT		= 0x05,
	VDM_CY_CMD_ENTER_FLASHING_MODE	= 0x06,
	VDM_CY_CMD_SEND_DATA		= 0x07,
	VDM_CY_CMD_FLASH_WRITE		= 0x08,
	VDM_CY_CMD_READ_DATA		= 0x09,
	VDM_CY_CMD_FLASH_READ		= 0x0a,
	VDM_CY_CMD_VALIDATE_FW		= 0x0b,
	VDM_CY_CMD_REASON_FOR_BOOT_MODE	= 0x0c,
	VDM_CY_CMD_GET_CHECKSUM		= 0x0d,
	VDM_CY_CMD_GET_FW_START_ADDR	= 0x0e,
	VDM_CY_CMD_SET_APP_PRIORITY	= 0x0f,
	VDM_CY_CMD_SEND_SIGNATURE	= 0x11,
	VDM_CY_CMD_GET_BOOT_TYPE	= 0x13,
	VDM_CY_CMD_GET_CUSTOMER_INFO	= 0x14,
};
#define VMD_CY_CMD_SIGNATURE_GET_SILICON_ID	0x53	/* S */
#define HPI_FW_IMAGE_UUID_SIZE	8

enum vdm_cy_command_response_code {
	VDM_CY_CMD_RESP_NO_RESP			= 0x00,
	VDM_CY_CMD_RESP_SUCCESS			= 0x02,
	VDM_CY_CMD_RESP_FALSH_DATA_AVAILABLE	= 0x03,
	VDM_CY_CMD_RESP_INVALID_COMMAND		= 0x05,
	VDM_CY_CMD_RESP_FLASH_WRITE_FAILED	= 0x07,
	VDM_CY_CMD_RESP_INVALID_FW		= 0x08,
	VDM_CY_CMD_RESP_INVALID_ARGUMENT	= 0x09,
	VDM_CY_CMD_RESP_NOT_SUPPORTED		= 0x0a,
	VDM_CY_CMD_RESP_INVALID_KEY		= 0x0b,
	VDM_CY_CMD_RESP_GENERIC_ERROR		= 0x0e,
};

union u_vdm_header {
	u32 vdo;
	struct {
		/*
		 * b14-b0: Available for Vendor Use, all defined by Cypress.
		 */
		/* b4-b0: Cypress defined commands */
		u32 ccg_cmd : 5;
		/* b7-b5: Sequence number of data packets */
		u32 seq_num : 3;
		u32 reserved1 : 3;	/* b10-b8: Shall be ignored */
		/* b12-b11: Command Type. The value see enum vdm_cmd_type. */
		u32 cmd_type : 2;
		/*
		 * b14-b13: Unstructured VDM command version,
		 * Only 00b-Version 1.0
		 */
		u32 cmd_ver : 2;

		/* b15: VDM Type, 0 = Unstructured VDM */
		u32 vdm_type : 1;
		u32 svid : 16;	/* b31-b16: Vednor ID, CYPRESS VID: 0x04b4 */
	};
} __packed;
#define U_VDM_HEADER_GET_SVID(vdm_header_vdo)	\
	((u16)(((vdm_header_vdo) >> 16) & 0x0000ffff))
#define U_VDM_HEADER_GET_VDM_TYPE(vdm_header_vdo)	\
	(((vdm_header_vdo) & 0x00008000) ? VDM_TYPE_STRUCTURED :	\
					   VDM_TYPE_UNSTRUCTURED)

union s_vdm_header {
	u32 vdo;
	struct {
		/*
		 * b4-b0: Standard commands or Vendor commands
		 * based on the @svid
		 */
		u32 sid_cmd : 5;	/* b4-b0 */
		u32 reserved1 : 1;	/* b5 */
		u32 vdm_cmd_type : 2;	/* b7-b6: VDM Command Type */
		u32 obj_position : 3;	/* b10-b8: Object Position */
		u32 reserved2 : 2;	/* b12-b11 */
		/*
		 * b14-b13: Structured VDM Version.
		 *	00b = Version 1.0
		 *	01b = Version 2.0
		 */
		u32 s_vdm_ver : 2;	/* b14-b13 */
		u32 vdm_type : 1;	/* b15: VDM Type, 1 = Structured VDM */
		u32 svid : 16;		/* b31-b16: Standard or Vendor ID */
	};
} __packed;
enum vdm_structured_vdm_version {
	STRUCTURED_VMD_VERSION_1	= 0,
	STRUCTURED_VMD_VERSION_2	= 1,
};

struct s_vdm_data {
	union s_vdm_header vdm_header;
	u32 vdo[VDM_MAX_VDO_NUM];
} __packed;

struct u_vdm_data {
	union u_vdm_header vdm_header;
	u32 vdo[VDM_MAX_VDO_NUM];
} __packed;

struct vdm_packet {
	union vdm_msg_header msg_header;
	union {
		struct u_vdm_data u_vdm_data;
		struct s_vdm_data s_vdm_data;
	};
} __packed;
#define VDM_PACKET_TO_VDM_TYPE(_vdm_packet_p)			\
	GET_VMD_PACKET_TYPE(					\
		((u32)*(((u8 *)(_vdm_packet_p)) +		\
			  sizeof(union vdm_msg_header)))	\
	)

union vdm_extended_msg_header {
	u16 extended_msg_header;
	struct {
		u16 data_size : 9;
		u16 resereved : 1;
		u16 request_chunk : 1;
		u16 chunk_num : 4;
		u16 chunked : 1;
	};
} __packed;
#define VDM_EXTENDED_PACKET_MAX_DATA_LEN	260
#define VDM_EXTENDED_PACKET_MAX_CHUNK_LEN	26
#define VDM_EXTENDED_PACKET_MAX_LEGACY_LEN	26

struct vdm_extended_packet {
	union vdm_msg_header msg_header;
	union vdm_extended_msg_header extended_msg_header;
	u8 data[0];	/* 0..260 bytes */
} __packed;

/*
 * The HPI VDM event data length = 4 + 4 * Number of VDOs.
 * The data format of the received VDM message is:
 *	Byte1..Byte0 - Message Header.
 *	Byte2 - SOP Type. 0 = SOP; 1 = SOP'; 2 = SOP''.
 *	Byte3 - Resereved.
 *	Byte7..Byte4 - VDM header VDO.
 *	Byte31..Byte8 - Followed 0 ~ 6 specific data VDOs.
 */
struct hpi_vdm_message {
	union vdm_msg_header msg_header;
	u8 sop_type;
	u8 resereved;
	union {
		union s_vdm_header s_vdm_header;
		union u_vdm_header u_vdm_header;
	};
	u32 vdo[VDM_MAX_VDO_NUM];
} __packed;
#define HPI_VDM_MSG_TO_VDM_TYPE(_hpi_vdm_msg_p)				\
	GET_VMD_PACKET_TYPE(						\
		((u32)*((u8 *)(_hpi_vdm_msg_p) +			\
			sizeof(union vdm_msg_header) + sizeof(u16)))	\
	)

union vdm_id_header_vdo {
	u32 vdo;
	struct {
		u32 svid			: 16;
		u32 reserved			: 7;
		u32 dfp_product_type		: 3;
		u32 modal_op_support		: 1;
		u32 ufp_cable_product_type	: 3;
		u32 capable_as_device		: 1;
		u32 capable_as_host		: 1;
	};
} __packed;

enum vdm_product_type {
	UNDEFINED		= 0,
	PDUSB_HUB		= 1,			/* DFP/UFP */
	PDUSB_PERIPHERAL	= 2,			/* UFP */
	PDUSB_HOST		= PDUSB_PERIPHERAL,	/* DFP */
	PASSIVE_CABLE		= 3,			/* Cable Plug */
	POWER_BRICK		= PASSIVE_CABLE,	/* DFP */
	ACTIVE_CABLE		= 4,			/* Cable Plug */
	AMA			= 5,	/* DFP/UFP: Alternate Mode Adapter */
};

union vdm_cert_stat_vdo {
	u32 vdo;
	u32 xid;
} __packed;

union vdm_product_vdo {
	u32 vdo;
	struct {
		u32 bcd_device     : 16;
		u32 usb_product_id : 16;
	};
} __packed;

union vdm_passive_cable_vdo {
	u32 vdo;
	struct {
		/*
		 * b2..b0 - USB SuperSpeed Signaling Support.
		 *	000b = USB 2.0 only, no SuperSpeed Support.
		 *	001b = [USB 3.1] Gen1.
		 *	010b = [USB 3.1] Gen1 and Gen2.
		 *	011b..111b = Reserved, shall not used.
		 */
		u32 ss_signal_type		: 3;
		u32 reserved			: 2;
		/*
		 * b6..b5 - VBUS current handling capability.
		 *	00b = Reserved, shall not used.
		 *	01b = 3A
		 *	10b = 5A
		 *	11b = Reserved, shall not used.
		 */
		u32 vbus_current		: 2;
		u32 reserved1			: 2;
		/*
		 * b10..b9 - Maximum cable VBUS Voltage.
		 *	00b = 20V
		 *	01b = 30V
		 *	10b = 40V
		 *	11b = 50V
		 */
		u32 maximun_vbus_voltage	: 2;
		/*
		 * b12..b11 - Cable Ternination Type.
		 *	00b = VCONN not required.
		 *	01b = VCONN required.
		 *	10b..11b = Reserved, shall not used.
		 */
		u32 termination_type		: 2;
		/*
		 * b16..b13 - Cable latency.
		 *	0000b ¨C Reserved, shall not be used
		 *	0001b ¨C <10ns (~1m)
		 *	0010b ¨C 10ns to 20ns (~2m)
		 *	0011b ¨C 20ns to 30ns (~3m)
		 *	0100b ¨C 30ns to 40ns (~4m)
		 *	0101b ¨C 40ns to 50ns (~5m)
		 *	0110b ¨C 50ns to 60ns (~6m)
		 *	0111b ¨C 60ns to 70ns (~7m)
		 *	1000b ¨C > 70ns (>~7m)
		 *	1001b ¡­.1111b Reserved, shall not be used
		 *	Includes latency of electronics in Active Cable
		 */
		u32 latency			: 4;
		u32 reserved2			: 1;
		/*
		 * b19..b18 - USB Type-C plug to USB Type-C/Captive.
		 *	00b = Reserved, shall not be used.
		 *	01b = Reserved, shall not be used.
		 *	10b = USB Type-C.
		 *	11b = Captive.
		 */
		u32 plug_to			: 2;
		u32 reserved3			: 1;
		/* b23..21 - Version Number of the VDO, v1.0 = 000b. */
		u32 vdo_version			: 3;
		/* b27..b24 - Firmware Version. */
		u32 fw_version			: 4;
		/* b31..b28 - HW Version. */
		u32 hw_version			: 4;
	};
} __packed;

union vdm_avtive_cable_vdo {
	u32 vdo;
	struct {
		/*
		 * b2..b0 - USB SuperSpeed Signaling Support.
		 *	000b = USB 2.0 only, no SuperSpeed Support.
		 *	001b = [USB 3.1] Gen1.
		 *	010b = [USB 3.1] Gen1 and Gen2.
		 *	011b..111b = Reserved, shall not used.
		 */
		u32 ss_signal_type		: 3;
		/*
		 * b3 - SOP'' Controller Present.
		 *	0 = NO SOP'' controller present.
		 *	1 = SOP'' controller present.
		 */
		u32 sop_dprim_present		: 1;
		/* b4 - VBUS through Cable. 0 = No; 1 = Yes. */
		u32 vbus_through_cable		: 1;
		/*
		 * b6..b5 - VBUS current handling capability. Valid only when
		 *   the VBUS through Cable is Yes; When No, should not be used.
		 *	00b = Reserved, shall not used.
		 *	01b = 3A
		 *	10b = 5A
		 *	11b = Reserved, shall not used.
		 */
		u32 vbus_current		: 2;
		u32 reserved1			: 2;
		/*
		 * b10..b9 - Maximum cable VBUS Voltage.
		 *	00b = 20V
		 *	01b = 30V
		 *	10b = 40V
		 *	11b = 50V
		 */
		u32 maximun_vbus_voltage	: 2;
		/*
		 * b12..b11 - Cable Ternination Type.
		 *	00b..01b = Reserved, shall not used.
		 *	10b = One end Active, one end passive, VCONN required.
		 *	11b = Both ends Active, VCONN required.
		 */
		u32 termination_type		: 2;
		/*
		 * b16..b13 - Cable latency.
		 *	0000b ¨C Reserved, shall not be used
		 *	0001b ¨C <10ns (~1m)
		 *	0010b ¨C 10ns to 20ns (~2m)
		 *	0011b ¨C 20ns to 30ns (~3m)
		 *	0100b ¨C 30ns to 40ns (~4m)
		 *	0101b ¨C 40ns to 50ns (~5m)
		 *	0110b ¨C 50ns to 60ns (~6m)
		 *	0111b ¨C 60ns to 70ns (~7m)
		 *	1000b ¨C > 70ns (>~7m)
		 *	1001b ¡­.1111b Reserved, shall not be used
		 *	Includes latency of electronics in Active Cable
		 */
		u32 latency			: 4;
		u32 reserved2			: 1;
		/*
		 * b19..b18 - USB Type-C plug to USB Type-C/Captive.
		 *	00b = Reserved, shall not be used.
		 *	01b = Reserved, shall not be used.
		 *	10b = USB Type-C.
		 *	11b = Captive.
		 */
		u32 plug_to			: 2;
		u32 reserved3			: 1;
		/* b23..21 - Version Number of the VDO, v1.0 = 000b. */
		u32 vdo_version			: 3;
		/* b27..b24 - Firmware Version. */
		u32 fw_version			: 4;
		/* b31..b28 - HW Version. */
		u32 hw_version			: 4;
	};
} __packed;

union vdm_ama_vdo {
	u32 vdo;
	struct {
		/*
		 * b2..b0 - USB SuperSpeed Signaling Support.
		 *	000b = USB 2.0 only, no SuperSpeed Support.
		 *	001b = [USB 3.1] Gen1.
		 *	010b = [USB 3.1] Gen1 and Gen2.
		 *	011b..111b = Reserved, shall not used.
		 */
		u32 ss_signal_type		: 3;
		/* b3 - VBUS required. 0 = No; 1 = Yes. */
		u32 vbus_required		: 1;
		/* b4 - VCONN required. 0 = No; 1 = Yes. */
		u32 vconn_required		: 1;
		/*
		 * b5..b5 - When VCONN required field is Yes, then this filed is
		 *  valid; If is No, then this filed should not be used.
		 *	000b = 1W
		 *	001b = 1.5W
		 *	010b = 2W
		 *	011b = 3W
		 *	100b = 4W
		 *	101b = 5W
		 *	110b = 6W
		 *	111b = Reserved, shall not be used
		 */
		u32 vconn_power			: 3;
		u32 reserved			: 13;
		/* b23..21 - Version Number of the VDO, v1.0 = 000b. */
		u32 vdo_version			: 3;
		/* b27..b24 - Firmware Version. */
		u32 fw_version			: 4;
		/* b31..b28 - HW Version. */
		u32 hw_version			: 4;
	};
} __packed;

struct vdm_resp_discover_id {
	union s_vdm_header vdm_header;
	union vdm_id_header_vdo id_header;
	union vdm_cert_stat_vdo cert_stat_vdo;
	union vdm_product_vdo product_vdo;
	u32 product_type_vdo[0];	/* 0..3 Product Type VDO(s). */
};

union vdm_svid_vdo {
	u32 vdo;
	struct {
		u32 svid1	: 16;  /* The second SVID in the VDO. */
		u32 svid0	: 16;  /* The fisrt SVID in the VDO. */
	};
};

struct vdm_resp_discover_svid {
	union s_vdm_header header;
	union vdm_svid_vdo svid_vdo[0];	/* 1..6 SVID VDO(s)*/
};

#define VDM_CY_FLASHING_MODE	0x0001
union vdm_mode_vdo {
	u32 vdo;
	u32 mode;
	struct {
		u32 cy_mode	: 16;  /* Cypress flashing mode ID, 0x0001 */
		u32 resereved	: 16;
	};
};

struct vdm_resp_discover_mode {
	union s_vdm_header vdm_header;
	union vdm_mode_vdo modes[0];	/* 0..6 Mode VDO(s)*/
};

/*
 * Source/Sink Capabilities message data format.
 * Event code = 0x91/0x92; Event length = 4 + 4 * Number of PDOs.
 */
struct hpi_capabilities_message {
	union vdm_msg_header msg_header;
	u8 sop_type;	/* Will be 0 ,see enum vdm_sop_type. */
	u8 reserved;
	u32 pdo[0];
} __packed;

union power_data_object {
	u32 pdo;
	struct source_fixed_supply {
		u32 maximum_current_in_10mA_units : 10;
		u32 voltage_in_50mV_units : 10;
		/*
		 * 00b = Peak current equals IOC (default)
		 * 01n =  Overload Capabilities:
		 *	1. Peak current equals 150% IOC for 1ms @ 5% duty
		 *	   cycle (low current equals 97% IOC for 19ms)
		 *	2. Peak current equals 125% IOC for 2ms @ 10% duty
		 *	   cycle (low current equals 97% IOC for 18ms)
		 *	3. Peak current equals 110% IOC for 10ms @ 50% duty
		 *	   cycle (low current equals 90% IOC for 10ms)
		 * 10b =  Overload Capabilities:
		 *	1. Peak current equals 200% IOC for 1ms @ 5% duty
		 *	   cycle (low current equals 95% IOC for 19ms)
		 *	2. Peak current equals 150% IOC for 2ms @ 10% duty
		 *	   cycle (low current equals 94% IOC for 18ms)
		 *	3. Peak current equals 125% IOC for 10ms @ 50% duty
		 *	   cycle (low current equals 75% IOC for 10ms)
		 * 11b =  Overload Capabilities:
		 *	1. Peak current equals 200% IOC for 1ms @ 5% duty
		 *	   cycle (low current equals 95% IOC for 19ms)
		 *	2. Peak current equals 175% IOC for 2ms @ 10% duty
		 *	   cycle (low current equals 92% IOC for 18ms)
		 *	3. Peak current equals 150% IOC for 10ms @ 50% duty
		 *	   cycle (low current equals 50% IOC for 10ms)
		 */
		u32 peak_current : 2;
		u32 reserved : 2;
		u32 unchunked_extended_messages_supported : 1;
		u32 dual_role_data : 1;
		u32 usb_communications_capable : 1;
		u32 externally_powered : 1;
		u32 usb_suspend_supported : 1;
		u32 dual_role_power : 1;
		/*
		 * 00b = Fixed supply (Vmin = Vmax)
		 * 01b = Battery
		 * 10b = Variable Supply (non-Battery)
		 * 11b = Reserved
		 */
		u32 power_supply_type : 2;
	} source_fixed_supply;

	struct source_battery {
		u32 maximum_allowable_power_in_250mW_units : 10;
		u32 minimum_voltage_in_50mV_units : 10;
		u32 maximum_voltage_in_50mV_units : 10;
		u32 power_supply_type : 2;
	} source_battery;

	struct source_variable_supply {
		u32 maximum_current_in_10mA_units : 10;
		u32 minimum_voltage_in_50mV_units : 10;
		u32 maximum_voltage_in_50mV_units : 10;
		u32 power_supply_type : 2;
	} source_variable_supply;

	struct sink_fixed_supply {
		u32 operational_current_in_10mA_units : 10;
		u32 voltage_in_50mV_units : 10;
		u32 reserved : 3;
		/*
		 * Fast Role Swap required USB Type-C Current.
		 *	00b = Fast Swap not supported (default)
		 *	01b = Default USB Power
		 *	10b = 1.5A @ 5V
		 *	11b = 3.0A @ 5V
		 */
		u32 fast_role_swap_required_current : 2;
		u32 dual_role_data : 1;
		u32 usb_communications_capable : 1;
		u32 externally_powered : 1;
		u32 higher_capability : 1;
		u32 dual_role_power : 1;
		u32 power_supply_type : 2;
	} sink_fixed_supply;

	struct sink_battery {
		u32 operational_current_in_10mA_units : 10;
		u32 minimum_voltage_in_50mV_units : 10;
		u32 maximum_voltage_in_50mV_units : 10;
		u32 power_supply_type : 2;
	} sink_battery;

	struct sink_variable_supply {
		u32 operational_power_in_250mW_units : 10;
		u32 minimum_voltage_in_50mV_units : 10;
		u32 maximum_voltage_in_50mV_units : 10;
		u32 power_supply_type : 2;
	} sink_variable_supply;
} __packed;

enum power_supply_type {
	POWER_TYPE_FIXED_SUPPLY		= 0,
	POWER_TYPE_BATTERY		= 1,
	POWER_TYPE_VARIABLE_SUPPLY	= 2,
};

union request_data_object {
	u32 rdo;

	/* Fxied and Variable Request Data Object */
	struct fixed_variable {
		u32 maximum_operating_current_10mA_units : 10;
		u32 operating_current_in_10mA_units : 10;
		u32 reserved : 3;
		u32 unchunked_extended_messages_supported : 1;
		u32 no_usb_suspend : 1;
		u32 usb_communications_capable : 1;
		u32 capability_mismatch : 1;
		u32 giveback_flag : 1;
		u32 object_position : 3;
		u32 reserved1 : 1;
	} fixed_variable;

	/* Fxied and Variable Request Data Object with GiveBack Support */
	struct fixed_variable_giveback {
		u32 minimum_operating_current_10mA_units : 10;
		u32 operating_current_in_10mA_units : 10;
		u32 reserved : 3;
		u32 unchunked_extended_messages_supported : 1;
		u32 no_usb_suspend : 1;
		u32 usb_communications_capable : 1;
		u32 capability_mismatch : 1;
		u32 giveback_flag : 1;
		u32 object_position : 3;
		u32 reserved1 : 1;
	} fixed_variable_giveback;

	/* Battery Request Data Object */
	struct battery {
		u32 maximum_operating_power_in_250mW_units : 10;
		u32 operating_power_in_250mW_units : 10;
		u32 reserved : 3;
		u32 unchunked_extended_messages_supported : 1;
		u32 no_usb_suspend : 1;
		u32 usb_communications_capable : 1;
		u32 capability_mismatch : 1;
		u32 giveback_flag : 1;
		u32 object_position : 3;
		u32 reserved1 : 1;
	} battery;

	/* Battery Request Data Object with GiveBack Support */
	struct battery_giveback {
		u32 minimum_operating_power_in_250mW_units : 10;
		u32 operating_power_in_250mW_units : 10;
		u32 reserved : 3;
		u32 unchunked_extended_messages_supported : 1;
		u32 no_usb_suspend : 1;
		u32 usb_communications_capable : 1;
		u32 capability_mismatch : 1;
		u32 giveback_flag : 1;
		u32 object_position : 3;
		u32 reserved1 : 1;
	} battery_giveback;
} __packed;

struct hpi_buffer {
	u8 *head;		/* Points to start of the memory buffer. */
	size_t size;		/* Total size in bytes of the memory buffer. */
};

struct hpi_v1_msg {
	u8 code;
	u8 len;
	u8 data[0];
} __packed;

struct hpi_v2_msg {
	u8 code;
	/*
	 * In CCGx FW, the length value is set in following format:
	 *	len[0] == WORD_GET_LSB(length)
	 *	len[1] == WORD_GET_LSB(length)
	 *	len[2] == WORD_GET_MSB(length)
	 * so, for HPIv2 FW, the response message length should be calculated
	 * as below format, the value of len[0] should be ignored:
	 *	real_msg_len = len[2] << 8 | len[1];
	 */
	u8 len[3];
	u8 data[0];
} __packed;

struct hpi_msg {
	u32 code;	/* The event code. */
	size_t len;	/* Tatal length in bytes of the message data. */
	u8 *data;	/* Points to message data. */
};

#define HPI_MSG_GET_CODE(_buf, _hpi_ver)	((u32)(u8)((_buf)[0]))
#define HPI_MSG_GET_LENGTH(_buf, _hpi_ver)				\
	(((_hpi_ver) == HPI_VERSION_1) ?				\
		((u32)(u8)((_buf)[1])) :				\
		(((u8)((_buf)[1])) | (((u8)((_buf)[2])) << 8) |	\
		 (((u8)((_buf)[3])) << 16)))
#define HPI_MSG_GET_DATA(_buf, _hpi_ver)			\
	(((_hpi_ver) == HPI_VERSION_1) ?			\
		(((u8 *)(_buf)) + sizeof(struct hpi_v1_msg)) :	\
		(((u8 *)(_buf)) + sizeof(struct hpi_v2_msg)))

#define HPI_MAX_BUFFER_SIZE(_hpi_ver)					\
	(((_hpi_ver) == HPI_VERSION_1) ?				\
		(sizeof(struct hpi_v1_msg) +				\
			(HPI_V1_REG_SIZE_OF_FLASH_RW)) :		\
		(sizeof(struct hpi_v2_msg) +				\
			(HPI_V2_REG_SIZE_OF_WRITE_DATA_MEMORY)))

#define HPI_MSG_EVENT_CODE_MASK	0x80
#define IS_HPI_EVENT_MSG(_code)		\
	(((_code)) & HPI_MSG_EVENT_CODE_MASK)
#define IS_HPI_RESPONSE_MSG(_code)	\
	!(IS_HPI_EVENT_MSG(_code))

/**
 * enum hpi_msg_return
 * @HPI_MSG_RETURN_IGNORED	message was not for the device command, pass it
 * @HPI_MSG_RETURN_NONE		message was not for the device command
 * @HPI_MSG_RETURN_HANDLED	message was handled by this device command
 */
enum hpi_msg_return {
	HPI_MSG_RETURN_IGNORED		= -1,
	HPI_MSG_RETURN_NONE		= 0,
	HPI_MSG_RETURN_HANDLED		= 1,
};
#define HPI_MSG_HANDLED(_ret)	((_ret) > 0)

struct hpi_device;
typedef enum hpi_msg_return (*hpi_cmd_filter_t)(struct hpi_device *hpidev,
						struct hpi_msg *msg);
typedef void (*hpi_cmd_cb_t)(struct hpi_device *hpidev, void *param,
		int errcode, void *data, size_t size);

typedef int (*hpi_reg_rw_t)(struct hpi_device *hpidev, void *buf, size_t size,
			    enum hpi_rw_mode rw_mode);


enum hpi_cmd_state {
	/*
	 * HPI_CMD_IDLE - indicates no command in processing currently.
	 */
	HPI_CMD_IDLE,
	/*
	 * HPI_CMD_INITED - indicates the command data struct hpi_command has
	 * has initialized and is ready to start a new command process.
	 * And the command lock has taken, the command will be started soon.
	 */
	HPI_CMD_INITED,
	/*
	 * HPI_CMD_ISSUED - indicates the command and corresponding data have
	 * been written to the CCG device through the communication bus.
	 */
	HPI_CMD_ISSUED,
	/*
	 * HPI_CMD_SYNC_MODE_CHECKED - indicates command issue thread has
	 * finished the command issue work and start to waiting for the
	 * response in sync mode or async mode.
	 */
	HPI_CMD_SYNC_MODE_CHECKED,
	/*
	 * HPI_CMD_RESPONSE_RECEIVED - indicates the response message for
	 * the command has been received, the command write is success or fail
	 */
	HPI_CMD_RESPONSE_RECEIVED,
	/*
	 * HPI_CMD_PRE_EVENTS_RCVD - indicates the command has more events
	 * from CCGx as a result besides the response message, and the first
	 * events without data messages has been received.
	 */
	HPI_CMD_PRE_EVENTS_RCVD,
	/*
	 * HPI_CMD_DATA_RECEIVED - indicates the command response data received
	 * after the command write successfully and executed by the CCG device.
	 */
	HPI_CMD_DATA_RECEIVED,
	/*
	 * HPI_CMD_SUF_EVENTS_RCVD - indicates the command has more events
	 * from CCGx as a result. And the events after the data contained
	 * messages has been received.
	 */
	HPI_CMD_SUF_EVENTS_RCVD,
	/*
	 * HPI_CMD_TIMEOUT - indicates the command waits timeout and still
	 * no expected events or data received from CCGx device.
	 */
	HPI_CMD_TIMEOUT,
	/*
	 * HPI_CMD_COMPLETED indicates the command has been finished
	 * successfully or encounterred error or timeout with no response.
	 */
	HPI_CMD_COMPLETED,
};


enum hpi_cmd_flag {
	/*
	 * Indicates the command response is special, the caller must set the
	 * specific filter to process the command response/event messages.
	 * The default process mechanism won't be used.
	 */
	HPI_CMD_FLAG_RAW,
	/*
	 * Indicates the command only have a response code message, no event
	 * code message will be involved.
	 */
	HPI_CMD_FLAG_RESP_ONLY,
	/*
	 * Indicates the command also have event messages need to be processed
	 * after the command response code message was received and success.
	 * And the event messages contains reset/recovery/error event messages,
	 * so, the commmand filter routine must be used to do special process,
	 * the driver default event filter process won't be called.
	 */
	HPI_CMD_FLAG_RESP_EVENT_RAW,
	/*
	 * Indicates the command also have event messages need to be processed
	 * after the command response code message was received and success.
	 * Any reset/recovery/error event message indicates the command failed,
	 * so, the driver default event filter routine will be used to process
	 * the command error events.
	 */
	HPI_CMD_FLAG_RESP_EVENT,
};

/*
 * struct hpi_command - represent a request of the operation through
 *   a cluster of registers
 */
struct hpi_command {
	struct mutex mlock;	/* Used to sync commands on same port. */
	spinlock_t slock;	/* Used to protect this data structure. */

	/* Indicates the command process stage. */
	enum hpi_cmd_state state;

	/*
	 * Used for command response timeout monitor in async mode.
	 * So the @state access must be protected by @slock.
	 */
	struct delayed_work timer;

	enum hpi_cmd_flag flag;

	/*
	 * When set to HPI_SYNC, the command issue thread wait/sleep until
	 * the command resposne/event/data message was received and finished,
	 * then wake up to continue the next process.
	 * When set to HPI_ASYNC, the command issue thread only send the
	 * command and data to the CCGx device, do not wait/sleep for the
	 * response messages and continue the process. The command response
	 * messages/data will be processed later in the caller set callback
	 * function when the command was finished. In HPI_ASYNC mode, a timer
	 * will be started to monitor the command response timeout.
	 */
	enum hpi_sync_mode sync_mode;
	/*
	 * Only used in HPI_SYNC mode, notify the command issue thread the
	 * command has been finished after the final message was received and
	 * processed.
	 */
	struct completion done;

	/*
	 * When the error code > 0, the error code indicates the CCG response
	 * message code; when the error code < 0, the error code indicates the
	 * system errno value; when the error code == 0, it indicates the
	 * command operation done success.
	 */
	int errcode;

	/*
	 * Command specific event filter function when event_filter_enabled is
	 * set to true.
	 */
	hpi_cmd_filter_t filter;
	uint cmd_id;	/* Input data for filter, such as command id. */
	void *data;	/* Output memory buffer for filter. */
	/*
	 * Size in bytes of the @data memory buffer.
	 * When return data, it inidcates the real size in bytes in the buffer.
	 */
	size_t *data_size;

	/*
	 * Async mode callback function and data after the command has been
	 * completed.
	 * @hpidev: instance of the USB-C port in this driver
	 * @param: instance of parameter for the callback function specified by
	 *	caller.
	 * @errcode: the final command executed result.
	 *	0	indicates success.
	 *	<0	indicates driver internal error encountered.
	 *	>0	indicates CCGx response with an error code and it's
	 *		the recevied error code.
	 * @data: the result of command response data if errcode is 0,
	 *	Otherwise it should be ignored.
	 * @size: the size in bytes of the @data memory buffer.
	 */
	hpi_cmd_cb_t callback;
	void *param;
	/*
	 * Used to store the copy of the event message data which existing
	 * in the async mode command.
	 */
	struct hpi_msg copy_msg;
	struct hpi_buffer copy_buf;

	struct hpi_msg msg;
	struct hpi_buffer msg_buf;
};

enum hpi_device_type {
	/* Indicates the struct hpi_device instance is invalid. */
	HPI_DEV_TYPE_UNKNOWN,
	/*
	 * The struct hpi_device instance can only operate on the
	 * CCG Device Information Registers.
	 * Such as CCG3 and CCG4 devices.
	 */
	HPI_DEV_TYPE_DEVICE,
	/*
	 * The struct hpi_device instance can only operate on the
	 * PD Policy/Status Registers.
	 * Such as CCG3 and CCG4 devices.
	 */
	HPI_DEV_TYPE_PORT,
	/*
	 * The struct hpi_device instance can operate on both of the
	 * CCG Device Information Registers and PD Policy/Status Registers.
	 * Such as for CCG1 and CCG2 devices.
	 */
	HPI_DEV_TYPE_MIXED,
};
#define IS_HPI_DEV_TYPE_PORT(_hpi_dev_type)	\
	((((_hpi_dev_type) == HPI_DEV_TYPE_PORT) ||	\
	  ((_hpi_dev_type) == HPI_DEV_TYPE_MIXED)) ? true : false)
#define IS_HPI_DEV_TYPE_DEVICE(_hpi_dev_type)	\
	((((_hpi_dev_type) == HPI_DEV_TYPE_DEVICE) ||	\
	  ((_hpi_dev_type) == HPI_DEV_TYPE_MIXED)) ? true : false)

/*
 * struct hpi_device - represent a cluster of registers can be operated
 */
struct hpi_device {
	struct kobject kobj;
	enum vdm_sop_type sysfs_sop_type;

	struct cyccg *cyccg;
	struct usbc_pdport_name_addr *pdport_name_addr;
	enum hpi_device_type dev_type;
	int idx;
	bool enabled;

	struct mutex mlock;
	spinlock_t slock;
	bool busying;

	/* Used to record the port status. */
	struct usbc_pdport_status status;
	bool is_swap_triggered_by_host;
	bool is_in_alt_mode;

	/* Avoid to be auto update again if successed when in retries. */
	unsigned long last_disconnect_time;
	bool cc_pa_update_done;
	bool cc_cable_update_done;
	struct work_struct cc_fw_work;
	enum cyccg_work_state cc_fw_work_state;
	bool do_cc_pa_update;
	bool do_cc_cable_update;

	u32 reg_base_addr;
	u32 reg_map_size;

	struct hpi_command cmd;

	struct list_head node;
	void (*event_callback)(char *port_name,
			u32 event_code, u8 *event_data, size_t size);
	struct mutex event_cb_mlock;
};

void hpi_cmd_timeout_function(struct work_struct *work);

void hpi_cmd_copy_return_data(struct hpi_device *hpidev, struct hpi_msg *msg);
void hpi_cmd_store_return_data(struct hpi_device *hpidev,
			       void *data, size_t size);
void hpi_cmd_set_state_errcode(struct hpi_device *hpidev,
		enum hpi_cmd_state new_state, int errcode);

enum hpi_msg_return hpi_device_default_command_handler(
		struct hpi_device *hpidev, struct hpi_msg *msg);

int hpi_device_read(struct hpi_device *hpidev,
		    u32 offset, size_t reg_size, void *buf, size_t buf_size);
int hpi_device_write(struct hpi_device *hpidev,
		     u32 offset, size_t reg_size, void *buf, size_t buf_size);

int hpi_message_read(struct hpi_device *hpidev, struct hpi_msg *msg);
int hpi_read_device_mode(struct cyccg *cyccg,
			 struct hpi_device_mode *device_mode);
int hpi_read_boot_mode_reason(struct cyccg *cyccg,
			      struct hpi_boot_mode_reason *boot_mode_reason);
int hpi_read_silicon_id(struct cyccg *cyccg, u16 *silicon_id);
int hpi_read_boot_loader_last_row(struct cyccg *cyccg, u16 *bl_last_row);
int hpi_read_intr_reg(struct cyccg *cyccg, union hpi_intr_reg *intr_reg);
int hpi_write_intr_reg(struct cyccg *cyccg, union hpi_intr_reg *intr_reg);
int hpi_clear_intr(struct hpi_device *hpidev);
int hpi_clear_all_intr(struct cyccg *cyccg);
int hpi_jump_to_boot_sync(struct cyccg *cyccg);
int hpi_jump_to_boot_async(struct cyccg *cyccg,
			   hpi_cmd_cb_t callback, void *param);
int hpi_jump_to_alt_fw_sync(struct cyccg *cyccg);
int hpi_jump_to_alt_fw_async(struct cyccg *cyccg,
			     hpi_cmd_cb_t callback, void *param);
int hpi_i2c_reset_sync(struct cyccg *cyccg);
int hpi_i2c_reset_async(struct cyccg *cyccg,
			hpi_cmd_cb_t callback, void *param);
int hpi_device_reset(struct cyccg *cyccg, enum hpi_sync_mode sync_mode,
		     hpi_cmd_cb_t callback, void *param);
int hpi_device_reset_sync(struct cyccg *cyccg);
int hpi_device_reset_async(struct cyccg *cyccg,
			   hpi_cmd_cb_t callback, void *param);
int hpi_enter_flashing_mode(struct cyccg *cyccg, enum hpi_sync_mode sync_mode,
			    hpi_cmd_cb_t callback, void *param);
int hpi_enter_flashing_mode_sync(struct cyccg *cyccg);
int hpi_enter_flashing_mode_async(struct cyccg *cyccg,
				  hpi_cmd_cb_t callback, void *param);
int hpi_validate_fw(struct cyccg *cyccg, enum ccg_fw_mode_type fw_mode,
		    enum hpi_sync_mode sync_mode,
		    hpi_cmd_cb_t callback, void *param);
int hpi_validate_fw_sync(struct cyccg *cyccg, enum ccg_fw_mode_type fw_mode);
int hpi_validate_fw_async(struct cyccg *cyccg, enum ccg_fw_mode_type fw_mode,
			  hpi_cmd_cb_t callback, void *param);
int hpi_flash_row_read(struct cyccg *cyccg, u16 row_num,
		       void *buf, size_t *size,
		       enum hpi_sync_mode sync_mode,
		       hpi_cmd_cb_t callback, void *param);
int hpi_flash_row_read_sync(struct cyccg *cyccg, u16 row_num,
			    void *buf, size_t *size);
int hpi_flash_row_read_async(struct cyccg *cyccg, u16 row_num,
			     void *buf, size_t *size,
			     hpi_cmd_cb_t callback, void *param);
int hpi_flash_row_write(struct cyccg *cyccg, u16 row_num,
			void *buf, size_t size,
			enum hpi_sync_mode sync_mode,
			hpi_cmd_cb_t callback, void *param);
int hpi_flash_row_write_sync(struct cyccg *cyccg, u16 row_num,
			     void *buf, size_t size);
int hpi_flash_row_write_async(struct cyccg *cyccg, u16 row_num,
			      void *buf, size_t size,
			      hpi_cmd_cb_t callback, void *param);
int hpi_read_all_version(struct cyccg *cyccg,
			 struct hpi_ccg_fw_version *all_version);
int hpi_read_fw2_version(struct cyccg *cyccg,
			 struct hpi_ccg_fw_version *all_version);
int hpi_read_fw_binary_location(struct cyccg *cyccg,
				u16 *fw1_start, u16 *fw2_start);
int hpi_pdport_enable(struct cyccg *cyccg, u8 port_mask,
		      enum hpi_sync_mode sync_mode,
		      hpi_cmd_cb_t callback, void *param);
int hpi_disable_all_dpport(struct cyccg *cyccg, enum hpi_sync_mode sync_mode,
			   hpi_cmd_cb_t callback, void *param);
int hpi_disable_all_dpport_sync(struct cyccg *cyccg);
int hpi_disable_all_dpport_async(struct cyccg *cyccg,
				 enum hpi_sync_mode sync_mode,
				 hpi_cmd_cb_t callback, void *param);
int hpi_read_pdport_bitmask(struct cyccg *cyccg, u8 *port_mask);
int hpi_disable_dpport(struct cyccg *cyccg, int port_index,
		       enum hpi_sync_mode sync_mode,
		       hpi_cmd_cb_t callback, void *param);
int hpi_disable_dpport_sync(struct cyccg *cyccg, int port_index);
int hpi_disable_dpport_async(struct cyccg *cyccg, int port_index,
			     hpi_cmd_cb_t callback, void *param);
int hpi_enable_dpport(struct cyccg *cyccg, int port_index,
		      enum hpi_sync_mode sync_mode,
		      hpi_cmd_cb_t callback, void *param);
int hpi_enable_dpport_sync(struct cyccg *cyccg, int port_index);
int hpi_enable_dpport_async(struct cyccg *cyccg, int port_index,
			    hpi_cmd_cb_t callback, void *param);
int hpi_sleep_ctrl(struct cyccg *cyccg, bool enable_deep_sleep,
		   enum hpi_sync_mode sync_mode,
		   hpi_cmd_cb_t callback, void *param);
int hpi_sleep_ctrl_sync(struct cyccg *cyccg, bool enable_deep_sleep,
			hpi_cmd_cb_t callback, void *param);
int hpi_sleep_ctrl_async(struct cyccg *cyccg, bool enable_deep_sleep,
			 hpi_cmd_cb_t callback, void *param);
int hpi_read_sleep_ctrl(struct cyccg *cyccg, bool *is_deep_sleep_enabled);
int hpi_set_battery_state(struct cyccg *cyccg, bool enable_dead_battery_ops,
			  enum hpi_sync_mode sync_mode,
			  hpi_cmd_cb_t callback, void *param);
int hpi_set_battery_state_sync(struct cyccg *cyccg,
			       bool enable_dead_battery_ops);
int hpi_set_battery_state_async(struct cyccg *cyccg,
				bool enable_dead_battery_ops,
				enum hpi_sync_mode sync_mode,
				hpi_cmd_cb_t callback, void *param);
int hpi_read_battery_state(struct cyccg *cyccg,
			   bool *is_dead_battery_ops_enabled);
int hpi_set_app_priority(struct cyccg *cyccg,
			 enum hpi_app_priority app_priority,
			 enum hpi_sync_mode sync_mode,
			 hpi_cmd_cb_t callback, void *param);
int hpi_set_app_priority_sync(struct cyccg *cyccg,
			 enum hpi_app_priority app_priority);
int hpi_set_app_priority_async(struct cyccg *cyccg,
			 enum hpi_app_priority app_priority,
			 enum hpi_sync_mode sync_mode,
			 hpi_cmd_cb_t callback, void *param);
int hpi_read_customer_info(struct cyccg *cyccg, void *buf, size_t *size,
			   enum hpi_sync_mode sync_mode,
			   hpi_cmd_cb_t callback, void *param);
int hpi_port_send_vdm_data_generic(struct hpi_device *port,
		enum vdm_sop_type vdm_mode, enum hpi_cmd_flag cmd_flag,
		void *vdm_data, size_t vdm_size,
		void *ret_vdm_data, size_t *ret_vdm_data_size,
		enum hpi_sync_mode sync_mode,
		hpi_cmd_cb_t callback, void *param,
		hpi_cmd_filter_t special_filter, unsigned long timeout);
int hpi_port_send_vdm_data(struct hpi_device *port,
			   enum vdm_sop_type vdm_mode,
			   void *vdm_data, size_t vdm_size,
			   void *ret_vdm_data, size_t *ret_vdm_data_size,
			   enum hpi_sync_mode sync_mode,
			   hpi_cmd_cb_t callback, void *param);
int hpi_port_send_vdm_data_sync(struct hpi_device *port,
				enum vdm_sop_type vdm_mode,
				void *vdm_data, size_t vdm_size,
				void *ret_vdm_data, size_t *ret_vdm_data_size);
int hpi_port_send_vdm_data_async(struct hpi_device *port,
				 enum vdm_sop_type vdm_mode,
				 void *vdm_data, size_t vdm_size,
				 hpi_cmd_cb_t callback, void *param);
int hpi_port_read_effective_source_pdo_mask(struct hpi_device *port, u8 *mask);
int hpi_port_read_effective_sink_pdo_mask(struct hpi_device *port, u8 *mask);
int hpi_port_select_source_pdo(struct hpi_device *port, u8 mask,
			       struct hpi_pd_src_sink_pdo_list *pdo_list_buf,
			       size_t pdo_list_buf_size,
			       enum hpi_sync_mode sync_mode,
			       hpi_cmd_cb_t callback, void *param);
int hpi_port_select_source_pdo_sync(struct hpi_device *port, u8 mask,
			struct hpi_pd_src_sink_pdo_list *pdo_list_buf,
			size_t pdo_list_buf_size);
int hpi_port_select_source_pdo_async(struct hpi_device *port, u8 mask,
			struct hpi_pd_src_sink_pdo_list *pdo_list_buf,
			size_t pdo_list_buf_size,
			hpi_cmd_cb_t callback, void *param);
int hpi_port_select_sink_pdo(struct hpi_device *port, u8 mask,
			     struct hpi_pd_src_sink_pdo_list *pdo_list_buf,
			     size_t pdo_list_buf_size,
			     enum hpi_sync_mode sync_mode,
			     hpi_cmd_cb_t callback, void *param);
int hpi_port_select_sink_pdo_sync(struct hpi_device *port, u8 mask,
			struct hpi_pd_src_sink_pdo_list *pdo_list_buf,
			size_t pdo_list_buf_size);
int hpi_port_select_sink_pdo_async(struct hpi_device *port, u8 mask,
			struct hpi_pd_src_sink_pdo_list *pdo_list_buf,
			size_t pdo_list_buf_size,
			hpi_cmd_cb_t callback, void *param);
int hpi_port_read_pd_status(struct hpi_device *port,
			    struct hpi_pd_status *pd_status);
int hpi_port_read_type_c_status(struct hpi_device *port,
				struct hpi_type_c_status *type_c_status);
int hpi_port_pd_control(struct hpi_device *port,
			enum hpi_pd_ctrl_command pd_ctrl_cmd,
			enum hpi_sync_mode sync_mode,
			hpi_cmd_cb_t callback, void *param);
int hpi_port_set_type_c_default_profile_sync(struct hpi_device *port);
int hpi_port_set_type_c_default_profile_async(struct hpi_device *port,
					      hpi_cmd_cb_t callback,
					      void *param);
int hpi_port_set_type_c_1_5_A_profile(struct hpi_device *port,
				      enum hpi_sync_mode sync_mode,
				      hpi_cmd_cb_t callback, void *param);
int hpi_port_set_type_c_3_A_profile(struct hpi_device *port,
				    enum hpi_sync_mode sync_mode,
				    hpi_cmd_cb_t callback, void *param);
int hpi_port_data_role_swap_sync(struct hpi_device *port);
int hpi_port_data_role_swap_async(struct hpi_device *port,
				  hpi_cmd_cb_t callback, void *param);
int hpi_port_power_role_swap_sync(struct hpi_device *port);
int hpi_port_power_role_swap_async(struct hpi_device *port,
				   hpi_cmd_cb_t callback, void *param);
int hpi_port_vconn_role_swap_sync(struct hpi_device *port);
int hpi_port_vconn_role_swap_async(struct hpi_device *port,
				   hpi_cmd_cb_t callback, void *param);
int hpi_port_switch_vconn_sync(struct hpi_device *port, bool sourcing_on);
int hpi_port_switch_vconn_async(struct hpi_device *port, bool sourcing_on,
				hpi_cmd_cb_t callback, void *param);
int hpi_port_switch_vconn_async(struct hpi_device *port, bool sourcing_on,
				hpi_cmd_cb_t callback, void *param);
int hpi_port_get_partner_source_capabilities_sync(
		struct hpi_device *port, void *buf, size_t *size);
int hpi_port_get_partner_source_capabilities_async(
		struct hpi_device *port, hpi_cmd_cb_t callback, void *param);
int hpi_port_get_partner_sink_capabilities_sync(
		struct hpi_device *port, void *buf, size_t *size);
int hpi_port_get_partner_sink_capabilities_async(
		struct hpi_device *port, hpi_cmd_cb_t callback, void *param);
int hpi_port_send_gotomin_sync(struct hpi_device *port);
int hpi_port_send_gotomin_async(struct hpi_device *port,
				hpi_cmd_cb_t callback, void *param);
int hpi_port_send_hard_reset(struct hpi_device *port,
			     enum hpi_sync_mode sync_mode,
			     hpi_cmd_cb_t callback, void *param);
int hpi_port_send_soft_reset(struct hpi_device *port,
			     enum hpi_sync_mode sync_mode,
			     hpi_cmd_cb_t callback, void *param);
int hpi_port_send_cable_reset(
		struct hpi_device *port, enum hpi_sync_mode sync_mode,
		hpi_cmd_cb_t callback, void *param);
int hpi_port_send_soft_reset_sop_prime(
		struct hpi_device *port, enum hpi_sync_mode sync_mode,
		hpi_cmd_cb_t callback, void *param);
int hpi_port_send_soft_reset_sop_dprime(
		struct hpi_device *port, enum hpi_sync_mode sync_mode,
		hpi_cmd_cb_t callback, void *param);
int hpi_port_ec_initialization_complete(
		struct hpi_device *port, enum hpi_sync_mode sync_mode,
		hpi_cmd_cb_t callback, void *param);
int hpi_port_ec_initialization_complete_sync(struct hpi_device *port);
int hpi_port_ec_initialization_complete_async(
		struct hpi_device *port, enum hpi_sync_mode sync_mode,
		hpi_cmd_cb_t callback, void *param);
int hpi_port_disable(struct hpi_device *port, enum hpi_sync_mode sync_mode,
		     hpi_cmd_cb_t callback, void *param);
int hpi_port_disable_sync(struct hpi_device *port);
int hpi_port_disable_async(struct hpi_device *port,
			   hpi_cmd_cb_t callback, void *param);
int hpi_port_enable(struct hpi_device *port, enum hpi_sync_mode sync_mode,
		    hpi_cmd_cb_t callback, void *param);
int hpi_port_enable_sync(struct hpi_device *port);
int hpi_port_enable_async(struct hpi_device *port,
		    hpi_cmd_cb_t callback, void *param);
int hpi_port_change_pd_port_parameters(struct hpi_device *port,
				       struct hpi_pd_port_change_config *config,
				       enum hpi_sync_mode sync_mode,
				       hpi_cmd_cb_t callback, void *param);
int hpi_port_abort_pending_pd_command(struct hpi_device *port,
				      enum hpi_sync_mode sync_mode,
				      hpi_cmd_cb_t callback, void *param);
int hpi_port_ovp_ocp_otp_triggered(struct hpi_device *port,
				   enum hpi_sync_mode sync_mode,
				   hpi_cmd_cb_t callback, void *param);
int hpi_port_read_source_pdo(struct hpi_device *port,
			     struct hpi_pd_src_sink_pdo_list *pdo_list_buf,
			     size_t *pdo_list_buf_size,
			     enum hpi_sync_mode sync_mode,
			     hpi_cmd_cb_t callback, void *param);
int hpi_port_read_sink_pdo(struct hpi_device *port,
			   struct hpi_pd_src_sink_pdo_list *pdo_list_buf,
			   size_t *pdo_list_buf_size,
			   enum hpi_sync_mode sync_mode,
			   hpi_cmd_cb_t callback, void *param);
int hpi_port_read_current_pdo(struct hpi_device *port, u32 *pdo);
int hpi_port_read_current_rdo(struct hpi_device *port, u32 *rdo);
int hpi_port_read_current_cable_vdo(struct hpi_device *port, u32 *vdo);
int hpi_port_read_ec_dp_hpd_control(struct hpi_device *port,
				    struct hpi_hpd_control *hpd_ctrl);
int hpi_port_ec_dp_hpd_control(struct hpi_device *port,
			       struct hpi_hpd_control *hpd_ctrl,
			       enum hpi_sync_mode sync_mode,
			       hpi_cmd_cb_t callback, void *param);
int hpi_port_read_ec_dp_mux_control(struct hpi_device *port,
				    enum hpi_dp_mux_config *dp_mux_config);
int hpi_port_ec_dp_mux_control(struct hpi_device *port,
			       enum hpi_dp_mux_config *dp_mux_config,
			       enum hpi_sync_mode sync_mode,
			       hpi_cmd_cb_t callback, void *param);
int hpi_port_read_trigger_dp_mode(struct hpi_device *port,
				  struct hpi_trigger_dp_mode *trigger_mode);
int hpi_port_trigger_dp_mode(struct hpi_device *port,
			     struct hpi_trigger_dp_mode *trigger_mode,
			     enum hpi_sync_mode sync_mode,
			     hpi_cmd_cb_t callback, void *param);
int hpi_port_read_dp_source_configure(struct hpi_device *port,
			   enum hpi_dp_source_config_mode *config);
int hpi_port_read_dp_sink_configure(struct hpi_device *port,
			   struct hpi_dp_sink_config_mode *config);
int hpi_port_read_dp_configure_mode(struct hpi_device *port, void *mode);
int hpi_port_dp_source_configure(struct hpi_device *port,
				 enum hpi_dp_source_config_mode *config,
				 enum hpi_sync_mode sync_mode,
				 hpi_cmd_cb_t callback, void *param);
int hpi_port_dp_sink_configure(struct hpi_device *port,
			       struct hpi_dp_sink_config_mode *config,
			       u32 *new_status_vdo,
			       enum hpi_sync_mode sync_mode,
			       hpi_cmd_cb_t callback, void *param);
int hpi_port_write_dp_config(struct hpi_device *port,
			     void *config, void *new_status_vdo,
			     enum hpi_sync_mode sync_mode,
			     hpi_cmd_cb_t callback, void *param);
int hpi_port_read_alt_mode_cmd(struct hpi_device *port,
			       struct hpi_alt_mode_cmd *alt_mode_cmd);
int hpi_port_alt_mode_cmd(struct hpi_device *port,
			  struct hpi_alt_mode_cmd *alt_mode_cmd,
			  enum hpi_sync_mode sync_mode,
			  hpi_cmd_cb_t callback, void *param);
int hpi_port_read_app_hw_cmd(struct hpi_device *port,
			       struct hpi_app_hw_cmd *app_hw_cmd);
int hpi_port_app_hw_cmd(struct hpi_device *port,
			  struct hpi_app_hw_cmd *app_hw_cmd,
			  enum hpi_sync_mode sync_mode,
			  hpi_cmd_cb_t callback, void *param);
int hpi_port_read_event_mask(struct hpi_device *port, u32 *event_mask);
int hpi_port_write_event_mask(struct hpi_device *port, u32 event_mask,
			      enum hpi_sync_mode sync_mode,
			      hpi_cmd_cb_t callback, void *param);
int hpi_port_write_event_mask_sync(struct hpi_device *port, u32 event_mask);
int hpi_port_write_event_mask_async(struct hpi_device *port, u32 event_mask,
				    hpi_cmd_cb_t callback, void *param);
int hpi_port_read_swap_response(struct hpi_device *port,
				struct hpi_swap_response_reg *swap_resp_reg);
int hpi_port_swap_response_update(struct hpi_device *port,
				  struct hpi_swap_response_reg *swap_resp_reg,
				  enum hpi_sync_mode sync_mode,
				  hpi_cmd_cb_t callback, void *param);
int hpi_port_swap_response_update(struct hpi_device *port,
				  struct hpi_swap_response_reg *swap_resp_reg,
				  enum hpi_sync_mode sync_mode,
				  hpi_cmd_cb_t callback, void *param);
int hpi_port_read_active_ec_modes(struct hpi_device *port,
				  bool *is_ec_has_active_alternate_modes);
int hpi_port_active_ec_modes(struct hpi_device *port,
			     bool is_ec_has_active_alternate_modes,
			     enum hpi_sync_mode sync_mode,
			     hpi_cmd_cb_t callback, void *param);
int hpi_port_read_vdm_ec_control(struct hpi_device *port,
				 struct hpi_vdm_ec_control *vdm_ec_ctrl);
int hpi_port_write_vdm_ec_control(struct hpi_device *port,
				  struct hpi_vdm_ec_control *vdm_ec_ctrl,
				  enum hpi_sync_mode sync_mode,
				  hpi_cmd_cb_t callback, void *param);
int hpi_port_read_cmd_timeout(struct hpi_device *port, u8 *cmd_timeout);
int hpi_port_write_cmd_timeout(struct hpi_device *port, u8 cmd_timeout,
			       enum hpi_sync_mode sync_mode,
			       hpi_cmd_cb_t callback, void *param);
int hpi_port_get_port_intr_status(struct hpi_device *port, u32 *intr_status);
int hpi_port_set_port_intr_status(struct hpi_device *port, u32 intr_status,
				  enum hpi_sync_mode sync_mode,
				  hpi_cmd_cb_t callback, void *param);
int hpi_port_read_disable_billboard_reset(struct hpi_device *port,
					  u8 *signature);
int hpi_port_read_billboard_reset_status(struct hpi_device *port,
					 bool *is_disabled);
int hpi_port_disable_billboard_reset(struct hpi_device *port, bool disabled,
				     enum hpi_sync_mode sync_mode,
				     hpi_cmd_cb_t callback, void *param);
int hpi_port_read_billboard_altmode_status(struct hpi_device *port,
		struct hpi_billboard_altmode_status *status);
int hpi_port_read_billboard_oper_model(struct hpi_device *port,
		struct hpi_billboard_oper_model *model_status);
int hpi_port_read_external_power_control(struct hpi_device *port,
		struct hpi_external_power_control *status);

int hpi_device_event_monitor(struct hpi_device *hpidev,
		enum hpi_sync_mode sync_mode,
		hpi_cmd_filter_t event_monitor_filter, unsigned long timeout,
		hpi_cmd_cb_t callback, void *param);
void hpi_cmd_get_async_callback_param(struct hpi_device *hpidev,
		hpi_cmd_cb_t callback, void *param);
int hpi_port_get_port_status(struct hpi_device *port,
			     struct usbc_pdport_status *port_status,
			     bool with_pdo_rdo_cable_vdo);

unsigned long get_bits_value(void *data, int bit_offset, int bit_num);
char *value_to_binary_string(unsigned long val, char *buf, size_t size);
char *hpi_vdm_sop_type_to_string(enum vdm_sop_type sop_type);
char *ccg_fw_mode_type_to_string(enum ccg_fw_mode_type mode_type,
				 enum ccg_version ccg_ver);
char *hpi_attached_dev_type_to_string(u8 attached_dev_type);
char *hpi_type_c_current_level_to_string(u8 current_level);
void hpi_dump_pending_events(struct hpi_device *hpidev);

void hex_dump(int flag, const void *hex_data, size_t size);
void hpi_pdo_dump(u32 pdo_data, bool is_source);
void hpi_capabilities_message_dump(struct hpi_device *port,
				   struct hpi_msg *msg);
void hpi_ccg_fw_version_dump(struct hpi_ccg_fw_version *ccg_fw_vers);


#define HPI_PORT_NAME(_port_p)						\
	((!(_port_p)) ? "NULL Port instance" :				\
		(((_port_p)->dev_type == HPI_DEV_TYPE_DEVICE) ?		\
			"CCG Device" :					\
			((_port_p)->pdport_name_addr ?			\
				(_port_p)->pdport_name_addr->name :	\
				"Unknown Port Name")))

#define ccg_err(fmt, ...)	\
	pr_err("%s: error: " fmt, __func__, ##__VA_ARGS__)
#define ccg_warn(fmt, ...)	\
	pr_warn("%s: warning: " fmt, __func__, ##__VA_ARGS__)
#if defined(CONFIG_CYCCG_DEBUG) && CONFIG_CYCCG_DEBUG
#define ccg_info(fmt, ...)	\
	pr_info("%s: info: " fmt, __func__, ##__VA_ARGS__)
#define ccg_dbg(fmt, ...)	\
	pr_err("%s: dbg: " fmt, __func__, ##__VA_ARGS__)
#else
#define ccg_info(fmt, ...)
#define ccg_dbg(fmt, ...)
#endif	/* CONFIG_CYCCG_DEBUG */
#if defined(CONFIG_CYCCG_VDEBUG) && CONFIG_CYCCG_VDEBUG
#define ccg_vdbg(fmt, ...)	\
	pr_err("%s: vdbg: " fmt, __func__, ##__VA_ARGS__)
#define ccg_dump(fmt, buf, size, ...)			\
do {							\
	ccg_vdbg(fmt, ##__VA_ARGS__);			\
	hex_dump(1, (buf), (size));			\
} while (0)
#else
#define ccg_vdbg(fmt, ...)
#define ccg_dump(fmt, buf, size, ...)
#endif /* CONFIG_CYCCG_VDEBUG */

#define port_err(fmt, port, ...)	\
	pr_err("%s: %s: error: " fmt, __func__,		\
		HPI_PORT_NAME(port), ##__VA_ARGS__)
#define port_warn(fmt, port, ...)	\
	pr_warn("%s: %s: warning: " fmt, __func__,	\
		HPI_PORT_NAME(port), ##__VA_ARGS__)
#if defined(CONFIG_CYCCG_DEBUG) && CONFIG_CYCCG_DEBUG
#define port_info(fmt, port, ...)	\
	pr_info("%s: %s: info: " fmt, __func__,	\
		HPI_PORT_NAME(port), ##__VA_ARGS__)
#define port_dbg(fmt, port, ...)	\
	pr_err("%s: %s: dbg: " fmt, __func__,		\
		HPI_PORT_NAME(port), ##__VA_ARGS__)
#else
#define port_info(fmt, port, ...)
#define port_dbg(fmt, port, ...)
#endif	/* CONFIG_CYCCG_DEBUG */
#if defined(CONFIG_CYCCG_VDEBUG) && CONFIG_CYCCG_VDEBUG
#define port_vdbg(fmt, port, ...)	\
	pr_err("%s: %s: vdbg: " fmt, __func__,		\
		HPI_PORT_NAME(port), ##__VA_ARGS__)
#define port_dump(fmt, port, buf, size, ...)		\
do {							\
	port_vdbg(fmt, (port), ##__VA_ARGS__);		\
	hex_dump(1, (buf), (size));			\
} while (0)
#else
#define port_vdbg(fmt, port, ...)
#define port_dump(fmt, port, buf, size, ...)
#endif	/* CONFIG_CYCCG_VDEBUG */



#endif  /* #ifndef _CYCCG_HPI_H */