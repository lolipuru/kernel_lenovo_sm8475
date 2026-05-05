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

#ifndef _CYCCG_H
#define _CYCCG_H

#include "linux/types.h"
#include <linux/input.h>

/* Contain the definitions and interfaces that public to other modules. */

#define CYCCG_NAME	"cyccg"
#define CYCCG_I2C_NAME	"cyccg_i2c"
#define CYCCG_SPI_NAME	"cyccg_spi"

#define CONFIG_CYCCG_DEBUG			1
#ifdef CONFIG_CYCCG_DEBUG
/* Set #define CONFIG_CYCCG_VDEBUG	0 to disable verbos debug messages. */
#define CONFIG_CYCCG_VDEBUG		(CONFIG_CYCCG_DEBUG)
#endif  /* CONFIG_CYCCG_DEBUG */
#define CONFIG_CONTROL_PMIC_USB_MODE		1
#define CONFIG_AUTO_HOST_CCG_FW_UPDATE		1
#define CONFIG_AUTO_POWER_ADAPTER_FW_UPDATE	0
#define CONFIG_AUTO_CABLE_FW_UPDATE		0
#define CONFIG_CUSTOMER_PD_TEST			0

#define HPI_V1_REG_ADDR_SIZE	1
#define HPI_V2_REG_ADDR_SIZE	2
#define HPI_V1_MAX_REG_RW_SIZE	128
#define HPI_V2_MAX_REG_RW_SIZE	512
#define HPI_MAX_REG_RW_SIZE	(HPI_V2_MAX_REG_RW_SIZE)
#define HPI_MAX_REG_ADDR_SIZE	(HPI_V2_REG_ADDR_SIZE)

struct ccg_bus_operations {
	u16 bustype;
	u16 bus;	/* The bus number of this bus. */
	u32 addr;	/* The device address on the bus. */

	/*
	 * detect - Execute bus protocol operations of device detecting
	 * &dev: Handle to the I2C/SPI Slave device
	 *
	 * This detects the exists of the device, return a negative errno code
	 * if not found or failed, else zero on success.
	 */
	int (*detect)(struct device *dev);
	/*
	 * read - Execute bus protocol operations to read data
	 * @dev: Handle to the I2C Slave device
	 * @reg_addr_bytes: the register address bytes based on HPI version
	 * @buf: Points to the buffer for return read data
	 * @size: Bytes of the data should be read from the device
	 * @addr: The start of the register address for reading
	 *
	 * This executes block read operation on the bus, return a negative
	 * errno code on failure, else zero on success.
	 */
	int (*read)(struct device *dev, size_t reg_addr_bytes,
		    void *buf, size_t size, u32 addr);
	/*
	 * write - Execute bus protocol operations to write data
	 * &dev: Handle to the I2C Slave device
	 * @reg_addr_bytes: the register address bytes based on HPI version
	 * @buf: Points to the buffer contains the data need to be written
	 * @size: Bytes of the data contained in the @buf memory buffer
	 * @addr: The start of the register address for writing
	 *
	 * This executes block write operation on the bus, return a negative
	 * errno code on failure, else zero on success.
	 */
	int (*write)(struct device *dev, size_t reg_addr_bytes,
		     void *buf, size_t size, u32 addr);
};

#define VDM_SVID_TERMINATING		0x0000
#define VDM_SVID_POWER_DELIVERY		0xff00
#define VDM_SVID_DISPLAY_PORT		0xff01
#define VDM_SVID_THUNDERBOLT		0x8087
#define VDM_SVID_CYPRESS		0x04b4

#define USBPD_MSG_HEADER_SIZE	2
#define USBPD_MAX_DATA_OBJS	7
#define USBPD_DATA_OBJ_SIZE	4
/* USBPD_VDM_DATA_VDOS number not including the VDM header data object. */
#define USBPD_VDM_DATA_VDOS	6
#define USBPD_VDM_DATA_SIZE	\
	((USBPD_VDM_DATA_VDOS) * (USBPD_DATA_OBJ_SIZE))
#define VDM_DATA_SIZE_TO_VDOS(_size)	((_size) / (USBPD_DATA_OBJ_SIZE))

enum usbc_pdport_mode {
	USB_UFP,
	USB_DFP,
	USB_DRP,
	USB_DEBUG,
	USB_AUDIO,
	USB_POWERED,
};

enum vdm_sop_type {
	VDM_SOP_TYPE_SOP	= 0x00,
	VDM_SOP_TYPE_SOP_PRIME	= 0x01,
	VDM_SOP_TYPE_SOP_DPRIME	= 0x02,
};

enum vdm_packet_type {
	VDM_TYPE_UNSTRUCTURED	= 0,
	VDM_TYPE_STRUCTURED	= 1,
};
#define VDM_PACKET_TYPE_MASK	0x00008000
#define GET_VMD_PACKET_TYPE(_vmd_header_vdo)		\
	(((_vmd_header_vdo) & VDM_PACKET_TYPE_MASK) ?	\
		VDM_TYPE_STRUCTURED : VDM_TYPE_UNSTRUCTURED)
#define GET_VDM_SVID(_vdm_header_vdo)	\
	((u16)(((_vdm_header_vdo) >> 16) & 0x0000ffff))

enum vdm_control_message_type {
	VDM_CTRL_MSG_TYPE_GOODCRC		= 0x01,
	VDM_CTRL_MSG_TYPE_GOTOMIN		= 0x02,
	VDM_CTRL_MSG_TYPE_ACCEPT		= 0x03,
	VDM_CTRL_MSG_TYPE_REJECT		= 0x04,
	VDM_CTRL_MSG_TYPE_PING			= 0x05,
	VDM_CTRL_MSG_TYPE_RS_RDY		= 0x06,
	VDM_CTRL_MSG_TYPE_GET_SRC_CAP		= 0x07,
	VDM_CTRL_MSG_TYPE_GET_SINK_CAP		= 0x08,
	VDM_CTRL_MSG_TYPE_DR_SWAP		= 0x09,
	VDM_CTRL_MSG_TYPE_PR_SWAP		= 0x0a,
	VDM_CTRL_MSG_TYPE_VCONN_SWAP		= 0x0b,
	VDM_CTRL_MSG_TYPE_WAIT			= 0x0c,
	VDM_CTRL_MSG_TYPE_SOFT_RESET		= 0x0d,
	VDM_CTRL_MSG_TYPE_NOT_SUPPORTED		= 0x10,
	VDM_CTRL_MSG_TYPE_GET_SRC_CAP_EXTENDED	= 0x11,
	VDM_CTRL_MSG_TYPE_GET_STATUS		= 0x12,
	VDM_CTRL_MSG_TYPE_FR_SWAP		= 0x13,
};

enum vdm_data_message_type {
	VDM_DATA_MSG_TYPE_SRC_CAP		= 0x01,
	VDM_DATA_MSG_TYPE_REQUEST		= 0x02,
	VDM_DATA_MSG_TYPE_BIST			= 0x03,
	VDM_DATA_MSG_TYPE_SINK_CAP		= 0x04,
	VDM_DATA_MSG_TYPE_BATTERY_STATUS	= 0x05,
	VDM_DATA_MSG_TYPE_ALERT			= 0x06,
	VDM_DATA_MSG_TYPE_VENDOR_DEFINED	= 0x0f,
};

union vdm_msg_header {
	u16 msg_header;
	struct {
		/*
		 * b4-b0: Message Type.
		 *   Detail value of this filed can refer to the definitions of
		 *   enum vdm_control_message_type when @data_objs = 0, and
		 *   enum vdm_data_message_type when @data_objs > 0.
		 */
		u16 msg_type : 5;
		/*
		 * b5: Port data role, 0 - UFP or 1 - DFP, SOP only;
		 *     Reserved for SOP' and SOP''.
		 */
		u16 port_data_role : 1;	/* b5: SOP only */
		/*
		 * Power Delivery Specification revision supported by Device.
		 *   00b - Revision 1.0
		 *   01b - Revision 2.0
		 *   10b - Revision 3.0
		 *   11b - reserved
		 */
		u16 spec_rev: 2;	/* b7-b6 */
		/*
		 * b8: if is SOP packet type, indicates the Port Power Role is
		 *	  0 - Sink; 1 - Source.
		 *     if is SOP' or SOP'' packet type, indicates the message
		 *     origiated from a Cable Plug or a DFP/UFP:
		 *	  0 - DFP/UFP;	1 - Cable Plug
		 */
		u16 port_power_role_cable_plug : 1;
		u16 message_id : 3;	/* b11-b9: MesssageID */
		u16 data_objs : 3;	/* b14-b12: Number of Data Objects */
		/* b15: 0 - Control or data Message; 1 - Entended Message */
		u16 extended : 1;
	};
} __packed;

struct usbpd_vdm_message {
	__le16 msg_header;
	u8 sop_type;
	u8 reserved;
	u32 vdo[0];	/* 0 .. USBPD_VDM_DATA_VDOS data VDOs. */
} __packed;

/*
 * Macro to generate the USB-C PD Port name based on the _port_num.
 * The value of _port_num can be set to 1 .. USBC_PDPORT_NUM.
 * The value of USBC_PDPORT_NUM must be set based on the real board desgin.
 */
#define TO_STR(s)	#s
#define USBC_PDPORT_NUM			8
#define USBC_PDPORT_NAME(_port_num)	TO_STR(USBC_PDPORT_##_port_num)
#define USBC_PDPORT_NAME_FORMAT		"USBC_PDPORT_%d"
#define USBC_PDPORT_NAME_SIZE		16
/*
 * TODO (Customer):
 * Alias macro can be defined here for easy of use.
 * e.g.:
 * The macro USBC_PDPORT_DEFAULT can be used when only one CCG device
 * and only one USB-C PD port in the board/system desgin.
 */
#define USBC_PDPORT_DEFAULT	USBC_PDPORT_NAME(1)

struct usbc_pdport_name_addr {
	char *name;	/* USB-C port name. */
	u16 bustype;	/* Bus type, I2C or SPI bus. */
	u16 bus;	/* Bus number. */
	u32 addr;	/* Address on the Bus. */
	/* Port index supported in same CCG device, started from 0 */
	int port_index;
	struct device *dev;	/* CCG device instance. */
};

struct usbc_pdport_status {
	/******** Type-C Status ********/
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
	 * b5: Ra Status.
	 *   0 - If CCG not detects Ra;
	 *   1 - CCG detects Ra.
	 *   This bit only valid when CCG is source,
	 */
	u8 ra_detected : 1;
	/*
	 * b7-b6: Type-C current level.
	 *   00 - Default; 01 - 1.5A; 02 - 3A.
	 */
	u8 current_level : 2;
	u8 reserved1;
	u16 reserved2;

	/******** Port PD Status ********/
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
	u8 reserved3 : 1;	/* b7 */
	/*
	 * b8: Current Port Power Role.
	 *  0 - Sink; 1 - Source.
	 *  This bit only valid when the contract_state is set.
	 */
	u8 current_power_role : 1;
	u8 reserved4 : 1;	/* b9 */
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
	u8 reserved5 : 2;	/* b15-b14 */
	u16 resereved6;

	u32 current_pdo;
	u32 current_rdo;
	u32 current_cable_vdo;
};

struct usbc_pdport_operations {
	int (*get_port_status)(char *port_name,
			struct usbc_pdport_status *port_status);
	int (*send_vdm)(char *port_name, u8 *vdm_cmd_data, size_t size);
	int (*data_role_swap)(char *port_name);
	int (*power_role_swap)(char *port_name);
	int (*vconn_swap)(char *port_name);
	int (*vconn_switch)(char *port_name, bool on);
};

/*
 * TODO (Customer):
 * Define the event code used in the event_callback callback function.
 * Customer can re-define the events required here and replace all places in
 * in the code where the event was used.
 */
enum usbc_pdport_event_code {
	PDPORT_TYPE_C_CONNECTED,      /* HPI_PD_RESP_TYPE_C_CONNECTED */
	PDPORT_TYPE_C_DISCONNECTED,   /* HPI_PD_RESP_TYPE_C_DISCONNECTED */
	PDPORT_CONTRACT_ESTABLISHED,  /* HPI_PD_RESP_PD_CONTRACT_ESTABLISHED */
	PDPORT_DR_SWAPPED,	      /* HPI_PD_RESP_SWAP_COMPLETE */
	PDPORT_PR_SWAPPED,	      /* HPI_PD_RESP_SWAP_COMPLETE */
	PDPORT_VCONN_SWAPPED,	      /* HPI_PD_RESP_SWAP_COMPLETE */
	PDPORT_VCONN_SWITCHED,	      /* HPI_PD_RESP_SUCCESS */
	PDPORT_PS_RDY,		      /* HPI_PD_RESP_PS_RDY */
	PDPORT_ACCEPT_RECEIVED,	      /* HPI_PD_RESP_ACCEPT_MESSAGE */
	PDPORT_SOURCE_CAP_RCVD,	      /* HPI_PD_RESP_SRC_CAP_RCVD */
	PDPORT_SINK_CAP_RCVD,	      /* HPI_PD_RESP_SINK_CAP_RCVD */
	PDPORT_VDM_RECEIVED,	      /* HPI_PD_RESP_VDM_RECEIVED */

	/*
	 * Indicates some error happened or command failed in the internal
	 * process of cyccg driver for the externel module sent command.
	 */
	PDPORT_CMD_FAIL_ERROR_DETECTED,
	/*
	 * The USB-C PD Port driver is removed, so send this event out to
	 * notify registered mouldes to unregister with it.
	 */
	PDPORT_DRV_REMOVED,
};

typedef void (*usbc_pdport_event_cb_t)(char *port_name,
			u32 event_code, u8 *event_data, size_t size);
extern struct usbc_pdport_name_addr *cyccg_get_usbc_pdport_name_addr_by_name(
		char *port_name);
extern struct usbc_pdport_name_addr *cyccg_get_usbc_pdport_name_addr_by_addr(
		u16 bustype, u16 bus, u32 addr, int port_index);
extern int cyccg_usbc_pdport_event_register(char *port_name,
		struct usbc_pdport_operations **pdport_ops,
		usbc_pdport_event_cb_t usbc_pdport_event_callback);
extern void cyccg_usbc_pdport_event_unregister(char *port_name);


extern int cyccg_probe(struct device *dev, int irq,
		struct ccg_bus_operations *ops);
extern int cyccg_remove(struct device *dev);
extern int cyccg_suspend(struct device *dev);
extern int cyccg_resume(struct device *dev);
extern const struct dev_pm_ops cyccg_pm_ops;

#if defined(CONFIG_CUSTOMER_PD_TEST) && CONFIG_CUSTOMER_PD_TEST
extern void customer_pd_init(void);
extern void customer_pd_uninit(void);
#endif

#endif  /* #ifndef _CYCCG_H */