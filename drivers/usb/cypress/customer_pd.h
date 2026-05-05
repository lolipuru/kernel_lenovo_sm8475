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

#ifndef _PLATFORM_SAMPLE_H
#define _PLATFORM_SAMPLE_H

#define USBC_PDPORT_BUSY_RETRY_TIME	2000	/* ms */

enum usbc_pdport_power_adapter_state {
	PDPORT_PA_NONE,
	PDPORT_PA_REQUEST_RANDOM_DATA_RCVD,
	PDPORT_PA_EC_RANDON_DATA_SENT,
	PDPORT_PA_ENCRIPTED_RANDON_DATA_RCVD,

	PDPORT_PA_DR_SWAP_SENT,
	PDPORT_PA_DR_SWAPPED,

	PDPORT_PA_POWER_TUNE_READY,
	PDPORT_PA_POWER_TUNE_CMD_SENT,
	PDPORT_PA_POWER_TUNE_CMD_RESP_RCVD,
};

union pa_vdm_current_voltage_data {
	u32 vdo;
	struct {
		/*
		 * The value of the current level should be set based on the
		 * capability of the supported current grades. The value can be
		 * set between 0x00 ~ 0x0f.
		 * e.g.:
		 *	level 0  = 0 mA
		 *	level 1  = 312 mA
		 *	level 2  = 625 mA
		 *	...	 = ...
		 *	level 8  = 2500 mA
		 *	...	 = ...
		 *	level 15 = 5000 mA
		 */
		u8 current_level;
		u8 reserved;
		/*
		 * The value of the voltage should be calculated based on
		 * following rule of voltage * 10.
		 * e.g.: The target voltage is 4.4V, then voltage value set
		 *	here must be 4.4 * 10 = 44 = 0x2c
		 *	If 5V, then 5.0 * 10 = 50 = 0x32
		 *	If max 10V, then 20.0 * 10 = 200 = 0xc8
		 */
		u8 voltage;
		u8 reserved1;
	} pa_request;
	struct {
		u8 current_level;
		u8 voltage;
		u16 id;	/* PA_VDM_CMD_ID*/
	} pa_response;
} __packed;
/*
 * TODO (Customer):
 * Update the PA_VDM_CURRENT_LEVEL_GRANULARITY value based on the real
 * capability of the Poware Adapter.
 * Note, here the 375mA is just random picked sample.
 */
#define PA_VDM_CURRENT_LEVEL_GRANULARITY	312	/* units: mA */

/*
 * TODO (Customer):
 * Update the Power Adapter VDM operation code base value based on the
 * real desgin.
 * Note, here the code base value 0x4d490000 is just random picked sample.
 */
#define PA_VDM_CMD_ID	0x4d49
#define PA_VDM_OPCODE(_code_idx)	\
	(((PA_VDM_CMD_ID) << 16) | ((_code_idx) & 0x00007fff))
enum usbc_pdport_pa_vdm_opcode {
	PA_VDM_OPCODE_REQUEST_RANDOM_DATA	= PA_VDM_OPCODE(0),
	PA_VDM_OPCODE_RANDOM_DATA		= PA_VDM_OPCODE(1),
	PA_VDM_OPCODE_ENCRIPTED_RANDOM_DATA	= PA_VDM_OPCODE(2),
	PA_VDM_OPCODE_POWER_TUNE_CMD		= PA_VDM_OPCODE(3),
	PA_VDM_OPCODE_POWER_TUNE_CMD_RESP	= PA_VDM_OPCODE(4),
};

struct usbc_pdport_pa_vmd_data {
	u16 msg_header;
	u8 sop_type;
	u8 reserved;
	u32 vdm_header;
	u32 vdo[USBPD_VDM_DATA_VDOS];
};

int usbc_pdport_tune_power_current(char *port_name,
		u16 power_mV, u16 current_mA);
int usbc_pdport_tune_power_current_sync(char *port_name,
		u16 voltage_mV, u16 current_mA);



#endif  /* #ifndef _PLATFORM_SAMPLE_H */