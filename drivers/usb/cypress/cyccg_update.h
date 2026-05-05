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

#ifndef _CYCCG_UPDATE_H
#define _CYCCG_UPDATE_H

/* CCGx FW image name definitions used on Phone / Notebook. */
#define CCG1_FW_IMAGE_NAME		"cy_ccg1_fw_img.cybin"
#define CCG2_FW_IMAGE_NAME		"cy_ccg2_fw_img.cybin"
#define CCG3_FW_IMAGE1_NAME		"cy_ccg3_fw_img1.cybin"
#define CCG3_FW_IMAGE2_NAME		"cy_ccg3_fw_img2.cybin"
#define CCG4_FW_IMAGE1_NAME		"cy_ccg4_fw_img1.cybin"
#define CCG4_FW_IMAGE2_NAME		"cy_ccg4_fw_img2.cybin"

/* CCGx FW image name definitions used on Power Adapter. */
#define CCG1_PA_FW_IMAGE_NAME		"cy_ccg1_pa_fw_img.cybin"
#define CCG2_PA_FW_IMAGE_NAME		"cy_ccg2_pa_fw_img.cybin"
#define CCG3_PA_FW_IMAGE1_NAME		"cy_ccg3_pa_fw_img1.cybin"
#define CCG3_PA_FW_IMAGE2_NAME		"cy_ccg3_pa_fw_img2.cybin"
#define CCG4_PA_FW_IMAGE1_NAME		"cy_ccg4_pa_fw_img1.cybin"
#define CCG4_PA_FW_IMAGE2_NAME		"cy_ccg4_pa_fw_img2.cybin"

/* CCGx FW image name definitions used on Cables. */
#define CCG1_CABLE_FW_IMAGE_NAME	"cy_ccg1_cable_fw_img.cybin"
#define CCG2_CABLE_FW_IMAGE_NAME	"cy_ccg2_cable_fw_img.cybin"
#define CCG3_CABLE_FW_IMAGE1_NAME	"cy_ccg3_cable_fw_img1.cybin"
#define CCG3_CABLE_FW_IMAGE2_NAME	"cy_ccg3_cable_fw_img2.cybin"
#define CCG4_CABLE_FW_IMAGE1_NAME	"cy_ccg4_cable_fw_img1.cybin"
#define CCG4_CABLE_FW_IMAGE2_NAME	"cy_ccg4_cable_fw_img2.cybin"

#define CCG_FW_IMAGE_NAME_EXT		".cybin"

#define CCG_MAX_FW_IMAGE_PART_NUM	2

#define CYCCG_FW_UPDATE_RETRIES			3
#define CYCCG_FW_UPDATE_RETRY_INTERVAL		500	/* ms */
#define CYCCG_FW_UPDATE_WAIT_CHECK_INTERVAL	1000	/* ms */

#define CYCCG_CC_AUTO_FW_UPDATE_RECONNECT_TIME	5000	/* ms */

#define CCG_MAX_FLASH_MACROS	2

/* Indicates which parts that the cybin fimware image contians. */
enum cybin_image_type {
	CYBIN_IMAGE_TYPE_UNKNOWN	= 0x00,
	/*
	 * Only contains the config table part or
	 * config table with one or two customer info data part.
	 */
	CYBIN_IMAGE_TYPE_CONFIG		= 0x01,
	/* Only contains the FW code/data part. */
	CYBIN_IMAGE_TYPE_FW		= 0x02,
	/* Contains both the config table and the FW code/data parts. */
	CYBIN_IMAGE_TYPE_CONFIG_FW	= 0x03,
};

/* Indicates the image macro information in the device flash layout. */
struct ccg_flash_macro_info {
	/* Flash row size in bytes, 128/256. */
	u16 row_size;
	/*
	 * The minmum flash row of the FW image, maybe equal to the config
	 * table start row when the config table rows were at the start of the
	 * image.
	 */
	u16 start_row;
	/*
	 * The last flash row of FW image, not including the metadata row,
	 * only including the config and app coding flash rows.
	 * The metadata row can refer to the metadata_row.
	 */
	u16 end_row;
	u16 metadata_row;
	/*
	 * The config rows maybe put in the head of the FW image or maybe in
	 * the middle of the FW image.
	 */
	u16 config_start_row;
	u16 config_end_row;
	/* The flash row the FW version is embedded in. */
	u16 fw_info_row;
	u16 fw_info_row_offset;
};

/**
 * For CCG3 and CCG4 devices, the start and last flash row number of Image1
 * and Image2 will be different, must be set correspondingly.
 */
struct cybin_fw_image {
	const struct firmware *fw;
	char name[NAME_MAX];

	/* Points to the buffer contains the whole firmware image data. */
	const u8 *image;
	/* Total size in bytes of the firmware image. */
	size_t size;

	/* Indicates which CCG device the FW image dedicated for. */
	u16 silicon_id;
	enum ccg_version ccg_ver;
	struct ccg_flash_macro_info *macro_info;
	enum ccg_fw_mode_type fw_type;
	enum cybin_image_type image_type;
	struct hpi_image_version image_ver;

	/* Following fields only valid when config table contained. */
	u8 table_checksum;
	u8 flash_checksum;	/* Only valid for CCG3/CCG4 config table. */
};

struct ccg_update_info {
	struct cybin_fw_image fw_images[CCG_MAX_FLASH_MACROS];
	/* Point to the FW image that match target CCG device for FW updating */
	struct cybin_fw_image *fw_image;

	/* The basic info of the target CCG device for FW updating. */
	enum vdm_sop_type sop_type;
	enum ccg_type ccg_type;
	struct ccg_info ccg_info;
	enum ccg_fw_mode_type running_mode;
	struct hpi_ccg_fw_version ccg_fw_vers;
	/* The VDM object position of the CY_MODE. */
	u8 cy_mode_obj_pos;
};

struct ccg_silicon_info {
	/*
	 * The two-byte silicon ID value is coded in Big-Endian format in the
	 * released cybin image file; and is coded in Little-Endia format
	 * when read from the device through the HPI READ_SILICON_ID interface.
	 */
	__be16 silicon_id;
	/* Silicon ID extension informaiton. */
	u8 family_id;
	u8 chip_revision_id;
} __packed;

struct cybin_image_head {
	struct ccg_silicon_info silicon;
	u8 chip_revision;
	u8 checksum_type;
} __packed;

struct cybin_record {
	char colon;	/* Record start, always colon ":". */
	u8 flash_array_id;
	/*
	 * Indicates the row number in the CCG flash device that should
	 * be written to.
	 */
	__be16 record_num;
	/*
	 * Equals to the device's flash row size, not incuding the last
	 * checksum byte.
	 */
	__be16 record_data_size;
	u8 record_data[0];

	/*
	 * The last byte of the cybin_record data is the checksum value of this
	 * record data, the checksum value should not be included in the
	 * record_data memory range.
	 * The checksum is calculated including the byte of flash_array_id to
	 * the last byte of record_data[record_data_size - 1],
	 * totally 5 + record_data_size bytes.
	 * u8 checksum = record_data[record_data_size];
	 */
} __packed;

char *ccg_type_to_string(enum ccg_type ccg_type);

int cyccg_port_cc_discover_identity(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		void *resp_buf, size_t *resp_buf_size);

int cyccg_port_cc_discover_svid(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		void *resp_buf, size_t *resp_buf_size);

int cyccg_port_cc_discover_mode(struct hpi_device *port,
		enum vdm_sop_type vdm_mode, u16 svid,
		void *resp_buf, size_t *resp_buf_size);

int cyccg_port_cc_enter_mode(struct hpi_device *port,
		enum vdm_sop_type vdm_mode, u16 svid, u8 mode_obj_pos,
		u32 *resp_vdo);

int cyccg_port_cc_exit_mode(struct hpi_device *port,
		enum vdm_sop_type vdm_mode, u16 svid, u8 mode_obj_pos);

int cyccg_port_cc_attention(struct hpi_device *port,
		enum vdm_sop_type vdm_mode, u16 svid, u8 mode_obj_pos,
		u32 *resp_vdo);


int cyccg_port_cc_get_device_mode(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		enum ccg_fw_mode_type *running_mode, u16 *last_flash_row);

int cyccg_port_cc_get_device_version(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		struct hpi_ccg_fw_version *ccg_fw_vers);
int cyccg_port_cc_get_silicon_id(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		u32 *silicon_id, u8 *uuid, size_t uuid_buf_size);
int cyccg_port_cc_device_reset(struct hpi_device *port,
		enum vdm_sop_type vdm_mode);
int cyccg_port_cc_jump_to_boot(struct hpi_device *port,
		enum vdm_sop_type vdm_mode);
int cyccg_port_cc_jump_to_alt_fw(struct hpi_device *port,
		enum vdm_sop_type vdm_mode);
int cyccg_port_cc_enter_flashing_mode(struct hpi_device *port,
		enum vdm_sop_type vdm_mode);
int cyccg_port_cc_validate_fw(struct hpi_device *port,
		enum vdm_sop_type vdm_mode, enum ccg_fw_mode_type type);
int cyccg_port_cc_get_boot_mode_reason(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		struct hpi_boot_mode_reason *reason);
int cyccg_port_cc_get_checksum(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		u32 flash_addr, size_t data_size, u32 *checksum);
int cyccg_port_cc_get_fw_start_addr(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		u32 *fw_image1_start_addr, u32 *fw_image2_start_addr);
int cyccg_port_cc_set_app_priority(struct hpi_device *port,
		enum vdm_sop_type vdm_mode, enum ccg_fw_mode_type fw_mode);
int cyccg_port_cc_send_signature(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		void *signature, size_t signature_buf_size);
int cyccg_port_cc_get_boot_type(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		struct ccg_bootloader_type *bl_type);
int cyccg_port_cc_get_customer_info(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		void *read_buf, size_t read_buf_size);
int cyccg_port_cc_flash_record_write(struct hpi_device *port,
		enum vdm_sop_type vdm_mode, struct cybin_record *record);
int cyccg_port_cc_flash_record_read(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		u16 flash_row_num, size_t flash_row_size,
		void *record_read_buf, size_t record_read_buf_size);

int cyccg_update_do_fw_update(struct cyccg *cyccg,
		enum ccg_type ccg_type, bool auto_update,
		const char *buf, size_t count);

int cyccg_update_cc_enter_dfp_mode(struct hpi_device *port);
int cyccg_update_cc_enter_cy_mode(struct hpi_device *port,
		enum vdm_sop_type sop_type, u8 *cy_mode_obj_pos);
int cyccg_update_cc_reset_and_init(struct hpi_device *port,
		enum vdm_sop_type sop_type, u8 cy_mode_obj_pos,
		bool force_reset);

int cyccg_update_cc_do_port_fw_update(struct hpi_device *port,
		enum ccg_type ccg_type, enum vdm_sop_type sop_type,
		bool auto_update, const char *buf, size_t count);

void cyccg_port_cc_fw_update_state_reset(
		struct hpi_device *port, struct hpi_msg *msg);

void cyccg_port_cc_fw_update_work(struct work_struct *work);

void cyccg_port_queue_cc_auto_fw_update(struct hpi_device *port);

void cyccg_auto_fw_update_work(struct work_struct *work);
void cyccg_queue_auto_fw_update(struct cyccg *cyccg);



#endif  /* _CYCCG_UPDATE_H */