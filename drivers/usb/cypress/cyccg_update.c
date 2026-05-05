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

#include <linux/firmware.h>
#include "cyccg_core.h"
#include "linux/delay.h"

#define fw_err(fmt, ...)	ccg_err(fmt, ##__VA_ARGS__)
#define fw_warn(fmt, ...)	ccg_warn(fmt, ##__VA_ARGS__)
#define fw_info(fmt, ...)	ccg_info(fmt, ##__VA_ARGS__)
#define fw_dbg(fmt, ...)	ccg_dbg(fmt, ##__VA_ARGS__)
#define fw_vdbg(fmt, ...)	ccg_vdbg(fmt, ##__VA_ARGS__)
#define fw_dump(fmt, buf, size, ...)		\
	ccg_dump(fmt, buf, size, ##__VA_ARGS__)

#define cc_port_err(fmt, port, ...)	port_err(fmt, port, ##__VA_ARGS__)
#define cc_port_warn(fmt, port, ...)	port_warn(fmt, port, ##__VA_ARGS__)
#define cc_port_info(fmt, port, ...)	port_info(fmt, port, ##__VA_ARGS__)
#define cc_port_dbg(fmt, port, ...)	port_dbg(fmt, port, ##__VA_ARGS__)
#define cc_port_vdbg(fmt, port, ...)	port_vdbg(fmt, port, ##__VA_ARGS__)
#define cc_port_dump(fmt, port, buf, size, ...)	\
	port_dump(fmt, port, buf, size, ##__VA_ARGS__)



/*
 * CCG1 Silicon ID: 0x04xx
 * CCG1 config table rows locates in the front of the image start row.
 */
static struct ccg_flash_macro_info ccg1_flash_macro_info_list[] = {
	/* CCG1 Mobile / Notebook rows info */
	{ 128, 0x0022, 0x00ff, 0x00ff, 0x001e, 0x0021, 0x0024, 0x0000 },
	/* CCG1 Mobile / EMCA image rows info */
	{ 128, 0x0056, 0x00ff, 0x00ff, 0x0052, 0x0055, 0x0058, 0x0000 },

	/* Indicates the end of the array. */
	{ 0, 0, 0, 0, 0, 0, 0, 0},
};

/*
 * CCG2 Silicon ID: 0x14xx
 * CCG2 config table rows locates in the front of the image start row.
 */
static struct ccg_flash_macro_info ccg2_flash_macro_info_list[] = {
	/* CCG2 Mobile / Notebook image rows info */
	{ 128, 0x0024, 0x00ff, 0x00ff, 0x0020, 0x0023, 0x0026, 0x0000 },
	/* CCG2 Mobile / Power Adapter / EMCA image rows info */
	{ 128, 0x0064, 0x00ff, 0x00ff, 0x0060, 0x0063, 0x0066, 0x0000 },

	/* Indicates the end of the array. */
	{ 0, 0, 0, 0, 0, 0, 0, 0},
};

/*
 * CCG3 Silicon ID: 0x1Dxx
 * CCG3 config table rows locates bwteen the image start and end rows.
 */
static struct ccg_flash_macro_info ccg3_flash_macro_info_list[] = {
	/* CCG3 Notebook/Power Adapter Image-1 rows info */
	{ 128, 0x0030, 0x01ff, 0x03ff, 0x0032, 0x0039, 0x0031, 0x0060 },
	/* CCG3 Notebook/Power Adapter Image-2 rows info */
	{ 128, 0x0200, 0x03fd, 0x03fe, 0x0202, 0x0209, 0x0201, 0x0060 },

	/* CCG3 Notebook Image-1 rows info */
	{ 128, 0x0030, 0x0214, 0x03ff, 0x0032, 0x0039, 0x0031, 0x0060 },
	/* CCG3 Notebook Image-2 rows info */
	{ 128, 0x0215, 0x03fd, 0x03fe, 0x0217, 0x021e, 0x0216, 0x0060 },

	/* CCG3 Notebook TBT/TBT_EVK Image-1 rows info */
	{ 128, 0x0030, 0x023a, 0x03ff, 0x0032, 0x0039, 0x0031, 0x0060 },
	/* CCG3 Notebook TBT/TBT_EVK Image-2 rows info */
	{ 128, 0x023b, 0x03fd, 0x03fe, 0x023d, 0x0245, 0x023c, 0x0060 },

	/* CCG3 CTD_US rows info */
	{ 128, 0x0030, 0x0215, 0x03ff, 0x0032, 0x0041, 0x0031, 0x0060 },
	/* CCG3 CTD_US Image-2 rows info */
	{ 128, 0x0216, 0x03fd, 0x03fe, 0x0217, 0x0228, 0x0217, 0x0060 },

	/* CCG3 DP_DONGLE rows info */
	{ 128, 0x0030, 0x01ff, 0x03ff, 0x0032, 0x0041, 0x0031, 0x0060 },
	/* CCG3 DP_DONGLE Image-2 rows info */
	{ 128, 0x0200, 0x03fd, 0x03fe, 0x0202, 0x0211, 0x0201, 0x0060 },

	/* Indicates the end of the array. */
	{ 0, 0, 0, 0, 0, 0, 0, 0},
};

/*
 * CCG4 Silicon ID: 0x18xx, 0x1Fxx
 * CCG4 config table rows locates bewteen the image start and end rows.
 */
static struct ccg_flash_macro_info ccg4_flash_macro_info_list[] = {
	/* CCG4 Notebook Image-1 rows info */
	{ 256, 0x0014, 0x00ff, 0x01ff, 0x0015, 0x0018, 0x0014, 0x00e0 },
	/* CCG4 Notebook Image-2 rows info */
	{ 256, 0x0100, 0x01fd, 0x01fe, 0x0101, 0x0104, 0x0100, 0x00e0 },

	/* CCG4 Notebook Image-1 rows info */
	{ 256, 0x0014, 0x0108, 0x01ff, 0x0015, 0x0018, 0x0014, 0x00e0 },
	/* CCG4 Notebook Image-2 rows info */
	{ 256, 0x0109, 0x01fd, 0x01fe, 0x010a, 0x010d, 0x0109, 0x00e0 },

	/* CCG4 Notebook Image-1 rows info */
	{ 256, 0x0018, 0x00ff, 0x01ff, 0x0019, 0x001c, 0x0018, 0x00e0 },
	/* CCG4 Notebook Image-2 rows info */
	{ 256, 0x0104, 0x01fd, 0x01fe, 0x0105, 0x0108, 0x0104, 0x00e0 },

	/* CCG4 Notebook TBT/TBT_EVK Image-1 rows info */
	{ 256, 0x0014, 0x0118, 0x01ff, 0x0015, 0x0018, 0x0014, 0x00e0 },
	/* CCG4 Notebook TBT/TBT_EVK Image-2 rows info */
	{ 256, 0x0119, 0x01fd, 0x01fe, 0x011a, 0x010d, 0x0119, 0x00e0 },

	/* Indicates the end of the array. */
	{ 0, 0, 0, 0, 0, 0, 0, 0},
};

char *ccg_type_to_string(enum ccg_type ccg_type)
{
	switch (ccg_type) {
	case CCG_NOTEBOOK_MOBILE_MONITOR:
		return "Notebook/Mobile/Monitor";
	case CCG_POWER_ADAPTER:
		return "Power Adapter";
	case CCG_CABLE:
		return "Cable";
	default:
		return "Unknown";
	}
}

static inline u8 cybin_record_get_checksum(struct cybin_record *cybin_record)
{
	size_t offset = be16_to_cpu(cybin_record->record_data_size);

	return cybin_record->record_data[offset];
}

static inline size_t cybin_record_size(struct cybin_record *cybin_record)
{
	return sizeof(struct cybin_record) +
			be16_to_cpu(cybin_record->record_data_size) + 1;
}

static inline u8 cybin_record_calculate_checksum(struct cybin_record *record)
{
	u8 *data = &record->flash_array_id;
	size_t size = be16_to_cpu(record->record_data_size) + 5;
	u8 csum = 0;

	for (; size > 0; size--)
		csum += data[size - 1];
	return (u8)((0xff - csum) + 1);
}

static u8 cybin_calculate_config_table_checksum(
		struct cybin_record *record,
		struct ccg_flash_macro_info *macro_info)
{
	size_t size = (size_t)be16_to_cpu(record->record_data_size);
	u16 row_num = be16_to_cpu(record->record_num);
	int offset;
	static u8 csum;

	if (row_num < macro_info->config_start_row ||
			row_num > macro_info->config_end_row)
		return 0;

	offset = 0;
	if (row_num == macro_info->config_start_row) {
		offset = HPI_CONFIG_TABLE_CHECKSUM_CALCULATE_START_OFFSET;
		csum = 0;
	}

	for (; offset < size; offset++)
		csum += record->record_data[offset];

	if (row_num == macro_info->config_end_row)
		return (u8)((0xff - csum) + 1);
	return csum;
}

static struct cybin_record *cybin_get_next_cybin_record(
		struct cybin_fw_image *fw_image, struct cybin_record *record)
{
	struct cybin_record *next_record = NULL;
	const u8 *boundary;
	const u8 *cursor;

	if (!fw_image || !fw_image->image || !fw_image->size)
		return NULL;

	boundary = fw_image->image + fw_image->size;
	if (record)
		cursor = (u8 *)record + cybin_record_size(record);
	else
		cursor = fw_image->image + sizeof(struct cybin_image_head);

	if (*cursor == '\r' && *(cursor + 1) == '\n' && *(cursor + 2) == ':')
		next_record = (struct cybin_record *)(cursor + 2);
	else if ((*cursor == '\r' || *cursor == '\n') && *(cursor + 1) == ':')
		next_record = (struct cybin_record *)(cursor + 1);
	else
		return NULL;	/* Not match the cybin_record format. */

	if (((u8 *)next_record + cybin_record_size(next_record)) > boundary)
		return NULL;	/* Invalid record data, exceed boundary. */

	return next_record;
}

/* Get the first cybin_record data in the image. */
static inline struct cybin_record *cybin_get_cybin_record(
		struct cybin_fw_image *fw_image)
{
	return cybin_get_next_cybin_record(fw_image, NULL);
}

/* Get the last cybin_record data in the image. */
static inline int cybin_calculate_total_record_num(
		struct cybin_fw_image *fw_image)
{
	struct cybin_record *record = cybin_get_cybin_record(fw_image);
	size_t fw_image_row_size;
	u8 *end_char;

	if (!record)
		return 0;

	end_char = (u8 *)record + cybin_record_size(record);
	if (*end_char == '\r' && *(end_char + 1) == '\n')
		fw_image_row_size = 2;
	else
		fw_image_row_size = 1;
	fw_image_row_size += cybin_record_size(record);
	return (fw_image->size -
			sizeof(struct cybin_image_head)) / fw_image_row_size;
}

/*
 * Get the record data in reverse order, if @record is NULL, then return the
 * last cybin_record data in the FW image.
 */
static inline struct cybin_record *cybin_get_prev_record(
		struct cybin_fw_image *fw_image, struct cybin_record *record)
{
	struct cybin_record *first_record = cybin_get_cybin_record(fw_image);
	size_t carrage_return_size;
	size_t image_row_size;
	const u8 *cursor;

	if (!first_record)
		return NULL;

	cursor = fw_image->image + sizeof(struct cybin_image_head);
	if (*cursor == '\r' && *(cursor + 1) == '\n')
		carrage_return_size = 2;
	else
		carrage_return_size = 1;
	image_row_size = carrage_return_size + cybin_record_size(first_record);

	if (record)
		cursor = (u8 *)record - image_row_size;
	else
		cursor = fw_image->image + fw_image->size - image_row_size;

	if (cursor < fw_image->image + sizeof(struct cybin_image_head))
		return NULL;

	if (*cursor == ':')
		return (struct cybin_record *)cursor;
	else if (*(cursor + carrage_return_size) == ':')
		return (struct cybin_record *)(cursor + carrage_return_size);
	else if (*(cursor - carrage_return_size) == ':')
		return (struct cybin_record *)(cursor - carrage_return_size);
	return NULL;
}

/*
 * The metadata is always the last flash row in the FW image if not only the
 * configuration table image.
 * The configuraiton table image only has the configuration table rows,
 */
static inline struct cybin_record *cybin_get_last_record(
		struct cybin_fw_image *fw_image)
{
	return cybin_get_prev_record(fw_image, NULL);
}

static inline struct cybin_record *cybin_get_metadata_record(
		struct cybin_fw_image *fw_image)
{
	return cybin_get_last_record(fw_image);
}

static void cybin_image_version_parse(
		struct hpi_image_version *image_ver, void *fw_ver_info)
{
	u8 *buf = fw_ver_info;

	memset(image_ver, 0, sizeof(struct hpi_image_version));
	image_ver->base.major = (buf[3] >> 4) & 0x0f;
	image_ver->base.minor = buf[3] & 0x0f;
	image_ver->base.patch_ver = buf[2];
	image_ver->base.build_number = get_unaligned_le16(&buf[0]);
	image_ver->app.major = (buf[7] >> 4) & 0x0f;
	image_ver->app.minor = buf[7] & 0x0f;
	image_ver->app.external_circuit_ver = buf[6];
	image_ver->app.name[0] = buf[5];
	image_ver->app.name[1] = buf[4];
}

static bool cybin_is_valid_record_num(u16 record_num,
		struct ccg_flash_macro_info *macro_info)
{
	if (record_num >= macro_info->start_row &&
			record_num <= macro_info->end_row)
		return true;

	if (record_num >= macro_info->config_start_row &&
			record_num <= macro_info->config_end_row)
		return true;

	if (record_num == macro_info->metadata_row)
		return true;

	return false;
}

static struct ccg_update_info *
cyccg_update_allocate_ccg_update_info(struct cyccg *cyccg)
{
	struct device *dev = cyccg->dev;
	struct ccg_update_info *update_info;

	update_info = devm_kzalloc(dev,
				sizeof(struct ccg_update_info), GFP_KERNEL);
	if (!update_info)
		return NULL;

	return update_info;
}

static int cyccg_update_read_ccg_device_info(
		struct cyccg *cyccg, struct ccg_update_info *update_info)
{
	struct hpi_device_mode device_mode;
	int err;

	fw_vdbg("<<<< enter\n");
	err = hpi_read_device_mode(cyccg, &device_mode);
	if (err) {
		fw_err("failed to read device_mode, %d\n", err);
		return err;
	}

	memcpy(&update_info->ccg_info, &cyccg->ccg_info,
		sizeof(struct ccg_info));
	memcpy(&update_info->ccg_fw_vers, &cyccg->ccg_state.ccg_fw_vers,
		sizeof(struct hpi_ccg_fw_version));
	update_info->running_mode = cyccg->ccg_state.running_mode;

	return 0;
}

/* The name string must be NULL terminated and end with ".cybin". */
static char *cyccg_update_fw_name_check_and_trim(char *name)
{
	char *next = name;
	char *fw_name_ext = NULL;

	if (!name || strlen(name) <= strlen(CCG_FW_IMAGE_NAME_EXT))
		return NULL;

	while (next) {
		fw_name_ext = strstr(next, CCG_FW_IMAGE_NAME_EXT);
		next = NULL;
		if (fw_name_ext) {
			next = fw_name_ext + strlen(CCG_FW_IMAGE_NAME_EXT);
			if (*next == '\n' || *next == '\r')
				*next = '\0';
			if (*next == '\0')
				break;
		}
	}

	if (fw_name_ext && strlen(name) < NAME_MAX)
		return name;
	return NULL;
}

static int cyccg_update_fw_image_names_parse(struct cyccg *cyccg,
		struct ccg_update_info *update_info,
		bool auto_update, const char *buf, size_t count)
{
	struct ccg_info *ccg_info = &update_info->ccg_info;
	enum ccg_type ccg_type = update_info->ccg_type;
	struct cybin_fw_image *image_info;
	int fw_names_count;
	char **fw_names;
	char *fw_name;
	int i;

	fw_vdbg("<<<< enter\n");
	fw_dbg("auto_update=%s\n", auto_update ? "true" : "false");
	/* Do driver internal auto firmware update. */
	if (auto_update) {
		switch (ccg_info->ccg_ver) {
		case CCG1:
			image_info = &update_info->fw_images[0];
			if (ccg_type == CCG_CABLE)
				strcpy(image_info->name,
						CCG1_CABLE_FW_IMAGE_NAME);
			else if (ccg_type == CCG_POWER_ADAPTER)
				strcpy(image_info->name, CCG1_PA_FW_IMAGE_NAME);
			else
				strcpy(image_info->name, CCG1_FW_IMAGE_NAME);

			break;
		case CCG2:
			image_info = &update_info->fw_images[0];
			if (ccg_type == CCG_CABLE)
				strcpy(image_info->name,
						CCG2_CABLE_FW_IMAGE_NAME);
			else if (ccg_type == CCG_POWER_ADAPTER)
				strcpy(image_info->name, CCG2_PA_FW_IMAGE_NAME);
			else
				strcpy(image_info->name, CCG2_FW_IMAGE_NAME);

			break;
		case CCG3:
			if (ccg_type == CCG_CABLE) {
				image_info = &update_info->fw_images[0];
				strcpy(image_info->name,
						CCG3_CABLE_FW_IMAGE1_NAME);
				image_info = &update_info->fw_images[1];
				strcpy(image_info->name,
						CCG3_CABLE_FW_IMAGE2_NAME);
			} else if (ccg_type == CCG_POWER_ADAPTER) {
				image_info = &update_info->fw_images[0];
				strcpy(image_info->name,
						CCG3_PA_FW_IMAGE1_NAME);
				image_info = &update_info->fw_images[1];
				strcpy(image_info->name,
						CCG3_PA_FW_IMAGE2_NAME);
			} else {
				image_info = &update_info->fw_images[0];
				strcpy(image_info->name, CCG3_FW_IMAGE1_NAME);
				image_info = &update_info->fw_images[1];
				strcpy(image_info->name, CCG3_FW_IMAGE2_NAME);
			}

			break;
		case CCG4:
			if (ccg_type == CCG_CABLE) {
				image_info = &update_info->fw_images[0];
				strcpy(image_info->name,
						CCG4_CABLE_FW_IMAGE1_NAME);
				image_info = &update_info->fw_images[1];
				strcpy(image_info->name,
						CCG4_CABLE_FW_IMAGE2_NAME);
			} else if (ccg_type == CCG_POWER_ADAPTER) {
				image_info = &update_info->fw_images[0];
				strcpy(image_info->name,
						CCG4_PA_FW_IMAGE1_NAME);
				image_info = &update_info->fw_images[1];
				strcpy(image_info->name,
						CCG4_PA_FW_IMAGE2_NAME);
			} else {
				image_info = &update_info->fw_images[0];
				strcpy(image_info->name, CCG4_FW_IMAGE1_NAME);
				image_info = &update_info->fw_images[1];
				strcpy(image_info->name, CCG4_FW_IMAGE2_NAME);
			}

			break;
		default:
			fw_err("unsupport ccg_ver = %d\n", ccg_info->ccg_ver);
			return -EINVAL;
		}

		return 0;
	}

	/* The firmware image names input from sysfs update_fw interface. */
	if (!buf || count <= strlen(CCG_FW_IMAGE_NAME_EXT) ||
			count > ((NAME_MAX + 1) * CCG_MAX_FLASH_MACROS)) {
		fw_err("invalid file name(s) length=%zu\n", count);
		return -EINVAL;
	}

	fw_names = argv_split(GFP_KERNEL, buf, &fw_names_count);
	if (!fw_names || !fw_names_count) {
		argv_free(fw_names);
		fw_err("failed to split input fw names\n");
		return -EINVAL;
	}


	fw_names_count = min(fw_names_count, CCG_MAX_FLASH_MACROS);
	for (i = 0; i < fw_names_count; i++) {
		image_info = &update_info->fw_images[i];
		fw_name = cyccg_update_fw_name_check_and_trim(fw_names[i]);
		if (!fw_name)
			continue;

		strcpy(image_info->name, fw_name);
		fw_dbg("get fw image name: %s\n", fw_name);
	}

	argv_free(fw_names);
	fw_vdbg(">>>> exit\n");
	return 0;
}


static int cyccg_update_request_firmware(struct cyccg *cyccg,
				  const struct firmware **fw_p, char *name)
{
	struct device *dev = cyccg->dev;
	int retries = 3;
	int err = -ENOENT;

	do {
		err = request_firmware(fw_p, name, dev);
		if (!err)
			break;

		/*
		 * Wait for the firmware load modules ready when
		 * CONFIG_FW_LOADER_USER_HELPER is not set and the
		 * cyccg driver modules is compiled into the kernel.
		 * Note, normally, the firmware image and the table image
		 * should be always there in the system, such as under
		 * /etc/firmware directory.
		 */
		if (err == -ENOENT)
			msleep(1000);
	} while (--retries);

	fw_dbg("request_firmware, %d\n", err);
	return err;
}

static int cyccg_update_flash_macro_info_parse(struct cybin_fw_image *fw_image,
		struct ccg_flash_macro_info *flash_macro_info, int index)
{
	struct cybin_record *first_record;
	struct cybin_record *second_last_record;
	struct cybin_record *last_metadata_record;
	u16 first_row_num;
	u16 second_last_row_num;
	u16 last_metadata_row_num;
	int config_row_count;
	int image_row_count;

	fw_vdbg("<<<< enter, index=%d\n", index);
	first_record = cybin_get_cybin_record(fw_image);
	last_metadata_record = cybin_get_metadata_record(fw_image);
	second_last_record =
		cybin_get_prev_record(fw_image, last_metadata_record);
	if (!first_record || !second_last_record || !last_metadata_record) {
		fw_dbg("invalid image format\n");
		return -EINVAL;
	}

	first_row_num = be16_to_cpu(first_record->record_num);
	second_last_row_num = be16_to_cpu(second_last_record->record_num);
	last_metadata_row_num = be16_to_cpu(last_metadata_record->record_num);
	image_row_count = cybin_calculate_total_record_num(fw_image);
	config_row_count = flash_macro_info->config_end_row -
				flash_macro_info->config_start_row + 1;
	if (image_row_count < config_row_count) {
		fw_dbg("invalid image size, %zu\n", fw_image->size);
		return -EINVAL;
	}

	/*
	 * Configuration table image won't contain metadata flash row data.
	 * The configuration table image data can be located in the head of the
	 * FW image, or embedded in the fw image, it won't be put in the end of
	 * the FW image.
	 */
	if (flash_macro_info->config_end_row == last_metadata_row_num) {
		if (first_row_num == flash_macro_info->config_start_row) {
			fw_dbg("only config table image\n");
			fw_image->image_type = CYBIN_IMAGE_TYPE_CONFIG;
		} else if (first_row_num == flash_macro_info->start_row &&
				first_row_num <=
					flash_macro_info->config_start_row) {
			fw_dbg("config table image and customer info\n");
			fw_image->image_type = CYBIN_IMAGE_TYPE_CONFIG;
		} else {
			fw_dbg("invalid config start row, not match\n");
			return -EINVAL;
		}
	} else if (flash_macro_info->metadata_row == last_metadata_row_num &&
			flash_macro_info->start_row == first_row_num) {
		if (second_last_row_num > flash_macro_info->end_row) {
			fw_dbg("the image row range not match\n");
			return -EINVAL;
		}

		if (first_row_num <= flash_macro_info->config_start_row) {
			fw_dbg("config table with app/data image\n");
			fw_image->image_type = CYBIN_IMAGE_TYPE_CONFIG_FW;
		} else if (first_row_num > flash_macro_info->config_end_row) {
			fw_dbg("only app/data image\n");
			fw_image->image_type = CYBIN_IMAGE_TYPE_FW;
		} else {
			fw_dbg("invalid config row range, not match\n");
			return -EINVAL;
		}
	} else if (flash_macro_info->metadata_row == last_metadata_row_num &&
			flash_macro_info->config_start_row == first_row_num) {
		if (second_last_row_num > flash_macro_info->end_row) {
			fw_dbg("the image row range not match\n");
			return -EINVAL;
		}

		if ((flash_macro_info->start_row <= first_row_num) ||
				flash_macro_info->start_row >
					flash_macro_info->config_end_row) {
			fw_dbg("config table with app/data image\n");
			fw_image->image_type = CYBIN_IMAGE_TYPE_CONFIG_FW;
		} else {
			fw_dbg("invalid app start row num, not match\n");
			return -EINVAL;
		}
	} else {
		fw_dbg("not match\n");
		return -EINVAL;
	}

	fw_image->macro_info = flash_macro_info;
	if (fw_image->ccg_ver <= CCG2) {
		fw_image->fw_type = CCG_FW_MODE_TYPE_APP;
	} else {
		/*
		 * For CCG3/CCG4 flash macro info array,
		 * The Image-1 macro info exists in 0,2,4,... index;
		 * The Image-2 macro info exists in 1,3,5,... index.
		 */
		if ((index % 2) == 0)
			fw_image->fw_type = CCG_FW_MODE_TYPE_FW_IMAGE1;
		else
			fw_image->fw_type = CCG_FW_MODE_TYPE_FW_IMAGE2;
	}

	fw_dbg("fw_image->ccg_ver = CCG%d\n", (int)fw_image->ccg_ver);
	fw_dbg("macro_info->row_size = %u\n", flash_macro_info->row_size);
	fw_dbg("macro_info->start_row = 0x%04x\n",
		flash_macro_info->start_row);
	fw_dbg("macro_info->end_row = 0x%04x\n", flash_macro_info->end_row);
	fw_dbg("macro_info->metadata_row = 0x%04x\n",
		flash_macro_info->metadata_row);
	fw_dbg("macro_info->config_start_row = 0x%04x\n",
		flash_macro_info->config_start_row);
	fw_dbg("macro_info->config_end_row = 0x%04x\n",
		flash_macro_info->config_end_row);
	fw_dbg("macro_info->fw_info_row = 0x%04x\n",
		flash_macro_info->fw_info_row);
	fw_dbg("macro_info->fw_info_row_offset = 0x%04x\n",
		flash_macro_info->fw_info_row_offset);
	fw_dbg("fw_image->image_type = %d\n", (int)fw_image->image_type);
	fw_dbg("fw_image->fw_type = %d\n", (int)fw_image->fw_type);

	return 0;
}

static int cyccg_update_check_fw_image(struct ccg_update_info *update_info,
		struct cybin_fw_image *fw_image)
{
	enum hpi_version hpi_ver = update_info->ccg_info.hpi_ver;
	struct cybin_record *record = cybin_get_cybin_record(fw_image);
	struct ccg_flash_macro_info *macro_info = fw_image->macro_info;
	struct hpi_image_version image_ver;
	struct hpi_config_table_header *table_header;
	u16 flahs_row_size = update_info->ccg_info.flash_row_size;
	u16 record_num;
	u16 record_data_size;
	u8 record_checksum;
	u8 calculated_checksum;
	u8 config_table_checksum;
	size_t config_table_size;

	fw_vdbg("<<<< enter, fw_name=%s\n", fw_image->name);
	if (!record || !macro_info) {
		fw_err("invalid image data or macro info\n");
		return -EINVAL;
	}

	while (record) {
		/* Check each record row data is valid. */
		record_num = be16_to_cpu(record->record_num);
		record_data_size = be16_to_cpu(record->record_data_size);
		record_checksum = cybin_record_get_checksum(record);
		calculated_checksum = cybin_record_calculate_checksum(record);

		if (!cybin_is_valid_record_num(record_num, macro_info) ||
				record->colon != ':' ||
				record->flash_array_id != 0 ||
				flahs_row_size != record_data_size ||
				record_checksum != calculated_checksum) {
			fw_dbg("failed on record data check\n");
			fw_dbg("flash_array_id=%u\n",
				record->flash_array_id);
			fw_dbg("record_num=%u\n", record_num);
			fw_dbg("record_data_size=%u, expect size=%u\n",
				record_data_size, flahs_row_size);
			fw_dbg("record_checksum=%u\n", record_checksum);
			fw_dbg("calculated_checksum=%u\n",
				calculated_checksum);
			return -EINVAL;
		}

		/* Get the firmware image version. */
		if (macro_info->fw_info_row == record_num) {
			cybin_image_version_parse(&image_ver,
				&record->record_data[
					macro_info->fw_info_row_offset]);
		}

		/* Check the configuration table data if it exists. */
		if (macro_info->config_start_row == record_num) {
			table_header = (struct hpi_config_table_header *)
						record->record_data;
			fw_image->table_checksum = table_header->table_checksum;
			fw_image->flash_checksum = table_header->flash_checksum;
			config_table_size = (macro_info->config_end_row -
					macro_info->config_start_row + 1) *
							macro_info->row_size;

			if (le16_to_cpu(table_header->signature) !=
					HPI_CONFIG_TABLE_SIGNATURE) {
				fw_dbg("invalid config signature=0x%04x\n",
					le16_to_cpu(table_header->signature));
				fw_dbg("expected signature=0x%04x\n",
					HPI_CONFIG_TABLE_SIGNATURE);
				return -EINVAL;
			}

			if (HPI(CONFIG_TABLE_VERSION, hpi_ver) >
					le16_to_cpu(table_header->table_ver)) {
				fw_dbg("invalid config table version=0x%04x\n",
					le16_to_cpu(table_header->table_ver));
				fw_dbg("expected table version>=0x%04x\n",
					HPI(CONFIG_TABLE_VERSION, hpi_ver));
				return -EINVAL;
			}

			if (HPI(CONFIG_TABLE_SIZE, hpi_ver) !=
					le16_to_cpu(table_header->table_size) ||
					HPI(CONFIG_TABLE_SIZE, hpi_ver) !=
						config_table_size) {
				fw_dbg("invalid config table size=0x%04x\n",
					le16_to_cpu(table_header->table_size));
				fw_dbg("expected table size=0x%04x\n",
					HPI(CONFIG_TABLE_SIZE, hpi_ver));
				return -EINVAL;
			}
		}

		if (record_num >= macro_info->config_start_row &&
				record_num <= macro_info->config_end_row) {
			config_table_checksum =
				cybin_calculate_config_table_checksum(
							record, macro_info);
			if (macro_info->config_end_row == record_num &&
					config_table_checksum !=
						fw_image->table_checksum) {
				fw_dbg("config table checksum not match\n");
				fw_dbg("calc checksum=%u, expected=%u\n",
					config_table_checksum,
					fw_image->table_checksum);
				return -EINVAL;
			}
		}

		record = cybin_get_next_cybin_record(fw_image, record);
	}

	memcpy(&fw_image->image_ver,
		&image_ver, sizeof(struct hpi_image_version));
	fw_dbg("image base version: %u.%u.%u.%u\n",
		image_ver.base.major, image_ver.base.minor,
		image_ver.base.patch_ver, image_ver.base.build_number);
	fw_dbg("image app version: %u.%u,%u,%c%c\n",
		image_ver.app.major, image_ver.app.minor,
		image_ver.app.external_circuit_ver,
		image_ver.app.name[0], image_ver.app.name[1]);
	return 0;
}

static int cyccg_update_load_check_fw_image(struct cyccg *cyccg,
		struct ccg_update_info *update_info, bool auto_update)
{
	struct ccg_info *ccg_info = &update_info->ccg_info;
	struct ccg_flash_macro_info *flash_macro_info_list;
	struct ccg_flash_macro_info *detect_macro_info;
	struct cybin_fw_image *fw_image;
	struct cybin_image_head *image_head;
	struct cybin_record *record;
	enum ccg_fw_mode_type expected_fw_mode;
	struct cybin_fw_image *tmp_fw_image = NULL;
	int i, j;
	int err;

	fw_vdbg("<<<< enter\n");

	/*
	 * For dual FW mode support CCG3/CCG4 device, the FW image only can be
	 * updated to the FW area that current not running, so the target FW
	 * area and image for updating must be different from the currently
	 * running mode. For legacy mode support CCG1/CCG2 device, the FW
	 * updating applied all through bootloader, and only has one FW image
	 * area, so the FW arar and image for updating must always be
	 * CCG_FW_MODE_TYPE_APP (equals to CCG_FW_MODE_TYPE_FW_IMAGE1).
	 */
	if (ccg_info->ccg_ver >= CCG3 &&
			update_info->running_mode == CCG_FW_MODE_TYPE_FW_IMAGE1)
		expected_fw_mode = CCG_FW_MODE_TYPE_FW_IMAGE2;
	else
		expected_fw_mode = CCG_FW_MODE_TYPE_FW_IMAGE1;

	for (i = 0; i < CCG_MAX_FLASH_MACROS; i++) {
		fw_image = &update_info->fw_images[i];
		if (!strlen(fw_image->name))
			continue;

		fw_dbg("try to load and check fwimage[%d]: %s\n",
			i, fw_image->name);

		/* Load the FW image data based on the FW image name. */
		err = cyccg_update_request_firmware(cyccg,
				&fw_image->fw, fw_image->name);
		if (err) {
			fw_dbg("failed to load fw image: %s, %d\n",
				fw_image->name, err);
			continue;
		}

		fw_image->image = fw_image->fw->data;
		fw_image->size = fw_image->fw->size;

		/* The silicon Id must be match. */
		image_head = (struct cybin_image_head *)fw_image->image;
		fw_image->silicon_id =
			be16_to_cpu(image_head->silicon.silicon_id);
		fw_image->ccg_ver =
			ccg_silicon_id_to_ccg_version(fw_image->silicon_id);
		if (ccg_info->silicon_id != fw_image->silicon_id ||
				fw_image->ccg_ver == CCG_UNKNOWN) {
			fw_dbg("%s, CCG:0x%04x <> Image:0x%04x\n",
				"silicon id mismatch or unsupport",
				ccg_info->silicon_id, fw_image->silicon_id);
			err = -EINVAL;
			continue;
		}

		/* Check the basic FW image format is valid. */
		record = cybin_get_cybin_record(fw_image);
		if (!record) {
			fw_dbg("invalid image format\n");
			err = -EINVAL;
			continue;
		}

		/* Get the flash macro info list baed on CCG version. */
		switch (fw_image->ccg_ver) {
		case CCG1:
			flash_macro_info_list = ccg1_flash_macro_info_list;
			break;
		case CCG2:
			flash_macro_info_list = ccg2_flash_macro_info_list;
			break;
		case CCG3:
			flash_macro_info_list = ccg3_flash_macro_info_list;
			break;
		case CCG4:
			flash_macro_info_list = ccg4_flash_macro_info_list;
			break;
		default:
			fw_dbg("target image for unknown CCG\n");
			err = -EINVAL;
			continue;
		}

		/* Found the matched flash macro info for the FW image. */
		for (j = 0; flash_macro_info_list[j].row_size; j++) {
			if (be16_to_cpu(record->record_data_size) !=
					flash_macro_info_list[j].row_size)
				continue;

			detect_macro_info = &flash_macro_info_list[j];
			err = cyccg_update_flash_macro_info_parse(fw_image,
						detect_macro_info, j);
			if (err) {
				fw_dbg("invalid image or not match\n");
				continue;
			}

			fw_dbg("valid image[%d]=%s found\n",
				i, fw_image->name);
			break;	/* FW image matchs a macro info found. */
		}

		/* Check the FW image data and checksum are valid. */
		err = cyccg_update_check_fw_image(update_info, fw_image);
		if (err) {
			fw_dbg("failed to check fw image, %d\n", err);
			continue;
		}

		/* Check the FW image is suitable for target CCG updating. */
		if (expected_fw_mode == fw_image->fw_type) {
			update_info->fw_image = fw_image;
			fw_vdbg("expected image=%s for updating found\n",
				fw_image->name);
			break;
		}

		if (!tmp_fw_image)
			tmp_fw_image = fw_image;
	}

	if (!update_info->fw_image) {
		fw_dbg("no valid FW image found\n");
		if (auto_update || !tmp_fw_image ||
				ccg_info->flash_mode == CCG_LEGACY_BOOT_MODE) {
			fw_err("no expected fw_type=%d image found, exit\n",
				expected_fw_mode);
			return -EINVAL;
		}

		/*
		 * Try to jump ALT FW later to force the FW update.
		 * This function only works when do manual force update, and
		 * for DUAL_FW support CCG device.
		 */
		update_info->fw_image = tmp_fw_image;
		fw_dbg("expected_fw_mode=%d\n", expected_fw_mode);
		fw_dbg("use fw_type=%d image=%s instead\n",
			tmp_fw_image->fw_type, tmp_fw_image->name);
	}

	return 0;
}

static int cyccg_update_bootloader_enter(
		struct cyccg *cyccg, struct ccg_update_info *update_info)
{
	struct hpi_device_mode device_mode;
	u16 bl_last_row;
	int err;

	fw_vdbg("<<<< enter\n");
	/* When disable DP ports failed, try to continue reset CCG direcetly. */
	err = hpi_disable_all_dpport_sync(cyccg);
	if (err)
		fw_err("failed to disable PD ports, %d\n", err);

	err = hpi_jump_to_boot_sync(cyccg);
	if (err)
		fw_err("failed to enter BL mode, %d\n", err);

	err = hpi_read_device_mode(cyccg, &device_mode);
	if (err) {
		fw_err("failed to read device_mode, %d\n", err);
		return err;
	}

	update_info->running_mode = device_mode.running_mode;
	if (device_mode.running_mode != CCG_FW_MODE_TYPE_BOOTLAODER) {
		fw_err("running_mode=%d, failed to enter BL mode\n",
			update_info->running_mode);
		return -EFAULT;
	}

	/* read and update bootloader last row value into ccg_info. */
	err = hpi_read_boot_loader_last_row(cyccg, &bl_last_row);
	if (!err)
		cyccg->ccg_info.bl_last_row_num = bl_last_row;

	return 0;
}

static int cyccg_update_jump_to_alt_fw(
		struct cyccg *cyccg, struct ccg_update_info *update_info)
{
	struct hpi_device_mode device_mode;
	int err;

	fw_vdbg("<<<< enter\n");

	/* When disable DP ports failed, try to continue reset CCG direcetly. */
	err = hpi_disable_all_dpport_sync(cyccg);
	if (err)
		fw_err("failed to disable PD ports, %d\n", err);

	err = hpi_jump_to_alt_fw_sync(cyccg);
	if (err) {
		fw_err("failed to jump to alt fw, %d\n", err);
		return err;
	}

	err = hpi_read_device_mode(cyccg, &device_mode);
	if (err) {
		fw_err("failed to read device_mode, %d\n", err);
		return err;
	}

	update_info->running_mode = device_mode.running_mode;
	fw_vdbg(">>>> exit\n");
	return 0;
}


static int cyccg_update_enter_flashing_mode(
		struct cyccg *cyccg, struct ccg_update_info *update_info)
{
	struct ccg_info *ccg_info = &update_info->ccg_info;
	struct hpi_device_mode device_mode;
	int err;

	fw_vdbg("<<<< enter\n");
	/* Get the latest CCG running_mode.*/
	err = hpi_read_device_mode(cyccg, &device_mode);
	if (err) {
		fw_err("failed to read device_mode, %d\n", err);
		return err;
	}
	update_info->running_mode = device_mode.running_mode;

	if (ccg_info->flash_mode == CCG_LEGACY_BOOT_MODE) {
		/* Verify the device running in bootloader mode. */
		if (update_info->running_mode != CCG_FW_MODE_TYPE_BOOTLAODER) {
			err = cyccg_update_bootloader_enter(cyccg, update_info);
			if (err) {
				fw_err("failed to enter BL mode, %d\n", err);
				return err;
			}
		}
	} else {
		if (update_info->running_mode ==
				update_info->fw_image->fw_type) {
			fw_dbg("running_mode=%d, fw_type=%d, not match\n",
				update_info->running_mode,
				update_info->fw_image->fw_type);
			fw_dbg("jump to ALT FW required\n");

			err = cyccg_update_jump_to_alt_fw(cyccg, update_info);
			if (err) {
				fw_err("failed to jump to alt fw, %d\n", err);
				return err;
			}

			if (update_info->running_mode ==
					update_info->fw_image->fw_type) {
				fw_err("running_mode=%d fw_type=%d, mismatch\n",
					update_info->running_mode,
					update_info->fw_image->fw_type);
				return -EINVAL;
			}

			fw_dbg("running_mode=%d, update to fw_type=%d\n",
				update_info->running_mode,
				update_info->fw_image->fw_type);
		}
	}

	/* Enter flashing mode. */
	err = hpi_enter_flashing_mode_sync(cyccg);
	if (err) {
		fw_err("failed to enter BL flashing mode, %d\n", err);
		return err;
	}

	return 0;
}

static int cyccg_update_write_fw_image(
		struct cyccg *cyccg, struct ccg_update_info *update_info)
{
	struct cybin_fw_image *fw_image = update_info->fw_image;
	struct cybin_record *record;
	u8 tmp_data[HPI_MAX_FLASH_RW_REG_SIZE];
	u16 record_num;
	u16 record_data_size;
	int err;

	fw_vdbg("<<<< enter\n");
	/* Clear the firmware metadata in flash memory. */
	if (update_info->fw_image->image_type > CYBIN_IMAGE_TYPE_CONFIG) {
		fw_dbg("clear metadata\n");
		record = cybin_get_metadata_record(fw_image);
		record_num = be16_to_cpu(record->record_num);
		record_data_size = be16_to_cpu(record->record_data_size);
		memset(tmp_data, 0, sizeof(tmp_data));
		err = hpi_flash_row_write_sync(cyccg, record_num,
				tmp_data, record_data_size);
		if (err) {
			fw_err("failed to write metadata row, %d\n", err);
			return err;
		}
	}

	/* Write each flash row to the device. */
	record = cybin_get_cybin_record(fw_image);
	while (record) {
		record_num = be16_to_cpu(record->record_num);
		record_data_size = be16_to_cpu(record->record_data_size);
		fw_dbg("write flash record row=%u, size=%u\n",
			record_num, record_data_size);
		err = hpi_flash_row_write_sync(cyccg, record_num,
				record->record_data, record_data_size);
		if (err) {
			fw_err("failed to write flash row=%u, %d\n",
				record_num, err);
			return err;
		}

		record = cybin_get_next_cybin_record(fw_image, record);
	}

	return 0;
}

static int cyccg_update_validate_fw_image(
		struct cyccg *cyccg, struct ccg_update_info *update_info)
{
	enum ccg_fw_mode_type fw_type = update_info->fw_image->fw_type;
	int err;

	err = hpi_validate_fw_sync(cyccg, fw_type);
	if (err) {
		fw_err("validate failed, invalid FW, %d\n", err);
		return err;
	}

	fw_dbg("validate fw image success\n");
	return 0;
}

static void cyccg_update_free_ccg_update_info(
		struct cyccg *cyccg, struct ccg_update_info *update_info)
{
	struct device *dev = cyccg->dev;
	struct cybin_fw_image *fw_image;
	int i;

	fw_vdbg("<<<< enter\n");
	for (i = 0; i < CCG_MAX_FLASH_MACROS; i++) {
		fw_image = &update_info->fw_images[i];
		fw_image->image = NULL;
		fw_image->macro_info = NULL;
		if (fw_image->fw)
			release_firmware(fw_image->fw);
	}

	devm_kfree(dev, update_info);
}

static int cyccg_update_reset_and_init(struct cyccg *cyccg, bool force_reset)
{
	int err;

	fw_dbg("force_reset=%d\n", force_reset);
	/* When disable DP ports failed, try to continue reset CCG direcetly. */
	if (cyccg->ccg_info.hpi_ver != HPI_VERSION_1) {
		err = hpi_disable_all_dpport_sync(cyccg);
		if (err)
			fw_err("failed to disable DP ports, %d\n", err);
	}

	err = cyccg_device_reset_and_init(cyccg, force_reset);
	if (err) {
		fw_err("failed to reset and init CCG device, %d\n", err);
		fw_dbg("CCG running_mode=%d\n", cyccg->ccg_state.running_mode);
		return err;
	}

	return 0;
}

int cyccg_update_do_fw_update(struct cyccg *cyccg,
		enum ccg_type ccg_type, bool auto_update,
		const char *buf, size_t count)
{
	struct ccg_update_info *update_info;
	struct hpi_app_version *app_ver;
	bool force_reset = false;
	u16 image_ver;
	u16 ccg_ver;
	int err;
	int ret;

	fw_dbg("auto_update=%d, buf=%s\n", auto_update, buf ?: "NULL");
	update_info = cyccg_update_allocate_ccg_update_info(cyccg);
	if (!update_info)
		return -ENOMEM;
	update_info->ccg_type = ccg_type;

	err = cyccg_update_read_ccg_device_info(cyccg, update_info);
	if (err) {
		fw_err("failed to read CCG device basic info, %d\n", err);
		goto out;
	}

	err = cyccg_update_fw_image_names_parse(cyccg, update_info,
						auto_update, buf, count);
	if (err) {
		fw_err("failed to parse fw image name(s), %d\n", err);
		goto out;
	}

	err = cyccg_update_load_check_fw_image(cyccg, update_info, auto_update);
	if (err) {
		fw_err("failed to load and check FW image, %d\n", err);
		goto out;
	}

	if (auto_update) {
		app_ver = &update_info->fw_image->image_ver.app;
		image_ver = app_ver->major << 8 | app_ver->minor;
		fw_dbg("FW image version: %u.%u,%u,%c%c\n",
			app_ver->major, app_ver->minor,
			app_ver->external_circuit_ver,
			app_ver->name[0], app_ver->name[1]);
		app_ver = &update_info->ccg_fw_vers.fw1_app.app;
		ccg_ver = app_ver->major << 8 | app_ver->minor;
		fw_dbg("CCG device version: %u.%u,%u,%c%c\n",
			app_ver->major, app_ver->minor,
			app_ver->external_circuit_ver,
			app_ver->name[0], app_ver->name[1]);
		if (update_info->running_mode != CCG_FW_MODE_TYPE_BOOTLAODER &&
				image_ver <= ccg_ver) {
			fw_dbg("FW image is old, skip. Return early\n");
			err = 0;
			goto out;
		}
	}

	force_reset = true;
	err = cyccg_update_enter_flashing_mode(cyccg, update_info);
	if (err) {
		fw_err("failed to load and check FW image, %d\n", err);
		goto out;
	}

	err = cyccg_update_write_fw_image(cyccg, update_info);
	if (err) {
		fw_err("failed to write FW image, %d\n", err);
		goto out;
	}

	err = cyccg_update_validate_fw_image(cyccg, update_info);
	if (err) {
		fw_err("failed to validate FW image, %d\n", err);
		goto out;
	}

out:
	ret = cyccg_update_reset_and_init(cyccg, force_reset);
	if (ret)
		fw_err("failed to reset and re-init CCG, %d\n", ret);

	cyccg_update_free_ccg_update_info(cyccg, update_info);
	fw_dbg("fw update done, %d\n", err ?: ret);
	return err ?: ret;
}

static inline int cyccg_auto_fw_update_thread_handler(struct cyccg *cyccg)
{
	int err;

	fw_vdbg("<<<< enter\n");
	err = cyccg_update_do_fw_update(cyccg,
				CCG_NOTEBOOK_MOBILE_MONITOR, true, NULL, 0);
	return err;
}

/*
 * Functions of firmware update through CC.
 */
static inline void cyccg_port_cc_s_vdm_header_init(
		union s_vdm_header *vdm_header,
		enum vdm_sid_command vdm_sid_cmd, u16 svid, u8 mode_obj_pos)
{
	memset(vdm_header, 0, sizeof(union s_vdm_header));
	vdm_header->sid_cmd = vdm_sid_cmd;
	vdm_header->vdm_cmd_type = VDM_CMD_TYPE_INITIATOR;
	vdm_header->obj_position = mode_obj_pos & 0x07;
	vdm_header->s_vdm_ver = STRUCTURED_VMD_VERSION_1;
	vdm_header->vdm_type = VDM_TYPE_STRUCTURED;
	vdm_header->svid = svid;

	put_unaligned_le32(vdm_header->vdo, &vdm_header->vdo);
}

static inline void cyccg_port_cc_u_vdm_header_init(
		union u_vdm_header *vdm_header,
		enum vdm_cy_command vdm_cy_cmd, u16 svid, u32 seq_num)
{
	memset(vdm_header, 0, sizeof(union u_vdm_header));
	vdm_header->svid = svid;
	vdm_header->vdm_type = VDM_TYPE_UNSTRUCTURED;

	vdm_header->ccg_cmd = vdm_cy_cmd;
	vdm_header->seq_num = seq_num;
	vdm_header->cmd_type = VDM_CMD_TYPE_INITIATOR;

	put_unaligned_le32(vdm_header->vdo, &vdm_header->vdo);
}

static int cyccg_port_cc_send_s_vdm_command(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		enum vdm_sid_command sid_cmd, u16 svid, u8 mode_obj_pos,
		void *resp_buf, size_t *resp_buf_size)
{
	u8 vdm_data[HPI_MAX_VDM_MSG_SIZE];
	struct hpi_vdm_message *vdm_resp = (struct hpi_vdm_message *)vdm_data;
	union vdm_msg_header msg_header;
	union s_vdm_header vdm_header;
	size_t vdm_resp_size = sizeof(vdm_data);
	int err;

	cc_port_vdbg("<<<< enter\n", port);
	cyccg_port_cc_s_vdm_header_init((union s_vdm_header *)vdm_data,
					sid_cmd, svid, mode_obj_pos);

	err = hpi_port_send_vdm_data_sync(port, vdm_mode,
			vdm_data, sizeof(union s_vdm_header),
			vdm_resp, &vdm_resp_size);
	if (err) {
		cc_port_err("failed to send s_vdm_command, %d\n", port, err);
		return err;
	}

	/*
	 * The return data is the whole HPI VDM message data.
	 * Detail of the data format can refer to the commensts for the
	 * struct hpi_vdm_message definition.
	 */
	if (resp_buf && resp_buf_size && *resp_buf_size) {
		*resp_buf_size = min(*resp_buf_size, vdm_resp_size);
		memcpy(resp_buf, vdm_data, *resp_buf_size);
	}

	msg_header.msg_header =
		get_unaligned_le16(&vdm_resp->msg_header.msg_header);
	vdm_header.vdo =
		get_unaligned_le32(&vdm_resp->s_vdm_header.vdo);
	if (vdm_resp_size != HPI_VDM_RESP_SIZE_BY_VDO_NUM(
					msg_header.data_objs) ||
			vdm_resp_size < HPI_VDM_RESP_SIZE_BY_VDO_NUM(1) ||
			vdm_resp->sop_type != (u8)vdm_mode ||
			vdm_header.sid_cmd != sid_cmd ||
			vdm_header.svid != svid ||
			vdm_header.obj_position != mode_obj_pos ||
			vdm_header.vdm_type != VDM_TYPE_STRUCTURED ||
			vdm_header.vdm_cmd_type != VDM_CMD_TYPE_RESP_ACK) {
		cc_port_err("invalid VDM response data:\n", port);
		cc_port_err("data_objs=%u, expected_data_objs>=%u\n",
			port, msg_header.data_objs, 1);
		cc_port_err("resp_size=%zu, expected_size=%u\n",
			port, vdm_resp_size, HPI_VDM_RESP_SIZE_BY_VDO_NUM(1));
		cc_port_err("sop_type=%u, expected_sop_type=%u\n",
			port, vdm_resp->sop_type, (u8)vdm_mode);
		cc_port_err("sid_cmd=%u, expected_sid_cmd=%u\n",
			port, vdm_header.sid_cmd, sid_cmd);
		cc_port_err("svid=0x%04x, expected_svid=0x%04x\n",
			port, vdm_header.svid, svid);
		cc_port_err("obj_pos=%u, expected_obj_pos=%u\n",
			port, vdm_header.obj_position, mode_obj_pos);
		cc_port_err("vdm_type=%u, expected_vdm_type=%u\n",
			port, vdm_header.vdm_type, VDM_TYPE_STRUCTURED);
		cc_port_err("cmd_type=%u, expected_cmd_type=%u\n",
			port, vdm_header.vdm_cmd_type, VDM_CMD_TYPE_RESP_ACK);
		if (vdm_header.vdm_cmd_type == VDM_CMD_TYPE_RESP_NACK)
			return -ENOTSUPP;
		return -EFAULT;
	}

	return 0;
}

/* Received data is struct vdm_resp_discover_id VDM data. */
int cyccg_port_cc_discover_identity(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		void *resp_buf, size_t *resp_buf_size)
{
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));
	if (vdm_mode == VDM_SOP_TYPE_SOP_DPRIME) {
		cc_port_err("DISCOVER_IDENTITY not support SOP''\n", port);
		return -EINVAL;
	}

	err = cyccg_port_cc_send_s_vdm_command(port, vdm_mode,
			VDM_SID_CMD_DISCOVER_IDENTITY,
			VDM_SVID_POWER_DELIVERY, 0, resp_buf, resp_buf_size);
	if (err) {
		cc_port_err("failed to do DISCOVER_IDENTITY, %d\n", port, err);
		return err;
	}

	if (resp_buf && resp_buf_size && *resp_buf_size)
		cc_port_dump("discover_identity data (%zu):\n", port,
			resp_buf, *resp_buf_size, *resp_buf_size);
	cc_port_vdbg(">>>> exit, success\n", port);
	return 0;
}

/* Received data is struct vdm_resp_discover_svid VDM data. */
int cyccg_port_cc_discover_svid(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		void *resp_buf, size_t *resp_buf_size)
{
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	if (vdm_mode == VDM_SOP_TYPE_SOP_DPRIME) {
		cc_port_err("DISCOVER_SVID not support SOP''\n", port);
		return -EINVAL;
	}

	err = cyccg_port_cc_send_s_vdm_command(port, vdm_mode,
			VDM_SID_CMD_DISCOVER_SVID,
			VDM_SVID_POWER_DELIVERY, 0, resp_buf, resp_buf_size);
	if (err) {
		cc_port_err("failed to do DISCOVER_SVID, %d\n", port, err);
		return err;
	}

	cc_port_vdbg(">>>> exit, success\n", port);
	return 0;
}

/* Received data is struct vdm_resp_discover_mode VDM data. */
int cyccg_port_cc_discover_mode(struct hpi_device *port,
		enum vdm_sop_type vdm_mode, u16 svid,
		void *resp_buf, size_t *resp_buf_size)
{
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	if (vdm_mode == VDM_SOP_TYPE_SOP_DPRIME) {
		cc_port_err("DISCOVER_MODE not support SOP''\n", port);
		return -EINVAL;
	}

	err = cyccg_port_cc_send_s_vdm_command(port, vdm_mode,
			VDM_SID_CMD_DISCOVER_MODES,
			svid, 0, resp_buf, resp_buf_size);
	if (err) {
		cc_port_err("failed to do DISCOVER_MODE, %d\n", port, err);
		return err;
	}

	cc_port_vdbg(">>>> exit, success\n", port);
	return 0;
}

int cyccg_port_cc_enter_mode(struct hpi_device *port,
		enum vdm_sop_type vdm_mode, u16 svid, u8 mode_obj_pos,
		u32 *resp_vdo)
{

	u8 vdm_data[HPI_MAX_VDM_MSG_SIZE];
	struct hpi_vdm_message *vdm_resp = (struct hpi_vdm_message *)vdm_data;
	union vdm_msg_header *msg_header = &vdm_resp->msg_header;
	size_t vdm_resp_size = sizeof(vdm_data);
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	if (mode_obj_pos < 1) {
		cc_port_err("invalid mode_obj_pos=%u\n", port, mode_obj_pos);
		return -EINVAL;
	}

	err = cyccg_port_cc_send_s_vdm_command(port, vdm_mode,
			VDM_SID_CMD_ENTER_MODE, svid, mode_obj_pos,
			vdm_resp, &vdm_resp_size);
	if (err) {
		cc_port_err("failed to do ENTER_MODE, %d\n", port, err);
		return err;
	}

	if (resp_vdo) {
		msg_header->msg_header =
			get_unaligned_le16(&msg_header->msg_header);
		if (vdm_resp_size == HPI_VDM_RESP_SIZE_BY_VDO_NUM(2) &&
				msg_header->data_objs == 2)
			*resp_vdo = get_unaligned_le32(&vdm_resp->vdo[0]);
		else
			*resp_vdo = 0;
	}

	spin_lock(&port->slock);
	port->is_in_alt_mode = true;
	spin_unlock(&port->slock);
	cc_port_vdbg(">>>> exit, success\n", port);
	return 0;
}

int cyccg_port_cc_exit_mode(struct hpi_device *port,
		enum vdm_sop_type vdm_mode, u16 svid, u8 mode_obj_pos)
{
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	if (mode_obj_pos < 1) {
		cc_port_err("invalid mode_obj_pos=%u\n", port, mode_obj_pos);
		return -EINVAL;
	}

	err = cyccg_port_cc_send_s_vdm_command(port, vdm_mode,
			VDM_SID_CMD_EXIT_MODE, svid, mode_obj_pos,
			NULL, NULL);
	if (err) {
		cc_port_err("failed to do EXIT_MODE, %d\n", port, err);
		return err;
	}

	spin_lock(&port->slock);
	port->is_in_alt_mode = false;
	spin_unlock(&port->slock);
	cc_port_vdbg(">>>> exit, success\n", port);
	return 0;
}

int cyccg_port_cc_attention(struct hpi_device *port,
		enum vdm_sop_type vdm_mode, u16 svid, u8 mode_obj_pos,
		u32 *resp_vdo)
{
	u8 vdm_data[HPI_MAX_VDM_MSG_SIZE];
	struct hpi_vdm_message *vdm_resp = (struct hpi_vdm_message *)vdm_data;
	union vdm_msg_header *msg_header = &vdm_resp->msg_header;
	size_t vdm_resp_size = sizeof(vdm_data);
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	if (mode_obj_pos < 1) {
		cc_port_err("invalid mode_obj_pos=%u\n", port, mode_obj_pos);
		return -EINVAL;
	}

	err = cyccg_port_cc_send_s_vdm_command(port, vdm_mode,
			VDM_SID_CMD_ATTENTION, svid, mode_obj_pos,
			vdm_resp, &vdm_resp_size);
	if (err) {
		cc_port_err("failed to do ATTENTION, %d\n", port, err);
		return err;
	}

	if (resp_vdo) {
		msg_header->msg_header =
			get_unaligned_le16(&msg_header->msg_header);
		if (vdm_resp_size == HPI_VDM_RESP_SIZE_BY_VDO_NUM(2) &&
				msg_header->data_objs == 2)
			*resp_vdo = get_unaligned_le32(&vdm_resp->vdo[0]);
		else
			*resp_vdo = 0;
	}

	cc_port_vdbg(">>>> exit, success\n", port);
	return 0;
}

/*
 * Cypress CCG defined Unstructured VDM command interfaces.
 */
static u8 min_data_objs_of_vdm_cy_command_response[] = {
	0, /* VDM_CY_CMD_RESERVED		= 0x00 */
	2, /* VDM_CY_CMD_GET_DEVICE_MODE	= 0x01 */
	7, /* VDM_CY_CMD_GET_DEVICE_VERSION	= 0x02 */
	2, /* VDM_CY_CMD_GET_SILICON_ID		= 0x03 */
	2, /* VDM_CY_CMD_DEVICE_RESET		= 0x04 */
	2, /* VDM_CY_CMD_JUMP_TO_BOOT		= 0x05 */
	2, /* VDM_CY_CMD_ENTER_FLASHING_MODE	= 0x06 */
	2, /* VDM_CY_CMD_SEND_DATA		= 0x07 */
	2, /* VDM_CY_CMD_FLASH_WRITE		= 0x08 */
	2, /* VDM_CY_CMD_READ_DATA		= 0x09 */
	2, /* VDM_CY_CMD_FLASH_READ		= 0x0a */
	2, /* VDM_CY_CMD_VALIDATE_FW		= 0x0b */
	2, /* VDM_CY_CMD_REASON_FOR_BOOT_MODE	= 0x0c */
	2, /* VDM_CY_CMD_GET_CHECKSUM		= 0x0d */
	3, /* VDM_CY_CMD_GET_FW_START_ADDR	= 0x0e */
	2, /* VDM_CY_CMD_SET_APP_PRIORITY	= 0x0f */
	2, /* VDM_CY_CMD_SEND_SIGNATURE		= 0x11 */
	2, /* VDM_CY_CMD_GET_BOOT_TYPE		= 0x13 */
	5, /* VDM_CY_CMD_GET_CUSTOMER_INFO	= 0x14 */
};
#define VDM_RESP_OBJS_OF_CY_CMD(_vdm_cy_cmd)	\
	(min_data_objs_of_vdm_cy_command_response[(_vdm_cy_cmd)])
#define HPI_U_VDM_RESP_SIZE(_vdm_cy_cmd)	\
	HPI_VDM_RESP_SIZE_BY_VDO_NUM(VDM_RESP_OBJS_OF_CY_CMD(_vdm_cy_cmd))

static int cyccg_port_cc_send_u_vdm_command(struct hpi_device *port,
		enum vdm_sop_type vdm_mode, enum vdm_cy_command cy_cmd,
		void *vdm_cmd_data, size_t vdm_cmd_data_size, u32 seq_num,
		void *resp_buf, size_t *resp_buf_size)
{
	u8 vdm_data[HPI_MAX_VDM_MSG_SIZE];
	struct hpi_vdm_message *vdm_resp = (struct hpi_vdm_message *)vdm_data;
	union vdm_msg_header msg_header;
	union u_vdm_header vdm_header;
	size_t vdm_resp_size = sizeof(vdm_data);
	size_t vdm_cmd_size;
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	if (vdm_cmd_data_size > VDM_MAX_DATA_SIZE ||
			vdm_cmd_data_size % VDM_VDO_OBJ_SIZE) {
		cc_port_err("invalid u_vdm command data size=%zu\n",
			port, vdm_cmd_data_size);
		return -EINVAL;
	}

	cyccg_port_cc_u_vdm_header_init((union u_vdm_header *)vdm_data,
			 cy_cmd, VDM_SVID_CYPRESS, seq_num);
	vdm_cmd_size = sizeof(union u_vdm_header);
	if (vdm_cmd_data && vdm_cmd_data_size) {
		memcpy(vdm_data + vdm_cmd_size,
			vdm_cmd_data, vdm_cmd_data_size);
		vdm_cmd_size += vdm_cmd_data_size;
	}

	err = hpi_port_send_vdm_data_sync(port, vdm_mode,
			vdm_data, vdm_cmd_size,
			vdm_resp, &vdm_resp_size);
	if (err) {
		cc_port_err("failed to run u_vdm command, %d\n", port, err);
		return err;
	}

	msg_header.msg_header =
			get_unaligned_le16(&vdm_resp->msg_header.msg_header);
	vdm_header.vdo =
			get_unaligned_le16(&vdm_resp->u_vdm_header.vdo);
	if (vdm_resp_size !=
			HPI_VDM_RESP_SIZE_BY_VDO_NUM(msg_header.data_objs) ||
			vdm_resp_size < HPI_U_VDM_RESP_SIZE(cy_cmd) ||
			vdm_resp->sop_type != (u8)vdm_mode ||
			vdm_header.ccg_cmd != cy_cmd ||
			(vdm_header.svid != VDM_SVID_CYPRESS &&
				vdm_header.svid != VDM_SVID_TERMINATING) ||
			vdm_header.vdm_type != VDM_TYPE_UNSTRUCTURED ||
			vdm_header.cmd_type != VDM_CMD_TYPE_RESP_ACK) {
		cc_port_err("invalid VDM response data:\n", port);
		cc_port_err("data_objs=%u, expected_data_objs>=%u\n",
			port, msg_header.data_objs,
			VDM_RESP_OBJS_OF_CY_CMD(cy_cmd));
		cc_port_err("resp_size=%zu, expected_size=%u\n",
			port, vdm_resp_size, HPI_U_VDM_RESP_SIZE(cy_cmd));
		cc_port_err("sop_type=%u, expected_sop_type=%u\n",
			port, vdm_resp->sop_type, (u8)vdm_mode);
		cc_port_err("cy_cmd=%u, expected_cy_cmd=%u\n",
			port, vdm_header.ccg_cmd, cy_cmd);
		cc_port_err("svid=0x%04x, expected_svid=0x%04x\n",
			port, vdm_header.svid, VDM_SVID_CYPRESS);
		cc_port_err("vdm_type=%u, expected_vdm_type=%u\n",
			port, vdm_header.vdm_type, VDM_TYPE_UNSTRUCTURED);
		cc_port_err("cmd_type=%u, expected_cmd_type=%u\n",
			port, vdm_header.cmd_type, VDM_CMD_TYPE_RESP_ACK);
		return -EFAULT;
	}

	/*
	 * The return data is the whole HPI VDM message data.
	 * Detail of the data format can refer to the commensts for the
	 * struct hpi_vdm_message definition.
	 */
	if (resp_buf && resp_buf_size && *resp_buf_size) {
		*resp_buf_size = min(*resp_buf_size, vdm_resp_size);
		memcpy(resp_buf, vdm_data, *resp_buf_size);
	}

	return 0;
}

int cyccg_port_cc_get_device_mode(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		enum ccg_fw_mode_type *running_mode, u16 *last_flash_row)
{
	u8 vdm_data[HPI_MAX_VDM_MSG_SIZE];
	struct hpi_vdm_message *vdm_resp = (struct hpi_vdm_message *)vdm_data;
	size_t vdm_resp_size = sizeof(vdm_data);
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	if (!running_mode)
		return -EINVAL;

	err = cyccg_port_cc_send_u_vdm_command(port, vdm_mode,
			VDM_CY_CMD_GET_DEVICE_MODE, NULL, 0, 0,
			vdm_resp, &vdm_resp_size);
	if (err) {
		cc_port_err("failed to do GET_DEVICE_MODE, %d\n", port, err);
		return err;
	}

	*running_mode =
		(enum ccg_fw_mode_type)get_unaligned_le32(&vdm_resp->vdo[0]);
	if (last_flash_row) {
		if (vdm_resp_size >= HPI_VDM_RESP_SIZE_BY_VDO_NUM(2) &&
				*running_mode == CCG_FW_MODE_TYPE_BOOTLAODER) {
			*last_flash_row =
				(u16)get_unaligned_le32(&vdm_resp->vdo[1]);
		} else {
			*last_flash_row = 0;
		}
	}

	cc_port_dbg("running_mode = %u\n", port,
		get_unaligned_le32(&vdm_resp->vdo[0]));
	if (vdm_resp_size >= HPI_VDM_RESP_SIZE_BY_VDO_NUM(2) &&
			*running_mode == CCG_FW_MODE_TYPE_BOOTLAODER)
		cc_port_dbg("bl_last_flash_row = 0x%x\n", port,
			get_unaligned_le32(&vdm_resp->vdo[1]));
	cc_port_vdbg("<<<< enter\n", port);
	return 0;
}

int cyccg_port_cc_get_device_version(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		struct hpi_ccg_fw_version *ccg_fw_vers)
{
	u8 vdm_data[HPI_MAX_VDM_MSG_SIZE];
	struct hpi_vdm_message *vdm_resp = (struct hpi_vdm_message *)vdm_data;
	size_t vdm_resp_size = sizeof(vdm_data);
	u32 vdo;
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	if (!ccg_fw_vers)
		return -EINVAL;

	err = cyccg_port_cc_send_u_vdm_command(port, vdm_mode,
			VDM_CY_CMD_GET_DEVICE_VERSION, NULL, 0, 0,
			vdm_resp, &vdm_resp_size);
	if (err) {
		cc_port_err("failed to GET_DEVICE_VERSION, %d\n", port, err);
		return err;
	}

	vdo = get_unaligned_le32(&vdm_resp->vdo[0]);
	memcpy(&ccg_fw_vers->btldr.base, &vdo, sizeof(u32));
	vdo = get_unaligned_le32(&vdm_resp->vdo[1]);
	memcpy(&ccg_fw_vers->btldr.app, &vdo, sizeof(u32));
	vdo = get_unaligned_le32(&vdm_resp->vdo[2]);
	memcpy(&ccg_fw_vers->fw1_app.base, &vdo, sizeof(u32));
	vdo = get_unaligned_le32(&vdm_resp->vdo[3]);
	memcpy(&ccg_fw_vers->fw1_app.app, &vdo, sizeof(u32));
	vdo = get_unaligned_le32(&vdm_resp->vdo[4]);
	memcpy(&ccg_fw_vers->fw2_app.base, &vdo, sizeof(u32));
	vdo = get_unaligned_le32(&vdm_resp->vdo[5]);
	memcpy(&ccg_fw_vers->fw2_app.app, &vdo, sizeof(u32));

	cc_port_dbg("bootloader base version: %u.%u.%u.%u\n", port,
		ccg_fw_vers->btldr.base.major,
		ccg_fw_vers->btldr.base.minor,
		ccg_fw_vers->btldr.base.patch_ver,
		ccg_fw_vers->btldr.base.build_number);
	cc_port_dbg("bootloader app version: %u.%u,%u,%c%c\n", port,
		ccg_fw_vers->btldr.app.major,
		ccg_fw_vers->btldr.app.minor,
		ccg_fw_vers->btldr.app.external_circuit_ver,
		ccg_fw_vers->btldr.app.name[0],
		ccg_fw_vers->btldr.app.name[1]);
	cc_port_dbg("FW image-1 base version: %u.%u.%u.%u\n", port,
		ccg_fw_vers->fw1_app.base.major,
		ccg_fw_vers->fw1_app.base.minor,
		ccg_fw_vers->fw1_app.base.patch_ver,
		ccg_fw_vers->fw1_app.base.build_number);
	cc_port_vdbg("FW image-1 app version: %u.%u,%u,%c%c\n", port,
		ccg_fw_vers->fw1_app.app.major,
		ccg_fw_vers->fw1_app.app.minor,
		ccg_fw_vers->fw1_app.app.external_circuit_ver,
		ccg_fw_vers->fw1_app.app.name[0],
		ccg_fw_vers->fw1_app.app.name[1]);
	cc_port_dbg("FW image-2 base version: %u.%u.%u.%u\n", port,
		ccg_fw_vers->fw2_app.base.major,
		ccg_fw_vers->fw2_app.base.minor,
		ccg_fw_vers->fw2_app.base.patch_ver,
		ccg_fw_vers->fw2_app.base.build_number);
	cc_port_dbg("FW image-2 app version: %u.%u,%u,%c%c\n", port,
		ccg_fw_vers->fw2_app.app.major,
		ccg_fw_vers->fw2_app.app.minor,
		ccg_fw_vers->fw2_app.app.external_circuit_ver,
		ccg_fw_vers->fw2_app.app.name[0],
		ccg_fw_vers->fw2_app.app.name[1]);
	return 0;
}

int cyccg_port_cc_get_silicon_id(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		u32 *silicon_id, u8 *uuid, size_t uuid_buf_size)
{
	u8 vdm_data[HPI_MAX_VDM_MSG_SIZE];
	struct hpi_vdm_message *vdm_resp = (struct hpi_vdm_message *)vdm_data;
	size_t vdm_resp_size = sizeof(vdm_data);
	u32 resp_code;
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	if (!silicon_id || (uuid && uuid_buf_size < 8))
		return -EINVAL;

	put_unaligned_le32(VMD_CY_CMD_SIGNATURE_GET_SILICON_ID, vdm_data);
	err = cyccg_port_cc_send_u_vdm_command(port, vdm_mode,
			VDM_CY_CMD_GET_SILICON_ID,
			vdm_data, VDM_VDO_OBJ_SIZE, 0,
			vdm_resp, &vdm_resp_size);
	if (err) {
		cc_port_err("failed to do GET_SILICON_ID, %d\n", port, err);
		return err;
	}

	resp_code = get_unaligned_le32(&vdm_resp->vdo[0]);
	if (resp_code != VDM_CY_CMD_RESP_SUCCESS) {
		cc_port_err("responsed with invalid code=%u\n",
			port, resp_code);
		return -EFAULT;
	}

	if (vdm_resp_size >=  HPI_VDM_RESP_SIZE_BY_VDO_NUM(3)) {
		*silicon_id = get_unaligned_le32(&vdm_resp->vdo[1]);
		if (vdm_resp_size == HPI_VDM_RESP_SIZE_BY_VDO_NUM(5) &&
				uuid &&
				uuid_buf_size >= HPI_FW_IMAGE_UUID_SIZE) {
			memcpy(uuid, &vdm_resp->vdo[2], HPI_FW_IMAGE_UUID_SIZE);
			cc_port_dump("UUID hex dump (8):\n", port, uuid, 8);
		}
	} else {
		cc_port_err("invalid VDM response msg size=%zu\n",
			port, vdm_resp_size);
		return -EINVAL;
	}

	cc_port_vdbg(">>>> exit, silicon_id = 0x%08x\n", port,
		get_unaligned_le32(&vdm_resp->vdo[1]));
	return 0;
}

static enum hpi_msg_return _cyccg_port_cc_vdm_device_reset_filter(
		struct hpi_device *port, struct hpi_msg *msg)
{
	cc_port_vdbg("<<<< enter, msg->code=0x%x, len=%zu\n", port,
		msg->code, msg->len);
	/*
	 * If success, no VDM response for the device reset command. And the
	 * Type-C and PD connection must be re-negotiated.
	 * if fail, VDM data response received with response code.
	 */
	switch (msg->code) {
	case HPI_PD_RESP_VDM_RECEIVED:
		hpi_cmd_copy_return_data(port, msg);
		cc_port_err("VDM response error code=%u received\n", port,
			get_unaligned_le32(
			&(((struct hpi_vdm_message *)msg->data)->vdo[0])));
		hpi_cmd_set_state_errcode(port, HPI_CMD_COMPLETED, -EINVAL);
		return HPI_MSG_RETURN_HANDLED;

	/*
	 * Following events are caused by the success of VDM device reset.
	 * The Type-C and PD port will be re-negotiated.
	 */
	case HPI_PD_RESP_TYPE_C_DISCONNECTED:
	case HPI_PD_RESP_TYPE_C_CONNECTED:
	case HPI_PD_RESP_SRC_CAP_RCVD:
	case HPI_PD_RESP_SINK_CAP_RCVD:
	case HPI_PD_RESP_PS_RDY:
		hpi_cmd_set_state_errcode(port, HPI_CMD_SUF_EVENTS_RCVD, 0);
		return HPI_MSG_RETURN_HANDLED;
	case HPI_PD_RESP_PD_CONTRACT_ESTABLISHED:
		hpi_cmd_set_state_errcode(port, HPI_CMD_COMPLETED, 0);
		return HPI_MSG_RETURN_HANDLED;
	default:
		break;
	}

	return HPI_MSG_RETURN_NONE;
}

int cyccg_port_cc_device_reset(struct hpi_device *port,
		enum vdm_sop_type vdm_mode)
{
	u8 vdm_data[HPI_MAX_VDM_MSG_SIZE];
	struct hpi_vdm_message *vdm_resp = (struct hpi_vdm_message *)vdm_data;
	size_t vdm_resp_size = sizeof(vdm_data);
	u32 signature = HPI_SIGNATURE_RESET;
	size_t vdm_cmd_size;
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	cyccg_port_cc_u_vdm_header_init((union u_vdm_header *)vdm_data,
			 VDM_CY_CMD_DEVICE_RESET, VDM_SVID_CYPRESS, 0);
	vdm_cmd_size = sizeof(union u_vdm_header);
	put_unaligned_le32(signature, &vdm_data[vdm_cmd_size]);
	vdm_cmd_size += sizeof(signature);
	err = hpi_port_send_vdm_data_generic(port,
			vdm_mode, HPI_CMD_FLAG_RESP_EVENT_RAW,
			vdm_data, vdm_cmd_size,
			vdm_resp, &vdm_resp_size,
			HPI_SYNC, NULL, NULL,
			_cyccg_port_cc_vdm_device_reset_filter, 3300);
	if (err) {
		cc_port_err("DEVICE_RESET failed, resp_code=0x%x, %d\n",
			port, get_unaligned_le32(&vdm_resp->vdo[0]), err);
		return err;
	}

	cc_port_vdbg(">>>> exit, reset completed success\n", port);
	return 0;
}

static int _cyccg_port_cc_jump_to_boot(struct hpi_device *port,
		enum vdm_sop_type vdm_mode, u32 signature)
{
	u8 vdm_data[HPI_MAX_VDM_MSG_SIZE];
	struct hpi_vdm_message *vdm_resp = (struct hpi_vdm_message *)vdm_data;
	size_t vdm_resp_size = sizeof(vdm_data);
	size_t vdm_cmd_size;
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s, signature=%c\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode), (char)(signature));

	vdm_cmd_size = sizeof(union u_vdm_header) + sizeof(signature);
	cyccg_port_cc_u_vdm_header_init((union u_vdm_header *)vdm_data,
			 VDM_CY_CMD_JUMP_TO_BOOT, VDM_SVID_CYPRESS, 0);
	vdm_cmd_size = sizeof(union u_vdm_header);
	put_unaligned_le32(signature, &signature);
	memcpy(vdm_data + vdm_cmd_size, &signature, sizeof(signature));
	vdm_cmd_size += sizeof(signature);

	err = hpi_port_send_vdm_data_generic(port,
			vdm_mode, HPI_CMD_FLAG_RESP_EVENT_RAW,
			vdm_data, vdm_cmd_size,
			vdm_resp, &vdm_resp_size,
			HPI_SYNC, NULL, NULL,
			_cyccg_port_cc_vdm_device_reset_filter, 3000);
	if (err) {
		cc_port_err("failed to do %s, resp_code=0x%x %d\n",
			port, signature == HPI_SIGNATURE_JUMP_TO_BOOT ?
				"JUMP_TO_BOOT" : "JUMP_TO_ALT_FW",
			get_unaligned_le32(&vdm_resp->vdo[0]), err);
		return err;
	}

	cc_port_vdbg(">>>> exit, %s completed success\n", port,
		signature == HPI_SIGNATURE_JUMP_TO_BOOT ?
				"JUMP_TO_BOOT" : "JUMP_TO_ALT_FW");
	return 0;
}

int cyccg_port_cc_jump_to_boot(struct hpi_device *port,
		enum vdm_sop_type vdm_mode)
{
	return _cyccg_port_cc_jump_to_boot(port, vdm_mode,
					HPI_SIGNATURE_JUMP_TO_BOOT);
}

int cyccg_port_cc_jump_to_alt_fw(struct hpi_device *port,
		enum vdm_sop_type vdm_mode)
{
	return _cyccg_port_cc_jump_to_boot(port, vdm_mode,
					HPI_SIGNATURE_JUMP_TO_ALT_FW);
}

int cyccg_port_cc_enter_flashing_mode(struct hpi_device *port,
		enum vdm_sop_type vdm_mode)
{
	u32 signature = HPI_SIGNATURE_ENTER_FLASHING_MODE;
	u8 vdm_data[HPI_MAX_VDM_MSG_SIZE];
	struct hpi_vdm_message *vdm_resp = (struct hpi_vdm_message *)vdm_data;
	size_t vdm_resp_size = sizeof(vdm_data);
	u32 resp_code;
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	put_unaligned_le32(signature, &signature);
	err = cyccg_port_cc_send_u_vdm_command(port, vdm_mode,
			VDM_CY_CMD_ENTER_FLASHING_MODE,
			&signature, sizeof(signature), 0,
			vdm_resp, &vdm_resp_size);
	if (err) {
		cc_port_err("failed to enter_flashing_mode, %d\n", port, err);
		return err;
	}

	resp_code = get_unaligned_le32(&vdm_resp->vdo[0]);
	if (resp_code != VDM_CY_CMD_RESP_SUCCESS) {
		cc_port_err("responsed with invalid code=%u\n",
			port, resp_code);
		return -EFAULT;
	}

	cc_port_vdbg(">>>> exit, enter flashing mode success\n", port);
	return 0;
}

static enum hpi_msg_return _cyccg_port_cc_vdm_send_read_data_filter(
		struct hpi_device *port, struct hpi_msg *msg)
{
	cc_port_vdbg("<<<< enter, msg->code=0x%x, len=%zu\n", port,
		msg->code, msg->len);
	/*
	 * If VDM send data is valid, no VDM response will be sent by CCG.
	 * if fail, a VDM response with error code will be sent.
	 */
	switch (msg->code) {
	case HPI_PD_RESP_SUCCESS:
		cc_port_dbg("VDM command CC_SEND_DATA write success\n", port);
		hpi_cmd_set_state_errcode(port, HPI_CMD_COMPLETED, 0);
		return HPI_MSG_RETURN_HANDLED;
	case HPI_PD_RESP_VDM_RECEIVED:
		hpi_cmd_copy_return_data(port, msg);
		cc_port_err("VDM response error code=%u received\n", port,
			get_unaligned_le32(
			&(((struct hpi_vdm_message *)msg->data)->vdo[0])));
		hpi_cmd_set_state_errcode(port, HPI_CMD_COMPLETED, -EINVAL);
		return HPI_MSG_RETURN_HANDLED;
	default:
		break;
	}

	return HPI_MSG_RETURN_NONE;
}

static int cyccg_port_cc_send_data(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		void *write_buf, size_t write_size)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	u8 vdm_data[HPI_MAX_VDM_MSG_SIZE];
	struct hpi_vdm_message *vdm_resp = (struct hpi_vdm_message *)vdm_data;
	size_t vdm_resp_size;
	size_t vdm_cmd_size;
	u8 *cursor;
	u32 seq_num;
	u32 left_size;
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	if (!write_buf || (write_size !=
				HPI_REG_SIZE_OF(FLASH_RW, HPI_VERSION_1) &&
			write_size !=
				HPI_REG_SIZE_OF(FLASH_RW, HPI_VERSION_2))) {
		cc_port_err("invalid send data buffer or size=%zu\n",
			port, write_size);
		return -EINVAL;
	}

	for (cursor = write_buf, seq_num = 1;
			cursor < ((u8 *)write_buf + write_size);
			cursor += VDM_MAX_DATA_SIZE, seq_num++) {
		cyccg_port_cc_u_vdm_header_init((union u_vdm_header *)vdm_data,
			VDM_CY_CMD_SEND_DATA, VDM_SVID_CYPRESS, seq_num);
		vdm_cmd_size = sizeof(union u_vdm_header);

		left_size = write_size - ((seq_num - 1) * VDM_MAX_DATA_SIZE);
		left_size = (left_size > VDM_MAX_DATA_SIZE) ?
					VDM_MAX_DATA_SIZE : left_size;
		memcpy(vdm_data + vdm_cmd_size, cursor, left_size);
		vdm_cmd_size += left_size;
		vdm_resp_size = sizeof(vdm_data);
		cc_port_dump("seq_num=%u, size=%zu\n", port,
			&vdm_data[0], vdm_cmd_size, seq_num, vdm_cmd_size);

		/*
		 * Must wait the VDM register control write response message.
		 * For the CC_SEND_DATA command, there was no response from
		 * the port partner if success.
		 */
		err = hpi_port_send_vdm_data_generic(port,
				vdm_mode, HPI_CMD_FLAG_RAW,
				vdm_data, vdm_cmd_size,
				vdm_resp, &vdm_resp_size,
				HPI_SYNC, NULL, NULL,
				_cyccg_port_cc_vdm_send_read_data_filter,
				HPI_TIME_OF(DEFAULT, hpi_ver));
		if (err && err != -ETIMEDOUT) {
			cc_port_err("SEND_DATA seq_num=%u failed\n",
				port, seq_num);
			cc_port_err("SEND_DATA resp_code=0x%x, %d\n", port,
				get_unaligned_le32(&vdm_resp->vdo[0]), err);
			return err;
		}

		/*
		 * 1) SEND_DATA has no response/data on success to EC, and a
		 *    minimum delay of 100us must be inserted between two
		 *    consecutive SEND_DATA VDM command;
		 * 2) Depending the CCGx FW design, CCG1 and CCG2 don't support
		 *    PD comamnd queue, but CCG3 and CCG4 support PD command
		 *    queue;
		 * 3) As defined in the USB PD spec R3 V1.0, the timer value of
		 *    VDMResponseTimer(24 ~ 30 ms) shall be used by the
		 *    Initiator's Policy Engine to ensure that a VDM Command
		 *    request needing a response. When the VDMResponseTimer
		 *    expires, the failure will be detected.
		 * so, the for CCG1 and CCG2 devices, the 100us delay is set;
		 * for CCG3 and CCG4 devices, the max 30ms delay is set to avoid
		 * any possible PD command busy caused issue.
		 */
		if (hpi_ver == HPI_VERSION_1)
			usleep_range(100, 200);
		else
			msleep(30);
	}

	cc_port_vdbg(">>>> exit, success\n", port);
	return 0;
}

static int cyccg_port_cc_flash_write(struct hpi_device *port,
		enum vdm_sop_type vdm_mode, u16 flash_row_num)
{
	u8 vdm_data[HPI_MAX_VDM_MSG_SIZE];
	struct hpi_vdm_message *vdm_resp = (struct hpi_vdm_message *)vdm_data;
	size_t vdm_resp_size = sizeof(vdm_data);
	u32 key_flash_row_num_vdo;
	u32 resp_code;
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	key_flash_row_num_vdo = HPI_SIGNATURE_FLASH_ROW_READ_WRITE & 0x000000ff;
	key_flash_row_num_vdo |= ((flash_row_num << 8) & 0x00ffff00);
	put_unaligned_le32(key_flash_row_num_vdo, &key_flash_row_num_vdo);
	err = cyccg_port_cc_send_u_vdm_command(port, vdm_mode,
			VDM_CY_CMD_FLASH_WRITE, &key_flash_row_num_vdo,
			sizeof(key_flash_row_num_vdo), 0,
			vdm_resp, &vdm_resp_size);
	if (err) {
		cc_port_err("failed to do FLASH_WRITE, %d\n", port, err);
		return err;
	}

	resp_code = get_unaligned_le32(&vdm_resp->vdo[0]);
	if (resp_code != VDM_CY_CMD_RESP_SUCCESS) {
		cc_port_err("responsed with invalid code=%u\n",
			port, resp_code);
		return -EFAULT;
	}

	cc_port_vdbg("flash write row_num=%u completed success\n",
			port, flash_row_num);
	return 0;
}

static int cyccg_port_cc_read_data(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		void *read_buf, size_t flash_row_size)
{
	u8 vdm_data[HPI_MAX_VDM_MSG_SIZE];
	struct hpi_vdm_message *vdm_resp = (struct hpi_vdm_message *)vdm_data;
	union vdm_msg_header *msg_header = &vdm_resp->msg_header;
	size_t vdm_resp_size;
	size_t left_size;
	size_t read_size;
	u8 *cursor;
	u32 seq_num;
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	left_size = flash_row_size;
	for (cursor = read_buf, seq_num = 1; left_size > 0; seq_num++) {
		vdm_resp_size = sizeof(vdm_data);
		err = cyccg_port_cc_send_u_vdm_command(port, vdm_mode,
				VDM_CY_CMD_READ_DATA, NULL, 0, seq_num,
				vdm_resp, &vdm_resp_size);
		if (err) {
			cc_port_err("failed to do READ_DATA, %d\n", port, err);
			return err;
		}

		msg_header->msg_header =
			get_unaligned_le16(&msg_header->msg_header);
		read_size = (msg_header->data_objs - 1) * VDM_VDO_OBJ_SIZE;
		memcpy(cursor, &vdm_resp->vdo[0], read_size);
		left_size -= read_size;
		cursor += read_size;
	}

	cc_port_vdbg(">>>> exit, success\n", port);
	return 0;
}

static int cyccg_port_cc_flash_read(struct hpi_device *port,
		enum vdm_sop_type vdm_mode, u16 flash_row_num)
{
	u8 vdm_data[HPI_MAX_VDM_MSG_SIZE];
	struct hpi_vdm_message *vdm_resp = (struct hpi_vdm_message *)vdm_data;
	size_t vdm_resp_size = sizeof(vdm_data);
	u32 key_flash_row_num_vdo;
	u32 resp_code;
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s, flash_row_num=%u\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode), flash_row_num);

	key_flash_row_num_vdo = HPI_SIGNATURE_FLASH_ROW_READ_WRITE & 0x000000ff;
	key_flash_row_num_vdo |= ((flash_row_num << 8) & 0x00ffff00);
	put_unaligned_le32(key_flash_row_num_vdo, &key_flash_row_num_vdo);
	err = cyccg_port_cc_send_u_vdm_command(port, vdm_mode,
			VDM_CY_CMD_FLASH_READ, &key_flash_row_num_vdo,
			sizeof(key_flash_row_num_vdo), 0,
			vdm_resp, &vdm_resp_size);
	if (err) {
		cc_port_err("failed to do FLASH_READ, %d\n", port, err);
		return err;
	}

	resp_code = get_unaligned_le32(&vdm_resp->vdo[0]);
	if (resp_code != VDM_CY_CMD_RESP_SUCCESS) {
		cc_port_err("responsed with invalid code=%u\n",
			port, resp_code);
		return -EFAULT;
	}

	cc_port_vdbg(">>>> exit, success\n", port);
	return 0;
}

int cyccg_port_cc_validate_fw(struct hpi_device *port,
		enum vdm_sop_type vdm_mode, enum ccg_fw_mode_type type)
{
	u8 vdm_data[HPI_MAX_VDM_MSG_SIZE];
	struct hpi_vdm_message *vdm_resp = (struct hpi_vdm_message *)vdm_data;
	size_t vdm_resp_size = sizeof(vdm_data);
	u32 type_vdo;
	u32 resp_code;
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	type_vdo = (u32)type;
	put_unaligned_le32(type_vdo, &type_vdo);
	err = cyccg_port_cc_send_u_vdm_command(port, vdm_mode,
			VDM_CY_CMD_VALIDATE_FW,
			&type_vdo, sizeof(type_vdo), 0,
			vdm_resp, &vdm_resp_size);
	if (err) {
		cc_port_err("failed to do VALIDATE_FW, %d\n", port, err);
		return err;
	}

	resp_code = get_unaligned_le32(&vdm_resp->vdo[0]);
	if (resp_code != VDM_CY_CMD_RESP_SUCCESS) {
		cc_port_err("responsed with invalid code=%u\n",
			port, resp_code);
		return -EFAULT;
	}

	cc_port_vdbg(">>>> exit, cc fw validate success\n", port);
	return 0;
}

int cyccg_port_cc_get_boot_mode_reason(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		struct hpi_boot_mode_reason *reason)
{
	u8 vdm_data[HPI_MAX_VDM_MSG_SIZE];
	struct hpi_vdm_message *vdm_resp = (struct hpi_vdm_message *)vdm_data;
	size_t vdm_resp_size = sizeof(vdm_data);
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	err = cyccg_port_cc_send_u_vdm_command(port, vdm_mode,
			VDM_CY_CMD_REASON_FOR_BOOT_MODE, NULL, 0, 0,
			vdm_resp, &vdm_resp_size);
	if (err) {
		cc_port_err("failed to do REASON_FOR_BOOT_MODE, %d\n",
			port, err);
		return err;
	}

	if (reason)
		*(u8 *)reason = get_unaligned_le32(&vdm_resp->vdo[0]);

	cc_port_vdbg("reason = 0x%02x\n", port,
		get_unaligned_le32(&vdm_resp->vdo[0]));
	return 0;
}

int cyccg_port_cc_get_checksum(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		u32 flash_addr, size_t data_size, u32 *checksum)
{
	u8 vdm_data[HPI_MAX_VDM_MSG_SIZE];
	struct hpi_vdm_message *vdm_resp = (struct hpi_vdm_message *)vdm_data;
	size_t vdm_resp_size = sizeof(vdm_data);
	u32 *vdo;
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	vdo = (u32 *)vdm_data;
	put_unaligned_le32(flash_addr, vdo);
	vdo += 1;
	put_unaligned_le32(data_size, vdo);
	err = cyccg_port_cc_send_u_vdm_command(port, vdm_mode,
			VDM_CY_CMD_GET_CHECKSUM,
			vdm_data, 2 * VDM_VDO_OBJ_SIZE, 0,
			vdm_resp, &vdm_resp_size);
	if (err) {
		cc_port_err("failed to do GET_CHECKSUM, %d\n", port, err);
		return err;
	}

	if (checksum)
		*checksum = get_unaligned_le32(&vdm_resp->vdo[0]);

	cc_port_vdbg("checksum = 0x%08x\n", port,
		get_unaligned_le32(&vdm_resp->vdo[0]));
	return 0;
}


int cyccg_port_cc_get_fw_start_addr(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		u32 *fw_image1_start_addr, u32 *fw_image2_start_addr)

{
	u8 vdm_data[HPI_MAX_VDM_MSG_SIZE];
	struct hpi_vdm_message *vdm_resp = (struct hpi_vdm_message *)vdm_data;
	size_t vdm_resp_size = sizeof(vdm_data);
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	err = cyccg_port_cc_send_u_vdm_command(port, vdm_mode,
			VDM_CY_CMD_GET_FW_START_ADDR,
			NULL, 0, 0,
			vdm_resp, &vdm_resp_size);
	if (err) {
		cc_port_err("failed to do get_fw_start_addr, %d\n", port, err);
		return err;
	}

	if (fw_image1_start_addr)
		*fw_image1_start_addr = get_unaligned_le32(&vdm_resp->vdo[0]);
	if (fw_image2_start_addr)
		*fw_image2_start_addr = get_unaligned_le32(&vdm_resp->vdo[1]);

	cc_port_vdbg("fw_image1_start_addr = 0x%08x\n",
		port, get_unaligned_le32(&vdm_resp->vdo[0]));
	cc_port_vdbg("fw_image2_start_addr = 0x%08x\n",
		port, get_unaligned_le32(&vdm_resp->vdo[1]));
	return 0;
}

int cyccg_port_cc_set_app_priority(struct hpi_device *port,
		enum vdm_sop_type vdm_mode, enum ccg_fw_mode_type fw_mode)
{
	u8 vdm_data[HPI_MAX_VDM_MSG_SIZE];
	struct hpi_vdm_message *vdm_resp = (struct hpi_vdm_message *)vdm_data;
	size_t vdm_resp_size = sizeof(vdm_data);
	u32 fw_mode_vdo;
	u32 resp_code;
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	put_unaligned_le32((u32)fw_mode, &fw_mode_vdo);
	err = cyccg_port_cc_send_u_vdm_command(port, vdm_mode,
			VDM_CY_CMD_SET_APP_PRIORITY,
			&fw_mode_vdo, sizeof(fw_mode_vdo), 0,
			vdm_resp, &vdm_resp_size);
	if (err) {
		cc_port_err("failed to do set_app_priority, %d\n",
			port, err);
		return err;
	}

	resp_code = get_unaligned_le32(&vdm_resp->vdo[0]);
	if (resp_code != VDM_CY_CMD_RESP_SUCCESS) {
		cc_port_err("responsed with invalid code=%u\n",
			port, resp_code);
		return -EFAULT;
	}

	cc_port_vdbg(">>>> exit success\n", port);
	return 0;
}

int cyccg_port_cc_send_signature(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		void *signature, size_t signature_buf_size)
{
	u8 vdm_data[HPI_MAX_VDM_MSG_SIZE];
	struct hpi_vdm_message *vdm_resp = (struct hpi_vdm_message *)vdm_data;
	size_t vdm_resp_size = sizeof(vdm_data);
	size_t vdm_data_size;
	size_t signature_size = 64;
	u8 *cursor;
	u32 seq_num;
	u32 resp_code;
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	/* The Digital Signature for a FW image is fixed 64 bytes. */
	if (!signature || signature_buf_size < signature_size)
		return -EINVAL;

	for (cursor = signature, seq_num = 1;
			cursor < ((u8 *)signature + signature_size);
			cursor += VDM_MAX_DATA_SIZE, seq_num++) {
		vdm_data_size = signature_size -
					((seq_num - 1) * VDM_MAX_DATA_SIZE);
		vdm_data_size = (vdm_data_size > VDM_MAX_DATA_SIZE) ?
					VDM_MAX_DATA_SIZE : vdm_data_size;
		cc_port_vdbg("seq_num=%u, vdm_data_size=%zu\n", port,
			seq_num, vdm_data_size);

		err = cyccg_port_cc_send_u_vdm_command(port, vdm_mode,
			VDM_CY_CMD_SEND_SIGNATURE,
			cursor, vdm_data_size, seq_num,
			vdm_resp, &vdm_resp_size);
		if (err) {
			cc_port_err("failed to SEND_SIGNATURE, %d\n",
				port, err);
			return err;
		}

		resp_code = get_unaligned_le32(&vdm_resp->vdo[0]);
		if (resp_code != VDM_CY_CMD_RESP_SUCCESS) {
			cc_port_err("resp with invalid code=%u\n",
				port, resp_code);
			return -EFAULT;
		}
	}

	cc_port_vdbg(">>>> exit success\n", port);
	return 0;
}

int cyccg_port_cc_get_boot_type(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		struct ccg_bootloader_type *bl_type)
{
	u8 vdm_data[HPI_MAX_VDM_MSG_SIZE];
	struct hpi_vdm_message *vdm_resp = (struct hpi_vdm_message *)vdm_data;
	size_t vdm_resp_size = sizeof(vdm_data);
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	err = cyccg_port_cc_send_u_vdm_command(port, vdm_mode,
			VDM_CY_CMD_GET_BOOT_TYPE, NULL, 0, 0,
			vdm_resp, &vdm_resp_size);
	if (err) {
		cc_port_err("failed to get_boot_type, %d\n", port, err);
		return err;
	}

	if (bl_type)
		*(u32 *)bl_type = get_unaligned_le32(&vdm_resp->vdo[0]);

	cc_port_vdbg("bl_type = 0x%08x\n", port,
		get_unaligned_le32(&vdm_resp->vdo[0]));
	return 0;
}

int cyccg_port_cc_get_customer_info(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		void *read_buf, size_t read_buf_size)
{
	u8 vdm_data[HPI_MAX_VDM_MSG_SIZE];
	struct hpi_vdm_message *vdm_resp = (struct hpi_vdm_message *)vdm_data;
	size_t vdm_resp_size;
	u8 *cursor;
	u32 seq_num;
	u32 resp_code;
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	/*
	 * The Customer specific information is fixed 32 bytes.
	 * And each time the GET_CUSTOMER_INFO command retrieve 16 bytes.
	 */
	if (!read_buf || read_buf_size < 32)
		return -EINVAL;

	for (cursor = read_buf, seq_num = 1;
			seq_num <= 2; cursor += 16, seq_num++) {
		vdm_resp_size = sizeof(vdm_data);
		err = cyccg_port_cc_send_u_vdm_command(port, vdm_mode,
			VDM_CY_CMD_GET_CUSTOMER_INFO, cursor, 16, seq_num,
			vdm_resp, &vdm_resp_size);
		if (err) {
			cc_port_err("GET_CUSTOMER_INFO error, %d\n", port, err);
			return err;
		}

		resp_code = get_unaligned_le32(&vdm_resp->vdo[0]);
		if (resp_code != VDM_CY_CMD_RESP_SUCCESS) {
			cc_port_err("resp with invalid code=%u\n",
				port, resp_code);
			return -EFAULT;
		}
	}

	cc_port_vdbg(">>>> exit success\n", port);
	return 0;
}

int cyccg_port_cc_flash_record_write(struct hpi_device *port,
		enum vdm_sop_type vdm_mode, struct cybin_record *record)
{
	size_t record_data_size = (size_t)be16_to_cpu(record->record_data_size);
	u16 record_row_num = be16_to_cpu(record->record_num);
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	err = cyccg_port_cc_send_data(port, vdm_mode,
				record->record_data, record_data_size);
	if (err) {
		cc_port_err("send record(row=%u) data failed, %d\n",
			port, record_row_num, err);
		return err;
	}

	err = cyccg_port_cc_flash_write(port, vdm_mode, record_row_num);
	if (err) {
		cc_port_err("do FLASH_WRITE failed, row_num=%u, %d\n",
			port, record_row_num, err);
		return err;
	}

	cc_port_vdbg("write flash record row=%u complete success\n",
		port, record_row_num);
	return 0;
}

int cyccg_port_cc_flash_record_read(struct hpi_device *port,
		enum vdm_sop_type vdm_mode,
		u16 flash_row_num, size_t flash_row_size,
		void *record_read_buf, size_t record_read_buf_size)
{
	int err;

	cc_port_vdbg("<<<< enter, vdm_mode=%s\n", port,
		hpi_vdm_sop_type_to_string(vdm_mode));

	if (!record_read_buf || record_read_buf_size < flash_row_size ||
			(flash_row_size !=
				HPI_REG_SIZE_OF(FLASH_RW, HPI_VERSION_1) &&
			 flash_row_size !=
				HPI_REG_SIZE_OF(FLASH_RW, HPI_VERSION_2))) {
		cc_port_err("invalid read_buf/size=%zu or row_size=%zu\n",
			port, record_read_buf_size, flash_row_size);
		return -EINVAL;
	}

	err = cyccg_port_cc_flash_read(port, vdm_mode, flash_row_num);
	if (err) {
		cc_port_err("do FLASH_READ failed, row_num=%u, %d\n",
			port, flash_row_num, err);
		return err;
	}

	err = cyccg_port_cc_read_data(port, vdm_mode,
				      record_read_buf, flash_row_size);
	if (err) {
		cc_port_err("READ_DATA failed, row_num=%u, %d\n",
			port, flash_row_num, err);
		return err;
	}

	cc_port_vdbg("read flash record row=%u complete success\n",
		port, flash_row_num);
	return 0;
}

static int cyccg_update_cc_port_init(struct hpi_device *port)
{
	u32 event_mask;
	u8 pdo_mask;
	int err;

	cc_port_vdbg("<<<< enter\n", port);

	event_mask = 0xffffffff;
	err = hpi_port_write_event_mask_sync(port, event_mask);
	if (err) {
		cc_port_err("failed to set enable all events, %d\n", port, err);
		return err;
	}

	pdo_mask = 0x01;
	err = hpi_port_select_source_pdo_sync(port, pdo_mask, NULL, 0);
	if (err) {
		cc_port_err("failed to set only 5V Source PDO, %d\n",
			port, err);
		return err;
	}

	pdo_mask = 0x01;
	err = hpi_port_select_sink_pdo_sync(port, pdo_mask, NULL, 0);
	if (err) {
		cc_port_err("failed to set only 5V Sink PDO, %d\n", port, err);
		return err;
	}

	err = hpi_port_ec_initialization_complete_sync(port);
	if (err) {
		cc_port_err("failed to complete EC init, %d\n", port, err);
		return err;
	}

	cc_port_vdbg(">>>> exit, success\n", port);
	return 0;
}

static enum hpi_msg_return _cyccg_update_cc_pd_established_event_filter(
		struct hpi_device *port, struct hpi_msg *msg)
{
	if (msg->code == HPI_PD_RESP_PD_CONTRACT_ESTABLISHED ||
		msg->code == HPI_PD_RESP_VDM_RECEIVED ||
		msg->code == HPI_PD_RESP_DP_SID_NOT_FOUND) {
		hpi_cmd_set_state_errcode(port, HPI_CMD_COMPLETED, 0);
		return HPI_MSG_RETURN_IGNORED;
	}

	return HPI_MSG_RETURN_NONE;
}

int cyccg_update_cc_enter_dfp_mode(struct hpi_device *port)
{
	struct hpi_pd_status pd_status;
	int err;

	cc_port_vdbg("<<<< enter\n", port);

	err = hpi_port_read_pd_status(port, &pd_status);
	if (err) {
		cc_port_err("failed to read PD status, %d\n", port, err);
		return err;
	}

	if (!pd_status.contract_established) {
		cc_port_err("PD contract not established\n", port);
		return -ENODEV;
	}

	err = cyccg_update_cc_port_init(port);
	if (err) {
		cc_port_err("failed init port for CC FW update, %d\n",
			port, err);
		return err;
	}

	err = hpi_port_read_pd_status(port, &pd_status);
	if (err) {
		cc_port_err("failed to read PD status, %d\n", port, err);
		return err;
	}

	if (!pd_status.contract_established) {
		/*
		 * Wait max 3250ms and check the PD contract status has been
		 * re-established again.
		 */
		hpi_device_event_monitor(port, HPI_SYNC,
				_cyccg_update_cc_pd_established_event_filter,
				3300, NULL, NULL);

		err = hpi_port_read_pd_status(port, &pd_status);
		if (err) {
			cc_port_err("failed to read PD status, %d\n",
				port, err);
			return err;
		}

		if (!pd_status.contract_established) {
			cc_port_err("PD contract not established\n", port);
			return -ENODEV;
		}
	}

	if (pd_status.current_data_role == PD_PORT_DATA_ROLE_DFP) {
		cc_port_vdbg("port data role has been in DFP\n", port);
		return 0;
	}

	err = hpi_port_data_role_swap_sync(port);
	if (err) {
		cc_port_err("failed to do port data role swap, %d\n",
			port, err);
		return err;
	}

	/*
	 * Wait Max for the DR_SWAPed DFP and UFP to finish possible
	 * negotiations, which may including the VDM events that would confuse
	 * the later VDM command/response with the port partner through CC.
	 */
	hpi_device_event_monitor(port, HPI_SYNC,
				_cyccg_update_cc_pd_established_event_filter,
				500, NULL, NULL);

#if 1
	/*
	 * Wait CCG FW finishes its internal communicating with the port partner
	 * after data role swap. Such as send source capabilities, cable
	 * discovery process and other operations.
	 * If the CCG FW has disabled the Cable Discovery function, this wait
	 * time can be disabled.
	 */
	msleep(1500);
#endif

	err = hpi_port_read_pd_status(port, &pd_status);
	if (err) {
		cc_port_err("failed to read all port status, %d\n",
			port, err);
		return err;
	}

	if (!pd_status.contract_established ||
			pd_status.current_data_role != PD_PORT_DATA_ROLE_DFP) {
		cc_port_err("port not working in PD DFP mode\n", port);
		return -ENODEV;
	}

	cc_port_vdbg(">>>> exit success\n", port);
	return 0;
}

int cyccg_update_cc_enter_cy_mode(struct hpi_device *port,
		enum vdm_sop_type sop_type, u8 *cy_mode_obj_pos)
{
	u8 vdm_data[HPI_MAX_VDM_MSG_SIZE];
	struct hpi_vdm_message *vdm_resp = (struct hpi_vdm_message *)vdm_data;
	size_t vdm_resp_size = sizeof(vdm_data);
	struct vdm_resp_discover_id *discover_id;
	struct vdm_resp_discover_svid *discover_svid;
	union vdm_svid_vdo *svid_vdo;
	struct vdm_resp_discover_mode *discover_mode;
	union vdm_mode_vdo *mode_vdo;
	bool found;
	int vdo_objs;
	int i;
	int err;

	cc_port_vdbg("<<<< enter, sop_type=%s\n", port,
		hpi_vdm_sop_type_to_string(sop_type));

	if (!cy_mode_obj_pos) {
		cc_port_err("cy_mode_obj_pos is NULL\n", port);
		return -EINVAL;
	}
	*cy_mode_obj_pos = 0;

	/* Find out if the device's Vendor ID is Cypress Vendor ID. */
	err = cyccg_port_cc_discover_identity(port, sop_type,
			vdm_resp, &vdm_resp_size);
	if (err) {
		cc_port_err("failed to do discover_identity, %d\n",
			port, err);
		return err;
	}

	discover_id = (struct vdm_resp_discover_id *)&vdm_resp->s_vdm_header;
	if (discover_id->id_header.svid != VDM_SVID_CYPRESS) {
		cc_port_err("ID header, not Cypress VID, SVID=0x%04x\n",
			port, discover_id->id_header.svid);
		return -ENOTSUPP;
	}

	/*
	 * Find out if the Cypress SVID is supported in the SVID list.
	 */
	found = false;
	do  {
		err = cyccg_port_cc_discover_svid(port, sop_type,
				vdm_resp, &vdm_resp_size);
		if (err) {
			cc_port_err("failed to do discover_svid, %d\n",
				port, err);
			return err;
		}

		discover_svid =	(struct vdm_resp_discover_svid *)
						&vdm_resp->s_vdm_header;
		vdm_resp->msg_header.msg_header =
			get_unaligned_le16(&vdm_resp->msg_header.msg_header);
		vdo_objs = vdm_resp->msg_header.data_objs - 1;
		for (i = 0; i < vdo_objs; i++) {
			svid_vdo = &discover_svid->svid_vdo[i];
			cc_port_dbg("svid_vdo[%d] = 0x%08x\n",
				port, i, svid_vdo->vdo);
			if (svid_vdo->svid0 == VDM_SVID_CYPRESS ||
					svid_vdo->svid1 == VDM_SVID_CYPRESS) {
				found = true;
				break;
			}
		}
	} while (vdo_objs == VDM_MAX_VDO_NUM &&
			svid_vdo->svid0 != VDM_SVID_TERMINATING &&
			svid_vdo->svid1 != VDM_SVID_TERMINATING);

	if (!found) {
		cc_port_err("SVID list, no Cypress SVID=0x%04x found\n",
			port, VDM_SVID_CYPRESS);
		return -ENOTSUPP;
	}

	/*
	 * Find out if the Cypress Flashing Mode is supported and
	 * retrieve the Cypress Flashing Mode index number.
	 */
	err = cyccg_port_cc_discover_mode(port, sop_type,
			VDM_SVID_CYPRESS, vdm_resp, &vdm_resp_size);
	if (err) {
		cc_port_err("failed to do discover_mode, %d\n", port, err);
		return err;
	}

	discover_mode = (struct vdm_resp_discover_mode *)
					&vdm_resp->s_vdm_header;
	vdm_resp->msg_header.msg_header =
		get_unaligned_le16(&vdm_resp->msg_header.msg_header);
	vdo_objs = vdm_resp->msg_header.data_objs - 1;
	for (i = 0; i < vdo_objs; i++) {
		mode_vdo = &discover_mode->modes[i];
		mode_vdo->vdo = get_unaligned_le32(&mode_vdo->vdo);
		cc_port_dbg("mode_vdo[%d] = 0x%08x\n", port, i, mode_vdo->vdo);
		if (mode_vdo->cy_mode == VDM_CY_FLASHING_MODE) {
			*cy_mode_obj_pos = i + 1;
			break;
		}
	}

	if (*cy_mode_obj_pos < 1) {
		cc_port_err("no Cypress flashing mode found\n", port);
		return -ENOTSUPP;
	}

	cc_port_dbg("found CY flashing mode, cy_mode_obj_pos=%u\n",
		port, *cy_mode_obj_pos);

	/* Enter Cypress Flashing Mode. */
	err = cyccg_port_cc_enter_mode(port, sop_type,
			VDM_SVID_CYPRESS, *cy_mode_obj_pos, NULL);
	if (err) {
		cc_port_err("failed to enter CY flashing mode, %d\n",
			port, err);
		return err;
	}

	cc_port_vdbg(">>>> exit success\n", port);
	return 0;
}

static int cyccg_update_cc_read_ccg_device_info(
		struct hpi_device *port, struct ccg_update_info *update_info)
{
	struct ccg_info *ccg_info = &update_info->ccg_info;
	u32 silicon_id;
	u32 fw1_start_addr;
	u32 fw2_start_addr;
	int err;

	cc_port_vdbg("<<<< enter\n", port);

	err = cyccg_update_cc_enter_dfp_mode(port);
	if (err) {
		cc_port_err("failed to enter DFP mode, %d\n", port, err);
		return err;
	}

	err = cyccg_update_cc_enter_cy_mode(port, update_info->sop_type,
					    &update_info->cy_mode_obj_pos);
	if (err) {
		cc_port_err("failed to enter CY flashing mode, %d\n",
			port, err);
		if (update_info->cy_mode_obj_pos < 1) {
			cc_port_dbg("attached dev not support CY mode\n", port);
			err = -ENOTSUPP;
		}
		return err;
	}

	err = cyccg_port_cc_get_silicon_id(port, update_info->sop_type,
			&silicon_id, ccg_info->uuid, HPI_FW_IMAGE_UUID_SIZE);
	if (err) {
		cc_port_err("failed to get silicon id, %d\n", port, err);
		return err;
	}

	ccg_info->silicon_id = (u16)silicon_id;
	ccg_info->ccg_ver = ccg_silicon_id_to_ccg_version(ccg_info->silicon_id);
	if (ccg_info->ccg_ver == CCG_UNKNOWN) {
		cc_port_err("unsupport silicon_ID=0x%04x\n",
			port, ccg_info->silicon_id);
		return -EINVAL;
	}
	cc_port_dbg("silicon id = 0x%04x\n", port, ccg_info->silicon_id);

	/* CCG3 following HPIv2, but the still using 128 bytes flash size. */
	if (ccg_info->ccg_ver <= CCG3)
		ccg_info->flash_row_size =
				HPI_REG_SIZE_OF(FLASH_RW, HPI_VERSION_1);
	else
		ccg_info->flash_row_size =
				HPI_REG_SIZE_OF(FLASH_RW, HPI_VERSION_2);

	if (ccg_info->ccg_ver <= CCG2) {
		ccg_info->hpi_ver = HPI_VERSION_1;
		ccg_info->flash_mode = CCG_LEGACY_BOOT_MODE;
	} else {
		ccg_info->hpi_ver = HPI_VERSION_2;
		ccg_info->flash_mode = CCG_DUAL_FW_MODE;
	}
	cc_port_dbg("ccg_ver = CCG%d\n", port, (int)ccg_info->ccg_ver);
	cc_port_dbg("hpi_ver = %d\n", port, (int)ccg_info->hpi_ver + 1);
	cc_port_dbg("flash_mode = %s\n", port,
		ccg_info->flash_mode == CCG_LEGACY_BOOT_MODE ?
			"CCG_LEGACY_BOOT_MODE" : "CCG_DUAL_FW_MODE");
	cc_port_dbg("flash_row_size = %u\n", port, ccg_info->flash_row_size);

	err = cyccg_port_cc_get_device_mode(port, update_info->sop_type,
			&update_info->running_mode, &ccg_info->bl_last_row_num);
	if (err) {
		cc_port_err("failed to running_mode, %d\n", port, err);
		return err;
	}

	err = cyccg_port_cc_get_device_version(port, update_info->sop_type,
			&update_info->ccg_fw_vers);
	if (err) {
		cc_port_err("failed to get device version, %d\n", port, err);
		return err;
	}

	if (ccg_info->ccg_ver >= CCG3) {
		err = cyccg_port_cc_get_boot_type(port, update_info->sop_type,
				&ccg_info->blt_type);
		if (err) {
			cc_port_err("failed to get boot loader type, %d\n",
				port, err);
			return err;
		}

		if (!ccg_info->blt_type.support_fw_update) {
			cc_port_err("CCG doesn't support FW update, %d\n",
				port, err);
			return -ENOTSUPP;
		}

		err = cyccg_port_cc_get_fw_start_addr(port,
					update_info->sop_type,
					&fw1_start_addr, &fw2_start_addr);
		if (err) {
			cc_port_err("failed to get FW start addr, %d\n",
				port, err);
			return err;
		}
		ccg_info->fw1_start_row_num =
				fw1_start_addr / ccg_info->flash_row_size;
		ccg_info->fw2_start_row_num =
				fw2_start_addr / ccg_info->flash_row_size;
		cc_port_dbg("cc get fw start addr success\n", port);
		cc_port_dbg("fw1_start_row=%u, fw2_start_row=%u\n", port,
			ccg_info->fw1_start_row_num,
			ccg_info->fw2_start_row_num);
	}

	cc_port_vdbg(">>>> exit success\n", port);
	return 0;
}

static int cyccg_update_cc_bootloader_enter(
		struct hpi_device *port, struct ccg_update_info *update_info)
{
	int err;

	cc_port_vdbg("<<<< enter\n", port);

	err = cyccg_port_cc_jump_to_boot(port, update_info->sop_type);
	if (err)
		cc_port_err("failed to enter BL mode, %d\n", port, err);

	err = cyccg_port_cc_get_device_mode(port, update_info->sop_type,
			&update_info->running_mode,
			&update_info->ccg_info.bl_last_row_num);
	if (err) {
		cc_port_err("failed to read running_mode, %d\n", port, err);
		return err;
	}

	if (update_info->running_mode != CCG_FW_MODE_TYPE_BOOTLAODER) {
		cc_port_err("running_mode=%d, failed enter BL mode\n",
			port, update_info->running_mode);
		return -EFAULT;
	}

	return 0;
}

static int cyccg_update_cc_jump_alt_fw(
		struct hpi_device *port, struct ccg_update_info *update_info)
{
	struct ccg_info *ccg_info = &update_info->ccg_info;
	int err;

	cc_port_vdbg("<<<< enter\n", port);

	err = cyccg_port_cc_jump_to_alt_fw(port, update_info->sop_type);
	if (err) {
		cc_port_err("failed to jump to alt fw, %d\n", port, err);
		return err;
	}

	/* Enter DFP mode CY mode. */
	err = cyccg_update_cc_read_ccg_device_info(port, update_info);
	if (err) {
		cc_port_err("failed to read device info, %d\n", port, err);
		return err;
	}

	err = cyccg_port_cc_get_device_mode(port, update_info->sop_type,
			&update_info->running_mode,
			&ccg_info->bl_last_row_num);
	if (err) {
		cc_port_err("failed to read device_mode, %d\n", port, err);
		return err;
	}

	cc_port_dbg("running_mode=%d, fw_type=%d\n", port,
		update_info->running_mode, update_info->fw_image->fw_type);
	cc_port_vdbg(">>>> exit\n", port);
	return 0;
}

static int cyccg_update_cc_enter_flashing_mode(
		struct hpi_device *port, struct ccg_update_info *update_info)
{
	struct ccg_info *ccg_info = &update_info->ccg_info;
	int err;

	cc_port_vdbg("<<<< enter\n", port);

	/* Get the latest CCG running_mode. */
	err = cyccg_port_cc_get_device_mode(port, update_info->sop_type,
			&update_info->running_mode, &ccg_info->bl_last_row_num);
	if (err) {
		cc_port_err("failed to read device_mode, %d\n", port, err);
		return err;
	}
	cc_port_dbg("running_mode = %d, flash_mode = %s\n", port,
		update_info->running_mode,
		ccg_info->flash_mode == CCG_LEGACY_BOOT_MODE ?
			"CCG_LEGACY_BOOT_MODE" : "CCG_DUAL_FW_MODE");

	if (ccg_info->flash_mode == CCG_LEGACY_BOOT_MODE) {
		/* Verify the device running in bootloader mode. */
		if (update_info->running_mode != CCG_FW_MODE_TYPE_BOOTLAODER) {
			err = cyccg_update_cc_bootloader_enter(port,
							       update_info);
			if (err) {
				cc_port_err("failed to enter BL mode, %d\n",
					port, err);
				return err;
			}

			err = cyccg_update_cc_enter_dfp_mode(port);
			if (err) {
				cc_port_err("failed to enter BL DFP mode, %d\n",
					port, err);
				return err;
			}
		}
	} else {
		if (update_info->running_mode ==
				update_info->fw_image->fw_type) {
			cc_port_dbg("running_mode=%d, fw_type=%d, not match\n",
				port, update_info->running_mode,
				update_info->fw_image->fw_type);
			cc_port_dbg("jump to ALT FW required\n", port);

			err = cyccg_update_cc_jump_alt_fw(port, update_info);
			if (err) {
				cc_port_err("failed to jump to alt fw, %d\n",
					port, err);
				return err;
			}

			if (update_info->running_mode ==
					update_info->fw_image->fw_type) {
				cc_port_err("running_mode=%d, fw_type=%d\n",
					port, update_info->running_mode,
					update_info->fw_image->fw_type);
				return -EINVAL;
			}

			cc_port_dbg("running_mode=%d, update to fw_type=%d\n",
				port, update_info->running_mode,
				update_info->fw_image->fw_type);
		}
	}

	/* Enter flashing mode. */
	err = cyccg_port_cc_enter_flashing_mode(port, update_info->sop_type);
	if (err) {
		cc_port_err("failed to enter BL flashing mode, %d\n",
			port, err);
		return err;
	}

	cc_port_vdbg(">>>> exit, success\n", port);
	return 0;
}

static int cyccg_update_cc_write_fw_image(
		struct hpi_device *port, struct ccg_update_info *update_info)
{
	struct cybin_fw_image *fw_image = update_info->fw_image;
	u8 data[HPI_MAX_FLASH_RW_REG_SIZE + sizeof(struct cybin_record) + 1];
	struct cybin_record *tmp_record = (struct cybin_record *)data;
	struct cybin_record *record;
	u16 record_num;
	int err;

	cc_port_vdbg("<<<< enter\n", port);

	/* Clear the firmware metadata in flash memory. */
	if (update_info->fw_image->image_type > CYBIN_IMAGE_TYPE_CONFIG) {
		cc_port_dbg("clear metadata\n", port);
		memset(data, 0, sizeof(data));
		record = cybin_get_metadata_record(fw_image);
		memcpy(&tmp_record, record, sizeof(struct cybin_record));
		err = cyccg_port_cc_flash_record_write(port,
				update_info->sop_type, record);
		if (err) {
			cc_port_err("write metadata row failed, %d\n",
				port, err);
			return err;
		}
	}

	/* Write each flash row to the device. */
	record = cybin_get_cybin_record(fw_image);
	while (record) {
		record_num = be16_to_cpu(record->record_num);
		cc_port_dbg("write flash record, row=0x%04x\n",
			port, record_num);

		err = cyccg_port_cc_flash_record_write(port,
				update_info->sop_type, record);
		if (err) {
			cc_port_err("failed to write flash row=0x%04x, %d\n",
				port, record_num, err);
			return err;
		}

		record = cybin_get_next_cybin_record(fw_image, record);
	}

	cc_port_vdbg(">>>> exit success\n", port);
	return 0;

}

static int cyccg_update_cc_validate_fw_image(
		struct hpi_device *port, struct ccg_update_info *update_info)
{
	struct cybin_fw_image *fw_image = update_info->fw_image;
	enum ccg_fw_mode_type fw_type = fw_image->fw_type;
	enum cybin_image_type image_type = fw_image->image_type;
	struct ccg_flash_macro_info *macro_info = fw_image->macro_info;
	u32 flash_addr;
	size_t data_size;
	u32 checksum;
	int err;

	cc_port_vdbg("<<<< enter\n", port);

	if (image_type >= CYBIN_IMAGE_TYPE_FW) {
		err = cyccg_port_cc_validate_fw(port,
				update_info->sop_type, fw_type);
		if (err) {
			cc_port_err("FW validate failed, %d\n", port, err);
			return err;
		}
		cc_port_dbg("CC validate FW success\n", port);
	}

	if (image_type == CYBIN_IMAGE_TYPE_CONFIG ||
			image_type == CYBIN_IMAGE_TYPE_CONFIG_FW) {
		flash_addr = HPI_CONFIG_TABLE_CHECKSUM_CALCULATE_START_OFFSET +
			macro_info->config_start_row * macro_info->row_size;
		data_size = ((1 + macro_info->config_end_row -
			macro_info->config_start_row) * macro_info->row_size) -
			HPI_CONFIG_TABLE_CHECKSUM_CALCULATE_START_OFFSET;
		err = cyccg_port_cc_get_checksum(port, update_info->sop_type,
				flash_addr, data_size, &checksum);
		if (err) {
			cc_port_err("get table checksum failed, %d\n",
				port, err);
			return err;
		}

		if (fw_image->table_checksum != (u8)checksum) {
			cc_port_dbg("table checksum not match\n", port);
			cc_port_dbg("expected=0x%02x, get=0x%02x\n",
				port, fw_image->table_checksum, (u8)checksum);
			return -EINVAL;
		}
		cc_port_dbg("CC validate config table success\n",
			port);
	}

	cc_port_vdbg(">>>> exit, success\n", port);
	return 0;
}

int cyccg_update_cc_reset_and_init(struct hpi_device *port,
		enum vdm_sop_type sop_type, u8 cy_mode_obj_pos,
		bool force_reset)
{
	struct hpi_pd_status pd_status;
	int timeout = 3300;
	int ret = 0;
	int err = 0;

	cc_port_vdbg("<<<< enter\n", port);

	if (!force_reset && cy_mode_obj_pos > 0) {
		cc_port_dbg("cc exit mode\n", port);
		err = cyccg_port_cc_exit_mode(port, sop_type,
				VDM_SVID_CYPRESS, cy_mode_obj_pos);
		if (err)
			cc_port_err("exit CY mode failed, %d\n", port, err);
	} else {
		if (cy_mode_obj_pos > 0) {
			cc_port_dbg("cc force reset\n", port);
			err = cyccg_port_cc_device_reset(port, sop_type);
			if (err)
				cc_port_err("reset port partner failed, %d\n",
					port, err);
		} else {
			cc_port_dbg("cc mode not entered, skip reset\n", port);
		}
	}
	ret = err;

	err = cyccg_port_init(port);
	if (err) {
		cc_port_err("failed to re-init host CCG port, %d\n",
			port, err);
		ret = ret ?: err;
	}

	do {
		err = hpi_port_read_pd_status(port, &pd_status);
		if (err) {
			cc_port_err("failed to read PD status, %d\n",
				port, err);
			break;
		}

		if (pd_status.contract_established) {
			cc_port_dbg("PD contract established, return early\n",
				port);
			break;
		}

		err = hpi_device_event_monitor(port, HPI_SYNC,
				_cyccg_update_cc_pd_established_event_filter,
				500, NULL, NULL);
		if (!err) {
			cc_port_dbg("PD contract established, success\n", port);
			break;
		} else if (err == -ETIMEDOUT) {
			timeout -= 500;
			continue;
		} else {
			cc_port_err("failed to event monitor, %d\n", port, err);
			break;
		}
	} while (timeout > 0);

	cc_port_vdbg(">>>> exit, %d\n", port, ret = ret ?: err);
	return ret = ret ?: err;
}

/*
 * cyccg_update_cc_do_port_fw_update - update the FW image for the port partner
 *	CCG device or the CCG device on the Cable.
 * @port - Instance of the hpi_device stands for the PD port connection.
 * @ccg_type - The target CCG device type, Noteboot/Mobile or Power Adapter or
 *	cable. It inidcates which firmware image names to be used in auto
 *	fimware update process.
 * @sop_type - Indicates the VDM packet type where the VDM data should be sent
 *	to.
 * @auto_update - Indicates the port FW update is triggered by the driver
 *	automatically FW update process or triggered through sysfs interface
 *	manually.
 * @buf - points to the buffer that user input in through the sysfs interface.
 *	For automatically FW update, it must be set to NULL.
 * @count - indicate the bytes of data in the @buf. For automatically FW update,
 *	it must be set to 0.
 *
 * On success, value 0 will be returned; otherwise, a negative errno value will
 * be returned.
 */
int cyccg_update_cc_do_port_fw_update(struct hpi_device *port,
		enum ccg_type ccg_type, enum vdm_sop_type sop_type,
		bool auto_update, const char *buf, size_t count)
{
	struct cyccg *cyccg = port->cyccg;
	struct ccg_update_info *update_info;
	struct hpi_app_version *app_ver;
	struct hpi_pd_status pd_status;
	struct hpi_type_c_status type_c_status;
	bool force_reset = false;
	int retries;
	u16 image_ver;
	u16 ccg_ver;
	int err, ret;

	cc_port_vdbg("<<<< enter, sop_type=%s, auto_update=%s, buf=%s\n", port,
		hpi_vdm_sop_type_to_string(sop_type),
		auto_update ? "true" : "false", buf ?: "NULL");

	/* The PD contract must has established for CC communication. */
	retries = 6;
	do {
		err = hpi_port_read_pd_status(port, &pd_status);
		if (err) {
			cc_port_err("failed to read pd_status, %d\n",
				port, err);
			return err;
		} else if (!pd_status.contract_established) {
			err = hpi_port_read_type_c_status(port, &type_c_status);
			if (err) {
				cc_port_err("read type_c_status failed, %d\n",
					port, err);
				return err;
			}

			if (!type_c_status.type_c_connected) {
				cc_port_dbg("Type-C not connected\n", port);
				return -ENODEV;
			}

			cc_port_dbg("PD Contract not established, waiting\n",
				port);
			err = -ENOTCONN;
			msleep(500);
		}
	} while (err && retries--);
	if (err)
		return err;

	update_info = cyccg_update_allocate_ccg_update_info(cyccg);
	if (!update_info)
		return -ENOMEM;
	update_info->ccg_type = ccg_type;
	update_info->sop_type = sop_type;
	cc_port_vdbg("update_info->ccg_type=%s, sop_type=%s\n", port,
		ccg_type_to_string(ccg_type),
		hpi_vdm_sop_type_to_string(sop_type));

	err = cyccg_update_cc_read_ccg_device_info(port, update_info);
	if (err) {
		cc_port_err("failed to read CCG device info, %d\n",
			port, err);
		goto out;
	}

	err = cyccg_update_fw_image_names_parse(cyccg, update_info,
						auto_update, buf, count);
	if (err) {
		cc_port_err("failed to parse fw image name(s), %d\n",
			port, err);
		goto out;
	}

	err = cyccg_update_load_check_fw_image(cyccg, update_info, auto_update);
	if (err) {
		cc_port_err("failed to load and check FW image, %d\n",
			port, err);
		goto out;
	}

	if (auto_update) {
		app_ver = &update_info->fw_image->image_ver.app;
		image_ver = app_ver->major << 8 | app_ver->minor;
		cc_port_dbg("FW image version: %u.%u,%u,%c%c\n", port,
			app_ver->major, app_ver->minor,
			app_ver->external_circuit_ver,
			app_ver->name[0], app_ver->name[1]);
		app_ver = &update_info->ccg_fw_vers.fw1_app.app;
		ccg_ver = app_ver->major << 8 | app_ver->minor;
		cc_port_dbg("CCG device version: %u.%u,%u,%c%c\n", port,
			app_ver->major, app_ver->minor,
			app_ver->external_circuit_ver,
			app_ver->name[0], app_ver->name[1]);
		if (update_info->running_mode != CCG_FW_MODE_TYPE_BOOTLAODER &&
				image_ver <= ccg_ver) {
			cc_port_dbg("old FW image, Return early\n", port);
			err = 0;
			goto out;
		}
	}

	force_reset = true;
	err = cyccg_update_cc_enter_flashing_mode(port, update_info);
	if (err) {
		cc_port_err("failed to load and check FW image, %d\n",
			port, err);
		goto out;
	}

	err = cyccg_update_cc_write_fw_image(port, update_info);
	if (err) {
		cc_port_err("failed to write FW image, %d\n", port, err);
		goto out;
	}

	err = cyccg_update_cc_validate_fw_image(port, update_info);
	if (err) {
		cc_port_err("failed to validate FW image, %d\n", port, err);
		goto out;
	}

out:
	ret = cyccg_update_cc_reset_and_init(port, update_info->sop_type,
			update_info->cy_mode_obj_pos, force_reset);
	if (ret)
		cc_port_err("failed to reset and re-init CCG, %d\n", port, ret);

	cyccg_update_free_ccg_update_info(cyccg, update_info);
	cc_port_vdbg("<<<< exit, cc do fw update done, %d\n", port, err ?: ret);
	return err ?: ret;
}

static int cyccg_port_auto_cc_fw_update_handler(struct hpi_device *port)
{
	int err = 0;

	cc_port_vdbg("<<<< enter\n", port);

	if (IS_ENABLED(CONFIG_AUTO_CABLE_FW_UPDATE) &&
			!port->cc_cable_update_done) {
		cc_port_vdbg("CC Cable FW update enabled\n", port);

		err = cyccg_update_cc_do_port_fw_update(port,
				CCG_CABLE, VDM_SOP_TYPE_SOP_PRIME,
				true, NULL, 0);
		if (!err) {
			port->cc_cable_update_done = true;
			cc_port_dbg("CC auto cable FW update success\n", port);
		} else if (err == -ENOTCONN || err == -ENODEV) {
			cc_port_dbg("no attached dev or connected dev\n", port);
		} else {
			cc_port_err("CC auto cable FW update failed, %d\n",
				port, err);
		}
		cc_port_vdbg("cc_cable_update_done = %d\n", port,
			port->cc_cable_update_done);
	}

	if (IS_ENABLED(CONFIG_AUTO_POWER_ADAPTER_FW_UPDATE) &&
			!port->cc_pa_update_done) {
		cc_port_vdbg("CC auto PA FW update enabled\n", port);

		err = cyccg_update_cc_do_port_fw_update(port,
				CCG_POWER_ADAPTER, VDM_SOP_TYPE_SOP,
				true, NULL, 0);
		if (!err) {
			port->cc_pa_update_done = true;
			cc_port_dbg("CC auto PA FW update success\n", port);
		} else if (err == -ENOTSUPP) {
			cc_port_dbg("attached dev not support CC FW update\n",
				port);
		} else if (err == -ENOTCONN || err == -ENODEV) {
			cc_port_dbg("no attached dev or connected dev\n", port);
		} else {
			cc_port_err("CC auto PA FW update failed, %d\n",
				port, err);
		}
		cc_port_vdbg("cc_pa_update_done = %d\n", port,
			port->cc_pa_update_done);
	}

	return err;
}

/*//////////////////////////////////////////////////////////////////////////*/

void cyccg_port_cc_fw_update_state_reset(
		struct hpi_device *port, struct hpi_msg *msg)
{
	unsigned long threshold_time;

	/*
	 * Reset these states when the Type-C state was disconnected and
	 * connected again, when the time between disconnect and re-connect
	 * must exceed 5 seconds. It's aimed to avoid internal reset, role swap
	 * or other event/state changes' disconnected and re-connected
	 * events caused re-do auto FW update again. But indeed, the attached
	 * device was not changed, so the update should be skipped when it has
	 * been successfully done already.
	 */
	spin_lock(&port->slock);

	/* Reset alternate mode state */
	switch (msg->code) {
	case HPI_PD_RESP_HARD_RESET:
	case HPI_PD_HARD_RESET_SENT:
	case HPI_PD_SOFT_RESET_SENT:
	case HPI_PD_RESP_TYPE_C_DISCONNECTED:
	case HPI_PD_RESP_TYPE_C_CONNECTED:
	case HPI_PD_RESP_PD_CONTRACT_ESTABLISHED:
		port->is_in_alt_mode = false;
		break;
	default:
		break;
	}

	/*
	 * When reset or swap operation happened, CCG and the port
	 * partner will be reconnected, in this situation,
	 * the auto update should not be triggered. Because the attached
	 * port partner device not changed.
	 */
	switch (msg->code) {
	case HPI_PD_RESP_RESET_COMPLETE:
	case HPI_PD_RESP_HARD_RESET:
	case HPI_PD_HARD_RESET_SENT:
	case HPI_PD_SOFT_RESET_SENT:
	case HPI_PD_CABLE_RESET_SENT:
	case HPI_PD_RESP_SWAP_COMPLETE:
	case HPI_PD_RESP_TYPE_C_DISCONNECTED:
		port->last_disconnect_time = jiffies;
		break;
	case HPI_PD_RESP_TYPE_C_CONNECTED:
		threshold_time = port->last_disconnect_time +
		       msecs_to_jiffies(CYCCG_CC_AUTO_FW_UPDATE_RECONNECT_TIME);
		if (time_after(jiffies, threshold_time)) {
			port->cc_pa_update_done = false;
			port->cc_cable_update_done = false;
		}
		break;
	default:
		break;
	}

	spin_unlock(&port->slock);
}

void cyccg_port_cc_fw_update_work(struct work_struct *work)
{
	struct hpi_device *port =
			container_of(work, struct hpi_device, cc_fw_work);
	int retries = CYCCG_FW_UPDATE_RETRIES;
	int err;

	cc_port_vdbg("<<<< enter\n", port);

	spin_lock(&port->slock);
	port->cc_fw_work_state = CYCCG_WORK_STATE_RUNNING;
	spin_unlock(&port->slock);

	cyccg_wait_for_idle(port);

	do {
		err = cyccg_port_auto_cc_fw_update_handler(port);
		if (!err) {
			cc_port_dbg("CC auto FW update done success\n", port);
			break;
		}

		if (err == -ENOTCONN || err == -ENODEV || err == -ENOTSUPP) {
			cc_port_dbg("no attached or not support dev\n", port);
			break;
		}

		msleep(CYCCG_FW_UPDATE_RETRY_INTERVAL);
		cc_port_dbg("CC auto FW update failed, %d. Retries=%d\n",
			port, err, CYCCG_FW_UPDATE_RETRIES - retries + 1);
	} while (err && --retries);

	cyccg_set_to_busying_state(port, false);

	spin_lock(&port->slock);
	port->cc_fw_work_state = CYCCG_WORK_STATE_NONE;
	spin_unlock(&port->slock);
}

void cyccg_port_queue_cc_auto_fw_update(struct hpi_device *port)
{
	if (!IS_ENABLED(CONFIG_AUTO_POWER_ADAPTER_FW_UPDATE) &&
			!IS_ENABLED(CONFIG_AUTO_CABLE_FW_UPDATE))
		return;

	spin_lock(&port->slock);

	if (port->cc_fw_work_state == CYCCG_WORK_STATE_NONE) {
		port->cc_fw_work_state = CYCCG_WORK_STATE_QUEUED;
		schedule_work(&port->cc_fw_work);
	}

	spin_unlock(&port->slock);

	cc_port_dbg("CC auto FW update work has been %s\n", port,
		port->cc_fw_work_state ? "queued" : "running");
}

void cyccg_auto_fw_update_work(struct work_struct *work)
{
	struct cyccg *cyccg = container_of(work, struct cyccg, fw_work);
	struct hpi_device *port;
	struct hpi_ccg_fw_version *ccg_fw_vers = &cyccg->ccg_state.ccg_fw_vers;
	int retries = CYCCG_FW_UPDATE_RETRIES;
	int i;
	int err;

	fw_vdbg("<<<< enter\n");

	spin_lock(&cyccg->slock);
	cyccg->fw_work_state = CYCCG_WORK_STATE_RUNNING;
	spin_unlock(&cyccg->slock);

	cyccg_wait_for_idle(&cyccg->hpi_dev);
	fw_dbg("cyccg_wait_for_idle success\n");

	/* Auto update FW image for host side CCG device. */
	fw_info("FW Versions Before auto update:\n");
	hpi_ccg_fw_version_dump(ccg_fw_vers);

	do {
		err = cyccg_auto_fw_update_thread_handler(cyccg);
		if (!err) {
			fw_dbg("Auto FW update done success\n");
			fw_info("FW Versions After auto update:\n");
			hpi_ccg_fw_version_dump(ccg_fw_vers);
			break;
		}

		msleep(CYCCG_FW_UPDATE_RETRY_INTERVAL);
		fw_dbg("Auto FW update failed, %d. Retries=%d\n",
			err, CYCCG_FW_UPDATE_RETRIES - retries + 1);
	} while (err && --retries);

	cyccg_set_to_busying_state(&cyccg->hpi_dev, false);

	spin_lock(&cyccg->slock);
	cyccg->fw_work_state = CYCCG_WORK_STATE_NONE;
	spin_unlock(&cyccg->slock);
	fw_dbg("auto_fw_update done, %d\n", err);

	/*
	 * Auto update FW image for attached CCG based Power Adapter or Cable
	 * device if they were configed to be supported.
	 */
	for (i = 0; i < cyccg->ccg_info.num_port; i++) {
		port = cyccg->ports[i];
		if (!port || port->dev_type < HPI_DEV_TYPE_PORT)
			continue;

		cyccg_port_queue_cc_auto_fw_update(port);
	}
}

void cyccg_queue_auto_fw_update(struct cyccg *cyccg)
{
	if (!IS_ENABLED(CONFIG_AUTO_HOST_CCG_FW_UPDATE))
		return;

	spin_lock(&cyccg->slock);
	if (cyccg->fw_work_state == CYCCG_WORK_STATE_NONE) {
		cyccg->fw_work_state = CYCCG_WORK_STATE_QUEUED;
		schedule_work(&cyccg->fw_work);
	}
	spin_unlock(&cyccg->slock);

	fw_vdbg("CCGx fw auto update work has been %s\n",
		cyccg->fw_work_state ? "queued" : "running");
}