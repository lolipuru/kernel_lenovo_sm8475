#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include "goodix_ts_core.h"

struct tp_platform_data {
    u32 key_number;
    u32 keys[4];
    u32 key_y_coords[4];
    u32 key_x_coords[4];
    u32 y_min;
    u32 max_touch_number;
};

struct tp_sys_info {
        struct class *tp_class;
        int index;
        struct device *dev;
        struct goodix_ts_core *ts_core;
};

struct game_pad_info {
        struct device *dev;
        struct tp_platform_data *pdata;
        struct input_dev *input_dev;
};

struct game_pad_info *lenovo_gamepad_data;

static atomic_t tp_device_count;
static int double_gesture_switch;
static u32 orientation_mode;
static u32 corner_inhibition_level;
static struct goodix_ts_cmd edge_inhibition_cmd = {
	.buf = { 0x00, 0x00, 0x06, 0x17, 0x00, 0x48, 0x00, 0x00 }
};

static bool charger_mode;
static struct goodix_ts_cmd enter_charger_cmd = {
	.buf = { 0x00, 0x00, 0x05, 0xaf, 0x01, 0xb5, 0x00, 0x00 }
};
static struct goodix_ts_cmd exit_charger_cmd = {
	.buf = { 0x00, 0x00, 0x05, 0xaf, 0x00, 0xb4, 0x00, 0x00 }
};

static bool game_mode;
static struct goodix_ts_cmd enter_game_cmd = {
	.buf = { 0x00, 0x00, 0x05, 0x27, 0x01, 0x2d, 0x00, 0x00 }
};
static struct goodix_ts_cmd exit_game_cmd = {
	.buf = { 0x00, 0x00, 0x05, 0x27, 0x00, 0x2c, 0x00, 0x00 }
};

static bool idle_1500hz;
static struct goodix_ts_cmd enter_idle_1500hz_cmd = {
	.buf = { 0x00, 0x00, 0x05, 0xc2, 0x01, 0xc8, 0x00, 0x00 }
};
static struct goodix_ts_cmd exit_idle_1500hz_cmd = {
	.buf = { 0x00, 0x00, 0x05, 0xc2, 0x00, 0xc7, 0x00, 0x00 }
};

extern int goodix_ts_fw_updating;
extern int goodix_gesture_enable(int enable);
static int lenovo_fake_gamepad_init(void);

int panel_orientation_mode(void)
{
	return orientation_mode;
}
EXPORT_SYMBOL(panel_orientation_mode);

void zui_input_report_atr_key(int x, int y, int press)
{
}

void zui_input_report_atr_key_1(int x, int y, int press)
{
}

void zui_input_report_atr_key_2(int x, int y, int press)
{
}

void zui_input_report_atr_key_n(int x, int y, int press, int slot_flag)
{
}

int enter_charger_mode(struct goodix_ts_core *cd)
{
	if (cd && cd->hw_ops && cd->hw_ops->send_cmd)
		return cd->hw_ops->send_cmd(cd, &enter_charger_cmd);
	return -EINVAL;
}

int exit_charger_mode(struct goodix_ts_core *cd)
{
	if (cd && cd->hw_ops && cd->hw_ops->send_cmd)
		return cd->hw_ops->send_cmd(cd, &exit_charger_cmd);
	return -EINVAL;
}

bool is_charger_mode(void)
{
	return charger_mode;
}

int set_edge_inhibition(struct goodix_ts_core *cd)
{
	if (goodix_ts_fw_updating) {
		pr_err("%s: updating fw\n", __func__);
		return -EACCES;
	}
	if (cd && cd->hw_ops && cd->hw_ops->send_cmd)
		return cd->hw_ops->send_cmd(cd, &edge_inhibition_cmd);
	return -EINVAL;
}

bool is_default_edge_inhibition(void)
{
	return (edge_inhibition_cmd.buf[4] | edge_inhibition_cmd.buf[5]) == 0;
}

int enter_tp_idle_1500hz(struct goodix_ts_core *cd)
{
	if (cd && cd->hw_ops && cd->hw_ops->send_cmd)
		return cd->hw_ops->send_cmd(cd, &enter_idle_1500hz_cmd);
	return -EINVAL;
}

int exit_tp_idle_1500hz(struct goodix_ts_core *cd)
{
	if (cd && cd->hw_ops && cd->hw_ops->send_cmd)
		return cd->hw_ops->send_cmd(cd, &exit_idle_1500hz_cmd);
	return -EINVAL;
}

bool is_tp_idle_1500hz(void)
{
	return idle_1500hz;
}

int enter_tp_game_sensitivity(struct goodix_ts_core *cd)
{
	if (cd && cd->hw_ops && cd->hw_ops->send_cmd)
		return cd->hw_ops->send_cmd(cd, &enter_game_cmd);
	return -EINVAL;
}

int exit_tp_game_sensitivity(struct goodix_ts_core *cd)
{
	if (cd && cd->hw_ops && cd->hw_ops->send_cmd)
		return cd->hw_ops->send_cmd(cd, &exit_game_cmd);
	return -EINVAL;
}

bool is_tp_game_sensitivity(void)
{
	return game_mode;
}

static ssize_t ft_gesture_wakeup_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return snprintf(buf, PAGE_SIZE, "%d\n", double_gesture_switch);
}

static ssize_t ft_gesture_wakeup_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;

	if (sscanf(buf, "%d", &val) < 0)
		return -EINVAL;
	double_gesture_switch = val ? 1 : 0;
	goodix_gesture_enable(double_gesture_switch);
	return count;
}

static ssize_t goodix_ts_charger_mode_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return snprintf(buf, PAGE_SIZE, "charger mode %s\n", charger_mode ? "on" : "off");
}

static ssize_t goodix_ts_charger_mode_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
	struct tp_sys_info *ts = dev_get_drvdata(dev);
	struct goodix_ts_core *cd = ts ? ts->ts_core : NULL;
	int ret;

	if (!buf || count == 0)
		return -EINVAL;

	if (goodix_ts_fw_updating) {
		pr_err("%s: updating fw\n", __func__);
		return -EACCES;
	}

	if (buf[0] == '0') {
		charger_mode = false;
		ret = exit_charger_mode(cd);
		if (ret < 0)
			pr_err("%s: exit charger mode fail\n", __func__);
	} else {
		charger_mode = true;
		ret = enter_charger_mode(cd);
		if (ret < 0)
			pr_err("%s: enter charger mode fail\n", __func__);
	}
	return count;
}

static ssize_t fts_point_mode_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static ssize_t fts_point_mode_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	sscanf(buf, "%d", &val);
	return count;
}

static ssize_t ft_finger_match_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	msleep(35);
        return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static ssize_t ft_finger_match_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	sscanf(buf, "%d", &val);
	return count;
}

static ssize_t orientation_mode_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return snprintf(buf, PAGE_SIZE, "%d\n", orientation_mode);
}

static ssize_t orientation_mode_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
	struct tp_sys_info *ts = dev_get_drvdata(dev);
	struct goodix_ts_core *cd = ts ? ts->ts_core : NULL;
	int val = 0, ret;

	if (!buf || count == 0)
		return -EINVAL;
	if (sscanf(buf, "%d", &val) < 0)
		return -EINVAL;
	if (val >= 3) {
		pr_err("%s: invalid orientation mode %d\n", __func__, val);
		return count;
	}
	orientation_mode = val;
	edge_inhibition_cmd.buf[4] = (u8)(val << 6);
	if (goodix_ts_fw_updating) {
		pr_err("%s: updating fw\n", __func__);
		return count;
	}
	ret = set_edge_inhibition(cd);
	if (ret < 0)
		pr_err("%s: set edge inhibition fail\n", __func__);
	return count;
}

static ssize_t corner_inhibition_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return snprintf(buf, PAGE_SIZE, "corner inhibition level %d\n", corner_inhibition_level);
}

static ssize_t corner_inhibition_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
	struct tp_sys_info *ts = dev_get_drvdata(dev);
	struct goodix_ts_core *cd = ts ? ts->ts_core : NULL;
	int val = 0, ret;

	if (!buf || count == 0)
		return -EINVAL;
	if (sscanf(buf, "%d", &val) < 0)
		return -EINVAL;
	if (val >= 4) {
		pr_err("%s: invaild corner inhibition level\n", __func__);
		return count;
	}
	corner_inhibition_level = val;
	edge_inhibition_cmd.buf[5] = (u8)(0x08 | (val << 6));
	if (goodix_ts_fw_updating) {
		pr_err("%s: updating fw\n", __func__);
		return count;
	}
	ret = set_edge_inhibition(cd);
	if (ret < 0)
		pr_err("%s: set corner inhibition fail\n", __func__);
	return count;
}

static ssize_t game_mode_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return snprintf(buf, PAGE_SIZE, "game mode %s\n", game_mode ? "on" : "off");
}

static ssize_t game_mode_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
	struct tp_sys_info *ts = dev_get_drvdata(dev);
	struct goodix_ts_core *cd = ts ? ts->ts_core : NULL;
	int val = 0, ret;

	if (!buf || count == 0)
		return -EINVAL;
	if (sscanf(buf, "%d", &val) < 0)
		return -EINVAL;
	if (val == 1) {
		game_mode = true;
		ret = enter_tp_game_sensitivity(cd);
		if (ret < 0)
			pr_err("%s: enter tp game sensitivity fail\n", __func__);
		idle_1500hz = true;
		ret = enter_tp_idle_1500hz(cd);
		if (ret < 0)
			pr_err("%s: enter tp idle 1500hz fail\n", __func__);
	} else if (val == 0) {
		game_mode = false;
		ret = exit_tp_game_sensitivity(cd);
		if (ret < 0)
			pr_err("%s: exit tp game sensitivity fail\n", __func__);
		idle_1500hz = false;
		ret = exit_tp_idle_1500hz(cd);
		if (ret < 0)
			pr_err("%s: exit tp idle 1500hz fail\n", __func__);
	}
	return count;
}

static char zui_buf[30];
static char zui_buf_0[30];
static char zui_buf_1[30];
static char zui_buf_2[30];

static ssize_t zui_touch_key_show_base(struct device *dev,
                struct device_attribute *attr, char *buf, int slot_flag)
{
	switch (slot_flag) {
		case 0 :
			return snprintf(buf, PAGE_SIZE, "%s\n", zui_buf_0);
			break;
		case 1 :
			return snprintf(buf, PAGE_SIZE, "%s\n", zui_buf_1);
			break;
		case 2 :
			return snprintf(buf, PAGE_SIZE, "%s\n", zui_buf_2);
			break;
		default :
			return snprintf(buf, PAGE_SIZE, "%s\n", zui_buf_0);
	}
        return snprintf(buf, PAGE_SIZE, "%s\n", zui_buf);
}

static ssize_t zui_touch_key_store_base(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count, int slot_flag)
{
       int x, y, press, pos;
       char x_str[30], y_str[30], press_str[30];
       char *temp_str, *temp_str2;

       switch (slot_flag) {
		case 0 :
			if (count < 30)
				strcpy(zui_buf_0, buf);
			break;
		case 1 :
                        if (count < 30)
                                strcpy(zui_buf_1, buf);
			break;
		case 2 :
			if (count < 30)
                                strcpy(zui_buf_2, buf);
			break;
		default :
                        if (count < 30)
                                strcpy(zui_buf_0, buf);
	}

       if (count < 30)
      	 	strcpy(zui_buf, buf);

       temp_str = strstr(zui_buf, ",");
       if (temp_str == NULL) {
		pr_info("zui_touch_key_store x format error\n");
		goto err;
	}
       pos = temp_str - zui_buf;
       strncpy(x_str, zui_buf, pos);
       x_str[pos] = '\0';

       temp_str2 = temp_str + 1;
       temp_str = strstr(temp_str2, ",");
       if (temp_str == NULL) {
		pr_info("zui_touch_key_store y format error\n");
		goto err;
	}
	pos = temp_str - temp_str2;
	strncpy(y_str, temp_str2, pos);
	y_str[pos] = '\0';

	temp_str2 = temp_str + 1;
	strcpy(press_str, temp_str2);

	pr_info("zui_touch_key_store str x=%s, y=%s, press=%s, slot_flag=%d\n", x_str, y_str, press_str, slot_flag);

	sscanf(x_str, "%d", &x);
	sscanf(y_str, "%d", &y);
	sscanf(press_str, "%d", &press);

	pr_info("zui_touch_key_store x=%d, y=%d, press=%d, slot_flag=%d\n", x, y, press, slot_flag);

	switch (slot_flag) {
		case 0 :
			zui_input_report_atr_key(x, y, press);
			break;
		case 1 :
			zui_input_report_atr_key_1(x, y, press);
			break;
		case 2 :
			zui_input_report_atr_key_2(x, y, press);
			break;
		case 3 :
		case 4 :
		case 5 :
		case 6 :
		case 7 :
		case 8 :
		case 9 :
			zui_input_report_atr_key_n(x, y, press, slot_flag);
			break;
		default :
			zui_input_report_atr_key(x, y, press);
	}

err:
       return count;
}

static ssize_t zui_touch_key_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return zui_touch_key_show_base(dev, attr, buf, 0);
}

static ssize_t zui_touch_key_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
	return zui_touch_key_store_base(dev, attr, buf, count, 0);
}

static ssize_t zui_touch_key_show_1(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return zui_touch_key_show_base(dev, attr, buf, 1);
}

static ssize_t zui_touch_key_store_1(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        return zui_touch_key_store_base(dev, attr, buf, count, 1);
}

static ssize_t zui_touch_key_show_2(struct device *dev,
                struct device_attribute *attr, char *buf)
{
      return zui_touch_key_show_base(dev, attr, buf, 2);
}

static ssize_t zui_touch_key_store_2(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        return zui_touch_key_store_base(dev, attr, buf, count, 2);
}

static ssize_t zui_touch_key_show_3(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return zui_touch_key_show_base(dev, attr, buf, 3);
}

static ssize_t zui_touch_key_store_3(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        return zui_touch_key_store_base(dev, attr, buf, count, 3);
}

static ssize_t zui_touch_key_show_4(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return zui_touch_key_show_base(dev, attr, buf, 4);
}

static ssize_t zui_touch_key_store_4(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        return zui_touch_key_store_base(dev, attr, buf, count, 4);
}

static ssize_t zui_touch_key_show_5(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return zui_touch_key_show_base(dev, attr, buf, 5);
}

static ssize_t zui_touch_key_store_5(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        return zui_touch_key_store_base(dev, attr, buf, count, 5);
}

static ssize_t zui_touch_key_show_6(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return zui_touch_key_show_base(dev, attr, buf, 6);
}

static ssize_t zui_touch_key_store_6(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        return zui_touch_key_store_base(dev, attr, buf, count, 6);
}

static ssize_t zui_touch_key_show_7(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return zui_touch_key_show_base(dev, attr, buf, 7);
}

static ssize_t zui_touch_key_store_7(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        return zui_touch_key_store_base(dev, attr, buf, count, 7);
}

static ssize_t zui_touch_key_show_8(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return zui_touch_key_show_base(dev, attr, buf, 8);
}

static ssize_t zui_touch_key_store_8(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        return zui_touch_key_store_base(dev, attr, buf, count, 8);
}

static ssize_t zui_touch_key_show_9(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return zui_touch_key_show_base(dev, attr, buf, 9);
}

static ssize_t zui_touch_key_store_9(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        return zui_touch_key_store_base(dev, attr, buf, count, 9);
}

static int synaptics_report_rate_switch_flag;

static ssize_t syna_report_rate_switch_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return snprintf(buf, PAGE_SIZE, "%d\n", synaptics_report_rate_switch_flag);
}

static ssize_t syna_report_rate_switch_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
       int val = 0;

       sscanf(buf, "%d", &val);
       synaptics_report_rate_switch_flag = val ? 1 : 0;
       return count;
}

static ssize_t zui_pad_key_store_base(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int databuf[2] = {0, 0};
	struct input_dev *input_dev;

	if (lenovo_gamepad_data == NULL ||
		lenovo_gamepad_data->input_dev == NULL) {
		pr_info("zui gamepad not ready.");
		return count;
	}
	input_dev = lenovo_gamepad_data->input_dev;

	if (2 == sscanf(buf, "%d %d", &databuf[1], &databuf[0])) {
		pr_info("zui gamepad report key 0x%x, action = %d.", databuf[1], databuf[0]);
		if (databuf[0] == 1) {
			input_report_key(input_dev, databuf[1], 1);
			input_sync(input_dev);
		} else if (databuf[0] == 0) {
			input_report_key(input_dev, databuf[1], 0);
			input_sync(input_dev);
		}
	}

	return count;
}

static ssize_t zui_pad_abs_store_base(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int databuf[2] = {0, 0};
	struct input_dev *input_dev;

	if (lenovo_gamepad_data == NULL ||
		lenovo_gamepad_data->input_dev == NULL) {
		pr_info("zui gamepad not ready.");
		return count;
	}
	input_dev = lenovo_gamepad_data->input_dev;

	if (2 == sscanf(buf, "%x %x", &databuf[1], &databuf[0])) {
		pr_info("zui gamepad report abs 0x%x, value = %d.", databuf[1], databuf[0]);
		input_report_abs(input_dev, databuf[1], databuf[0]);
		input_sync(input_dev);
	}
	return count;
}

static ssize_t zui_pad_key_store_0(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        return zui_pad_key_store_base(dev, attr, buf, count);
}

static ssize_t zui_pad_key_show_0(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t zui_pad_key_store_1(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        return zui_pad_key_store_base(dev, attr, buf, count);
}

static ssize_t zui_pad_key_show_1(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t zui_pad_abs_store_0(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        return zui_pad_abs_store_base(dev, attr, buf, count);
}

static ssize_t zui_pad_abs_show_0(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t zui_pad_abs_store_1(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        return zui_pad_abs_store_base(dev, attr, buf, count);
}

static ssize_t zui_pad_abs_show_1(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t zui_pad_enable_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
	int rc = 0, val;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (lenovo_gamepad_data == NULL) {
		pr_info("zui gamepad not ready.");
		return count;
	}

	if (1 == val) {
		lenovo_fake_gamepad_init();
	} else if (0 == val) {
		if (lenovo_gamepad_data->input_dev) {
			input_unregister_device(lenovo_gamepad_data->input_dev);
			lenovo_gamepad_data->input_dev = NULL;
		}
	}
	return count;
}

static ssize_t zui_pad_enable_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	return 0;
}

static const char *lenovo_fake_phys[2] = { "gamepad/input0", "gamepad/input1" };
static int lenovo_fake_gamepad_init(void)
{
    int ret = 0;
    struct tp_platform_data *pdata;
    struct input_dev *input_dev;
    int pdata_size = sizeof(struct tp_platform_data);

    pr_info("%s enter\n", __func__);
    lenovo_gamepad_data->pdata = kzalloc(pdata_size, GFP_KERNEL);
    if (!lenovo_gamepad_data->pdata) {
        pr_err("allocate memory for platform_data fail");
        return -ENOMEM;
    }
    pdata = lenovo_gamepad_data->pdata;
    (void)pdata;

    input_dev = input_allocate_device();
    if (!input_dev) {
        pr_info("Failed to allocate memory for input device");
        return -ENOMEM;
    }

    /* Init and register Input device */
    input_dev->phys = lenovo_fake_phys[0];
    input_dev->name = "lenovo game pad";
    input_dev->id.bustype = BUS_VIRTUAL;
    input_dev->id.vendor = 0x17ef;
    input_dev->id.product = 0x0000;
    input_dev->id.version = 1;

    __set_bit(EV_SYN, input_dev->evbit);
    __set_bit(EV_KEY, input_dev->evbit);
    __set_bit(ABS_X, input_dev->absbit);
    __set_bit(ABS_Y, input_dev->absbit);
    __set_bit(ABS_Z, input_dev->absbit);
    __set_bit(ABS_RX, input_dev->absbit);
    __set_bit(ABS_RY, input_dev->absbit);
    __set_bit(ABS_RZ, input_dev->absbit);
    __set_bit(ABS_HAT0X, input_dev->absbit);
    __set_bit(ABS_HAT0Y, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_X, -32768, 32768, 2, 4);
    input_set_abs_params(input_dev, ABS_Y, -32768, 32768, 2, 4);
    input_set_abs_params(input_dev, ABS_Z, 0, 1023, 2, 4);
    input_set_abs_params(input_dev, ABS_RX, -32768, 32768, 2, 4);
    input_set_abs_params(input_dev, ABS_RY, -32768, 32768, 2, 4);
    input_set_abs_params(input_dev, ABS_RZ, 0, 1023, 2, 4);

    input_set_capability(input_dev, EV_KEY, BTN_GAMEPAD);
    input_set_capability(input_dev, EV_KEY, BTN_WEST);
    input_set_capability(input_dev, EV_KEY, BTN_SOUTH);
    input_set_capability(input_dev, EV_KEY, BTN_EAST);
    input_set_capability(input_dev, EV_KEY, BTN_NORTH);
    input_set_capability(input_dev, EV_KEY, BTN_TL);
    input_set_capability(input_dev, EV_KEY, BTN_TR);
    input_set_capability(input_dev, EV_KEY, BTN_TL2);
    input_set_capability(input_dev, EV_KEY, BTN_TR2);
    input_set_capability(input_dev, EV_KEY, BTN_SELECT);
    input_set_capability(input_dev, EV_KEY, BTN_START);
    input_set_capability(input_dev, EV_KEY, BTN_THUMBL);
    input_set_capability(input_dev, EV_KEY, BTN_THUMBR);
    input_set_capability(input_dev, EV_KEY, BTN_MODE);

    ret = input_register_device(input_dev);
    if (ret) {
        input_set_drvdata(input_dev, NULL);
        input_free_device(input_dev);
        input_dev = NULL;
        return ret;
    }

    lenovo_gamepad_data->input_dev = input_dev;
    pr_info("%s end\n", __func__);

    return 0;
}

static int lenovo_vr_gamepad(struct device *dev)
{
	pr_info("%s\n", __func__);
	lenovo_gamepad_data = kzalloc(sizeof(struct game_pad_info), GFP_KERNEL);
	if (!lenovo_gamepad_data) {
		pr_err("allocate memory for lenovo_gamepad_data fail");
		return -ENOMEM;
	}
	lenovo_gamepad_data->dev = dev;
	return 0;
}

static struct device_attribute attrs[] = {
        __ATTR(gesture_on, 0664,
                        ft_gesture_wakeup_show,
                        ft_gesture_wakeup_store),
        __ATTR(ft_charger_mode, 0664,
                        goodix_ts_charger_mode_show,
                        goodix_ts_charger_mode_store),
        __ATTR(point, 0664,
                        fts_point_mode_show,
                        fts_point_mode_store),
        __ATTR(finger_match, 0664,
                        ft_finger_match_show,
                        ft_finger_match_store),
        __ATTR(orientation_mode, 0664,
                        orientation_mode_show,
                        orientation_mode_store),
        __ATTR(corner_inhibition, 0664,
                        corner_inhibition_show,
                        corner_inhibition_store),
        __ATTR(game_mode, 0664,
                        game_mode_show,
                        game_mode_store),
	__ATTR(touch_key, 0664,
                        zui_touch_key_show,
                        zui_touch_key_store),
        __ATTR(touch_key1, 0664,
                        zui_touch_key_show_1,
                        zui_touch_key_store_1),
        __ATTR(touch_key2, 0664,
                        zui_touch_key_show_2,
                        zui_touch_key_store_2),
        __ATTR(touch_key3, 0664,
                        zui_touch_key_show_3,
                        zui_touch_key_store_3),
         __ATTR(touch_key4, 0664,
                        zui_touch_key_show_4,
                        zui_touch_key_store_4),
        __ATTR(touch_key5, 0664,
                        zui_touch_key_show_5,
                        zui_touch_key_store_5),
        __ATTR(touch_key6, 0664,
                        zui_touch_key_show_6,
                        zui_touch_key_store_6),
        __ATTR(touch_key7, 0664,
                        zui_touch_key_show_7,
                        zui_touch_key_store_7),
        __ATTR(touch_key8, 0664,
                        zui_touch_key_show_8,
                        zui_touch_key_store_8),
        __ATTR(touch_key9, 0664,
                        zui_touch_key_show_9,
                        zui_touch_key_store_9),
       __ATTR(report_rate, 0664,
                        syna_report_rate_switch_show,
                        syna_report_rate_switch_store),
        __ATTR(pad_key0, 0664,
                        zui_pad_key_show_0,
                        zui_pad_key_store_0),
        __ATTR(pad_key1, 0664,
                        zui_pad_key_show_1,
                        zui_pad_key_store_1),
         __ATTR(pad_abs0, 0664,
                        zui_pad_abs_show_0,
                        zui_pad_abs_store_0),
        __ATTR(pad_abs1, 0664,
                        zui_pad_abs_show_1,
                        zui_pad_abs_store_1),
      __ATTR(input_enable, 0664,
                        zui_pad_enable_show,
                        zui_pad_enable_store),
};

int tp_gesture_ctl_class(struct goodix_ts_core *core_data)
{
       int attr_count = 0;
	int err;
 	struct tp_sys_info *ts;

       ts = kzalloc(sizeof(*ts), GFP_KERNEL);
       if (!ts)
               return -ENOMEM;
       memset(ts, 0, sizeof(*ts));
       ts->tp_class = class_create(THIS_MODULE, "tp_gesture_ctl_class");
       if (IS_ERR(ts->tp_class))
       {
       	printk("create tp class err!");
	}
	else
     	       atomic_set(&tp_device_count, 0);

       ts->index = atomic_inc_return(&tp_device_count);
       ts->dev = device_create(ts->tp_class, NULL,
       MKDEV(0, ts->index), NULL, "tp_gesture_ctl_class");
       if (IS_ERR(ts->dev))
       {
           printk("create device err!");
       }
       lenovo_vr_gamepad(ts->dev);
       for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
              err = sysfs_create_file(&ts->dev->kobj,
                               &attrs[attr_count].attr);
              if (err < 0) {
                   pr_err("%s: Failed to create sysfs attributes\n", __func__);
                   return err;
             }
        }
        ts->ts_core = core_data;
        dev_set_drvdata(ts->dev, ts);

	return 0;
}
