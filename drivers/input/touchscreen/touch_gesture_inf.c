#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/delay.h>

struct tp_sys_info{
        struct class *tp_class;
        int index;
        struct device *dev;
};
static atomic_t tp_device_count;
//int gesture_dubbleclick_en = 0;
static int double_gesture_switch;

extern void synaptics_gesture_en(int enable);

int synaptics_get_finger_match(void);

static ssize_t ft_gesture_wakeup_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return snprintf(buf, PAGE_SIZE, "%d\n",double_gesture_switch);
}

static ssize_t ft_gesture_wakeup_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
       int val;

       sscanf(buf, "%d", &val);
	if(val)
		double_gesture_switch = 1;
	else
		double_gesture_switch = 0;

	synaptics_gesture_en(double_gesture_switch);

       return count;

}

static ssize_t ft_finger_match_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	int ft_finger_match = 0;

	msleep(35);
	ft_finger_match = synaptics_get_finger_match();

        return snprintf(buf, PAGE_SIZE, "%d\n",ft_finger_match);
}

static ssize_t ft_finger_match_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
       int val;

       sscanf(buf, "%d", &val);

       return count;
}

//add by gongdb begin
#ifdef CONFIG_PRODUCT_MOBA
extern void zui_input_report_atr_key(int x, int y, int press);
extern void zui_input_report_atr_key_1(int x, int y, int press);
extern void zui_input_report_atr_key_2(int x, int y, int press);

static char zui_buf[30];
static char zui_buf_0[30];
static char zui_buf_1[30];
static char zui_buf_2[30];
static ssize_t zui_touch_key_show_base(struct device *dev,
                struct device_attribute *attr, char *buf, int slot_flag)
{
	switch (slot_flag) {
		case 0 :
			return snprintf(buf, PAGE_SIZE, "%s\n",zui_buf_0);
			break;
		case 1 :
			return snprintf(buf, PAGE_SIZE, "%s\n",zui_buf_1);
			break;
		case 2 :
			return snprintf(buf, PAGE_SIZE, "%s\n",zui_buf_2);
			break;
		default :
			return snprintf(buf, PAGE_SIZE, "%s\n",zui_buf_0);
	}
        return snprintf(buf, PAGE_SIZE, "%s\n",zui_buf);
}

static ssize_t zui_touch_key_store_base(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count, int slot_flag)
{
       int x, y, press, pos;
       char x_str[30], y_str[30], press_str[30];
       char *temp_str, *temp_str2;

       switch (slot_flag) {
		case 0 :
			if (sizeof(buf) < 30)
				strcpy(zui_buf_0, buf);
			break;
		case 1 :
                        if (sizeof(buf) < 30)
                                strcpy(zui_buf_1, buf);

			break;
		case 2 :
			if (sizeof(buf) < 30)
                                strcpy(zui_buf_2, buf);

			break;
		default :
                        if (sizeof(buf) < 30)
                                strcpy(zui_buf_0, buf);
	}
	
       if (sizeof(buf) < 30)
      	 	strcpy(zui_buf, buf);

       temp_str = strstr(zui_buf, ",");
    
       if (temp_str == NULL) {
		printk("zui_touch_key_store x format error\n");
		goto err;
	}
       pos = temp_str - zui_buf;
       strncpy(x_str, zui_buf, pos);
       x_str[pos] = '\0';

       temp_str2 = temp_str + 1;

       temp_str = strstr(temp_str2, ",");
       if (temp_str == NULL) {
		printk("zui_touch_key_store y format error\n");
		goto err;
	}
	pos = temp_str - temp_str2;
	strncpy(y_str, temp_str2, pos);
	y_str[pos] = '\0';

	temp_str2 = temp_str + 1;
	strcpy(press_str, temp_str2);

	printk("zui_touch_key_store str x=%s, y=%s, press=%s, slot_flag=%d\n", x_str, y_str, press_str, slot_flag);

	sscanf(x_str, "%d", &x);
	sscanf(y_str, "%d", &y);
	sscanf(press_str, "%d", &press);
	
	printk("zui_touch_key_store x=%d, y=%d, press=%d, slot_flag=%d\n", x, y, press, slot_flag);

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

static int synaptics_report_rate_switch_flag = 0;
extern void synaptics_report_rate_switch(int enable);

static ssize_t syna_report_rate_switch_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return snprintf(buf, PAGE_SIZE, "%d\n", synaptics_report_rate_switch_flag);
}

static ssize_t syna_report_rate_switch_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
       int val;

       sscanf(buf, "%d", &val);
        if(val)
                synaptics_report_rate_switch_flag = 1;
        else
                synaptics_report_rate_switch_flag = 0;

       synaptics_report_rate_switch(synaptics_report_rate_switch_flag);

       return count;
}
#endif
//add by gongdb end
static struct device_attribute attrs[] = {
        __ATTR(gesture_on, 0664,
                        ft_gesture_wakeup_show,
                        ft_gesture_wakeup_store),
        __ATTR(finger_match, 0664,
                        ft_finger_match_show,
                        ft_finger_match_store),
#ifdef CONFIG_PRODUCT_MOBA
	__ATTR(touch_key, 0664,
                        zui_touch_key_show,
                        zui_touch_key_store),
        __ATTR(touch_key1, 0664,
                        zui_touch_key_show_1,
                        zui_touch_key_store_1),
        __ATTR(touch_key2, 0664,
                        zui_touch_key_show_2,
                        zui_touch_key_store_2),
        __ATTR(report_rate, 0664,
                        syna_report_rate_switch_show,
                        syna_report_rate_switch_store),
#endif
};

static int tp_gesture_ctl_class(void)
{
       int attr_count = 0;
	int err;
 	struct tp_sys_info *ts;

       ts = kzalloc(sizeof(*ts), GFP_KERNEL);
       memset(ts, 0, sizeof(*ts));
       ts->tp_class = class_create(THIS_MODULE, "touch");
       if (IS_ERR(ts->tp_class))
       {
       	printk("create tp class err!");
	}
	else
     	       atomic_set(&tp_device_count, 0);

	ts->index = atomic_inc_return(&tp_device_count);
       ts->dev = device_create(ts->tp_class, NULL,
       MKDEV(0, ts->index), NULL, "tp_dev");
	if (IS_ERR(ts->dev))
	{
              printk("create device err!");
	}
       for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
              err = sysfs_create_file(&ts->dev->kobj,
                              &attrs[attr_count].attr);
              if (err < 0) {
                   pr_err("%s: Failed to create sysfs attributes\n", __func__);
                   return err;
             }
        }
        dev_set_drvdata(ts->dev,ts);
        //end tp class to show tp info

	return 0;
}

static int __init tp_gesture_ctl_init(void)
{
	return tp_gesture_ctl_class();
}

static void __exit tp_gesture_ctl_exit(void)
{
	//if (ts & ts->tp_class)
	//	class_destroy(ts->tp_class);
}

module_init(tp_gesture_ctl_init);
module_exit(tp_gesture_ctl_exit);

MODULE_AUTHOR("xxx <xxx@android.com>");
MODULE_DESCRIPTION("tp gesture control class driver");
MODULE_LICENSE("GPL");