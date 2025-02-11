#ifndef _LENOVO_SYS_TEMP_IOCTL_H
#define _LENOVO_SYS_TEMP_IOCTL_H

#include <linux/ioctl.h>

#define LENOVO_SYS_TEMP_IOCTL_NAME "lenovo_sys_temp"
#define LENOVO_SYS_TEMP_NAME_LENGTH 20

struct __attribute__((__packed__)) lenovo_sys_temp_ioctl {
	int temperature;
	char name[LENOVO_SYS_TEMP_NAME_LENGTH];
};

enum {
	LENOVO_SYS_TEMP_SET = 0x00,
	LENOVO_SYS_TEMP_MAX_NUM,
};

#define LENOVO_SYS_TEMP_MAGIC_NUM 0xDF


#define LENOVO_SYS_TEMP_SET_TEMP _IOW(LENOVO_SYS_TEMP_MAGIC_NUM, \
				   LENOVO_SYS_TEMP_SET, \
				   struct lenovo_sys_temp_ioctl)

#endif
