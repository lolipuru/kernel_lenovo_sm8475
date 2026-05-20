#ifndef __AW8680X_H__
#define __AW8680X_H__

/*********************************************************
 *
 * struct
 *
 ********************************************************/
#include "aw_type.h"
#include "aw_protocol_data.h"
#include "aw_protocol_type.h"

/* Default Top P0(220, 200), P1(910, 930)
  * Bottom P2(220,1510), P3(910, 2240)
  */
#define LSCREEN_XRANGE_LEFT                220 /* P0 x*/
#define LSCREEN_XRANGE_RIGHT               910 /* P1 x */
#define RSCREEN_XRANGE_LEFT                220 /* P2 x */
#define RSCREEN_XRANGE_RIGHT               910 /* P3 x */
#define TOPSCREEN_YRANGE_BOTTOM            200 /* P0 y */
#define TOPSCREEN_YRANGE_TOP               930 /* P1 y */
#define BOTTOMSCREEN_YRANGE_BOTTOM         1510 /* P2 y */
#define BOTTOMSCREEN_YRANGE_TOP            2240 /* P3 y */

#define SOC_APP_DATA_TYPE                  0x21
#define CPU_IN_UBOOT                       0x00000001
#define CPU_IN_FLASH_BOOT                  0x00010002
#define FLASH_BASE_ADDR                    0x01001000
#define FLASH_MAX_ADDR                     0x01010000
#define SRAM_BASE_ADDR                     0x20001000
#define SRAM_MAX_ADDR                      0x20002000
#define ERASE_BYTE_MAX                     512
#define WRITE_FLASH_MAX                    64
#define READ_FLASH_MAX                     64
#define WRITE_SRAM_MAX                     (64 * 3)

#define REG_ADDR                           0x01
#define SOC_ADDR                           0x00000000
#define SOC_DATA_LEN                       0x0000
#define SOC_READ_LEN                       0x0000

#define KEY_DATA_ADDR                      0x12
#define REG_SCAN_MODE_SWITCH_ADDR          0x56

#define RESET_INIT_TIME                    5
#define CHIP_INIT_TIME                     15
#define JUMP_INIT_TIME                     20

/* register */
#define RAW_DATA_NUM                       3
#define FORCE_DATA_NUM                     3
#define BASE_DATA_NUM                      3
#define DIFF_DATA_NUM                      3
#define ADC_DATA_NUM                       4
#define RAW_DATA_LEN                       6
#define FORCE_DATA_LEN                     6
#define BASE_DATA_LEN                      6
#define DIFF_DATA_LEN                      6
#define ADC_DATA_LEN                       8
#define PRESS_THRESHOLD_LEN                8
#define TP_DATA_LEN                        10
#define RAW_DATA_ADDR                      0x19
#define FORCE_DATA_ADDR                    0x20
#define BASE_DATA_ADDR                     0x12
#define DIFF_DATA_ADDR                     0x13
#define ADC_DATA_ADDR                      0x14
#define PRESS_THRESHOLD_ADDR               0x5b
#define TP_DATA_ADDR                       0xb8

#define HALF_SCREEN_LENGTH                 1230 //2460*2
#define TP_DATA_ADDR                       0xb8
#define TP_DATA_LEN                         10
enum return_flag_enum {
	ERR_FLAG = 1,
	NOT_NEED_UPDATE = 2,
	RST_REGISTER_ERR = 3,
	WAKE_REGISTER_ERR = 4,
	IRQ_REGISTER_ERR = 5,
	IRQ_THREAD_ERR = 6,
};

enum report_mode_enum {
	IRQ_MODE_SET = true,
	NOT_IRQ_MODE_SET = false,
	WAKE_MODE_SET = true,
	NOT_WAKE_MODE_SET = false,
};

enum scan_mode_switch_enum {
	SWITCH = 1,
	HIGH_SPEED = 2,
	LOW_SPEED = 3,
	POWER_OFF = 4,
};

struct data_container {
	unsigned int len;
	unsigned char data[];
};

struct aw8680x_sensor_info {
	unsigned char key_data;
	unsigned char press_threshold[PRESS_THRESHOLD_LEN];
	unsigned short adc_data[ADC_DATA_NUM];
	unsigned short raw_data[RAW_DATA_NUM];
	unsigned short force_data[FORCE_DATA_NUM];
	unsigned short base_data[BASE_DATA_NUM];
	unsigned short diff_data[DIFF_DATA_NUM];
};

struct aw8680x {
	struct i2c_client *i2c;
	struct device *dev;
	struct input_dev *input;
	struct workqueue_struct *tp_workqueue;
	struct work_struct report_tp_work;
	struct delayed_work bin_work;
	struct aw_bin *sram_bin;
	struct aw_bin *flash_bin;
	struct aw8680x_sensor_info info;
	struct task_struct *thread;
	bool adb_update_flag;
	bool update_finished_flag;
	bool ic_status;
	bool irq_mode;
	bool wake_mode;
	bool polling_set;
	bool suspend_mode;
	bool ftp_enable;
	int ack_flag;
	uint32_t screen_xrange[4];
	uint32_t screen_yrange[4];
	uint8_t current_status[2];
	uint8_t last_status[2];

	GUI_TO_SOC_S p_gui_data_s;
	unsigned char p_protocol_tx_data[PROTOCOL_TOTAL_LEN];
	unsigned char p_protocol_rx_data[PROTOCOL_TOTAL_LEN];

	char read_data[256];
	unsigned char irq_key_data;
	unsigned char module_id;
	unsigned char reg_addr;
	unsigned short read_len;
	unsigned int polling_cycle;
	unsigned int app_current_addr;
	unsigned int flash_app_addr;
	unsigned int sram_addr;
	unsigned int cpu_in_flash;
	unsigned int flash_app_len;
	struct notifier_block fb_notif;/*register to control tp report*/
	struct notifier_block fb_notif_drm;/*register to control tp report*/
	int top_screen_x;
	int top_screen_y;
	int lower_screen_x;
	int lower_screen_y;

	int irq_gpio;
	int reset_gpio;
	int wake_gpio;
	struct pinctrl *active_pinctrl;
	struct pinctrl_state *pinctrl_reset_state;
	struct pinctrl_state *pinctrl_int_state;
	struct pinctrl_state *pinctrl_wakeup_state;
};

#endif
