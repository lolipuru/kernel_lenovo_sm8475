/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2018-2020, The Linux Foundation. All rights reserved.
 */
#ifndef FSA4480_I2C_H
#define FSA4480_I2C_H

#include <linux/of.h>
#include <linux/notifier.h>

enum fsa_function {
	FSA_MIC_GND_SWAP,
	FSA_USBC_ORIENTATION_CC1,
	FSA_USBC_ORIENTATION_CC2,
	FSA_USBC_DISPLAYPORT_DISCONNECTED,
	FSA_EVENT_MAX,
};

struct fsa4480_priv {
       struct regmap *regmap;
       struct device *dev;
       struct notifier_block ucsi_nb;
       struct notifier_block ucsi_nb_2;
       atomic_t usbc_mode;
       struct work_struct usbc_analog_work;
       struct work_struct usbc_analog_work_2;
       struct blocking_notifier_head fsa4480_notifier;
       struct blocking_notifier_head fsa4480_notifier_2;
       struct mutex notification_lock;
};

struct fsa4480_reg_val {
       u16 reg;
       u8 val;
};

struct fsa4480_logic_switch_data{
    int irq_gpio;
    struct device   *dev;
};
extern unsigned int cc_port_num;
extern struct fsa4480_logic_switch_data *fsa4480_logic_data;
extern bool audio_switch_C1_enable;
extern bool audio_switch_C2_enable;
extern unsigned int audio_4480_headset_ref;
extern bool audio_boot_flag;
extern struct mutex audio_detect_completed_mutex;
extern struct fsa4480_priv *audio_4480_priv1;
extern bool audio_boot_flag;
extern bool audio_first_delay;
extern struct fsa4480_priv *audio_4480_priv2;


#if IS_ENABLED(CONFIG_QCOM_FSA4480_I2C)
int fsa4480_switch_event(struct device_node *node,
			 enum fsa_function event);
int fsa4480_reg_notifier(struct notifier_block *nb,
			 struct device_node *node);
int fsa4480_unreg_notifier(struct notifier_block *nb,
			   struct device_node *node);
int fsa4480_switch_event_2(struct device_node *node,
			 enum fsa_function event);
int fsa4480_reg_notifier_2(struct notifier_block *nb,
			 struct device_node *node);
int fsa4480_unreg_notifier_2(struct notifier_block *nb,
			   struct device_node *node);
#else
static inline int fsa4480_switch_event(struct device_node *node,
				       enum fsa_function event)
{
	return 0;
}

static inline int fsa4480_reg_notifier(struct notifier_block *nb,
				       struct device_node *node)
{
	return 0;
}

static inline int fsa4480_unreg_notifier(struct notifier_block *nb,
					 struct device_node *node)
{
	return 0;
}
#endif /* CONFIG_QCOM_FSA4480_I2C */

#endif /* FSA4480_I2C_H */

