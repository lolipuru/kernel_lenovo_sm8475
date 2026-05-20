/*****************************************************************************
* File: utils.h
*
* (c) 2016 Sentons Inc. - All Rights Reserved.
*
* All information contained herein is and remains the property of Sentons
* Incorporated and its suppliers if any. The intellectual and technical
* concepts contained herein are proprietary to Sentons Incorporated and its
* suppliers and may be covered by U.S. and Foreign Patents, patents in
* process, and are protected by trade secret or copyright law. Dissemination
* of this information or reproduction of this material is strictly forbidden
* unless prior written permission is obtained from Sentons Incorporated.
*
* SENTONS PROVIDES THIS SOURCE CODE STRICTLY ON AN "AS IS" BASIS,
* WITHOUT ANY WARRANTY WHATSOEVER, AND EXPRESSLY DISCLAIMS ALL
* WARRANTIES, EXPRESS, IMPLIED OR STATUTORY WITH REGARD THERETO, INCLUDING
* THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
* PURPOSE, TITLE OR NON-INFRINGEMENT OF THIRD PARTY RIGHTS. SENTONS SHALL
* NOT BE LIABLE FOR ANY DAMAGES SUFFERED BY YOU AS A RESULT OF USING,
* MODIFYING OR DISTRIBUTING THIS SOFTWARE OR ITS DERIVATIVES.
*****************************************************************************/
#include <linux/module.h>
#include "device.h"

#ifndef UTILS_H
#define UTILS_H

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/

/*==========================================================================*/
/* PROTOTYPES                                                               */
/*==========================================================================*/
uint32_t get_time_in_ms(void);
int string_to_uint32(const char *str, uint32_t *value);
int write_register(struct snt8100fsr *snt8100fsr, int reg, void *value);
int read_register(struct snt8100fsr *snt8100fsr, int reg, void *value);
int do_write_reg_script_file(struct snt8100fsr *snt8100fsr, const char *file_location, bool wake_device);
void snt8100_read_profile(const char *file_location, char *buf_ptr, int *sz_read);
int snt8100_write_cfgbank_reg(uint16_t cfgbank_id, uint16_t cfgbank_reg, uint16_t reg_val);
int snt8100_read_cfgbank_reg(uint16_t cfgbank_id, uint16_t cfgbank_reg, uint16_t* reg_val);
int snt_activity_request_force(int force);

int cust_write_registers(void *dev, int reg, int num, void *value);
int cust_read_registers(void *dev, int reg, int num, void *value);
void set_tap_gesture_enable(void *dev, uint16_t tap_id, uint16_t enable);
void set_slider_gesture_enable(void *dev, uint16_t slider_id, uint16_t enable);
void set_slider_gesture(void *dev, uint16_t slider_id, uint16_t *reg_val);
void set_touch_enable(void *dev, uint16_t reg_val);
bool is_gas_gesture(uint8_t bar_id);
#endif // PROTOTYPE_H
