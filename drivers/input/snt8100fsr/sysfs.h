/*****************************************************************************
* File: sysfs.h
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
#ifndef SYSFS_H
#define SYSFS_H

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/
extern volatile int snt8100_flagVerbose;
#define VERBOSE(format,...) do {            \
                                if (snt8100_flagVerbose) printk(KERN_WARNING "[EDGE] %s:" format, __func__, ##__VA_ARGS__, "\n"); \
                            } while (0)

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/

/*==========================================================================*/
/* PROTOTYPES                                                               */
/*==========================================================================*/
#ifdef DYNAMIC_PWR_CTL
int snt_activity_request(void);

#endif

int snt_sysfs_init(struct snt8100fsr *snt8100fsr, bool enable);

#endif // SYSFS_H

