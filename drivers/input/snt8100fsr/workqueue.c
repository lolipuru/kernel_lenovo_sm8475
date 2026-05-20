/*****************************************************************************
* File: workqueue.c
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
#include <linux/kernel.h>

#include "workqueue.h"
#include "memory.h"
#include "config.h"
#include "debug.h"

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/
#define QUEUE_NAME "snt8100fsr-queue"

/*==========================================================================*/
/* CONSTANTS                                                                */
/*==========================================================================*/

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/

/*==========================================================================*/
/* LOCAL PROTOTYPES                                                         */
/*==========================================================================*/

/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/
static struct workqueue_struct *wq;

/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/
int snt_workqueue_init(void) {
    wq = create_workqueue(QUEUE_NAME);
    if (!wq) {
        PRINT_CRIT("Unable to create_workqueue(%s)", QUEUE_NAME);
        return -1;
    }

    return 0;
}

void workqueue_cleanup() {
    /* Wait for all the work in the queue to complete before destroying the work queue */
    flush_workqueue(wq);
    destroy_workqueue(wq);
    wq = NULL;
}

bool workqueue_queue_work(void *work, unsigned long delay) {
    //PRINT_FUNC("0x%p, %lums", work, delay);
    delay = msecs_to_jiffies(delay);
    if(NULL == wq){
        PRINT_CRIT("workqueue_queue_work wq = NULL");
        return false;
    } else if(NULL == work){
        PRINT_CRIT("workqueue_queue_work work = NULL");
        return false;
    }
    return queue_delayed_work(wq, work, delay);
}

bool workqueue_mod_work(void *work, unsigned long delay) {
    //PRINT_FUNC("0x%p, %lums", work, delay);
    delay = msecs_to_jiffies(delay);
    return mod_delayed_work(wq, work, delay);
}

bool workqueue_cancel_work(void *work) {
    //PRINT_FUNC("0x%p", work);
    if(NULL == work){
        PRINT_CRIT("workqueue_cancel_work work = NULL");
        return false;
    }
    return cancel_delayed_work(work);
}
