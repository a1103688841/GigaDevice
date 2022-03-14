#ifndef __MY_TASK_H
#define __MY_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// 事件触发接口
extern void send_event_dmm_manu();

extern void freertos_1ms_thread();
extern void free_rtos_init();

#endif

