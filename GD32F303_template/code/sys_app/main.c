/************************************************** 
 * @Author: shuren
 * @Date: 2022-02-17 15:08:45
 * @LastEditTime: 2022-02-18 15:29:21
 * @LastEditors: shuren
 * @Description: 
 * @FilePath: \GD32F303\code\sys_app\main.c
 * @桃之夭夭，灼灼其华。之子于归， 宜其室家。
 **************************************************/


/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"
/*
*************************************************************************
*                        函数声明&&任务句柄
*************************************************************************
*/
extern void bsp_init();/* 用于初始化板载相关资源 */
/* 用于创建任务 */
static TaskHandle_t AppTaskCreate_Handle = NULL;
static void AppTaskCreate(void);

/* Test_Task任务实现 */
static TaskHandle_t Test_Task_Handle = NULL;
static void Test_Task(void* pvParameters);

int main()
{
  BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
	bsp_init();
  /* 开发板硬件初始化 */

   /* 创建AppTaskCreate任务 */
  xReturn = xTaskCreate((TaskFunction_t )AppTaskCreate,  /* 任务入口函数 */
                        (const char*    )"AppTaskCreate",/* 任务名字 */
                        (uint16_t       )512,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )1, /* 任务的优先级 */
                        (TaskHandle_t*  )&AppTaskCreate_Handle);/* 任务控制块指针 */ 
  /* 启动任务调度 */           
  if(pdPASS == xReturn)
    vTaskStartScheduler();   /* 启动任务，开启调度 */
  else
    return -1;  
  
  while(1);   /* 正常不会执行到这里 */    
}
/***********************************************************************
  * @ 函数名  ： AppTaskCreate
  * @ 功能说明： 为了方便管理，所有的任务创建函数都放在这个函数里面
  * @ 参数    ： 无  
  * @ 返回值  ： 无
  **********************************************************************/
static void AppTaskCreate(void)
{
  BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
  
  taskENTER_CRITICAL();           //进入临界区
  
  /* 创建Test_Task任务 */
  xReturn = xTaskCreate((TaskFunction_t )Test_Task, /* 任务入口函数 */
                        (const char*    )"Test_Task",/* 任务名字 */
                        (uint16_t       )128,   /* 任务栈大小 */
                        (void*          )NULL,	/* 任务入口函数参数 */
                        (UBaseType_t    )1,	    /* 任务的优先级 */
                        (TaskHandle_t*  )&Test_Task_Handle);/* 任务控制块指针 */
  if(pdPASS == xReturn)NULL;
  vTaskDelete(AppTaskCreate_Handle); //删除AppTaskCreate任务
  
  taskEXIT_CRITICAL();            //退出临界区
}
static void Test_Task(void* pvParameters)
{
    //用于保存上次时间。调用后系统自动更新
    static portTickType PreviousWakeTime; 
    /* 获取当前系统时间 */ 
    PreviousWakeTime = xTaskGetTickCount(); 
    while (1)
    {   
      //=========================同步部分====================================
      /* 调用绝对延时函数,任务时间间隔为*/ 
      vTaskDelayUntil( &PreviousWakeTime, pdMS_TO_TICKS(300)); 
      led_toggle();
    }
}

