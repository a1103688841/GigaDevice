
#include <stdbool.h>

#include "bsp.h"
#include "my_queue.h"
#include "my_task.h"

#include "timer_tool.h"
#include "st7789v.h"
#include "key.h"
#include "input.h"
#include "beep.h"
#include "nor_little_fs.h"
#include "project.h"
#include "store_test_data.h"
#include "nor_sum_store.h"

//HY3131
#include "meas_config.h"
#include "meas_cont.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "log.h"
#include "display.h"
#include "display_controller.h"
#include "setup.h"
#include "power_manager.h"
#include "port_check.h"
#include "function_tips.h"
#include "illuminate.h"
#include "dmm_status.h"
#include "bluetooth_controller.h"

/************************************************************************/
/*      Print Message                                                   */
/************************************************************************/
#define MODULE_PRINT_INFO_EN // 屏蔽不定义 关闭此模块打印信息
#ifdef MODULE_PRINT_INFO_EN
//#define PRINT_DETAIL_MODE // module detail print message

#include "myprint.h"  /* 其他地方myprint.h不可出现此之前 */

#ifdef PRINTF_INFO_EN
static int prn_level = 0;/* setting module print level */
#endif

#else

#define PRINT(fmt, ...)             ( (void)0 )
#define PRN_HEXS(a,b)               ( (void)0 )
#define PRN_LEVEL(a,b,fmt, ...)     ( (void)0 )

#endif
/************************************************************************/

/* freertos */
#define TIME_COUNTER_HZ  10000
volatile unsigned long tim_1ms_num = 0;
// 系统统计时间使用
void configureTimerForRunTimeStats(void)
{
    //上电只初始化掉一次
    tim_1ms_num   = 0;
}

unsigned long getRunTimeCounterValue(void)
{
    unsigned long tim_cnt_val;
    tim_cnt_val  = get_systick_val();
    return tim_1ms_num * (TIME_COUNTER_HZ/configTICK_RATE_HZ) + tim_cnt_val/(get_systick_freq()/TIME_COUNTER_HZ);
}

extern void xPortSysTickHandler(void);
void freertos_1ms_thread()
{
    if (xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED) { //系统已经运行
        xPortSysTickHandler();
    }
    tim_1ms_num++;
}
/* end freertos */

struct task_cfg_s {
    TaskFunction_t task_func;
    const char *  task_name;
    const uint16_t stack_size;
    UBaseType_t priority;
};

struct task_param_s {
    const struct task_cfg_s *p_task_cfg;
    unsigned short id;
    unsigned long  call_cnt;
    void *  parameters;
    TaskHandle_t taskHandle;
    QueueHandle_t queue;
};

enum event_id_e {
    EVENT_KEY,       // 按键输入
    EVENT_CHG_DIAL,  // 转盘改变
    EVENT_CH_SELECT, // 测量改变
    EVENT_CH_RANGE,  // 挡位改变 带宽修改
    EVENT_DMM_MANU,  // 手动同步配置
    EVENT_DMM_CAL,   // 手动校准同步配置
};

struct event_s {
    enum event_id_e event_id;
    unsigned short sender_id;
    void *p_data; //传结构、数据
};

void make_event(struct event_s *p_event,unsigned short sender_id,enum event_id_e event_id, const void* p_data)
{
    p_event->event_id  = event_id;
    p_event->sender_id = sender_id;
    p_event->p_data    = (void*)p_data;
}

void createTasks(struct task_param_s *p_task_param, const struct task_cfg_s *p_task_cfg, int num)
{
    for(int i=0;i < num; i++){
        p_task_param->id = i;
        p_task_param->call_cnt   = 0;
        p_task_param->p_task_cfg = p_task_cfg;
        p_task_param->parameters = p_task_param;
        xTaskCreate( 
                p_task_param->p_task_cfg->task_func,
                p_task_param->p_task_cfg->task_name,
                p_task_param->p_task_cfg->stack_size,
                p_task_param->parameters,
                p_task_param->p_task_cfg->priority,
                &p_task_param->taskHandle);

        p_task_param->queue = xQueueCreate(10, sizeof(struct event_s));
        p_task_param++;
        p_task_cfg++;
    }
}

void task_call_msg();
QueueHandle_t get_task_queue(const char *task_name);
void send_task_event(unsigned short sender_id,enum event_id_e event_id,char data,char task_name[])
{
    struct event_s event;
    QueueHandle_t task_queue = get_task_queue(task_name);
    if (task_queue) {
        make_event(&event,sender_id,event_id,(void*)&data);
        xQueueSendToBack(task_queue, &event, 0);
    }
}

/*
 * dbg_cmd
 */
#include "dbg_cmd.h"
#ifdef DBG_CMD_EN
static bool dbg_cmd_func()
{
    if (dbg_cmd_exec("help", "", "")) {
        DBG_CMD_PRN(".task\r\n");
        return false;
    }
    if (dbg_cmd_exec("exit", "", "")) {
#ifdef MODULE_PRINT_INFO_EN
        prn_level = 0;
#endif
        return false;
    }
    if (dbg_cmd_exec(".task", "", "")) {
        dbg_cmd_print_msg_en();
    }
    if (dbg_cmd_exec("taskmsg", "", "")) {
#ifdef MODULE_PRINT_INFO_EN
        DBG_CMD_PRN("TaskPrf:%d\r\n",prn_level);
#endif
        return true;
    }
#ifdef MODULE_PRINT_INFO_EN
    if (dbg_cmd_exec("taskprf", "1", "<0~1>")) {
        prn_level = get_param_char(0);
        return true;
    }
#endif
    if (dbg_cmd_exec("TaskList", "", "")) {
        char *buf;
        buf = pvPortMalloc(sizeof(512));// 注意防止超出数组大小
        vTaskList(buf);
        DBG_CMD_PRN("%ldHz\r\n",configCPU_CLOCK_HZ);
        DBG_CMD_PRN("R:running R:Ready B:Blocked S:suspended\r\n");
        DBG_CMD_PRN("task_name     state   prior   stack     Id\r\n");
        DBG_CMD_PRN("%s",buf);
        vPortFree(buf);
        buf = NULL;
        return true;
    }
    if (dbg_cmd_exec("TaskState", "", "")) {
        char *buf;
        buf = pvPortMalloc(sizeof(512));// 注意防止超出数组大小
        vTaskGetRunTimeStats(buf);
        DBG_CMD_PRN("task_name  time_count(0.1ms)  usage_pec\r\n");
        DBG_CMD_PRN("%s",buf);
        vPortFree(buf);
        buf = NULL;
        return true;
    }

    if (dbg_cmd_exec("HeapSize", "", "")) {
        DBG_CMD_PRN("heap:%ld min:%ld total:%ld\r\n",xPortGetFreeHeapSize(),xPortGetMinimumEverFreeHeapSize(),configTOTAL_HEAP_SIZE);
        return true;
    }

    if (dbg_cmd_exec("taskcall", "", "")) {
        task_call_msg();
        return true;
    }

    if (dbg_cmd_exec("taskevent", "211s", "<id> <event> <data> <task_name>")) {
        send_task_event(get_param_short(0),get_param_char(0),get_param_char(1),get_param_string(0));
        return true;
    }
    
    return false;
}
#endif

// task app
extern struct queue_s rxd1_queue, txd1_queue;
extern struct queue_s rxd2_queue, txd2_queue;
void dbg_cmd_task(void *pvParameters)
{
    struct task_param_s *p_task = (struct task_param_s*)pvParameters;
    static int cnt = 0;
    char rxd;
    for(;;) {
        vTaskDelay(100);
#if 0
        while (out_queue(&rxd1_queue, &rxd) != QUEUE_EMPTY) {
            put_txd1_queue(rxd);
        }
#endif
        
        while (out_queue(&rxd2_queue, &rxd) != QUEUE_EMPTY) {
#ifdef DBG_CMD_EN
            dbg_cmd_rxd(rxd);
#endif
        }
        cnt++;
        if (cnt > 10) {
            cnt = 0;
#ifdef DBG_CMD_EN
            dbg_cmd_1s_thread();
#endif
        }
        p_task->call_cnt++;
    }
}   

void input_task(void *pvParameters)
{
    struct task_param_s *p_task = (struct task_param_s*)pvParameters;
    struct event_s event;
    QueueHandle_t dis_task_queue = get_task_queue("dis");
    /* 获取当前系统时间 调用后系统自动更新*/ 
    static portTickType PreviousWakeTime; 
    PreviousWakeTime = xTaskGetTickCount(); 
    for (;;) {
        key_10ms_thread_isr();
        input_10ms_thread_isr();
        beep_10ms_thread_isr();

        // input key send to dis
        if (dis_task_queue) {
            if(key_enable) {  
                key_enable = 0;
                if (select_key_wkup_lock && (key_val == _KEY_SELECT)) {
                    select_key_wkup_lock = 0;
                } else if(select_key_wkup_lock && (key_val == _KEY_LONG_SELECT)) {
                    select_key_wkup_lock = 0;
                } else {
                    select_key_wkup_lock = 0;
                    make_event(&event,p_task->id,EVENT_KEY,(void*)&key_val);
                    xQueueSendToBack(dis_task_queue, &event, 0);
                }
            }            
        }

        p_task->call_cnt++;
        vTaskDelayUntil( &PreviousWakeTime, pdMS_TO_TICKS(10)); 
    }
}


void prj_task(void *pvParameters)
{
    struct task_param_s *p_task = (struct task_param_s*)pvParameters;
    struct event_s event;
    QueueHandle_t dis_task_queue = get_task_queue("dis");
    QueueHandle_t hy3131_cfg_task_queue = get_task_queue("hy_cfg");

    for (;;) {
        project_100ms_thread();
        if(get_dial_sync()) {
            if (dis_task_queue) {
                make_event(&event,p_task->id,EVENT_CHG_DIAL,(void*)&p_task->call_cnt);
                xQueueSendToBack(dis_task_queue, &event, 0);
            }
            if (hy3131_cfg_task_queue) {
                make_event(&event,p_task->id,EVENT_CHG_DIAL,(void*)&p_task->call_cnt);
                xQueueSendToBack(hy3131_cfg_task_queue, &event, 0);
            }
        }
        if(get_select_sync()) {
            if (hy3131_cfg_task_queue) {
                make_event(&event,p_task->id,EVENT_CH_SELECT,(void*)&p_task->call_cnt);
                xQueueSendToBack(hy3131_cfg_task_queue, &event, 0);
            }
        }
        if(get_range_sync()) {
            if (hy3131_cfg_task_queue) {
                make_event(&event,p_task->id,EVENT_CH_RANGE,(void*)&p_task->call_cnt);
                xQueueSendToBack(hy3131_cfg_task_queue, &event, 0);
            }
        }
        p_task->call_cnt++;
        vTaskDelay(100);
    }
}

void bsp_task(void *pvParameters)
{
    struct task_param_s *p_task = (struct task_param_s*)pvParameters;
    for (;;) {
        bsp_1s_thread();
        p_task->call_cnt++;
        vTaskDelay(1000);
    }
}

void ancillary_task(void *pvParameters)
{
    struct task_param_s *p_task = (struct task_param_s*)pvParameters;
    static unsigned char wait_rtc_init_complete = 0;
    static unsigned char equip_back_light_init  = false;

    for (;;) {
        vTaskDelay(500);

        light_time_timing();
        apo_time_timing();
        key_led_timing();

        tips_timing(&message_tips[POWER_20_WRN]    );
        tips_timing(&message_tips[POWER_10_WRN]    );
        tips_timing(&message_tips[POWER_05_WRN]    );
        tips_timing(&message_tips[CONNECT_V_WRN]   );
        tips_timing(&message_tips[CONNECT_A_WRN]   );
        tips_timing(&message_tips[CONNECT_MA_WRN]  );
        tips_timing(&message_tips[WAIT_MEAS_TEMP]  );
        tips_timing(&message_tips[PRESS_CLEAN_DATA]);
        tips_timing(&message_tips[REC_BL_TURN_OFF] );
        tips_timing(&message_tips[CHECK_CAP_TYPE]  );
        message_label_timing(&save_data_message    );

        power_data_handler();
        power_low_handler();
        port_check_handler();

        if ((get_test_mode() == MEAS_MODE_TEMP_C) || (get_test_mode() == MEAS_MODE_TEMP_F))
            temp_meas_tips_handler();
        if ((get_test_mode() == MEAS_MODE_CAP) && ((get_dmm_range_pos(DMM_MAIN) == 5) || (get_dmm_range_pos(DMM_MAIN) == 6)))
            cap_meas_tips_handler();
        if (get_test_mode() == MEAS_MODE_SHORT)
            duration_sound(get_res_cont_state());

        if (wait_rtc_init_complete == 10) {
            rtc_param_init();
            wait_rtc_init_complete = 255;
        }

        if (wait_rtc_init_complete < 255)
            wait_rtc_init_complete++;

        if (equip_is_startup() && (!equip_back_light_init)) {
            light_time_reset();
            equip_back_light_init = true;
        }

        bluetooth_event_doing();
        record_handler();
    }
}

void dis_task(void *pvParameters)
{
    struct task_param_s *p_task = (struct task_param_s*)pvParameters;
    QueueHandle_t *evnt_queue  = p_task->queue;
    struct event_s event;
    unsigned long mask_num = 0;
    unsigned char relative_buf[12];
    for(;;) {
        BaseType_t status = xQueueReceive(evnt_queue, &event, pdMS_TO_TICKS(100));
        if( status == pdPASS ) {
            switch(event.event_id) {
                case EVENT_KEY:
                    KEYT *p_key;
                    p_key = (KEYT*) event.p_data;
                    key_event_doing();
                    break;
                case EVENT_CHG_DIAL:
                    menubar_function_skip();
                    if (CAL_get_mode_en()) {
                        set_menubar_function(DMM_ADJUSTMENT);
                        set_menubar_page(0);
                    }
                    break;
            }
        }

        set_measure_value(get_fmt_val_str(FMT_MAIN));
        set_measure_unit(get_fmt_unit_str(FMT_MAIN));

        if (get_rel_en() == true) {
            snprintf(relative_buf, 12, "%s%s", get_fmt_val_str(FMT_SUB), get_fmt_unit_str(FMT_SUB));
            set_relative_value(relative_buf);
        } else if (get_fmt_en(FMT_SUB) == true) {
            snprintf(relative_buf, 12, "%s%s", get_fmt_val_str(FMT_SUB), get_fmt_unit_str(FMT_SUB));
            set_deputy_display_value(relative_buf);
        }

        key_highlight_effect();
        display_flush(NULL);
        tft_flush(NULL);
        p_task->call_cnt++;
    }
}

SemaphoreHandle_t mcu_spi_mutex =NULL;
void hy3131_cfg(void *pvParameters)
{
    struct task_param_s *p_task = (struct task_param_s*)pvParameters;
    QueueHandle_t *evnt_queue  = p_task->queue;
    struct event_s event;
    BaseType_t xReturn;
    char syn_s;
    (void)xReturn;
    char main_mode, main_range;
    char dual_mode, dual_range;
    for(;;)
    {
        xReturn = xQueueReceive(evnt_queue, &event, portMAX_DELAY); /* 一直等 */
        //进入临界区，保护全局变量
        taskENTER_CRITICAL();
        {
            main_mode = get_dmm_test_mode(DMM_MAIN);
            main_range = get_dmm_range_pos(DMM_MAIN);
        }
        taskEXIT_CRITICAL();
        hy3131_cfg_meas(main_mode, main_range);
        set_dmm_delay_start();
        p_task->call_cnt++;
        //防止任务饿死
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
void hy3131_manu(void *pvParameters)
{
	struct task_param_s *p_task = (struct task_param_s*)pvParameters;
    QueueHandle_t *evnt_queue  = p_task->queue;
    struct event_s event;
    char main_mode, main_range, cal_en;
    float g_val;
    for(;;)
    {
        BaseType_t status = xQueueReceive(evnt_queue, &event, portMAX_DELAY);
        //进入临界区，保护全局变量
        taskENTER_CRITICAL();
        {
            main_mode = get_dmm_test_mode(DMM_MAIN);
            main_range = get_dmm_range_pos(DMM_MAIN);
            cal_en = CAL_get_mode_en();
        }
        taskEXIT_CRITICAL();
        if( status == pdPASS ) {
            switch(event.event_id) {
                case EVENT_DMM_CAL:
                    switch(main_mode)
                    {
                        case MEAS_MODE_VAC:
                            meas_set_acc(main_range);
                            break;
                        case MEAS_MODE_WAC:
                            meas_set_phase(get_dmm_wac_phase());
                            break;
                        case MEAS_MODE_TEMP_F:
                        case MEAS_MODE_TEMP_C:
                            meas_set_g_val();
                            break;                
                    }
                    break;
                case EVENT_DMM_MANU:
                    switch(main_mode)
                    {
                        case MEAS_MODE_CAP:
                            meas_set_mf(main_range);
                            break;
                        case MEAS_MODE_WAC:
                            meas_set_w();
                            break;
                        case MEAS_MODE_TEMP_F:
                        case MEAS_MODE_TEMP_C:
                            meas_set_kits90();
                            break;
                    }
                    break;
            }
        }
        //防止任务饿死
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
void hy3131_cont(void *pvParameters)
{    
    struct task_param_s *p_task = (struct task_param_s*)pvParameters;
    unsigned long cycle_time_ms = 10;
    char        main_mode, main_range;
    char        dual_mode, dual_range;
    static float   hy3131_f32[DMM_DATA_TYPE_NUM];
    bool        intf_trigger = FALSE; 
    //用于保存上次时间。调用后系统自动更新
    static portTickType PreviousWakeTime; 
    /* 获取当前系统时间 */ 
    PreviousWakeTime = xTaskGetTickCount(); 
    for(;;)
    {   
//=========================同步部分====================================
        /* 调用绝对延时函数,任务时间间隔为*/ 
        vTaskDelayUntil( &PreviousWakeTime, pdMS_TO_TICKS(cycle_time_ms)); 
        
        //临界区，保护全局变量
        taskENTER_CRITICAL();
        {
            main_mode = get_dmm_test_mode(DMM_MAIN);
            main_range = get_dmm_range_pos(DMM_MAIN);
        }
        taskEXIT_CRITICAL(); 
        if(meas_get_sample() == TRUE)
        {
            intf_trigger = FALSE; 
            intf_trigger = meas_read_count(main_mode, main_range,hy3131_f32, cycle_time_ms);
            meas_cfg_peak();
        }
        meas_rate_cnt(intf_trigger, cycle_time_ms);

        //换挡间隔时间管理
        project_dmm_delay(cycle_time_ms);
        if(intf_trigger == TRUE)
        {
            intf_trigger = FALSE;
            //数据处理
            project_dmm_data_deal(hy3131_f32);
        }
        p_task->call_cnt++;
    }
}
void bluetooth_task(void *pvParameters)
{
     //用于保存上次时间。调用后系统自动更新
    static portTickType PreviousWakeTime; 
    static uint8_t cyc_ms = 30;
    static unsigned int last_handler_times = 0;
    uint8_t state;
    /* 获取当前系统时间 */ 
    PreviousWakeTime = xTaskGetTickCount(); 
    for(;;)
    {   
        /* 调用绝对延时函数,任务时间间隔为*/ 
        vTaskDelayUntil( &PreviousWakeTime, pdMS_TO_TICKS(cyc_ms)); 
        //临界区，保护全局变量
        taskENTER_CRITICAL();
        {
            state = get_buletooth_status() | get_record_status();
            if (get_record_status() == false)
                last_handler_times = 2200;
        }
        taskEXIT_CRITICAL(); 
        if(state == true || last_handler_times > 0)
        {
            // info("state:%d", state);
            bt_get_deal(cyc_ms);
            bt_set_inquire_record(cyc_ms);
            bt_send_meas(cyc_ms);
            if (last_handler_times > 0)
                last_handler_times--;
        }
    }   
}
// 事件触发接口
void send_event_dmm_manu()
{
    send_task_event(0x80,EVENT_DMM_MANU,0,"hy_manu");
}
void send_event_dmm_cal()
{
    send_task_event(0x80,EVENT_DMM_CAL,0,"hy_manu");
}
// task config
#define TASK_NUM 10
static struct task_param_s task_param_tbl[TASK_NUM];
const struct task_cfg_s task_cfg_tbl[TASK_NUM] = {
    // task_func   name          stack(4B)  prio
    {dbg_cmd_task,  "dbg_cmd",    512,       30   }, // 100
    {input_task,    "input",      128,       22   }, // 10
    {bsp_task,      "bsp",        128,       2    }, // 1000
    {ancillary_task,"ancillary",  256,       2    }, // 500
    {dis_task,      "dis",        512,       2    }, // 200
    {bluetooth_task,"bt",         128,       2    }, // 100
    {prj_task,      "prj",        512,       24   }, // 100
    {hy3131_manu,   "hy_manu",    128,       28   }, // 
    {hy3131_cfg,    "hy_cfg",     128,       26   }, // 
    {hy3131_cont,   "hy_cont",    256,       20   }, // 1
};
// end task app

void task_call_msg()
{
    PRINT("task_name  call_count\r\n");
    for (int i=0;i < TASK_NUM;i++) {
        PRINT("%-10s %ld\r\n",task_param_tbl[i].p_task_cfg->task_name,task_param_tbl[i].call_cnt);
    }
}

QueueHandle_t get_task_queue(const char *task_name)
{
    for (int i = 0;i < TASK_NUM;i++)
    if (strcmp(task_param_tbl[i].p_task_cfg->task_name, task_name) == NULL) {
        return task_param_tbl[i].queue;
    }
    PRINT( "get %s error!\r\n",task_name);
    return NULL;
}

void free_rtos_init()
{
    timer_tool_init();
    st7789v_init();
    key_init();
    input_init();
    beep_init();
    //nor_little_fs_init();
    CAL_read_data();
    nor_sum_store_init();
    store_test_data_init();

    // --- display part ----
    set_back_light(0);
    read_usr_setup();
    widget_size_init();
    menubar_function_init();
    tft_clear(BLACK, NULL);
    // ---------------------
    hy3131_init();
    project_init();
	bluetooth_init();
#ifdef DBG_CMD_EN
    dbg_cmd_add_list((CMD_FUNC_T)dbg_cmd_func);
#endif
 
    //taskENTER_CRITICAL();           //进入临界区
    //taskEXIT_CRITICAL();            //退出临界区
    mcu_spi_mutex = xSemaphoreCreateMutex();
    //创建任务
    createTasks(task_param_tbl,task_cfg_tbl,TASK_NUM);
    //开启任务调度
    vTaskStartScheduler();
}
