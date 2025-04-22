/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-5-30      Bernard      the first version
 */

#include <rtthread.h>
#include "rk3588.h"
#include "hal_canfd.h"
#include "hal_pinctrl.h"
#include "ros_test.h"
#include "time.h"


rt_thread_t can_entry = NULL;
rt_thread_t can_entry1 = NULL;


void  can_test_entry(void *args)
{
    while(1)
    {
        rt_kprintf("can send !\n");
        rt_thread_mdelay(1000);
    }
}


void  can_test_entry1(void *args)
{
    while(1)
    {
        rt_kprintf("can1 send !\n");
        rt_thread_mdelay(1000);
    }
}


void  rk3588pinctl_debug(void)
{
    rt_kprintf("use BANK : %d, GPIO PIN %d,%d,funcs 0x%x\n",GPIO_BANK4,GPIO_PIN_A3,GPIO_PIN_A4,PIN_CONFIG_MUX_FUNC10);
}





int main(int argc, char** argv)
{   

    //rt_system_scheduler_start();
    rt_kprintf("Hi, this is RT-Thread!!\n");
    rt_kprintf("can init !\n");

    
    rk3588pinctl_debug();
    
    microros_ping_pong();
   

    // can_entry =  rt_thread_create("can entry", //线程名字
    //     can_test_entry, //入口函数
    //     RT_NULL, //入口函数参数
    //     4096, //栈大小
    //     10, //线程优先级
    //     5000); //线程时间片大小
    
    // if(can_entry == NULL)
    // {
    //     rt_kprintf("can entry error !\n");
    //     return;
    // }
    // rt_thread_startup(can_entry);




    // can_entry1 =  rt_thread_create("can1 entry", //线程名字
    //     can_test_entry1, //入口函数
    //     RT_NULL, //入口函数参数
    //     4096, //栈大小
    //     10, //线程优先级
    //     5000); //线程时间片大小
    
    // if(can_entry1 == NULL)
    // {
    //     rt_kprintf("can1 entry error !\n");
    //     return;
    // }
    // rt_thread_startup(can_entry1);
     
    //rt_kprintf("main exit !\n");
 
    return 0;
}

