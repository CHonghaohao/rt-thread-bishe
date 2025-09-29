#include <rtthread.h>

// ---- Build fixes -----
// undefined reference to `__ctype_ptr__' 
#include <ctype.h>
#define _CTYPE_DATA_0_127 \
    _C, _C, _C, _C, _C, _C, _C, _C, \
    _C, _C|_S, _C|_S, _C|_S,    _C|_S,  _C|_S,  _C, _C, \
    _C, _C, _C, _C, _C, _C, _C, _C, \
    _C, _C, _C, _C, _C, _C, _C, _C, \
    _S|_B,  _P, _P, _P, _P, _P, _P, _P, \
    _P, _P, _P, _P, _P, _P, _P, _P, \
    _N, _N, _N, _N, _N, _N, _N, _N, \
    _N, _N, _P, _P, _P, _P, _P, _P, \
    _P, _U|_X,  _U|_X,  _U|_X,  _U|_X,  _U|_X,  _U|_X,  _U, \
    _U, _U, _U, _U, _U, _U, _U, _U, \
    _U, _U, _U, _U, _U, _U, _U, _U, \
    _U, _U, _U, _P, _P, _P, _P, _P, \
    _P, _L|_X,  _L|_X,  _L|_X,  _L|_X,  _L|_X,  _L|_X,  _L, \
    _L, _L, _L, _L, _L, _L, _L, _L, \
    _L, _L, _L, _L, _L, _L, _L, _L, \
    _L, _L, _L, _P, _P, _P, _P, _C
#define _CTYPE_DATA_128_255 \
    0,  0,  0,  0,  0,  0,  0,  0, \
    0,  0,  0,  0,  0,  0,  0,  0, \
    0,  0,  0,  0,  0,  0,  0,  0, \
    0,  0,  0,  0,  0,  0,  0,  0, \
    0,  0,  0,  0,  0,  0,  0,  0, \
    0,  0,  0,  0,  0,  0,  0,  0, \
    0,  0,  0,  0,  0,  0,  0,  0, \
    0,  0,  0,  0,  0,  0,  0,  0, \
    0,  0,  0,  0,  0,  0,  0,  0, \
    0,  0,  0,  0,  0,  0,  0,  0, \
    0,  0,  0,  0,  0,  0,  0,  0, \
    0,  0,  0,  0,  0,  0,  0,  0, \
    0,  0,  0,  0,  0,  0,  0,  0, \
    0,  0,  0,  0,  0,  0,  0,  0, \
    0,  0,  0,  0,  0,  0,  0,  0, \
    0,  0,  0,  0,  0,  0,  0,  0

char _ctype_b[128 + 256] = {
    _CTYPE_DATA_128_255,
    _CTYPE_DATA_0_127,
    _CTYPE_DATA_128_255
};

char __EXPORT *__ctype_ptr__ = (char *) _ctype_b + 127;
// ---- Build fixes -----

#if defined MICRO_ROS_USE_SERIAL

#include "micro_ros_rtt.h"
#include <stdio.h>
#include <stdbool.h>
#include <sys/time.h>

#define DBG_SECTION_NAME  "micro_ros_serial"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

static int sem_initialized = 0;
static struct rt_semaphore rx_sem;
static rt_device_t micro_ros_serial;

#ifndef MICRO_ROS_SERIAL_NAME
    #define MICRO_ROS_SERIAL_NAME "vport0p1"
#endif

#define micro_rollover_useconds 4294967295

int clock_gettime(clockid_t unused, struct timespec *tp)
{
    (void)unused;
     //1s + 100
    uint64_t m = rt_tick_get() * 1000 / RT_TICK_PER_SECOND * 1000;
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    
    //这个地方可能有问题
    return 0;
}

static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&rx_sem);

    return RT_EOK;
}

bool rtt_transport_open(struct uxrCustomTransport * transport)
{
    micro_ros_serial = rt_device_find(MICRO_ROS_SERIAL_NAME);
    if (!micro_ros_serial)
    {
        LOG_E("Failed to open device %s", MICRO_ROS_SERIAL_NAME);
        return 0;
    }
    if(sem_initialized == 0)
    {
        rt_sem_init(&rx_sem, "micro_ros_rx_sem", 0, RT_IPC_FLAG_FIFO);
        sem_initialized = 1;
    }
    // rt_device_open(micro_ros_serial, RT_DEVICE_FLAG_RDWR);
    rt_device_open(micro_ros_serial, RT_DEVICE_FLAG_INT_RX);
    rt_device_set_rx_indicate(micro_ros_serial, uart_input);
    return 1;
}

bool rtt_transport_close(struct uxrCustomTransport * transport)
{
    rt_device_close(micro_ros_serial);
    rt_sem_detach(&rx_sem);
    sem_initialized = 0;
    return 1;
}

size_t rtt_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode)
{    
    // rt_kprintf("rtt_transport_write !!!\n");
    // rt_tick_t tick = rt_tick_get(); 
    // uint64_t tick_ms = (uint64_t)tick * 1000 / RT_TICK_PER_SECOND;
    // rt_kprintf("WRITE 系统启动时间：%llu ms\n", tick_ms);
    if(micro_ros_serial == NULL)
    {
        rtt_transport_open(NULL);
    }
    // rt_kprintf("send : %s , buflen : %d, len: %d \n", buf, rt_strlen(buf), len);

    // rt_kprintf("buf 的十六进制数据 (len = %d):\n", len);
    // for (size_t i = 0; i < len; i++)
    // {
    //     rt_kprintf("%02X ", buf[i]); // 每个字节两位十六进制
    //     if ((i + 1) % 16 == 0)       // 每 16 个字节换行
    //         rt_kprintf("\n");
    // }
    // if (len % 16 != 0) // 如果最后一行不足 16 个字节，补一个换行
    //     rt_kprintf("\n");
    return rt_device_write(micro_ros_serial, 0, buf, len);
}

size_t rtt_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{   
    // rt_kprintf("rt_transport_read !!!\n");
    //len = rt_device_read(micro_ros_serial, 0, &buf, sizeof(&buf));
    int tick = rt_tick_get();
    for (int i = 0; i < len; ++i)
    {
        // rt_kprintf("Read i: %d\n", i);
        // rt_tick_t tick = rt_tick_get(); 
        // uint64_t tick_ms = (uint64_t)tick * 1000 / RT_TICK_PER_SECOND;
        // rt_kprintf("READ0 系统启动时间：%llu ms\n", tick_ms);
        if(sem_initialized == 0)
        {
            rt_sem_init(&rx_sem, "micro_ros_rx_sem", 0, RT_IPC_FLAG_FIFO);
            sem_initialized = 1;
        }
        
        while (rt_device_read(micro_ros_serial, -1, &buf[i], 1) != 1)
        {
            //rt_kprintf("rt_device_read go on !!!");
            rt_sem_take(&rx_sem, timeout / 4);   //sem_take timeout can not release;
            // tick = rt_tick_get(); 
            // tick_ms = (uint64_t)tick * 1000 / RT_TICK_PER_SECOND;
            // rt_kprintf("时间：%llu ms\n", tick_ms);
            if( (rt_tick_get() - tick) > timeout )
            {
                // LOG_E("Read timeout");
                // rt_kprintf("Read timeout");
                return i;
            }
        }
        // tick = rt_tick_get(); 
        // tick_ms = (uint64_t)tick * 1000 / RT_TICK_PER_SECOND;
        // rt_kprintf("READ1 系统启动时间：%llu ms\n", tick_ms);
    }

    // rt_tick_t tick = rt_tick_get(); 
    // uint64_t tick_ms = (uint64_t)tick * 1000 / RT_TICK_PER_SECOND;
    // rt_kprintf("READ 系统启动时间：%llu ms\n", tick_ms);
    // if(sem_initialized == 0)
    // {
    //     rt_sem_init(&rx_sem, "micro_ros_rx_sem", 0, RT_IPC_FLAG_FIFO);
    //     sem_initialized = 1;
    // }
    
    // while (rt_device_read(micro_ros_serial, -1, &buf, len) == 0)
    // {
    //         //rt_kprintf("rt_device_read go on !!!");
    //     rt_sem_take(&rx_sem, timeout / 4);   //sem_take timeout can not release;
    //     if( (rt_tick_get() - tick) > timeout )
    //     {
    //         rt_kprintf("Read timeout");
    //         return 0;
    //     }
    // }

    // rt_kprintf("recv : %s , buflen : %d, len: %d \n", buf, rt_strlen(buf), len);

    // rt_kprintf("buf 的十六进制数据 (len = %d):\n", len);
    // for (size_t i = 0; i < len; i++)
    // {
    //     rt_kprintf("%02X ", buf[i]); // 每个字节两位十六进制
    //     if ((i + 1) % 16 == 0)       // 每 16 个字节换行
    //         rt_kprintf("\n");
    // }
    // if (len % 16 != 0) // 如果最后一行不足 16 个字节，补一个换行
    //     rt_kprintf("\n");
    return len;
}

#endif // MICRO_ROS_USE_SERIAL
