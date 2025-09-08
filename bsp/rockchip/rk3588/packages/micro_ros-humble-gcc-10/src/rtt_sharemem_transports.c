#include <rtthread.h>

// ---- Build fixes -----
// undefined reference to `__ctype_ptr__' 
#if defined MICRO_ROS_USE_SHAREMEM

#include "micro_ros_rtt.h"
#include <stdio.h>
#include <stdbool.h>
#include <sys/time.h>
#include "ivc.h"

#define DBG_SECTION_NAME  "micro_ros_shm"
#define DBG_LEVEL         DBG_LOG
#include <rtdbg.h>

static int sem_initialized = 0;
static struct rt_semaphore rx_sem;
static rt_device_t micro_ros_shm;

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
    micro_ros_shm = rt_device_find(MICRO_ROS_SHAREMEM_NAME);
    if (!micro_ros_shm)
    {
        LOG_E("Failed to open device %s", MICRO_ROS_SHAREMEM_NAME);
        return 0;
    }
    rt_kprintf("rtt transport open\n");
    return 1;
}

bool rtt_transport_close(struct uxrCustomTransport * transport)
{
    rt_kprintf("rtt transport close\n");
    return 1;
}

size_t rtt_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode)
{    
    rt_kprintf("rtt transport white\n");
    // rt_kprintf("rtt_transport_write !!!\n");
    // rt_tick_t tick = rt_tick_get(); 
    // uint64_t tick_ms = (uint64_t)tick * 1000 / RT_TICK_PER_SECOND;
    // rt_kprintf("WRITE 系统启动时间：%llu ms\n", tick_ms);
    
    // if(micro_ros_shm == NULL)
    // {
    //     rtt_transport_open(NULL);
    // }
    // return rt_device_write(micro_ros_shm, 0, buf, len);
    // ringbuf_write(g_ivc_serial.tx_buf, buf, len);
    rt_kprintf("send : %s , buflen : %d, len: %d \n", buf, rt_strlen(buf), len);
    // kick_guest0();
    return ivc_write(micro_ros_shm, 0, (const void *)buf, len);
}

size_t rtt_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{   
    rt_kprintf("rtt transport read\n");
    int tick = rt_tick_get();
    for (int i = 0; i < len; ++i)
    {
        if(sem_initialized == 0)
        {
            rt_sem_init(&rx_sem, "micro_ros_rx_sem", 0, RT_IPC_FLAG_FIFO);
            sem_initialized = 1;
        }
        
        // while (rt_device_read(micro_ros_shm, -1, &buf[i], 1) != 1)
        while (ivc_read(micro_ros_shm, -1, &buf[i], 1) != 1)
        {
            rt_sem_take(&rx_sem, timeout / 4);  

            if( (rt_tick_get() - tick) > timeout )
            {
                return i;
            }
        }
    }
    // ivc_read(g_ivc_serial, 0, (const void *)buf, len);
    return len;
}

#endif // MICRO_ROS_USE_SHAREMEM
