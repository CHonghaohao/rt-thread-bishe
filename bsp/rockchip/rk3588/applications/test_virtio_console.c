#include <rtthread.h>
#include <rtdevice.h>

#define VIRTIO_CONSOLE_DEVICE_NAME "vport0p1"
#define READ_WAIT_TIME 1000  // 等待时间，单位：毫秒
#define SEND_COUNT 100       // 发送次数
#define SEND_INTERVAL 500    // 发送间隔，单位：毫秒

static void test_virtio_console_write(void)
{
    rt_device_t dev;
    rt_size_t size;
    char recv_buf[128];
    int i;

    /* 查找Virtio-Console设备 */
    dev = rt_device_find(VIRTIO_CONSOLE_DEVICE_NAME);
    if (dev == RT_NULL)
    {
        rt_kprintf("Virtio-Console device %s not found!\n", VIRTIO_CONSOLE_DEVICE_NAME);
        return;
    }

    // 添加调试信息，确认设备状态
    rt_kprintf("Device state: 0x%x\n", dev->flag);
    
    /* 打开设备 */
    if (rt_device_open(dev, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("Failed to open Virtio-Console device %s!\n", VIRTIO_CONSOLE_DEVICE_NAME);
        return;
    }


    /* 循环发送100次数据 */
    char dynamic_buf[64];  // 定义一个足够大的缓冲区存储动态生成的消息
    for (i = 0; i < SEND_COUNT; i++)
    {
        // 使用sprintf动态生成包含序号的消息
        sprintf(dynamic_buf, "Hello, I'm RTthread! seq: %d", i + 1);

        rt_kprintf("===== Sending iteration %d/%d =====\n", i + 1, SEND_COUNT);
        
        // 获取实际生成的字符串长度
        size_t actual_length = strlen(dynamic_buf);

        /* 向设备写入数据 */
        size = rt_device_write(dev, 0, dynamic_buf, actual_length);
        if (size > 0)
        {
            rt_kprintf("Write %d bytes to Virtio-Console %s: %s\n", size, VIRTIO_CONSOLE_DEVICE_NAME, dynamic_buf);
        }
        else
        {
            rt_kprintf("Failed to write to Virtio-Console device %s!\n", VIRTIO_CONSOLE_DEVICE_NAME);
            rt_kprintf("Write error code: %d\n", rt_get_errno());
        }

        // 等待一段时间，确保有数据到达
        rt_thread_mdelay(READ_WAIT_TIME);

        rt_kprintf("--------------------------------\n");
        /* 从设备读取数据 */
        size = rt_device_read(dev, 0, recv_buf, sizeof(recv_buf));
        if (size > 0)
        {
            recv_buf[size] = '\0';
            rt_kprintf("Read %d bytes from Virtio-Console %s: %s\n", size, VIRTIO_CONSOLE_DEVICE_NAME, recv_buf);
        }
        else
        {
            rt_kprintf("Failed to read from Virtio-Console device %s!\n", VIRTIO_CONSOLE_DEVICE_NAME);
            rt_kprintf("Read error code: %d\n", rt_get_errno());
        }
        // 每次发送后添加间隔
        rt_thread_mdelay(SEND_INTERVAL);
    }

    /* 关闭设备 */
    // rt_device_close(dev);
}
MSH_CMD_EXPORT(test_virtio_console_write, Test_VirtioConsole_device);