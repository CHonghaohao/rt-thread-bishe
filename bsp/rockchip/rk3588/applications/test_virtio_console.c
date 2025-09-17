// rt_virtio_console_echo.c
#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <stdint.h>

#define VIRTIO_CONSOLE_DEVICE_NAME "vport0p1"
#define MAX_PAYLOAD                512

static rt_device_t dev;

static rt_ssize_t read_exact(rt_device_t d, void *buf, rt_size_t len)
{
    uint8_t *p = buf;
    rt_size_t left = len;
    while (left)
    {
        rt_size_t r = rt_device_read(d, 0, p, left);
        if (r == 0)
        {
            // 阻塞一会儿等数据（视驱动行为决定是否需要）
            // rt_thread_mdelay(1);
            continue;
        }
        p += r;
        left -= r;
    }
    return len;
}

static rt_ssize_t write_exact(rt_device_t d, const void *buf, rt_size_t len)
{
    const uint8_t *p = buf;
    rt_size_t left = len;
    while (left)
    {
        rt_size_t w = rt_device_write(d, 0, p, left);
        if (w == 0)
        {
            // rt_thread_mdelay(1);
            continue;
        }
        p += w;
        left -= w;
    }
    return len;
}

static int recv_frame(uint8_t *buf, uint16_t *out_len)
{
    uint8_t hdr[2];
    if (read_exact(dev, hdr, 2) != 2) return -1;
    uint16_t len = (uint16_t)hdr[0] | ((uint16_t)hdr[1] << 8);
    if (len > MAX_PAYLOAD) return -1;
    if (read_exact(dev, buf, len) != len) return -1;
    *out_len = len;
    return 0;
}

static int send_frame(const uint8_t *buf, uint16_t len)
{
    uint8_t hdr[2] = {(uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
    if (write_exact(dev, hdr, 2) != 2) return -1;
    if (write_exact(dev, buf, len) != len) return -1;
    return 0;
}

static void virtio_console_echo(void)
{
    dev = rt_device_find(VIRTIO_CONSOLE_DEVICE_NAME);
    if (!dev)
    {
        rt_kprintf("device %s not found\n", VIRTIO_CONSOLE_DEVICE_NAME);
        return;
    }
    if (rt_device_open(dev, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("open %s failed\n", VIRTIO_CONSOLE_DEVICE_NAME);
        return;
    }

    // 先阻塞读：等待 Linux 的 HELLO（解决“不会等待”的问题）
    uint8_t buf[MAX_PAYLOAD];
    uint16_t len = 0;
    if (recv_frame(buf, &len) == 0)
    {
        // 回一帧，完成握手
        send_frame(buf, len);
    }

    // 进入回显循环：收到什么回什么
    while (1)
    {
        if (recv_frame(buf, &len) == 0)
        {
            // 这里也可做校验/处理后再回
            send_frame(buf, len);
        }
        // else
        // {
        //     // 可选：短暂休眠避免空转
        //     rt_thread_mdelay(1);
        // }
    }
}

MSH_CMD_EXPORT(virtio_console_echo, virtio console echo test);
