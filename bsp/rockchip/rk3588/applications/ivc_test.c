#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <stdint.h>

// -------------- 配置参数（根据实际需求调整）--------------
#define IVC_DEV_NAME   "ivc" // IVC 设备名（需与驱动注册的一致）
#define SEND_DATA_LEN  31    // 发送数据长度（示例：31字节）
#define RECV_BUF_SIZE  64    // 接收缓冲区大小（建议大于最大预期数据长度）
#define READ_BATCH_LEN 10    // 接收端分批次读取长度（示例：每次读10字节）

// -------------- 全局变量（接收端用）--------------
static rt_device_t g_ivc_dev = RT_NULL;         // IVC 设备句柄
static uint8_t g_recv_buf[RECV_BUF_SIZE] = {0}; // 接收缓冲区


/**
 * @brief IVC 发送端示例：向对端发送数据
 * @param send_data 待发送数据（支持字符串/二进制）
 * @param data_len  待发送数据长度（字节）
 * @return 0 成功，-1 失败
 */
static int ivc_send_example(const uint8_t *send_data, uint32_t data_len)
{
    // 1. 检查参数合法性
    if (send_data == NULL || data_len == 0 || data_len > 1024) // 1024是驱动中MAX_VALID_TOTAL
    {
        rt_kprintf("[IVC_SEND] 无效参数：数据为空或长度超限（最大1024字节）\n");
        return -1;
    }

    // 2. 打开IVC设备（若未打开）
    if (g_ivc_dev == RT_NULL)
    {
        rt_device_t dev = rt_device_find(IVC_DEV_NAME);
        //g_ivc_dev = rt_device_open(dev, RT_DEVICE_OFLAG_RDWR);
        if (rt_device_open(g_ivc_dev, RT_DEVICE_FLAG_RDWR) != RT_EOK)
        {
            rt_kprintf("[IVC_SEND] 打开设备 %s 失败\n", IVC_DEV_NAME);
            return -1;
        }
        rt_kprintf("[IVC_SEND] 成功打开设备 %s（句柄：%p）\n", IVC_DEV_NAME, g_ivc_dev);
    }

    // 3. 调用 ivc_write 发送数据（驱动会自动封装“长度+数据”格式）
    rt_size_t write_len = rt_device_write(g_ivc_dev, 0, send_data, data_len);
    if (write_len != data_len)
    {
        rt_kprintf("[IVC_SEND] 发送失败：请求发送%u字节，实际发送%u字节\n",
                   data_len, write_len);
        return -1;
    }

    // 4. 打印发送结果
    rt_kprintf("[IVC_SEND] 发送成功（总长度%u字节）\n", data_len);
    rt_kprintf("[IVC_SEND] 发送数据（十六进制）：");
    for (uint32_t i = 0; i < data_len; i++)
    {
        rt_kprintf("%02x ", send_data[i]);
        if ((i + 1) % 16 == 0) // 每16字节换行，便于阅读
            rt_kprintf("\n              ");
    }
    rt_kprintf("\n[IVC_SEND] 发送数据（字符串）：%.*s\n", data_len, send_data);

    return 0;
}


/**
 * @brief IVC 接收端示例：从对端接收数据（分批次读取）
 * @return 0 成功，-1 失败
 */
static int ivc_recv_example(void)
{
    // 1. 打开IVC设备（若未打开）
    if (g_ivc_dev == RT_NULL)
    {
        g_ivc_dev = rt_device_find(IVC_DEV_NAME);
        // g_ivc_dev = rt_device_open(dev, RT_DEVICE_OFLAG_RDWR);
        if (rt_device_open(g_ivc_dev, RT_DEVICE_FLAG_RDWR) != RT_EOK)
        {
            rt_kprintf("[IVC_RECV] 打开设备 %s 失败\n", IVC_DEV_NAME);
            return -1;
        }
        rt_kprintf("[IVC_RECV] 成功打开设备 %s（句柄：%p）\n", IVC_DEV_NAME, g_ivc_dev);
    }

    // 2. 分批次读取数据（直到读完所有数据）
    rt_size_t total_recv_len = 0;            // 累计接收长度
    rt_memset(g_recv_buf, 0, RECV_BUF_SIZE); // 清空接收缓冲区

    while (1)
    {
        uint32_t read_len = READ_BATCH_LEN; // 本次请求读取长度（分批次）
        // 调用 ivc_read 读取数据（驱动会自动解析总长度，跟踪读取偏移）
        rt_size_t actual_len = rt_device_read(g_ivc_dev, 0, g_recv_buf + total_recv_len, read_len);

        if (actual_len == 0)
        {
            // 实际长度为0：表示所有数据已读完（驱动内部重置偏移量）
            rt_kprintf("[IVC_RECV] 所有数据读取完成（累计%u字节）\n", total_recv_len);
            break;
        }
        else if (actual_len == (rt_size_t)-1)
        {
            // 读取失败（如参数错误、共享内存获取失败）
            rt_kprintf("[IVC_RECV] 读取失败\n");
            total_recv_len = 0;
            break;
        }

        // 3. 更新累计接收长度
        total_recv_len += actual_len;
        if (total_recv_len >= RECV_BUF_SIZE)
        {
            rt_kprintf("[IVC_RECV] 接收缓冲区已满（%u字节），停止读取\n", RECV_BUF_SIZE);
            break;
        }
    }

    // 4. 打印接收结果（仅当累计接收长度>0时）
    if (total_recv_len > 0)
    {
        rt_kprintf("[IVC_RECV] 接收数据（十六进制）：");
        for (uint32_t i = 0; i < total_recv_len; i++)
        {
            rt_kprintf("%02x ", g_recv_buf[i]);
            if ((i + 1) % 16 == 0)
                rt_kprintf("\n              ");
        }
        rt_kprintf("\n[IVC_RECV] 接收数据（字符串）：%.*s\n", total_recv_len, g_recv_buf);
    }

    return total_recv_len > 0 ? 0 : -1;
}


/**
 * @brief MSH 命令：发送测试数据（示例：发送字符串）
 * 用法：ivc_send_test
 */
static void ivc_send_test(void)
{
    // 示例1：发送字符串（长度31字节，与你驱动中示例一致）
    const uint8_t send_str[] = "hello zone0! I'm zone1. test";
    uint32_t str_len = strlen((const char *)send_str);

    // 调用发送示例函数
    if (ivc_send_example(send_str, str_len) != 0)
    {
        rt_kprintf("[IVC_SEND_TEST] 发送测试失败\n");
    }
    else
    {
        rt_kprintf("[IVC_SEND_TEST] 发送测试完成\n");
    }

    // // 示例2：发送二进制数据（可选，如需测试二进制传输可取消注释）
    // uint8_t binary_data[] = {0x7e, 0x00, 0x00, 0x13, 0x00, 0x81, 0x00, 0x00, 0x00, 0x04};
    // ivc_send_example(binary_data, sizeof(binary_data));
}
// 导出为 MSH 命令，可在 RT-Thread 控制台直接调用
MSH_CMD_EXPORT(ivc_send_test, IVC send test(send string to peer));


/**
 * @brief MSH 命令：接收测试数据（分批次读取）
 * 用法：ivc_recv_test
 */
static void ivc_recv_test(void)
{
    rt_kprintf("[IVC_RECV_TEST] 开始接收测试（分批次读取，每次%u字节）\n", READ_BATCH_LEN);
    // 调用接收示例函数
    if (ivc_recv_example() != 0)
    {
        rt_kprintf("[IVC_RECV_TEST] 接收测试失败\n");
    }
    else
    {
        rt_kprintf("[IVC_RECV_TEST] 接收测试完成\n");
    }
}
// 导出为 MSH 命令
MSH_CMD_EXPORT(ivc_recv_test, IVC recv test(read data from peer));


/**
 * @brief 关闭IVC设备（可选，如需释放资源可调用）
 * 用法：ivc_close_dev
 */
static void ivc_close_dev(void)
{
    if (g_ivc_dev != RT_NULL)
    {
        rt_device_close(g_ivc_dev);
        rt_kprintf("[IVC_CLOSE] 成功关闭设备 %s\n", IVC_DEV_NAME);
        g_ivc_dev = RT_NULL; // 重置句柄
    }
    else
    {
        rt_kprintf("[IVC_CLOSE] 设备 %s 未打开\n", IVC_DEV_NAME);
    }
}
MSH_CMD_EXPORT(ivc_close_dev, close IVC device);
