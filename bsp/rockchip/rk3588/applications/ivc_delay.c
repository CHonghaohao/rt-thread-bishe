#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <limits.h> // 用于INT_MAX

// 配置参数（与发送端保持一致，删除 TEST_COUNT）
#define IVC_DEV_NAME   "ivc"
#define MSG_PREFIX     "TEST_UINT:"
#define MSG_PREFIX_LEN strlen(MSG_PREFIX)
#define RECV_BUF_SIZE  32 // 足够存储非负整数字符串
#define SEND_BUF_SIZE  32
#define READ_BATCH_LEN 10

// 全局变量（仅由测试线程访问，无需互斥锁）
static rt_device_t g_ivc_dev = RT_NULL;
static uint8_t g_recv_buf[RECV_BUF_SIZE] = {0};
static uint8_t g_send_buf[SEND_BUF_SIZE] = {0};
static rt_bool_t g_test_running = RT_FALSE; // 测试运行状态标记

/**
 * 初始化IVC设备
 */
static int ivc_init(void)
{
    // 查找并打开IVC设备
    if (g_ivc_dev == RT_NULL)
    {
        g_ivc_dev = rt_device_find(IVC_DEV_NAME);
        if (g_ivc_dev == RT_NULL)
        {
            rt_kprintf("[IVC_UINT] 未找到IVC设备 %s\n", IVC_DEV_NAME);
            return -1;
        }

        // 以“中断接收”模式打开设备（确保数据到达时触发通知）
        if (rt_device_open(g_ivc_dev, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
        {
            rt_kprintf("[IVC_UINT] 打开设备 %s 失败\n", IVC_DEV_NAME);
            g_ivc_dev = RT_NULL;
            return -1;
        }
        rt_kprintf("[IVC_UINT] 成功打开IVC设备（非负范围: 0 至 %d）\n", INT_MAX);
        rt_kprintf("[IVC_UINT] 等待Linux端发送数据...\n");
    }
    return 0;
}

/**
 * 读取消息（等待数据到达，无数据时阻塞，避免CPU空转）
 */
static rt_size_t ivc_read_msg(void)
{
    if (g_ivc_dev == NULL)
    {
        return 0;
    }

    rt_memset(g_recv_buf, 0, RECV_BUF_SIZE);
    rt_size_t actual_len = 0;

    // 阻塞读取：无数据时等待，直到有数据或超时（超时设为0表示永久等待）
    actual_len = rt_device_read(g_ivc_dev, 0, g_recv_buf, RECV_BUF_SIZE - 1); // 留1字节存终止符


    // rt_kprintf("\n[RECV] - 实际长度：%u 字节\n", actual_len);
    // rt_kprintf("[RECV] - 读取数据（十六进制）：");
    // rt_kprintf("\n              ");
    // uint8_t *hex_buf = (uint8_t *)g_recv_buf;
    // for (uint32_t i = 0; i < actual_len; i++)
    // {
    //     rt_kprintf("%02x ", hex_buf[i]);
    //     if ((i + 1) % 10 == 0) rt_kprintf("\n              "); // 每10字节换行，对齐显示
    // }
    // rt_kprintf("\n");
    // rt_kprintf("[RECV] - data 内容（字符串） = %s\n", (const char *)hex_buf);


    if (actual_len == (rt_size_t)-1)
    {
        rt_kprintf("[IVC_UINT] 读取失败\n");
        return 0;
    }
    else if (actual_len == 0)
    {
        rt_kprintf("[IVC_UINT] 无数据可读（超时或设备关闭）\n");
        return 0;
    }

    // 添加字符串终止符，避免解析错误
    g_recv_buf[actual_len] = '\0';
    return actual_len;
}

/**
 * 发送回复消息（非负数字加1）
 */
static int ivc_send_reply(int num)
{
    if (num < 0)
    {
        rt_kprintf("[IVC_UINT] 错误：接收到负数 %d\n", num);
        return -1;
    }
    if (g_ivc_dev == NULL)
    {
        return -1;
    }

    // 数字加1（非负保证）
    int reply_num = num + 1;

    // 构建回复消息（包含终止符）
    rt_size_t reply_len = rt_snprintf((char *)g_send_buf, SEND_BUF_SIZE,
                                      "%s%d", MSG_PREFIX, reply_num);
    if (reply_len <= 0 || reply_len >= SEND_BUF_SIZE)
    {
        rt_kprintf("[IVC_UINT] 回复消息过长\n");
        return -1;
    }
    reply_len += 1; // 包含'\0'终止符

    // 发送回复
    rt_size_t actual_send = rt_device_write(g_ivc_dev, 0, g_send_buf, reply_len);


    // rt_kprintf("\n[SEND] - 实际长度：%u 字节\n", reply_len);
    // rt_kprintf("[SEND] - 读取数据（十六进制）：");
    // rt_kprintf("\n              ");
    // uint8_t *hex_buf = (uint8_t *)g_send_buf;
    // for (uint32_t i = 0; i < reply_len; i++)
    // {
    //     rt_kprintf("%02x ", hex_buf[i]);
    //     if ((i + 1) % 16 == 0) rt_kprintf("\n              "); // 每10字节换行，对齐显示
    // }
    // rt_kprintf("\n");
    // rt_kprintf("[SEND] - data 内容（字符串） = %s\n", (const char *)hex_buf);


    if (actual_send != reply_len)
    {
        rt_kprintf("[IVC_UINT] 回复发送失败（实际发送%d字节，期望%d字节）\n", actual_send, reply_len);
        return -1;
    }

    rt_kprintf("[IVC_UINT] 处理完成：接收%d → 回复%d\n", num, reply_num);
    return 0;
}

/**
 * 测试线程：永久监听Linux端数据，被动响应
 */
static void ivc_uint_test_thread(void *parameter)
{
    if (ivc_init() != 0)
    {
        rt_kprintf("[IVC_UINT] 初始化失败，退出测试\n");
        g_test_running = RT_FALSE;
        return;
    }

    g_test_running = RT_TRUE;
    uint32_t total_count = 0; // 仅用于统计总处理次数，非限制条件

    // 永久循环：直到设备关闭或主动停止
    while (g_test_running && g_ivc_dev != RT_NULL)
    {
        // 1. 阻塞读取Linux端数据（无数据时等待）
        rt_size_t recv_len = ivc_read_msg();
        if (recv_len <= MSG_PREFIX_LEN)
        {
            rt_thread_mdelay(10); // 短延时后重试，避免CPU空转
            continue;
        }


        // 2. 验证消息格式（必须以MSG_PREFIX开头）
        if (strncmp((char *)g_recv_buf, MSG_PREFIX, MSG_PREFIX_LEN) != 0)
        {
            rt_kprintf("[IVC_UINT] 格式错误: %s（需以\"%s\"开头）\n", g_recv_buf, MSG_PREFIX);
            rt_thread_mdelay(10);
            continue;
        }

        // 3. 提取并转换数字
        char *num_str = (char *)g_recv_buf + MSG_PREFIX_LEN;
        int num;
        if (rt_sscanf(num_str, "%d", &num) != 1 || num < 0)
        {
            rt_kprintf("[IVC_UINT] 数字无效: %s\n", num_str);
            rt_thread_mdelay(10);
            continue;
        }

        // 4. 发送加1后的回复
        if (ivc_send_reply(num) == 0)
        {
            total_count++;
            // 每处理100次打印一次统计（可选，用于查看进度）
            if (total_count % 100 == 0)
            {
                rt_kprintf("[IVC_UINT] 累计处理 %d 次\n", total_count);
            }
        }


        // 短延时：降低CPU占用，避免频繁轮询
        rt_thread_mdelay(1);
    }

    // 测试结束后清理
    rt_kprintf("\n[IVC_UINT] 测试停止\n");
    rt_kprintf("累计处理次数: %d\n", total_count);
    if (g_ivc_dev != RT_NULL)
    {
        rt_device_close(g_ivc_dev);
        g_ivc_dev = RT_NULL;
    }
    g_test_running = RT_FALSE;
}

/**
 * 新增：停止测试命令（可选，用于主动终止监听）
 */
static void ivc_uint_stop(void)
{
    g_test_running = RT_FALSE;
    // 关闭设备触发读取超时，退出监听循环
    if (g_ivc_dev != RT_NULL)
    {
        rt_device_close(g_ivc_dev);
        g_ivc_dev = RT_NULL;
    }
    rt_kprintf("[IVC_UINT] 已停止测试\n");
}

/**
 * MSH命令：启动非负整数测试（永久监听）
 */
static void ivc_uint_test(void)
{
    if (g_test_running)
    {
        rt_kprintf("[IVC_UINT] 已有测试在运行，无需重复启动\n");
        return;
    }

    rt_thread_t thread = rt_thread_create("ivc_uint_thread",
                                          ivc_uint_test_thread,
                                          RT_NULL,
                                          8192,    // 栈大小
                                          20,      // 优先级（低于中断线程，高于普通线程）
                                          100000); // 时间片
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
        rt_kprintf("[IVC_UINT] 测试线程已启动（等待Linux端数据）\n");
    }
    else
    {
        rt_kprintf("[IVC_UINT] 创建线程失败\n");
    }
}

// 导出MSH命令：启动测试+停止测试
MSH_CMD_EXPORT(ivc_uint_test, IVC非负整数测试：永久监听Linux数据，回复加1结果);
MSH_CMD_EXPORT(ivc_uint_stop, 停止IVC非负整数测试);
