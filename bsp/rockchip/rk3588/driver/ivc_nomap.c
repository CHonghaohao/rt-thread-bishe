#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <stdint.h>
#include <ivc.h>

/* IVC memory & IRQ */
#define IVC_TRQn 66
#define IVC_BASE 0xd0000000
//#define IVC_BASE     0x83fd000
#define IVC_SIZE          0x1000
#define control_table_IPA IVC_BASE
#define shared_mem_IPA    (IVC_BASE + 0x1000)

#define IVC_MB_SIZE 16 //
void *buffer;

static struct rt_mailbox ivc_mb;
static char ivc_mb_pool[IVC_MB_SIZE * 4];

ivc_serial_dev_t g_ivc_serial;
extern ring_buffer_t *ivc_get_tx_buffer(void);
extern ring_buffer_t *ivc_get_rx_buffer(void);

void kick_guest0(void)
{
    struct ivc_control_table *ct = (void *)control_table_IPA;
    ct->ipi_invoke = 0;
    // 写操作后加内存屏障，确保数据同步到物理内存
    rt_hw_dmb();
}

void kick_guest1(void)
{
    struct ivc_control_table *ct = (void *)control_table_IPA;
    ct->ipi_invoke = 1;
    rt_hw_dmb();
}


/*  */
int ringbuf_write(ring_buffer_t *rb, const void *data, uint32_t len)
{
#if 0
	uint32_t next = (rb->write_idx + 1) % SLOT_NUM;
	if (next == rb->read_idx)
		return -1; // 满

	if (len > SLOT_SIZE) len = SLOT_SIZE;
	rb->slots[rb->write_idx].len = len;
	memcpy(rb->slots[rb->write_idx].data, data, len);
	__sync_synchronize();
	rb->write_idx = next;
#endif
    // 1. 参数合法性检查（避免空指针或无效长度）
    if (data == NULL || len == 0)
    {
        rt_kprintf("ringbuf_write: 无效参数（data为空或len=0）\n");
        return -1;
    }
    rt_kprintf("ringbuf_write: data 地址 = %p, len（十进制）= %u, len（十六进制）= 0x%x\n",
               data, len, len);

    rt_kprintf("ringbuf_write: data 内容（十六进制） = ");
    const uint8_t *byte_data = (const uint8_t *)data; // 转换为字节指针
    for (uint32_t i = 0; i < len; i++)
    {
        rt_kprintf("%02x ", byte_data[i]); // 按两位十六进制打印，不足补0
        // 每16个字节换行，方便阅读
        if ((i + 1) % 16 == 0)
        {
            rt_kprintf("\n");
        }
    }
    rt_kprintf("\n"); // 结束换行
    rt_kprintf("ringbuf_write: data 内容（字符串） = %s\n", (const char *)data);
    char *txaddr = ivc_get_tx_buffer();
    if (txaddr == NULL)
    {
        rt_kprintf("ringbuf_write: 获取 tx 缓冲区失败\n");
        return -1;
    }

    uint32_t *len_ptr = (uint32_t *)txaddr;      // 长度字段指针（共享内存首地址）
    *len_ptr = len;                              // 存储数据长度
    char *data_addr = txaddr + sizeof(uint32_t); // 数据存储地址（跳过长度字段）

    // 拷贝数据到共享内存（从 data_addr 开始，长度为 len）
    rt_memcpy(data_addr, data, len);
    rt_hw_cpu_dcache_clean(txaddr, sizeof(uint32_t) + len);
    rt_kprintf("ringbuf_write: 成功写入（长度：%u 字节，数据地址：%p）\n", len, data_addr);
    return 0;
}

// 1. 协议配置（与发送端对齐）
#define LEN_FIELD_BYTES   4               // 总长度字段占 4 字节
#define MAX_VALID_TOTAL   1024            // 最大合法总长度（防止解析到垃圾值，如 0xffffffff）
#define DATA_START_OFFSET LEN_FIELD_BYTES // 实际业务数据的起始偏移（跳过长度字段）

// 2. 读取偏移量（跟踪已读取到业务数据的哪个位置）
static uint32_t g_read_offset = 0;
// 3. 缓存解析出的总数据长度（避免每次读取都重复解析）
static uint32_t g_cached_total_len = 0;


/**
 * @brief 从共享内存动态解析总数据长度（TOTAL_DATA_LEN）
 * @param rx_base 共享内存基地址
 * @return 有效总长度（>0 成功，0 失败）
 */
static uint32_t get_total_len_from_shared_mem(char *rx_base)
{
    if (rx_base == NULL)
    {
        rt_kprintf("get_total_len: 共享内存基地址为空\n");
        return 0;
    }

    // 解析长度字段（前 LEN_FIELD_BYTES 字节）
    uint32_t total_len = 0;
    if (LEN_FIELD_BYTES == 4)
    {
        // 长度字段占 4 字节
        total_len = *(uint32_t *)rx_base;
    }
    else
    {
        rt_kprintf("get_total_len: 不支持的长度字段字节数（%u）\n", LEN_FIELD_BYTES);
        return 0;
    }

    // 检查总长度合法性（避免垃圾值，如 0 或超过最大限制）
    if (total_len == 0 || total_len > MAX_VALID_TOTAL)
    {
        rt_kprintf("get_total_len: 无效总长度（%u 字节），可能是垃圾数据\n", total_len);
        return 0;
    }

    rt_kprintf("get_total_len: 动态解析总长度 = %u 字节\n", total_len);
    return total_len;
}


/**
 * @brief 分批次读取共享内存数据（动态获取总长度）
 * @param rb 环形缓冲区指针（本例暂用不到，可保留接口）
 * @param buf 接收数据的缓冲区
 * @param len 输入：buf 最大容量；输出：实际读取长度
 * @return 0 成功，-1 失败
 */
int ringbuf_read(ring_buffer_t *rb, void *buf, uint32_t *len)
{
    if (buf == NULL || len == NULL || *len == 0)
    {
        rt_kprintf("ringbuf_read: 无效参数（buf/len为空或请求长度为0）\n");
        return -1;
    }
    rt_hw_dmb();
    // 获取共享内存基地址
    char *rx_base = (char *)ivc_get_rx_buffer();
    if (rx_base == NULL)
    {
        rt_kprintf("ringbuf_read: 获取共享内存失败\n");
        *len = 0;
        return -1;
    }
    // 动态解析总数据长度（首次读取时解析，后续复用缓存值）
    if (g_cached_total_len == 0)
    {
        g_cached_total_len = get_total_len_from_shared_mem(rx_base);
        if (g_cached_total_len == 0)
        {
            rt_kprintf("ringbuf_read: 总长度解析失败，无法读取数据\n");
            *len = 0;
            return -1;
        }
    }
    // 计算业务数据的实际范围（跳过长度字段）
    char *data_base = rx_base + DATA_START_OFFSET;               // 业务数据基地址
    uint32_t remaining_len = g_cached_total_len - g_read_offset; // 剩余未读长度
    // 检查是否已读完所有业务数据
    if (remaining_len == 0)
    {
        rt_kprintf("ringbuf_read: 所有数据已读完（总长度：%u 字节）\n", g_cached_total_len);
        *len = 0;
        // 重置状态：准备读取下一段新数据
        g_read_offset = 0;
        g_cached_total_len = 0;
        return 0;
    }
    // 计算本次实际读取长度（取“请求长度”和“剩余长度”的最小值，防溢出）
    uint32_t actual_read_len = (*len < remaining_len) ? *len : remaining_len;
    // 复制数据到目标缓冲区（从当前偏移量开始）
    char *read_start = data_base + g_read_offset; // 本次读取起始地址
    rt_memcpy(buf, read_start, actual_read_len);
    // 更新读取偏移量（下次从新位置开始）
    g_read_offset += actual_read_len;
    // 返回实际读取长度
    *len = actual_read_len;
    rt_kprintf("\nringbuf_read: 读取完成 | 起始偏移：%u | 实际长度：%u 字节\n",
               g_read_offset - actual_read_len, actual_read_len);
    rt_kprintf("读取数据（十六进制）：");
    uint8_t *hex_buf = (uint8_t *)buf;
    for (uint32_t i = 0; i < actual_read_len; i++)
    {
        rt_kprintf("%02x ", hex_buf[i]);
        if ((i + 1) % 10 == 0) rt_kprintf("\n              "); // 每10字节换行，对齐显示
    }
    rt_kprintf("\n");
    rt_hw_dmb();
    rt_hw_cpu_dcache_clean((uint8_t *)buf, actual_read_len);
    return 0;
}

/*  poll */
static int ivc_poll(void)
{
    char buf[64];
    uint32_t len = 0;
    rt_ubase_t msg;

    while (1)
    {
        if (rt_sem_take(ivc_devs->irq_sem, RT_WAITING_FOREVER) == RT_EOK)
        {
            while (ivc_devs->received_irq > 0)
            {
                ivc_devs->received_irq--;
                len = 10;
                ringbuf_read(g_ivc_serial.rx_buf, buf, &len);
                len = 10;
                ringbuf_read(g_ivc_serial.rx_buf, buf, &len);
                // 打印读取到的数据（假设是字符串）
                ivc_devs->received_irq = 0;
            }
            rt_kprintf("write前\n");
            buffer = "hello zone0! I'm zone1. test";
            len = 31;
            ringbuf_write(g_ivc_serial.tx_buf, buffer, len);
            rt_kprintf("write后\n");
            rt_kprintf("addr : %p - %p , send : %s\n", &g_ivc_serial, g_ivc_serial.tx_buf, buffer);
            kick_guest0();
        }
    }

    return 0;
}

/* IRQ handler */
void ivc_irq_handler(int vector, void *param)
{
    (void)vector;
    (void)param;
    ivc_devs->received_irq++;
    rt_sem_release(ivc_devs->irq_sem);
    return;
}

/*  */
rt_size_t ivc_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    rt_kprintf("[IVC] 调用ivc_write！\n");
    ivc_serial_dev_t *ivc = (ivc_serial_dev_t *)dev->user_data;
    if (ringbuf_write(ivc->tx_buf, buffer, size) == 0)
    {
        ivc->kick_remote();
        return size;
    }
    return 0;
}

/*  */
rt_size_t ivc_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_kprintf("[IVC] 调用ivc_read！\n");
    ivc_serial_dev_t *ivc = (ivc_serial_dev_t *)dev->user_data;
    // uint32_t len = 0;
    if (ringbuf_read(ivc->rx_buf, buffer, &size) == 0)
        return size;

    return 0;
}

static rt_err_t ivc_open(rt_device_t dev, rt_uint16_t oflag)
{
    rt_kprintf("[IVC] 调用ivc_open！\n");
    return RT_EOK;
}
static rt_err_t ivc_close(rt_device_t dev)
{
    return RT_EOK;
}
static rt_err_t ivc_control(rt_device_t dev, int cmd, void *args)
{
    return RT_EOK;
}

static struct rt_device_ops ivc_ops = {
    .init = RT_NULL,
    .open = ivc_open,
    .close = ivc_close,
    .read = ivc_read,
    .write = ivc_write,
    .control = ivc_control};

ring_buffer_t *ivc_get_rx_buffer(void)
{
    return (ring_buffer_t *)shared_mem_IPA;
}

ring_buffer_t *ivc_get_tx_buffer(void)
{
    // 使用控制表的全局地址
    struct ivc_control_table *ct = (void *)control_table_IPA;
    // 基于共享内存的全局地址计算 tx 地址
    uintptr_t tx_virt_addr = (uintptr_t)shared_mem_IPA + ct->out_sec_size * ct->peer_id;
    // 检查地址是否超出共享内存范围（0x1000 是共享内存大小）
    if (tx_virt_addr >= (uintptr_t)shared_mem_IPA + 0x2000)
    {
        rt_kprintf("[IVC] tx 地址超出范围：0x%lx\n", tx_virt_addr);
        return NULL;
    }
    return (ring_buffer_t *)tx_virt_addr;
}

void rt_ivc_init(void)
{
    ivc_devs = rt_malloc(sizeof(struct ivc_dev));
    ivc_devs->received_irq = 0;
    ivc_devs->irq_sem = rt_sem_create("ivc_irq", 0, RT_IPC_FLAG_FIFO);

    g_ivc_serial.tx_buf = ivc_get_tx_buffer();
    g_ivc_serial.rx_buf = ivc_get_rx_buffer();
    g_ivc_serial.kick_remote = kick_guest0;

    g_ivc_serial.dev.type = RT_Device_Class_Char;
    g_ivc_serial.dev.user_data = &g_ivc_serial;
#ifdef RT_USING_DEVICE_OPS
    g_ivc_serial.dev.ops = &ivc_ops;
#endif

    rt_device_register(&g_ivc_serial.dev, "ivc", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);

    rt_hw_interrupt_install(IVC_TRQn, ivc_irq_handler, RT_NULL, "ivc");
    rt_hw_interrupt_umask(IVC_TRQn);
}
MSH_CMD_EXPORT(rt_ivc_init, ivc device init);

void rt_ivc_poll(void)
{
    rt_thread_t th = rt_thread_create("ivcpoll", (void *)ivc_poll, NULL, 8192, 20, 100000);
    if (th)
        rt_thread_startup(th);
}
MSH_CMD_EXPORT(rt_ivc_poll, ivc device init);

