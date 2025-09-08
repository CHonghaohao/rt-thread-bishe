#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <stdint.h>
#include <ivc.h>

/* IVC memory & IRQ */
#define IVC_TRQn     66
#define IVC_BASE     0xd0000000
//#define IVC_BASE     0x83fd000
#define IVC_SIZE     0x1000
#define control_table_IPA  IVC_BASE
#define shared_mem_IPA     (IVC_BASE + 0x1000)





#define IVC_MB_SIZE  16  //
void *buffer;

static struct rt_mailbox ivc_mb;
static char ivc_mb_pool[IVC_MB_SIZE * 4];  

ivc_serial_dev_t g_ivc_serial;
extern  ring_buffer_t *ivc_get_tx_buffer(void);
extern  ring_buffer_t *ivc_get_rx_buffer(void);

/*  */
void kick_guest1(void) {
    struct ivc_control_table *ct = (void*)control_table_IPA;
    ct->ipi_invoke = 1;
}
void kick_guest0(void) {
    struct ivc_control_table *ct = (void*)control_table_IPA;
    ct->ipi_invoke = 0;
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
    rt_kprintf("ringbuf_write: data 地址 = %p, len（十进制）= %u, len（十六进制）= 0x%x\n", 
               data, len, len);
    if (data != NULL && len > 0) {
        rt_kprintf("ringbuf_write: data 内容（十六进制） = ");
        const uint8_t *byte_data = (const uint8_t *)data; // 转换为字节指针
        for (uint32_t i = 0; i < len; i++) {
            rt_kprintf("%02x ", byte_data[i]); // 按两位十六进制打印，不足补0
            // 每16个字节换行，方便阅读
            if ((i + 1) % 16 == 0) {
                rt_kprintf("\n");
            }
        }
        rt_kprintf("\n"); // 结束换行
    }
    char *txaddr =ivc_get_tx_buffer();
    rt_kprintf("ringbuf_write: data 内容（字符串） = %s\n", (const char *)data);
    rt_memcpy(txaddr, data, len);
    rt_hw_cpu_dcache_clean(txaddr,0x100);

    return 0;
}

static uint32_t g_rx_buf_offset = 0;
int ringbuf_read(ring_buffer_t *rb, void *buf, uint32_t *len)
{
#if 0
	if (rb->read_idx == rb->write_idx)
		return -1;

	uint32_t l = rb->slots[rb->read_idx].len;
	if (l > SLOT_SIZE) l = SLOT_SIZE;
	memcpy(buf, rb->slots[rb->read_idx].data, l);
	*len = l;
	rb->read_idx = (rb->read_idx + 1) % SLOT_NUM;
#endif
    rt_hw_dmb();
    buffer =(char *)ivc_get_rx_buffer();
    // uint32_t buffer_len = buffer.size();
    // rt_kprintf("读取到数据: %s\n", buffer); 
    // uint32_t copy_len; 
    // copy_len = *len;
    // rt_memcpy(buf, buffer, copy_len);
    // *len = copy_len;

    char *rx_buf_base = (char *)ivc_get_rx_buffer();
    #define IVC_CURRENT_VALID_LEN 40 
    uint32_t remaining_len = IVC_CURRENT_VALID_LEN - g_rx_buf_offset;
    if (remaining_len == 0)
    {
        rt_kprintf("ringbuf_read: 无剩余数据可读取（已全部消费）\n");
        *len = 0;
        g_rx_buf_offset = 0; // 重置偏移量（准备下次接收新数据）
        return -1;
    }
    // 确定本次实际读取长度（取“请求的 len”和“剩余长度”的最小值，避免越界）
    uint32_t actual_read_len = (*len > remaining_len) ? remaining_len : *len;

    // 计算本次读取的“起始地址”（基地址 + 已消费偏移量）
    char *read_start_addr = rx_buf_base + g_rx_buf_offset;

    // 按长度复制数据（从起始地址复制 actual_read_len 字节到 buf）
    rt_memcpy(buf, read_start_addr, actual_read_len);
    // 更新偏移量
    g_rx_buf_offset += actual_read_len;

    // 返回实际读取长度
    *len = actual_read_len;

    // 验证分次读取结果
    rt_kprintf("ringbuf_read: 读取完成 | 起始偏移量：%u | 实际读取长度：%u | 读取数据（十六进制）：", 
               g_rx_buf_offset - actual_read_len, actual_read_len);
    uint8_t *hex_buf = (uint8_t *)buf;
    for (uint32_t i = 0; i < actual_read_len; i++)
    {
        rt_kprintf("%02x ", hex_buf[i]);
    }
    rt_kprintf("\n");
    rt_kprintf("读取到数据（字符串）: %.*s\n", (int)actual_read_len, (char *)hex_buf);


    rt_hw_dmb();
    rt_hw_cpu_dcache_clean(buffer ,0x100);

    return 0;
}

/*  poll */
static int ivc_poll(void)
{
    char buf[64];
    uint32_t len = 0;
     rt_ubase_t msg;

    while (1){
        if (rt_sem_take(ivc_devs->irq_sem, RT_WAITING_FOREVER) == RT_EOK){
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
    g_rx_buf_offset = 0;
	rt_sem_release(ivc_devs->irq_sem);
	return ;
}

/*  */
rt_size_t ivc_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
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
    ivc_serial_dev_t *ivc = (ivc_serial_dev_t *)dev->user_data;
    uint32_t len = 0;
    if (ringbuf_read(ivc->rx_buf, buffer, &len) == 0)
        return len;

    return 0;
}

static rt_err_t ivc_open(rt_device_t dev, rt_uint16_t oflag) { return RT_EOK; }
static rt_err_t ivc_close(rt_device_t dev) { return RT_EOK; }
static rt_err_t ivc_control(rt_device_t dev, int cmd, void *args) { return RT_EOK; }

static struct rt_device_ops ivc_ops = {
    .init    = RT_NULL,
    .open    = ivc_open,
    .close   = ivc_close,
    .read    = ivc_read,
    .write   = ivc_write,
    .control = ivc_control
};

/* */
ring_buffer_t *ivc_get_rx_buffer(void)
{
    return (ring_buffer_t *)(shared_mem_IPA + 0x0000); //
}
ring_buffer_t *ivc_get_tx_buffer(void)
{
    struct ivc_control_table *ct = (void*)control_table_IPA;
    //rt_kprintf("... peer_id: 0x%x\n", ct->peer_id);  // 十六进制带0x前缀
    return (ring_buffer_t *)(shared_mem_IPA + ct->out_sec_size * ct->peer_id); //
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
    g_ivc_serial.dev.ops = &ivc_ops;

    rt_device_register(&g_ivc_serial.dev, "ivc", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);

    rt_hw_interrupt_install(IVC_TRQn, ivc_irq_handler, RT_NULL, "ivc");
    rt_hw_interrupt_umask(IVC_TRQn);

    // rt_thread_t th = rt_thread_create("ivcpoll", (void*)ivc_poll, NULL, 8192, 20, 100000);
    // if (th) 
	//     rt_thread_startup(th);
}
MSH_CMD_EXPORT(rt_ivc_init, ivc device init);

void rt_ivc_poll(void)
{
    rt_thread_t th = rt_thread_create("ivcpoll", (void*)ivc_poll, NULL, 8192, 20, 100000);
    if (th) 
	    rt_thread_startup(th);
}
MSH_CMD_EXPORT(rt_ivc_poll, ivc device init);

