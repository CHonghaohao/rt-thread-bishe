#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <stdint.h>

/* IVC memory & IRQ */
#define IVC_TRQn     66
#define IVC_BASE     0xd0000000
//#define IVC_BASE     0x83fd000
#define IVC_SIZE     0x1000
#define control_table_IPA  IVC_BASE
#define shared_mem_IPA     (IVC_BASE + 0x1000)

#define SLOT_NUM    64
#define SLOT_SIZE   64





#define IVC_MB_SIZE  16  //
void *buffer;

//static struct rt_mailbox ivc_mb;
//static char ivc_mb_pool[IVC_MB_SIZE * 4];  




typedef struct {
    uint32_t len;
    uint8_t data[SLOT_SIZE];
} ivc_slot_t;

typedef struct {
    volatile uint32_t write_idx;
    volatile uint32_t read_idx;
    ivc_slot_t slots[SLOT_NUM];
} ring_buffer_t;

struct ivc_control_table {
    volatile uint32_t ivc_id;
    volatile uint32_t max_peers;
    volatile uint32_t rw_sec_size;
    volatile uint32_t out_sec_size;
    volatile uint32_t peer_id;
    volatile uint32_t ipi_invoke;
} __attribute__((packed));

/*  */
static struct ivc_dev {
    volatile int received_irq;
    rt_sem_t irq_sem;  // 
} *ivc_devs;

typedef struct {
    struct rt_device dev;
    ring_buffer_t *tx_buf;
    ring_buffer_t *rx_buf;
    rt_sem_t rx_sem;
    void (*kick_remote)(void);  // 
} ivc_serial_dev_t;

static ivc_serial_dev_t g_ivc_serial;
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
static int ringbuf_write(ring_buffer_t *rb,  void *data, uint32_t len)
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
    char *txaddr =ivc_get_tx_buffer();
    rt_strcpy(txaddr, data);
    rt_hw_cpu_dcache_clean(txaddr,0x100);

    return 0;
}

static int ringbuf_read(ring_buffer_t *rb, void *buf, uint32_t *len)
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
    buffer =ivc_get_rx_buffer();
    rt_hw_cpu_dcache_clean(buffer,0x100);
    rt_hw_dmb();


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

		    ringbuf_read(g_ivc_serial.rx_buf, buf, &len) ;
		   

		    ivc_devs->received_irq = 0;

		    
	    }
	    ringbuf_write(g_ivc_serial.tx_buf, buffer, &len);
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
	return ;
}

/*  */
static rt_size_t ivc_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
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
static rt_size_t ivc_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
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
    return (ring_buffer_t *)(shared_mem_IPA + ct->out_sec_size * ct->peer_id); //
}

void rt_ivc_init(void)
{
    ivc_devs = rt_malloc(sizeof(struct ivc_dev));
    ivc_devs->received_irq = 0;
    ivc_devs->irq_sem = rt_sem_create("ivc_irq", 0, RT_IPC_FLAG_FIFO);


    g_ivc_serial.tx_buf = ivc_get_tx_buffer();
    g_ivc_serial.rx_buf = ivc_get_rx_buffer();
    g_ivc_serial.kick_remote = kick_guest1;

    g_ivc_serial.dev.type = RT_Device_Class_Char;
    g_ivc_serial.dev.user_data = &g_ivc_serial;
    g_ivc_serial.dev.ops = &ivc_ops;

    rt_device_register(&g_ivc_serial.dev, "ivc", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);

    rt_hw_interrupt_install(IVC_TRQn, ivc_irq_handler, RT_NULL, "ivc");
    rt_hw_interrupt_umask(IVC_TRQn);

    rt_thread_t th = rt_thread_create("ivcpoll", (void*)ivc_poll, NULL, 8192, 10, 100000);
    if (th) 
	    rt_thread_startup(th);
}
INIT_APP_EXPORT(rt_ivc_init);

