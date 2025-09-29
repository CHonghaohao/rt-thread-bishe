#ifndef __IVC_H__
#define __IVC_H__

#include <ioremap.h>
#include <stdint.h>
#include <string.h>

#define IVC_SIZE    0x1000
#define IVC_TRQn    66
#define IVC_p0_TRQn 65

#define SLOT_NUM  64
#define SLOT_SIZE 64

#define CONFIG_MAX_IVC_CONFIGS 2
#define HVISOR_HC_IVC_INFO     5

#define control_table_IPA IVC_BASE
#define shared_mem_IPA    (IVC_BASE + 0x1000)

#define TIME_LIMIT 2000

typedef int poll_table_struct; //TODO:把linux的等待换进来

struct ivc_control_table
{
    volatile uint32_t ivc_id;
    volatile uint32_t max_peers;
    volatile uint32_t rw_sec_size;
    volatile uint32_t out_sec_size;
    volatile uint32_t peer_id;
    volatile uint32_t ipi_invoke;
} __attribute__((packed));
typedef struct ivc_control_table ivc_cttable_t;

struct ivc_info
{
    uint64_t len;
    uint64_t ivc_ct_ipas[CONFIG_MAX_IVC_CONFIGS];
    uint64_t ivc_shmem_ipas[CONFIG_MAX_IVC_CONFIGS];
    uint64_t ivc_ids[CONFIG_MAX_IVC_CONFIGS];
    uint64_t ivc_irqs[CONFIG_MAX_IVC_CONFIGS];
};
typedef struct ivc_info ivc_info_t;

typedef struct
{
    uint32_t len;
    uint8_t data[SLOT_SIZE];
} ivc_slot_t;

typedef struct
{
    volatile uint32_t write_idx;
    volatile uint32_t read_idx;
    ivc_slot_t slots[SLOT_NUM];
} ring_buffer_t;

/*  */
static struct ivc_dev
{
    volatile int received_irq;
    rt_sem_t irq_sem; // 这种会导致中断信号量累计，
    rt_event_t ivc_evt;
} *ivc_devs;

typedef struct
{
    struct rt_device dev;
    ring_buffer_t *tx_buf;
    ring_buffer_t *rx_buf;
    rt_sem_t rx_sem;
    void (*kick_remote)(void); //
} ivc_serial_dev_t;

struct rb_state
{
    uint32_t cur_len;
    uint32_t cur_read;
    uint32_t cur_rpos; // payload 起始位置（绕开长度字段 4B 后的 ring 位置）
    rt_bool_t in_partial;
};

extern struct ivc_dev *ivc_devs;
extern ivc_serial_dev_t g_ivc_serial;

void ivc_irq_handler(int vector, void *parameter);
void rt_ivc_init(void);
// int ivc_send(char* s);
void send_test(void);
static int hvisor_ivc_info(void);
int ringbuf_write(ring_buffer_t *, const void *, uint32_t);
int ringbuf_read(ring_buffer_t *, void *, uint32_t *);
rt_size_t ivc_write(rt_device_t, rt_off_t, const void *, rt_size_t);
rt_size_t ivc_read(rt_device_t, rt_off_t, void *, rt_size_t);
void kick_guest0(void);
void kick_guest1(void);

#endif

