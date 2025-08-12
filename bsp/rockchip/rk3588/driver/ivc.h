#ifndef __IVC_H__
#define __IVC_H__

#include <ioremap.h>
#include <stdint.h>
#include <string.h>

#define IVC_BASE 0x83fd000
#define IVC_SIZE 0x1000
#define IVC_TRQn 66
#define IVC_p0_TRQn 65

#define CONFIG_MAX_IVC_CONFIGS 2
#define HVISOR_HC_IVC_INFO 5

#define control_table_IPA IVC_BASE
#define shared_mem_IPA (IVC_BASE + 0x1000)

#define TIME_LIMIT 2000

typedef int poll_table_struct; //TODO:把linux的等待换进来

struct ivc_dev {
    // dev_t dev_id;
    // struct cdev cdev;
    // struct device *device;
    // struct task_struct *task;
    // wait_queue_head_t wq;
    // int idx;
    // int ivc_id;
    // int ivc_irq;
    int received_irq; // receive irq count
};

struct ivc_control_table {
    volatile uint32_t ivc_id;
    volatile uint32_t max_peers;
    volatile uint32_t rw_sec_size;
    volatile uint32_t out_sec_size;
    volatile uint32_t peer_id;
    volatile uint32_t ipi_invoke;
} __attribute__((packed));
typedef struct ivc_control_table ivc_cttable_t;

struct ivc_info {
    uint64_t len;
    uint64_t ivc_ct_ipas[CONFIG_MAX_IVC_CONFIGS];
    uint64_t ivc_shmem_ipas[CONFIG_MAX_IVC_CONFIGS];
    uint64_t ivc_ids[CONFIG_MAX_IVC_CONFIGS];
    uint64_t ivc_irqs[CONFIG_MAX_IVC_CONFIGS];
};
typedef struct ivc_info ivc_info_t;

extern struct ivc_dev *ivc_devs;

void ivc_irq_handler(int vector, void* parameter);
void rt_ivc_init(void);
int ivc_poll(void);
//int ivc_poll(poll_table_struct* wait);
int ivc_send(char* s);
void send_test(void);
static int hvisor_ivc_info(void);

#endif 
