#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <stdint.h>
#include <ivc.h>

ivc_serial_dev_t g_ivc_serial;

#define IVC_BASE      0xd0000000u
#define IVC_CT_IPA    (IVC_BASE + 0x0000u) //控制表
#define IVC_SHMEM_IPA (IVC_BASE + 0x1000u)


#define RB_HDR_BYTES 8u
#define RB_WPTR_OFF  0u
#define RB_RPTR_OFF  4u
#define RB_DATA_OFF  RB_HDR_BYTES
#define RB_SECT_SIZE 0x1000u
#define RB_DATA_SIZE (RB_SECT_SIZE - RB_HDR_BYTES)


static inline uint32_t rb_norm(uint32_t x)
{
    return (x >= RB_DATA_SIZE) ? (x - RB_DATA_SIZE) : x;
}
static inline uint32_t rb_readable(uint32_t w_ptr, uint32_t r_ptr)
{
    return (w_ptr >= r_ptr) ? (w_ptr - r_ptr) : (RB_DATA_SIZE - (r_ptr - w_ptr));
}
static inline uint32_t rb_writable(uint32_t w_ptr, uint32_t r_ptr)
{
    return (RB_DATA_SIZE - 1) - rb_readable(w_ptr, r_ptr);
}
static inline uint32_t rb_load32(volatile void *base, uint32_t off)
{
    rt_hw_rmb();
    return *(volatile uint32_t *)((uintptr_t)base + off);
}
static inline void rb_store32(volatile void *base, uint32_t off, uint32_t v)
{
    *(volatile uint32_t *)((uintptr_t)base + off) = v;
    rt_hw_wmb();
}
static inline void rb_copy_from_ring(volatile void *base, uint32_t pos, void *dst, uint32_t n)
{
    uint32_t first = (n < (RB_DATA_SIZE - pos)) ? n : (RB_DATA_SIZE - pos);

    rt_memcpy(dst, (const void *)((uintptr_t)base + RB_DATA_OFF + pos), first);
    if (n > first)
    {
        rt_memcpy((uint8_t *)dst + first, (const void *)((uintptr_t)base + RB_DATA_OFF), n - first);
    }
}
static inline void rb_copy_to_ring(volatile void *base, uint32_t pos, const void *src, uint32_t n)
{
    uint32_t first = (n < (RB_DATA_SIZE - pos)) ? n : (RB_DATA_SIZE - pos);
    rt_memcpy((void *)((uintptr_t)base + RB_DATA_OFF + pos), src, first);
    if (n > first)
    {
        rt_memcpy((void *)((uintptr_t)base + RB_DATA_OFF), (const uint8_t *)src + first, n - first);
    }
    rt_hw_wmb();
}

static inline volatile void *ivc_rx_ring_base(void)
{
    return (volatile void *)(uintptr_t)(IVC_SHMEM_IPA + 0x0000u);
}
static inline volatile void *ivc_tx_ring_base(void)
{
    return (volatile void *)(uintptr_t)(IVC_SHMEM_IPA + 0x1000u);
}

static inline void ivc_kick_remote(void)
{
    volatile struct ivc_control_table *ct = (volatile struct ivc_control_table *)(uintptr_t)IVC_CT_IPA;
    ct->ipi_invoke = 0;
    rt_hw_dmb();
}


// RT 端 TX：写一条报文（len + data，可一次或多次调用写多条）
int ivc_rb_write(const void *data, uint32_t len)
{
    volatile void *base = ivc_tx_ring_base();
    if (!data || len == 0 || len > RB_SECT_SIZE) return -RT_EINVAL;

    uint32_t w_ptr = rb_load32(base, RB_WPTR_OFF);
    uint32_t r_ptr = rb_load32(base, RB_RPTR_OFF);
    uint32_t need = 4 + len;

    // rt_kprintf("[IVC] 调用ivc_rb_write!\n");
    // rt_kprintf("ivc_rb_write w_ptr: %d - r_ptr: %d\n", w_ptr, r_ptr);
    // rt_kprintf("ivc_rb_write need: %d\n", need);

    if (rb_writable(w_ptr, r_ptr) < need)
    {
        return -RT_EFULL;
    }

    //写长度
    uint32_t le_len = len;
    rb_copy_to_ring(base, w_ptr, &le_len, 4);
    w_ptr = rb_norm(w_ptr + 4);

    // 写payload
    rb_copy_to_ring(base, w_ptr, data, len);
    w_ptr = rb_norm(w_ptr + len);

    //更新w_ptr
    rb_store32(base, RB_WPTR_OFF, w_ptr);
    // rt_kprintf("ivc_rb_write w_ptr: %d\n", w_ptr);
    ivc_kick_remote();
    return len;
}

// ----------- RT 端 RX：可分段读取一条报文
static struct rb_state g_rx = {0};
int ivc_rb_read(void *buf, uint32_t *inout_len)
{
    if (!buf || !inout_len || *inout_len == 0) return -RT_EINVAL;

    volatile void *base = ivc_rx_ring_base();
    uint32_t w_ptr = rb_load32(base, RB_WPTR_OFF);
    uint32_t r_ptr = rb_load32(base, RB_RPTR_OFF);
    uint32_t avail = rb_readable(w_ptr, r_ptr);
    // rt_kprintf("[IVC] 调用ivc_rb_read！\n");
    // rt_kprintf("ivc_rb_read w_ptr: %d - r_ptr: %d\n", w_ptr, r_ptr);
    // rt_kprintf("ivc_rb_read avail: %d\n", avail);
    /* 还没开始读一条：先拿 4B len */
    if (!g_rx.in_partial)
    {
        if (avail < 4)
        {
            return 0; // 暂时无数据可读
        }
        uint32_t le_len = 0;
        rb_copy_from_ring(base, r_ptr, &le_len, 4);
        g_rx.cur_len = le_len;
        if (g_rx.cur_len == 0 || g_rx.cur_len > RB_SECT_SIZE)
        {
            // 丢弃4B数据
            r_ptr = rb_norm(r_ptr + 4);
            rb_store32(base, RB_RPTR_OFF, r_ptr);
            return -RT_EIO;
        }
        g_rx.cur_rpos = rb_norm(r_ptr + 4);
        g_rx.cur_read = 0;
        g_rx.in_partial = RT_TRUE;
    }

    // 读 payload
    uint32_t left = g_rx.cur_len - g_rx.cur_read;
    if ((int32_t)left <= 0)
    {
        // 推进r_ptr
        r_ptr = rb_norm(r_ptr + 4 + g_rx.cur_len);
        rb_store32(base, RB_RPTR_OFF, r_ptr);
        g_rx.in_partial = RT_FALSE;
        *inout_len = 0;
        return 0;
    }

    uint32_t tocp = (*inout_len < left) ? *inout_len : left;
    uint32_t pos = rb_norm(g_rx.cur_rpos + g_rx.cur_read);
    rb_copy_from_ring(base, pos, buf, tocp);
    g_rx.cur_read += tocp;
    *inout_len = tocp;

    // 如果本条已读完，推进r_ptr并退出 partial 状态
    // rt_kprintf("ivc_rb_read g_rx.cur_read: %d  g_rx.cur_len: %d\n", g_rx.cur_read, g_rx.cur_len);
    // rt_kprintf("ivc_rb_read tocp: %d\n", tocp);
    if (g_rx.cur_read >= g_rx.cur_len)
    {
        r_ptr = rb_norm(r_ptr + 4 + g_rx.cur_len);
        rb_store32(base, RB_RPTR_OFF, r_ptr);
        // rt_kprintf("ivc_rb_read r_ptr: %d\n", r_ptr);
        uint32_t r_ptr_read = rb_load32(base, RB_RPTR_OFF);
        // rt_kprintf("ivc_rb_read r_ptr_read: %d\n", r_ptr_read);
        g_rx.in_partial = RT_FALSE;
        ivc_devs->received_irq--;
    }
    return tocp; // 返回本次读的字节数
}

static void ivc_poll(void)
{
    char buf[128];
    uint32_t want;
    int rc;

    while (1)
    {
        // 阻塞等待中断信号，避免忙等
        // rt_sem_take(ivc_devs->irq_sem, RT_WAITING_FOREVER); rt_tick_from_millisecond(100)
        // rt_kprintf("阻塞等待\n");
        if (rt_event_recv(ivc_devs->ivc_evt,
                          0x01,
                          RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                          rt_tick_from_millisecond(100),
                          RT_NULL)
            != RT_EOK)
        {
            continue; // 超时，直接下一轮
        }

        // 收到中断后，可能有多条报文，需要一直读完
        do {
            want = sizeof(buf);
            rc = ivc_rb_read(buf, &want);
            // rt_kprintf("ivc_poll 收到消息: %d\n", rc);
            if (rc > 0)
            {
                ivc_rb_write(buf, want);
                // rt_kprintf("ivc_poll 发送消息: %d\n", want);
            }
            else if (rc < 0)
            {
                rt_kprintf("ivc_rb_read error: %d\n", rc);
                break;
            }
        } while (rc > 0);
    }
}


/* IRQ handler */
void ivc_irq_handler(int vector, void *param)
{
    (void)vector;
    (void)param;
    g_rx.in_partial = RT_FALSE;
    g_rx.cur_len = g_rx.cur_read = g_rx.cur_rpos = 0;
    ivc_devs->received_irq++;
    // rt_kprintf("[IVC] 收到中断 received_irq: %d\n", ivc_devs->received_irq);
    // rt_kprintf("\n");
    // rt_sem_release(ivc_devs->irq_sem);
    rt_event_send(ivc_devs->ivc_evt, 0x01);

    return;
}

//----------- 设备 read/write 封装（与你现有 ops 对接）
rt_size_t ivc_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    // rt_kprintf("ivc_write\n");
    RT_UNUSED(dev);
    RT_UNUSED(pos);
    int rc = ivc_rb_write(buffer, (uint32_t)size);
    return (rc > 0) ? (rt_size_t)rc : 0;
}

rt_size_t ivc_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    RT_UNUSED(dev);
    RT_UNUSED(pos);

    uint32_t want = (uint32_t)size;
    int rc;

    while (1)
    {
        rc = ivc_rb_read(buffer, &want);
        if (rc > 0)
        {
            // 成功读到数据，直接返回
            return (rt_size_t)rc;
        }
        else if (rc < 0)
        {
            // 出现错误，直接返回错误码
            return rc;
        }

        // 等待一次中断事件，改成超时 RT_WAITING_FOREVER --> rt_tick_from_millisecond(100)
        rt_uint32_t recved = 0;
        rt_err_t evt_ret = rt_event_recv(ivc_devs->ivc_evt,
                                         0x01,
                                         RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                                         rt_tick_from_millisecond(100),
                                         &recved);
        if (evt_ret != RT_EOK)
        {
            return evt_ret;
        }
    }
}


static rt_err_t ivc_open(rt_device_t dev, rt_uint16_t oflag)
{
    // rt_kprintf("[IVC] 调用ivc_open！\n");
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
    .control = ivc_control

};


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
    ivc_devs->ivc_evt = rt_event_create("ivc_evt", RT_IPC_FLAG_FIFO);

    g_ivc_serial.dev.parent.type = RT_Object_Class_Device;   // 类型：设备对象
    g_ivc_serial.dev.parent.flag = 0;                        // 标志位：默认0
    rt_memset(g_ivc_serial.dev.parent.name, 0, RT_NAME_MAX); // 初始化名称（清空）

    g_ivc_serial.dev.type = RT_Device_Class_Char;
    g_ivc_serial.dev.ref_count = 0;             // 引用计数初始化为0
    g_ivc_serial.dev.user_data = &g_ivc_serial; // 绑定用户数据
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
    rt_thread_t th = rt_thread_create("ivcpoll", (void *)ivc_poll, NULL, 8192, 20, 100000);
    if (th)
        rt_thread_startup(th);
}
MSH_CMD_EXPORT(rt_ivc_poll, ivc device init);


#include <stddef.h>
#define __MMU_INTERNAL
#include "mm_page.h"
#include "mmu.h"
#include "tlb.h"
#include "ioremap.h"
#ifdef RT_USING_SMART
#include <lwp_mm.h>
#endif
#include <rthw.h>
#include <mm_aspace.h>

static unsigned long *rt_hw_mmu_query(rt_aspace_t aspace, void *vaddr, int *plvl_shf);
static int cmd_check_mair(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: %s <virtual_addr>\n", argv[0]);
        return -1;
    }

    /* 1️⃣ 解析用户输入的虚拟地址 */
    unsigned long long va = strtoull(argv[1], NULL, 0);
    rt_kprintf("Input VA = 0x%llx\n", va);

    /* 2️⃣ 查询页表项(PTE) */
    int level_shift;
    rt_ubase_t *pte = rt_hw_mmu_query(&rt_kernel_space, (void *)va, &level_shift);
    if (!pte)
    {
        rt_kprintf("Translation fault: no mapping for 0x%llx\n", va);
        return -1;
    }

    rt_kprintf("PTE = 0x%lx\n", *pte);

    /* 3️⃣ 提取 AttrIndx */
    unsigned int attridx = (*pte >> 2) & 0x7;
    rt_kprintf("AttrIndx = %u\n", attridx);

    /* 4️⃣ 读取 MAIR_EL1 */
    unsigned long long mair;
    __asm__ volatile("mrs %0, mair_el1" : "=r"(mair));
    rt_kprintf("MAIR_EL1 = 0x%016llx\n", mair);

    /* 5️⃣ 解析该索引对应的 8bit 属性 */
    unsigned int mair_val = (mair >> (attridx * 8)) & 0xff;
    rt_kprintf("MAIR_EL1[slot %u] = 0x%02x\n", attridx, mair_val);

    /*
       常见值对照:
         0x00 -> Device-nGnRnE
         0x44 -> Normal Non-cacheable (Write-Combine)
         0xff -> Normal Write-Back, Read/Write Allocate
    */

    return 0;
}

/* 注册到 msh 命令行 */
MSH_CMD_EXPORT(cmd_check_mair, check MAIR attr of a virtual address);


#define TCR_CONFIG_TBI0 rt_hw_mmu_config_tbi(0)
#define TCR_CONFIG_TBI1 rt_hw_mmu_config_tbi(1)

#define MMU_LEVEL_MASK   0x1ffUL
#define MMU_LEVEL_SHIFT  9
#define MMU_ADDRESS_BITS 39
#define MMU_ADDRESS_MASK 0x0000fffffffff000UL
#define MMU_ATTRIB_MASK  0xfff0000000000ffcUL

#define MMU_TYPE_MASK  3UL
#define MMU_TYPE_USED  1UL
#define MMU_TYPE_BLOCK 1UL
#define MMU_TYPE_TABLE 3UL
#define MMU_TYPE_PAGE  3UL

#define MMU_TBL_BLOCK_2M_LEVEL 2
#define MMU_TBL_PAGE_4k_LEVEL  3
#define MMU_TBL_LEVEL_NR       4

/* restrict virtual address on usage of RT_NULL */
#ifndef KERNEL_VADDR_START
#define KERNEL_VADDR_START 0x1000
#endif
static unsigned long *rt_hw_mmu_query(rt_aspace_t aspace, void *vaddr, int *plvl_shf)
{
    int level;
    unsigned long va = (unsigned long)vaddr;
    unsigned long *cur_lv_tbl;
    unsigned long page;
    unsigned long off;
    int level_shift = MMU_ADDRESS_BITS;

    cur_lv_tbl = aspace->page_table;
    RT_ASSERT(cur_lv_tbl);

    for (level = 0; level < MMU_TBL_PAGE_4k_LEVEL; level++)
    {
        off = (va >> level_shift);
        off &= MMU_LEVEL_MASK;

        if (!(cur_lv_tbl[off] & MMU_TYPE_USED))
        {
            *plvl_shf = level_shift;
            return (void *)0;
        }

        page = cur_lv_tbl[off];
        if ((page & MMU_TYPE_MASK) == MMU_TYPE_BLOCK)
        {
            *plvl_shf = level_shift;
            return &cur_lv_tbl[off];
        }

        cur_lv_tbl = (unsigned long *)(page & MMU_ADDRESS_MASK);
        cur_lv_tbl = (unsigned long *)((unsigned long)cur_lv_tbl - PV_OFFSET);
        level_shift -= MMU_LEVEL_SHIFT;
    }
    /* now is level MMU_TBL_PAGE_4k_LEVEL */
    off = (va >> ARCH_PAGE_SHIFT);
    off &= MMU_LEVEL_MASK;
    page = cur_lv_tbl[off];

    *plvl_shf = level_shift;
    if (!(page & MMU_TYPE_USED))
    {
        return (void *)0;
    }
    return &cur_lv_tbl[off];
}
