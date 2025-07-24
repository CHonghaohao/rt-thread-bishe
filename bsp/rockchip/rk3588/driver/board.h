/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-5-30      Bernard      the first version
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include <rk3588.h>
#include <rtthread.h>

#define rt_inline                   static __inline

#define UART_MMIO_SIZE  0x100

#define UART_IRQ_BASE   (363)
#define UART0_IRQ       (UART_IRQ_BASE + 0)
#define UART1_IRQ       (UART_IRQ_BASE + 1)
#define UART2_IRQ       (UART_IRQ_BASE + 2)
#define UART3_IRQ       (UART_IRQ_BASE + 3)
#define UART4_IRQ       (UART_IRQ_BASE + 4)
#define UART5_IRQ       (UART_IRQ_BASE + 5)
#define UART6_IRQ       (UART_IRQ_BASE + 6)
#define UART7_IRQ       (UART_IRQ_BASE + 7)
#define UART8_IRQ       (UART_IRQ_BASE + 8)
#define UART9_IRQ       (UART_IRQ_BASE + 9)



/* GIC */
#define MAX_HANDLERS        454
#define GIC_IRQ_START       0
#define ARM_GIC_NR_IRQS     454
#define ARM_GIC_MAX_NR      1

#define IRQ_ARM_IPI_KICK    0
#define IRQ_ARM_IPI_CALL    1

#define GIC_PL600_DISTRIBUTOR_PPTR      0xfe600000
#define GIC_PL600_REDISTRIBUTOR_PPTR    0xfe660000
#define GIC_PL600_CONTROLLER_PPTR       RT_NULL
#define GIC_PL600_ITS_PPTR              0xfe640000

rt_inline uint32_t platform_get_gic_dist_base(void)
{
    return GIC_PL600_DISTRIBUTOR_PPTR;
}

rt_inline uint32_t platform_get_gic_redist_base(void)
{
    return GIC_PL600_REDISTRIBUTOR_PPTR;
}

rt_inline uint32_t platform_get_gic_cpu_base(void)
{
    return GIC_PL600_CONTROLLER_PPTR;
}

rt_inline uint32_t platform_get_gic_its_base(void)
{
    return GIC_PL600_ITS_PPTR;
}

/* VirtIO */
#define VIRTIO_MMIO_BASE    0xff9e0000
#define VIRTIO_MMIO_SIZE    0x00000200
#define VIRTIO_MAX_NR       1
#define VIRTIO_IRQ_BASE     (64 + 16)
// #define VIRTIO_VENDOR_ID    0x554d4551  /* "QEMU" */
#define VIRTIO_VENDOR_ID    0x48564953  /* "hvisor-tool" */

extern unsigned char __bss_start;
extern unsigned char __bss_end;

#define RT_HW_PAGE_START    RT_ALIGN((unsigned long)&__bss_end, 0x1000)
#define RT_HW_PAGE_END      (RT_HW_PAGE_START + 0x100000)

#define RT_HW_HEAP_BEGIN    (void *)(RT_HW_PAGE_END)
#define RT_HW_HEAP_END      (void *)(RT_HW_HEAP_BEGIN + 64 * 1024 * 1024)

void rt_hw_board_init(void);

#endif /* __BOARD_H__ */
