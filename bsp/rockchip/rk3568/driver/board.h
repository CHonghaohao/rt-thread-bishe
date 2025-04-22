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

#include "rtthread.h"

extern unsigned char __bss_start;
extern unsigned char __bss_end;

#define RT_HW_PAGE_START    RT_ALIGN((unsigned long)&__bss_end, 0x1000)
#define RT_HW_PAGE_END      (RT_HW_PAGE_START + 0x100000)

#define RT_HW_HEAP_BEGIN    (void *)(RT_HW_PAGE_END)
#define RT_HW_HEAP_END      (void *)(RT_HW_HEAP_BEGIN + 64 * 1024 * 1024)

// #define GRF_PMU_BASE    0xFDC20000
// #define GRF_SYS_BASE    0xFDC60000
// #define CRU_BASE        0xFDD20000
#define GRF_PMU_BASE    0xFD588000
#define GRF_SYS_BASE    0xFD58C000
#define CRU_BASE        0xFD7C0000

 /* UART */
// #define UART_MMIO_BASE  0xfe650000
// #define UART0_MMIO_BASE 0xfdd50000
#define UART_MMIO_BASE  0xfeb40000
#define UART0_MMIO_BASE 0xfd890000
#define UART1_MMIO_BASE (UART_MMIO_BASE + 0)
#define UART2_MMIO_BASE (UART_MMIO_BASE + 0x10000)
#define UART3_MMIO_BASE (UART_MMIO_BASE + 0x20000)
#define UART4_MMIO_BASE (UART_MMIO_BASE + 0x30000)
#define UART5_MMIO_BASE (UART_MMIO_BASE + 0x40000)
#define UART6_MMIO_BASE (UART_MMIO_BASE + 0x50000)
#define UART7_MMIO_BASE (UART_MMIO_BASE + 0x60000)
#define UART8_MMIO_BASE (UART_MMIO_BASE + 0x70000)
#define UART9_MMIO_BASE (UART_MMIO_BASE + 0x80000)

#define UART_MMIO_SIZE  0x100

// #define UART_IRQ_BASE   (32 + 116)
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

/* GPIO */
// #define GPIO0_MMIO_BASE 0xfdd60000
// #define GPIO1_MMIO_BASE 0xfe740000
// #define GPIO2_MMIO_BASE 0xfe750000
// #define GPIO3_MMIO_BASE 0xfe760000
// #define GPIO4_MMIO_BASE 0xfe770000
#define GPIO0_MMIO_BASE 0xfd8a0000
#define GPIO1_MMIO_BASE 0xfec20000
#define GPIO2_MMIO_BASE 0xfec30000
#define GPIO3_MMIO_BASE 0xfec40000
#define GPIO4_MMIO_BASE 0xfec50000

#define GPIO_MMIO_SIZE  0x100

// #define GPIO_IRQ_BASE   (32 + 33)
#define GPIO_IRQ_BASE   (309)
#define GPIO0_IRQ       (GPIO_IRQ_BASE + 0)
#define GPIO1_IRQ       (GPIO_IRQ_BASE + 1)
#define GPIO2_IRQ       (GPIO_IRQ_BASE + 2)
#define GPIO3_IRQ       (GPIO_IRQ_BASE + 3)
#define GPIO4_IRQ       (GPIO_IRQ_BASE + 4)

/* MMC */
// #define MMC0_MMIO_BASE  0xfe310000  /* sdhci */
// #define MMC1_MMIO_BASE  0xfe2b0000  /* sdmmc0 */
// #define MMC2_MMIO_BASE  0xfe2c0000  /* sdmmc1 */
// #define MMC3_MMIO_BASE  0xfe000000  /* sdmmc2 */
#define MMC0_MMIO_BASE  0xfe2e0000  /* sdhci */
#define MMC1_MMIO_BASE  0xfe2b0000  /* sdmmc0 */
#define MMC2_MMIO_BASE  0xfe2c0000  /* sdmmc1 */
#define MMC3_MMIO_BASE  0xfe000000  /* sdmmc2 */

#define MMC0_MMIO_SIZE  0x10000
#define MMC_MMIO_SIZE   0x4000

// #define MMC0_IRQ        (32 + 19)
// #define MMC1_IRQ        (32 + 98)
// #define MMC2_IRQ        (32 + 99)
// #define MMC3_IRQ        (32 + 100)

/* Ethernet */
// #define GMAC0_MMIO_BASE 0xfe2a0000
// #define GMAC1_MMIO_BASE 0xfe010000

// #define GMAC_MMIO_SIZE  0x10000

// #define GMAC0_MAC_IRQ   (32 + 27)
// #define GMAC0_WAKE_IRQ  (32 + 24)
// #define GMAC1_MAC_IRQ   (32 + 32)
// #define GMAC1_WAKE_IRQ  (32 + 29)

/* GIC */
#define MAX_HANDLERS        454
#define GIC_IRQ_START       0
#define ARM_GIC_NR_IRQS     454
#define ARM_GIC_MAX_NR      1

#define IRQ_ARM_IPI_KICK    0
#define IRQ_ARM_IPI_CALL    1

// #define GIC_PL600_DISTRIBUTOR_PPTR      0xfd400000
// #define GIC_PL600_REDISTRIBUTOR_PPTR    0xfd460000
// #define GIC_PL600_CONTROLLER_PPTR       RT_NULL
// #define GIC_PL600_ITS_PPTR              0xfd440000
#define GIC_PL600_DISTRIBUTOR_PPTR      0xfe600000
#define GIC_PL600_REDISTRIBUTOR_PPTR    0xfe660000
#define GIC_PL600_CONTROLLER_PPTR       RT_NULL
#define GIC_PL600_ITS_PPTR              0xfe640000

rt_inline rt_uint32_t platform_get_gic_dist_base(void)
{
    return GIC_PL600_DISTRIBUTOR_PPTR;
}

rt_inline rt_uint32_t platform_get_gic_redist_base(void)
{
    return GIC_PL600_REDISTRIBUTOR_PPTR;
}

rt_inline rt_uint32_t platform_get_gic_cpu_base(void)
{
    return GIC_PL600_CONTROLLER_PPTR;
}

rt_inline rt_uint32_t platform_get_gic_its_base(void)
{
    return GIC_PL600_ITS_PPTR;
}

void rt_hw_board_init(void);

#endif /* __BOARD_H__ */
