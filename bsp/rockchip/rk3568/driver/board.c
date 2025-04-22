/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-3-08      GuEe-GUI     the first version
 */

#include <rthw.h>
#include <rtthread.h>

#include <mmu.h>
#include <rtdevice.h>
#include <gicv3.h>
#include <gtimer.h>
#include <cpuport.h>
#include <interrupt.h>
#include <ioremap.h>
#include <psci.h>
#include <board.h>
#include <drv_uart.h>

#include "mm_page.h"
#include "rk3588.h"
#include "hal_pinctrl.h"
#include "hal_canfd.h"
#include "hal_timer.h"
#include <gtimer.h>

#define  CANTRQ  375



#define PLATFORM_MEM_TALBE(va, size) va, ((unsigned long)va + size - 1)


// struct PMU1_IOC_REG *virt_PMU1_IOC_BASE;
// struct PMU2_IOC_REG *virt_PMU2_IOC_BASE;
// struct BUS_IOC_REG *virt_BUS_IOC_BASE;

struct mem_desc platform_mem_desc[] =
{
    {PLATFORM_MEM_TALBE(0x20000000,                    0x10000000), 0x20000000,                   NORMAL_MEM},
    {PLATFORM_MEM_TALBE(GRF_PMU_BASE,                  0x10000),    GRF_PMU_BASE,                 DEVICE_MEM},
    {PLATFORM_MEM_TALBE(GRF_SYS_BASE,                  0x10000),    GRF_SYS_BASE,                 DEVICE_MEM},
    {PLATFORM_MEM_TALBE(CRU_BASE,                      0x10000),    CRU_BASE,                     DEVICE_MEM},
    {PLATFORM_MEM_TALBE(UART0_MMIO_BASE,               0x10000),    UART0_MMIO_BASE,              DEVICE_MEM},
    {PLATFORM_MEM_TALBE(UART1_MMIO_BASE,               0x90000),    UART1_MMIO_BASE,              DEVICE_MEM},
    {PLATFORM_MEM_TALBE(GIC_PL600_DISTRIBUTOR_PPTR,    0x10000),    GIC_PL600_DISTRIBUTOR_PPTR,   DEVICE_MEM},
    {PLATFORM_MEM_TALBE(GIC_PL600_REDISTRIBUTOR_PPTR,  0xc0000),    GIC_PL600_REDISTRIBUTOR_PPTR, DEVICE_MEM},
    
    
    /*add pinctrl mem */
    {PLATFORM_MEM_TALBE(PMU1_IOC_BASE,  0x4000),                    PMU1_IOC_BASE, DEVICE_MEM},
    {PLATFORM_MEM_TALBE(PMU2_IOC_BASE,  0x4000),                    PMU2_IOC_BASE, DEVICE_MEM},
    {PLATFORM_MEM_TALBE(BUS_IOC_BASE,  0x4000),                    BUS_IOC_BASE, DEVICE_MEM},
    {PLATFORM_MEM_TALBE(VCCIO1_4_IOC_BASE,  0x4000),                    VCCIO1_4_IOC_BASE, DEVICE_MEM},
    {PLATFORM_MEM_TALBE(VCCIO3_5_IOC_BASE,  0x4000),                   VCCIO3_5_IOC_BASE, DEVICE_MEM},
    {PLATFORM_MEM_TALBE(VCCIO2_IOC_BASE,  0x4000),                    VCCIO2_IOC_BASE, DEVICE_MEM},
    {PLATFORM_MEM_TALBE(EMMC_IOC_BASE,  0x4000),                      EMMC_IOC_BASE, DEVICE_MEM},
    {PLATFORM_MEM_TALBE(VCCIO6_IOC_BASE,  0x4000),                    VCCIO6_IOC_BASE, DEVICE_MEM},
    {PLATFORM_MEM_TALBE(CAN2_BASE,  0x10000),                    CAN2_BASE, DEVICE_MEM},
    {PLATFORM_MEM_TALBE(TIMER0_BASE,  0x8000),                    TIMER5_BASE, DEVICE_MEM},
#ifdef PKG_USING_RT_OPENAMP
    {PLATFORM_MEM_TALBE(AMP_SHARE_MEMORY_ADDRESS, AMP_SHARE_MEMORY_SIZE), AMP_SHARE_MEMORY_ADDRESS, NORMAL_MEM},
#endif /* PKG_USING_RT_OPENAMP */
};


const rt_uint32_t platform_mem_desc_size = sizeof(platform_mem_desc) / sizeof(platform_mem_desc[0]);

void idle_wfi(void)
{
    __asm__ volatile ("wfi");      //休眠
}


static void can_isr(int irqno, void *param)
{  
    struct CANFD_MSG recvbuff;
   if(irqno == CANTRQ )
   {   
       
       
       rt_kprintf("CAN receive !!!\n");
       HAL_CANFD_Receive((struct CAN_REG *)param,&recvbuff);

       rt_kprintf("buff is %*.s\n",64,recvbuff.data);

       recvbuff.extId += 1;
       recvbuff.ide = CANFD_ID_EXTENDED;

       HAL_CANFD_Transmit((struct CAN_REG *)param,&recvbuff);


       HAL_CANFD_GetInterrupt((struct CAN_REG *)param);
   }
}



void can_init(void)
{  

   
    /*pinctrl*/

      
    HAL_PINCTRL_SetParam(GPIO_BANK3,GPIO_PIN_C5,PIN_CONFIG_MUX_FUNC9 | PIN_CONFIG_PUL_NORMAL |PIN_CONFIG_DRV_LEVEL2);
    HAL_PINCTRL_SetParam(GPIO_BANK3,GPIO_PIN_C4,PIN_CONFIG_MUX_FUNC9 | PIN_CONFIG_PUL_NORMAL |PIN_CONFIG_DRV_LEVEL2);
   


    /*can controller init*/
    struct CANFD_CONFIG  can2test = {
        .canfdMode= CANFD_MODE_FD,
        .bps = CANFD_BPS_500KBAUD,
    };
    

    HAL_CANFD_Init(CAN2_BASE,&can2test);
    HAL_CANFD_Start(CAN2_BASE);
    
    /*rt_hw_interrupt config*/
    rt_hw_interrupt_umask(CANTRQ);  //enabled can irq
    rt_hw_interrupt_install(CANTRQ, can_isr, (void *)CAN2_BASE, "can_uart");

    struct CANFD_MSG sendbuff;

    for(int i =0; i < 20; i++)
    {   
        sendbuff.extId = 0x123456;
        sendbuff.ide = CANFD_ID_EXTENDED;
        //sendbuff.fdf = CANFD_FD_FORMAT;

        memcpy(sendbuff.data,"hello!",7);
        HAL_CANFD_Transmit(CAN2_BASE,&sendbuff);
    }

}


void rk3588_uart7_changem2(void)
{
    // virt_PMU1_IOC_BASE = rt_ioremap(PMU1_IOC_BASE,0x4000);
    // virt_PMU2_IOC_BASE = rt_ioremap(PMU2_IOC_BASE,0x4000);
    // virt_BUS_IOC_BASE = rt_ioremap(BUS_IOC_BASE,0x4000);

     
    HAL_PINCTRL_SetParam(GPIO_BANK1,GPIO_PIN_B4,PIN_CONFIG_MUX_FUNC10 | PIN_CONFIG_PUL_UP | PIN_CONFIG_DRV_LEVEL1);
    HAL_PINCTRL_SetParam(GPIO_BANK1,GPIO_PIN_B5,PIN_CONFIG_MUX_FUNC10 | PIN_CONFIG_PUL_UP | PIN_CONFIG_DRV_LEVEL1);
    

    //uart  pullup  and  pull down

    
    //HAL_PINCTRL_SetIOMUX(GPIO_BANK4,GPIO_PIN_A3,PIN_CONFIG_MUX_FUNC10);
    //HAL_PINCTRL_SetIOMUX(GPIO_BANK4,GPIO_PIN_A4,PIN_CONFIG_MUX_FUNC10);

    // HAL_PINCTRL_SetParam(GPIO_BANK4,GPIO_PIN_A3,PIN_CONFIG_MUX_FUNC10 | 
    // PIN_CONFIG_PUL_UP | PIN_CONFIG_DRV_LEVEL1 );
    // HAL_PINCTRL_SetParam(GPIO_BANK4,GPIO_PIN_A4,PIN_CONFIG_MUX_FUNC10 | PIN_CONFIG_PUL_UP |PIN_CONFIG_DRV_LEVEL1);

}

static void rt_hw_timer_isr_brain(int vector, void *parameter)
{   
    rt_tick_increase();
    HAL_TIMER_ClrInt((struct TIMER_REG *)parameter);   
}


void brain_timer_init(void)
{   //338
    HAL_TIMER_Init(TIMER5_BASE,TIMER_FREE_RUNNING);
    HAL_TIMER_SetCount(TIMER5_BASE,24*10000);            //24M 每10ms中断一次

    rt_hw_interrupt_umask(TIMER5_IRQn);
    rt_hw_interrupt_install(TIMER5_IRQn, rt_hw_timer_isr_brain, TIMER5_BASE, "tick");
    HAL_TIMER_Start_IT(TIMER5_BASE);

}


void rt_hw_board_init(void)
{
    extern unsigned long MMUTable[512];
    rt_region_t init_page_region;

    rt_hw_mmu_map_init(&rt_kernel_space, (void *) 0x20000000, 0xE0000000 - 1, MMUTable, 0);
    
    init_page_region.start = RT_HW_PAGE_START;
    init_page_region.end = RT_HW_PAGE_END;
    
    rt_page_init(init_page_region);//light-1
    
    rt_hw_mmu_setup(&rt_kernel_space, platform_mem_desc, platform_mem_desc_size);
    
#ifdef RT_USING_HEAP
    /* initialize memory system */
    rt_system_heap_init(RT_HW_HEAP_BEGIN, RT_HW_HEAP_END);
#endif
    /* initialize hardware interrupt */
    rt_hw_interrupt_init();
    
    /* initialize uart */
    rk3588_uart7_changem2(); 
    rt_hw_uart_init();

    can_init();

    /* initialize timer for os tick */
    //rt_hw_gtimer_init();  
    brain_timer_init();

    rt_thread_idle_sethook(idle_wfi);
    
#if defined(RT_USING_CONSOLE) && defined(RT_USING_DEVICE)
    /* set console device */
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif

#ifdef RT_USING_HEAP
    /* initialize memory system */
    rt_kprintf("heap: [0x%08x - 0x%08x]\n", RT_HW_HEAP_BEGIN, RT_HW_HEAP_END);
#endif

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

#ifdef RT_USING_SMP
    /* install IPI handle */
    rt_hw_ipi_handler_install(RT_SCHEDULE_IPI, rt_scheduler_ipi_handler);
    arm_gic_umask(0, IRQ_ARM_IPI_KICK);
#endif
}

void reboot(void)
{
    psci_system_reboot();

    void *cur_base = rt_ioremap((void *) CRU_BASE, 0x100);
    HWREG32(cur_base + 0x00D4) = 0xfdb9;
    HWREG32(cur_base + 0x00D8) = 0xeca8;
}
MSH_CMD_EXPORT(reboot, reboot...);

static void print_cpu_id(int argc, char *argv[])
{
    rt_kprintf("rt_hw_cpu_id:%d\n", rt_hw_cpu_id());
}
MSH_CMD_EXPORT_ALIAS(print_cpu_id, cpuid, print_cpu_id);

#ifdef RT_USING_AMP
void start_cpu(int argc, char *argv[])
{
    rt_uint32_t status;
    status = rt_psci_cpu_on(0x3, (rt_uint64_t) 0x7A000000);
    rt_kprintf("arm_psci_cpu_on 0x%X\n", status);
}
MSH_CMD_EXPORT(start_cpu, start_cpu);

#ifdef RT_AMP_SLAVE
void rt_hw_cpu_shutdown(void)
{
    rt_psci_cpu_off(0);
}
#endif /* RT_AMP_SLAVE */
#endif /* RT_USING_AMP */

#if defined(RT_USING_SMP) || defined(RT_USING_AMP)
rt_uint64_t rt_cpu_mpidr_early[] =
{
    [0] = 0x80000000,
    [1] = 0x80000100,
    [2] = 0x80000200,
    [3] = 0x80000300,
    [RT_CPUS_NR] = 0
};
#endif

#ifdef RT_USING_SMP
void rt_hw_secondary_cpu_up(void)
{
    int i;
    extern void _secondary_cpu_entry(void);
    rt_uint64_t entry = (rt_uint64_t)rt_kmem_v2p(_secondary_cpu_entry);

    for (i = 1; i < RT_CPUS_NR; ++i)
    {
        rt_psci_cpu_on(rt_cpu_mpidr_early[i], entry);
    }
}

extern unsigned long MMUTable[];

void secondary_cpu_c_start(void)
{
    rt_hw_mmu_ktbl_set((unsigned long)MMUTable);
    rt_hw_spin_lock(&_cpus_lock);

    arm_gic_cpu_init(0, platform_get_gic_cpu_base());
    arm_gic_redist_init(0, platform_get_gic_redist_base());
    rt_hw_vector_init();
    rt_hw_gtimer_local_enable();
    arm_gic_umask(0, IRQ_ARM_IPI_KICK);

    rt_kprintf("\rcall cpu %d on success\n", rt_hw_cpu_id());

    rt_system_scheduler_start();
}

void rt_hw_secondary_cpu_idle_exec(void)
{
    rt_hw_wfe();
}
#endif
