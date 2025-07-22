/**
  * Copyright (c) 2021 Fuzhou Rockchip Electronics Co., Ltd
  *
  * SPDX-License-Identifier: Apache-2.0
  ******************************************************************************
  * @file    drv_gmac.c
  * @author  David Wu
  * @version V0.1
  * @date    01-Jul-2021
  * @brief   ETHERNET Driver
  *
  ******************************************************************************
  */

/** @addtogroup RKBSP_Driver_Reference
 *  @{
 */

/** @addtogroup ETHERNET
 *  @{
 */

/** @defgroup ETHERNET_How_To_Use How To Use
 *  @{

  See more information, click [here](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/netdev/netdev)

 @} */

#include <rthw.h>
// #include "drv_pm.h"
#include "drv_heap.h"
#include "hal_bsp.h"
#include "hal_gmac.h"
#include "drv_clock.h"
#include "drv_gmac.h"
#ifdef RT_USING_PM
#include <drivers/pm.h>
#endif

#ifdef RT_USING_GMAC



#ifdef RT_USING_LWIP

#include <netif/ethernetif.h>
#include "lwipopts.h"

#define RK_GMAC_DEBUG 0

#define MAX_ADDR_LEN 6
#define ETH_RXBUFNB 10
#define ETH_TXBUFNB 10

static void rockchip_eth_irq(int irq, void *param);

#if RK_GMAC_DEBUG
#define rk_gmac_dbg(dev, fmt, ...) \
    rt_kprintf("%s: " fmt, ((struct rt_device *)dev)->parent.name, ##__VA_ARGS__)

#define __is_print(ch) ((unsigned int)((ch) - ' ') < 127u - ' ')
static void rt_rockchip_dump_hex(const rt_uint8_t *ptr, rt_size_t buflen)
{
    unsigned char *buf = (unsigned char *)ptr;
    int i, j;

    rt_kprintf("len: 0x%X: ", buflen);
    for (i = 0; i < buflen; i += 16)
    {
        rt_kprintf("%08X: ", i);

        for (j = 0; j < 16; j++)
            if (i + j < buflen)
                rt_kprintf("%02X ", buf[i + j]);
            else
                rt_kprintf("   ");
        rt_kprintf(" ");

        for (j = 0; j < 16; j++)
            if (i + j < buflen)
                rt_kprintf("%c", __is_print(buf[i + j]) ? buf[i + j] : '.');
        rt_kprintf("\n");
    }
}
#else
#define rk_gmac_dbg(dev, fmt, ...) \
do { \
} while(0)

static void rt_rockchip_dump_hex(const rt_uint8_t *ptr, rt_size_t buflen)
{
}
#endif

/* GMAC controller driver's private data. */
struct rockchip_eth
{
    /* inherit from ethernet device */
    struct eth_device parent;

    struct rt_semaphore sem_lock;
    struct rt_event tx_event;

    /* interface address info, hw address */
    rt_uint8_t  dev_addr[MAX_ADDR_LEN];

    const char *name;
    rt_uint8_t id;
    uint32_t pmState;

    const struct rockchip_eth_config *config;

    /* irq handler */
    rt_isr_handler_t irq_handler;

    /* HAL */
    struct GMAC_HANDLE instance;
    const struct HAL_GMAC_DEV *dev;

    struct GMAC_Desc  *rx_desc;
    struct GMAC_Desc  *tx_desc;

    rt_uint8_t *rx_buff;
    rt_uint8_t *tx_buff;
};

extern void rt_hw_cpu_dcache_invalidate(void *addr, int size);
extern void rt_hw_cpu_dcache_clean(void *addr, int size);

/* interrupt service routine */
void rt_rockchip_eth_irq(struct rockchip_eth *eth)
{
    rt_uint32_t status;
    /* enter interrupt */
    rt_interrupt_enter();
    status = HAL_GMAC_IRQHandler(&eth->instance);

    if (status & DMA_HANLE_RX)
    {
        /* a frame has been received */
        eth_device_ready(&eth->parent);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}

/**
 * is_zero_ethaddr - Determine if give Ethernet address is all zeros.
 * @addr: Pointer to a six-byte array containing the Ethernet address
 *
 * Return true if the address is all zeroes.
 */
static inline int is_zero_ethaddr(const rt_uint8_t *addr)
{
    return !(addr[0] | addr[1] | addr[2] | addr[3] | addr[4] | addr[5]);
}

/**
 * is_multicast_ethaddr - Determine if the Ethernet address is a multicast.
 * @addr: Pointer to a six-byte array containing the Ethernet address
 *
 * Return true if the address is a multicast address.
 * By definition the broadcast address is also a multicast address.
 */
static inline int is_multicast_ethaddr(const rt_uint8_t *addr)
{
    return 0x01 & addr[0];
}

/*
 * is_broadcast_ethaddr - Determine if the Ethernet address is broadcast
 * @addr: Pointer to a six-byte array containing the Ethernet address
 *
 * Return true if the address is the broadcast address.
 */
static inline int is_broadcast_ethaddr(const rt_uint8_t *addr)
{
    return (addr[0] & addr[1] & addr[2] & addr[3] & addr[4] & addr[5]) ==
           0xff;
}

/*
 * is_valid_ethaddr - Determine if the given Ethernet address is valid
 * @addr: Pointer to a six-byte array containing the Ethernet address
 *
 * Check that the Ethernet address (GMAC) is not 00:00:00:00:00:00, is not
 * a multicast address, and is not FF:FF:FF:FF:FF:FF.
 *
 * Return true if the address is valid.
 */
static inline int is_valid_ethaddr(const rt_uint8_t *addr)
{
    /* FF:FF:FF:FF:FF:FF is a multicast address so we don't need to
     * explicitly check for it here. */
    return !is_multicast_ethaddr(addr) && !is_zero_ethaddr(addr);
}

/**
 * net_random_ethaddr - Generate software assigned random Ethernet address
 * @addr: Pointer to a six-byte array containing the Ethernet address
 *
 * Generate a random Ethernet address (GMAC) that is not multicast
 * and has the local assigned bit set.
 */
static inline void net_random_ethaddr(rt_uint8_t *addr)
{
    unsigned int seed = HAL_TIMER_GetCount(TIMER5_BASE) & 0xffffffff;
    rt_int8_t i;

    for (i = 0; i < 6; i++)
        addr[i] = rand_r(&seed);

    addr[0] &= 0xfe;    /* clear multicast bit */
    addr[0] |= 0x02;    /* set local assignment bit (IEEE802) */
}

/* initialization function */
static rt_err_t rt_rockchip_eth_init(rt_device_t dev)
{
    struct rockchip_eth *eth = (struct rockchip_eth *)dev;
    struct GMAC_HANDLE *pGMAC;
    rt_err_t ret;
    pGMAC = &eth->instance;

    eth->rx_desc = rt_malloc_uncache((ETH_RXBUFNB) * sizeof(struct GMAC_Desc));
    if (!eth->rx_desc)
    {
        rk_gmac_dbg(&eth->parent, "rt_malloc_uncache failed\n");
        return -RT_ENOMEM;
    }

    eth->tx_desc = rt_malloc_uncache((ETH_TXBUFNB) * sizeof(struct GMAC_Desc));
    if (!eth->tx_desc)
    {
        rk_gmac_dbg(&eth->parent, "rt_malloc_uncache failed\n");
        ret = -RT_ENOMEM;
        goto err;
    }
    
    rt_memset(eth->rx_desc, 0, sizeof(struct GMAC_Desc) * (ETH_RXBUFNB));
    rt_memset(eth->tx_desc, 0, sizeof(struct GMAC_Desc) * (ETH_TXBUFNB));
    eth->rx_buff = rt_malloc_align(HAL_GMAC_MAX_PACKET_SIZE * ETH_RXBUFNB, HAL_GMAC_MAX_PACKET_SIZE);
    if (!eth->rx_buff)
    {
        rk_gmac_dbg(&eth->parent, "rt_malloc_align failed\n");
        ret = -RT_ENOMEM;
        goto err;
    }
    eth->tx_buff = rt_malloc_align(HAL_GMAC_MAX_PACKET_SIZE * ETH_TXBUFNB, HAL_GMAC_MAX_PACKET_SIZE);
    if (!eth->tx_buff)
    {
        rk_gmac_dbg(&eth->parent, "rt_malloc_align failed\n");
        ret = -RT_ENOMEM;
        goto err;
    }

    rt_memset(eth->rx_buff, 0, HAL_GMAC_MAX_PACKET_SIZE * ETH_RXBUFNB);
    rt_memset(eth->tx_buff, 0, HAL_GMAC_MAX_PACKET_SIZE * ETH_TXBUFNB);
    /* Initialize Rx Descriptors list */
    HAL_GMAC_DMARxDescInit(pGMAC, eth->rx_desc, eth->rx_buff, ETH_RXBUFNB);
    /* Initialize Tx Descriptors list */
    HAL_GMAC_DMATxDescInit(pGMAC, eth->tx_desc, eth->tx_buff, ETH_TXBUFNB);
    /* Register irq */
    rt_hw_interrupt_install(eth->dev->irqNum, rockchip_eth_irq, eth, eth->parent.parent.parent.name); ///eth
    rt_hw_interrupt_umask(eth->dev->irqNum);

    // clk_enable_by_id(eth->dev->pclkGateID);
    // clk_enable_by_id(eth->dev->clkGateID);

    /* Enable GMAC and DMA transmission and reception */

    ret = HAL_GMAC_Start(pGMAC, eth->dev_addr);
    if (ret == HAL_OK)
    {
        rk_gmac_dbg(&eth->parent, "GMAC hardware started\n");
        return RT_EOK;
    }
    else
    {
        rk_gmac_dbg(&eth->parent, "GMAC hardware start faild: %d\n", ret);
    }

err:
    if (eth->rx_desc)
        rt_free_uncache(eth->rx_desc);
    if (eth->tx_desc)
        rt_free_uncache(eth->tx_desc);

    if (eth->rx_buff)
        rt_free_align(eth->rx_buff);
    if (eth->tx_buff)
        rt_free_align(eth->tx_buff);

    return ret;
}

static rt_err_t rt_rockchip_eth_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t rt_rockchip_eth_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_size_t rt_rockchip_eth_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_set_errno(-RT_ENOSYS);
    return RT_EOK;
}

static rt_size_t rt_rockchip_eth_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    rt_set_errno(-RT_ENOSYS);
    return RT_EOK;
}

/* mac address control function */
static rt_err_t rt_rockchip_eth_control(rt_device_t dev, int cmd, void *args)
{
    struct rockchip_eth *eth = (struct rockchip_eth *)dev;

    switch (cmd)
    {
    case NIOCTL_GADDR:
        if (args) rt_memcpy(args, eth->dev_addr, 6);
        else return -RT_ERROR;
        break;

    default :
        break;
    }
    return RT_EOK;
}

/* phy hardware reset function */
static void rt_rockchip_phy_reset(const struct rockchip_eth_config *config)
{
#ifdef RT_USING_GMAC0
    HAL_PINCTRL_SetIOMUX(GPIO_BANK4, GPIO_PIN_C4, PIN_CONFIG_MUX_FUNC1 | PIN_CONFIG_PUL_UP | PIN_CONFIG_DRV_LEVEL1);//需要配置GMAC0_MDIO
    HAL_PINCTRL_SetIOMUX(GPIO_BANK4, GPIO_PIN_C5, PIN_CONFIG_MUX_FUNC1 | PIN_CONFIG_PUL_UP | PIN_CONFIG_DRV_LEVEL1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK2, GPIO_PIN_B6, PIN_CONFIG_MUX_FUNC1 | PIN_CONFIG_PUL_UP | PIN_CONFIG_DRV_LEVEL1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK2, GPIO_PIN_B7, PIN_CONFIG_MUX_FUNC1 | PIN_CONFIG_PUL_UP | PIN_CONFIG_DRV_LEVEL1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK2, GPIO_PIN_C0, PIN_CONFIG_MUX_FUNC1 | PIN_CONFIG_PUL_UP | PIN_CONFIG_DRV_LEVEL1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK2, GPIO_PIN_C1, PIN_CONFIG_MUX_FUNC1 | PIN_CONFIG_PUL_UP | PIN_CONFIG_DRV_LEVEL1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK2, GPIO_PIN_C2, PIN_CONFIG_MUX_FUNC1 | PIN_CONFIG_PUL_UP | PIN_CONFIG_DRV_LEVEL1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK4, GPIO_PIN_C2, PIN_CONFIG_MUX_FUNC1 | PIN_CONFIG_PUL_UP | PIN_CONFIG_DRV_LEVEL1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK2, GPIO_PIN_B0, PIN_CONFIG_MUX_FUNC1 | PIN_CONFIG_PUL_UP | PIN_CONFIG_DRV_LEVEL1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK2, GPIO_PIN_B3, PIN_CONFIG_MUX_FUNC1 | PIN_CONFIG_PUL_UP | PIN_CONFIG_DRV_LEVEL1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK2, GPIO_PIN_A6, PIN_CONFIG_MUX_FUNC1 | PIN_CONFIG_PUL_UP | PIN_CONFIG_DRV_LEVEL1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK2, GPIO_PIN_A7, PIN_CONFIG_MUX_FUNC1 | PIN_CONFIG_PUL_UP | PIN_CONFIG_DRV_LEVEL1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK2, GPIO_PIN_B1, PIN_CONFIG_MUX_FUNC1 | PIN_CONFIG_PUL_UP | PIN_CONFIG_DRV_LEVEL1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK2, GPIO_PIN_B2, PIN_CONFIG_MUX_FUNC1 | PIN_CONFIG_PUL_UP | PIN_CONFIG_DRV_LEVEL1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK4, GPIO_PIN_C3, PIN_CONFIG_MUX_FUNC1 | PIN_CONFIG_PUL_UP | PIN_CONFIG_DRV_LEVEL1);
#endif
#ifdef RT_USING_GMAC1
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3, GPIO_PIN_A6, PIN_CONFIG_MUX_FUNC1);//需要配置GMAC1_MDIO
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3, GPIO_PIN_C2, PIN_CONFIG_MUX_FUNC1);//需要配置GMAC1_MDIO
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3, GPIO_PIN_C3, PIN_CONFIG_MUX_FUNC1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3, GPIO_PIN_B3, PIN_CONFIG_MUX_FUNC1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3, GPIO_PIN_B4, PIN_CONFIG_MUX_FUNC1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3, GPIO_PIN_B5, PIN_CONFIG_MUX_FUNC1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3, GPIO_PIN_A7, PIN_CONFIG_MUX_FUNC1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3, GPIO_PIN_B0, PIN_CONFIG_MUX_FUNC1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3, GPIO_PIN_B1, PIN_CONFIG_MUX_FUNC1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3, GPIO_PIN_A5, PIN_CONFIG_MUX_FUNC1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3, GPIO_PIN_A4, PIN_CONFIG_MUX_FUNC1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3, GPIO_PIN_A0, PIN_CONFIG_MUX_FUNC1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3, GPIO_PIN_A1, PIN_CONFIG_MUX_FUNC1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3, GPIO_PIN_A2, PIN_CONFIG_MUX_FUNC1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3, GPIO_PIN_A3, PIN_CONFIG_MUX_FUNC1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3, GPIO_PIN_B6, PIN_CONFIG_MUX_FUNC1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3, GPIO_PIN_C1, PIN_CONFIG_MUX_FUNC1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3, GPIO_PIN_C0, PIN_CONFIG_MUX_FUNC1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3, GPIO_PIN_B7, PIN_CONFIG_MUX_FUNC1);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK3, GPIO_PIN_B2, PIN_CONFIG_MUX_FUNC1);
#endif
    rt_thread_mdelay(1);
    if (config->reset_gpio_bank)
    {
        // HAL_GPIO_SetPinDirection(config->reset_gpio_bank,
        //                          config->reset_gpio_num,
        //                          GPIO_OUT);
        HAL_GPIO_SetPinLevel(config->reset_gpio_bank,
                             config->reset_gpio_num,
                             GPIO_HIGH);
        rt_thread_mdelay(config->reset_delay_ms[0]);
        HAL_GPIO_SetPinLevel(config->reset_gpio_bank,
                             config->reset_gpio_num,
                             GPIO_LOW);
        rt_thread_mdelay(config->reset_delay_ms[1]);

        HAL_GPIO_SetPinLevel(config->reset_gpio_bank,
                             config->reset_gpio_num,
                             GPIO_HIGH);
        rt_thread_mdelay(config->reset_delay_ms[2]);
    }
}

/* phy link status monitor thread */
static void phy_monitor_thread_entry(void *parameter)
{
    struct rockchip_eth *eth = (struct rockchip_eth *)parameter;
    struct GMAC_HANDLE *pGMAC = &eth->instance;
    rt_size_t status;
    status = HAL_GMAC_PHYStartup(pGMAC);
    if (status)
    {
        rk_gmac_dbg(&eth->parent, "HAL_PHY_Startup() failed: %d\n", status);
        return;
    }
    while (1)
    {
        status = HAL_GMAC_PHYUpdateLink(pGMAC);
        if ((status == HAL_OK) && (pGMAC->phyStatus.link != pGMAC->phyStatus.oldLink))
        {
            if(pGMAC->phyStatus.link) rt_kprintf("eth link up!\n");
            else rt_kprintf("eth link down!\n");
            if (pGMAC->phyStatus.link)
            {
                status = HAL_GMAC_PHYParseLink(pGMAC);
                if (PHY_SPEED_1000M ==  pGMAC->phyStatus.speed)
                {
                    rt_kprintf("%s: 1000M\n", eth->parent.parent.parent.name);
                }
                else if (PHY_SPEED_100M ==  pGMAC->phyStatus.speed)
                {
                    rt_kprintf("%s: 100M\n", eth->parent.parent.parent.name);
                }
                else
                {
                    rt_kprintf("%s: 10M\n", eth->parent.parent.parent.name);
                }

                if (PHY_DUPLEX_HALF == pGMAC->phyStatus.duplex)
                {
                    rt_kprintf("%s: half dumplex\n", eth->parent.parent.parent.name);
                }
                else
                {
                    rt_kprintf("%s: full dumplex\n", eth->parent.parent.parent.name);
                }

                if (pGMAC->phyStatus.pause)
                {
                    rt_kprintf("%s: flow control rx/tx\n", eth->parent.parent.parent.name);
                }
                else
                {
                    rt_kprintf("%s: flow control off\n", eth->parent.parent.parent.name);
                }
                //调整GMAC_REG->CONFIGURATION
                status = HAL_GMAC_AdjustLink(pGMAC, eth ->config->tx_delay, eth ->config->rx_delay);
                if (status != HAL_OK)
                {
                    rk_gmac_dbg(&eth->parent, "HAL_GMAC_AdjustLink() failed: %d\n", status);
                }

                //LAST

                // uint32_t value = READ_REG(pGMAC->pReg->MAC_PACKET_FILTER);
                // value |= 0x10404;
                // WRITE_REG(pGMAC->pReg->MAC_PACKET_FILTER, value);
                uint32_t value = READ_REG(pGMAC->pReg->MAC_CONFIGURATION);
                value |= (GMAC_CONFIG_TE | GMAC_CONFIG_RE);
                WRITE_REG(pGMAC->pReg->MAC_CONFIGURATION, value);
                rt_kprintf("%s: link up.\n", eth->parent.parent.parent.name);
                eth_device_linkchange(&eth->parent, RT_TRUE);
            }
            else
            {
                uint32_t value = READ_REG(pGMAC->pReg->MAC_CONFIGURATION);
                value &= ~(GMAC_CONFIG_TE | GMAC_CONFIG_RE);
                WRITE_REG(pGMAC->pReg->MAC_CONFIGURATION, value);
                eth_device_linkchange(&eth->parent, RT_FALSE);
                rt_kprintf("%s: link down.\n", eth->parent.parent.parent.name);
            }
        }
        rt_thread_delay(RT_TICK_PER_SECOND * 10); //降低mdio频率
    }
}

const struct rt_device_ops eth_ops =
{
    .init = rt_rockchip_eth_init,
    .open = rt_rockchip_eth_open,
    .close = rt_rockchip_eth_close,
    .read = rt_rockchip_eth_read,
    .write = rt_rockchip_eth_write,
    .control = rt_rockchip_eth_control,
};


#ifdef RT_USING_GMAC
const struct rockchip_eth_config rockchip_eth_config_table[] =
{
#ifdef RT_USING_GMAC0
    {
        .id = GMAC0,
        .mode = RGMII_MODE,
        .phy_addr = 1,

        .external_clk = false,

        .reset_gpio_bank = GPIO2,
        .reset_gpio_num = GPIO_PIN_D3,
        .reset_delay_ms = {0, 20, 100},

        .tx_delay = 0x43,
        .rx_delay = 0,
    },
#endif

#ifdef RT_USING_GMAC1
    {
        .id = GMAC1,
        .mode = RGMII_MODE,
        .phy_addr = 1,

        .external_clk = false,

        .reset_gpio_bank = GPIO2,
        .reset_gpio_num = GPIO_PIN_D1,
        .reset_delay_ms = {0, 20, 100},

        .tx_delay = 0x43,
        .rx_delay = 0,
    },
#endif
};
#endif



// #define DEFINE_ROCKCHIP_ETH(ID)                             
// static void rockchip_eth##ID##_irq(int irq, void *param);                                                               
// static struct rockchip_eth eth0 =                        
// {                                                           
//     .name = "eth0",                                       
//     .id = 0,                                                       
//     .dev = &g_gmac0Dev,                                 
//     .irq_handler = rockchip_eth_irq,                  
// };  
static struct rockchip_eth eth1 =                        
{                                                           
    .name = "eth1",                                       
    .id = 1,                                                       
    .dev = &g_gmac1Dev,                                 
    .irq_handler = rockchip_eth_irq,                  
};                                                           
                                                            
static void rockchip_eth_irq(int irq, void *param)    
{                                                  
    // rt_rockchip_eth_irq(&eth0);
    rt_rockchip_eth_irq(&eth1);                             
}

// #ifdef RT_USING_GMAC0
// DEFINE_ROCKCHIP_ETH(0)
// #endif

// #ifdef RT_USING_GMAC1
// DEFINE_ROCKCHIP_ETH(1)
// #endif

static struct rockchip_eth *const rockchip_eth_table[] =
{
#ifdef RT_USING_GMAC0
    &eth0,
#endif

#ifdef RT_USING_GMAC1
    &eth1,
#endif

    RT_NULL
};

/* Ethernet Device Interface */
/* transmit packet. */
rt_err_t rt_rockchip_eth_tx(rt_device_t dev, struct pbuf *p)
{
    struct rockchip_eth *eth = (struct rockchip_eth *)dev;
    struct GMAC_HANDLE *pGMAC = &eth->instance;
    rt_uint8_t *ptr = RT_NULL;
    rt_err_t status = RT_EOK;

    if (!pGMAC->phyStatus.link)
        return RT_EOK;

    /* Check the frame length. */
    if (p->tot_len > HAL_GMAC_MAX_FRAME_SIZE)
        return RT_EOK;

    /* lock ETH device */
    rt_sem_take(&eth ->sem_lock, RT_WAITING_FOREVER);
    /* copy data to tx buffer */
    ptr = (rt_uint8_t *)HAL_GMAC_GetTXBuffer(pGMAC);
    pbuf_copy_partial(p, ptr, p->tot_len, 0);
    rt_rockchip_dump_hex(p->payload, p->tot_len);
    rt_hw_cpu_dcache_clean(ptr, p->tot_len);
    status = HAL_GMAC_Send(pGMAC, ptr, p->tot_len);
    if (status)
    {
        rk_gmac_dbg(&eth->parent, "GMAC send failed: %d\n", status);
    }
    else
    {
        rk_gmac_dbg(&eth->parent, "GMAC send successed: %d\n", status);
    } 

    /* unlock ETH device */
    rt_sem_release(&eth ->sem_lock);

    return status;
}

/* reception packet. */
struct pbuf *rt_rockchip_eth_rx(rt_device_t dev)
{
    struct rockchip_eth *eth = (struct rockchip_eth *)dev;
    struct GMAC_HANDLE *pGMAC = &eth->instance;
    struct pbuf *p = RT_NULL;
    rt_uint8_t *ptr = RT_NULL;
    rt_int32_t size;

    if (!pGMAC->phyStatus.link)
        return RT_NULL;

    /* lock ETH device */
    rt_sem_take(&eth ->sem_lock, RT_WAITING_FOREVER);

    ptr = HAL_GMAC_Recv(pGMAC, &size);

    if (size > 0 && ptr)
    {
        rt_hw_cpu_dcache_invalidate(ptr, size);
        /* allocate buffer */
        p = pbuf_alloc(PBUF_LINK, size, PBUF_RAM);
        if (p != RT_NULL)
        {
            pbuf_take(p, ptr, size);
        }
        rt_rockchip_dump_hex(p->payload, p->tot_len);
        HAL_GMAC_CleanRX(pGMAC);
    }
    else
    {
        rk_gmac_dbg(&eth->parent, "GMAC recv failed: %d\n", size);
        /* unlock ETH device */
        rt_sem_release(&eth ->sem_lock);

        return RT_NULL;
    }

    /* unlock ETH device */
    rt_sem_release(&eth ->sem_lock);

    return p;
}

int rt_rockchip_hw_eth_init(void)
{
    struct rockchip_eth *const *eth = rockchip_eth_table;
    const struct rockchip_eth_config *cfg;    
    rt_err_t state = RT_EOK;    

    for (eth = rockchip_eth_table; *eth != RT_NULL; eth++)
    {
        const struct HAL_GMAC_DEV *gmac_dev = (*eth)->dev;   //g_gmac0Dev

        struct GMAC_PHY_Config config;
        eGMAC_PHY_Interface interface;
        char name[32] = {0};
        rt_uint32_t freq;
        rt_size_t status;

        (*eth)->config = RT_NULL;

        for (cfg = rockchip_eth_config_table; cfg->id != 0; cfg++)
        {
            if ((*eth)->dev->pReg == cfg->id)
            {
                (*eth)->config = cfg;
            }
        }
        if (!(*eth)->config)    
            continue;
        /* PHY reset */
        //修改了(*eth)->config,对phy reset的gpio进行配置
        rt_rockchip_phy_reset((*eth)->config);//应该没什么问题
        //mode为RGMII_MODE
        interface = (*eth)->config->mode;

        // if (interface == PHY_INTERFACE_MODE_RGMII)
        // {
        //     clk_set_rate(gmac_dev->clkID, 125000000);
        // }
        // else
        // {
        //     clk_set_rate(gmac_dev->clkID, 50000000);
        // }

        //freq=pclkID
        // freq = clk_get_rate(gmac_dev->pclkID); // need to configurate (*eth)->config clocks!!!
        freq = 125000000; //rk3588和rk3568时钟配置流程不同，这里直接写死
        // 对(*eth)->instance进行配置
        HAL_GMAC_Init(&(*eth)->instance, gmac_dev->pReg, freq, interface,
                      (*eth)->config->external_clk); //external_clk = 0
        config.speed = 1000;
        config.maxSpeed = 1000;
        config.duplexMode = PHY_DUPLEX_FULL;   //全双工 1
        config.neg = PHY_AUTONEG_DISABLE;      //自动协商 0
        config.interface = interface;          //为RGMII_MODE
        config.phyAddress = (*eth)->config->phy_addr; //1
        //初始化PHY
        status = HAL_GMAC_PHYInit(&(*eth)->instance, &config);
        if (status)
        {
            rk_gmac_dbg(&eth->parent, "PHY init failed!\n");
            return status;
        } 
        else rk_gmac_dbg(&eth->parent, "PHY init successed!\n");

        if (!is_valid_ethaddr((*eth)->dev_addr))
        {
            net_random_ethaddr((*eth)->dev_addr);
            rt_kprintf("Dev mac addr :");
            for (int i = 0; i < 6; ++i)
                rt_kprintf(" %x", ((*eth)->dev_addr)[i]);
            rt_kprintf(" \n");
        }
        //将MAC地址写入GMAC寄存器
        HAL_GMAC_WriteHWAddr(&(*eth)->instance, (*eth)->dev_addr);
        (*eth)->parent.parent.ops           = &eth_ops;
        (*eth)->parent.parent.user_data = *eth;
        (*eth)->parent.parent.type          = RT_Device_Class_NetIf;

        (*eth)->parent.eth_rx = rt_rockchip_eth_rx;
        (*eth)->parent.eth_tx = rt_rockchip_eth_tx;

        sprintf(name, "e%d_wait", (*eth)->id);
        /* init tx semaphore */
        rt_sem_init(&(*eth)->sem_lock, name, 1, RT_IPC_FLAG_FIFO);
        rt_memset(name, 0, sizeof(name));
        sprintf(name, "e%d", (*eth)->id);
        
        state = eth_device_init(&((*eth)->parent), name);
        if (RT_EOK == state)
        {
            rt_kprintf("eth_device_init success : %s\n", name);
        }
        else
        {
            rt_kprintf("eth_device_init faild: %d\n", state);
        }
        eth_device_linkchange(&((*eth)->parent), RT_FALSE);
        /* start phy monitor */
        {
            rt_thread_t tid;

            rt_memset(name, 0, sizeof(name));
            sprintf(name, "e%d_phy", (*eth)->id);
            tid = rt_thread_create(name,
                                   phy_monitor_thread_entry,
                                   (*eth),
                                   1024 * 5,
                                   RT_THREAD_PRIORITY_MAX - 2,
                                   2);
            if (tid != RT_NULL)
                rt_thread_startup(tid);
        }
    }
    return state;
}
INIT_DEVICE_EXPORT(rt_rockchip_hw_eth_init);

/** @} */

/** @} */

#if RK_GMAC_DEBUG
#ifdef RT_USING_FINSH
#include <finsh.h>

static int8_t eth_id = 1;
void switch_id(int8_t id)
{
    eth_id = id;
}

struct rockchip_eth *get_rockchip_eth_data(void)
{
    switch (eth_id)
    {
    case 0:
#ifdef RT_USING_GMAC0
        return &eth0;
#else
        return RT_NULL;
#endif
    case 1:
#ifdef RT_USING_GMAC1
        return &eth1;
#else
        return RT_NULL;
#endif
    default:
        return RT_NULL;
    }
    return RT_NULL;
}

void phy_read(uint32_t phyReg)
{
    struct GMAC_HANDLE *pGMAC;
    struct rockchip_eth *eth;
    int data;

    eth = get_rockchip_eth_data();
    if (eth)
        pGMAC = &eth->instance;
    else
    {
        rt_kprintf("ETH IS NULL !!!\n");
        return;
    }   

    data = HAL_GMAC_MDIORead(pGMAC, 0, phyReg);
    if (data >= 0)
    {
        rt_kprintf("PHY_Read: %02X --> %08X\n", phyReg, data);
    }
    else
    {
        rt_kprintf("PHY_Read: %02X --> faild\n", phyReg);
    }
}

void phy_write(uint32_t phyReg, uint32_t data)
{
    struct GMAC_HANDLE *pGMAC;
    struct rockchip_eth *eth;
    int status;

    eth = get_rockchip_eth_data();
    if (eth)
        pGMAC = &eth->instance;
    else
        return;

    status = HAL_GMAC_MDIOWrite(pGMAC, 0, phyReg, data);
    if (!status)
    {
        rt_kprintf("PHY_Write: %02X --> %08X\n", phyReg, data);
    }
    else
    {
        rt_kprintf("PHY_Write: %02X --> faild\n", phyReg);
    }
}

void phy_dump(void)
{
    struct GMAC_HANDLE *pGMAC;
    struct rockchip_eth *eth;
    int i, data1;

    eth = get_rockchip_eth_data();
    if (eth)
        pGMAC = &eth->instance;
    else
        return;

    for (i = 0; i < 64; i++)
    {
        data1 = HAL_GMAC_MDIORead(pGMAC, 1, i);
        if (data1 < 0)
        {
            rt_kprintf("phy_dump: %02X --> faild\n", i);
            break;
        }

        rt_kprintf("%02X --> %08X\n ", i, data1);
    }
}

void dump_net_stat(void)
{
    struct rockchip_eth *eth;
    struct GMAC_HANDLE *pGMAC;
    char *buf, *p;

    eth = get_rockchip_eth_data();
    if (eth)
        pGMAC = &eth->instance;
    else
        return;
    
    rt_kprintf("num_tx_bytes = %ld\n", pGMAC->extraStatus.txPktN);
    rt_kprintf("num_rx_bytes = %ld\n",  pGMAC->extraStatus.rxPktN);
    rt_kprintf("num_tx_pkts = %ld\n", pGMAC->extraStatus.txPktN);
    rt_kprintf("num_rx_pkts = %ld\n",  pGMAC->extraStatus.rxPktN);
    rt_kprintf("num_tx_pkts_err = %ld\n", pGMAC->extraStatus.txErrors);
    rt_kprintf("num_rx_pkts_err = %ld\n", pGMAC->extraStatus.rxErrors);
    rt_kprintf("carrier %s\n", (pGMAC->phyStatus.link) ? "on" : "off");
    rt_kprintf("no useful rx desc %ld\n",pGMAC->extraStatus.rxBufUnavIRQ);
    rt_kprintf("rx process stop %ld\n",pGMAC->extraStatus.rxProcessStoppedIRQ);
    rt_kprintf("rx watchdog outtime %ld\n",pGMAC->extraStatus.rxWatchdogIRQ);
    rt_kprintf("early emit tx irq %ld\n",pGMAC->extraStatus.txEarlyIRQ);
    rt_kprintf("ttx proc stop%ld\n",pGMAC->extraStatus.rxProcessStoppedIRQ);
    rt_kprintf("bus error %ld\n",pGMAC->extraStatus.fatalBusErrorIRQ);
    rt_kprintf("normal irq %ld\n",pGMAC->extraStatus.normalIRQN);
    rt_kprintf("rx normal irq %ld\n",pGMAC->extraStatus.rxNormalIRQN);
    rt_kprintf("tx irq complete %ld\n",pGMAC->extraStatus.txNormallIRQN);
    rt_kprintf("early emit rx irq %ld\n",pGMAC->extraStatus.rxEarlyIRQ);
    rt_kprintf("transmit buffer unvaliable %ld\n",pGMAC->extraStatus.txBufUnavIRQ);
}

void dump_desc_stat(void)
{
    struct rockchip_eth *eth;
    int i = 0;

    eth = get_rockchip_eth_data();
    if (!eth)
        return;
    rt_kprintf("- %p - %p - \n", &(eth->instance.rxDescs), &(eth->instance.txDescs));
    rt_kprintf("- %p - %p - \n", (eth->instance.rxDescs), (eth->instance.txDescs));
    rt_kprintf("- %x - %x - \n", (eth->instance.rxDescs), (eth->instance.txDescs));
    for (i = 0; i < ETH_RXBUFNB; i++)
    {
        rt_kprintf("[%2d] des0: 0x%x, des1: 0x%x, des2: 0x%x, des3: 0x%x\n",
                   i,
                   eth->rx_desc[i].des0,
                   eth->rx_desc[i].des1,
                   eth->rx_desc[i].des2,
                   eth->rx_desc[i].des3);
    }
    rt_kprintf("-------------------------------------------------\n");
    for (i = 0; i < ETH_TXBUFNB; i++)
    {
        rt_kprintf("[%2d] des0: 0x%x, des1: 0x%x, des2: 0x%x, des3: 0x%8x\n",
                   i,
                   eth->tx_desc[i].des0,
                   eth->tx_desc[i].des1,
                   eth->tx_desc[i].des2,
                   eth->tx_desc[i].des3);

    }
}

MSH_CMD_EXPORT(switch_id, switch eth id);
MSH_CMD_EXPORT(phy_read, read phy register);
MSH_CMD_EXPORT(phy_write, write phy register);
MSH_CMD_EXPORT(phy_dump, dump phy registers);
MSH_CMD_EXPORT(dump_net_stat, dump network info);
MSH_CMD_EXPORT(dump_desc_stat, dump description info);

void print_gmac_val(void)
{
    uint32_t test1 = 0, test2;

    // test1 = READ_REG(GMAC0 ->MAC_CONFIGURATION);               
    test2 = READ_REG(GMAC1 ->MAC_CONFIGURATION); 
    rt_kprintf("GMAC CONF TEST 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->MAC_EXT_CONFIGURATION);
    test2 = READ_REG(GMAC1 ->MAC_EXT_CONFIGURATION);
    rt_kprintf("GMAC MAC_EXT_CONFIGURATION 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->MAC_PACKET_FILTER);
    test2 = READ_REG(GMAC1 ->MAC_PACKET_FILTER);
    rt_kprintf("GMAC MAC_PACKET_FILTER 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->MAC_WATCHDOG_TIMEOUT);
    test2 = READ_REG(GMAC1 ->MAC_WATCHDOG_TIMEOUT);
    rt_kprintf("GMAC MAC_WATCHDOG_TIMEOUT 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->MAC_Q0_TX_FLOW_CTRL);
    test2 = READ_REG(GMAC1 ->MAC_Q0_TX_FLOW_CTRL);
    rt_kprintf("GMAC MAC_Q0_TX_FLOW_CTRL 0: %x, 1: %x \n", test1, test2);
     // test1 = READ_REG(GMAC0 ->MAC_RXQ_CTRL0);
    test2 = READ_REG(GMAC1 ->MAC_RXQ_CTRL0);
    rt_kprintf("GMAC MAC_RXQ_CTRL0 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->MAC_RXQ_CTRL1);
    test2 = READ_REG(GMAC1 ->MAC_RXQ_CTRL1);
    rt_kprintf("GMAC MAC_RXQ_CTRL1 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->MAC_RXQ_CTRL2);
    test2 = READ_REG(GMAC1 ->MAC_RXQ_CTRL2);
    rt_kprintf("GMAC MAC_RXQ_CTRL2 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->MAC_INTERRUPT_STATUS);
    test2 = READ_REG(GMAC1 ->MAC_INTERRUPT_STATUS);
    rt_kprintf("GMAC INT STATUS TEST 0: %x, 1: %x \n", test1, test2);
    // // test1 = READ_REG(GMAC0 ->MAC_INTERRUPT_ENABLE);
    test2 = READ_REG(GMAC1 ->MAC_INTERRUPT_ENABLE);
    rt_kprintf("GMAC MAC_INTERRUPT_ENABLE 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->MAC_RX_TX_STATUS);
    test2 = READ_REG(GMAC1 ->MAC_RX_TX_STATUS);
    rt_kprintf("GMAC MAC_RX_TX_STATUS 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->MAC_PHYIF_CONTROL_STATUS);
    test2 = READ_REG(GMAC1 ->MAC_PHYIF_CONTROL_STATUS);
    rt_kprintf("GMAC MAC_PHYIF_CONTROL_STATUS 0: %x, 1: %x \n", test1, test2); 
    // test1 = READ_REG(GMAC0 ->MAC_CSR_SW_CTRL);
    test2 = READ_REG(GMAC1 ->MAC_CSR_SW_CTRL);
    rt_kprintf("GMAC MAC_CSR_SW_CTRL 0: %x, 1: %x \n", test1, test2);
    // // test1 = READ_REG(GMAC0 ->MAC_DEBUG);
    test2 = READ_REG(GMAC1 ->MAC_DEBUG);
    rt_kprintf("GMAC MAC_DEBUG 0: %x, 1: %x \n\n", test1, test2);

    // test1 = READ_REG(GMAC0 ->MMC_CONTROL);
    test2 = READ_REG(GMAC1 ->MMC_CONTROL);
    rt_kprintf("GMAC MMC_CONTROL 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->MMC_RX_INTERRUPT);
    test2 = READ_REG(GMAC1 ->MMC_RX_INTERRUPT);
    rt_kprintf("GMAC MMC_RX_INTERRUPT 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->MMC_TX_INTERRUPT);
    test2 = READ_REG(GMAC1 ->MMC_TX_INTERRUPT);
    rt_kprintf("GMAC MMC_TX_INTERRUPT 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->MMC_TX_INTERRUPT_MASK);
    test2 = READ_REG(GMAC1 ->MMC_TX_INTERRUPT_MASK);
    rt_kprintf("GMAC MMC_TX_INTERRUPT_MASK 0: %x, 1: %x \n", test1, test2);
     // test1 = READ_REG(GMAC0 ->TX_OCTET_COUNT_GOOD_BAD);               
    test2 = READ_REG(GMAC1 ->TX_OCTET_COUNT_GOOD_BAD); 
    rt_kprintf("GMAC TX_OCTET_COUNT_GOOD_BAD 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->TX_PACKET_COUNT_GOOD_BAD);               
    test2 = READ_REG(GMAC1 ->TX_PACKET_COUNT_GOOD_BAD); 
    rt_kprintf("GMAC TX_PACKET_COUNT_GOOD_BAD 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->TX_PACKET_COUNT_GOOD);               
    test2 = READ_REG(GMAC1 ->TX_PACKET_COUNT_GOOD); 
    rt_kprintf("GMAC TX_PACKET_COUNT_GOOD 0: %x, 1: %x \n", test1, test2);

    // test1 = READ_REG(GMAC0 ->MTL_TXQ0_OPERATION_MODE);
    test2 = READ_REG(GMAC1 ->MTL_TXQ0_OPERATION_MODE);
    rt_kprintf("GMAC MTL_TXQ0_OPERATION_MODE 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->MTL_TXQ0_UNDERFLOW);
    test2 = READ_REG(GMAC1 ->MTL_TXQ0_UNDERFLOW);
    rt_kprintf("GMAC MTL_TXQ0_UNDERFLOW 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->MTL_TXQ0_DEBUG);
    test2 = READ_REG(GMAC1 ->MTL_TXQ0_DEBUG);
    rt_kprintf("GMAC MTL_TXQ0_DEBUG 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->MTL_TXQ0_ETS_STATUS);
    test2 = READ_REG(GMAC1 ->MTL_TXQ0_ETS_STATUS);
    rt_kprintf("GMAC MTL_TXQ0_ETS_STATUS 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->MTL_TXQ0_QUANTUM_WEIGHT);
    test2 = READ_REG(GMAC1 ->MTL_TXQ0_QUANTUM_WEIGHT);
    rt_kprintf("GMAC MTL_TXQ0_QUANTUM_WEIGHT 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->MTL_TXQ0_ETS_STATUS);
    test2 = READ_REG(GMAC1 ->MTL_TXQ0_ETS_STATUS);
    rt_kprintf("GMAC MTL_TXQ0_ETS_STATUS 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->MTL_RXQ0_OPERATION_MODE);
    test2 = READ_REG(GMAC1 ->MTL_RXQ0_OPERATION_MODE);
    rt_kprintf("GMAC MTL_RXQ0_OPERATION_MODE 0: %x, 1: %x \n", test1, test2);

    
    // test1 = READ_REG(GMAC0 ->DMA_MODE);
    test2 = READ_REG(GMAC1 ->DMA_MODE);
    rt_kprintf("GMAC DMA_MODE 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->DMA_SYSBUS_MODE);
    test2 = READ_REG(GMAC1 ->DMA_SYSBUS_MODE);
    rt_kprintf("GMAC DMA_SYSBUS_MODE 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->DMA_INTERRUPT_STATUS);
    test2 = READ_REG(GMAC1 ->DMA_INTERRUPT_STATUS);
    rt_kprintf("GMAC DMA_INTERRUPT_STATUS 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->DMA_DEBUG_STATUS0);
    test2 = READ_REG(GMAC1 ->DMA_DEBUG_STATUS0);
    rt_kprintf("GMAC DMA_DEBUG_STATUS0 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->DMA_CH0_CONTROL);
    test2 = READ_REG(GMAC1 ->DMA_CH0_CONTROL);
    rt_kprintf("GMAC DMA_CH0_CONTROL 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->DMA_CH0_TX_CONTROL);
    test2 = READ_REG(GMAC1 ->DMA_CH0_TX_CONTROL);
    rt_kprintf("GMAC DMA_CH0_TX_CONTROL 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->DMA_CH0_RX_CONTROL);
    test2 = READ_REG(GMAC1 ->DMA_CH0_RX_CONTROL);
    rt_kprintf("GMAC DMA_CH0_RX_CONTROL 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->DMA_CH0_TXDESC_LIST_ADDRESS);
    test2 = READ_REG(GMAC1 ->DMA_CH0_TXDESC_LIST_ADDRESS);
    rt_kprintf("GMAC DMA_CH0_TXDESC_LIST_ADDRESS 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->DMA_CH0_RXDESC_LIST_ADDRESS);
    test2 = READ_REG(GMAC1 ->DMA_CH0_RXDESC_LIST_ADDRESS);
    rt_kprintf("GMAC DMA_CH0_RXDESC_LIST_ADDRESS 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->DMA_CH0_TXDESC_TAIL_POINTER);
    test2 = READ_REG(GMAC1 ->DMA_CH0_TXDESC_TAIL_POINTER);
    rt_kprintf("GMAC DMA_CH0_TXDESC_TAIL_POINTER 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->DMA_CH0_RXDESC_TAIL_POINTER);
    test2 = READ_REG(GMAC1 ->DMA_CH0_RXDESC_TAIL_POINTER);
    rt_kprintf("GMAC DMA_CH0_RXDESC_TAIL_POINTER 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->DMA_CH0_TXDESC_RING_LENGTH);
    test2 = READ_REG(GMAC1 ->DMA_CH0_TXDESC_RING_LENGTH);
    rt_kprintf("GMAC DMA_CH0_TXDESC_RING_LENGTH 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->DMA_CH0_RXDESC_RING_LENGTH);
    test2 = READ_REG(GMAC1 ->DMA_CH0_RXDESC_RING_LENGTH);
    rt_kprintf("GMAC DMA_CH0_RXDESC_RING_LENGTH 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->DMA_CH0_INTERRUPT_ENABLE);
    test2 = READ_REG(GMAC1 ->DMA_CH0_INTERRUPT_ENABLE);
    rt_kprintf("GMAC DMA_CH0_INTERRUPT_ENABLE 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->DMA_CH0_CURRENT_APP_TXDESC);
    test2 = READ_REG(GMAC1 ->DMA_CH0_CURRENT_APP_TXDESC);
    rt_kprintf("GMAC DMA_CH0_CURRENT_APP_TXDESC 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->DMA_CH0_CURRENT_APP_RXDESC);
    test2 = READ_REG(GMAC1 ->DMA_CH0_CURRENT_APP_RXDESC);
    rt_kprintf("GMAC DMA_CH0_CURRENT_APP_RXDESC 0: %x, 1: %x \n", test1, test2);
    test1 = READ_REG(GMAC1 ->RESERVED1150);
    test2 = READ_REG(GMAC1 ->DMA_CH0_CURRENT_APP_TXBUFFER);
    rt_kprintf("GMAC DMA_CH0_CURRENT_APP_TXBUFFER 0: %x, 1: %x \n", test1, test2);
    test1 = READ_REG(GMAC1 ->RESERVED1158);
    test2 = READ_REG(GMAC1 ->DMA_CH0_CURRENT_APP_RXBUFFER);
    rt_kprintf("GMAC DMA_CH0_CURRENT_APP_RXBUFFER 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->DMA_CH0_STATUS);
    test2 = READ_REG(GMAC1 ->DMA_CH0_STATUS);
    rt_kprintf("GMAC DMA_CH0_STATU 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->DMA_CH0_MISS_FRAME_CNT);
    test2 = READ_REG(GMAC1 ->DMA_CH0_MISS_FRAME_CNT);
    rt_kprintf("GMAC DMA_CH0_MISS_FRAME_CNT 0: %x, 1: %x \n", test1, test2);
    // test1 = READ_REG(GMAC0 ->DMA_CH0_RX_ERI_CNT);
    test2 = READ_REG(GMAC1 ->DMA_CH0_RX_ERI_CNT);
    rt_kprintf("GMAC DMA_CH0_RX_ERI_CNT 0: %x, 1: %x \n", test1, test2);  
    
    // test1 = READ_REG(CRU->CLKSEL_CON[83]);
    // test2 = READ_REG(CRU->CLKSEL_CON[84]);
    // rt_kprintf("CRU->CLKSEL_CON[83] : %x, CRU->CLKSEL_CON[84] : %x \n", test1, test2);
    // test1 = READ_REG(CRU->GATE_CON[35]);
    // rt_kprintf("CRU->GATE_CON[35] : %x\n", test1);
    // test1 = READ_REG(CRU->GATE_CON[32]);
    // test2 = READ_REG(CRU->GATE_CON[34]);
    // rt_kprintf("GATE_CON[32] : %x, GATE_CON[34] : %x \n", test1, test2);
    // test1 = READ_REG(CRU->SOFTRST_CON[32]);
    // rt_kprintf("SOFTRST_CON[32] : %x\n", test1);
    test1 = READ_REG(PHP_GRF->CLK_CON1);
    test2 = READ_REG(SYS_GRF->SOC_CON7);
    rt_kprintf("GMAC PHP_GRF->CLK_CON1: %x, SYS_GRF->SOC_CON7: %x \n", test1, test2);
    test1 = READ_REG(PHP_GRF->GMAC_CON0);
    rt_kprintf("GMAC PHP_GRF->GMAC_CON0: %x\n", test1);
    test1 = READ_REG(SYS_GRF->SOC_CON8);
    test2 = READ_REG(SYS_GRF->SOC_CON9);
    rt_kprintf("GMAC SYS_GRF->SOC_CON8: %x, SYS_GRF->SOC_CON9: %x \n", test1, test2);
    
}
MSH_CMD_EXPORT(print_gmac_val, dump_gmac...);
// FINSH_FUNCTION_EXPORT
#endif
#endif
#endif
#endif
