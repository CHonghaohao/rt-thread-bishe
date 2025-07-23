/**
  * Copyright (c) 2021 Fuzhou Rockchip Electronics Co., Ltd
  *
  * SPDX-License-Identifier: Apache-2.0
  ******************************************************************************
  * @file    drv_gmac.h
  * @author  David Wu
  * @date    17-Jun-2021
  * @brief   gmac eth driver
  *
  ******************************************************************************
  */

#ifndef _DRV_GMAC_H_
#define _DRV_GMAC_H_

/*******************************************************************************
 * Included Files
 ******************************************************************************/

#include <rtdevice.h>

#ifdef RT_USING_GMAC

#include "hal_base.h"

/*******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/
enum phy_mode
{
    MII_MODE,
    RMII_MODE,
    RGMII_MODE,
    NONE_MODE,  /* Must be last */
};

/*******************************************************************************
 * Public Types
 ******************************************************************************/

struct GMAC_REG;

/* GMAC consumer config data. */
struct rockchip_eth_config
{
    struct GMAC_REG *id;
    enum phy_mode mode;
    uint32_t speed;
    uint32_t max_speed;
    uint16_t phy_addr;

    bool external_clk;

    /* phy reset gpio */
    struct GPIO_REG *reset_gpio_bank;
    ePINCTRL_GPIO_PINS reset_gpio_num;
    uint32_t reset_delay_ms[3];

    int32_t tx_delay;
    int32_t rx_delay;
};

/*******************************************************************************
 * Public Data
 ******************************************************************************/

extern const struct rockchip_eth_config rockchip_eth_config_table[];

/*******************************************************************************
 * Inline Functions
 ******************************************************************************/

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

 /* GMAC config */
#define GMAC_CONFIG_GPSLCE (1 << GMAC_MAC_CONFIGURATION_GPSLCE_SHIFT)
#define GMAC_CONFIG_IPC    (1 << GMAC_MAC_CONFIGURATION_IPC_SHIFT)
#define GMAC_CONFIG_2K     (1 << GMAC_MAC_CONFIGURATION_S2KP_SHIFT)
#define GMAC_CONFIG_CST    (1 << GMAC_MAC_CONFIGURATION_CST_SHIFT)
#define GMAC_CONFIG_ACS    (1 << GMAC_MAC_CONFIGURATION_ACS_SHIFT)
#define GMAC_CONFIG_WD     (1 << GMAC_MAC_CONFIGURATION_WD_SHIFT)
#define GMAC_CONFIG_BE     (1 << GMAC_MAC_CONFIGURATION_BE_SHIFT)
#define GMAC_CONFIG_JD     (1 << GMAC_MAC_CONFIGURATION_JD_SHIFT)
#define GMAC_CONFIG_JE     (1 << GMAC_MAC_CONFIGURATION_JE_SHIFT)
#define GMAC_CONFIG_PS     (1 << GMAC_MAC_CONFIGURATION_PS_SHIFT)
#define GMAC_CONFIG_FES    (1 << GMAC_MAC_CONFIGURATION_FES_SHIFT)
#define GMAC_CONFIG_DM     (1 << GMAC_MAC_CONFIGURATION_DM_SHIFT)
#define GMAC_CONFIG_DCRS   (1 << GMAC_MAC_CONFIGURATION_DCRS_SHIFT)
#define GMAC_CONFIG_TE     (1 << GMAC_MAC_CONFIGURATION_TE_SHIFT)
#define GMAC_CONFIG_RE     (1 << GMAC_MAC_CONFIGURATION_RE_SHIFT)

#define GMAC_CORE_INIT (GMAC_CONFIG_JD | GMAC_CONFIG_PS | GMAC_CONFIG_ACS | \
                        GMAC_CONFIG_BE | GMAC_CONFIG_DCRS)

/* GMAC HW features1 bitmap */
#define GMAC_HW_FEAT_AVSEL       (1 << GMAC_MAC_HW_FEATURE1_AVSEL_SHIFT)
#define GMAC_HW_TSOEN            (1 << GMAC_MAC_HW_FEATURE1_TSOEN_SHIFT)
#define GMAC_HW_TXFIFOSIZE_SHIFT GMAC_MAC_HW_FEATURE1_TXFIFOSIZE_SHIFT
#define GMAC_HW_RXFIFOSIZE_SHIFT GMAC_MAC_HW_FEATURE1_RXFIFOSIZE_SHIFT
#define GMAC_HW_TXFIFOSIZE       (0x1f << GMAC_MAC_HW_FEATURE1_TXFIFOSIZE_SHIFT)
#define GMAC_HW_RXFIFOSIZE       (0x1f << GMAC_MAC_HW_FEATURE1_RXFIFOSIZE_SHIFT)

/* Flow Ctrl */
#define GMAC_Q0_TX_FLOW_CTRL_PT_SHIFT GMAC_MAC_Q0_TX_FLOW_CTRL_PT_SHIFT
#define GMAC_Q0_TX_FLOW_CTRL_PT_MASK  0xffff
#define GMAC_Q0_TX_FLOW_CTRL_TFE      (1 << GMAC_MAC_Q0_TX_FLOW_CTRL_TFE_SHIFT)
#define GMAC_RX_FLOW_CTRL_RFE         (1 << GMAC_MAC_RX_FLOW_CTRL_RFE_SHIFT)

#define HAL_PAUSE_TIME 0xffff

/* MMC register */
#define MMC_CNTRL_COUNTER_RESET (1 << GMAC_MMC_CONTROL_CNTRST_SHIFT)
/* When set, do not roll over zero after reaching the max value */
#define MMC_CNTRL_COUNTER_STOP_ROLLOVER (1 << GMAC_MMC_CONTROL_CNTSTOPRO_SHIFT)
/* Reset after reading */
#define MMC_CNTRL_RESET_ON_READ (1 << GMAC_MMC_CONTROL_RSTONRD_SHIFT)
/* Freeze counter values to the current value */
#define MMC_CNTRL_COUNTER_FREEZER  (1 << GMAC_MMC_CONTROL_CNTFREEZ_SHIFT)
#define MMC_CNTRL_PRESET           (1 << GMAC_MMC_CONTROL_CNTPRST_SHIFT)
#define MMC_CNTRL_FULL_HALF_PRESET (1 << GMAC_MMC_CONTROL_CNTPRSTLVL_SHIFT)

#define MMC_RX_INTR_MASK 0x0c /* MMC Interrupt Mask */
#define MMC_TX_INTR_MASK 0x10 /* MMC Interrupt Mask */
#define MMC_DEFAULT_MASK 0xffffffff

/* Mdc/Mdio register */
#define GMAC_CSR_60_100M  0x0 /* MDC = clk_scr_i/42 */
#define GMAC_CSR_100_150M 0x1 /* MDC = clk_scr_i/62 */
#define GMAC_CSR_20_35M   0x2 /* MDC = clk_scr_i/16 */
#define GMAC_CSR_35_60M   0x3 /* MDC = clk_scr_i/26 */
#define GMAC_CSR_150_250M 0x4 /* MDC = clk_scr_i/102 */
#define GMAC_CSR_250_300M 0x5 /* MDC = clk_scr_i/122 */

#define GMAC_MDIO_ADDRESS_PA_SHIFT  (GMAC_MAC_MDIO_ADDRESS_PA_SHIFT)
#define GMAC_MDIO_ADDRESS_RDA_SHIFT (GMAC_MAC_MDIO_ADDRESS_RDA_SHIFT)
#define GMAC_MDIO_ADDRESS_CR_SHIFT  (GMAC_MAC_MDIO_ADDRESS_CR_SHIFT)
#define GMAC_MDIO_ADDRESS_SKAP      (0x1 << GMAC_MAC_MDIO_ADDRESS_SKAP_SHIFT)
#define GMAC_MDIO_ADDRESS_GOC_SHIFT (GMAC_MAC_MDIO_ADDRESS_GOC_0_SHIFT)
#define GMAC_MDIO_ADDRESS_GOC_READ  (0x3 << GMAC_MAC_MDIO_ADDRESS_GOC_0_SHIFT)
#define GMAC_MDIO_ADDRESS_GOC_WRITE (0x1 << GMAC_MAC_MDIO_ADDRESS_GOC_0_SHIFT)
#define GMAC_MDIO_ADDRESS_C45E      (0x1 << GMAC_MAC_MDIO_ADDRESS_C45E_SHIFT)
#define GMAC_MDIO_ADDRESS_GB        (0x1 << GMAC_MAC_MDIO_ADDRESS_GB_SHIFT)

#define GMAC_MDIO_DATA_GD_MASK 0xffff

#define GMAC_MDIO_TIMEOUT 100 /* ms */

/* MTL */
#define MTL_TXQ0_OPERATION_MODE_TSF (1 << GMAC_MTL_TXQ0_OPERATION_MODE_TSF_SHIFT)
#define MTL_TXQ0_OPERATION_MODE_FTQ (1 << GMAC_MTL_TXQ0_OPERATION_MODE_FTQ_SHIFT)

#define MTL_TXQ0_DEBUG_TXQSTS       (1 << GMAC_MTL_TXQ0_DEBUG_TXQSTS_SHIFT)
#define MTL_TXQ0_DEBUG_TRCSTS_SHIFT GMAC_MTL_TXQ0_DEBUG_TRCSTS_SHIFT
#define MTL_TXQ0_DEBUG_TRCSTS_MASK  GMAC_MTL_TXQ0_DEBUG_TRCSTS_MASK

#define MTL_RXQ0_OPERATION_MODE_EHFC (1 << GMAC_MTL_RXQ0_OPERATION_MODE_EHFC_SHIFT)
#define MTL_RXQ0_OPERATION_MODE_RSF  (1 << GMAC_MTL_RXQ0_OPERATION_MODE_RSF_SHIFT)
#define MTL_RXQ0_OPERATION_MODE_FEP  (1 << GMAC_MTL_RXQ0_OPERATION_MODE_FEP_SHIFT)
#define MTL_RXQ0_OPERATION_MODE_FUP  (1 << GMAC_MTL_RXQ0_OPERATION_MODE_FUP_SHIFT)

#define MTL_OP_MODE_TXQEN_MASK (3 << 2)
#define MTL_OP_MODE_TXQEN_AV   (1 << 2)
#define MTL_OP_MODE_TXQEN      (1 << 3)
#define MTL_OP_MODE_TSF        (1 << 1)

#define MTL_OP_MODE_TQS_MASK  (0x1f<< 16)
#define MTL_OP_MODE_TQS_SHIFT 16

#define MTL_OP_MODE_TTC_MASK  GMAC_MTL_TXQ0_OPERATION_MODE_TTC_MASK
#define MTL_OP_MODE_TTC_SHIFT GMAC_MTL_TXQ0_OPERATION_MODE_TTC_SHIFT

#define MTL_OP_MODE_TTC_32  0
#define MTL_OP_MODE_TTC_64  (1 << MTL_OP_MODE_TTC_SHIFT)
#define MTL_OP_MODE_TTC_96  (2 << MTL_OP_MODE_TTC_SHIFT)
#define MTL_OP_MODE_TTC_128 (3 << MTL_OP_MODE_TTC_SHIFT)
#define MTL_OP_MODE_TTC_192 (4 << MTL_OP_MODE_TTC_SHIFT)
#define MTL_OP_MODE_TTC_256 (5 << MTL_OP_MODE_TTC_SHIFT)
#define MTL_OP_MODE_TTC_384 (6 << MTL_OP_MODE_TTC_SHIFT)
#define MTL_OP_MODE_TTC_512 (7 << MTL_OP_MODE_TTC_SHIFT)

#define MTL_OP_MODE_RQS_SHIFT 20
#define MTL_OP_MODE_RQS_MASK  (0x3ff <<MTL_OP_MODE_RQS_SHIFT)

#define MTL_OP_MODE_RFD_SHIFT 14
#define MTL_OP_MODE_RFD_MASK  (0x3f << MTL_OP_MODE_RFD_SHIFT)

#define MTL_OP_MODE_RFA_SHIFT 8
#define MTL_OP_MODE_RFA_MASK  (0x3f << MTL_OP_MODE_RFA_SHIFT)

#define MTL_OP_MODE_RTC_SHIFT 3
#define MTL_OP_MODE_RTC_MASK  (0x3 << MTL_OP_MODE_RTC_SHIFT)

#define MTL_OP_MODE_RTC_32  (1 << MTL_OP_MODE_RTC_SHIFT)
#define MTL_OP_MODE_RTC_64  0
#define MTL_OP_MODE_RTC_96  (2 << MTL_OP_MODE_RTC_SHIFT)
#define MTL_OP_MODE_RTC_128 (3 << MTL_OP_MODE_RTC_SHIFT)

#define SF_DMA_MODE 1 /* DMA STORE-AND-FORWARD Operation Mode */

/*  GMAC Interrupt bitmap*/
#define GMAC_INT_RGSMIIS   (1 << 0)
#define GMAC_INT_PCS_LINK  (1 << 1)
#define GMAC_INT_PCS_ANE   (1 << 2)
#define GMAC_INT_PCS_PHYIS (1 << 3)
#define GMAC_INT_PMT_EN    (1 << 4)
#define GMAC_INT_LPI_EN    (1 << 5)

/*  MTL interrupt */
#define MTL_RX_OVERFLOW_INT_EN (1 << 24)
#define MTL_RX_OVERFLOW_INT    (1 << 16)

#define GMAC_PCS_IRQ_DEFAULT (GMAC_INT_RGSMIIS | GMAC_INT_PCS_LINK | \
                              GMAC_INT_PCS_ANE)

#define GMAC_INT_DEFAULT_ENABLE (GMAC_INT_PMT_EN | GMAC_INT_LPI_EN)
/* DMA MODE */
#define DMA_MODE_SWR (0x1 << GMAC_DMA_MODE_SWR_SHIFT)

#define DMA_SYSBUS_MODE_EAME   (1 << 11)
#define DMA_SYSBUS_MODE_BLEN16 (1 << GMAC_DMA_SYSBUS_MODE_BLEN16_SHIFT)
#define DMA_SYSBUS_MODE_BLEN8  (1 << GMAC_DMA_SYSBUS_MODE_BLEN8_SHIFT)
#define DMA_SYSBUS_MODE_BLEN4  (1 << GMAC_DMA_SYSBUS_MODE_BLEN4_SHIFT)

#define DMA_CH0_TX_CONTROL_TXPBL_SHIFT GMAC_DMA_CH0_TX_CONTROL_TXPBL_SHIFT
#define DMA_CH0_TX_CONTROL_TXPBL_MASK  0x3f
#define DMA_CH0_TX_CONTROL_OSF         (1 << GMAC_DMA_CH0_TX_CONTROL_OSF_SHIFT)
#define DMA_CH0_TX_CONTROL_ST          (1 << GMAC_DMA_CH0_TX_CONTROL_ST_SHIFT)
#define DMA_CH0_CONTROL_PBLX8          (1 << GMAC_DMA_CH0_CONTROL_PBLX8_SHIFT)

#define DMA_CH0_RX_CONTROL_RXPBL_SHIFT GMAC_DMA_CH0_RX_CONTROL_RXPBL_SHIFT
#define DMA_CH0_RX_CONTROL_RXPBL_MASK  0x3f
#define DMA_CH0_RX_CONTROL_RBSZ_SHIFT  GMAC_DMA_CH0_RX_CONTROL_RBSZ_3_0_SHIFT
#define DMA_CH0_RX_CONTROL_RBSZ_MASK   0x3fff
#define DMA_CH0_RX_CONTROL_SR          (1 << GMAC_DMA_CH0_RX_CONTROL_SR_SHIFT)

#define DMA_CHAN_STATUS_REB       (0x7 << GMAC_DMA_CH0_STATUS_REB_SHIFT)
#define DMA_CHAN_STATUS_REB_SHIFT (GMAC_DMA_CH0_STATUS_REB_SHIFT)
#define DMA_CHAN_STATUS_TEB       (0x7 << GMAC_DMA_CH0_STATUS_TEB_SHIFT)
#define DMA_CHAN_STATUS_TEB_SHIFT (GMAC_DMA_CH0_STATUS_TEB_SHIFT)
#define DMA_CHAN_STATUS_NIS       (1 << GMAC_DMA_CH0_STATUS_NIS_SHIFT)
#define DMA_CHAN_STATUS_AIS       (1 << GMAC_DMA_CH0_STATUS_AIS_SHIFT)
#define DMA_CHAN_STATUS_CDE       (1 << GMAC_DMA_CH0_STATUS_CDE_SHIFT)
#define DMA_CHAN_STATUS_FBE       (1 << GMAC_DMA_CH0_STATUS_FBE_SHIFT)
#define DMA_CHAN_STATUS_ERI       (1 << GMAC_DMA_CH0_STATUS_ERI_SHIFT)
#define DMA_CHAN_STATUS_ETI       (1 << GMAC_DMA_CH0_STATUS_ETI_SHIFT)
#define DMA_CHAN_STATUS_RWT       (1 << GMAC_DMA_CH0_STATUS_RWT_SHIFT)
#define DMA_CHAN_STATUS_RPS       (1 << GMAC_DMA_CH0_STATUS_RPS_SHIFT)
#define DMA_CHAN_STATUS_RBU       (1 << GMAC_DMA_CH0_STATUS_RBU_SHIFT)
#define DMA_CHAN_STATUS_RI        (1 << GMAC_DMA_CH0_STATUS_RI_SHIFT)
#define DMA_CHAN_STATUS_TBU       (1 << GMAC_DMA_CH0_STATUS_TBU_SHIFT)
#define DMA_CHAN_STATUS_TPS       (1 << GMAC_DMA_CH0_STATUS_TPS_SHIFT)
#define DMA_CHAN_STATUS_TI        (1 << GMAC_DMA_CH0_STATUS_TI_SHIFT)

/* Interrupt enable bits per channel */
#define DMA_CHAN_INTR_ENA_NIE  (1 << GMAC_DMA_CH0_INTERRUPT_ENABLE_NIE_SHIFT)
#define DMA_CHAN_INTR_ENA_AIE  (1 << GMAC_DMA_CH0_INTERRUPT_ENABLE_AIE_SHIFT)
#define DMA_CHAN_INTR_ENA_CDE  (1 << GMAC_DMA_CH0_INTERRUPT_ENABLE_CDEE_SHIFT)
#define DMA_CHAN_INTR_ENA_FBE  (1 << GMAC_DMA_CH0_INTERRUPT_ENABLE_FBEE_SHIFT)
#define DMA_CHAN_INTR_ENA_ERE  (1 << GMAC_DMA_CH0_INTERRUPT_ENABLE_ERIE_SHIFT)
#define DMA_CHAN_INTR_ENA_ETE  (1 << GMAC_DMA_CH0_INTERRUPT_ENABLE_ETIE_SHIFT)
#define DMA_CHAN_INTR_ENA_RWE  (1 << GMAC_DMA_CH0_INTERRUPT_ENABLE_RWTE_SHIFT)
#define DMA_CHAN_INTR_ENA_RSE  (1 << GMAC_DMA_CH0_INTERRUPT_ENABLE_RSE_SHIFT)
#define DMA_CHAN_INTR_ENA_RBUE (1 << GMAC_DMA_CH0_INTERRUPT_ENABLE_RBUE_SHIFT)
#define DMA_CHAN_INTR_ENA_RIE  (1 << GMAC_DMA_CH0_INTERRUPT_ENABLE_RIE_SHIFT)
#define DMA_CHAN_INTR_ENA_TBUE (1 << GMAC_DMA_CH0_INTERRUPT_ENABLE_TBUE_SHIFT)
#define DMA_CHAN_INTR_ENA_TSE  (1 << GMAC_DMA_CH0_INTERRUPT_ENABLE_TXSE_SHIFT))
#define DMA_CHAN_INTR_ENA_TIE  (1 << GMAC_DMA_CH0_INTERRUPT_ENABLE_TIE_SHIFT)

#define DMA_CHAN_INTR_NORMAL (DMA_CHAN_INTR_ENA_NIE | \
                              DMA_CHAN_INTR_ENA_RIE | \
                              DMA_CHAN_INTR_ENA_TIE)

#define DMA_CHAN_INTR_ABNORMAL (DMA_CHAN_INTR_ENA_AIE | \
                                DMA_CHAN_INTR_ENA_AIE | \
                                DMA_CHAN_INTR_ENA_CD  | \
                                DMA_CHAN_INTR_ENA_ERE | \
                                DMA_CHAN_INTR_ENA_ETE | \
                                DMA_CHAN_INTR_ENA_FBE)
/* DMA default interrupt mask */
#define DMA_CHAN_INTR_DEFAULT_MASK (DMA_CHAN_INTR_NORMAL | \
                                    DMA_CHAN_INTR_ABNORMAL)

/* Description related */
#define GMAC_DESC3_OWN   (0x1 << 31)
#define GMAC_DESC3_IOC   (0x1 << 30)
#define GMAC_DESC3_FD    (0x1 << 29)
#define GMAC_DESC3_LD    (0x1 << 28)
#define GMAC_DESC3_BUF1V (0x1 << 24)

#define DES3_ERROR_SUMMARY (1 << 15)
#define DES3_ERROR_SUMMARY (1 << 15)

/* Generic MII registers. */
#define MII_BMCR        0x00    /* Basic mode control register */
#define MII_BMSR        0x01    /* Basic mode status register  */
#define MII_PHYSID1     0x02    /* PHYS ID 1                   */
#define MII_PHYSID2     0x03    /* PHYS ID 2                   */
#define MII_ADVERTISE   0x04    /* Advertisement control reg   */
#define MII_LPA         0x05    /* Link partner ability reg    */
#define MII_EXPANSION   0x06    /* Expansion register          */
#define MII_CTRL1000    0x09    /* 1000BASE-T control          */
#define MII_STAT1000    0x0a    /* 1000BASE-T status           */
#define MII_MMD_CTRL    0x0d    /* MMD Access Control Register */
#define MII_MMD_DATA    0x0e    /* MMD Access Data Register    */
#define MII_ESTATUS     0x0f    /* Extended Status             */
#define MII_DCOUNTER    0x12    /* Disconnect counter          */
#define MII_FCSCOUNTER  0x13    /* False carrier counter       */
#define MII_NWAYTEST    0x14    /* N-way auto-neg test reg     */
#define MII_RERRCOUNTER 0x15    /* Receive error counter       */
#define MII_SREVISION   0x16    /* Silicon revision            */
#define MII_RESV1       0x17    /* Reserved...                 */
#define MII_LBRERROR    0x18    /* Lpback, rx, bypass error    */
#define MII_PHYADDR     0x19    /* PHY address                 */
#define MII_RESV2       0x1a    /* Reserved...                 */
#define MII_TPISTATUS   0x1b    /* TPI status for 10mbps       */
#define MII_NCONFIG     0x1c    /* Network interface config    */

/* Basic mode control register. */
#define BMCR_RESV      x003f      /* Unused...                   */
#define BMCR_SPEED1000 0x0040     /* MSB of Speed (1000)         */
#define BMCR_CTST      0x0080     /* Collision test              */
#define BMCR_FULLDPLX  0x0100     /* Full duplex                 */
#define BMCR_ANRESTART 0x0200     /* Auto negotiation restart    */
#define BMCR_ISOLATE   0x0400     /* Isolate data paths from MII */
#define BMCR_PDOWN     0x0800     /* Enable low power state      */
#define BMCR_ANENABLE  0x1000     /* Enable auto negotiation     */
#define BMCR_SPEED100  0x2000     /* Select 100Mbps              */
#define BMCR_LOOPBACK  0x4000     /* TXD loopback bits           */
#define BMCR_RESET     0x8000     /* Reset to default state      */
#define BMCR_SPEED10   0x0000     /* Select 10Mbps               */

/* Basic mode status register. */
#define BMSR_ERCAP        0x0001  /* Ext-reg capability          */
#define BMSR_JCD          0x0002  /* Jabber detected             */
#define BMSR_LSTATUS      0x0004  /* Link status                 */
#define BMSR_ANEGCAPABLE  0x0008  /* Able to do auto-negotiation */
#define BMSR_RFAULT       0x0010  /* Remote fault detected       */
#define BMSR_ANEGCOMPLETE 0x0020  /* Auto-negotiation complete   */
#define BMSR_RESV         0x00c0  /* Unused...                   */
#define BMSR_ESTATEN      0x0100  /* Extended Status in R15      */
#define BMSR_100HALF2     0x0200  /* Can do 100BASE-T2 HDX       */
#define BMSR_100FULL2     0x0400  /* Can do 100BASE-T2 FDX       */
#define BMSR_10HALF       0x0800  /* Can do 10mbps, half-duplex  */
#define BMSR_10FULL       0x1000  /* Can do 10mbps, full-duplex  */
#define BMSR_100HALF      0x2000  /* Can do 100mbps, half-duplex */
#define BMSR_100FULL      0x4000  /* Can do 100mbps, full-duplex */
#define BMSR_100BASE4     0x8000  /* Can do 100mbps, 4k packets  */

/* Advertisement control register. */
#define ADVERTISE_SLCT          0x001f  /* Selector bits               */
#define ADVERTISE_CSMA          0x0001  /* Only selector supported     */
#define ADVERTISE_10HALF        0x0020  /* Try for 10mbps half-duplex  */
#define ADVERTISE_1000XFULL     0x0020  /* Try for 1000BASE-X full-duplex */
#define ADVERTISE_10FULL        0x0040  /* Try for 10mbps full-duplex  */
#define ADVERTISE_1000XHALF     0x0040  /* Try for 1000BASE-X half-duplex */
#define ADVERTISE_100HALF       0x0080  /* Try for 100mbps half-duplex */
#define ADVERTISE_1000XPAUSE    0x0080  /* Try for 1000BASE-X pause    */
#define ADVERTISE_100FULL       0x0100  /* Try for 100mbps full-duplex */
#define ADVERTISE_1000XPSE_ASYM 0x0100  /* Try for 1000BASE-X asym pause */
#define ADVERTISE_100BASE4      0x0200  /* Try for 100mbps 4k packets  */
#define ADVERTISE_PAUSE_CAP     0x0400  /* Try for pause               */
#define ADVERTISE_PAUSE_ASYM    0x0800  /* Try for asymetric pause     */
#define ADVERTISE_RESV          0x1000  /* Unused...                   */
#define ADVERTISE_RFAULT        0x2000  /* Say we can detect faults    */
#define ADVERTISE_LPACK         0x4000  /* Ack link partners response  */
#define ADVERTISE_NPAGE         0x8000  /* Next page bit               */

#define ADVERTISE_FULL (ADVERTISE_100FULL | ADVERTISE_10FULL | \
                        ADVERTISE_CSMA)
#define ADVERTISE_ALL  (ADVERTISE_10HALF | ADVERTISE_10FULL | \
                        ADVERTISE_100HALF | ADVERTISE_100FULL)

/* Link partner ability register. */
#define LPA_SLCT            0x001f    /* Same as advertise selector  */
#define LPA_10HALF          0x0020    /* Can do 10mbps half-duplex   */
#define LPA_1000XFULL       0x0020    /* Can do 1000BASE-X full-duplex */
#define LPA_10FULL          0x0040    /* Can do 10mbps full-duplex   */
#define LPA_1000XHALF       0x0040    /* Can do 1000BASE-X half-duplex */
#define LPA_100HALF         0x0080    /* Can do 100mbps half-duplex  */
#define LPA_1000XPAUSE      0x0080    /* Can do 1000BASE-X pause     */
#define LPA_100FULL         0x0100    /* Can do 100mbps full-duplex  */
#define LPA_1000XPAUSE_ASYM 0x0100    /* Can do 1000BASE-X pause asym*/
#define LPA_100BASE4        0x0200    /* Can do 100mbps 4k packets   */
#define LPA_PAUSE_CAP       0x0400    /* Can pause                   */
#define LPA_PAUSE_ASYM      0x0800    /* Can pause asymetrically     */
#define LPA_RESV            0x1000    /* Unused...                   */
#define LPA_RFAULT          0x2000    /* Link partner faulted        */
#define LPA_LPACK           0x4000    /* Link partner acked us       */
#define LPA_NPAGE           0x8000    /* Next page bit               */

#define LPA_DUPLEX (LPA_10FULL | LPA_100FULL)
#define LPA_100    (LPA_100FULL | LPA_100HALF | LPA_100BASE4)

/* Expansion register for auto-negotiation. */
#define EXPANSION_NWAY        0x0001    /* Can do N-way auto-nego      */
#define EXPANSION_LCWP        0x0002    /* Got new RX page code word   */
#define EXPANSION_ENABLENPAGE 0x0004    /* This enables npage words    */
#define EXPANSION_NPCAPABLE   0x0008    /* Link partner supports npage */
#define EXPANSION_MFAULTS     0x0010    /* Multiple faults detected    */
#define EXPANSION_RESV        0xffe0    /* Unused...                   */

#define ESTATUS_1000_XFULL 0x8000       /* Can do 1000BX Full */
#define ESTATUS_1000_XHALF 0x4000       /* Can do 1000BX Half */
#define ESTATUS_1000_TFULL 0x2000       /* Can do 1000BT Full          */
#define ESTATUS_1000_THALF 0x1000       /* Can do 1000BT Half          */

/* N-way test register. */
#define NWAYTEST_RESV1    0x00ff        /* Unused...                   */
#define NWAYTEST_LOOPBACK 0x0100        /* Enable loopback for N-way   */
#define NWAYTEST_RESV2    0xfe00        /* Unused...                   */

/* 1000BASE-T Control register */
#define ADVERTISE_1000FULL    0x0200  /* Advertise 1000BASE-T full duplex */
#define ADVERTISE_1000HALF    0x0100  /* Advertise 1000BASE-T half duplex */
#define CTL1000_AS_MASTER     0x0800
#define CTL1000_ENABLE_MASTER 0x1000

/* 1000BASE-T Status register */
#define LPA_1000LOCALRXOK 0x2000        /* Link partner local receiver status */
#define LPA_1000REMRXOK   0x1000        /* Link partner remote receiver status */
#define LPA_1000FULL      0x0800        /* Link partner 1000BASE-T full duplex */
#define LPA_1000HALF      0x0400        /* Link partner 1000BASE-T half duplex */

/* Indicates what features are advertised by the interface. */
#define ADVERTISED_10baseT_Half      (1 << 0)
#define ADVERTISED_10baseT_Full      (1 << 1)
#define ADVERTISED_100baseT_Half     (1 << 2)
#define ADVERTISED_100baseT_Full     (1 << 3)
#define ADVERTISED_1000baseT_Half    (1 << 4)
#define ADVERTISED_1000baseT_Full    (1 << 5)
#define ADVERTISED_Autoneg           (1 << 6)
#define ADVERTISED_TP                (1 << 7)
#define ADVERTISED_AUI               (1 << 8)
#define ADVERTISED_MII               (1 << 9)
#define ADVERTISED_FIBRE             (1 << 10)
#define ADVERTISED_BNC               (1 << 11)
#define ADVERTISED_10000baseT_Full   (1 << 12)
#define ADVERTISED_Pause             (1 << 13)
#define ADVERTISED_Asym_Pause        (1 << 14)
#define ADVERTISED_2500baseX_Full    (1 << 15)
#define ADVERTISED_Backplane         (1 << 16)
#define ADVERTISED_1000baseKX_Full   (1 << 17)
#define ADVERTISED_10000baseKX4_Full (1 << 18)
#define ADVERTISED_10000baseKR_Full  (1 << 19)
#define ADVERTISED_10000baseR_FEC    (1 << 20)
#define ADVERTISED_1000baseX_Half    (1 << 21)
#define ADVERTISED_1000baseX_Full    (1 << 22)

/* MII_LPA */
#define PHY_ANLPAR_PSB_802_3 0x0001
#define PHY_ANLPAR_PSB_802_9 0x0002

/* MII_CTRL1000 masks */
#define PHY_1000BTCR_1000FD 0x0200
#define PHY_1000BTCR_1000HD 0x0100

/* MII_STAT1000 masks */
#define PHY_1000BTSR_MSCF   0x8000
#define PHY_1000BTSR_MSCR   0x4000
#define PHY_1000BTSR_LRS    0x2000
#define PHY_1000BTSR_RRS    0x1000
#define PHY_1000BTSR_1000FD 0x0800
#define PHY_1000BTSR_1000HD 0x0400

/* PHY EXSR */
#define ESTATUS_1000XF 0x8000
#define ESTATUS_1000XH 0x4000

#define ETH_FCS_LEN 4 /* Octets in the FCS */
#endif
#endif /* _DRV_GMAC_H_ */
