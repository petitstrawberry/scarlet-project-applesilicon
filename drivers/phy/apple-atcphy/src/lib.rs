#![no_std]
#![allow(dead_code)]

//! Apple Type-C PHY driver.
//!
//! # Provenance
//!
//! PHY registers, mode transitions, and calibration handling were implemented
//! with reference to Asahi Linux's `drivers/phy/apple/atc.c`. See the
//! repository `ATTRIBUTION.md`.

extern crate alloc;

use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec::Vec;
use scarlet::sync::IrqSpinLock;

use scarlet::{
    arch::mmio,
    device::{
        DeviceInfo,
        manager::{DeviceManager, DriverPriority},
        phy::{Phy, PhyError, PhyHandle, PhyMode, PhyOrientation, PhyProvider},
        platform::{
            PlatformDeviceDriver, PlatformDeviceInfo, resource::PlatformDeviceResourceType,
        },
        reset::ResetController,
    },
    early_println, println,
};

// =============================================================================
// =============================================================================

const ATCPHY_POWER_CTRL: usize = 0x20000;
const ATCPHY_POWER_STAT: usize = 0x20004;
const ATCPHY_MISC: usize = 0x20008;

const ATCPHY_POWER_SLEEP_SMALL: u32 = 1 << 0;
const ATCPHY_POWER_SLEEP_BIG: u32 = 1 << 1;
const ATCPHY_POWER_CLAMP_EN: u32 = 1 << 2;
const ATCPHY_POWER_APB_RESET_N: u32 = 1 << 3;
const ATCPHY_POWER_PHY_RESET_N: u32 = 1 << 4;

const ATCPHY_MISC_RESET_N: u32 = 1 << 0;
const ATCPHY_MISC_LANE_SWAP: u32 = 1 << 2;

const ACIOPHY_CFG0: usize = 0x08;
const ACIOPHY_CFG0_COMMON_BIG_OV: u32 = 1 << 1;
const ACIOPHY_CFG0_COMMON_SMALL_OV: u32 = 1 << 3;
const ACIOPHY_CFG0_COMMON_CLAMP_OV: u32 = 1 << 5;
const ACIOPHY_CFG0_RX_SMALL_OV: u32 = 0x3 << 8;
const ACIOPHY_CFG0_RX_BIG_OV: u32 = 0x3 << 12;
const ACIOPHY_CFG0_RX_CLAMP_OV: u32 = 0x3 << 16;

const ACIOPHY_SLEEP_CTRL: usize = 0x1b0;
const ACIOPHY_SLEEP_CTRL_TX_BIG_OV: u32 = 0x3 << 2;
const ACIOPHY_SLEEP_CTRL_TX_SMALL_OV: u32 = 0x3 << 6;
const ACIOPHY_SLEEP_CTRL_TX_CLAMP_OV: u32 = 0x3 << 10;

const ACIOPHY_TOP_BIST_CIOPHY_CFG1: usize = 0x84;
const ACIOPHY_TOP_BIST_CIOPHY_CFG1_CLK_EN: u32 = 1 << 27;
const ACIOPHY_TOP_BIST_CIOPHY_CFG1_BIST_EN: u32 = 1 << 28;
const ACIOPHY_TOP_BIST_OV_CFG: usize = 0x8c;
const ACIOPHY_TOP_BIST_OV_CFG_LN0_RESET_N_OV: u32 = 1 << 13;
const ACIOPHY_TOP_BIST_OV_CFG_LN0_PWR_DOWN_OV: u32 = 1 << 25;
const ACIOPHY_TOP_BIST_READ_CTRL: usize = 0x90;
const ACIOPHY_TOP_BIST_READ_CTRL_LN0_PHY_STATUS_RE: u32 = 1 << 2;
const ACIOPHY_TOP_PHY_STAT: usize = 0x9c;
const ACIOPHY_TOP_PHY_STAT_LN0_READY: u32 = 1 << 0;
const ACIOPHY_TOP_PHY_STAT_LN0_BUSY: u32 = 1 << 23;
const ACIOPHY_TOP_BIST_PHY_CFG0: usize = 0xa8;
const ACIOPHY_TOP_BIST_PHY_CFG0_LN0_RESET_N: u32 = 1 << 0;
const ACIOPHY_TOP_BIST_PHY_CFG1: usize = 0xac;
const ACIOPHY_TOP_BIST_PHY_CFG1_LN0_PWR_DOWN_MASK: u32 = 0xf << 10;
const ACIOPHY_TOP_BIST_PHY_CFG1_LN0_PWR_DOWN_ON: u32 = 3 << 10;

const AUSPLL_FSM_CTRL: usize = 0x1014;
const AUSPLL_APB_CMD_OVERRIDE: usize = 0x2000;
const AUSPLL_APB_CMD_OVERRIDE_REQ: u32 = 1 << 0;
const AUSPLL_APB_CMD_OVERRIDE_ACK: u32 = 1 << 1;
const AUSPLL_APB_CMD_OVERRIDE_CMD: u32 = 0x0fff_fff8;
const AUSPLL_APB_CMD_OVERRIDE_UNK28: u32 = 1 << 28;

const AUSPLL_FREQ_DESC_A: usize = 0x2080;
const AUSPLL_FD_FREQ_COUNT_TARGET: u32 = 0x0000_03ff;
const AUSPLL_FD_FBDIVN_HALF: u32 = 1 << 10;
const AUSPLL_FD_REV_DIVN: u32 = 0x0000_3800;
const AUSPLL_FD_KI_MAN: u32 = 0x0003_c000;
const AUSPLL_FD_KI_EXP: u32 = 0x003c_0000;
const AUSPLL_FD_KP_MAN: u32 = 0x03c0_0000;
const AUSPLL_FD_KP_EXP: u32 = 0x3c00_0000;
const AUSPLL_FD_KPKI_SCALE_HBW: u32 = 0xc000_0000;
const AUSPLL_FREQ_DESC_B: usize = 0x2084;
const AUSPLL_FD_FBDIVN_FRAC_DEN: u32 = 0x0000_3fff;
const AUSPLL_FD_FBDIVN_FRAC_NUM: u32 = 0x0fff_c000;
const AUSPLL_FREQ_DESC_C: usize = 0x2088;
const AUSPLL_FD_SDM_SSC_STEP: u32 = 0x0000_00ff;
const AUSPLL_FD_SDM_SSC_EN: u32 = 1 << 8;
const AUSPLL_FD_PCLK_DIV_SEL: u32 = 0x0000_3e00;
const AUSPLL_FD_LFSDM_DIV: u32 = 0x0000_c000;
const AUSPLL_FD_LFCLK_CTRL: u32 = 0x000f_0000;
const AUSPLL_FD_VCLK_OP_DIVN: u32 = 0x0030_0000;
const AUSPLL_FD_VCLK_PRE_DIVN: u32 = 1 << 22;
const AUSPLL_CLKOUT_MASTER: usize = 0x2200;
const AUSPLL_CLKOUT_MASTER_PCLK_DRVR_EN: u32 = 1 << 2;
const AUSPLL_CLKOUT_MASTER_PCLK2_DRVR_EN: u32 = 1 << 4;
const AUSPLL_CLKOUT_MASTER_REFBUFCLK_DRVR_EN: u32 = 1 << 6;
const AUSPLL_CLKOUT_DIV: usize = 0x2208;
const AUSPLL_CLKOUT_PLLA_REFBUFCLK_DI: u32 = 0x001f_0000;
const AUSPLL_BGR: usize = 0x2214;
const AUSPLL_BGR_CTRL_AVAIL: u32 = 1 << 0;
const AUSPLL_CLKOUT_DTC_VREG: usize = 0x2220;
const AUSPLL_DTC_VREG_BYPASS: u32 = 1 << 7;
const AUSPLL_FREQ_CFG: usize = 0x2224;
const AUSPLL_FREQ_REFCLK: u32 = 0x3;

const AUS_UNK_A20: usize = 0x0a20;
const AUS_UNK_A20_TX_CAL_CODE: u32 = 0x00f0_0000;
const ACIOPHY_CMN_SHM_STS_REG0: usize = 0x0a74;
const ACIOPHY_CMN_SHM_STS_REG0_CMD_READY: u32 = 1 << 0;

const CIO3PLL_CLK_CTRL: usize = 0x2a00;
const CIO3PLL_CLK_PCLK_EN: u32 = 1 << 1;
const CIO3PLL_CLK_REFCLK_EN: u32 = 1 << 5;

const ACIOPHY_LANE_MODE: usize = 0x48;
const ACIOPHY_LANE_MODE_RX0: u32 = 0x7 << 0;
const ACIOPHY_LANE_MODE_TX0: u32 = 0x7 << 3;
const ACIOPHY_LANE_MODE_RX1: u32 = 0x7 << 6;
const ACIOPHY_LANE_MODE_TX1: u32 = 0x7 << 9;
const ACIOPHY_LANE_MODE_MASK: u32 =
    ACIOPHY_LANE_MODE_RX0 | ACIOPHY_LANE_MODE_TX0 | ACIOPHY_LANE_MODE_RX1 | ACIOPHY_LANE_MODE_TX1;
const ACIOPHY_CROSSBAR: usize = 0x4c;
const ACIOPHY_CROSSBAR_PROTOCOL_MASK: u32 = 0x1f;
const ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP: u32 = 0x10;
const ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP_SWAPPED: u32 = 0x11;
const ACIOPHY_CROSSBAR_PROTOCOL_DP: u32 = 0x14;
const ACIOPHY_CROSSBAR_DP_SINGLE_PMA_MASK: u32 = 0x1ffe0;
const ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK008: u32 = 0x008 << 5;
const ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK100: u32 = 0x100 << 5;
const ACIOPHY_CROSSBAR_DP_BOTH_PMA: u32 = 1 << 17;

const ACIOPHY_LANE_MODE_USB3: u32 = 0x1;
const ACIOPHY_LANE_MODE_DP: u32 = 0x2;

const PHY_TYPE_USB2: u32 = 3;
const PHY_TYPE_USB3: u32 = 4;

const ACIOPHY_PLL_COMMON_CTRL: usize = 0x1028;
const ACIOPHY_PLL_WAIT_FOR_CMN_READY_BEFORE_RESET_EXIT: u32 = 1 << 24;
const ACIOPHY_DP_CTRL0: usize = 0x7000;
const DP_PMA_BYTECLK_RESET: u32 = 1 << 0;
const DP_MAC_DIV20_CLK_SEL: u32 = 1 << 1;
const DPTXPHY_PMA_LANE_RESET_N: u32 = 1 << 2;
const DPTXPHY_PMA_LANE_RESET_N_OV: u32 = 1 << 3;
const DPTX_PCLK1_SELECT: u32 = 0x7 << 4;
const DPTX_PCLK2_SELECT: u32 = 0x7 << 7;
const DPRX_PCLK_SELECT: u32 = 0x7 << 10;
const DPTX_PCLK1_ENABLE: u32 = 1 << 13;
const DPTX_PCLK2_ENABLE: u32 = 1 << 14;
const DPRX_PCLK_ENABLE: u32 = 1 << 15;
const ACIOPHY_DP_PCLK_STAT: usize = 0x7044;
const ACIOPHY_AUSPLL_LOCK: u32 = 1 << 3;

const LN0_AUSPMA_RX_TOP: usize = 0x9000;
const LN0_AUSPMA_RX_SHM: usize = 0xb000;
const LN0_AUSPMA_TX_SHM: usize = 0xd000;
const LN1_AUSPMA_RX_TOP: usize = 0x10000;
const LN1_AUSPMA_RX_SHM: usize = 0x12000;
const LN1_AUSPMA_TX_SHM: usize = 0x14000;
const LN_AUSPMA_RX_TOP_PMAFSM: usize = 0x0010;
const LN_AUSPMA_RX_TOP_PMAFSM_PCS_OV: u32 = 1 << 0;
const LN_AUSPMA_RX_TOP_PMAFSM_PCS_REQ: u32 = 1 << 9;
const LN_AUSPMA_RX_TOP_TJ_CFG_RX_TXMODE: usize = 0x00f0;
const LN_RX_TXMODE: u32 = 1 << 0;

const LN_RX_CTLE_CTRL0: usize = 0x00;
const LN_RX_AFE_CTRL1: usize = 0x04;
const LN_RX_DFE_CTRL10: usize = 0x28;
const LN_RX_DFE_CTRL11: usize = 0x2c;
const LN_RX_DFE_CTRL12: usize = 0x30;
const LN_RX_DFE_CTRL13: usize = 0x34;
const LN_RX_SAVOS_CTRL16: usize = 0x48;
const LN_RX_TX_CTRL17: usize = 0x4c;
const LN_RX_TX_CTRL18: usize = 0x50;
const LN_RX_TERM_CTRL19: usize = 0x54;
const LN_RX_VREF_CTRL22: usize = 0x60;

const LN_TX_CLK_EN: u32 = 1 << 20;
const LN_TX_CLK_EN_OV: u32 = 1 << 21;
const LN_RX_DIV20_RESET_N_OV: u32 = 1 << 29;
const LN_RX_DIV20_RESET_N: u32 = 1 << 30;
const LN_DTVREG_ADJUST: u32 = 0xf800_0000;
const LN_DTVREG_BIG_EN: u32 = 1 << 23;
const LN_DTVREG_BIG_EN_OV: u32 = 1 << 24;
const LN_DTVREG_SML_EN: u32 = 1 << 25;
const LN_DTVREG_SML_EN_OV: u32 = 1 << 26;
const LN_TX_BYTECLK_RESET_SYNC_CLR: u32 = 1 << 22;
const LN_TX_BYTECLK_RESET_SYNC_CLR_OV: u32 = 1 << 23;
const LN_TX_BYTECLK_RESET_SYNC_EN: u32 = 1 << 24;
const LN_TX_BYTECLK_RESET_SYNC_EN_OV: u32 = 1 << 25;
const LN_TX_HRCLK_SEL: u32 = 1 << 28;
const LN_TX_HRCLK_SEL_OV: u32 = 1 << 29;
const LN_TX_PBIAS_EN: u32 = 1 << 30;
const LN_TX_PBIAS_EN_OV: u32 = 1 << 31;
const LN_TX_PRE_EN: u32 = 1 << 0;
const LN_TX_PRE_EN_OV: u32 = 1 << 1;
const LN_TX_PST1_EN: u32 = 1 << 2;
const LN_TX_PST1_EN_OV: u32 = 1 << 3;
const LN_DTVREG_ADJUST_OV: u32 = 1 << 15;
const LN_RXTERM_EN: u32 = 1 << 21;
const LN_RXTERM_EN_OV: u32 = 1 << 22;
const LN_RXTERM_PULLUP_LEAK_EN: u32 = 1 << 23;
const LN_RXTERM_PULLUP_LEAK_EN_OV: u32 = 1 << 24;
const LN_TX_CAL_CODE: u32 = 0x3e00_0000;
const LN_TX_CAL_CODE_OV: u32 = 1 << 30;
const LN_TX_MARGIN: u32 = 0x000f_8000;
const LN_TX_MARGIN_OV: u32 = 1 << 20;
const LN_TX_MARGIN_LSB: u32 = 1 << 21;
const LN_TX_MARGIN_LSB_OV: u32 = 1 << 22;
const LN_TX_MARGIN_P1: u32 = 0x0780_0000;
const LN_TX_MARGIN_P1_OV: u32 = 1 << 27;
const LN_TX_MARGIN_P1_LSB: u32 = 0x3000_0000;
const LN_TX_MARGIN_P1_LSB_OV: u32 = 1 << 30;
const LN_TX_P1_CODE: u32 = 0xf;
const LN_TX_P1_CODE_OV: u32 = 1 << 4;
const LN_TX_P1_LSB_CODE: u32 = 0x3 << 5;
const LN_TX_P1_LSB_CODE_OV: u32 = 1 << 7;
const LN_TX_MARGIN_PRE: u32 = 0x7 << 8;
const LN_TX_MARGIN_PRE_OV: u32 = 1 << 11;
const LN_TX_MARGIN_PRE_LSB: u32 = 0x3 << 12;
const LN_TX_MARGIN_PRE_LSB_OV: u32 = 1 << 14;
const LN_TX_PRE_LSB_CODE: u32 = 0x3 << 15;
const LN_TX_PRE_LSB_CODE_OV: u32 = 1 << 17;
const LN_TX_PRE_CODE: u32 = 0xf << 18;
const LN_TX_PRE_CODE_OV: u32 = 1 << 22;
const LN_TX_TEST_EN: u32 = 1 << 21;
const LN_TX_TEST_EN_OV: u32 = 1 << 22;
const LN_TX_EN: u32 = 1 << 23;
const LN_TX_EN_OV: u32 = 1 << 24;
const LN_TX_CLK_DLY_CTRL_TAPGEN: u32 = 0x7 << 25;
const LN_TX_CLK_DIV2_EN: u32 = 1 << 28;
const LN_TX_CLK_DIV2_EN_OV: u32 = 1 << 29;
const LN_TX_CLK_DIV2_RST: u32 = 1 << 30;
const LN_TX_CLK_DIV2_RST_OV: u32 = 1 << 31;
const LN_VREF_ADJUST_GRAY: u32 = 0x1f << 7;
const LN_VREF_ADJUST_GRAY_OV: u32 = 1 << 12;
const LN_VREF_BIAS_SEL: u32 = 0x3 << 13;
const LN_VREF_BIAS_SEL_OV: u32 = 1 << 15;
const LN_VREF_BOOST_EN: u32 = 1 << 16;
const LN_VREF_BOOST_EN_OV: u32 = 1 << 17;
const LN_VREF_EN: u32 = 1 << 18;
const LN_VREF_EN_OV: u32 = 1 << 19;
const LN_VREF_LPBKIN_DATA: u32 = 0x3 << 28;
const LN_VREF_TEST_RXLPBKDT_EN: u32 = 1 << 30;
const LN_VREF_TEST_RXLPBKDT_EN_OV: u32 = 1 << 31;

const LN_TX_CFG_MAIN_REG0: usize = 0x00;
const LN_BYTECLK_RESET_SYNC_EN_OV: u32 = 1 << 2;
const LN_BYTECLK_RESET_SYNC_EN: u32 = 1 << 3;
const LN_BYTECLK_RESET_SYNC_CLR_OV: u32 = 1 << 4;
const LN_BYTECLK_RESET_SYNC_CLR: u32 = 1 << 5;
const LN_BYTECLK_RESET_SYNC_SEL_OV: u32 = 1 << 6;
const LN_TX_CFG_MAIN_REG1: usize = 0x04;
const LN_TXA_DIV2_EN_OV: u32 = 1 << 8;
const LN_TXA_DIV2_EN: u32 = 1 << 9;
const LN_TXA_DIV2_RESET_OV: u32 = 1 << 10;
const LN_TXA_DIV2_RESET: u32 = 1 << 11;
const LN_TXA_CLK_EN_OV: u32 = 1 << 22;
const LN_TXA_CLK_EN: u32 = 1 << 23;
const LN_TX_IMP_REG0: usize = 0x08;
const LN_TXA_CAL_CTRL_OV: u32 = 1 << 0;
const LN_TXA_CAL_CTRL: u32 = 0x0007_fffe;
const LN_TXA_CAL_CTRL_BASE_OV: u32 = 1 << 19;
const LN_TXA_CAL_CTRL_BASE: u32 = 0x00f0_0000;
const LN_TXA_HIZ_OV: u32 = 1 << 29;
const LN_TXA_HIZ: u32 = 1 << 30;
const LN_TX_IMP_REG2: usize = 0x10;
const LN_TXA_MARGIN_OV: u32 = 1 << 0;
const LN_TXA_MARGIN: u32 = 0x0007_fffe;
const LN_TXA_MARGIN_2R_OV: u32 = 1 << 19;
const LN_TXA_MARGIN_2R: u32 = 1 << 20;
const LN_TX_IMP_REG3: usize = 0x14;
const LN_TXA_MARGIN_POST_OV: u32 = 1 << 0;
const LN_TXA_MARGIN_POST: u32 = 0x0000_07fe;
const LN_TXA_MARGIN_POST_2R_OV: u32 = 1 << 11;
const LN_TXA_MARGIN_POST_2R: u32 = 1 << 12;
const LN_TXA_MARGIN_POST_4R_OV: u32 = 1 << 13;
const LN_TXA_MARGIN_POST_4R: u32 = 1 << 14;
const LN_TXA_MARGIN_PRE_OV: u32 = 1 << 15;
const LN_TXA_MARGIN_PRE: u32 = 0x003f_0000;
const LN_TXA_MARGIN_PRE_2R_OV: u32 = 1 << 22;
const LN_TXA_MARGIN_PRE_2R: u32 = 1 << 23;
const LN_TXA_MARGIN_PRE_4R_OV: u32 = 1 << 24;
const LN_TXA_MARGIN_PRE_4R: u32 = 1 << 25;
const LN_TX_LDOCLK: usize = 0x24;
const LN_LDOCLK_BYPASS_SML_OV: u32 = 1 << 8;
const LN_LDOCLK_BYPASS_SML: u32 = 1 << 9;
const LN_LDOCLK_BYPASS_BIG_OV: u32 = 1 << 10;
const LN_LDOCLK_BYPASS_BIG: u32 = 1 << 11;
const LN_LDOCLK_EN_SML_OV: u32 = 1 << 12;
const LN_LDOCLK_EN_SML: u32 = 1 << 13;
const LN_LDOCLK_EN_BIG_OV: u32 = 1 << 14;
const LN_LDOCLK_EN_BIG: u32 = 1 << 15;

const LPDPTX_AUX_CTRL: usize = 0x0000;
const LPDPTX_AUX_CTRL_PWRDN: u32 = 1 << 4;
const LPDPTX_AUX_RXOFFSET: u32 = 0x03c0_0000;
const LPDPTX_AUX_LDO_CTRL: usize = 0x0008;
const LPDPTX_AUX_MARGIN: usize = 0x000c;
const LPDPTX_MARGIN_RCAL_RXOFFSET_EN: u32 = 1 << 5;
const LPDPTX_MARGIN_RCAL_TXSWING: u32 = 0x7c0;
const LPDPTX_AUX_SHM_CTRL0: usize = 0x0204;
const LPDPTX_AUX_SEL_LF_DATA: u32 = 1 << 15;
const LPDPTX_AUX_SHM_CTRL1: usize = 0x0208;
const LPDPTX_PMA_PHYS_ADJ: u32 = 0x7 << 20;
const LPDPTX_PMA_PHYS_ADJ_OV: u32 = 1 << 19;
const LPDPTX_AUX_CONTROL: usize = 0x4000;
const LPDPTX_AUX_PWN_DOWN: u32 = 1 << 4;
const LPDPTX_AUX_CLAMP_EN: u32 = 1 << 2;
const LPDPTX_SLEEP_B_BIG_IN: u32 = 1 << 1;
const LPDPTX_SLEEP_B_SML_IN: u32 = 1 << 0;
const LPDPTX_TXTERM_CODEMSB: u32 = 1 << 10;
const LPDPTX_TXTERM_CODE: u32 = 0x3e0;

// =============================================================================
// =============================================================================

const USB2PHY_USBCTL: usize = 0x00;
const USB2PHY_CTL: usize = 0x04;
const USB2PHY_SIG: usize = 0x08;
const USB2PHY_MISCTUNE: usize = 0x1c;

const USB2PHY_USBCTL_RUN: u32 = 1 << 1;
const USB2PHY_USBCTL_ISOLATION: u32 = 1 << 2;

const USB2PHY_CTL_RESET: u32 = 1 << 0;
const USB2PHY_CTL_PORT_RESET: u32 = 1 << 1;
const USB2PHY_CTL_APB_RESET_N: u32 = 1 << 2;
const USB2PHY_CTL_SIDDQ: u32 = 1 << 3;

const USB2PHY_SIG_VBUSDET_FORCE_VAL: u32 = 1 << 0;
const USB2PHY_SIG_VBUSDET_FORCE_EN: u32 = 1 << 1;
const USB2PHY_SIG_VBUSVLDEXT_FORCE_VAL: u32 = 1 << 2;
const USB2PHY_SIG_VBUSVLDEXT_FORCE_EN: u32 = 1 << 3;
const USB2PHY_SIG_HOST: u32 = 7 << 12;

const USB2PHY_MISCTUNE_APBCLK_GATE_OFF: u32 = 1 << 29;
const USB2PHY_MISCTUNE_REFCLK_GATE_OFF: u32 = 1 << 30;

// =============================================================================
// =============================================================================

const PIPEHANDLER_OVERRIDE: usize = 0x00;
const PIPEHANDLER_OVERRIDE_VALUES: usize = 0x04;
const PIPEHANDLER_MUX_CTRL: usize = 0x0c;
const PIPEHANDLER_LOCK_REQ: usize = 0x10;
const PIPEHANDLER_LOCK_ACK: usize = 0x14;
const PIPEHANDLER_NONSELECTED_OVERRIDE: usize = 0x20;

const PIPEHANDLER_OVERRIDE_RXVALID: u32 = 1 << 0;
const PIPEHANDLER_OVERRIDE_RXDETECT: u32 = 1 << 2;

const PIPEHANDLER_OVERRIDE_VAL_RXDETECT0: u32 = 1 << 1;
const PIPEHANDLER_OVERRIDE_VAL_RXDETECT1: u32 = 1 << 2;

const PIPEHANDLER_MUX_CTRL_DATA_MASK: u32 = 0x7;
const PIPEHANDLER_MUX_CTRL_CLK_MASK: u32 = 0x7 << 3;
const PIPEHANDLER_MUX_CTRL_CLK_OFF: u32 = 0;
const PIPEHANDLER_MUX_CTRL_CLK_USB3: u32 = 1;
const PIPEHANDLER_MUX_CTRL_CLK_DUMMY: u32 = 4;
const PIPEHANDLER_MUX_CTRL_DATA_USB3: u32 = 0;
const PIPEHANDLER_MUX_CTRL_DATA_DUMMY: u32 = 2;

const PIPEHANDLER_LOCK_EN: u32 = 1 << 0;
const PIPEHANDLER_LOCK_ACK_TIMEOUT_US: u64 = 1_000;
const ACIOPHY_STATUS_TIMEOUT_US: u64 = 10_000;

const PIPEHANDLER_AON_GEN: usize = 0x1c;
const PIPEHANDLER_AON_GEN_DWC3_FORCE_CLAMP_EN: u32 = 1 << 4;
const PIPEHANDLER_AON_GEN_DWC3_RESET_N: u32 = 1 << 0;

const PIPEHANDLER_NATIVE_RESET: u32 = 1 << 12;
const PIPEHANDLER_DUMMY_PHY_EN: u32 = 1 << 15;
const PIPEHANDLER_NATIVE_POWER_DOWN_MASK: u32 = 0xf;

const PIPEHANDLER_MUX_CTRL_DATA_DP: u32 = 4;
const PIPEHANDLER_MUX_CTRL_CLK_DP: u32 = 4;

// =============================================================================
// Hardware Tunable
// =============================================================================

/// One hardware tunable entry: `[offset, mask, value]` applied to an MMIO region.
///
/// The bootloader (m1n1) pre-processes EFUSE calibration data into these
/// register-level tunables and injects them into the device tree.
#[derive(Debug, Clone)]
pub struct HardwareTunable {
    /// Register offset from the target MMIO base.
    pub offset: u32,
    /// Bit mask selecting the register fields controlled by this tunable.
    pub mask: u32,
    /// Value to OR into the masked register fields.
    pub value: u32,
}

impl HardwareTunable {
    /// Parse a tunable array from device tree property bytes.
    ///
    /// Property contains big-endian u32 triplets: `[offset, mask, value, ...]`.
    pub fn parse_from_property(prop_bytes: &[u8]) -> Vec<Self> {
        let mut tunables = Vec::new();
        let chunks = prop_bytes.chunks_exact(12);
        for chunk in chunks {
            let offset = u32::from_be_bytes(chunk[0..4].try_into().unwrap_or([0; 4]));
            let mask = u32::from_be_bytes(chunk[4..8].try_into().unwrap_or([0; 4]));
            let value = u32::from_be_bytes(chunk[8..12].try_into().unwrap_or([0; 4]));
            tunables.push(Self {
                offset,
                mask,
                value,
            });
        }
        tunables
    }

    /// Apply this tunable to a 32-bit register read from `base + offset`.
    pub fn apply(&self, base: usize) {
        let old = unsafe { mmio::read32(base + self.offset as usize) };
        let new = (old & !self.mask) | self.value;
        if new != old {
            unsafe { mmio::write32(base + self.offset as usize, new) };
        }
    }
}

/// Apply a slice of tunables to an MMIO base.
///
/// # Arguments
///
/// * `tunables` - Tunable entries to apply in order.
/// * `base` - Virtual MMIO base address for the target register block.
pub fn apply_tunables(tunables: &[HardwareTunable], base: usize) {
    for t in tunables {
        t.apply(base);
    }
}

/// Parse an `apple,tunable-*` property from the device info.
fn parse_tunable_prop(device: &PlatformDeviceInfo, name: &str) -> Vec<HardwareTunable> {
    device
        .property(name)
        .map(|p| HardwareTunable::parse_from_property(p.value()))
        .unwrap_or_default()
}

// =============================================================================
// ATC PHY Mode
// =============================================================================

/// Supported Apple ATC PHY protocol modes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AtcPhyMode {
    /// USB 3.x SuperSpeed mode.
    Usb3,
    /// DisplayPort-only mode.
    DisplayPort,
    /// Combined USB 3.x and DisplayPort mode.
    Usb3Dp,
}

#[derive(Clone, Copy)]
struct DpLinkRateConfig {
    freqinit_count_target: u32,
    fbdivn_frac_den: u32,
    fbdivn_frac_num: u32,
    pclk_div_sel: u32,
    lfclk_ctrl: u32,
    vclk_op_divn: u32,
    plla_clkout_vreg_bypass: bool,
    txa_ldoclk_bypass: bool,
    txa_div2_en: bool,
}

fn field_prep(mask: u32, value: u32) -> u32 {
    (value << mask.trailing_zeros()) & mask
}

// =============================================================================
// ATC PHY Instance
// =============================================================================

/// Apple ATC PHY hardware instance.
///
/// The instance owns mapped MMIO bases and bootloader-provided tunable tables
/// used to initialize USB3 and DisplayPort lanes.
pub struct AppleAtcPhy {
    core_paddr: usize,
    core_base: usize,
    lpdptx_base: Option<usize>,
    axi2af_base: Option<usize>,
    usb2phy_base: usize,
    pipehandler_base: usize,
    common_a: Vec<HardwareTunable>,
    common_b: Vec<HardwareTunable>,
    axi2af_tunables: Vec<HardwareTunable>,
    lane0_usb: Vec<HardwareTunable>,
    lane1_usb: Vec<HardwareTunable>,
    lane0_dp: Vec<HardwareTunable>,
    lane1_dp: Vec<HardwareTunable>,
    pipehandler_up: bool,
    swap_lanes: bool,
    dp_mode: Option<AtcPhyMode>,
    dp_link_rate: Option<u32>,
    active_dp_lanes: u32,
}

impl AppleAtcPhy {
    /// Create a new Apple ATC PHY instance from mapped MMIO regions.
    ///
    /// # Arguments
    ///
    /// * `core_base` - Virtual base for the ATC PHY core register block.
    /// * `lpdptx_base` - Optional virtual base for the LPDP TX register block.
    /// * `axi2af_base` - Optional virtual base for the AXI2AF register block.
    /// * `usb2phy_base` - Virtual base for the USB2 PHY register block.
    /// * `pipehandler_base` - Virtual base for the pipehandler register block.
    ///
    /// # Returns
    ///
    /// An uninitialized PHY instance with empty tunable tables.
    pub fn new(
        core_paddr: usize,
        core_base: usize,
        lpdptx_base: Option<usize>,
        axi2af_base: Option<usize>,
        usb2phy_base: usize,
        pipehandler_base: usize,
    ) -> Self {
        Self {
            core_paddr,
            core_base,
            lpdptx_base,
            axi2af_base,
            usb2phy_base,
            pipehandler_base,
            common_a: Vec::new(),
            common_b: Vec::new(),
            axi2af_tunables: Vec::new(),
            lane0_usb: Vec::new(),
            lane1_usb: Vec::new(),
            lane0_dp: Vec::new(),
            lane1_dp: Vec::new(),
            pipehandler_up: false,
            swap_lanes: false,
            dp_mode: None,
            dp_link_rate: None,
            active_dp_lanes: 0,
        }
    }

    fn core_read32(&self, offset: usize) -> u32 {
        unsafe { mmio::read32(self.core_base + offset) }
    }

    fn core_write32(&self, offset: usize, val: u32) {
        unsafe { mmio::write32(self.core_base + offset, val) }
    }

    fn core_set32(&self, offset: usize, bits: u32) {
        self.core_write32(offset, self.core_read32(offset) | bits);
    }

    fn core_clear32(&self, offset: usize, bits: u32) {
        self.core_write32(offset, self.core_read32(offset) & !bits);
    }

    fn core_mask32(&self, offset: usize, mask: u32, set: u32) {
        let old = self.core_read32(offset);
        self.core_write32(offset, (old & !mask) | set);
    }

    fn usb2phy_read32(&self, offset: usize) -> u32 {
        unsafe { mmio::read32(self.usb2phy_base + offset) }
    }

    fn usb2phy_write32(&self, offset: usize, val: u32) {
        unsafe { mmio::write32(self.usb2phy_base + offset, val) }
    }

    fn usb2phy_set32(&self, offset: usize, bits: u32) {
        self.usb2phy_write32(offset, self.usb2phy_read32(offset) | bits);
    }

    fn usb2phy_clear32(&self, offset: usize, bits: u32) {
        self.usb2phy_write32(offset, self.usb2phy_read32(offset) & !bits);
    }

    fn ph_read32(&self, offset: usize) -> u32 {
        unsafe { mmio::read32(self.pipehandler_base + offset) }
    }

    fn ph_write32(&self, offset: usize, val: u32) {
        unsafe { mmio::write32(self.pipehandler_base + offset, val) }
    }

    fn ph_set32(&self, offset: usize, bits: u32) {
        self.ph_write32(offset, self.ph_read32(offset) | bits);
    }

    fn ph_clear32(&self, offset: usize, bits: u32) {
        self.ph_write32(offset, self.ph_read32(offset) & !bits);
    }

    fn ph_mask32(&self, offset: usize, mask: u32, set: u32) {
        let old = self.ph_read32(offset);
        self.ph_write32(offset, (old & !mask) | set);
    }

    fn lpdptx_read32(&self, offset: usize) -> u32 {
        unsafe { mmio::read32(self.lpdptx_base.unwrap() + offset) }
    }

    fn lpdptx_write32(&self, offset: usize, val: u32) {
        unsafe { mmio::write32(self.lpdptx_base.unwrap() + offset, val) }
    }

    fn lpdptx_set32(&self, offset: usize, bits: u32) {
        self.lpdptx_write32(offset, self.lpdptx_read32(offset) | bits);
    }

    fn lpdptx_clear32(&self, offset: usize, bits: u32) {
        self.lpdptx_write32(offset, self.lpdptx_read32(offset) & !bits);
    }

    fn lpdptx_mask32(&self, offset: usize, mask: u32, set: u32) {
        self.lpdptx_write32(offset, (self.lpdptx_read32(offset) & !mask) | (set & mask));
    }

    fn axi2af_read32(&self, offset: usize) -> u32 {
        unsafe { mmio::read32(self.axi2af_base.unwrap() + offset) }
    }

    fn axi2af_write32(&self, offset: usize, val: u32) {
        unsafe { mmio::write32(self.axi2af_base.unwrap() + offset, val) }
    }

    fn poll_core(
        &self,
        offset: usize,
        mask: u32,
        domain: &'static str,
    ) -> Result<(), &'static str> {
        let mut remaining_us = 100_000;
        while remaining_us != 0 {
            if self.core_read32(offset) & mask == mask {
                return Ok(());
            }
            scarlet::time::udelay(100);
            remaining_us -= 100;
        }
        println!("[apple-atcphy] timeout waiting for {} power domain", domain);
        Err("apple-atcphy: core power domain timeout")
    }

    fn poll_core_bit(&self, offset: usize, bit: u32, set: bool, timeout_us: u64) -> bool {
        let mut remaining = timeout_us;
        while remaining != 0 {
            if (self.core_read32(offset) & bit != 0) == set {
                return true;
            }
            scarlet::time::udelay(10);
            remaining = remaining.saturating_sub(10);
        }
        false
    }

    fn pipehandler_lock(&self) -> Result<(), &'static str> {
        if self.ph_read32(PIPEHANDLER_LOCK_REQ) & PIPEHANDLER_LOCK_EN != 0 {
            println!("[apple-atcphy] warning: pipehandler already locked");
            return Ok(());
        }

        self.ph_set32(PIPEHANDLER_LOCK_REQ, PIPEHANDLER_LOCK_EN);
        let mut remaining = PIPEHANDLER_LOCK_ACK_TIMEOUT_US;
        while remaining != 0 {
            if self.ph_read32(PIPEHANDLER_LOCK_ACK) & PIPEHANDLER_LOCK_EN != 0 {
                return Ok(());
            }
            scarlet::time::udelay(10);
            remaining = remaining.saturating_sub(10);
        }

        self.ph_clear32(PIPEHANDLER_LOCK_REQ, PIPEHANDLER_LOCK_EN);
        println!("[apple-atcphy] warning: pipehandler lock not acknowledged");
        Err("apple-atcphy: pipehandler lock not acknowledged")
    }

    fn pipehandler_unlock(&self) -> Result<(), &'static str> {
        self.ph_clear32(PIPEHANDLER_LOCK_REQ, PIPEHANDLER_LOCK_EN);
        let mut remaining = PIPEHANDLER_LOCK_ACK_TIMEOUT_US;
        while remaining != 0 {
            if self.ph_read32(PIPEHANDLER_LOCK_ACK) & PIPEHANDLER_LOCK_EN == 0 {
                return Ok(());
            }
            scarlet::time::udelay(10);
            remaining = remaining.saturating_sub(10);
        }

        println!("[apple-atcphy] warning: pipehandler unlock not acknowledged");
        Err("apple-atcphy: pipehandler unlock not acknowledged")
    }

    fn pipehandler_check(&self) -> Result<(), &'static str> {
        if self.ph_read32(PIPEHANDLER_LOCK_ACK) & PIPEHANDLER_LOCK_EN == 0 {
            return Ok(());
        }

        println!("[apple-atcphy] warning: pipehandler is locked; releasing it");
        self.pipehandler_unlock()
    }

    fn usb2_power_on(&self) {
        let sig = USB2PHY_SIG_VBUSDET_FORCE_VAL
            | USB2PHY_SIG_VBUSDET_FORCE_EN
            | USB2PHY_SIG_VBUSVLDEXT_FORCE_VAL
            | USB2PHY_SIG_VBUSVLDEXT_FORCE_EN;
        let host = self.usb2phy_read32(USB2PHY_SIG) & USB2PHY_SIG_HOST;
        self.usb2phy_write32(USB2PHY_SIG, sig | host);
        scarlet::time::udelay(10);

        self.usb2phy_clear32(USB2PHY_CTL, USB2PHY_CTL_SIDDQ);
        scarlet::time::udelay(10);

        self.usb2phy_clear32(USB2PHY_CTL, USB2PHY_CTL_RESET);
        scarlet::time::udelay(10);
        self.usb2phy_clear32(USB2PHY_CTL, USB2PHY_CTL_PORT_RESET);
        scarlet::time::udelay(10);
        self.usb2phy_set32(USB2PHY_CTL, USB2PHY_CTL_APB_RESET_N);
        scarlet::time::udelay(10);

        self.usb2phy_clear32(USB2PHY_MISCTUNE, USB2PHY_MISCTUNE_APBCLK_GATE_OFF);
        self.usb2phy_clear32(USB2PHY_MISCTUNE, USB2PHY_MISCTUNE_REFCLK_GATE_OFF);

        self.usb2phy_write32(USB2PHY_USBCTL, USB2PHY_USBCTL_RUN);
    }

    fn usb2_power_off(&self) {
        self.usb2phy_write32(USB2PHY_USBCTL, USB2PHY_USBCTL_ISOLATION);
        scarlet::time::udelay(10);

        self.usb2phy_set32(USB2PHY_CTL, USB2PHY_CTL_SIDDQ);
        scarlet::time::udelay(10);

        self.usb2phy_set32(USB2PHY_CTL, USB2PHY_CTL_PORT_RESET);
        scarlet::time::udelay(10);
        self.usb2phy_set32(USB2PHY_CTL, USB2PHY_CTL_RESET);
        scarlet::time::udelay(10);
        self.usb2phy_clear32(USB2PHY_CTL, USB2PHY_CTL_APB_RESET_N);
        scarlet::time::udelay(10);

        self.usb2phy_set32(USB2PHY_MISCTUNE, USB2PHY_MISCTUNE_APBCLK_GATE_OFF);
        self.usb2phy_set32(USB2PHY_MISCTUNE, USB2PHY_MISCTUNE_REFCLK_GATE_OFF);
    }

    fn usb2_set_mode(&self, mode: PhyMode) -> Result<(), PhyError> {
        match mode {
            PhyMode::UsbHost | PhyMode::UsbOtg => {
                self.usb2phy_set32(USB2PHY_SIG, USB2PHY_SIG_HOST);
                println!("[apple-atcphy] usb2 mode host");
                Ok(())
            }
            PhyMode::UsbDevice => {
                self.usb2phy_clear32(USB2PHY_SIG, USB2PHY_SIG_HOST);
                println!("[apple-atcphy] usb2 mode device");
                Ok(())
            }
            _ => Err(PhyError::InvalidMode),
        }
    }

    fn core_power_on(&self) -> Result<(), &'static str> {
        self.core_set32(ATCPHY_MISC, ATCPHY_MISC_RESET_N);

        self.core_set32(ATCPHY_POWER_CTRL, ATCPHY_POWER_SLEEP_SMALL);
        self.poll_core(ATCPHY_POWER_STAT, ATCPHY_POWER_SLEEP_SMALL, "small")?;

        self.core_set32(ATCPHY_POWER_CTRL, ATCPHY_POWER_SLEEP_BIG);
        self.poll_core(ATCPHY_POWER_STAT, ATCPHY_POWER_SLEEP_BIG, "big")?;

        self.core_clear32(ATCPHY_POWER_CTRL, ATCPHY_POWER_CLAMP_EN);
        self.core_set32(ATCPHY_POWER_CTRL, ATCPHY_POWER_APB_RESET_N);

        Ok(())
    }

    fn core_power_off(&self) -> Result<(), &'static str> {
        // Fairydust tears the low-power DPTX AUX block down before clamping
        // and sleeping the ATC core.  Leaving AUX in the bootloader state and
        // merely cycling the core makes its registers look initialized after
        // enable_dp_aux(), but the DCP firmware cannot complete its first
        // DPCD transaction after ACTIVATE.
        self.disable_dp_aux();

        self.core_clear32(ATCPHY_POWER_CTRL, ATCPHY_POWER_PHY_RESET_N);
        self.core_set32(ATCPHY_POWER_CTRL, ATCPHY_POWER_CLAMP_EN);
        self.core_clear32(ATCPHY_MISC, ATCPHY_MISC_RESET_N | ATCPHY_MISC_LANE_SWAP);
        self.core_clear32(ATCPHY_POWER_CTRL, ATCPHY_POWER_APB_RESET_N);

        self.core_clear32(ATCPHY_POWER_CTRL, ATCPHY_POWER_SLEEP_BIG);
        if !self.poll_core_bit(ATCPHY_POWER_STAT, ATCPHY_POWER_SLEEP_BIG, false, 1_000) {
            return Err("apple-atcphy: failed to sleep big power domain");
        }

        self.core_clear32(ATCPHY_POWER_CTRL, ATCPHY_POWER_SLEEP_SMALL);
        if !self.poll_core_bit(ATCPHY_POWER_STAT, ATCPHY_POWER_SLEEP_SMALL, false, 1_000) {
            return Err("apple-atcphy: failed to sleep small power domain");
        }
        Ok(())
    }

    fn disable_dp_aux(&self) {
        if self.lpdptx_base.is_none() {
            return;
        }

        self.lpdptx_set32(LPDPTX_AUX_CONTROL, LPDPTX_AUX_PWN_DOWN);
        self.lpdptx_set32(LPDPTX_AUX_CTRL, LPDPTX_AUX_CTRL_PWRDN);
        self.lpdptx_set32(LPDPTX_AUX_CONTROL, LPDPTX_AUX_CLAMP_EN);
        self.lpdptx_clear32(LPDPTX_AUX_CONTROL, LPDPTX_SLEEP_B_SML_IN);
        scarlet::time::udelay(10);
        self.lpdptx_clear32(LPDPTX_AUX_CONTROL, LPDPTX_SLEEP_B_BIG_IN);
        scarlet::time::udelay(10);

        self.core_clear32(ACIOPHY_DP_CTRL0, DPTXPHY_PMA_LANE_RESET_N);
        self.core_clear32(ACIOPHY_DP_CTRL0, DPRX_PCLK_ENABLE);
        self.core_clear32(ACIOPHY_DP_CTRL0, DPTX_PCLK1_ENABLE);
        self.core_clear32(ACIOPHY_DP_CTRL0, DPTX_PCLK2_ENABLE);
    }

    fn prepare_bootloader_state(&mut self) -> Result<(), &'static str> {
        self.dwc3_reset_assert_raw();
        self.usb2_power_off();
        self.core_power_off()?;
        self.setup_pipehandler();
        self.pipehandler_up = false;
        early_println!("[apple-atcphy] bootloader state cleared; PIPE set to dummy");
        Ok(())
    }

    fn configure_crossbar(&self) {
        self.configure_lanes(self.dp_mode.unwrap_or(AtcPhyMode::Usb3));
    }

    fn configure_lanes(&self, mode: AtcPhyMode) {
        let (lane0_mode, lane1_mode, protocol, single_pma, both_pma, set_swap, dp_lanes) =
            match (mode, self.swap_lanes) {
                (AtcPhyMode::DisplayPort, false) => (
                    ACIOPHY_LANE_MODE_DP,
                    ACIOPHY_LANE_MODE_DP,
                    ACIOPHY_CROSSBAR_PROTOCOL_DP,
                    ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK100,
                    true,
                    false,
                    [true, true],
                ),
                (AtcPhyMode::DisplayPort, true) => (
                    ACIOPHY_LANE_MODE_DP,
                    ACIOPHY_LANE_MODE_DP,
                    ACIOPHY_CROSSBAR_PROTOCOL_DP,
                    ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK008,
                    false,
                    false,
                    [true, true],
                ),
                (_, true) => (
                    ACIOPHY_LANE_MODE_DP,
                    ACIOPHY_LANE_MODE_USB3,
                    ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP_SWAPPED,
                    ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK008,
                    false,
                    true,
                    [true, false],
                ),
                (_, false) => (
                    ACIOPHY_LANE_MODE_USB3,
                    ACIOPHY_LANE_MODE_DP,
                    ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP,
                    ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK008,
                    false,
                    false,
                    [false, true],
                ),
            };

        let crossbar = self.core_read32(ACIOPHY_CROSSBAR);
        let mut crossbar = (crossbar
            & !(ACIOPHY_CROSSBAR_PROTOCOL_MASK
                | ACIOPHY_CROSSBAR_DP_SINGLE_PMA_MASK
                | ACIOPHY_CROSSBAR_DP_BOTH_PMA))
            | protocol
            | single_pma;
        if both_pma {
            crossbar |= ACIOPHY_CROSSBAR_DP_BOTH_PMA;
        }
        self.core_write32(ACIOPHY_CROSSBAR, crossbar);
        if set_swap {
            self.core_set32(ATCPHY_MISC, ATCPHY_MISC_LANE_SWAP);
        } else {
            self.core_clear32(ATCPHY_MISC, ATCPHY_MISC_LANE_SWAP);
        }

        let lane_mode_before = self.core_read32(ACIOPHY_LANE_MODE);
        let lane_mode =
            (lane0_mode << 0) | (lane0_mode << 3) | (lane1_mode << 6) | (lane1_mode << 9);
        // ACIOPHY_LANE_MODE contains fields outside the four protocol selectors.
        // Preserve them exactly as the upstream Apple ATC PHY driver does; a
        // whole-register write here drops calibration/state installed by the
        // preceding tunables before DCP attempts its first AUX transaction.
        self.core_mask32(ACIOPHY_LANE_MODE, ACIOPHY_LANE_MODE_MASK, lane_mode);
        let lane_mode_after = self.core_read32(ACIOPHY_LANE_MODE);
        println!(
            "[apple-atcphy] lane mode {:?} reversed={} reg={:#x}->{:#x} crossbar={:#x}",
            mode,
            self.swap_lanes,
            lane_mode_before,
            lane_mode_after,
            self.core_read32(ACIOPHY_CROSSBAR)
        );

        for (base, is_dp) in [LN0_AUSPMA_RX_TOP, LN1_AUSPMA_RX_TOP]
            .into_iter()
            .zip(dp_lanes)
        {
            if is_dp {
                self.core_set32(
                    base + LN_AUSPMA_RX_TOP_PMAFSM,
                    LN_AUSPMA_RX_TOP_PMAFSM_PCS_OV,
                );
                scarlet::time::udelay(10);
                self.core_clear32(
                    base + LN_AUSPMA_RX_TOP_PMAFSM,
                    LN_AUSPMA_RX_TOP_PMAFSM_PCS_REQ,
                );
            } else {
                self.core_clear32(
                    base + LN_AUSPMA_RX_TOP_PMAFSM,
                    LN_AUSPMA_RX_TOP_PMAFSM_PCS_OV,
                );
                scarlet::time::udelay(10);
            }
        }
    }

    fn enable_dp_aux(&mut self) {
        self.core_set32(
            ACIOPHY_DP_CTRL0,
            DPTXPHY_PMA_LANE_RESET_N | DPTXPHY_PMA_LANE_RESET_N_OV,
        );
        self.core_mask32(
            ACIOPHY_DP_CTRL0,
            DPRX_PCLK_SELECT,
            field_prep(DPRX_PCLK_SELECT, 1),
        );
        self.core_set32(ACIOPHY_DP_CTRL0, DPRX_PCLK_ENABLE);
        self.core_mask32(
            ACIOPHY_DP_CTRL0,
            DPTX_PCLK1_SELECT,
            field_prep(DPTX_PCLK1_SELECT, 1),
        );
        self.core_set32(ACIOPHY_DP_CTRL0, DPTX_PCLK1_ENABLE);
        self.core_mask32(
            ACIOPHY_DP_CTRL0,
            DPTX_PCLK2_SELECT,
            field_prep(DPTX_PCLK2_SELECT, 1),
        );
        self.core_set32(ACIOPHY_DP_CTRL0, DPTX_PCLK2_ENABLE);
        self.core_set32(
            ACIOPHY_PLL_COMMON_CTRL,
            ACIOPHY_PLL_WAIT_FOR_CMN_READY_BEFORE_RESET_EXIT,
        );

        self.lpdptx_set32(LPDPTX_AUX_CONTROL, LPDPTX_AUX_CLAMP_EN);
        self.lpdptx_set32(LPDPTX_AUX_CONTROL, LPDPTX_SLEEP_B_SML_IN);
        scarlet::time::udelay(10);
        self.lpdptx_set32(LPDPTX_AUX_CONTROL, LPDPTX_SLEEP_B_BIG_IN);
        scarlet::time::udelay(10);
        self.lpdptx_clear32(LPDPTX_AUX_CONTROL, LPDPTX_AUX_CLAMP_EN);
        self.lpdptx_clear32(
            LPDPTX_AUX_CONTROL,
            LPDPTX_AUX_PWN_DOWN | LPDPTX_TXTERM_CODEMSB,
        );
        self.lpdptx_mask32(
            LPDPTX_AUX_CONTROL,
            LPDPTX_TXTERM_CODE,
            field_prep(LPDPTX_TXTERM_CODE, 0x16),
        );
        self.lpdptx_set32(LPDPTX_AUX_LDO_CTRL, 0x1c00);
        self.lpdptx_mask32(
            LPDPTX_AUX_SHM_CTRL1,
            LPDPTX_PMA_PHYS_ADJ,
            field_prep(LPDPTX_PMA_PHYS_ADJ, 5),
        );
        self.lpdptx_set32(LPDPTX_AUX_SHM_CTRL1, LPDPTX_PMA_PHYS_ADJ_OV);
        self.lpdptx_clear32(LPDPTX_AUX_MARGIN, LPDPTX_MARGIN_RCAL_RXOFFSET_EN);
        self.lpdptx_clear32(LPDPTX_AUX_CTRL, LPDPTX_AUX_CTRL_PWRDN);
        self.lpdptx_set32(LPDPTX_AUX_SHM_CTRL0, LPDPTX_AUX_SEL_LF_DATA);
        self.lpdptx_mask32(
            LPDPTX_AUX_CTRL,
            LPDPTX_AUX_RXOFFSET,
            field_prep(LPDPTX_AUX_RXOFFSET, 3),
        );
        self.lpdptx_mask32(
            LPDPTX_AUX_MARGIN,
            LPDPTX_MARGIN_RCAL_TXSWING,
            field_prep(LPDPTX_MARGIN_RCAL_TXSWING, 12),
        );
        self.dp_link_rate = None;
    }

    fn configure_dp_lane(&self, lane: usize, cfg: &DpLinkRateConfig) -> Result<(), &'static str> {
        let (tx_shm, rx_shm, rx_top) = match lane {
            0 => (LN0_AUSPMA_TX_SHM, LN0_AUSPMA_RX_SHM, LN0_AUSPMA_RX_TOP),
            1 => (LN1_AUSPMA_TX_SHM, LN1_AUSPMA_RX_SHM, LN1_AUSPMA_RX_TOP),
            _ => return Err("apple-atcphy: invalid DP lane"),
        };

        self.core_set32(tx_shm + LN_TX_LDOCLK, LN_LDOCLK_EN_SML);
        self.core_set32(tx_shm + LN_TX_LDOCLK, LN_LDOCLK_EN_SML_OV);
        scarlet::time::udelay(10);
        self.core_set32(tx_shm + LN_TX_LDOCLK, LN_LDOCLK_EN_BIG);
        self.core_set32(tx_shm + LN_TX_LDOCLK, LN_LDOCLK_EN_BIG_OV);
        scarlet::time::udelay(10);
        let bypass_small = LN_LDOCLK_BYPASS_SML | LN_LDOCLK_BYPASS_SML_OV;
        let bypass_big = LN_LDOCLK_BYPASS_BIG | LN_LDOCLK_BYPASS_BIG_OV;
        if cfg.txa_ldoclk_bypass {
            self.core_set32(tx_shm + LN_TX_LDOCLK, bypass_small);
        } else {
            self.core_clear32(tx_shm + LN_TX_LDOCLK, bypass_small);
        }
        scarlet::time::udelay(10);
        if cfg.txa_ldoclk_bypass {
            self.core_set32(tx_shm + LN_TX_LDOCLK, bypass_big);
        } else {
            self.core_clear32(tx_shm + LN_TX_LDOCLK, bypass_big);
        }
        scarlet::time::udelay(10);

        self.core_set32(
            tx_shm + LN_TX_CFG_MAIN_REG0,
            LN_BYTECLK_RESET_SYNC_SEL_OV
                | LN_BYTECLK_RESET_SYNC_EN
                | LN_BYTECLK_RESET_SYNC_EN_OV
                | LN_BYTECLK_RESET_SYNC_CLR_OV,
        );
        self.core_clear32(tx_shm + LN_TX_CFG_MAIN_REG0, LN_BYTECLK_RESET_SYNC_CLR);
        if cfg.txa_div2_en {
            self.core_set32(tx_shm + LN_TX_CFG_MAIN_REG1, LN_TXA_DIV2_EN);
        } else {
            self.core_clear32(tx_shm + LN_TX_CFG_MAIN_REG1, LN_TXA_DIV2_EN);
        }
        self.core_set32(
            tx_shm + LN_TX_CFG_MAIN_REG1,
            LN_TXA_DIV2_EN_OV | LN_TXA_CLK_EN | LN_TXA_CLK_EN_OV | LN_TXA_DIV2_RESET_OV,
        );
        self.core_clear32(tx_shm + LN_TX_CFG_MAIN_REG1, LN_TXA_DIV2_RESET);

        self.core_mask32(
            tx_shm + LN_TX_IMP_REG0,
            LN_TXA_CAL_CTRL_BASE,
            field_prep(LN_TXA_CAL_CTRL_BASE, 0xf),
        );
        self.core_set32(tx_shm + LN_TX_IMP_REG0, LN_TXA_CAL_CTRL_BASE_OV);
        let tx_cal_code = (self.core_read32(AUS_UNK_A20) & AUS_UNK_A20_TX_CAL_CODE)
            >> AUS_UNK_A20_TX_CAL_CODE.trailing_zeros();
        self.core_mask32(
            tx_shm + LN_TX_IMP_REG0,
            LN_TXA_CAL_CTRL,
            field_prep(LN_TXA_CAL_CTRL, (1u32 << tx_cal_code).saturating_sub(1)),
        );
        self.core_set32(tx_shm + LN_TX_IMP_REG0, LN_TXA_CAL_CTRL_OV);

        self.core_clear32(tx_shm + LN_TX_IMP_REG2, LN_TXA_MARGIN | LN_TXA_MARGIN_2R);
        self.core_set32(
            tx_shm + LN_TX_IMP_REG2,
            LN_TXA_MARGIN_OV | LN_TXA_MARGIN_2R_OV,
        );
        self.core_clear32(
            tx_shm + LN_TX_IMP_REG3,
            LN_TXA_MARGIN_POST
                | LN_TXA_MARGIN_POST_2R
                | LN_TXA_MARGIN_POST_4R
                | LN_TXA_MARGIN_PRE
                | LN_TXA_MARGIN_PRE_2R
                | LN_TXA_MARGIN_PRE_4R,
        );
        self.core_set32(
            tx_shm + LN_TX_IMP_REG3,
            LN_TXA_MARGIN_POST_OV
                | LN_TXA_MARGIN_POST_2R_OV
                | LN_TXA_MARGIN_POST_4R_OV
                | LN_TXA_MARGIN_PRE_OV
                | LN_TXA_MARGIN_PRE_2R_OV
                | LN_TXA_MARGIN_PRE_4R_OV,
        );
        self.core_clear32(tx_shm + LN_TX_IMP_REG0, LN_TXA_HIZ);
        self.core_set32(tx_shm + LN_TX_IMP_REG0, LN_TXA_HIZ_OV);

        self.core_clear32(rx_shm + LN_RX_AFE_CTRL1, LN_RX_DIV20_RESET_N);
        self.core_set32(rx_shm + LN_RX_AFE_CTRL1, LN_RX_DIV20_RESET_N_OV);
        scarlet::time::udelay(10);
        self.core_set32(rx_shm + LN_RX_AFE_CTRL1, LN_RX_DIV20_RESET_N);
        self.core_set32(
            rx_shm + LN_RX_DFE_CTRL12,
            LN_TX_BYTECLK_RESET_SYNC_EN | LN_TX_BYTECLK_RESET_SYNC_EN_OV,
        );
        self.core_mask32(
            rx_shm + LN_RX_SAVOS_CTRL16,
            LN_TX_CAL_CODE,
            field_prep(LN_TX_CAL_CODE, tx_cal_code),
        );
        self.core_set32(rx_shm + LN_RX_SAVOS_CTRL16, LN_TX_CAL_CODE_OV);
        self.core_mask32(
            rx_shm + LN_RX_TERM_CTRL19,
            LN_TX_CLK_DLY_CTRL_TAPGEN,
            field_prep(LN_TX_CLK_DLY_CTRL_TAPGEN, 3),
        );
        self.core_clear32(rx_shm + LN_RX_DFE_CTRL10, LN_DTVREG_ADJUST);
        self.core_set32(rx_shm + LN_RX_DFE_CTRL13, LN_DTVREG_ADJUST_OV);
        self.core_clear32(rx_shm + LN_RX_SAVOS_CTRL16, LN_RXTERM_EN);
        self.core_set32(rx_shm + LN_RX_SAVOS_CTRL16, LN_RXTERM_EN_OV);
        self.core_clear32(rx_shm + LN_RX_TERM_CTRL19, LN_TX_TEST_EN);
        self.core_set32(rx_shm + LN_RX_TERM_CTRL19, LN_TX_TEST_EN_OV);

        self.core_set32(
            rx_shm + LN_RX_VREF_CTRL22,
            LN_VREF_TEST_RXLPBKDT_EN | LN_VREF_TEST_RXLPBKDT_EN_OV,
        );
        self.core_mask32(
            rx_shm + LN_RX_VREF_CTRL22,
            LN_VREF_LPBKIN_DATA,
            field_prep(LN_VREF_LPBKIN_DATA, 3),
        );
        self.core_mask32(
            rx_shm + LN_RX_VREF_CTRL22,
            LN_VREF_BIAS_SEL,
            field_prep(LN_VREF_BIAS_SEL, 2),
        );
        self.core_set32(rx_shm + LN_RX_VREF_CTRL22, LN_VREF_BIAS_SEL_OV);
        self.core_mask32(
            rx_shm + LN_RX_VREF_CTRL22,
            LN_VREF_ADJUST_GRAY,
            field_prep(LN_VREF_ADJUST_GRAY, 0x18),
        );
        self.core_set32(
            rx_shm + LN_RX_VREF_CTRL22,
            LN_VREF_ADJUST_GRAY_OV
                | LN_VREF_EN
                | LN_VREF_EN_OV
                | LN_VREF_BOOST_EN
                | LN_VREF_BOOST_EN_OV,
        );
        scarlet::time::udelay(10);
        self.core_clear32(rx_shm + LN_RX_VREF_CTRL22, LN_VREF_BOOST_EN);
        self.core_set32(rx_shm + LN_RX_VREF_CTRL22, LN_VREF_BOOST_EN_OV);
        scarlet::time::udelay(10);

        self.core_clear32(rx_shm + LN_RX_DFE_CTRL13, LN_TX_PRE_EN | LN_TX_PST1_EN);
        self.core_set32(
            rx_shm + LN_RX_DFE_CTRL13,
            LN_TX_PRE_EN_OV | LN_TX_PST1_EN_OV,
        );
        self.core_clear32(rx_shm + LN_RX_DFE_CTRL12, LN_TX_PBIAS_EN);
        self.core_set32(rx_shm + LN_RX_DFE_CTRL12, LN_TX_PBIAS_EN_OV);
        self.core_clear32(rx_shm + LN_RX_SAVOS_CTRL16, LN_RXTERM_PULLUP_LEAK_EN);
        self.core_set32(rx_shm + LN_RX_SAVOS_CTRL16, LN_RXTERM_PULLUP_LEAK_EN_OV);
        self.core_set32(rx_top + LN_AUSPMA_RX_TOP_TJ_CFG_RX_TXMODE, LN_RX_TXMODE);

        if cfg.txa_div2_en {
            self.core_set32(rx_shm + LN_RX_TERM_CTRL19, LN_TX_CLK_DIV2_EN);
        } else {
            self.core_clear32(rx_shm + LN_RX_TERM_CTRL19, LN_TX_CLK_DIV2_EN);
        }
        self.core_set32(rx_shm + LN_RX_TERM_CTRL19, LN_TX_CLK_DIV2_EN_OV);
        self.core_clear32(rx_shm + LN_RX_TERM_CTRL19, LN_TX_CLK_DIV2_RST);
        self.core_set32(rx_shm + LN_RX_TERM_CTRL19, LN_TX_CLK_DIV2_RST_OV);
        self.core_clear32(rx_shm + LN_RX_DFE_CTRL12, LN_TX_HRCLK_SEL);
        self.core_set32(rx_shm + LN_RX_DFE_CTRL12, LN_TX_HRCLK_SEL_OV);

        self.core_clear32(
            rx_shm + LN_RX_TX_CTRL17,
            LN_TX_MARGIN | LN_TX_MARGIN_LSB | LN_TX_MARGIN_P1 | LN_TX_MARGIN_P1_LSB,
        );
        self.core_set32(
            rx_shm + LN_RX_TX_CTRL17,
            LN_TX_MARGIN_OV | LN_TX_MARGIN_LSB_OV | LN_TX_MARGIN_P1_OV | LN_TX_MARGIN_P1_LSB_OV,
        );
        self.core_clear32(
            rx_shm + LN_RX_TX_CTRL18,
            LN_TX_P1_CODE
                | LN_TX_P1_LSB_CODE
                | LN_TX_MARGIN_PRE
                | LN_TX_MARGIN_PRE_LSB
                | LN_TX_PRE_LSB_CODE
                | LN_TX_PRE_CODE,
        );
        self.core_set32(
            rx_shm + LN_RX_TX_CTRL18,
            LN_TX_P1_CODE_OV
                | LN_TX_P1_LSB_CODE_OV
                | LN_TX_MARGIN_PRE_OV
                | LN_TX_MARGIN_PRE_LSB_OV
                | LN_TX_PRE_LSB_CODE_OV
                | LN_TX_PRE_CODE_OV,
        );
        self.core_set32(
            rx_shm + LN_RX_DFE_CTRL11,
            LN_DTVREG_SML_EN | LN_DTVREG_SML_EN_OV,
        );
        scarlet::time::udelay(10);
        self.core_set32(
            rx_shm + LN_RX_DFE_CTRL11,
            LN_DTVREG_BIG_EN | LN_DTVREG_BIG_EN_OV,
        );
        scarlet::time::udelay(10);
        self.core_mask32(
            rx_shm + LN_RX_DFE_CTRL10,
            LN_DTVREG_ADJUST,
            field_prep(LN_DTVREG_ADJUST, 0xa),
        );
        self.core_set32(rx_shm + LN_RX_DFE_CTRL13, LN_DTVREG_ADJUST_OV);
        scarlet::time::udelay(10);
        self.core_set32(rx_shm + LN_RX_TERM_CTRL19, LN_TX_EN | LN_TX_EN_OV);
        scarlet::time::udelay(10);
        self.core_set32(rx_shm + LN_RX_CTLE_CTRL0, LN_TX_CLK_EN | LN_TX_CLK_EN_OV);
        self.core_clear32(rx_shm + LN_RX_DFE_CTRL12, LN_TX_BYTECLK_RESET_SYNC_CLR);
        self.core_set32(rx_shm + LN_RX_DFE_CTRL12, LN_TX_BYTECLK_RESET_SYNC_CLR_OV);
        Ok(())
    }

    fn auspll_apb_command(&self, command: u32) -> Result<(), &'static str> {
        self.core_mask32(
            AUSPLL_APB_CMD_OVERRIDE,
            AUSPLL_APB_CMD_OVERRIDE_CMD,
            field_prep(AUSPLL_APB_CMD_OVERRIDE_CMD, command),
        );
        self.core_set32(
            AUSPLL_APB_CMD_OVERRIDE,
            AUSPLL_APB_CMD_OVERRIDE_REQ | AUSPLL_APB_CMD_OVERRIDE_UNK28,
        );
        if !self.poll_core_bit(
            AUSPLL_APB_CMD_OVERRIDE,
            AUSPLL_APB_CMD_OVERRIDE_ACK,
            true,
            10_000,
        ) {
            self.core_clear32(AUSPLL_APB_CMD_OVERRIDE, AUSPLL_APB_CMD_OVERRIDE_REQ);
            return Err("apple-atcphy: AUSPLL APB command not acknowledged");
        }
        self.core_clear32(AUSPLL_APB_CMD_OVERRIDE, AUSPLL_APB_CMD_OVERRIDE_REQ);
        Ok(())
    }

    /// Program the DisplayPort PLL and lane clocks for a negotiated link rate
    /// in megabits per second per lane.
    pub fn configure_dp_link_rate(&mut self, link_rate: u32) -> Result<(), &'static str> {
        if link_rate == 0 {
            self.dp_link_rate = None;
            return Ok(());
        }
        if self.dp_mode.is_none() {
            return Err("apple-atcphy: DP mode is not initialized");
        }
        if self.dp_link_rate == Some(link_rate) {
            return Ok(());
        }

        let cfg = match link_rate {
            1620 => DpLinkRateConfig {
                freqinit_count_target: 0x21c,
                fbdivn_frac_den: 0,
                fbdivn_frac_num: 0,
                pclk_div_sel: 0x13,
                lfclk_ctrl: 5,
                vclk_op_divn: 2,
                plla_clkout_vreg_bypass: true,
                txa_ldoclk_bypass: true,
                txa_div2_en: true,
            },
            2700 => DpLinkRateConfig {
                freqinit_count_target: 0x1c2,
                fbdivn_frac_den: 0x3ffe,
                fbdivn_frac_num: 0x1fff,
                pclk_div_sel: 9,
                lfclk_ctrl: 5,
                vclk_op_divn: 2,
                plla_clkout_vreg_bypass: true,
                txa_ldoclk_bypass: true,
                txa_div2_en: false,
            },
            5400 => DpLinkRateConfig {
                freqinit_count_target: 0x1c2,
                fbdivn_frac_den: 0x3ffe,
                fbdivn_frac_num: 0x1fff,
                pclk_div_sel: 4,
                lfclk_ctrl: 5,
                vclk_op_divn: 0,
                plla_clkout_vreg_bypass: true,
                txa_ldoclk_bypass: true,
                txa_div2_en: false,
            },
            8100 => DpLinkRateConfig {
                freqinit_count_target: 0x2a3,
                fbdivn_frac_den: 0x3ffc,
                fbdivn_frac_num: 0x2ffd,
                pclk_div_sel: 4,
                lfclk_ctrl: 6,
                vclk_op_divn: 0,
                plla_clkout_vreg_bypass: false,
                txa_ldoclk_bypass: false,
                txa_div2_en: false,
            },
            _ => return Err("apple-atcphy: unsupported DP link rate"),
        };

        if !self.poll_core_bit(
            ACIOPHY_CMN_SHM_STS_REG0,
            ACIOPHY_CMN_SHM_STS_REG0_CMD_READY,
            true,
            10_000,
        ) {
            return Err("apple-atcphy: common PLL command interface is not ready");
        }

        self.core_clear32(AUSPLL_FREQ_CFG, AUSPLL_FREQ_REFCLK);
        self.core_mask32(
            AUSPLL_FREQ_DESC_A,
            AUSPLL_FD_FREQ_COUNT_TARGET,
            field_prep(AUSPLL_FD_FREQ_COUNT_TARGET, cfg.freqinit_count_target),
        );
        self.core_clear32(
            AUSPLL_FREQ_DESC_A,
            AUSPLL_FD_FBDIVN_HALF | AUSPLL_FD_REV_DIVN | AUSPLL_FD_KPKI_SCALE_HBW,
        );
        self.core_mask32(
            AUSPLL_FREQ_DESC_A,
            AUSPLL_FD_KI_MAN,
            field_prep(AUSPLL_FD_KI_MAN, 8),
        );
        self.core_mask32(
            AUSPLL_FREQ_DESC_A,
            AUSPLL_FD_KI_EXP,
            field_prep(AUSPLL_FD_KI_EXP, 3),
        );
        self.core_mask32(
            AUSPLL_FREQ_DESC_A,
            AUSPLL_FD_KP_MAN,
            field_prep(AUSPLL_FD_KP_MAN, 8),
        );
        self.core_mask32(
            AUSPLL_FREQ_DESC_A,
            AUSPLL_FD_KP_EXP,
            field_prep(AUSPLL_FD_KP_EXP, 7),
        );
        self.core_mask32(
            AUSPLL_FREQ_DESC_B,
            AUSPLL_FD_FBDIVN_FRAC_DEN,
            field_prep(AUSPLL_FD_FBDIVN_FRAC_DEN, cfg.fbdivn_frac_den),
        );
        self.core_mask32(
            AUSPLL_FREQ_DESC_B,
            AUSPLL_FD_FBDIVN_FRAC_NUM,
            field_prep(AUSPLL_FD_FBDIVN_FRAC_NUM, cfg.fbdivn_frac_num),
        );
        self.core_clear32(
            AUSPLL_FREQ_DESC_C,
            AUSPLL_FD_SDM_SSC_STEP | AUSPLL_FD_SDM_SSC_EN,
        );
        self.core_mask32(
            AUSPLL_FREQ_DESC_C,
            AUSPLL_FD_PCLK_DIV_SEL,
            field_prep(AUSPLL_FD_PCLK_DIV_SEL, cfg.pclk_div_sel),
        );
        self.core_mask32(
            AUSPLL_FREQ_DESC_C,
            AUSPLL_FD_LFSDM_DIV,
            field_prep(AUSPLL_FD_LFSDM_DIV, 1),
        );
        self.core_mask32(
            AUSPLL_FREQ_DESC_C,
            AUSPLL_FD_LFCLK_CTRL,
            field_prep(AUSPLL_FD_LFCLK_CTRL, cfg.lfclk_ctrl),
        );
        self.core_mask32(
            AUSPLL_FREQ_DESC_C,
            AUSPLL_FD_VCLK_OP_DIVN,
            field_prep(AUSPLL_FD_VCLK_OP_DIVN, cfg.vclk_op_divn),
        );
        self.core_set32(AUSPLL_FREQ_DESC_C, AUSPLL_FD_VCLK_PRE_DIVN);
        self.core_mask32(
            AUSPLL_CLKOUT_DIV,
            AUSPLL_CLKOUT_PLLA_REFBUFCLK_DI,
            field_prep(AUSPLL_CLKOUT_PLLA_REFBUFCLK_DI, 7),
        );
        if cfg.plla_clkout_vreg_bypass {
            self.core_set32(AUSPLL_CLKOUT_DTC_VREG, AUSPLL_DTC_VREG_BYPASS);
        } else {
            self.core_clear32(AUSPLL_CLKOUT_DTC_VREG, AUSPLL_DTC_VREG_BYPASS);
        }
        self.core_set32(AUSPLL_BGR, AUSPLL_BGR_CTRL_AVAIL);
        self.core_set32(
            AUSPLL_CLKOUT_MASTER,
            AUSPLL_CLKOUT_MASTER_PCLK_DRVR_EN
                | AUSPLL_CLKOUT_MASTER_PCLK2_DRVR_EN
                | AUSPLL_CLKOUT_MASTER_REFBUFCLK_DRVR_EN,
        );

        self.auspll_apb_command(0)?;
        if !self.poll_core_bit(ACIOPHY_DP_PCLK_STAT, ACIOPHY_AUSPLL_LOCK, true, 10_000) {
            return Err("apple-atcphy: DisplayPort PLL did not lock");
        }
        self.auspll_apb_command(0x2800)?;

        match (self.dp_mode.unwrap(), self.swap_lanes) {
            (AtcPhyMode::DisplayPort, _) => {
                self.configure_dp_lane(0, &cfg)?;
                self.configure_dp_lane(1, &cfg)?;
            }
            (_, true) => self.configure_dp_lane(0, &cfg)?,
            (_, false) => self.configure_dp_lane(1, &cfg)?,
        }
        self.core_clear32(
            ACIOPHY_DP_CTRL0,
            DP_PMA_BYTECLK_RESET | DP_MAC_DIV20_CLK_SEL,
        );
        self.dp_link_rate = Some(link_rate);
        println!(
            "[apple-atcphy] DP link rate configured: {} Mbps/lane",
            link_rate
        );
        Ok(())
    }

    /// Configure orientation and enter a DP-capable Type-C mode.
    pub fn configure_displayport(
        &mut self,
        mode: AtcPhyMode,
        reverse: bool,
    ) -> Result<(), &'static str> {
        if !matches!(mode, AtcPhyMode::DisplayPort | AtcPhyMode::Usb3Dp) {
            return Err("apple-atcphy: requested mode is not DisplayPort-capable");
        }
        if self.dp_mode == Some(mode) && self.swap_lanes == reverse {
            println!(
                "[apple-atcphy] reusing boot-time {:?} mode (reversed={})",
                mode, reverse
            );
            return Ok(());
        }
        self.swap_lanes = reverse;
        self.init_dp(mode)
    }

    /// Maximum logical DisplayPort lane count for the current Type-C mode.
    pub fn max_dp_lane_count(&self) -> u32 {
        match self.dp_mode {
            Some(AtcPhyMode::DisplayPort) => 4,
            Some(AtcPhyMode::Usb3Dp) => 2,
            _ => 0,
        }
    }

    /// Record the lane count negotiated by DCP's DPTX service.
    pub fn set_active_dp_lane_count(&mut self, lanes: u32) -> Result<(), &'static str> {
        if lanes == 3 || lanes > self.max_dp_lane_count() {
            return Err("apple-atcphy: invalid active DP lane count");
        }
        self.active_dp_lanes = lanes;
        Ok(())
    }

    /// Current negotiated DisplayPort lane count.
    pub fn active_dp_lane_count(&self) -> u32 {
        self.active_dp_lanes
    }

    /// Physical address of this ATC instance's t8103 display crossbar block.
    pub fn display_crossbar_paddr(&self) -> usize {
        self.core_paddr + 0x4c000
    }

    fn configure_pipehandler_usb3(&mut self, host: bool) -> Result<(), &'static str> {
        if self.pipehandler_up {
            return Ok(());
        }

        self.pipehandler_check()?;

        if host {
            self.ph_clear32(
                PIPEHANDLER_OVERRIDE_VALUES,
                PIPEHANDLER_OVERRIDE_VAL_RXDETECT0 | PIPEHANDLER_OVERRIDE_VAL_RXDETECT1,
            );
            self.ph_set32(PIPEHANDLER_OVERRIDE, PIPEHANDLER_OVERRIDE_RXVALID);
            self.ph_set32(PIPEHANDLER_OVERRIDE, PIPEHANDLER_OVERRIDE_RXDETECT);

            self.pipehandler_lock()?;

            self.core_set32(
                ACIOPHY_TOP_BIST_PHY_CFG0,
                ACIOPHY_TOP_BIST_PHY_CFG0_LN0_RESET_N,
            );
            self.core_set32(
                ACIOPHY_TOP_BIST_OV_CFG,
                ACIOPHY_TOP_BIST_OV_CFG_LN0_RESET_N_OV,
            );
            if !self.poll_core_bit(
                ACIOPHY_TOP_PHY_STAT,
                ACIOPHY_TOP_PHY_STAT_LN0_BUSY,
                false,
                ACIOPHY_STATUS_TIMEOUT_US,
            ) {
                println!("[apple-atcphy] warning: lane 0 remained busy before BIST");
            }

            self.core_set32(
                ACIOPHY_TOP_BIST_READ_CTRL,
                ACIOPHY_TOP_BIST_READ_CTRL_LN0_PHY_STATUS_RE,
            );
            self.core_clear32(
                ACIOPHY_TOP_BIST_READ_CTRL,
                ACIOPHY_TOP_BIST_READ_CTRL_LN0_PHY_STATUS_RE,
            );
            self.core_mask32(
                ACIOPHY_TOP_BIST_PHY_CFG1,
                ACIOPHY_TOP_BIST_PHY_CFG1_LN0_PWR_DOWN_MASK,
                ACIOPHY_TOP_BIST_PHY_CFG1_LN0_PWR_DOWN_ON,
            );
            self.core_set32(
                ACIOPHY_TOP_BIST_OV_CFG,
                ACIOPHY_TOP_BIST_OV_CFG_LN0_PWR_DOWN_OV,
            );
            self.core_set32(
                ACIOPHY_TOP_BIST_CIOPHY_CFG1,
                ACIOPHY_TOP_BIST_CIOPHY_CFG1_CLK_EN,
            );
            self.core_set32(
                ACIOPHY_TOP_BIST_CIOPHY_CFG1,
                ACIOPHY_TOP_BIST_CIOPHY_CFG1_BIST_EN,
            );
            self.core_write32(ACIOPHY_TOP_BIST_CIOPHY_CFG1, 0);

            if !self.poll_core_bit(
                ACIOPHY_TOP_PHY_STAT,
                ACIOPHY_TOP_PHY_STAT_LN0_READY,
                true,
                ACIOPHY_STATUS_TIMEOUT_US,
            ) {
                println!("[apple-atcphy] warning: lane 0 did not become ready during BIST");
            }
            if !self.poll_core_bit(
                ACIOPHY_TOP_PHY_STAT,
                ACIOPHY_TOP_PHY_STAT_LN0_BUSY,
                false,
                ACIOPHY_STATUS_TIMEOUT_US,
            ) {
                println!("[apple-atcphy] warning: lane 0 remained busy after BIST");
            }

            let nonselected = self.ph_read32(PIPEHANDLER_NONSELECTED_OVERRIDE);
            self.ph_write32(
                PIPEHANDLER_NONSELECTED_OVERRIDE,
                (nonselected & !PIPEHANDLER_NATIVE_POWER_DOWN_MASK) | 3,
            );
            self.ph_clear32(PIPEHANDLER_NONSELECTED_OVERRIDE, PIPEHANDLER_NATIVE_RESET);

            self.core_write32(ACIOPHY_TOP_BIST_OV_CFG, 0);
            self.core_set32(
                ACIOPHY_TOP_BIST_CIOPHY_CFG1,
                ACIOPHY_TOP_BIST_CIOPHY_CFG1_CLK_EN,
            );
            self.core_set32(
                ACIOPHY_TOP_BIST_CIOPHY_CFG1,
                ACIOPHY_TOP_BIST_CIOPHY_CFG1_BIST_EN,
            );
        }

        self.ph_mask32(
            PIPEHANDLER_MUX_CTRL,
            PIPEHANDLER_MUX_CTRL_CLK_MASK,
            PIPEHANDLER_MUX_CTRL_CLK_OFF << 3,
        );
        scarlet::time::udelay(10);
        self.ph_mask32(
            PIPEHANDLER_MUX_CTRL,
            PIPEHANDLER_MUX_CTRL_DATA_MASK,
            PIPEHANDLER_MUX_CTRL_DATA_USB3,
        );
        scarlet::time::udelay(10);
        self.ph_mask32(
            PIPEHANDLER_MUX_CTRL,
            PIPEHANDLER_MUX_CTRL_CLK_MASK,
            PIPEHANDLER_MUX_CTRL_CLK_USB3 << 3,
        );
        scarlet::time::udelay(10);

        self.ph_clear32(PIPEHANDLER_OVERRIDE, PIPEHANDLER_OVERRIDE_RXVALID);
        self.ph_clear32(PIPEHANDLER_OVERRIDE, PIPEHANDLER_OVERRIDE_RXDETECT);

        if host {
            if self.pipehandler_unlock().is_err() {
                println!("[apple-atcphy] warning: failed to unlock USB3 pipehandler");
            }
        }
        self.pipehandler_up = true;
        println!("[apple-atcphy] USB3 pipehandler configured (host={})", host);
        Ok(())
    }

    fn configure_pipehandler_dummy(&mut self) -> Result<(), &'static str> {
        self.pipehandler_check()?;
        self.ph_clear32(
            PIPEHANDLER_OVERRIDE_VALUES,
            PIPEHANDLER_OVERRIDE_VAL_RXDETECT0 | PIPEHANDLER_OVERRIDE_VAL_RXDETECT1,
        );
        self.ph_set32(PIPEHANDLER_OVERRIDE, PIPEHANDLER_OVERRIDE_RXVALID);
        self.ph_set32(PIPEHANDLER_OVERRIDE, PIPEHANDLER_OVERRIDE_RXDETECT);
        if self.pipehandler_lock().is_err() {
            println!("[apple-atcphy] warning: failed to lock dummy pipehandler");
        }

        self.ph_mask32(
            PIPEHANDLER_MUX_CTRL,
            PIPEHANDLER_MUX_CTRL_CLK_MASK,
            PIPEHANDLER_MUX_CTRL_CLK_OFF << 3,
        );
        scarlet::time::udelay(10);
        self.ph_mask32(
            PIPEHANDLER_MUX_CTRL,
            PIPEHANDLER_MUX_CTRL_DATA_MASK,
            PIPEHANDLER_MUX_CTRL_DATA_DUMMY,
        );
        scarlet::time::udelay(10);
        self.ph_mask32(
            PIPEHANDLER_MUX_CTRL,
            PIPEHANDLER_MUX_CTRL_CLK_MASK,
            PIPEHANDLER_MUX_CTRL_CLK_DUMMY << 3,
        );
        scarlet::time::udelay(10);

        if self.pipehandler_unlock().is_err() {
            println!("[apple-atcphy] warning: failed to unlock dummy pipehandler");
        }
        self.ph_mask32(
            PIPEHANDLER_NONSELECTED_OVERRIDE,
            PIPEHANDLER_NATIVE_POWER_DOWN_MASK,
            2,
        );
        self.ph_set32(PIPEHANDLER_NONSELECTED_OVERRIDE, PIPEHANDLER_NATIVE_RESET);
        self.pipehandler_up = false;
        Ok(())
    }

    fn setup_pipehandler(&self) {
        self.ph_mask32(
            PIPEHANDLER_MUX_CTRL,
            PIPEHANDLER_MUX_CTRL_CLK_MASK,
            PIPEHANDLER_MUX_CTRL_CLK_OFF << 3,
        );
        scarlet::time::udelay(10);
        self.ph_mask32(
            PIPEHANDLER_MUX_CTRL,
            PIPEHANDLER_MUX_CTRL_DATA_MASK,
            PIPEHANDLER_MUX_CTRL_DATA_DUMMY,
        );
        scarlet::time::udelay(10);
        self.ph_mask32(
            PIPEHANDLER_MUX_CTRL,
            PIPEHANDLER_MUX_CTRL_CLK_MASK,
            PIPEHANDLER_MUX_CTRL_CLK_DUMMY << 3,
        );
        scarlet::time::udelay(10);
    }

    fn dwc3_reset_assert_raw(&self) {
        self.ph_clear32(PIPEHANDLER_AON_GEN, PIPEHANDLER_AON_GEN_DWC3_RESET_N);
        self.ph_set32(PIPEHANDLER_AON_GEN, PIPEHANDLER_AON_GEN_DWC3_FORCE_CLAMP_EN);
    }

    fn dwc3_reset_assert(&mut self) {
        println!("[apple-atcphy] dwc3 reset assert");
        self.dwc3_reset_assert_raw();

        if self.pipehandler_up {
            if self.configure_pipehandler_dummy().is_err() {
                println!("[apple-atcphy] warning: failed to switch PIPE to dummy");
            }
        }
        self.usb2_power_off();
    }

    fn dwc3_reset_deassert(&mut self) {
        println!("[apple-atcphy] dwc3 reset deassert");
        self.ph_clear32(PIPEHANDLER_AON_GEN, PIPEHANDLER_AON_GEN_DWC3_FORCE_CLAMP_EN);
        self.ph_set32(PIPEHANDLER_AON_GEN, PIPEHANDLER_AON_GEN_DWC3_RESET_N);
    }

    fn configure_pipehandler_dp(&self, swap_lanes: bool) {
        let (lane0, _lane1) = if swap_lanes { (1, 0) } else { (0, 1) };

        let nonselected = self.ph_read32(PIPEHANDLER_NONSELECTED_OVERRIDE);
        self.ph_write32(
            PIPEHANDLER_NONSELECTED_OVERRIDE,
            (nonselected & !PIPEHANDLER_NATIVE_POWER_DOWN_MASK) | 3,
        );
        self.ph_clear32(PIPEHANDLER_NONSELECTED_OVERRIDE, PIPEHANDLER_NATIVE_RESET);

        // Configure the DP lane
        let _dp_lane = if lane0 == 0 { 0 } else { 1 };
        let mut mux = self.ph_read32(PIPEHANDLER_MUX_CTRL);

        mux = (mux & !PIPEHANDLER_MUX_CTRL_CLK_MASK) | (PIPEHANDLER_MUX_CTRL_CLK_OFF << 3);
        self.ph_write32(PIPEHANDLER_MUX_CTRL, mux);
        scarlet::time::udelay(10);

        mux = (mux & !PIPEHANDLER_MUX_CTRL_DATA_MASK) | PIPEHANDLER_MUX_CTRL_DATA_DP;
        self.ph_write32(PIPEHANDLER_MUX_CTRL, mux);
        scarlet::time::udelay(10);

        mux = (mux & !PIPEHANDLER_MUX_CTRL_CLK_MASK) | (PIPEHANDLER_MUX_CTRL_CLK_DP << 3);
        self.ph_write32(PIPEHANDLER_MUX_CTRL, mux);
        scarlet::time::udelay(10);
    }

    fn apply_mode_tunables(&self, mode: AtcPhyMode, swap_lanes: bool) {
        let (lane0_idx, lane1_idx) = if swap_lanes { (1, 0) } else { (0, 1) };

        apply_tunables(&self.common_a, self.core_base);

        if let Some(axi2af_base) = self.axi2af_base {
            apply_tunables(&self.axi2af_tunables, axi2af_base);
        }

        apply_tunables(&self.common_b, self.core_base);

        match mode {
            AtcPhyMode::Usb3 | AtcPhyMode::Usb3Dp => {
                apply_tunables(
                    if lane0_idx == 0 {
                        &self.lane0_usb
                    } else {
                        &self.lane1_usb
                    },
                    self.core_base,
                );
                apply_tunables(
                    if lane1_idx == 0 {
                        &self.lane0_dp
                    } else {
                        &self.lane1_dp
                    },
                    self.core_base,
                );
            }
            AtcPhyMode::DisplayPort => {
                apply_tunables(
                    if lane0_idx == 0 {
                        &self.lane0_dp
                    } else {
                        &self.lane1_dp
                    },
                    self.core_base,
                );
                apply_tunables(
                    if lane1_idx == 0 {
                        &self.lane0_dp
                    } else {
                        &self.lane1_dp
                    },
                    self.core_base,
                );
            }
        }
    }

    /// Print the software and hardware state relevant to DisplayPort AUX and
    /// link negotiation. This is intentionally read-only so it can be called
    /// from the DCP callback path without changing PHY sequencing.
    pub fn log_displayport_state(&self, stage: &str) {
        println!(
            "[apple-atcphy] DP snapshot stage={} core={:#x} mode={:?} reversed={} pipe-up={} rate={:?} lanes={}",
            stage,
            self.core_paddr,
            self.dp_mode,
            self.swap_lanes,
            self.pipehandler_up,
            self.dp_link_rate,
            self.active_dp_lanes
        );
        println!(
            "[apple-atcphy] DP snapshot tunables common={}/{} axi2af={} usb={}/{} dp={}/{}",
            self.common_a.len(),
            self.common_b.len(),
            self.axi2af_tunables.len(),
            self.lane0_usb.len(),
            self.lane1_usb.len(),
            self.lane0_dp.len(),
            self.lane1_dp.len()
        );
        println!(
            "[apple-atcphy] DP snapshot core misc={:#x} power={:#x}/{:#x} cfg0={:#x} sleep={:#x} lane={:#x} crossbar={:#x}",
            self.core_read32(ATCPHY_MISC),
            self.core_read32(ATCPHY_POWER_CTRL),
            self.core_read32(ATCPHY_POWER_STAT),
            self.core_read32(ACIOPHY_CFG0),
            self.core_read32(ACIOPHY_SLEEP_CTRL),
            self.core_read32(ACIOPHY_LANE_MODE),
            self.core_read32(ACIOPHY_CROSSBAR)
        );
        println!(
            "[apple-atcphy] DP snapshot clocks dpctrl={:#x} cio3pll={:#x} pll-common={:#x} pclk-stat={:#x} common-stat={:#x}",
            self.core_read32(ACIOPHY_DP_CTRL0),
            self.core_read32(CIO3PLL_CLK_CTRL),
            self.core_read32(ACIOPHY_PLL_COMMON_CTRL),
            self.core_read32(ACIOPHY_DP_PCLK_STAT),
            self.core_read32(ACIOPHY_CMN_SHM_STS_REG0)
        );
        if self.lpdptx_base.is_some() {
            println!(
                "[apple-atcphy] DP snapshot aux control={:#x} ctrl={:#x} ldo={:#x} margin={:#x} shm={:#x}/{:#x}",
                self.lpdptx_read32(LPDPTX_AUX_CONTROL),
                self.lpdptx_read32(LPDPTX_AUX_CTRL),
                self.lpdptx_read32(LPDPTX_AUX_LDO_CTRL),
                self.lpdptx_read32(LPDPTX_AUX_MARGIN),
                self.lpdptx_read32(LPDPTX_AUX_SHM_CTRL0),
                self.lpdptx_read32(LPDPTX_AUX_SHM_CTRL1)
            );
        } else {
            println!("[apple-atcphy] DP snapshot aux unmapped");
        }
        println!(
            "[apple-atcphy] DP snapshot pipe override={:#x}/{:#x} mux={:#x} lock={:#x}/{:#x} aon={:#x} nonselected={:#x}",
            self.ph_read32(PIPEHANDLER_OVERRIDE),
            self.ph_read32(PIPEHANDLER_OVERRIDE_VALUES),
            self.ph_read32(PIPEHANDLER_MUX_CTRL),
            self.ph_read32(PIPEHANDLER_LOCK_REQ),
            self.ph_read32(PIPEHANDLER_LOCK_ACK),
            self.ph_read32(PIPEHANDLER_AON_GEN),
            self.ph_read32(PIPEHANDLER_NONSELECTED_OVERRIDE)
        );
    }

    /// Initialize the PHY in USB3 mode.
    ///
    /// # Returns
    ///
    /// `Ok(())` when the PHY is powered and configured for USB3 operation.
    pub fn init(&mut self) -> Result<(), &'static str> {
        println!("[apple-atcphy] initializing...");

        self.dp_mode = None;
        self.dp_link_rate = None;
        self.active_dp_lanes = 0;

        self.usb2_power_on();
        self.core_power_on()?;
        self.apply_mode_tunables(AtcPhyMode::Usb3, self.swap_lanes);

        // These are override bits, not complete register values. Preserve the
        // boot/calibration state in the remaining fields across USB3 -> DP
        // mode transitions.
        self.core_set32(AUSPLL_FSM_CTRL, 0x1fe000);
        self.core_set32(AUSPLL_APB_CMD_OVERRIDE, AUSPLL_APB_CMD_OVERRIDE_UNK28);

        self.core_set32(ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_SMALL_OV);
        scarlet::time::udelay(10);
        self.core_set32(ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_BIG_OV);
        scarlet::time::udelay(10);
        self.core_set32(ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_CLAMP_OV);
        scarlet::time::udelay(10);

        self.core_mask32(ACIOPHY_SLEEP_CTRL, ACIOPHY_SLEEP_CTRL_TX_SMALL_OV, 3 << 6);
        scarlet::time::udelay(10);
        self.core_mask32(ACIOPHY_SLEEP_CTRL, ACIOPHY_SLEEP_CTRL_TX_BIG_OV, 3 << 2);
        scarlet::time::udelay(10);
        self.core_mask32(ACIOPHY_SLEEP_CTRL, ACIOPHY_SLEEP_CTRL_TX_CLAMP_OV, 3 << 10);
        scarlet::time::udelay(10);

        self.core_mask32(ACIOPHY_CFG0, ACIOPHY_CFG0_RX_BIG_OV, 3 << 12);
        scarlet::time::udelay(10);
        self.core_mask32(ACIOPHY_CFG0, ACIOPHY_CFG0_RX_SMALL_OV, 3 << 8);
        scarlet::time::udelay(10);
        self.core_mask32(ACIOPHY_CFG0, ACIOPHY_CFG0_RX_CLAMP_OV, 3 << 16);
        scarlet::time::udelay(10);

        self.configure_crossbar();

        self.core_set32(CIO3PLL_CLK_CTRL, CIO3PLL_CLK_PCLK_EN);
        self.core_set32(CIO3PLL_CLK_CTRL, CIO3PLL_CLK_REFCLK_EN);

        self.core_set32(ATCPHY_POWER_CTRL, ATCPHY_POWER_PHY_RESET_N);

        println!("[apple-atcphy] initialized (USB3 PHY)");
        Ok(())
    }

    /// Initialize the PHY in a DisplayPort-capable mode.
    ///
    /// # Arguments
    ///
    /// * `mode` - DisplayPort-related ATC PHY mode to apply.
    ///
    /// # Returns
    ///
    /// `Ok(())` when the PHY is powered and configured for the requested mode.
    pub fn init_dp(&mut self, mode: AtcPhyMode) -> Result<(), &'static str> {
        if self.lpdptx_base.is_none() || self.axi2af_base.is_none() {
            return Err("apple-atcphy: lpdptx/axi2af regions not mapped, cannot init DP");
        }

        println!("[apple-atcphy] initializing in DP mode ({:?})...", mode);

        self.dp_mode = Some(mode);
        self.dp_link_rate = None;
        self.active_dp_lanes = 0;
        self.usb2_power_on();
        self.core_power_on()?;

        // DWC3 may already have brought the USB3 PIPE interface up before the
        // Type-C controller reports DP Alt Mode.  Reprogramming the ATC lane
        // mux and AUX path while that interface is live leaves the PHY in a
        // half-transitioned state: USB continues to work, but DCP cannot talk
        // to the sink over AUX.  Quiesce PIPE for the mode change and restore
        // it after the USB3+DP lane layout has been installed.  This mirrors
        // the transition used by Asahi's ATC PHY driver.
        let pipehandler_was_up = self.pipehandler_up;
        if pipehandler_was_up {
            println!(
                "[apple-atcphy] quiescing USB3 pipehandler for {:?} transition",
                mode
            );
            self.configure_pipehandler_dummy()?;
        }

        self.apply_mode_tunables(mode, self.swap_lanes);

        self.core_set32(AUSPLL_FSM_CTRL, 0x1fe000);
        self.core_set32(AUSPLL_APB_CMD_OVERRIDE, AUSPLL_APB_CMD_OVERRIDE_UNK28);

        self.core_set32(ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_SMALL_OV);
        scarlet::time::udelay(10);
        self.core_set32(ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_BIG_OV);
        scarlet::time::udelay(10);
        self.core_set32(ACIOPHY_CFG0, ACIOPHY_CFG0_COMMON_CLAMP_OV);
        scarlet::time::udelay(10);
        self.core_mask32(ACIOPHY_SLEEP_CTRL, ACIOPHY_SLEEP_CTRL_TX_SMALL_OV, 3 << 6);
        scarlet::time::udelay(10);
        self.core_mask32(ACIOPHY_SLEEP_CTRL, ACIOPHY_SLEEP_CTRL_TX_BIG_OV, 3 << 2);
        scarlet::time::udelay(10);
        self.core_mask32(ACIOPHY_SLEEP_CTRL, ACIOPHY_SLEEP_CTRL_TX_CLAMP_OV, 3 << 10);
        scarlet::time::udelay(10);
        self.core_mask32(ACIOPHY_CFG0, ACIOPHY_CFG0_RX_BIG_OV, 3 << 12);
        scarlet::time::udelay(10);
        self.core_mask32(ACIOPHY_CFG0, ACIOPHY_CFG0_RX_SMALL_OV, 3 << 8);
        scarlet::time::udelay(10);
        self.core_mask32(ACIOPHY_CFG0, ACIOPHY_CFG0_RX_CLAMP_OV, 3 << 16);
        scarlet::time::udelay(10);

        self.enable_dp_aux();
        self.core_set32(
            CIO3PLL_CLK_CTRL,
            CIO3PLL_CLK_PCLK_EN | CIO3PLL_CLK_REFCLK_EN,
        );
        self.configure_lanes(mode);
        self.core_set32(ATCPHY_POWER_CTRL, ATCPHY_POWER_PHY_RESET_N);

        // During the normal boot path DWC3 is still held in reset here.  Keep
        // PIPE on the dummy endpoint until DWC3 has initialized and asks for
        // its USB host role, matching Fairydust's Type-C mux -> role-switch
        // ordering.  Only restore PIPE here for the legacy late-transition
        // path where it was already live on entry.
        if pipehandler_was_up && mode == AtcPhyMode::Usb3Dp {
            self.configure_pipehandler_usb3(true)?;
        }

        self.log_displayport_state("init-dp");
        println!("[apple-atcphy] initialized ({:?} mode)", mode);
        Ok(())
    }

    fn set_orientation(&mut self, orientation: PhyOrientation) -> Result<(), PhyError> {
        match orientation {
            PhyOrientation::None | PhyOrientation::Normal => {
                self.swap_lanes = false;
                println!("[apple-atcphy] orientation normal");
                Ok(())
            }
            PhyOrientation::Reverse => {
                self.swap_lanes = true;
                println!("[apple-atcphy] orientation reverse");
                Ok(())
            }
        }
    }
}

struct AppleAtcPhyProvider {
    phy: Arc<IrqSpinLock<AppleAtcPhy>>,
    lanes: Vec<Arc<AppleAtcPhyLane>>,
}

impl AppleAtcPhyProvider {
    fn new(phy: Arc<IrqSpinLock<AppleAtcPhy>>) -> Self {
        Self {
            phy: Arc::clone(&phy),
            lanes: alloc::vec![
                Arc::new(AppleAtcPhyLane::new(Arc::clone(&phy), PHY_TYPE_USB2)),
                Arc::new(AppleAtcPhyLane::new(phy, PHY_TYPE_USB3)),
            ],
        }
    }
}

impl PhyProvider for AppleAtcPhyProvider {
    fn name(&self) -> &'static str {
        "apple-atcphy"
    }

    fn phy_cells(&self) -> usize {
        1
    }

    fn get_phy(&self, spec: &[u32]) -> Result<PhyHandle, PhyError> {
        if spec.len() != self.phy_cells() {
            return Err(PhyError::NotFound);
        }

        let lane = self
            .lanes
            .iter()
            .find(|lane| lane.phy_type() == spec[0])
            .ok_or(PhyError::NotFound)?;
        Ok(PhyHandle::new(lane.clone()))
    }
}

impl ResetController for AppleAtcPhyProvider {
    fn name(&self) -> &'static str {
        "apple-atcphy-reset"
    }

    fn reset_cells(&self) -> usize {
        0
    }

    fn assert_reset(&self, spec: &[u32]) -> Result<(), &'static str> {
        if !spec.is_empty() {
            return Err("apple-atcphy: invalid reset specifier");
        }
        self.phy.lock().dwc3_reset_assert();
        Ok(())
    }

    fn deassert_reset(&self, spec: &[u32]) -> Result<(), &'static str> {
        if !spec.is_empty() {
            return Err("apple-atcphy: invalid reset specifier");
        }
        self.phy.lock().dwc3_reset_deassert();
        Ok(())
    }
}

struct AppleAtcPhyLane {
    phy: Arc<IrqSpinLock<AppleAtcPhy>>,
    lane: u32,
    mode: IrqSpinLock<Option<PhyMode>>,
    orientation: IrqSpinLock<Option<PhyOrientation>>,
}

impl AppleAtcPhyLane {
    fn new(phy: Arc<IrqSpinLock<AppleAtcPhy>>, phy_type: u32) -> Self {
        Self {
            phy,
            lane: phy_type,
            mode: IrqSpinLock::new(None),
            orientation: IrqSpinLock::new(None),
        }
    }

    fn phy_type(&self) -> u32 {
        self.lane
    }

    fn atc_mode(&self) -> Result<AtcPhyMode, PhyError> {
        if self.lane == PHY_TYPE_USB3 {
            if let Some(mode) = self.phy.lock().dp_mode {
                return Ok(mode);
            }
        }
        match *self.mode.lock() {
            Some(PhyMode::UsbHost | PhyMode::UsbDevice | PhyMode::UsbOtg) | None => {
                Ok(AtcPhyMode::Usb3)
            }
            Some(PhyMode::DisplayPort) => Ok(AtcPhyMode::DisplayPort),
            Some(PhyMode::Other(0)) => Ok(AtcPhyMode::Usb3),
            Some(PhyMode::Other(1)) => Ok(AtcPhyMode::Usb3Dp),
            Some(_) => Err(PhyError::InvalidMode),
        }
    }

    fn power_on_current_mode(&self) -> Result<(), PhyError> {
        let mode = self.atc_mode()?;
        let mut phy = self.phy.lock();
        match (self.lane, mode) {
            (PHY_TYPE_USB2, AtcPhyMode::Usb3) => {
                let phy_mode = (*self.mode.lock()).unwrap_or(PhyMode::UsbHost);
                phy.usb2_set_mode(phy_mode)
            }
            (PHY_TYPE_USB3, AtcPhyMode::Usb3) => {
                phy.init().map_err(|_| PhyError::PowerOnFailed)?;
                Ok(())
            }
            (_, AtcPhyMode::DisplayPort | AtcPhyMode::Usb3Dp) => {
                phy.init_dp(mode).map_err(|_| PhyError::PowerOnFailed)
            }
            _ => Err(PhyError::PowerOnFailed),
        }
    }
}

impl Phy for AppleAtcPhyLane {
    fn name(&self) -> &'static str {
        match self.lane {
            PHY_TYPE_USB2 => "apple-atcphy-usb2",
            PHY_TYPE_USB3 => "apple-atcphy-usb3",
            _ => "apple-atcphy-lane",
        }
    }

    fn power_on(&self) -> Result<(), PhyError> {
        self.power_on_current_mode()
    }

    fn power_off(&self) -> Result<(), PhyError> {
        let mut phy = self.phy.lock();
        match self.lane {
            PHY_TYPE_USB2 => {
                phy.usb2_power_off();
                Ok(())
            }
            PHY_TYPE_USB3 => {
                if phy.configure_pipehandler_dummy().is_err() {
                    println!("[apple-atcphy] warning: failed to switch PIPE to dummy");
                }
                phy.pipehandler_up = false;
                phy.core_power_off().map_err(|_| PhyError::PowerOffFailed)
            }
            _ => Err(PhyError::PowerOffFailed),
        }
    }

    fn reset(&self) -> Result<(), PhyError> {
        self.power_on_current_mode().map_err(|error| match error {
            PhyError::PowerOnFailed => PhyError::ResetFailed,
            other => other,
        })
    }

    fn set_mode(&self, mode: PhyMode) -> Result<(), PhyError> {
        match mode {
            PhyMode::UsbHost
            | PhyMode::UsbDevice
            | PhyMode::UsbOtg
            | PhyMode::DisplayPort
            | PhyMode::Other(0)
            | PhyMode::Other(1) => {
                *self.mode.lock() = Some(mode);
                if self.lane == PHY_TYPE_USB2 {
                    self.phy.lock().usb2_set_mode(mode)?;
                } else if self.lane == PHY_TYPE_USB3 {
                    match mode {
                        PhyMode::UsbHost | PhyMode::UsbOtg | PhyMode::Other(0) => {
                            let mut phy = self.phy.lock();
                            if phy.dp_mode == Some(AtcPhyMode::DisplayPort) {
                                phy.configure_pipehandler_dummy()
                                    .map_err(|_| PhyError::HardwareError)?;
                            } else {
                                phy.configure_pipehandler_usb3(true)
                                    .map_err(|_| PhyError::HardwareError)?;
                            }
                        }
                        PhyMode::UsbDevice => {
                            let mut phy = self.phy.lock();
                            if phy.dp_mode == Some(AtcPhyMode::DisplayPort) {
                                phy.configure_pipehandler_dummy()
                                    .map_err(|_| PhyError::HardwareError)?;
                            } else {
                                phy.configure_pipehandler_usb3(false)
                                    .map_err(|_| PhyError::HardwareError)?;
                            }
                        }
                        _ => {}
                    }
                }
                Ok(())
            }
            _ => Err(PhyError::InvalidMode),
        }
    }

    fn get_mode(&self) -> Option<PhyMode> {
        *self.mode.lock()
    }

    fn set_orientation(&self, orientation: PhyOrientation) -> Result<(), PhyError> {
        *self.orientation.lock() = Some(orientation);
        if self.lane == PHY_TYPE_USB3 {
            let mut phy = self.phy.lock();
            phy.set_orientation(orientation)?;
            if self.mode.lock().is_some() {
                match orientation {
                    PhyOrientation::None | PhyOrientation::Normal | PhyOrientation::Reverse => {
                        phy.configure_crossbar();
                    }
                }
            }
        }
        Ok(())
    }

    fn get_orientation(&self) -> Option<PhyOrientation> {
        *self.orientation.lock()
    }
}

// =============================================================================
// Global Registry
// =============================================================================

struct AtcPhyEntry {
    instance: Arc<IrqSpinLock<AppleAtcPhy>>,
    phandle: u32,
    core_paddr: usize,
}

static ATC_PHY_REGISTRY: IrqSpinLock<alloc::vec::Vec<AtcPhyEntry>> =
    IrqSpinLock::new(alloc::vec::Vec::new());

/// Register an ATC PHY instance in the legacy local registry.
///
/// # Arguments
///
/// * `phy` - ATC PHY instance to store.
/// * `phandle` - Firmware phandle associated with the PHY node.
///
/// # Returns
///
/// Numeric local registry ID assigned to the instance.
pub fn register_atcphy(phy: AppleAtcPhy, phandle: u32) -> u32 {
    register_atcphy_shared(phy, phandle).0
}

fn register_atcphy_shared(phy: AppleAtcPhy, phandle: u32) -> (u32, Arc<IrqSpinLock<AppleAtcPhy>>) {
    let mut guard = ATC_PHY_REGISTRY.lock();
    let id = guard.len() as u32;
    let core_paddr = phy.core_paddr;
    let instance = Arc::new(IrqSpinLock::new(phy));
    guard.push(AtcPhyEntry {
        instance: Arc::clone(&instance),
        phandle,
        core_paddr,
    });
    (id, instance)
}

/// Look up a registered ATC PHY instance by local registry ID.
///
/// # Arguments
///
/// * `id` - Local registry ID returned by [`register_atcphy`].
///
/// # Returns
///
/// Shared ATC PHY instance, or `None` when `id` is unknown.
pub fn get_atcphy(id: u32) -> Option<Arc<IrqSpinLock<AppleAtcPhy>>> {
    let guard = ATC_PHY_REGISTRY.lock();
    guard.get(id as usize).map(|e| Arc::clone(&e.instance))
}

/// Look up a registered ATC PHY instance by firmware phandle.
///
/// # Arguments
///
/// * `phandle` - Firmware phandle used when the PHY was registered.
///
/// # Returns
///
/// Shared ATC PHY instance, or `None` when no matching registration exists.
pub fn get_atcphy_by_phandle(phandle: u32) -> Option<Arc<IrqSpinLock<AppleAtcPhy>>> {
    let guard = ATC_PHY_REGISTRY.lock();
    guard
        .iter()
        .find(|e| e.phandle == phandle)
        .map(|e| Arc::clone(&e.instance))
}

/// Look up a registered ATC PHY by its core MMIO physical address.
///
/// The local registry ID is probe-order dependent: on Apple systems firmware
/// may disable the unused Type-C controller, causing ATC1 to be registered as
/// local ID 0.  Hardware routes such as DCP's DPTX target must therefore use
/// the physical ATC identity instead of the registry position.
pub fn get_atcphy_by_core_paddr(core_paddr: usize) -> Option<Arc<IrqSpinLock<AppleAtcPhy>>> {
    let guard = ATC_PHY_REGISTRY.lock();
    guard
        .iter()
        .find(|entry| entry.core_paddr == core_paddr)
        .map(|entry| Arc::clone(&entry.instance))
}

// =============================================================================
// Platform Driver
// =============================================================================

fn probe_fn(device: &PlatformDeviceInfo) -> Result<(), &'static str> {
    let mem_resources: Vec<_> = device
        .get_resources()
        .iter()
        .filter(|r| matches!(r.res_type, PlatformDeviceResourceType::MEM))
        .collect();

    if mem_resources.len() < 5 {
        return Err("apple-atcphy: expected at least 5 memory resources");
    }

    let core_paddr = mem_resources[0].start;
    let core_size = mem_resources[0].end - mem_resources[0].start + 1;

    let lpdptx_paddr = mem_resources[1].start;
    let lpdptx_size = mem_resources[1].end - mem_resources[1].start + 1;

    let axi2af_paddr = mem_resources[2].start;
    let axi2af_size = mem_resources[2].end - mem_resources[2].start + 1;

    let usb2phy_paddr = mem_resources[3].start;
    let usb2phy_size = mem_resources[3].end - mem_resources[3].start + 1;

    let pipehandler_paddr = mem_resources[4].start;
    let pipehandler_size = mem_resources[4].end - mem_resources[4].start + 1;

    early_println!(
        "[apple-atcphy] probing {} core={:#x} lpdptx={:#x} axi2af={:#x} usb2phy={:#x} ph={:#x}",
        device.name(),
        core_paddr,
        lpdptx_paddr,
        axi2af_paddr,
        usb2phy_paddr,
        pipehandler_paddr
    );

    let core_base = scarlet::vm::ioremap(core_paddr, core_size)
        .map_err(|_| "apple-atcphy: ioremap core failed")?;
    let lpdptx_base = scarlet::vm::ioremap(lpdptx_paddr, lpdptx_size).ok();
    let axi2af_base = scarlet::vm::ioremap(axi2af_paddr, axi2af_size).ok();
    let usb2phy_base = scarlet::vm::ioremap(usb2phy_paddr, usb2phy_size)
        .map_err(|_| "apple-atcphy: ioremap usb2phy failed")?;
    let pipehandler_base = scarlet::vm::ioremap(pipehandler_paddr, pipehandler_size)
        .map_err(|_| "apple-atcphy: ioremap pipehandler failed")?;

    let mut phy = AppleAtcPhy::new(
        core_paddr,
        core_base,
        lpdptx_base,
        axi2af_base,
        usb2phy_base,
        pipehandler_base,
    );

    phy.common_a = parse_tunable_prop(device, "apple,tunable-common-a");
    phy.common_b = parse_tunable_prop(device, "apple,tunable-common-b");
    phy.axi2af_tunables = parse_tunable_prop(device, "apple,tunable-axi2af");
    phy.lane0_usb = parse_tunable_prop(device, "apple,tunable-lane0-usb");
    phy.lane1_usb = parse_tunable_prop(device, "apple,tunable-lane1-usb");
    phy.lane0_dp = parse_tunable_prop(device, "apple,tunable-lane0-dp");
    phy.lane1_dp = parse_tunable_prop(device, "apple,tunable-lane1-dp");

    let tunable_count = phy.common_a.len()
        + phy.common_b.len()
        + phy.axi2af_tunables.len()
        + phy.lane0_usb.len()
        + phy.lane1_usb.len()
        + phy.lane0_dp.len()
        + phy.lane1_dp.len();
    if tunable_count > 0 {
        early_println!(
            "[apple-atcphy] loaded {} tunables (common={}/{}, axi2af={}, usb={}/{}, dp={}/{})",
            tunable_count,
            phy.common_a.len(),
            phy.common_b.len(),
            phy.axi2af_tunables.len(),
            phy.lane0_usb.len(),
            phy.lane1_usb.len(),
            phy.lane0_dp.len(),
            phy.lane1_dp.len()
        );
    }

    phy.prepare_bootloader_state()?;

    let phandle = device
        .property("phandle")
        .and_then(|p| p.as_usize())
        .map(|v| v as u32)
        .or_else(|| {
            device
                .property("linux,phandle")
                .and_then(|p| p.as_usize())
                .map(|v| v as u32)
        })
        .unwrap_or(0);

    let (_id, phy_instance) = register_atcphy_shared(phy, phandle);
    let provider = Arc::new(AppleAtcPhyProvider::new(phy_instance));
    DeviceManager::get_manager()
        .register_phy_controller(phandle, Arc::clone(&provider) as Arc<dyn PhyProvider>);
    DeviceManager::get_manager()
        .register_reset_controller(phandle, provider as Arc<dyn ResetController>);

    early_println!("[apple-atcphy] registered (id={})", _id);
    Ok(())
}

fn remove_fn(_device: &PlatformDeviceInfo) -> Result<(), &'static str> {
    Ok(())
}

fn register_atcphy_driver() {
    let driver = PlatformDeviceDriver::new(
        "apple-atcphy",
        probe_fn,
        remove_fn,
        alloc::vec!["apple,t8103-atcphy", "apple,t6000-atcphy"],
    );

    // PHY must be registered before DWC3 (Core), so use Critical priority.
    // PHY nodes appear after USB nodes in Apple FDT, causing probe order issue.
    DeviceManager::get_manager().register_driver(Box::new(driver), DriverPriority::Critical);
}

scarlet::driver_initcall!(register_atcphy_driver);

#[used]
static SCARLET_DRIVER_APPLE_ATCPHY_ANCHOR: fn() = force_link;

/// Keep the driver object linked into kernel builds that rely on initcall anchors.
#[inline(never)]
pub fn force_link() {}
