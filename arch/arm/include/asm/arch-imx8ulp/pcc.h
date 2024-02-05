/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021 NXP
 */

#ifndef _ASM_ARCH_IMX8ULP_PCC_H
#define _ASM_ARCH_IMX8ULP_PCC_H

#include <asm/arch/cgc.h>

enum pcc1_entry {
	ADC1_PCC1_SLOT = 34,
};

enum pcc3_entry {
	DMA1_MP_PCC3_SLOT = 1,
	DMA1_CH0_PCC3_SLOT = 2,
	DMA1_CH1_PCC3_SLOT = 3,
	DMA1_CH2_PCC3_SLOT = 4,
	DMA1_CH3_PCC3_SLOT = 5,
	DMA1_CH4_PCC3_SLOT = 6,
	DMA1_CH5_PCC3_SLOT = 7,
	DMA1_CH6_PCC3_SLOT = 8,
	DMA1_CH7_PCC3_SLOT = 9,
	DMA1_CH8_PCC3_SLOT = 10,
	DMA1_CH9_PCC3_SLOT = 11,
	DMA1_CH10_PCC3_SLOT = 12,
	DMA1_CH11_PCC3_SLOT = 13,
	DMA1_CH12_PCC3_SLOT = 14,
	DMA1_CH13_PCC3_SLOT = 15,
	DMA1_CH14_PCC3_SLOT = 16,
	DMA1_CH15_PCC3_SLOT = 17,
	DMA1_CH16_PCC3_SLOT = 18,
	DMA1_CH17_PCC3_SLOT = 19,
	DMA1_CH18_PCC3_SLOT = 20,
	DMA1_CH19_PCC3_SLOT = 21,
	DMA1_CH20_PCC3_SLOT = 22,
	DMA1_CH21_PCC3_SLOT = 23,
	DMA1_CH22_PCC3_SLOT = 24,
	DMA1_CH23_PCC3_SLOT = 25,
	DMA1_CH24_PCC3_SLOT = 26,
	DMA1_CH25_PCC3_SLOT = 27,
	DMA1_CH26_PCC3_SLOT = 28,
	DMA1_CH27_PCC3_SLOT = 29,
	DMA1_CH28_PCC3_SLOT = 30,
	DMA1_CH29_PCC3_SLOT = 31,
	DMA1_CH30_PCC3_SLOT = 32,
	DMA1_CH31_PCC3_SLOT = 33,
	MU0_B_PCC3_SLOT = 34,
	MU3_A_PCC3_SLOT = 35,
	LLWU1_PCC3_SLOT = 38,
	UPOWER_PCC3_SLOT = 40,
	WDOG3_PCC3_SLOT = 42,
	WDOG4_PCC3_SLOT = 43,
	XRDC_MGR_PCC3_SLOT = 47,
	SEMA42_1_PCC3_SLOT = 48,
	ROMCP1_PCC3_SLOT = 49,
	LPIT1_PCC3_SLOT = 50,
	TPM4_PCC3_SLOT = 51,
	TPM5_PCC3_SLOT = 52,
	FLEXIO1_PCC3_SLOT = 53,
	I3C2_PCC3_SLOT = 54,
	LPI2C4_PCC3_SLOT = 55,
	LPI2C5_PCC3_SLOT = 56,
	LPUART4_PCC3_SLOT = 57,
	LPUART5_PCC3_SLOT = 58,
	LPSPI4_PCC3_SLOT = 59,
	LPSPI5_PCC3_SLOT = 60,
};

enum pcc4_entry {
	FLEXSPI2_PCC4_SLOT = 1,
	TPM6_PCC4_SLOT = 2,
	TPM7_PCC4_SLOT = 3,
	LPI2C6_PCC4_SLOT = 4,
	LPI2C7_PCC4_SLOT = 5,
	LPUART6_PCC4_SLOT = 6,
	LPUART7_PCC4_SLOT = 7,
	SAI4_PCC4_SLOT = 8,
	SAI5_PCC4_SLOT = 9,
	PCTLE_PCC4_SLOT = 10,
	PCTLF_PCC4_SLOT = 11,
	SDHC0_PCC4_SLOT = 13,
	SDHC1_PCC4_SLOT = 14,
	SDHC2_PCC4_SLOT = 15,
	USB0_PCC4_SLOT = 16,
	USBPHY_PCC4_SLOT = 17,
	USB1_PCC4_SLOT = 18,
	USB1PHY_PCC4_SLOT = 19,
	USB_XBAR_PCC4_SLOT = 20,
	ENET_PCC4_SLOT = 21,
	SFA1_PCC4_SLOT = 22,
	RGPIOE_PCC4_SLOT = 30,
	RGPIOF_PCC4_SLOT = 31,
};

enum pcc5_entry {
	DMA2_MP_PCC5_SLOT = 0,
	DMA2_CH0_PCC5_SLOT = 1,
	DMA2_CH1_PCC5_SLOT = 2,
	DMA2_CH2_PCC5_SLOT = 3,
	DMA2_CH3_PCC5_SLOT = 4,
	DMA2_CH4_PCC5_SLOT = 5,
	DMA2_CH5_PCC5_SLOT = 6,
	DMA2_CH6_PCC5_SLOT = 7,
	DMA2_CH7_PCC5_SLOT = 8,
	DMA2_CH8_PCC5_SLOT = 9,
	DMA2_CH9_PCC5_SLOT = 10,
	DMA2_CH10_PCC5_SLOT = 11,
	DMA2_CH11_PCC5_SLOT = 12,
	DMA2_CH12_PCC5_SLOT = 13,
	DMA2_CH13_PCC5_SLOT = 14,
	DMA2_CH14_PCC5_SLOT = 15,
	DMA2_CH15_PCC5_SLOT = 16,
	DMA2_CH16_PCC5_SLOT = 17,
	DMA2_CH17_PCC5_SLOT = 18,
	DMA2_CH18_PCC5_SLOT = 19,
	DMA2_CH19_PCC5_SLOT = 20,
	DMA2_CH20_PCC5_SLOT = 21,
	DMA2_CH21_PCC5_SLOT = 22,
	DMA2_CH22_PCC5_SLOT = 23,
	DMA2_CH23_PCC5_SLOT = 24,
	DMA2_CH24_PCC5_SLOT = 25,
	DMA2_CH25_PCC5_SLOT = 26,
	DMA2_CH26_PCC5_SLOT = 27,
	DMA2_CH27_PCC5_SLOT = 28,
	DMA2_CH28_PCC5_SLOT = 29,
	DMA2_CH29_PCC5_SLOT = 30,
	DMA2_CH30_PCC5_SLOT = 31,
	DMA2_CH31_PCC5_SLOT = 32,
	MU2_B_PCC5_SLOT = 33,
	MU3_B_PCC5_SLOT = 34,
	SEMA42_2_PCC5_SLOT = 35,
	CMC2_PCC5_SLOT = 36,
	AVD_SIM_PCC5_SLOT = 37,
	LPAV_CGC_PCC5_SLOT = 38,
	PCC5_PCC5_SLOT = 39,
	TPM8_PCC5_SLOT = 40,
	SAI6_PCC5_SLOT = 41,
	SAI7_PCC5_SLOT = 42,
	SPDIF_PCC5_SLOT = 43,
	ISI_PCC5_SLOT = 44,
	CSI_REGS_PCC5_SLOT = 45,
	CSI_PCC5_SLOT = 47,
	DSI_PCC5_SLOT = 48,
	WDOG5_PCC5_SLOT = 50,
	EPDC_PCC5_SLOT = 51,
	PXP_PCC5_SLOT = 52,
	SFA2_PCC5_SLOT = 53,
	GPU2D_PCC5_SLOT = 60,
	GPU3D_PCC5_SLOT = 61,
	DCNANO_PCC5_SLOT = 62,
	LPDDR4_PCC5_SLOT = 66,
	CSI_CLK_UI_PCC5_SLOT = 67,
	CSI_CLK_ESC_PCC5_SLOT = 68,
	RGPIOD_PCC5_SLOT = 69,
};

/* PCC registers */
#define PCC_PR_OFFSET	31
#define PCC_PR_MASK		(0x1 << PCC_PR_OFFSET)
#define PCC_CGC_OFFSET	30
#define PCC_CGC_MASK	(0x1 << PCC_CGC_OFFSET)
#define PCC_INUSE_OFFSET	29
#define PCC_INUSE_MASK		(0x1 << PCC_INUSE_OFFSET)
#define PCC_PCS_OFFSET	24
#define PCC_PCS_MASK	(0x7 << PCC_PCS_OFFSET)
#define PCC_FRAC_OFFSET	3
#define PCC_FRAC_MASK	(0x1 << PCC_FRAC_OFFSET)
#define PCC_PCD_OFFSET	0
#define PCC_PCD_MASK	(0x7 << PCC_PCD_OFFSET)

enum pcc_clksrc_type {
	CLKSRC_PER_PLAT = 0,
	CLKSRC_PER_BUS = 1,
	CLKSRC_NO_PCS = 2,
};

enum pcc_div_type {
	PCC_HAS_DIV,
	PCC_NO_DIV,
};

enum pcc_rst_b {
	PCC_HAS_RST_B,
	PCC_NO_RST_B,
};

/* This structure keeps info for each pcc slot */
struct pcc_entry {
	u32 pcc_base;
	u32 pcc_slot;
	enum pcc_clksrc_type clksrc;
	enum pcc_div_type div;
	enum pcc_rst_b rst_b;
};

int pcc_clock_enable(int pcc_controller, int pcc_clk_slot, bool enable);
int pcc_clock_sel(int pcc_controller, int pcc_clk_slot, enum cgc_clk src);
int pcc_clock_div_config(int pcc_controller, int pcc_clk_slot, bool frac, u8 div);
bool pcc_clock_is_enable(int pcc_controller, int pcc_clk_slot);
int pcc_clock_get_clksrc(int pcc_controller, int pcc_clk_slot, enum cgc_clk *src);
int pcc_reset_peripheral(int pcc_controller, int pcc_clk_slot, bool reset);
u32 pcc_clock_get_rate(int pcc_controller, int pcc_clk_slot);
#endif
