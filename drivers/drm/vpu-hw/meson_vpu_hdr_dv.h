/* SPDX-License-Identifier: (GPL-2.0+ OR MIT) */
/*
 * Copyright (c) 2019 Amlogic, Inc. All rights reserved.
 */

#ifndef _MESON_VPU_HDR_DV_H_
#define _MESON_VPU_HDR_DV_H_

#define VPP_OSD1_IN_SIZE                           0x1df1
#define VPP_OSD2_IN_SIZE                           0x1df3

#define T7_HDR2_IN_SIZE                           0x1a5c
#define T7_CORE2C_DMA_CTRL                         0x6139
#define T7_CORE2C_DMA_PORT                         0x613f
#define S5_CORE2C_DMA_CTRL     0x0d39
#define S5_CORE2C_DMA_PORT     0x0d3f

enum hdr_index {
	HDR1_INDEX,
	HDR2_INDEX,
	S5_HDR2_INDEX
};

struct hdr_reg_s {
	u32 vpp_osd_in_size;
};
#endif
