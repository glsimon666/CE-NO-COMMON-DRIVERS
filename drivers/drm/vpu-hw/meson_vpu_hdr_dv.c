// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (c) 2019 Amlogic, Inc. All rights reserved.
 */

#include "meson_vpu_pipeline.h"
#ifdef CONFIG_AMLOGIC_MEDIA_ENHANCEMENT_DOLBYVISION
#include <linux/amlogic/media/amdolbyvision/dolby_vision.h>
#endif

#ifdef CONFIG_AMLOGIC_MEDIA_ENHANCEMENT_DOLBYVISION
static bool core2c_update_lut = true;
/*module_param(core2c_update_lut, bool, 0664);*/
/*MODULE_PARM_DESC(core2c_update_lut, "core2c_update_lut");*/
#endif

static struct hdr_reg_s osd_hdr_reg[MESON_MAX_HDRS] = {
	{
		VPP_OSD1_IN_SIZE,
	},
	{
		VPP_OSD2_IN_SIZE,
	}
};

static struct hdr_reg_s t7_osd_hdr_reg[MESON_MAX_HDRS] = {
	{
		VPP_OSD1_IN_SIZE,//TO check
	},
	{
		T7_HDR2_IN_SIZE,
	}
};

static int hdr_check_state(struct meson_vpu_block *vblk,
			   struct meson_vpu_block_state *state,
		struct meson_vpu_pipeline_state *mvps)
{
	struct meson_vpu_hdr *hdr = to_hdr_block(vblk);

	if (state->checked)
		return 0;

	state->checked = true;

	//vpu_block_check_input(vblk, state, mvps);

	MESON_DRM_BLOCK("%s check_state called.\n", hdr->base.name);
	return 0;
}

static void hdr_set_state(struct meson_vpu_block *vblk,
			  struct meson_vpu_block_state *state,
			  struct meson_vpu_block_state *old_state)
{
	struct meson_vpu_hdr *hdr = to_hdr_block(vblk);
	//struct meson_vpu_hdr_state *hdr_state = to_hdr_state(state);

	MESON_DRM_BLOCK("%s set_state called.\n", hdr->base.name);
}

static void s7d_hdr_set_state(struct meson_vpu_block *vblk,
			  struct meson_vpu_block_state *state,
			  struct meson_vpu_block_state *old_state)
{
	struct meson_vpu_hdr *hdr = to_hdr_block(vblk);
	struct meson_vpu_pipeline *pipeline = hdr->base.pipeline;
	struct hdr_reg_s *reg = hdr->reg;
	struct rdma_reg_ops *reg_ops = state->sub->reg_ops;
	struct meson_vpu_pipeline_state *mvps;
	u32 hsize, vsize;

	mvps = priv_to_pipeline_state(pipeline->obj.state);

	hsize = mvps->plane_info[vblk->index].src_w +
		mvps->plane_info[vblk->index].src_w_offset4hdr;
	vsize = mvps->plane_info[vblk->index].src_h;

	reg_ops->rdma_write_reg(reg->vpp_osd_in_size, hsize | (vsize << 16));
	MESON_DRM_BLOCK("%s set_state called.\n", hdr->base.name);
}

static void hdr_dump_register(struct drm_printer *p, struct meson_vpu_block *vblk)
{
	int hdr_index;
	u32 value, reg_addr;
	char buff[16];
	struct meson_vpu_hdr *hdr;
	struct hdr_reg_s *reg;

	hdr_index = vblk->index;
	hdr = to_hdr_block(vblk);
	reg = hdr->reg;

	snprintf(buff, 16, "VPP_OSD%d", hdr_index + 1);

	reg_addr = reg->vpp_osd_in_size;
	value = meson_drm_read_reg(reg_addr);
	drm_printf(p, "%s_%-35s\taddr: 0x%04X\tvalue: 0x%08X\n", buff,
		"_IN_SIZE", reg_addr, value);
}

static void t7_hdr_set_state(struct meson_vpu_block *vblk,
			  struct meson_vpu_block_state *state,
			  struct meson_vpu_block_state *old_state)
{
	struct meson_vpu_hdr *hdr = to_hdr_block(vblk);
	struct meson_vpu_pipeline *pipeline = hdr->base.pipeline;
	struct hdr_reg_s *reg = hdr->reg;
	struct rdma_reg_ops *reg_ops = state->sub->reg_ops;
	struct meson_vpu_pipeline_state *mvps, *old_mvps;
	u32 hsize, vsize;
#ifdef CONFIG_AMLOGIC_MEDIA_ENHANCEMENT_DOLBYVISION
	u32 *p_core2_lut = NULL;
	u32 lut_count = 256 * 5;
	u32 i;
#endif

	mvps = priv_to_pipeline_state(pipeline->obj.state);

	if (vblk->index == HDR2_INDEX) {
		hsize = mvps->scaler_param[MESON_OSD3].output_width;
		vsize = mvps->scaler_param[MESON_OSD3].output_height;

		MESON_DRM_BLOCK("%s set_state,input size:%u,%u.\n", hdr->base.name, hsize, vsize);
		reg_ops->rdma_write_reg(reg->vpp_osd_in_size, hsize | (vsize << 16));

		old_mvps = meson_vpu_pipeline_get_old_state(pipeline, old_state->obj.state);
		if (old_mvps && mvps->plane_info[MESON_OSD3].enable &&
		    !old_mvps->plane_info[MESON_OSD3].enable) {
#ifdef CONFIG_AMLOGIC_MEDIA_ENHANCEMENT_DOLBYVISION
			p_core2_lut = get_core2_lut();
			MESON_DRM_BLOCK("p_core2_lut %px\n", p_core2_lut);

			if (p_core2_lut && is_amdv_enable() && core2c_update_lut) {
				/* bit3=1, disable latch (same as S5) */
				reg_ops->rdma_write_reg(T7_CORE2C_DMA_CTRL, 0x1409);

				for (i = 0; i < lut_count; i += 4) {
					reg_ops->rdma_write_reg(T7_CORE2C_DMA_PORT,
						p_core2_lut[i + 3]);
					reg_ops->rdma_write_reg(T7_CORE2C_DMA_PORT,
						p_core2_lut[i + 2]);
					reg_ops->rdma_write_reg(T7_CORE2C_DMA_PORT,
						p_core2_lut[i + 1]);
					reg_ops->rdma_write_reg(T7_CORE2C_DMA_PORT,
						p_core2_lut[i]);
				}
			}
#endif
		}
	}
	MESON_DRM_BLOCK("%s set_state called.\n", hdr->base.name);
}

static void s5_hdr_set_state(struct meson_vpu_block *vblk,
			  struct meson_vpu_block_state *state,
			  struct meson_vpu_block_state *old_state)
{
	struct meson_vpu_hdr *hdr = to_hdr_block(vblk);
	struct meson_vpu_pipeline *pipeline = hdr->base.pipeline;
	//struct hdr_reg_s *reg = hdr->reg;
	struct meson_vpu_pipeline_state *mvps, *old_mvps;
	u32 hsize, vsize;
#ifdef CONFIG_AMLOGIC_MEDIA_ENHANCEMENT_DOLBYVISION
	struct rdma_reg_ops *reg_ops = state->sub->reg_ops;
	u32 *p_core2_lut = NULL;
	u32 lut_count = 256 * 5;
	u32 i;
#endif

	mvps = priv_to_pipeline_state(pipeline->obj.state);
	old_mvps = meson_vpu_pipeline_get_old_state(pipeline, old_state->obj.state);
	if (!old_mvps)
		return;

	if (vblk->index == HDR1_INDEX) {
		//TODO
	} else if (vblk->index == S5_HDR2_INDEX) {
		if (mvps->plane_info[MESON_OSD3].enable &&
			!old_mvps->plane_info[MESON_OSD3].enable) {
			MESON_DRM_BLOCK("state changed\n");

			hsize = mvps->plane_info[MESON_OSD3].src_w;
			vsize = mvps->plane_info[MESON_OSD3].src_h;
			MESON_DRM_BLOCK("%s set_state,input size:%u,%u.\n",
				hdr->base.name, hsize, vsize);
#ifdef CONFIG_AMLOGIC_MEDIA_ENHANCEMENT_DOLBYVISION
			p_core2_lut =  get_core2_lut();
			MESON_DRM_BLOCK("p_core2_lut %px\n", p_core2_lut);

			if (p_core2_lut && is_amdv_enable() && core2c_update_lut) {
				/*bit3=1,disable latch*/
				reg_ops->rdma_write_reg(S5_CORE2C_DMA_CTRL, 0x1409);

				for (i = 0; i < lut_count; i += 4) {
					reg_ops->rdma_write_reg(S5_CORE2C_DMA_PORT,
						p_core2_lut[i + 3]);
					reg_ops->rdma_write_reg(S5_CORE2C_DMA_PORT,
						p_core2_lut[i + 2]);
					reg_ops->rdma_write_reg(S5_CORE2C_DMA_PORT,
						p_core2_lut[i + 1]);
					reg_ops->rdma_write_reg(S5_CORE2C_DMA_PORT,
						p_core2_lut[i]);
				}
			}
#endif
		}
	}
	MESON_DRM_BLOCK("%s set_state called.\n", hdr->base.name);
}

static void hdr_enable(struct meson_vpu_block *vblk,
		       struct meson_vpu_block_state *state)
{
	struct meson_vpu_hdr *hdr = to_hdr_block(vblk);

	MESON_DRM_BLOCK("%s enable called.\n", hdr->base.name);
}

static void hdr_disable(struct meson_vpu_block *vblk,
			struct meson_vpu_block_state *state)
{
	struct meson_vpu_hdr *hdr = to_hdr_block(vblk);

	MESON_DRM_BLOCK("%s disable called.\n", hdr->base.name);
}

static void hdr_init(struct meson_vpu_block *vblk)
{
	struct meson_vpu_hdr *hdr = to_hdr_block(vblk);

	hdr->reg = &osd_hdr_reg[vblk->index];
	MESON_DRM_BLOCK("%s hw_init called.\n", hdr->base.name);
}

static void t7_hdr_init(struct meson_vpu_block *vblk)
{
	struct meson_vpu_hdr *hdr = to_hdr_block(vblk);

	hdr->reg = &t7_osd_hdr_reg[vblk->index];
	MESON_DRM_BLOCK("%s hw_init called.\n", hdr->base.name);
}

struct meson_vpu_block_ops hdr_ops = {
	.check_state = hdr_check_state,
	.update_state = hdr_set_state,
	.enable = hdr_enable,
	.disable = hdr_disable,
};

struct meson_vpu_block_ops s7d_hdr_ops = {
	.check_state = hdr_check_state,
	.update_state = s7d_hdr_set_state,
	.dump_register = hdr_dump_register,
	.enable = hdr_enable,
	.disable = hdr_disable,
	.init = hdr_init,
};

struct meson_vpu_block_ops t7_hdr_ops = {
	.check_state = hdr_check_state,
	.update_state = t7_hdr_set_state,
	.enable = hdr_enable,
	.disable = hdr_disable,
	.init = t7_hdr_init,
};

struct meson_vpu_block_ops s5_hdr_ops = {
	.check_state = hdr_check_state,
	.update_state = s5_hdr_set_state,
	.enable = hdr_enable,
	.disable = hdr_disable,
	.init = hdr_init,
};

static int db_check_state(struct meson_vpu_block *vblk,
			     struct meson_vpu_block_state *state,
		struct meson_vpu_pipeline_state *mvps)
{
	struct meson_vpu_db *mvd = to_db_block(vblk);

	if (state->checked)
		return 0;

	state->checked = true;

	//vpu_block_check_input(vblk, state, mvps);

	MESON_DRM_BLOCK("%s check_state called.\n", mvd->base.name);
	return 0;
}

static void db_set_state(struct meson_vpu_block *vblk,
			    struct meson_vpu_block_state *state,
			    struct meson_vpu_block_state *old_state)
{
	struct meson_vpu_db *mvd = to_db_block(vblk);

	MESON_DRM_BLOCK("%s set_state called.\n", mvd->base.name);
}

static void db_enable(struct meson_vpu_block *vblk,
			 struct meson_vpu_block_state *state)
{
	struct meson_vpu_db *mvd = to_db_block(vblk);

	MESON_DRM_BLOCK("%s enable called.\n", mvd->base.name);
}

static void db_disable(struct meson_vpu_block *vblk,
			  struct meson_vpu_block_state *state)
{
	struct meson_vpu_db *mvd = to_db_block(vblk);

	MESON_DRM_BLOCK("%s disable called.\n", mvd->base.name);
}

struct meson_vpu_block_ops db_ops = {
	.check_state = db_check_state,
	.update_state = db_set_state,
	.enable = db_enable,
	.disable = db_disable,
};
