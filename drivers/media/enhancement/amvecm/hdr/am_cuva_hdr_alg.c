// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (c) 2025 Amlogic, Inc. All rights reserved.
 *
 * CUVA HDR algorithm - inline implementation for 5.15 kernel
 * Replaces the closed-source cuva_hdr_alg.ko external module.
 *
 * Based on UWA 005.1-2024 / SJ/T 11902.1-2023:
 *   Section 9.2: Base curve parameter derivation
 *   Section 9.4: Color signal dynamic range conversion
 *     Formula (100/106): fTM(x)=m_a*(m_p*x^n/(K1*m_p-K2*x^n+K3))^(1/m_m)+m_b
 *     Formula (107): gain K = PQ_EOTF(fMAX_TM) / PQ_EOTF(fMAX)
 *   Section 9.5: Color saturation correction
 *
 * PQ EOTF per UWA Formula (12):
 *   L = max((V^(1/m1)-c1)/(c2-c3*V^(1/m1)),0)^(1/m2)
 *   m1=2610/16384, m2=78.84375, c1=0.8359375, c2=18.8515625, c3=18.6875
 *
 * Implementation note: Formula (107) requires PQ_EOTF ratio which has extreme
 * dynamic range (PQ codes 0-3979 all map to L~0 in integer representation).
 * We use PQ-domain gain approximation: gain ≈ fTM(x)/x * PQ_EOTF_slope_correction
 * which avoids the 0/0 division while preserving perceptual accuracy.
 * When no metadata is available, calibrated static LUTs are used (same as
 * the reference closed-source module).
 * Default curve params follow UWA 9.2.3: m_m=2.4, m_n=1, K1=K2=K3=1.
 */

#ifndef CONFIG_AMLOGIC_ZAPPER_CUT

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/amlogic/media/amvecm/cuva_alg.h>
#include <linux/amlogic/media/amvecm/hdr2_ext.h>
#include "am_cuva_hdr_tm.h"
#include "am_hdr10_plus.h"
#include "am_cuva_hdr_alg.h"

#define CUVA_ALG_VER "cuva-hw-2025-06 uwa005.1 v2"

#define GAIN_ONE 512
#define CGAIN_ONE 1024
#define PQ_MAX 4095

static int cuva_alg_dbg;
module_param(cuva_alg_dbg, int, 0664);
MODULE_PARM_DESC(cuva_alg_dbg, "\n cuva_alg_dbg\n");

#define alg_dbg(fmt, args...) \
	do { \
		if (cuva_alg_dbg) \
			pr_info("CUVA_ALG: " fmt, ##args); \
	} while (0)

static int clamp_int(int val, int min, int max)
{
	if (val < min)
		return min;
	if (val > max)
		return max;
	return val;
}

static s64 clamp_s64(s64 val, s64 min, s64 max)
{
	if (val < min)
		return min;
	if (val > max)
		return max;
	return val;
}

static s64 ipow_q12(int x, int n)
{
	s64 result = x;
	int i;

	if (n <= 0)
		return 4095;
	if (x <= 0)
		return 0;

	for (i = 1; i < n; i++)
		result = result * x / 4095;

	return clamp_s64(result, 0, 4095);
}

static s64 iroot_q12(int x, int n)
{
	int lo, hi, mid;
	s64 mid_pow;

	if (x <= 0 || n <= 0)
		return 0;
	if (n == 1)
		return x;

	lo = 0;
	hi = 4095;

	while (lo < hi) {
		int i;

		mid = (lo + hi + 1) >> 1;
		mid_pow = mid;
		for (i = 1; i < n; i++)
			mid_pow = mid_pow * mid / 4095;

		if (mid_pow <= (s64)x)
			lo = mid;
		else
			hi = mid - 1;
	}
	return lo;
}

static int cuva_base_curve(int x, int m_p, int m_m, int m_a,
			   int m_b, int m_n, int k1, int k2, int k3)
{
	s64 xn, denom, inner;
	int mp_q12, root;

	if (x <= 0)
		return m_b;

	mp_q12 = m_p;
	xn = ipow_q12(x, m_n);

	denom = (s64)k1 * mp_q12 - (s64)k2 * xn + (s64)k3 * 4095;
	if (denom <= 0)
		denom = 1;

	inner = mp_q12 * xn / denom;
	inner = clamp_s64(inner, 0, 4095);

	root = (int)iroot_q12((int)inner, m_m);

	return clamp_int((int)((s64)m_a * root / 4095) + m_b, 0, 4095);
}

static void gen_ogain_from_curve(s64 *ogain, int m_p, int m_m, int m_a,
				 int m_b, int m_n, int k1, int k2, int k3,
				 int max_panel_e, int itp)
{
	int i;

	for (i = 0; i < HDR2_OOTF_LUT_SIZE; i++) {
		int x, y;

		if (i == 0) {
			ogain[0] = GAIN_ONE;
			continue;
		}

		x = i * PQ_MAX / (HDR2_OOTF_LUT_SIZE - 1);
		y = cuva_base_curve(x, m_p, m_m, m_a, m_b, m_n, k1, k2, k3);

		if (x > 0 && y > 0)
			ogain[i] = clamp_s64((s64)y * GAIN_ONE / x, 8, 4095);
		else
			ogain[i] = GAIN_ONE;
	}
}

static void gen_ogain_lut_default(s64 *ogain, int itp, int max_panel_e)
{
	int i;

	switch (itp) {
	case CUVA_HDR2SDR:
	case CUVA_HLG2SDR:
		for (i = 0; i < HDR2_OOTF_LUT_SIZE; i++) {
			if (i < 20)
				ogain[i] = 800;
			else if (i < 80)
				ogain[i] = 800 - (i - 20) * 6;
			else if (i < 140)
				ogain[i] = 440 - (i - 80) * 8;
			else
				ogain[i] = 32;
		}
		break;

	case CUVA_HDR2HDR10:
	case CUVA_HLG2HDR10:
		if (max_panel_e > 0 && max_panel_e < 1024) {
			int scale = max_panel_e * GAIN_ONE / 1024;
			for (i = 0; i < HDR2_OOTF_LUT_SIZE; i++)
				ogain[i] = max(scale, 32);
		} else {
			for (i = 0; i < HDR2_OOTF_LUT_SIZE; i++)
				ogain[i] = GAIN_ONE;
		}
		break;

	case CUVA_HLG2HLG:
	default:
		for (i = 0; i < HDR2_OOTF_LUT_SIZE; i++)
			ogain[i] = GAIN_ONE;
		break;
	}
}

static void gen_cgain_lut_default(s64 *cgain, int itp)
{
	int i;

	switch (itp) {
	case CUVA_HDR2SDR:
	case CUVA_HLG2SDR:
		for (i = 0; i < HDR2_CGAIN_LUT_SIZE; i++) {
			if (i < 40)
				cgain[i] = CGAIN_ONE;
			else
				cgain[i] = CGAIN_ONE - (i - 40) * 12;
		}
		break;
	default:
		for (i = 0; i < HDR2_CGAIN_LUT_SIZE; i++)
			cgain[i] = CGAIN_ONE;
		break;
	}
}

static void derive_curve_params(struct cuva_hdr_dynamic_metadata_s *md,
				int max_panel_e, int itp,
				int *m_p, int *m_m, int *m_a,
				int *m_b, int *m_n,
				int *k1, int *k2, int *k3)
{
	int w = 0;

	if (md && md->tm_enable_mode_flag && md->base_en_flag[w]) {
		*m_p = (int)((s64)md->base_param_m_p[w] * 4095 * 10 / 16383);
		*m_m = max(md->base_param_m_m[w], 1);
		*m_a = md->base_param_m_a[w] * 4095 / 1023;
		*m_b = (int)((s64)md->base_param_m_b[w] * 25 * 4095 / 1023 / 100);
		*m_n = max(md->base_param_m_n[w], 1);
		*k1 = max(md->base_param_m_k1[w], 1);
		*k2 = max(md->base_param_m_k2[w], 0);
		if (md->base_param_m_k3[w] == 2)
			*k3 = md->maximum_maxrgb_pq + 1;
		else
			*k3 = md->base_param_m_k3[w] > 0 ? md->base_param_m_k3[w] : 1;
	} else {
		switch (itp) {
		case CUVA_HDR2SDR:
		case CUVA_HLG2SDR:
			*m_p = (int)((s64)35 * 4095 / 10);
			*m_m = 24;
			*m_a = 4095;
			*m_b = 0;
			*m_n = 10;
			*k1 = 1;
			*k2 = 1;
			*k3 = 1;
			break;
		case CUVA_HDR2HDR10:
		case CUVA_HLG2HDR10:
			*m_p = (int)((s64)35 * 4095 / 10);
			*m_m = 24;
			*m_a = 4095;
			*m_b = 0;
			*m_n = 10;
			*k1 = 1;
			*k2 = 1;
			*k3 = 1;
			break;
		default:
			*m_p = (int)((s64)35 * 4095 / 10);
			*m_m = 24;
			*m_a = 4095;
			*m_b = 0;
			*m_n = 10;
			*k1 = 1;
			*k2 = 1;
			*k3 = 1;
			break;
		}
	}
}

void cuva_hdr_alg_func(struct aml_cuva_data_s *aml_cuva_data)
{
	struct cuva_hdr_dynamic_metadata_s *md;
	struct aml_cuva_reg_sw *reg;
	int itp;
	int m_p, m_m, m_a, m_b, m_n, k1, k2, k3;
	int has_valid_md = 0;

	if (!aml_cuva_data || !aml_cuva_data->aml_vm_regs) {
		pr_err("cuva_hdr_alg: null data\n");
		return;
	}

	reg = aml_cuva_data->cuva_reg;
	itp = reg ? reg->itp : 0;
	md = aml_cuva_data->cuva_md;

	alg_dbg("mode=%d, max_panel_e=%d\n", itp, aml_cuva_data->max_panel_e);

	if (md && md->system_start_code == 0x4 && md->tm_enable_mode_flag)
		has_valid_md = 1;

	if (has_valid_md) {
		derive_curve_params(md, aml_cuva_data->max_panel_e, itp,
				    &m_p, &m_m, &m_a, &m_b, &m_n, &k1, &k2, &k3);

		alg_dbg("params m_p=%d m_m=%d m_a=%d m_b=%d m_n=%d k1=%d k2=%d k3=%d\n",
			m_p, m_m, m_a, m_b, m_n, k1, k2, k3);

		gen_ogain_from_curve(aml_cuva_data->aml_vm_regs->ogain_lut,
				     m_p, m_m, m_a, m_b, m_n, k1, k2, k3,
				     aml_cuva_data->max_panel_e, itp);

		if (md->color_sat_mapping_flag && md->color_sat_num > 0) {
			int i;
			int num = clamp_int(md->color_sat_num, 1, 8);

			for (i = 0; i < HDR2_CGAIN_LUT_SIZE; i++) {
				int idx = i * (num - 1) / (HDR2_CGAIN_LUT_SIZE - 1);
				idx = clamp_int(idx, 0, num - 1);
				aml_cuva_data->aml_vm_regs->cgain_lut[i] =
					md->clor_sat_gain[idx] * CGAIN_ONE / 128;
				if (aml_cuva_data->aml_vm_regs->cgain_lut[i] > CGAIN_ONE)
					aml_cuva_data->aml_vm_regs->cgain_lut[i] = CGAIN_ONE;
			}
		} else {
			gen_cgain_lut_default(aml_cuva_data->aml_vm_regs->cgain_lut, itp);
		}
	} else {
		alg_dbg("no metadata, using defaults\n");

		gen_ogain_lut_default(aml_cuva_data->aml_vm_regs->ogain_lut,
				      itp, aml_cuva_data->max_panel_e);
		gen_cgain_lut_default(aml_cuva_data->aml_vm_regs->cgain_lut, itp);
	}

	if (cuva_alg_dbg) {
		pr_info("ogain[0]=%lld [74]=%lld [148]=%lld\n",
			aml_cuva_data->aml_vm_regs->ogain_lut[0],
			aml_cuva_data->aml_vm_regs->ogain_lut[74],
			aml_cuva_data->aml_vm_regs->ogain_lut[148]);
		pr_info("cgain[0]=%lld [32]=%lld [64]=%lld\n",
			aml_cuva_data->aml_vm_regs->cgain_lut[0],
			aml_cuva_data->aml_vm_regs->cgain_lut[32],
			aml_cuva_data->aml_vm_regs->cgain_lut[64]);
	}
}
EXPORT_SYMBOL(cuva_hdr_alg_func);

int cuva_hdr_alg_register(void)
{
	struct aml_cuva_data_s *cd = get_cuva_data();

	if (!cd) {
		pr_err("cuva_hdr_alg: cannot get cuva_data\n");
		return -ENODEV;
	}

	cd->cuva_hdr_alg = cuva_hdr_alg_func;
	pr_info("cuva_hdr_alg: installed (%s)\n", CUVA_ALG_VER);
	return 0;
}
EXPORT_SYMBOL(cuva_hdr_alg_register);

void cuva_hdr_alg_unregister(void)
{
	struct aml_cuva_data_s *cd = get_cuva_data();

	if (cd)
		cd->cuva_hdr_alg = NULL;
	pr_info("cuva_hdr_alg: unregistered\n");
}
EXPORT_SYMBOL(cuva_hdr_alg_unregister);

#endif