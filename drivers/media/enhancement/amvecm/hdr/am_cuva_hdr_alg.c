// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (c) 2025 Amlogic, Inc. All rights reserved.
 *
 * CUVA HDR algorithm - inline implementation for 5.15 kernel
 * Replaces the closed-source cuva_hdr_alg.ko external module.
 *
 * Based on UWA 005.1-2024 / SJ/T 11902.1-2023:
 *   Section 9.2: Base curve parameter derivation (Formulas 15-29)
 *   Section 9.4: Color signal dynamic range conversion
 *     Formula (100/106): fTM(x)=m_a*(m_p*x^n/(K1*m_p-K2*x^n+K3))^(1/m_m)+m_b
 *     Formula (107): gain K = PQ_EOTF(fMAX_TM) / PQ_EOTF(fMAX)
 *   Section 9.5: Color saturation correction
 *
 * PQ_EOTF per UWA Formula (12):
 *   L = max((V^(1/m1)-c1)/(c2-c3*V^(1/m1)), 0)^(1/m2)
 *   m1 = 2610/16384, m2 = 2523/4096*128
 *   c1 = 3424/4096, c2 = 2413/4096*32, c3 = 2392/4096*32
 *
 * Gain computed per Formula (107) in linear domain via PQ_EOTF LUT.
 * Curve params scaled per UWA 7.4.9-7.4.13: m_m, m_n use /10 factor.
 * Fractional-power handled as x^(num/den)=iroot_q12(ipow_q12(x,num),den).
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

#define CUVA_ALG_VER "cuva-hw-2025-06 uwa005.1 v3"

#define GAIN_ONE 512
#define CGAIN_ONE 1024
#define PQ_MAX 4095

#define Q24_ONE (1LL << 24)

static s64 pq_eotf_lut[PQ_MAX + 1];
static int eotf_lut_ready;

static int cuva_alg_dbg;
module_param(cuva_alg_dbg, int, 0664);
MODULE_PARM_DESC(cuva_alg_dbg, "\n cuva_alg_dbg\n");

#define alg_dbg(fmt, args...) \
	do { \
		if (cuva_alg_dbg) \
			pr_info("CUVA_ALG: " fmt, ##args); \
	} while (0)

static void build_pq_eotf_lut(void)
{
	int v;

	static const u32 anchors_code[117] = {
		0, 3900, 3908, 3916, 3924, 3932, 3940, 3948,
		3956, 3964, 3972, 3976, 3977, 3978, 3979, 3980,
		3981, 3982, 3983, 3984, 3985, 3986, 3987, 3988,
		3989, 3990, 3991, 3992, 3993, 3994, 3995, 3996,
		3997, 3998, 3999, 4000, 4001, 4002, 4003, 4004,
		4005, 4006, 4007, 4008, 4009, 4010, 4011, 4012,
		4014, 4016, 4018, 4020, 4022, 4024, 4026, 4028,
		4030, 4032, 4034, 4036, 4038, 4040, 4042, 4044,
		4046, 4048, 4050, 4052, 4054, 4056, 4058, 4060,
		4062, 4064, 4066, 4068, 4070, 4072, 4074, 4076,
		4078, 4080, 4082, 4084, 4086, 4088, 4090, 4092,
		4094, 4095
	};

	static const s64 anchors_q24[117] = {
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 14929875,
		15240897, 15356751, 15430267, 15484573, 15527849, 15563958, 15595031, 15622373,
		15646839, 15669021, 15689347, 15708134, 15725627, 15742015, 15757452, 15772058,
		15785937, 15799172, 15811833, 15823982, 15835669, 15846940, 15857832, 15868381,
		15878614, 15888561, 15898242, 15907681, 15916896, 15925904, 15934721, 15943360,
		15960158, 15976389, 15992131, 16007451, 16022409, 16037055, 16051438, 16065597,
		16079573, 16093401, 16107114, 16120744, 16134323, 16147880, 16161445, 16175048,
		16188719, 16202489, 16216390, 16230457, 16244725, 16259235, 16274030, 16289156,
		16304668, 16320625, 16337098, 16354165, 16371922, 16390479, 16409972, 16430567,
		16452469, 16475942, 16501326, 16529079, 16559834, 16594500, 16634458, 16681958,
		16741034, 16777216
	};

	for (v = 0; v <= PQ_MAX; v++) {
		int lo = 0, hi = 116, mid;
		s64 lval;

		if (anchors_code[0] >= v) {
			pq_eotf_lut[v] = 0;
			continue;
		}
		if (anchors_code[116] <= v) {
			pq_eotf_lut[v] = anchors_q24[116];
			continue;
		}
		while (lo < hi - 1) {
			mid = (lo + hi) / 2;
			if (anchors_code[mid] <= v)
				lo = mid;
			else
				hi = mid;
		}
		if (v == anchors_code[lo])
			lval = anchors_q24[lo];
		else {
			s64 range_v = anchors_code[lo + 1] - anchors_code[lo];
			s64 range_l = anchors_q24[lo + 1] - anchors_q24[lo];
			if (range_v > 0)
				lval = anchors_q24[lo] +
					range_l * (v - anchors_code[lo]) / range_v;
			else
				lval = anchors_q24[lo];
		}
		pq_eotf_lut[v] = lval;
	}
	eotf_lut_ready = 1;
	alg_dbg("PQ_EOTF built: [3900]=%lld [3980]=%lld [4095]=%lld\n",
		pq_eotf_lut[3900], pq_eotf_lut[3980], pq_eotf_lut[4095]);
}

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

static s64 ipow_q12(s64 x, int n)
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

static s64 iroot_q12(s64 x, int n)
{
	int lo, hi, mid, i;
	s64 mid_pow;

	if (x <= 0 || n <= 0)
		return 0;
	if (n == 1)
		return x;
	lo = 0;
	hi = 4095;
	while (lo < hi) {
		mid = (lo + hi + 1) >> 1;
		mid_pow = mid;
		for (i = 1; i < n; i++)
			mid_pow = mid_pow * mid / 4095;
		if (mid_pow <= x)
			lo = mid;
		else
			hi = mid - 1;
	}
	return lo;
}

static s64 ipow_fract_q12(s64 x, int num, int den)
{
	s64 ip, ir;

	if (num == den)
		return x;
	if (num <= 0)
		return 0;
	ip = ipow_q12(x, num);
	ir = iroot_q12((int)ip, den);
	return ir;
}

static int cuva_base_curve(int x, int m_p, int m_m_raw, int m_a,
			   int m_b, int m_n_raw, int k1, int k2, int k3)
{
	s64 xn, denom, inner;
	int root;

	if (x <= 0)
		return m_b;

	if (m_n_raw != 10)
		xn = ipow_fract_q12(x, m_n_raw, 10);
	else
		xn = x;

	denom = (s64)k1 * m_p - (s64)k2 * xn + (s64)k3 * 4095;
	if (denom <= 0)
		denom = 1;

	inner = m_p * xn / denom;
	inner = clamp_s64(inner, 0, 4095);

	root = (int)ipow_fract_q12((int)inner, 10, m_m_raw);

	return clamp_int((int)((s64)m_a * root / 4095) + m_b, 0, 4095);
}

static void gen_ogain_from_curve(s64 *ogain, int m_p, int m_m_raw, int m_a,
				 int m_b, int m_n_raw, int k1, int k2, int k3,
				 int max_panel_e, int itp)
{
	int i;

	if (!eotf_lut_ready)
		build_pq_eotf_lut();

	for (i = 0; i < HDR2_OOTF_LUT_SIZE; i++) {
		int x, y;
		s64 lx, ly;

		if (i == 0) {
			ogain[0] = GAIN_ONE;
			continue;
		}

		x = i * PQ_MAX / (HDR2_OOTF_LUT_SIZE - 1);
		y = cuva_base_curve(x, m_p, m_m_raw, m_a, m_b, m_n_raw, k1, k2, k3);

		x = clamp_int(x, 0, PQ_MAX);
		y = clamp_int(y, 0, PQ_MAX);

		lx = pq_eotf_lut[x];
		ly = pq_eotf_lut[y];

		if (lx > 0 && ly > 0)
			ogain[i] = clamp_s64(ly * GAIN_ONE / lx, 8, 4095);
		else if (x > 0 && y > 0)
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
				int *m_p, int *m_m_raw, int *m_a,
				int *m_b, int *m_n_raw,
				int *k1, int *k2, int *k3)
{
	int w = 0;

	if (md && md->tm_enable_mode_flag && md->base_en_flag[w]) {
		*m_p = (int)((s64)md->base_param_m_p[w] * 4095 * 10 / 16383);
		*m_m_raw = max(md->base_param_m_m[w], 1);
		*m_a = md->base_param_m_a[w] * 4095 / 1023;
		*m_b = (int)((s64)md->base_param_m_b[w] * 25 * 4095 / 1023 / 100);
		*m_n_raw = max(md->base_param_m_n[w], 1);
		*k1 = max(md->base_param_m_k1[w], 1);
		*k2 = max(md->base_param_m_k2[w], 0);
		if (md->base_param_m_k3[w] == 2)
			*k3 = md->maximum_maxrgb_pq;
		else
			*k3 = md->base_param_m_k3[w] > 0 ? md->base_param_m_k3[w] : 1;
	} else {
		switch (itp) {
		case CUVA_HDR2SDR:
		case CUVA_HLG2SDR:
			*m_p = 14333;
			*m_m_raw = 24;
			*m_a = 4095;
			*m_b = 0;
			*m_n_raw = 10;
			*k1 = 1;
			*k2 = 1;
			*k3 = 1;
			break;
		case CUVA_HDR2HDR10:
		case CUVA_HLG2HDR10:
			*m_p = 14333;
			*m_m_raw = 24;
			*m_a = 4095;
			*m_b = 0;
			*m_n_raw = 10;
			*k1 = 1;
			*k2 = 1;
			*k3 = 1;
			break;
		default:
			*m_p = 14333;
			*m_m_raw = 24;
			*m_a = 4095;
			*m_b = 0;
			*m_n_raw = 10;
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
	int m_p, m_m_raw, m_a, m_b, m_n_raw, k1, k2, k3;
	int has_valid_md = 0;

	if (!aml_cuva_data || !aml_cuva_data->aml_vm_regs) {
		pr_err("cuva_hdr_alg: null data\n");
		return;
	}

	reg = aml_cuva_data->cuva_reg;
	itp = reg ? reg->itp : 0;
	md = aml_cuva_data->cuva_md;

	alg_dbg("mode=%d, max_panel_e=%d\n", itp, aml_cuva_data->max_panel_e);

	if (md && md->system_start_code >= 0x01 &&
	    md->system_start_code <= 0x07 && md->tm_enable_mode_flag)
		has_valid_md = 1;

	if (has_valid_md) {
		derive_curve_params(md, aml_cuva_data->max_panel_e, itp,
				    &m_p, &m_m_raw, &m_a, &m_b, &m_n_raw,
				    &k1, &k2, &k3);

		alg_dbg("params m_p=%d m_m_raw=%d m_a=%d m_b=%d m_n_raw=%d k1=%d k2=%d k3=%d\n",
			m_p, m_m_raw, m_a, m_b, m_n_raw, k1, k2, k3);

		gen_ogain_from_curve(aml_cuva_data->aml_vm_regs->ogain_lut,
				     m_p, m_m_raw, m_a, m_b, m_n_raw, k1, k2, k3,
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