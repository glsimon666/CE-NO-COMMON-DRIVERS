/* SPDX-License-Identifier: (GPL-2.0+ OR MIT) */
/*
 * Copyright (c) 2025 Amlogic, Inc. All rights reserved.
 */

#ifndef CONFIG_AMLOGIC_ZAPPER_CUT
#ifndef AM_CUVA_HDR_ALG_H
#define AM_CUVA_HDR_ALG_H

#include <linux/amlogic/media/amvecm/cuva_alg.h>

void cuva_hdr_alg_func(struct aml_cuva_data_s *aml_cuva_data);
int cuva_hdr_alg_register(void);
void cuva_hdr_alg_unregister(void);

#endif
#endif
