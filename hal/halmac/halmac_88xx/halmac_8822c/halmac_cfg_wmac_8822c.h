/******************************************************************************
 *
 * Copyright(c) 2017 - 2021 Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 *****************************************************************************/

#ifndef _HALMAC_CFG_WMAC_8822C_H_
#define _HALMAC_CFG_WMAC_8822C_H_

#include "../../halmac_api.h"

#if HALMAC_8822C_SUPPORT

enum halmac_ret_status
cfg_drv_info_8822c(struct halmac_adapter *adapter,
		   enum halmac_drv_info drv_info);

enum halmac_ret_status
init_low_pwr_8822c(struct halmac_adapter *adapter);

void
cfg_rxgck_fifo_8822c(struct halmac_adapter *adapter, u8 enable);

void
cfg_rx_ignore_8822c(struct halmac_adapter *adapter,
		    struct halmac_mac_rx_ignore_cfg *cfg);

void
cfg_ampdu_8822c(struct halmac_adapter *adapter,
		struct halmac_ampdu_config *cfg);

#endif/* HALMAC_8822C_SUPPORT */

#endif/* _HALMAC_CFG_WMAC_8822C_H_ */
