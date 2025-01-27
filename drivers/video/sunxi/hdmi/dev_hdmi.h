/*
 * Copyright (C) 2007-2012 Allwinner Technology Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __DEV_DISPLAY_H__
#define __DEV_DISPLAY_H__

#include "drv_hdmi_i.h"

typedef struct {
	__bool bopen;
	__disp_tv_mode_t mode;
	__u32 base_hdmi;
} hdmi_info_t;

extern hdmi_info_t ghdmi;

extern struct class *hdmi_class;

#endif
