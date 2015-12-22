/*
 * Copyright (C) 2007-2012 Allwinner Technology Co., Ltd.
 * Copyright (C) 2015 Joachim Damm
 *.
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.> See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __HDMI_CEC__
#define __HDMI_CEC__


#define DEVICE_NAME "CEC"
#define CLASS_NAME "cec"
 
#define AUTHOR "Joachim Damm <dammj@gmx.de>"
#define DESCRIPTION "CEC device driver for sunxi"
#define VERSION "0.1"
 
/* We'll use our own macros for printk */
#define dbg(format, arg...) do { if (debug) pr_info(CLASS_NAME ": %s: " format, __FUNCTION__, ## arg); } while (0)
#define err(format, arg...) pr_err(CLASS_NAME ": " format, ## arg)
#define info(format, arg...) pr_info(CLASS_NAME ": " format, ## arg)
#define warn(format, arg...) pr_warn(CLASS_NAME ": " format, ## arg)

/* time us */
#define HDMI_CEC_START_BIT_LOW_TIME 3700
#define HDMI_CEC_START_BIT_WHOLE_TIME 4500
#define HDMI_CEC_DATA_BIT0_LOW_TIME 1500
#define HDMI_CEC_DATA_BIT1_LOW_TIME  600
#define HDMI_CEC_DATA_BIT_WHOLE_TIME 2400
/* ack */
#define HDMI_CEC_NORMAL_MSG_ACK 0X0
#define HDMI_CEC_BROADCAST_MSG_ACK 0X1

#define SIG_LO 0X0
#define SIG_HI 0X1

enum __hdmi_cec_logical_address {
    HDMI_CEC_LADDR_TV,
    HDMI_CEC_LADDR_RECORDER1,
    HDMI_CEC_LADDR_RECORDER2,
    HDMI_CEC_LADDR_TUNER1,
    HDMI_CEC_LADDR_PLAYER1,
    HDMI_CEC_LADDR_AUDIO,
    HDMI_CEC_LADDR_TUNER2,
    HDMI_CEC_LADDR_TUNER3,
    HDMI_CEC_LADDR_PLAYER2,
    HDMI_CEC_LADDR_RECORDER3,
    HDMI_CEC_LADDR_TUNER4,
    HDMI_CEC_LADDR_PLAYER3,
    HDMI_CEC_LADDR_RESERVED1,
    HDMI_CEC_LADDR_RESERVED2,
    HDMI_CEC_LADDR_SPECIFIC,
    HDMI_CEC_LADDR_BROADCAST,
};

extern unsigned int cec_phy_addr;

int sunxi_cec_init(void);
void sunxi_cec_exit(void);

#endif
