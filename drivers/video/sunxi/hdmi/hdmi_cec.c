/*
 * Copyright (C) 2007-2012 Allwinner Technology Co., Ltd.
 * Copyright (C) 2015 Joachim Damm
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/kthread.h>
#include <video/sunxi_disp_ioctl.h>
#include <linux/io.h>
#include <linux/delay.h>

#include "hdmi_cec.h"
#include "hdmi_core.h"
#include "dev_hdmi.h"
#include "../disp/sunxi_disp_regs.h"

/* CEC Rx buffer size */
#define CEC_RX_BUFF_SIZE            16
/* CEC Tx buffer size */
#define CEC_TX_BUFF_SIZE            16

#define CEC_IOC_MAGIC        'c'
#define CEC_IOC_SETLADDR     _IOW(CEC_IOC_MAGIC, 0, unsigned int)


/* Device variables */
//static struct class* cec_class = NULL;
static struct device* cec_device = NULL;
static int cec_major;
/* A mutex will ensure that only one process accesses our device */
static DEFINE_MUTEX(cec_device_mutex);

unsigned int cec_phy_addr;

static __u32 cec_logical_addr;

struct task_struct *rxtx_task;
static int count_lo;
static int count_hi;
static int message_to_tx;
static int timeout_rx;
static char tx_buffer[CEC_TX_BUFF_SIZE];
static char rx_buffer[CEC_RX_BUFF_SIZE];
static int tx_size;
static int rx_size;
static int tx_state;
static int rx_state;

wait_queue_head_t waitq_rx, waitq_tx;

/* Module parameters that can be provided on insmod */
static bool debug = false; /* print extra debug info */
module_param(cec_phy_addr, uint, S_IRUGO);
MODULE_PARM_DESC(cec_phy_addr, "cec physical address, obtained from EDID");


static __s32 hdmi_cec_write_reg_bit(__u32 data)    /* 1bit */
{
    if (data & 0x1)
	writel(readl(HDMI_CEC) | 0x200, HDMI_CEC);
    else
	writel(readl(HDMI_CEC) & (~0x200), HDMI_CEC);

    return 0;
}

static __s32 hdmi_cec_read_reg_bit(void)    /* 1bit */
{
    return (readl(HDMI_CEC) >> 8) & 0x1;
}

static __s32 hdmi_cec_wait_for_signal(__s32 signal, __u32 timeout)
{
    __u32 count_timeout = 0;

    while (hdmi_cec_read_reg_bit() != signal)
    {
	count_timeout++;
	usleep_range(50,50);
	// no change in time, timeout
	if (count_timeout > timeout)
	{
	    return -1;
	}
    }
    return 0;
}

static __s32 hdmi_cec_send_ack(__u32 ack)    /* 1bit */
{
    hdmi_cec_write_reg_bit(ack);

    usleep_range(1400, 1400);

    hdmi_cec_write_reg_bit(SIG_HI);

    usleep_range(800, 800);

    return 0;
}

static __s32 hdmi_cec_receive_ack(__u32 ack)    /* 1bit */
{
    __u32 data;

    hdmi_cec_write_reg_bit(SIG_LO);

    usleep_range(400, 400);

    hdmi_cec_write_reg_bit(SIG_HI);

    usleep_range(600, 600);

    data = hdmi_cec_read_reg_bit();
    // wait for end of bit
    usleep_range(1200,1200);

    if (data != ack)
    {
	return -1;
    }
    return 0;
}

static __s32 hdmi_cec_send_bit(__u32 data)    /* 1bit */
{
    __u32 low_time, whole_time;

    low_time = (data == 1) ? 600 : 1400;
    whole_time = 2200;

    hdmi_cec_write_reg_bit(SIG_LO);

    usleep_range(low_time, low_time);

    hdmi_cec_write_reg_bit(SIG_HI);

    if (hdmi_cec_wait_for_signal(SIG_HI, 3))
    {
	warn("could not return to hi state\n");
	return -1;
    }
    usleep_range(whole_time - low_time, whole_time - low_time);

    return 0;
}

static __s32 hdmi_cec_receive_bit(__u32 *data)    /*1bit */
{
    if (hdmi_cec_wait_for_signal(SIG_LO, 40))
    {
	return -1;
    }

    // wait for safe sample period
    usleep_range(1000,1000);
    *data = hdmi_cec_read_reg_bit();

    // wait for falling edge
    usleep_range(1200,1200);

    if (hdmi_cec_wait_for_signal(SIG_LO, 16))
    {
	return -1;
    }
    return 0;
}

static __s32 hdmi_cec_send_startbit(void)
{
//    __u32 low_time, whole_time;

//    low_time = HDMI_CEC_START_BIT_LOW_TIME;
//    whole_time = HDMI_CEC_START_BIT_WHOLE_TIME;

    hdmi_cec_write_reg_bit(SIG_LO);

    usleep_range(3700, 3700);

    hdmi_cec_write_reg_bit(SIG_HI);

    if (hdmi_cec_wait_for_signal(SIG_HI, 3))
    {
	return -1;
    }

    usleep_range(750, 750);

    return 0;
}

static __s32 hdmi_cec_wait_for_startbit(void)
{
    // wait for hi of startbit, if we arrive here signal can be lo for 1600 - 3400 us
    if (hdmi_cec_wait_for_signal(SIG_HI, 46))
    {
	return -1;
    }
    // Wait for falling edge
    usleep_range(700, 700);

    if (hdmi_cec_wait_for_signal(SIG_LO, 9))
    {
	return -1;
    }
    return 0;
}

static __s32 hdmi_cec_send_byte(__u32 data, __u32 eom, __u32 ack)  /* 1byte */
{
    __u32 i;
    __u32 bit;

    for (i = 0; i < 8; i++) {
	bit = (data & 0x80) >> 7;
	data = data << 1;

	if (hdmi_cec_send_bit(bit))
	{
	    return -1;
	}
    }

    if (hdmi_cec_send_bit(eom))
    {
	return -1;
    }

    if (hdmi_cec_receive_ack(ack))
    {
	return -1;
    }

    return 0;
}

static __s32 hdmi_cec_receive_byte(char *data, __u32 *eom)    /* 1byte */
{
    __u32 i;
    __u32 data_bit = 0;
    char data_byte = 0;

    for (i = 0; i < 8; i++) {
	if(hdmi_cec_receive_bit(&data_bit))
	{
	    return -1;
	}
	data_byte = data_byte << 1;
	data_byte |= data_bit;
    }

    *data = data_byte;

    if (hdmi_cec_receive_bit(eom))
    {
	return -1;
    }

    return 0;
}

static __s32 hdmi_cec_send_msg(void)
{
    int follower_addr = 0;
    int i;
    __u32 ack;
    __u32 eom;

    message_to_tx = 0;
    if ((tx_size <= 0) || (tx_size > CEC_TX_BUFF_SIZE))
    {
	return -1;
    }

    follower_addr = tx_buffer[0] & 0x0f;

    // broadcast ack or individual ack
    ack = (follower_addr == HDMI_CEC_LADDR_BROADCAST) ?
	    HDMI_CEC_BROADCAST_MSG_ACK : HDMI_CEC_NORMAL_MSG_ACK;

    hdmi_cec_send_startbit();

    for (i = 0; i < tx_size; i++) 
    {
        eom = ((i+1) == tx_size) ? 1 : 0;
	if (hdmi_cec_send_byte(tx_buffer[i], eom, ack))
	{
	    return -1;
	}
    }
    return 0;
}

static __s32 hdmi_cec_receive_msg(void)
{
    __u32 rx_count = 0;
    __u32 eom;
    __u32 ack;
    __u32 follower_addr = 0;

    if (hdmi_cec_wait_for_startbit())
    {
	return -1;
    }

    if (hdmi_cec_receive_byte(&rx_buffer[0], &eom))
    {
	return -1;
    }

    follower_addr = rx_buffer[0] & 0x0f;

    // broadcast ack or individual ack
    ack = (follower_addr == cec_logical_addr) ?
	    HDMI_CEC_NORMAL_MSG_ACK : HDMI_CEC_BROADCAST_MSG_ACK;

    hdmi_cec_send_ack(ack);
    rx_count++;

    while (!eom)
    {
	if (rx_count > CEC_RX_BUFF_SIZE)
	{
	    return -1;
	}

	if (hdmi_cec_receive_byte(&rx_buffer[rx_count], &eom))
	{
	    return -1;
	}
	hdmi_cec_send_ack(ack);
	rx_count++;
    }

    rx_size = rx_count;
    rx_state = 1;
    wake_up_interruptible(&waitq_rx);
    return 0;
}

int rxtx_thread(void *data)
{
    count_lo = 0;
    count_hi = 0;
    while(1)
    {
	// get cec input
	if (hdmi_cec_read_reg_bit() == SIG_HI)
	{
	    count_hi++;
	    count_lo = 0;
	}
	else
	{
	    count_lo++;
	    count_hi = 0;
	}

	// maybe a startbit, try to receive message
	if (count_lo >= 2)
	{
	    hdmi_cec_receive_msg();
	    count_lo = 0;
	    count_hi = 0;
	}

	// send message if there is something to send and line is free
	if (message_to_tx && (count_hi > 12))
	{
	    tx_state = hdmi_cec_send_msg();
	    wake_up_interruptible(&waitq_tx);
	    count_lo = 0;
	    count_hi = 0;
	} 
	
	// timeout for poll
	if (count_hi > 1000)
	{
	    count_hi = 12;
	    timeout_rx = 1;
	    wake_up_interruptible(&waitq_rx);
	}
	if (kthread_should_stop())
	{
	    return 0;
	}
	
	usleep_range(1600, 1700);

    }
}

static __s32 hdmi_cec_enable(__bool en)
{
    if (en)
	{
	    writel(readl(HDMI_CEC) | 0x800, HDMI_CEC);
	    hdmi_cec_write_reg_bit(SIG_HI);
	}
    else
	{
	    writel(readl(HDMI_CEC) & (~0x800), HDMI_CEC);
	    hdmi_cec_write_reg_bit(SIG_LO);
	}
    return 0;
}

static int sunxi_cec_open(struct inode *inode, struct file *file)
{
    if (!mutex_trylock(&cec_device_mutex)) {
	warn("another process is accessing the device\n");
	return -EBUSY;
    }

    hdmi_cec_enable (1);

    rxtx_task = kthread_run(rxtx_thread,NULL,"cec_rxtx");

    return 0;
}

static int sunxi_cec_release(struct inode *inode, struct file *file)
{
    kthread_stop(rxtx_task);
    hdmi_cec_enable(0);
    mutex_unlock(&cec_device_mutex);
    return 0;
}

static ssize_t sunxi_cec_read(struct file *file, char __user *buffer,
	    size_t count, loff_t *ppos)
{
    if (rx_state == 1)
    {
	if (rx_size > count) 
	{
	return -1;
	}	
	if (copy_to_user(buffer, rx_buffer, rx_size)) 
	{
	err("copy_to_user() failed!\n");
	return -EFAULT;
	}
	rx_state = 0;
	return rx_size;
    }
    return 0;
}

static ssize_t sunxi_cec_write(struct file *file, const char __user *buffer,
	    size_t count, loff_t *ppos)
{
    if (count > CEC_TX_BUFF_SIZE || count == 0)
    {
	return -1;
    }

    if (copy_from_user(tx_buffer, buffer, count))
    {
    err(" copy_from_user() failed!\n");
	return -EFAULT;
    }
    tx_size = count;
    tx_state = 1; // Daten senden
    message_to_tx = 1;
    /* wait for interrupt */
    if (wait_event_interruptible_timeout(waitq_tx,
	tx_state != 1,
	msecs_to_jiffies(3000)) == 0) {
	err("error : waiting for interrupt is timed out\n");
	return -ERESTARTSYS;
    }

    if (tx_state == -1)
	return -1;

    return count;
}

static long sunxi_cec_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
    u32 laddr;

    switch (cmd) {
    case CEC_IOC_SETLADDR:
	if (get_user(laddr, (u32 __user *) arg))
	    return -EFAULT;

	cec_logical_addr = laddr;
	break;

    default:
	return -EINVAL;
    }

    return 0;
}

static u32 sunxi_cec_poll(struct file *file, poll_table *wait)
{
    poll_wait(file, &waitq_rx, wait);

    if (rx_state == 1)
    {
	return POLLIN | POLLRDNORM;
    }
    if (timeout_rx)
    {
	timeout_rx = 0;
	return POLLERR;
    }
    return 0;
}

static const struct file_operations cec_fops = {
    .owner   = THIS_MODULE,
    .open    = sunxi_cec_open,
    .release = sunxi_cec_release,
    .read    = sunxi_cec_read,
    .write   = sunxi_cec_write,
    .unlocked_ioctl = sunxi_cec_ioctl,
    .poll    = sunxi_cec_poll,
};


int sunxi_cec_init(void)
{
//extern class hdmi_class;    extern class hdmi_class;
    /* First, see if we can dynamically allocate a major for our device */
    cec_major = register_chrdev(0, DEVICE_NAME, &cec_fops);
    if (cec_major < 0) {
	err("failed to register device: error %d\n", cec_major);
	return -1;
    }

    cec_device = device_create(hdmi_class, NULL, MKDEV(cec_major, 0), NULL, DEVICE_NAME);
    if (IS_ERR(cec_device)) {
	err("failed to create device '%s_%s'\n", CLASS_NAME, DEVICE_NAME);
	return -1;
    }

    init_waitqueue_head(&waitq_rx);
    init_waitqueue_head(&waitq_tx);

    mutex_init(&cec_device_mutex);
    return 0;
}

void sunxi_cec_exit(void)
{
    device_destroy(hdmi_class, MKDEV(cec_major, 0));
    unregister_chrdev(cec_major, DEVICE_NAME);
}

