/* SPDX-License-Identifier: GPL-2.0+ WITH Linux-syscall-note */
/*
 * g_tmc.h -- Header file for USB Test and Measurement gadget driver
 *
 * Copyright (C) 2026 matt
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __LINUX_USB_G_TMC_H
#define __LINUX_USB_G_TMC_H

#include <linux/types.h>

typedef enum
{
	RL_STATE_LOCAL,				// LOCS
	RL_STATE_LOCAL_LOCKOUT,		// LWLS
	RL_STATE_REMOTE,			// REMS
	RL_STATE_REMOTE_LOCKOUT,	// RWLS
} gadget_tmc488_localremote_state;

typedef struct
{
	u8 MsgID;
	u8 bTag;
	u8 bTagInverse;
	u8 __reserved;
	u32 TransferSize;
	u8 bmTransferAttributes;
	u8 TermChar;
} gadget_tmc_header;
#define GADGET_TMC_HEADER_SIZE sizeof(gadget_tmc_header)

#define GADGET_TMC_STATUS_SUCCESS							0x01
#define GADGET_TMC_STATUS_PENDING							0x02
#define GADGET_TMC_STATUS_FAILED							0x80
#define GADGET_TMC_STATUS_TRANSFER_NOT_IN_PROGRESS			0x81
#define GADGET_TMC_STATUS_TRANSFER_SPLIT_NOT_IN_PROGRESS	0x82
#define GADGET_TMC_STATUS_SPLIT_IN_PROGRESS					0x83
#define GADGET_TMC488_STATUS_INTERRUPT_IN_BUSY				0x20

#define GADGET_TMC_DEV_DEP_MSG_OUT				1
#define GADGET_TMC_REQUEST_DEV_DEP_MSG_IN		2
#define GADGET_TMC_DEV_DEP_MSG_IN				2
#define GADGET_TMC_VENDOR_SPECIFIC_OUT			126
#define GADGET_TMC_REQUEST_VENDOR_SPECIFIC_IN	127
#define GADGET_TMC_VENDOR_SPECIFIC_IN			127
#define GADGET_TMC488_TRIGGER					128

#define GADGET_TMC_REQUEST_INITIATE_ABORT_BULK_OUT			1
#define GADGET_TMC_REQUEST_CHECK_ABORT_BULK_OUT_STATUS		2
#define GADGET_TMC_REQUEST_INITIATE_ABORT_BULK_IN			3
#define GADGET_TMC_REQUEST_CHECK_ABORT_BULK_IN_STATUS		4
#define GADGET_TMC_REQUEST_INITIATE_CLEAR					5
#define GADGET_TMC_REQUEST_CHECK_CLEAR_STATUS				6
#define GADGET_TMC_REQUEST_GET_CAPABILITIES					7
#define GADGET_TMC_REQUEST_INDICATOR_PULSE					64
#define GADGET_TMC488_REQUEST_READ_STATUS_BYTE				128
#define GADGET_TMC488_REQUEST_REN_CONTROL					160
#define GADGET_TMC488_REQUEST_GOTO_LOCAL					161
#define GADGET_TMC488_REQUEST_LOCAL_LOCKOUT					162

#define GADGET_TMC_CAPABILITY_INDICATOR_PULSE				0x04
#define GADGET_TMC488_CAPABILITY_TRIGGER					0x01
#define GADGET_TMC488_CAPABILITY_GOTO_LOCAL					0x02
#define GADGET_TMC488_CAPABILITY_LOCAL_LOCKOUT				0x02
#define GADGET_TMC488_CAPABILITY_REN_CONTROL				0x02
#define GADGET_TMC488_CAPABILITY_INTERFACE_4882				0x04
#define GADGET_TMC488_CAPABILITY_DT1						0x01
#define GADGET_TMC488_CAPABILITY_RL1						0x02
#define GADGET_TMC488_CAPABILITY_SR1						0x04
#define GADGET_TMC488_CAPABILITY_SCPI						0x08

/* The 'g' code is also used by gadgetfs and g_printer ioctl requests.
 * Don't add any colliding codes to either driver, and keep
 * them in unique ranges (size 0x60 for now).
 */
#define GADGET_TMC_IOCTL_ABORT_BULK_OUT		_IOW('g', 0x61, unsigned char)
#define GADGET_TMC_IOCTL_ABORT_BULK_IN		_IOW('g', 0x62, unsigned char)
#define GADGET_TMC488_IOCTL_GET_STB			_IOR('g', 0x63, unsigned char)
#define GADGET_TMC488_IOCTL_SET_STB			_IOW('g', 0x64, unsigned char)
#define GADGET_TMC488_IOCTL_GET_RL_STATE	_IOR('g', 0x65, unsigned char)
#define GADGET_TMC488_IOCTL_SET_RL_STATE	_IOW('g', 0x66, unsigned char)
#define GADGET_TMC_IOCTL_GET_HEADER			_IOR('g', 0x67, gadget_tmc_header)

#endif /* __LINUX_USB_G_PRINTER_H */
