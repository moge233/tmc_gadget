/*
 * f_usbtmc.h
 *
 *  Created on: Jan 24, 2024
 *      Author: matt
 */

#ifndef USB_U_TMC_H_
#define USB_U_TMC_H_

#include <linux/printk.h>

#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/usb/tmc.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/types.h>


#define NUM_BULK_REQUESTS				1

#define TMC_INTF						0
#define TMC_NUM_ENDPOINTS				3
#define TMC_488_SUBCLASS				3

#define TMC_BULK_ENDPOINT_SIZE			512
#define TMC_INTR_ENDPOINT_SIZE			16

#define TMC_DEV_DEP_MSG_OUT				1
#define TMC_REQUEST_DEV_DEP_MSG_IN		2
#define TMC_DEV_DEP_MSG_IN				2
#define TMC_VENDOR_SPECIFIC_OUT			126
#define TMC_REQUEST_VENDOR_SPECIFIC_IN	127
#define TMC_VENDOR_SPECIFIC_IN			127
#define TMC_488_TRIGGER					128

enum tmc_state {
	TMC_STATE_DISCONNECTED,
	TMC_STATE_CONNECTED,
};

enum tmc_remote_local_state {
	LOCS,
	LWLS,
	REMS,
	RWLS,
};

enum tmc_transfer_attributes {
	TMC_XFER_END_OF_MSG = 1,
	TMC_XFER_TERM_CHAR_ENABLED = 2,
};

struct f_tmc_opts {
	struct usb_function_instance			func_inst;
	unsigned int							interface;
	struct mutex 							lock;

	/* TMC Capabilities */
	/* Section 4.2.1.8 of USB TMC Specification Revision 1.0*/
	u16 bcdUSBTMC;
	u8 bmInterfaceCapabilities;
	u8 bmDeviceCapabilities;

	/* 488 Capabilities */
	/* Section 4.2.2 of USB TMC/USB 488 Subclass Specification Revision 1.0 */
	u16 bcdUSB488;
	u8 bmInterfaceCapabilities488;
	u8 bmDeviceCapabilities488;
};

struct capability_response {
	u8 USBTMC_status;
	u8 reserved1;
	u16 bcdUSBTMC;
	u8 bmInterfaceCapabilities;
	u8 bmDeviceCapabilities;
	u8 reserved2[6];
	u16 bcdUSB488;
	u8 bmInterfaceCapabilities488;
	u8 bmDeviceCapabilities488;
	u8 reserved3[8];
};

struct status_response {
	u8 USBTMC_status;
	u8 tag;
	u8 status_byte;
};

struct interrupt_response {
	u8 tag;
	u8 status_byte;
};

struct tmc_header {
	u8 MsgID;
	u8 bTag;
	u8 bTagInverse;
	u8 reserved1;
	u32 TransferSize;
	u8 bmTransferAttributes;
	u8 TermChar;
	u8 reserved2[2];
};
#define TMC_HEADER_SIZE sizeof(struct tmc_header)

struct tmc_device {
	spinlock_t lock;						/* Lock this structure */
	struct mutex		lock_tmc_io;		/* Lock buffer lists during read/write calls */
	struct usb_gadget *gadget;
	s8 interface;

	struct usb_ep *bulk_in_ep;
	struct usb_ep *bulk_out_ep;
	struct usb_ep *interrupt_ep;

	wait_queue_head_t rx_wait;				/* Wait until there is data to be read */
	wait_queue_head_t tx_wait;				/* Wait until there are write buffers available to use. */

	struct list_head rx_reqs;				/* List of free RX structs */
	struct list_head rx_reqs_active;		/* List of Active RX xfers */
	struct list_head rx_buffers;			/* List of completed xfers */

	struct list_head tx_reqs;				/* List of free TX structs */
	struct list_head tx_reqs_active;		/* List of Active TX xfers */

	struct usb_function func;

	enum tmc_state state;
	bool ren;
	u8 status_byte;

	struct capability_response capabilities;
	struct status_response status;
	struct interrupt_response interrupt;

	struct tmc_header header;
	bool header_required;
	size_t current_msg_bytes;
	struct usb_request *current_rx_req;
	size_t current_rx_bytes;
	u8 *current_rx_buf;

	struct usb_request *interrupt_req;

	struct device dev;
	struct cdev	cdev;
};

#define fi_to_f_tmc_opts(f)	container_of(f, struct f_tmc_opts, func_inst)

static inline struct tmc_device *func_to_tmc(struct usb_function *f) {
	return container_of(f, struct tmc_device, func);
}

#endif /* USB_U_TMC_H_ */
