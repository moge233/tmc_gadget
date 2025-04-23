/*
 * f_usbtmc.h
 *
 *  Created on: Jan 24, 2024
 *      Author: matt
 */

#ifndef USB_U_TMC_H_
#define USB_U_TMC_H_

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>


#define TMC_GADGET_NUM_ENDPOINTS		3
#define TMC_GADGET_SUBCLASS				3

#define TMC_GADGET_BULK_ENDPOINT_SIZE	512
#define TMC_GADGET_INTR_ENDPOINT_SIZE	16

#define TMC_DEV_DEP_MSG_OUT				1
#define TMC_REQUEST_DEV_DEP_MSG_IN		2
#define TMC_DEV_DEP_MSG_IN				2
#define TMC_VENDOR_SPECIFIC_OUT			126
#define TMC_REQUEST_VENDOR_SPECIFIC_IN	127
#define TMC_VENDOR_SPECIFIC_IN			127
#define TMC_488_TRIGGER					128


enum tmc_gadget_remote_local_state {
	LOCS,
	LWLS,
	REMS,
	RWLS,
};

enum tmc_gadget_rl_state_machine_event {
	TMC_EVENT_INITIATE_CLEAR,
	TMC_EVENT_TRIGGER,
	TMC_EVENT_DEV_DEP_MSG_OUT,
	TMC_EVENT_GOTO_LOCAL,
	TMC_EVENT_LOCAL_LOCKOUT,
	TMC_EVENT_BUS_ACTIVITY,
};

enum tmc_gadget_transfer_attributes {
	TMC_XFER_END_OF_MSG = 1,
	TMC_XFER_TERM_CHAR_ENABLED = 2,
};

struct tmc488_capabilities {
	__u16 bcdUSBTMC;
	__u8 bmUSBTMCInterfaceCapabilities;
	__u8 bmUSBTMCDeviceCapabilities;
	__u8 reserved[6];
	__u16 bcdUSB488;
	__u8 bmUSB488InterfaceCapabilities;
	__u8 bmUSB488DeviceCapabilities;
} __attribute__((packed));

struct capability_response {
	__u8 USBTMC_status;
	__u8 reserved1;
	__u16 bcdUSBTMC;
	__u8 bmUSBTMCInterfaceCapabilities;
	__u8 bmUSBTMCDeviceCapabilities;
	__u8 reserved2[6];
	__u16 bcdUSB488;
	__u8 bmUSB488InterfaceCapabilities;
	__u8 bmUSB488DeviceCapabilities;
	__u8 reserved3[8];
} __attribute__((packed));

struct status_response {
	__u8 USBTMC_status;
	__u8 tag;
	__u8 status_byte;
} __attribute__((packed));

struct interrupt_response {
	__u8 tag;
	__u8 status_byte;
} __attribute__((packed));

struct status_byte_response {
	__u8 tag;
	__u8 status_byte;
} __attribute__((packed));

struct tmc_header {
	__u8 MsgID;
	__u8 bTag;
	__u8 bTagInverse;
	__u8 reserved1;
	__u32 TransferSize;
	__u8 bmTransferAttributes;
	__u8 TermChar;
	__u8 reserved2[2];
} __attribute__((packed));
#define TMC_GADGET_HEADER_SIZE sizeof(struct tmc_header)

struct tmc_device {
	/*
	 * Members that are generally required for USB gadgets
	 */
	struct usb_gadget *gadget;
	struct usb_function func;

	/*
	 * Device driver members
	 */
	struct device dev;
	struct cdev	cdev;

	/*
	 * TMC gadget status members
	 */
	__s8 interface;
	bool is_shutdown;
	bool connection_reset;

	/*
	 * TMC device capabilities
	 */
	struct tmc488_capabilities device_capabilities;

	/*
	 * Message status members
	 */
	struct tmc_header current_header;
	bool new_header_required;

	__u32 current_rx_bytes;				/* Current available RX bytes to read */
	__u32 current_rx_bytes_remaining;	/* Total length of the current TMC message */
	__u8 *current_rx_buf;				/* Start of the current RX buffer */

	__u32 current_tx_bytes;				/* Current available TX bytes to send */
	__u32 current_tx_bytes_remaining;	/* Remaining bytes in a TX transfer */
	__u8 *current_tx_buf;				/* Start of the current TX buffer */

	__u8 previous_bulk_out_tag;			/* The last bulk OUT bTag */
	__u8 previous_bulk_in_tag;			/* The last bulk IN bTag */

	bool bulk_out_queued;
	bool bulk_in_queued;
	bool intr_in_queued;
	bool abort_bulk_in_complete;

	/*
	 * Synchronization members
	 */
	spinlock_t lock;						/* Lock this structure */
	struct mutex io_lock;				/* Lock during read/write calls */
	wait_queue_head_t rx_wait;				/* Wait until there is data to be read */
	wait_queue_head_t tx_wait;				/* Wait until we can write. */
	bool rx_complete;
	bool tx_pending;

	/*
	 * Endpoint structures
	 */
	struct usb_ep *bulk_in_ep;
	struct usb_ep *bulk_out_ep;
	struct usb_ep *interrupt_ep;

	/*
	 * Endpoint requests
	 */
	struct usb_request *bulk_out_req;
	struct usb_request *bulk_in_req;
	struct usb_request *interrupt_req;

};

struct f_tmc_opts {
	struct usb_function_instance func_inst;
	unsigned int interface;
	struct mutex io_lock;
	struct tmc_device *tmc;

	/* TMC Capabilities */
	/* Section 4.2.1.8 of USB TMC Specification Revision 1.0*/
	__u16 bcdUSBTMC;
	__u8 bmInterfaceCapabilities;
	__u8 bmDeviceCapabilities;

	/* 488 Capabilities */
	/* Section 4.2.2 of USB TMC/USB 488 Subclass Specification Revision 1.0 */
	__u16 bcdUSB488;
	__u8 bmInterfaceCapabilities488;
	__u8 bmDeviceCapabilities488;
};

static inline struct f_tmc_opts *tmc_gadget_function_instance_to_f_tmc_opts(const struct usb_function_instance *func_inst)
{
	return container_of(func_inst, struct f_tmc_opts, func_inst);
}

static inline struct tmc_device *tmc_gadget_function_to_tmc_device(struct usb_function *func) {
	return container_of(func, struct tmc_device, func);
}

static inline struct f_tmc_opts *tmc_gadget_config_item_to_f_tmc_opts(struct config_item *item)
{
	return container_of(to_config_group(item), struct f_tmc_opts, func_inst.group);
}

static inline struct usb_endpoint_descriptor *tmc_gadget_get_ep_descriptor(struct usb_gadget *gadget,
					struct usb_endpoint_descriptor *fs,
					struct usb_endpoint_descriptor *hs,
					struct usb_endpoint_descriptor *ss)
{
	switch (gadget->speed) {
		case USB_SPEED_SUPER:
			return ss;
		case USB_SPEED_HIGH:
			return hs;
		default:
			return fs;
	}
}


#endif /* USB_U_TMC_H_ */
