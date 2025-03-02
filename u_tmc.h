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
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/types.h>

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


enum tmc_remote_local_state {
	LOCS,
	LWLS,
	REMS,
	RWLS,
};

enum tmc_rl_state_machine_event {
	TMC_EVENT_INITIATE_CLEAR,
	TMC_EVENT_TRIGGER,
	TMC_EVENT_DEV_DEP_MSG_OUT,
	TMC_EVENT_GOTO_LOCAL,
	TMC_EVENT_LOCAL_LOCKOUT,
	TMC_EVENT_BUS_ACTIVITY,
};

enum tmc_transfer_attributes {
	TMC_XFER_END_OF_MSG = 1,
	TMC_XFER_TERM_CHAR_ENABLED = 2,
};

struct tmc488_capabilities {
	uint16_t bcdUSBTMC;
	uint8_t bmInterfaceCapabilities;
	uint8_t bmDeviceCapabilities;
	uint8_t reserved[6];
	uint16_t bcdUSB488;
	uint8_t bmInterfaceCapabilities488;
	uint8_t bmDeviceCapabilities488;
};

struct capability_response {
	uint8_t USBTMC_status;
	uint8_t reserved1;
	uint16_t bcdUSBTMC;
	uint8_t bmInterfaceCapabilities;
	uint8_t bmDeviceCapabilities;
	uint8_t reserved2[6];
	uint16_t bcdUSB488;
	uint8_t bmInterfaceCapabilities488;
	uint8_t bmDeviceCapabilities488;
	uint8_t reserved3[8];
};

struct status_response {
	uint8_t USBTMC_status;
	uint8_t tag;
	uint8_t status_byte;
};

struct interrupt_response {
	uint8_t tag;
	uint8_t status_byte;
};

struct status_byte_response {
	uint8_t tag;
	uint8_t status_byte;
};

struct tmc_header {
	uint8_t MsgID;
	uint8_t bTag;
	uint8_t bTagInverse;
	uint8_t reserved1;
	uint32_t TransferSize;
	uint8_t bmTransferAttributes;
	uint8_t TermChar;
	uint8_t reserved2[2];
};
#define TMC_HEADER_SIZE sizeof(struct tmc_header)

typedef void (*IndicatorPulseHandler)(void);

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
	int8_t interface;
	bool is_shutdown;
	bool connection_reset;

	/*
	 * TMC specification members
	 */
	uint8_t ren;
	uint8_t status_byte;
	enum tmc_remote_local_state rlstate;

	/*
	 * TMC response members
	 */
	struct tmc488_capabilities capabilities;
	struct status_response status;

	/*
	 * TMS message structure and message status members
	 */
	struct tmc_header header;
	bool header_required;
	bool tx_header_required;
	size_t current_msg_bytes;				/* Total length of the current TMC message */
	size_t current_tx_msg_bytes;			/* Total length of the current TX TMC message */
	size_t current_remaining_tx_msg_bytes;	/* Total remaining length of the current TX TMC message */
	struct usb_request *current_rx_req;
	size_t current_rx_bytes;				/* Current available RX bytes to read */
	uint8_t *current_rx_buf;
	size_t current_tx_bytes;				/* Current available TX bytes to send */
	uint8_t previous_bulk_out_tag;			/* The last bulk OUT bTag */
	uint8_t previous_bulk_in_tag;			/* The last bulk IN bTag */
	bool bulk_out_queued;
	bool bulk_in_queued;
	bool intr_in_queued;
	bool abort_bulk_in_complete;
	uint8_t termchar;

	/*
	 * Synchronization members
	 */
	spinlock_t lock;						/* Lock this structure */
	struct mutex lock_tmc_io;				/* Lock buffer lists during read/write calls */
	wait_queue_head_t rx_wait;				/* Wait until there is data to be read */
	wait_queue_head_t tx_wait;				/* Wait until there are write buffers available to use. */
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

	/*
	 * Misc. members
	 */
	IndicatorPulseHandler indicator_pulse_handler;
};

struct f_tmc_opts {
	struct usb_function_instance			func_inst;
	unsigned int							interface;
	struct mutex 							lock;
	struct tmc_device						*tmc;

	/* TMC Capabilities */
	/* Section 4.2.1.8 of USB TMC Specification Revision 1.0*/
	uint16_t bcdUSBTMC;
	uint8_t bmInterfaceCapabilities;
	uint8_t bmDeviceCapabilities;

	/* 488 Capabilities */
	/* Section 4.2.2 of USB TMC/USB 488 Subclass Specification Revision 1.0 */
	uint16_t bcdUSB488;
	uint8_t bmInterfaceCapabilities488;
	uint8_t bmDeviceCapabilities488;
};

#define fi_to_f_tmc_opts(f)	container_of(f, struct f_tmc_opts, func_inst)

static inline struct tmc_device *func_to_tmc(struct usb_function *f) {
	return container_of(f, struct tmc_device, func);
}


#endif /* USB_U_TMC_H_ */
