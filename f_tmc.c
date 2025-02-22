/*
 * f_usbtmc.c
 *
 *  Created on: Jan 24, 2024
 *      Author: matt
 */

#include <linux/errno.h>
#include <linux/configfs.h>
#include <linux/poll.h>
#include <uapi/linux/usb/tmc.h>
#include <uapi/linux/usb/tmc_dev.h>

#include "u_tmc.h"

static int major, minor;
static u8 ren = 0;


/*-------------------------------------------------------------------------
 * Static descriptors
 *-------------------------------------------------------------------------*/

static struct usb_endpoint_descriptor tmc_bulk_in_ep_fs = {
	.bLength			= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor tmc_bulk_out_ep_fs = {
	.bLength			= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_OUT,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor tmc_interrupt_ep_fs = {
	.bLength			= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_XFER_INT,
	.bInterval			= 1,
};

static struct usb_interface_descriptor tmc_intf = {
	.bLength				= USB_DT_INTERFACE_SIZE,
	.bDescriptorType		= USB_DT_INTERFACE,
	/* .bInterfaceNumber	= DYNAMIC, */
	.bNumEndpoints			= TMC_NUM_ENDPOINTS,
	.bInterfaceClass		= USB_CLASS_APP_SPEC,
	.bInterfaceSubClass		= TMC_488_SUBCLASS,
	.bInterfaceProtocol		= 1,
	/* .iInterface			= DYNAMIC, */
};

static struct usb_descriptor_header *tmc_descriptors_fs[] = {
		(struct usb_descriptor_header *) &tmc_intf,
		(struct usb_descriptor_header *) &tmc_bulk_in_ep_fs,
		(struct usb_descriptor_header *) &tmc_bulk_out_ep_fs,
		(struct usb_descriptor_header *) &tmc_interrupt_ep_fs,
		NULL
};

static struct usb_endpoint_descriptor tmc_bulk_in_ep_hs = {
	.bLength			= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= cpu_to_le16(512),
};

static struct usb_endpoint_descriptor tmc_bulk_out_ep_hs = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= cpu_to_le16(512),
};

static struct usb_endpoint_descriptor tmc_interrupt_ep_hs = {
	.bLength			= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bmAttributes		= USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize		= cpu_to_le16(16),
	.bInterval			= 1,
};

static struct usb_descriptor_header *tmc_descriptors_hs[] = {
		(struct usb_descriptor_header *) &tmc_intf,
		(struct usb_descriptor_header *) &tmc_bulk_in_ep_hs,
		(struct usb_descriptor_header *) &tmc_bulk_out_ep_hs,
		(struct usb_descriptor_header *) &tmc_interrupt_ep_hs,
		NULL
};

static inline struct usb_endpoint_descriptor *ep_desc(struct usb_gadget *gadget,
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


/*-------------------------------------------------------------------------
 * Static strings
 *-------------------------------------------------------------------------*/

static struct usb_string tmc_en_us_strings[] = {
	{ }
};

static struct usb_gadget_strings tmc_stringtab = {
	.language = 0x0409,	/* en-us */
	.strings = tmc_en_us_strings,
};

static struct usb_gadget_strings *tmc_function_strings[] = {
	&tmc_stringtab,
	NULL,
};


/*-------------------------------------------------------------------------
 * Utility functions
 *-------------------------------------------------------------------------*/

static void tmc_req_free(struct usb_ep *ep, struct usb_request *req)
{
	if(ep != NULL && req != NULL) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static int tmc_try_halt_bulk_out_endpoint(struct tmc_device *tmc)
{

	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);
	u16 retry_count = 1000;
	int err = 0;
	while(((err = usb_ep_set_halt(tmc->bulk_out_ep)) == -EAGAIN) && (retry_count > 0)) {
		--retry_count;
	}

	return err;
}

static struct usb_request *tmc_req_alloc(struct usb_ep *ep, unsigned len, gfp_t gfp_flags)
{
	struct usb_request	*req;

	req = usb_ep_alloc_request(ep, gfp_flags);

	if (req != NULL) {
		req->length = len;
		req->buf = kmalloc(len, gfp_flags);
		if (req->buf == NULL) {
			usb_ep_free_request(ep, req);
			return NULL;
		}
	}

	return req;
}

static int tmc_ioctl_set_stb(struct tmc_device *tmc, void __user *arg)
{
	u8 status_byte;

	if (get_user(status_byte, (__u8 __user *)arg))
		return -EFAULT;

	tmc->status_byte = status_byte;
	return 0;
}

static int tmc_rl_state_machine(struct tmc_device *tmc, enum tmc_rl_state_machine_event event)
{
	switch(tmc->rlstate) {
	case LOCS:
		if(tmc->ren) {
			switch(event) {
			case TMC_EVENT_INITIATE_CLEAR:
			case TMC_EVENT_TRIGGER:
			case TMC_EVENT_DEV_DEP_MSG_OUT:
				tmc->rlstate = REMS;
				break;
			case TMC_EVENT_LOCAL_LOCKOUT:
				tmc->rlstate = LWLS;
				break;
			default:
				break;
			}
		}
		break;
	case LWLS:
		switch(event) {
		case TMC_EVENT_INITIATE_CLEAR:
		case TMC_EVENT_TRIGGER:
		case TMC_EVENT_DEV_DEP_MSG_OUT:
			tmc->rlstate = RWLS;
			break;
		default:
			break;
		}
		break;
	case REMS:
		switch(event) {
		case TMC_EVENT_GOTO_LOCAL:
		case TMC_EVENT_BUS_ACTIVITY:
			tmc->rlstate = LOCS;
			break;
		case TMC_EVENT_LOCAL_LOCKOUT:
			tmc->rlstate = RWLS;
			break;
		default:
			break;
		}
		break;
	case RWLS:
		switch(event) {
		case TMC_EVENT_GOTO_LOCAL:
		case TMC_EVENT_BUS_ACTIVITY:
			tmc->rlstate = LWLS;
			break;
		default:
			break;
		}
		break;
	}

	return 0;
}


/*-------------------------------------------------------------------------
 * Char device
 *-------------------------------------------------------------------------*/

static void tmc_function_bulk_out_req_complete(struct usb_ep *ep, struct usb_request *req)
{

	struct tmc_device *tmc = (struct tmc_device *)ep->driver_data;
	int status = req->status;
	unsigned long flags = 0;

	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);

	spin_lock_irqsave(&tmc->lock, flags);

	switch(status) {
		case 0:
			if(req->actual > 0) {
				/*
				 * Parse the TMC Bulk OUT Header and verify the contents
				 * per USBTMC Spec. Section 3.2.2.3
				 */
				if(!tmc->current_msg_bytes) {
					if(req->actual < TMC_HEADER_SIZE) {
						/*
						 * Halt the bulk out endpoint and discard any received data
						 * There will not be any requests queued up at this point
						 */
						usb_ep_set_halt(tmc->bulk_out_ep);
						dev_err(&tmc->dev, "error: message length less than TMC header size\n");
						break;
					}

					tmc->header_required = false;
					memcpy(&tmc->header, req->buf, TMC_HEADER_SIZE);

					/*
					 * Check the MsgID value to make sure it is recognized and supported
					 */
					if((tmc->header.MsgID != TMC_DEV_DEP_MSG_OUT) &&
							(tmc->header.MsgID != TMC_REQUEST_DEV_DEP_MSG_IN) &&
							(tmc->header.MsgID != TMC_VENDOR_SPECIFIC_OUT) &&
							(tmc->header.MsgID != TMC_REQUEST_VENDOR_SPECIFIC_IN) &&
							(tmc->header.MsgID != TMC_488_TRIGGER))
					{
						/*
						 * Halt the bulk out endpoint and discard any received data
						 * There will not be any requests queued up at this point
						 */
						usb_ep_set_halt(tmc->bulk_out_ep);
						printk("error: unrecognized header MsgID value (%u)\n", tmc->header.MsgID);
						break;
					}

					if((tmc->header.MsgID == TMC_REQUEST_DEV_DEP_MSG_IN) ||
							(tmc->header.MsgID == TMC_REQUEST_VENDOR_SPECIFIC_IN)) {
						tmc->current_msg_bytes = TMC_HEADER_SIZE;
					}
					else {
						tmc->current_msg_bytes = TMC_HEADER_SIZE + tmc->header.TransferSize;
					}

					/*
					 * Check the transfer attributes for a termination character
					 */
					if(tmc->header.MsgID == TMC_REQUEST_DEV_DEP_MSG_IN) {
						if(tmc->header.bmTransferAttributes & 2) {
							tmc->termchar = tmc->header.TermChar;
						}
					}
					/*
					 * The total number of bytes sent must be a multiple of 4 so check
					 * for alignment bytes here
					 */
					tmc->current_msg_bytes += (tmc->current_msg_bytes % 4)
									? (4 - (tmc->current_msg_bytes % 4))
									: 0;
				}
				tmc->rx_complete = true;
				tmc->bulk_out_queued = false;
			}
			else {
				tmc->rx_complete = false;
			}
			break;
		case -ECONNRESET:
			tmc->header_required = true;
			tmc->rx_complete = false;
			tmc->connection_reset = true;
			dev_dbg(&tmc->dev, "---- case ECONNRESET, status: %d\n", status);
			break;
		case -ESHUTDOWN:
			dev_dbg(&tmc->dev, "---- case ESHUTDOWN, status: %d\n", status);
			break;
		case -ECONNABORTED:
			dev_dbg(&tmc->dev, "---- case ECONNABORTED, status: %d\n", status);
			break;
		case -EOVERFLOW:
			dev_dbg(&tmc->dev, "---- case EOVERFLOW, status: %d\n", status);
			break;
		case -EPIPE:
			dev_dbg(&tmc->dev, "---- case EPIPE, status: %d\n", status);
			break;
		default:
			dev_dbg(&tmc->dev, "---- case default, status: %d\n", status);
			break;
	}

	wake_up_interruptible(&tmc->rx_wait);
	spin_unlock_irqrestore(&tmc->lock, flags);

	return;
}

static void tmc_function_bulk_in_req_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct tmc_device *tmc = ep->driver_data;

	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);

	switch (req->status) {
	case 0:
		tmc->bulk_in_queued = false;
		break;
	default:
		printk("tx err %d\n", req->status);
		fallthrough;
	case -ECONNRESET:		/* unlink */
	case -ESHUTDOWN:		/* disconnect etc */
		printk("%s: SHUTDOWN\n", __func__);
		break;
	}

	spin_lock(&tmc->lock);
	tmc->tx_pending = false;
	wake_up_interruptible(&tmc->tx_wait);
	spin_unlock(&tmc->lock);

	return;
}

static void tmc_function_interrupt_req_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct tmc_device *tmc = ep->driver_data;
	tmc->intr_in_queued = false;
	return;
}

static int tmc_setup_bulk_out_req(struct tmc_device *tmc)
{
	struct usb_request *req;
	int error = 0;

	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);

	req = tmc->bulk_out_req;

	/* The USB Host sends us whatever amount of data it wants to
	 * so we always set the length field to the full buffer size.
	 */
	req->length = TMC_BULK_ENDPOINT_SIZE;
	req->complete = tmc_function_bulk_out_req_complete;

	/* here, we unlock, and only unlock, to avoid deadlock. */
	spin_unlock(&tmc->lock);
	error = usb_ep_queue(tmc->bulk_out_ep, req, GFP_ATOMIC);
	spin_lock(&tmc->lock);

	if (error) {
		dev_err(&tmc->dev, "%s, %s: could not queue bulk out endpoint (error %d)\n", __FILE__, __func__, error);
	}
	else {
		tmc->bulk_out_queued = true;
	}

	return error;
}

static ssize_t tmc_function_fops_read(struct file * file, char __user *buf, size_t len, loff_t *offset)
{
	struct tmc_device *tmc  = file->private_data;
	size_t bytes_copied;
	size_t current_rx_bytes;
	u8 *current_rx_buf;
	struct usb_request *req;
	unsigned long flags = 0;
	size_t size;

	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);

	if(len == 0) {
		return -EINVAL;
	}

	mutex_lock(&tmc->lock_tmc_io);
	spin_lock_irqsave(&tmc->lock, flags);

	bytes_copied = 0;
	current_rx_bytes = tmc->current_rx_bytes;

	if(!current_rx_bytes) {
		int err = tmc_setup_bulk_out_req(tmc);
		if(err) {
			return (ssize_t) err;
		}
	}

	/* Check if there is any data in the read buffers. Please note that
	 * current_rx_bytes is the number of bytes in the current rx buffer.
	 * If it is zero then check if there are any other rx_buffers that
	 * are on the completed list. We are only out of data if all rx
	 * buffers are empty.
	 */
	if((current_rx_bytes == 0) && !tmc->rx_complete) {
		/* Turn interrupts back on before sleeping. */
		spin_unlock_irqrestore(&tmc->lock, flags);

		/*
		 * If no data is available check if this is a NON-Blocking
		 * call or not.
		 */
		if (file->f_flags & (O_NONBLOCK | O_NDELAY)) {
			mutex_unlock(&tmc->lock_tmc_io);
			return -EAGAIN;
		}

		/* Sleep until data is available */
		wait_event_interruptible(tmc->rx_wait, ((true == tmc->rx_complete) || (true == tmc->connection_reset)));
		spin_lock_irqsave(&tmc->lock, flags);
	}

	/* We have data to return then copy it to the caller's buffer.*/
	while((current_rx_bytes || tmc->rx_complete) && len) {
		if(!tmc->header_required && (tmc->header.MsgID != 0)) {
			/*
			 * Mark the message as currently being processed
			 */
			tmc->header.MsgID = 0;
		}

		if(current_rx_bytes == 0) {
			req = tmc->bulk_out_req;

			if(req->actual && req->buf) {
				current_rx_bytes = req->actual;
				current_rx_buf = req->buf;
			}
		}
		else {
			current_rx_bytes = tmc->current_rx_bytes;
			current_rx_buf = tmc->current_rx_buf;
		}

		/* Don't leave irqs off while doing memory copies */
		spin_unlock_irqrestore(&tmc->lock, flags);

		if(len > current_rx_bytes)
			size = current_rx_bytes;
		else
			size = len;

		size -= copy_to_user(buf, current_rx_buf, size);
		bytes_copied += size;
		len -= size;
		buf += size;

		tmc->current_msg_bytes -= bytes_copied;
		if(!tmc->current_msg_bytes)
		{
			tmc->header_required = true;
		}

		spin_lock_irqsave(&tmc->lock, flags);

		/*
		 * If we aren't returning all the data left in this RX request
		 * buffer then adjust the amount of data left in the buffer.
		 * Otherwise if we are done with this RX request buffer then
		 * re-queue it to get any incoming data from the USB host.
		 */
		if(size < current_rx_bytes) {
			current_rx_bytes -= size;
			current_rx_buf += size;
		}
		else
		{
			current_rx_bytes = 0;
			current_rx_buf = NULL;
			tmc->rx_complete = false;
			tmc_req_free(tmc->bulk_out_ep, tmc->bulk_out_req);
			tmc->bulk_out_req = tmc_req_alloc(tmc->bulk_out_ep, TMC_BULK_ENDPOINT_SIZE, GFP_KERNEL);
		}
	}

	tmc->current_rx_buf = current_rx_buf;
	tmc->current_rx_bytes = current_rx_bytes;

	spin_unlock_irqrestore(&tmc->lock, flags);
	mutex_unlock(&tmc->lock_tmc_io);

	if(tmc->connection_reset) {
		tmc->connection_reset = false;
		return -ECONNRESET;
	}

	if (bytes_copied)
		return bytes_copied;
	else
		return -EAGAIN;
}

static ssize_t tmc_function_fops_write(struct file *file, const char __user *buf, size_t len, loff_t *offset)
{
	struct tmc_device *tmc = file->private_data;
	unsigned long flags;
	size_t size;
	size_t bytes_copied = 0;
	struct usb_request *req;
	int value;

	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);

	if (len == 0)
		return -EINVAL;

	mutex_lock(&tmc->lock_tmc_io);
	spin_lock_irqsave(&tmc->lock, flags);

	if(tmc->tx_pending) {
		/* Turn interrupts back on before sleeping. */
		spin_unlock_irqrestore(&tmc->lock, flags);

		/*
		 * If write buffers are available check if this is
		 * a NON-Blocking call or not.
		 */
		if (file->f_flags & (O_NONBLOCK | O_NDELAY)) {
			mutex_unlock(&tmc->lock_tmc_io);
			return -EAGAIN;
		}

		wait_event_interruptible(tmc->tx_wait, (false == tmc->tx_pending));
		spin_lock_irqsave(&tmc->lock, flags);
	}

	while((false == tmc->tx_pending) && len) {

		if (len > TMC_BULK_ENDPOINT_SIZE)
			size = TMC_BULK_ENDPOINT_SIZE;
		else
			size = len;

		req = tmc->bulk_in_req;

		req->complete = tmc_function_bulk_in_req_complete;
		req->length = size;

		/* Check if we need to send a zero length packet. */
		if (len > size)
		{
			/* They will be more TX requests so no yet. */
			req->zero = 0;
		}
		else
		{
			/* If the data amount is not a multiple of the
			 * maxpacket size then send a zero length packet.
			 */
			req->zero = ((len % TMC_BULK_ENDPOINT_SIZE) == 0);
		}

		/* Don't leave irqs off while doing memory copies */
		spin_unlock_irqrestore(&tmc->lock, flags);

		if(copy_from_user(req->buf, buf, size)) {
			tmc->tx_pending = false;
			mutex_unlock(&tmc->lock_tmc_io);
			return bytes_copied;
		}

		bytes_copied += size;
		len -= size;
		buf += size;

		spin_lock_irqsave(&tmc->lock, flags);

		/* here, we unlock, and only unlock, to avoid deadlock. */
		spin_unlock(&tmc->lock);
		value = usb_ep_queue(tmc->bulk_in_ep, req, GFP_ATOMIC);
		spin_lock(&tmc->lock);
		if (value) {
			tmc->tx_pending = false;
			spin_unlock_irqrestore(&tmc->lock, flags);
			mutex_unlock(&tmc->lock_tmc_io);
			return -EAGAIN;
		}
		else {
			tmc->tx_pending = true;
			tmc->bulk_in_queued = true;
		}
	}

	spin_unlock_irqrestore(&tmc->lock, flags);
	mutex_unlock(&tmc->lock_tmc_io);

	if (bytes_copied)
		return bytes_copied;
	else
		return -EAGAIN;
}

static __poll_t tmc_function_fops_poll(struct file *file, struct poll_table_struct *wait)
{
	// TODO: f_tmc_poll
	unsigned long flags = 0;
	struct tmc_device *tmc  = file->private_data;
	__poll_t	ret = 0;

	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);

	mutex_lock(&tmc->lock_tmc_io);
	spin_lock_irqsave(&tmc->lock, flags);

	tmc_setup_bulk_out_req(tmc);
	spin_unlock_irqrestore(&tmc->lock, flags);
	mutex_unlock(&tmc->lock_tmc_io);

	poll_wait(file, &tmc->rx_wait, wait);
	poll_wait(file, &tmc->tx_wait, wait);

	spin_lock_irqsave(&tmc->lock, flags);

	if(!tmc->tx_pending)
		ret |= EPOLLOUT | EPOLLWRNORM;

	if(likely(tmc->current_rx_bytes) || tmc->rx_complete)
		ret |= EPOLLIN | EPOLLRDNORM;

	spin_unlock_irqrestore(&tmc->lock, flags);

	return ret;
}

long tmc_gadget_function_fops_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct tmc_device *tmc  = file->private_data;
	long ret = 0;

	dev_dbg(&tmc->dev, "%s, %s: cmd %u, arg %lu\n", __FILE__, __func__, cmd, arg);

	unsigned long flags = 0;
	spin_lock_irqsave(&tmc->lock, flags);
	switch(cmd)
	{
	case USBTMC_DEV_IOCTL_ABORT_BULK_OUT:
		usb_ep_dequeue(tmc->bulk_out_ep, tmc->bulk_out_req);
		break;
	case USBTMC_DEV_IOCTL_ABORT_BULK_IN:
		usb_ep_dequeue(tmc->bulk_in_ep, tmc->bulk_out_req);
		break;
	case USBTMC_DEV_IOCTL_SET_STB:
		ret = (long) tmc_ioctl_set_stb(tmc, (void __user *)arg);
		break;
	default:
		break;
	}
	spin_unlock_irqrestore(&tmc->lock, flags);
	return 0;
}

static int tmc_function_fops_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;

	return 0;
}

static int tmc_function_fops_open(struct inode *inode, struct file *file)
{
	struct tmc_device *tmc;

	tmc = container_of(inode->i_cdev, struct tmc_device, cdev);
	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);
	file->private_data = tmc;
	return 0;
}

static const struct file_operations f_tmc_fops = {
	.owner			= THIS_MODULE,
	.open			= tmc_function_fops_open,
	.release		= tmc_function_fops_release,
	.write			= tmc_function_fops_write,
	.read			= tmc_function_fops_read,
	.poll			= tmc_function_fops_poll,
	.unlocked_ioctl = tmc_gadget_function_fops_ioctl,
	.llseek			= noop_llseek,
};

static void tmc_function_device_release(struct device *dev)
{
	// TODO: tmc_function_device_release. What needs to be done in here?
	return;
}

static const struct class tmc_class = {
	.name = "tmc",
};

static int tmc_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_tmc_opts *opts;
	struct usb_ep *ep;
	struct tmc_device *tmc = func_to_tmc(f);
	int ret = -EINVAL;

	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);

	opts = fi_to_f_tmc_opts(f->fi);

	mutex_init(&opts->lock);

	/*
	 * Get TMC/488 capabilities from ConfigFS
	 * These are user-configured values because the device using the driver may or may not
	 * have certain features. This allows a user to configure the driver to suit their device.
	 */
	tmc->capabilities.bcdUSBTMC = opts->bcdUSBTMC;
	tmc->capabilities.bmInterfaceCapabilities = opts->bmInterfaceCapabilities;
	tmc->capabilities.bmDeviceCapabilities = opts->bmDeviceCapabilities;
	tmc->capabilities.bcdUSB488 = opts->bcdUSB488;
	tmc->capabilities.bmInterfaceCapabilities488 = opts->bmInterfaceCapabilities488;
	tmc->capabilities.bmDeviceCapabilities488 = opts->bmDeviceCapabilities488;

	/* Allocate interface IDs. */
	ret = usb_interface_id(c, f);
	if(ret < 0) {
		printk("could not allocate interface ID\n");
	}
	tmc_intf.bInterfaceNumber = ret;

	/* finish hookup to lower layer ... */
	tmc->gadget = c->cdev->gadget;

	/* Allocate endpoints. */
	ret = -ENODEV;
	ep = usb_ep_autoconfig(c->cdev->gadget, &tmc_bulk_in_ep_fs);
	if (!ep) {
		printk("Unable to allocate bulk in EP\n");
		return ret;
	}
	else {
		tmc->bulk_in_ep = ep;
	}

	ep = usb_ep_autoconfig(c->cdev->gadget, &tmc_bulk_out_ep_fs);
	if (!ep) {
		printk("Unable to allocate bulk out EP\n");
		return ret;
	}
	else {
		tmc->bulk_out_ep = ep;
	}

	ep = usb_ep_autoconfig(c->cdev->gadget, &tmc_interrupt_ep_fs);
	if (!ep) {
		printk("Unable to allocate interrupt EP\n");
		return ret;
	}
	else {
		tmc->interrupt_ep = ep;
	}

	/* All endpoints are dual speed */
	tmc_bulk_in_ep_hs.bEndpointAddress = tmc_bulk_in_ep_fs.bEndpointAddress;
	tmc_bulk_out_ep_hs.bEndpointAddress = tmc_bulk_out_ep_fs.bEndpointAddress;
	tmc_interrupt_ep_hs.bEndpointAddress = tmc_interrupt_ep_fs.bEndpointAddress;

	/* Copy descriptors */
	ret = usb_assign_descriptors(f, tmc_descriptors_fs, tmc_descriptors_hs, NULL, NULL);
	if(ret) {
		printk("Failed to assign descriptors\n");
		return ret;
	}

	ret = -ENOMEM;
	tmc->bulk_in_req = tmc_req_alloc(tmc->bulk_in_ep, TMC_BULK_ENDPOINT_SIZE, GFP_KERNEL);
	if(!tmc->bulk_in_req) {
		printk("Failed to allocate bulk_in_req\n");
		goto error_bulk_in_req;
	}

	tmc->bulk_out_req = tmc_req_alloc(tmc->bulk_out_ep, TMC_BULK_ENDPOINT_SIZE, GFP_KERNEL);
	if(!tmc->bulk_out_req) {
		printk("Failed to allocate bulk_out_req\n");
		goto error_bulk_out_req;
	}

	/* Allocate interrupt request and its buffer */
	tmc->interrupt_req = tmc_req_alloc(tmc->interrupt_ep, TMC_INTR_ENDPOINT_SIZE, GFP_KERNEL);
	if(!tmc->interrupt_req) {
		printk("Failed to allocate interrupt req\n");
		goto error_interrupt_req;
	}

	memset(&tmc->header, 0, TMC_HEADER_SIZE);
	tmc->header_required = true;

	/* Create the char device */
	cdev_init(&tmc->cdev, &f_tmc_fops);
	ret = cdev_device_add(&tmc->cdev, &tmc->dev);
	if(ret) {
		printk("Unable to create character device\n");
		goto error_cdev;
	}

	tmc->connection_reset = false;
	tmc->bulk_out_queued = false;
	tmc->bulk_in_queued = false;
	tmc->intr_in_queued = false;
	tmc->ren = 0;
	tmc->rlstate = LOCS;
	tmc->termchar = 0;
	tmc->indicator_pulse_handler = NULL;

	opts->tmc = tmc;

	return 0;

error_cdev:
	cdev_del(&tmc->cdev);

error_interrupt_req:
	tmc_req_free(tmc->bulk_out_ep, tmc->bulk_out_req);

error_bulk_out_req:
	tmc_req_free(tmc->bulk_in_ep, tmc->bulk_in_req);

error_bulk_in_req:

	usb_free_all_descriptors(f);
	return ret;
}

static void tmc_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct tmc_device *tmc = func_to_tmc(f);

	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);

	cdev_device_del(&tmc->cdev, &tmc->dev);

	/* Free all memory for this driver. */
	if(tmc->bulk_in_req != NULL) {
		tmc_req_free(tmc->bulk_in_ep, tmc->bulk_in_req);
	}

	if(tmc->bulk_out_req != NULL) {
		tmc_req_free(tmc->bulk_out_ep, tmc->bulk_out_req);
	}

	if(tmc->interrupt_req != NULL) {
		tmc_req_free(tmc->interrupt_ep, tmc->interrupt_req);
	}

	usb_free_all_descriptors(f);
}

static int set_tmc_interface(struct tmc_device *tmc)
{

	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);

	int result = 0;

	tmc->bulk_in_ep->desc = ep_desc(tmc->gadget, &tmc_bulk_in_ep_fs, &tmc_bulk_in_ep_hs, NULL);
	tmc->bulk_in_ep->driver_data = tmc;

	tmc->bulk_out_ep->desc = ep_desc(tmc->gadget, &tmc_bulk_out_ep_fs, &tmc_bulk_out_ep_hs, NULL);
	tmc->bulk_out_ep->driver_data = tmc;

	tmc->interrupt_ep->desc = ep_desc(tmc->gadget, &tmc_interrupt_ep_fs, &tmc_interrupt_ep_hs, NULL);
	tmc->interrupt_ep->driver_data = tmc;

	result = usb_ep_enable(tmc->bulk_in_ep);
	if (result != 0) {
		printk("failed to enable bulk IN endpoint\n");
		printk("enable %s --> %d\n", tmc->bulk_in_ep->name, result);
		goto done;
	}

	result = usb_ep_enable(tmc->bulk_out_ep);
	if (result != 0) {
		printk("failed to enable bulk OUT endpoint\n");
		printk("enable %s --> %d\n", tmc->bulk_out_ep->name, result);
		goto done;
	}

	result = usb_ep_enable(tmc->interrupt_ep);
	if (result != 0) {
		printk("failed to enable interrupt IN endpoint\n");
		printk("enable %s --> %d\n", tmc->interrupt_ep->name, result);
		goto done;
	}

done:
	/* on error, disable any endpoints  */
	if (result != 0) {
		(void) usb_ep_disable(tmc->bulk_in_ep);
		(void) usb_ep_disable(tmc->bulk_out_ep);
		(void) usb_ep_disable(tmc->interrupt_ep);
		tmc->bulk_in_ep->desc = NULL;
		tmc->bulk_out_ep->desc = NULL;
		tmc->interrupt_ep->desc = NULL;
	}

	/* caller is responsible for cleanup on error */
	return result;
}

void tmc_reset_interface(struct tmc_device *tmc)
{

	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);

	unsigned long	flags;

	if (tmc->interface < 0)
		return;

	if (tmc->bulk_in_ep->desc)
		usb_ep_disable(tmc->bulk_in_ep);

	if (tmc->bulk_out_ep->desc)
		usb_ep_disable(tmc->bulk_out_ep);

	if (tmc->interrupt_ep->desc)
		usb_ep_disable(tmc->interrupt_ep);

	spin_lock_irqsave(&tmc->lock, flags);
	tmc->bulk_in_ep->desc = NULL;
	tmc->bulk_out_ep->desc = NULL;
	tmc->interrupt_ep->desc = NULL;
	tmc->interface = -1;
	spin_unlock_irqrestore(&tmc->lock, flags);
}

static int set_interface(struct tmc_device *tmc, unsigned number)
{
	int	result = 0;

	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);

	/* Free the current interface */
	tmc_reset_interface(tmc);

	result = set_tmc_interface(tmc);
	if (result)
		tmc_reset_interface(tmc);
	else
		tmc->interface = number;

	return result;
}

static int tmc_function_set_alt(struct usb_function *f, unsigned interface, unsigned alt)
{
	struct tmc_device *tmc = func_to_tmc(f);
	int ret = -ENOTSUPP;

	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);

	if(!alt)
		ret = set_interface(tmc, interface);

	return ret;
}

static void tmc_function_disable(struct usb_function *f)
{

	struct tmc_device *tmc = func_to_tmc(f);

	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);

	tmc_reset_interface(tmc);
}

static int tmc_function_ctrl_req_initiate_clear(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);
	int ret = 0;
	u16 w_length = le16_to_cpu(ctrl->wLength);

	tmc_rl_state_machine(tmc, TMC_EVENT_INITIATE_CLEAR);

	// Dequeue the bulk out endpoint if necessary
	if(tmc->bulk_out_queued) {
		ret = usb_ep_dequeue(tmc->bulk_out_ep, tmc->bulk_out_req);
		if(ret) {
			dev_err(&tmc->dev, "%s, could not dequeue bulk out endpoint (%d)\n", __func__, ret);
		}
		tmc->bulk_out_queued = false;
	}

	// Dequeue the bulk in endpoint if necessary
	if(tmc->bulk_in_queued) {
		ret = usb_ep_dequeue(tmc->bulk_in_ep, tmc->bulk_in_req);
		if(ret) {
			dev_err(&tmc->dev, "%s, could not dequeue bulk in endpoint (%d)\n", __func__, ret);
		}
		tmc->bulk_in_queued = false;
	}

	// Halt the bulk out endpoint
	ret = tmc_try_halt_bulk_out_endpoint(tmc);
	if(ret) {
		dev_err(&tmc->dev, "error halting bulk out endpoint (%d)\n", ret);
		u8 response[] = { USBTMC_DEV_STATUS_FAILED };

		memcpy(req->buf, (void *) &response, 1);
		req->zero = 0;
		req->length = w_length;
		ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);

		if(ret) {
			dev_err(&tmc->dev, "error queueing initiate clear response on control endpoint (%d)\n", ret);
		}
	}
	else {
		// Queue the control endpoint response from USBTMC 1.00 Table 31
		u8 response[] = { USBTMC_DEV_STATUS_SUCCESS };

		memcpy(req->buf, (void *) &response, 1);
		req->zero = 0;
		req->length = w_length;
		ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);

		if(ret) {
			dev_err(&tmc->dev, "error queueing initiate clear response on control endpoint (%d)\n", ret);
		}
		else {
			// Clear all input and output buffers

			// Flush the bulk out FIFO if necessary
			if(usb_ep_fifo_status(tmc->bulk_out_ep) > 0) {
				usb_ep_fifo_flush(tmc->bulk_out_ep);
			}

			// Flush the bulk in FIFO if necessary
			if(usb_ep_fifo_status(tmc->bulk_in_ep) > 0) {
				usb_ep_fifo_flush(tmc->bulk_in_ep);
			}
		}
	}
	return ret;
}

static int tmc_function_ctrl_req_check_clear_status(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);

	// TODO: handle the case where the bulk in FIFO cannot be emptied
	int ret = 0;
	u16 w_length = le16_to_cpu(ctrl->wLength);
	u8 response[] = { USBTMC_DEV_STATUS_SUCCESS, 0 };

	memcpy(req->buf, (void *) &response, w_length);
	req->zero = 0;
	req->length = w_length;
	ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
	return ret;
}

static int tmc_function_ctrl_req_get_capabilities(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
	int ret = -EOPNOTSUPP;
	u16 w_length = le16_to_cpu(ctrl->wLength);

	struct capability_response response;
	memset(&response, 0, sizeof(struct capability_response));

	response.bcdUSBTMC = cpu_to_le16(tmc->capabilities.bcdUSBTMC);
	response.bmInterfaceCapabilities = tmc->capabilities.bmInterfaceCapabilities;
	response.bmDeviceCapabilities = tmc->capabilities.bmDeviceCapabilities;
	response.bcdUSB488 = cpu_to_le16(tmc->capabilities.bcdUSB488);
	response.bmInterfaceCapabilities488 = tmc->capabilities.bmInterfaceCapabilities488;
	response.bmDeviceCapabilities488 = tmc->capabilities.bmDeviceCapabilities488;

	response.USBTMC_status = USBTMC_DEV_STATUS_SUCCESS;
	if(w_length != sizeof(struct capability_response)) {
		response.USBTMC_status = USBTMC_DEV_STATUS_FAILED;
	}
	ret = min_t(unsigned short, w_length, sizeof(struct capability_response));
	memcpy(req->buf, (void *) &response, ret);

	req->zero = 0;
	req->length = ret;
	ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);

	return ret;
}

static int tmc_function_ctrl_req_indicator_pulse(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{

	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);
	int ret = 0;
	if(tmc->capabilities.bmInterfaceCapabilities & USBTMC488_CAPABILITY_488_DOT_2) {
		u16 w_length = le16_to_cpu(ctrl->wLength);
		u8 response[] = { USBTMC_DEV_STATUS_SUCCESS };

		memcpy(req->buf, (void *) &response, w_length);
		req->zero = 0;
		req->length = w_length;
		ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
	}
	else {
		// TODO: Unhandled request; return a STALL handshake packet?
	}
	return ret;
}

static int tmc_function_ctrl_req_read_status_byte(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{

	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);

	int ret = -EOPNOTSUPP;
	u16 w_value = le16_to_cpu(ctrl->wValue);
	u16 w_length = le16_to_cpu(ctrl->wLength);

	struct status_byte_response response;

	// Attempt to send the status byte via the interrupt endpoint
	// per USBTMC 4.3.1.2
	if(tmc->capabilities.bmDeviceCapabilities488 & USBTMC488_CAPABILITY_488_DOT_2) {

		dev_dbg(&tmc->dev, "---- tmc->status_byte: %u\n", tmc->status_byte);
		// USBTMC/USB488 3.4.2 Table 7
		response.tag = (u8) (USB_DIR_IN | (0x7F & w_value));
		response.status_byte = tmc->status_byte;

		ret = sizeof(response);

		tmc->interrupt_req->length = ret;
		tmc->interrupt_req->zero = 0;
		tmc->interrupt_req->complete = tmc_function_interrupt_req_complete;
		tmc->interrupt_req->context = tmc;
		memcpy(tmc->interrupt_req->buf, (void *) &(response), ret);

		// Attempt to send it on the interrupt endpoint first
		ret = usb_ep_queue(tmc->interrupt_ep, tmc->interrupt_req, GFP_ATOMIC);
		if(!ret) {
			tmc->intr_in_queued = true;
		}

		dev_dbg(&tmc->dev, "---- queued interrupt request; ret = %d\n", ret);

		// Handle the control endpoint now
		if(!ret) {
			tmc->status.USBTMC_status = USBTMC_STATUS_SUCCESS;
			tmc->status.tag = (u8) w_value;
			tmc->status.status_byte = tmc->status_byte;

			ret = w_length;
			memcpy(req->buf, (void *) &(tmc->status), ret);

			req->zero = 0;
			req->length = ret;
			ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);

			dev_dbg(&tmc->dev, "---- status byte copied to control endpoint\n");
		}
	}
	else
	{
		// When a device does not have an interrupt endpoint, the device must
		// queue a control endpoint response as shown in Table 12
		// per USBTMC 4.3.1.1
		tmc->status.USBTMC_status = USBTMC_DEV_STATUS_SUCCESS;
		tmc->status.tag = (u8) w_value;
		tmc->status.status_byte = tmc->status_byte;

		ret = w_length;
		memcpy(req->buf, (void *) &(tmc->status), ret);

		req->zero = 0;
		req->length = ret;
		ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);

		dev_dbg(&tmc->dev, "---- status byte copied to control endpoint\n");
	}

	return ret;
}

static int tmc_function_ctrl_req_ren_control(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
	int ret = 0;
	u16 w_value = le16_to_cpu(ctrl->wValue);
	u16 w_length = le16_to_cpu(ctrl->wLength);
	if(tmc->capabilities.bmDeviceCapabilities488 & USBTMC488_CAPABILITY_REN_CONTROL) {
		ren = 0xFF & w_value;
		u8 response[] = { USBTMC_DEV_STATUS_SUCCESS };

		memcpy(req->buf, (void *) &response, w_length);
		req->zero = 0;
		req->length = w_length;
		ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
	}
	else {
		// Return a stall handshake packet
	}

	return ret;
}

static int tmc_function_ctrl_req_goto_local(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
	int ret = -EOPNOTSUPP;

	if(tmc->capabilities.bmDeviceCapabilities488 & USBTMC488_CAPABILITY_REN_CONTROL) {
		// TODO:
	}
	else {
		// TODO: Stall the control endpoint
	}

	return ret;
}

static int tmc_function_ctrl_req_local_lockout(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
	int ret = -EOPNOTSUPP;

	if(tmc->capabilities.bmDeviceCapabilities488 & USBTMC488_CAPABILITY_RL1) {
		// TODO:
	}

	return ret;
}

static int tmc_function_ctrl_req_initiate_abort_bulk_out(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
	int ret = 0;
	int status = 0;
	u8 response[2] = { 0 };

	tmc->header_required = true;
	tmc->current_rx_bytes = 0;
	tmc->current_msg_bytes = 0;

	u8 current_tag = tmc->header.bTag;
	u8 bulk_out_tag = ctrl->wValue & 0xFF;

	memset(&tmc->header, 0, sizeof(struct tmc_header));

	if(current_tag == bulk_out_tag)
	{
		status = USBTMC_DEV_STATUS_SUCCESS;
		usb_ep_set_halt(tmc->bulk_out_ep);

		req->buf = response;
		req->zero = 0;
		req->length = 2;
		ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if(ret < 0) {
			dev_err(&tmc->dev, "usb_ep_queue failed on gadget->ep0 with error code %d\n", ret);
		}

		usb_ep_fifo_flush(tmc->bulk_out_ep);
	}
	else
	{
		status = USBTMC_DEV_STATUS_TRANSFER_NOT_IN_PROGRESS;
	}

	return ret;
}

static int tmc_function_ctrl_req_check_abort_bulk_out_status(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
	int ret = -EOPNOTSUPP;
	// TODO:
	return ret;
}

static int tmc_function_ctrl_req_initiate_abort_bulk_in(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
	int ret = -EOPNOTSUPP;
	// TODO:
	return ret;
}

static int tmc_function_ctrl_req_check_abort_bulk_in_status(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
	int ret = -EOPNOTSUPP;
	// TODO:
	return ret;
}

static int tmc_function_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct tmc_device *tmc = func_to_tmc(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request	*req = cdev->req;
	int value = -EOPNOTSUPP;
	u16 w_index = le16_to_cpu(ctrl->wIndex);
	u16 w_value = le16_to_cpu(ctrl->wValue);
	u16 w_length = le16_to_cpu(ctrl->wLength);

	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);

	if(ctrl->bRequestType == (USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_IN)) {
		switch (ctrl->bRequest) {
			case USBTMC_REQUEST_INITIATE_CLEAR:
				value = tmc_function_ctrl_req_initiate_clear(cdev, tmc, ctrl, req);
				break;
			case USBTMC_REQUEST_CHECK_CLEAR_STATUS:
				value = tmc_function_ctrl_req_check_clear_status(cdev, tmc, ctrl, req);
				break;
			case USBTMC_REQUEST_GET_CAPABILITIES:
				value = tmc_function_ctrl_req_get_capabilities(cdev, tmc, ctrl, req);
				break;
			case USBTMC_REQUEST_INDICATOR_PULSE:
				value = tmc_function_ctrl_req_indicator_pulse(cdev, tmc, ctrl, req);
				break;
			case USBTMC488_REQUEST_READ_STATUS_BYTE:
				value = tmc_function_ctrl_req_read_status_byte(cdev, tmc, ctrl, req);
				break;
			case USBTMC488_REQUEST_REN_CONTROL:
				value = tmc_function_ctrl_req_ren_control(cdev, tmc, ctrl, req);
				break;
			case USBTMC488_REQUEST_GOTO_LOCAL:
				value = tmc_function_ctrl_req_goto_local(cdev, tmc, ctrl, req);
				break;
			case USBTMC488_REQUEST_LOCAL_LOCKOUT:
				value = tmc_function_ctrl_req_local_lockout(cdev, tmc, ctrl, req);
				break;
			default:
				dev_info(&tmc->dev, "%s: unhandled request (bRequest: %d)\n", __func__, ctrl->bRequest);
				dev_info(&tmc->dev, "%s: wValue: %d, wIndex: %d, wLength: %d\n", __func__, w_value, w_index, w_length);
		}
	}
	else if(ctrl->bRequestType == (USB_TYPE_CLASS | USB_RECIP_ENDPOINT | USB_DIR_IN)) {
		switch(ctrl->bRequest) {
			case USBTMC_REQUEST_INITIATE_ABORT_BULK_OUT:
				value = tmc_function_ctrl_req_initiate_abort_bulk_out(cdev, tmc, ctrl, req);
				break;
			case USBTMC_REQUEST_CHECK_ABORT_BULK_OUT_STATUS:
				value = tmc_function_ctrl_req_check_abort_bulk_out_status(cdev, tmc, ctrl, req);
				break;
			case USBTMC_REQUEST_INITIATE_ABORT_BULK_IN:
				value = tmc_function_ctrl_req_initiate_abort_bulk_in(cdev, tmc, ctrl, req);
				break;
			case USBTMC_REQUEST_CHECK_ABORT_BULK_IN_STATUS:
				value = tmc_function_ctrl_req_check_abort_bulk_in_status(cdev, tmc, ctrl, req);
				break;
			default:
				dev_info(&tmc->dev, "%s: unhandled request (bRequest: %d)\n", __func__, ctrl->bRequest);
				dev_info(&tmc->dev, "%s: wValue: %d, wIndex: %d, wLength: %d\n", __func__, w_value, w_index, w_length);
		}
	}

	return value;
}

static void tmc_function_suspend(struct usb_function *f)
{
	struct tmc_device *tmc = func_to_tmc(f);
	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);
	return;
}

static void tmc_function_resume(struct usb_function *f)
{
	struct tmc_device *tmc = func_to_tmc(f);
	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);
	return;
}


static bool tmc_req_match(struct usb_function *f, const struct usb_ctrlrequest *ctrl, bool config0)
{
	bool ret = false;

	switch(ctrl->bRequest) {
	case USBTMC_REQUEST_INITIATE_ABORT_BULK_OUT:
	case USBTMC_REQUEST_CHECK_ABORT_BULK_OUT_STATUS:
	case USBTMC_REQUEST_INITIATE_ABORT_BULK_IN:
	case USBTMC_REQUEST_CHECK_ABORT_BULK_IN_STATUS:
	case USBTMC_REQUEST_INITIATE_CLEAR:
	case USBTMC_REQUEST_CHECK_CLEAR_STATUS:
	case USBTMC_REQUEST_GET_CAPABILITIES:
	case USBTMC_REQUEST_INDICATOR_PULSE:
		/* We are 488-compliant so we must handle the following requests as well */
	case USBTMC488_REQUEST_READ_STATUS_BYTE:
	case USBTMC488_REQUEST_REN_CONTROL:
	case USBTMC488_REQUEST_GOTO_LOCAL:
	case USBTMC488_REQUEST_LOCAL_LOCKOUT:
		ret = true;
		break;
	}
	return ret;
}

static void tmc_free_func(struct usb_function *f)
{
	struct tmc_device *tmc = func_to_tmc(f);

	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);

	kfree(tmc);
}

static struct usb_function *tmc_alloc_func(struct usb_function_instance *fi)
{
	struct tmc_device *tmc;
	int ret = -1;

	tmc = kzalloc(sizeof(*tmc), GFP_KERNEL);
	if (tmc == NULL) {
		printk("%s failed\n", __FUNCTION__);
		return ERR_PTR(-ENOMEM);
	}

	/* Initialize the char device */
	device_initialize(&tmc->dev);
	tmc->dev.release = tmc_function_device_release;
	tmc->dev.class = &tmc_class;
	tmc->dev.devt = MKDEV(major, minor);
	ret = dev_set_name(&tmc->dev, "tmc");
	if (ret) {
		printk("Unable to set device name\n");
		return ERR_PTR(ret);
	}

	dev_dbg(&tmc->dev, "%s, %s\n", __FILE__, __func__);

	memset(&tmc->status, 0, sizeof(struct status_response));

	/* Register the function. */
	tmc->func.name = "tmc";
	tmc->func.strings = tmc_function_strings;
	tmc->func.bind = tmc_function_bind;
	tmc->func.unbind = tmc_function_unbind;
	tmc->func.set_alt = tmc_function_set_alt;
	tmc->func.disable = tmc_function_disable;
	tmc->func.setup = tmc_function_setup;
	tmc->func.suspend = tmc_function_suspend;
	tmc->func.resume = tmc_function_resume;
	tmc->func.free_func = tmc_free_func;
	tmc->func.req_match = tmc_req_match;

	tmc->rx_complete = false;
	tmc->tx_pending = false;

	spin_lock_init(&tmc->lock);
	mutex_init(&tmc->lock_tmc_io);
	init_waitqueue_head(&tmc->rx_wait);
	init_waitqueue_head(&tmc->tx_wait);

	tmc->current_rx_req = NULL;
	tmc->current_rx_bytes = 0;
	tmc->current_rx_buf = NULL;

	return &tmc->func;
}

static inline struct f_tmc_opts *to_f_tmc_opts(struct config_item *item)
{
	return container_of(to_config_group(item), struct f_tmc_opts, func_inst.group);
}

static void tmc_attr_release(struct config_item *item)
{
	struct f_tmc_opts *opts = to_f_tmc_opts(item);

	usb_put_function_instance(&opts->func_inst);
}

static struct configfs_item_operations tmc_item_ops = {
	.release = tmc_attr_release,
};

/* ConfigFS USB TMC Capabilities */
static ssize_t f_tmc_bcdUSBTMC_show(struct config_item *item, char *page)
{
	struct f_tmc_opts *opts = to_f_tmc_opts(item);
	int result;

	mutex_lock(&opts->lock);
	result = sprintf(page, "%d", opts->bcdUSBTMC);
	mutex_unlock(&opts->lock);

	return result;
}

static ssize_t f_tmc_bcdUSBTMC_store(struct config_item *item, const char *page, size_t len)
{
	struct f_tmc_opts *opts = to_f_tmc_opts(item);
	int result;
	u16 num;

	mutex_lock(&opts->lock);
	result = kstrtou16(page, 0, &num);
	if(result) {
		mutex_unlock(&opts->lock);
		return result;
	}

	opts->bcdUSBTMC = num;
	result = len;
	mutex_unlock(&opts->lock);

	return result;
}
CONFIGFS_ATTR(f_tmc_, bcdUSBTMC);

static ssize_t f_tmc_bmInterfaceCapabilities_show(struct config_item *item, char *page)
{
	struct f_tmc_opts *opts = to_f_tmc_opts(item);
	int result;

	mutex_lock(&opts->lock);
	result = sprintf(page, "%d", opts->bmInterfaceCapabilities);
	mutex_unlock(&opts->lock);

	return result;
}

static ssize_t f_tmc_bmInterfaceCapabilities_store(struct config_item *item, const char *page, size_t len)
{
	struct f_tmc_opts *opts = to_f_tmc_opts(item);
	int result;
	u8 num;

	mutex_lock(&opts->lock);
	result = kstrtou8(page, 0, &num);
	if(result) {
		mutex_unlock(&opts->lock);
		return result;
	}

	opts->bmInterfaceCapabilities = num;
	result = len;
	mutex_unlock(&opts->lock);

	return result;
}
CONFIGFS_ATTR(f_tmc_, bmInterfaceCapabilities);

static ssize_t f_tmc_bmDeviceCapabilities_show(struct config_item *item, char *page)
{
	struct f_tmc_opts *opts = to_f_tmc_opts(item);
	int result;

	mutex_lock(&opts->lock);
	result = sprintf(page, "%d", opts->bmDeviceCapabilities);
	mutex_unlock(&opts->lock);

	return result;
}

static ssize_t f_tmc_bmDeviceCapabilities_store(struct config_item *item, const char *page, size_t len)
{
	struct f_tmc_opts *opts = to_f_tmc_opts(item);
	int result;
	u8 num;

	mutex_lock(&opts->lock);
	result = kstrtou8(page, 0, &num);
	if(result) {
		mutex_unlock(&opts->lock);
		return result;
	}

	opts->bmDeviceCapabilities = num;
	result = len;
	mutex_unlock(&opts->lock);

	return result;
}
CONFIGFS_ATTR(f_tmc_, bmDeviceCapabilities);

static ssize_t f_tmc_bcdUSB488_show(struct config_item *item, char *page)
{
	struct f_tmc_opts *opts = to_f_tmc_opts(item);
	int result;

	mutex_lock(&opts->lock);
	result = sprintf(page, "%d", opts->bcdUSB488);
	mutex_unlock(&opts->lock);

	return result;
}

static ssize_t f_tmc_bcdUSB488_store(struct config_item *item, const char *page, size_t len)
{
	struct f_tmc_opts *opts = to_f_tmc_opts(item);
	int result;
	u16 num;

	mutex_lock(&opts->lock);
	result = kstrtou16(page, 0, &num);
	if(result) {
		mutex_unlock(&opts->lock);
		return result;
	}

	opts->bcdUSB488 = num;
	result = len;
	mutex_unlock(&opts->lock);

	return result;
}
CONFIGFS_ATTR(f_tmc_, bcdUSB488);

static ssize_t f_tmc_bmInterfaceCapabilities488_show(struct config_item *item, char *page)
{
	struct f_tmc_opts *opts = to_f_tmc_opts(item);
	int result;

	mutex_lock(&opts->lock);
	result = sprintf(page, "%d", opts->bmInterfaceCapabilities488);
	mutex_unlock(&opts->lock);

	return result;
}

static ssize_t f_tmc_bmInterfaceCapabilities488_store(struct config_item *item, const char *page, size_t len)
{
	struct f_tmc_opts *opts = to_f_tmc_opts(item);
	int result;
	u8 num;

	mutex_lock(&opts->lock);
	result = kstrtou8(page, 0, &num);
	if(result) {
		mutex_unlock(&opts->lock);
		return result;
	}

	opts->bmInterfaceCapabilities488 = num;
	result = len;
	mutex_unlock(&opts->lock);

	return result;
}
CONFIGFS_ATTR(f_tmc_, bmInterfaceCapabilities488);

static ssize_t f_tmc_bmDeviceCapabilities488_show(struct config_item *item, char *page)
{
	struct f_tmc_opts *opts = to_f_tmc_opts(item);
	int result;

	mutex_lock(&opts->lock);
	result = sprintf(page, "%d", opts->bmDeviceCapabilities488);
	mutex_unlock(&opts->lock);

	return result;
}

static ssize_t f_tmc_bmDeviceCapabilities488_store(struct config_item *item, const char *page, size_t len)
{
	struct f_tmc_opts *opts = to_f_tmc_opts(item);
	int result;
	u8 num;

	mutex_lock(&opts->lock);
	result = kstrtou8(page, 0, &num);
	if(result) {
		mutex_unlock(&opts->lock);
		return result;
	}

	opts->bmDeviceCapabilities488 = num;
	result = len;
	mutex_unlock(&opts->lock);

	return result;
}
CONFIGFS_ATTR(f_tmc_, bmDeviceCapabilities488);

static ssize_t f_tmc_ren_show(struct config_item *item, char *page)
{
	struct f_tmc_opts *opts = to_f_tmc_opts(item);
	int result;

	mutex_lock(&opts->lock);
	result = sprintf(page, "%d", ren);
	mutex_unlock(&opts->lock);

	return result;
}

static ssize_t f_tmc_ren_store(struct config_item *item, const char *page, size_t len)
{
	struct f_tmc_opts *opts = to_f_tmc_opts(item);
	int result;
	u8 num;

	mutex_lock(&opts->lock);
	result = kstrtou8(page, 0, &num);
	if(result) {
		mutex_unlock(&opts->lock);
		return result;
	}

	ren = num;
	result = len;
	mutex_unlock(&opts->lock);

	return result;
}

CONFIGFS_ATTR(f_tmc_, ren);

static ssize_t f_tmc_interface_show(struct config_item *item, char *page)
{
	return sprintf(page, "%u\n", to_f_tmc_opts(item)->interface);
}

CONFIGFS_ATTR_RO(f_tmc_, interface);

static struct configfs_attribute *tmc_attrs[] = {
	&f_tmc_attr_interface,
	&f_tmc_attr_bcdUSBTMC,
	&f_tmc_attr_bmInterfaceCapabilities,
	&f_tmc_attr_bmDeviceCapabilities,
	&f_tmc_attr_bcdUSB488,
	&f_tmc_attr_bmInterfaceCapabilities488,
	&f_tmc_attr_bmDeviceCapabilities488,
	&f_tmc_attr_ren,
	NULL,
};

static const struct config_item_type tmc_func_type = {
	.ct_item_ops = &tmc_item_ops,
	.ct_attrs = tmc_attrs,
	.ct_owner = THIS_MODULE,
};

static void tmc_free_instance(struct usb_function_instance *fi)
{
	struct f_tmc_opts *opts;

	opts = container_of(fi, struct f_tmc_opts, func_inst);
	kfree(opts);
}

static struct usb_function_instance *tmc_alloc_instance(void)
{
	struct f_tmc_opts *opts;
	dev_t dev;
	int status = -1;

	opts = kzalloc(sizeof(*opts), GFP_KERNEL);
	if (!opts)
		return ERR_PTR(-ENOMEM);
	opts->func_inst.free_func_inst = tmc_free_instance;

	status = class_register(&tmc_class);
	if (status) {
		printk("Could not register tmc class\n");
		return NULL;
	}

	status = alloc_chrdev_region(&dev, 0, 1, "tmc");
	if (status) {
		class_unregister(&tmc_class);
		return NULL;
	}

	major = MAJOR(dev);
	minor = 0;

	config_group_init_type_name(&opts->func_inst.group, "", &tmc_func_type);

	return &opts->func_inst;
}

DECLARE_USB_FUNCTION_INIT(tmc, tmc_alloc_instance, tmc_alloc_func);
MODULE_LICENSE("GPL");
