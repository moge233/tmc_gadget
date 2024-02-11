/*
 * f_usbtmc.c
 *
 *  Created on: Jan 24, 2024
 *      Author: matt
 */

#include <linux/configfs.h>
#include <linux/poll.h>
#include <uapi/linux/usb/tmc.h>

#include "u_tmc.h"

static int major, minor;
static bool ren = false;

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

#ifdef RLSM
static void remote_local_state_machine(struct tmc_device *tmc, int event)
{
	switch(tmc->remote_local_state)
	{
		case LOCS:
			if(tmc->ren)
			{
				switch(event)
				{
					case TMC_EVENT_INITIATE_CLEAR:
					case TMC_EVENT_488_TRIGGER:
					case TMC_EVENT_DEV_DEP_MGS_OUT:
						tmc->remote_local_state = REMS;
						// TODO: GOTO REMOTE
						break;
					case TMC_EVENT_LOCAL_LOCKOUT:
						tmc->remote_local_state = LWLS;
						// TODO: SET LOCKOUT
						break;
					default:
						// No change
						break;
				}
			}
			break;
		case LWLS:
			if(tmc->ren)
			{
				switch(event)
				{
					case TMC_EVENT_INITIATE_CLEAR:
					case TMC_EVENT_488_TRIGGER:
					case TMC_EVENT_DEV_DEP_MGS_OUT:
						tmc->remote_local_state = RWLS;
						// TODO: GOTO REMOTE
					case TMC_EVENT_DETACHED:
						tmc->remote_local_state = LOCS;
						// TODO: RESET LOCKOUT
					default:
						// No change
						break;
				}
			}
			else
			{
				tmc->remote_local_state = LOCS;
				// TODO: RESET LOCKOUT
			}
			break;
		case REMS:
			if(tmc->ren)
			{
				switch(event)
				{
					case TMC_EVENT_RTL:
						tmc->remote_local_state = LOCS;
						break;
					case TMC_EVENT_GO_TO_LOCAL:
					case TMC_EVENT_DETACHED:
						tmc->remote_local_state = LOCS;
						// TODO: GOTO LOCAL
						break;
					case TMC_EVENT_LOCAL_LOCKOUT:
						tmc->remote_local_state = RWLS;
						// TODO: SET LOCKOUT
						break;
					default:
						// No change
						break;
				}
			}
			else
			{
				tmc->remote_local_state = LOCS;
				// TODO: GOTO LOCLA
			}
			break;
		case RWLS:
			if(tmc->ren)
			{
				switch(event)
				{
				case TMC_EVENT_DETACHED:
					tmc->remote_local_state = LOCS;
					// TODO: RESET LOCKOUT
					// TODO: GOTO LOCAL
					break;
				case TMC_EVENT_GO_TO_LOCAL:
					tmc->remote_local_state = LWLS;
					// TODO: GOTO LOCAL
					break;
				default:
					// No change
					break;
				}
			}
			else
			{
				tmc->remote_local_state = LOCS;
				// TODO: RESET LOCKOUT
				// TODO: GOTO LOCAL
			}
			break;
		default:
			// Invalid state -- transition to LOCS
			tmc->remote_local_state = LOCS;
			// TODO: RESET LOCKOUT
			// TODO: GOTO LOCAL
			break;
	}
}
#endif

static void tmc_function_req_complete(struct usb_ep *ep, struct usb_request *req)
{
	// TODO
	return;
}

static void tmc_function_bulk_out_req_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct tmc_device *tmc = (struct tmc_device *)ep->driver_data;
	int status = req->status;
	unsigned long flags = 0;

	spin_lock_irqsave(&tmc->lock, flags);

	switch(status)
	{
		case 0:

			/*
			 * Parse the TMC Bulk OUT Header and verify the contents
			 * per USBTMC Spec. Section 3.2.2.3
			 */
			if(req->actual > 0)
			{
				list_add_tail(&req->list, &tmc->rx_buffers);
			}
			else
			{
				list_add(&req->list, &tmc->rx_reqs);
			}
			break;
		case -ECONNRESET:
		case -ESHUTDOWN:
			list_add(&req->list, &tmc->rx_reqs);
			break;
		case -ECONNABORTED:
			list_add(&req->list, &tmc->rx_reqs);
			break;
		case -EOVERFLOW:
		default:
			printk("Status: %d\n", req->status);
			list_add(&req->list, &tmc->rx_reqs);
	}
	wake_up_interruptible(&tmc->rx_wait);
	spin_unlock_irqrestore(&tmc->lock, flags);

	return;
}

static void tmc_function_bulk_in_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct tmc_device *tmc = ep->driver_data;

	switch (req->status) {
	case 0:
		break;
	default:
		printk("tx err %d\n", req->status);
		fallthrough;
	case -ECONNRESET:		/* unlink */
	case -ESHUTDOWN:		/* disconnect etc */
		break;
	}

	spin_lock(&tmc->lock);
	/* Take the request struct off the active list and put it on the
	 * free list.
	 */
	list_del_init(&req->list);
	list_add(&req->list, &tmc->tx_reqs);
	wake_up_interruptible(&tmc->tx_wait);

	spin_unlock(&tmc->lock);
	return;
}

static void tmc_req_free(struct usb_ep *ep, struct usb_request *req)
{
	if(ep != NULL && req != NULL)
	{
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
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

static void tmc_setup_bulk_out_reqs(struct tmc_device *tmc)
{
	struct usb_request *req;

	while(likely(!list_empty(&tmc->rx_reqs)))
	{
		int error;

		req = container_of(tmc->rx_reqs.next, struct usb_request, list);
		list_del_init(&req->list);

		/* The USB Host sends us whatever amount of data it wants to
		 * so we always set the length field to the full buffer size.
		 * If the amount of data is more than the read() caller asked
		 * for it will be stored in the request buffer until it is
		 * asked for by read().
		 */
		req->length = TMC_BULK_ENDPOINT_SIZE;
		req->complete = tmc_function_bulk_out_req_complete;

		/* here, we unlock, and only unlock, to avoid deadlock. */
		spin_unlock(&tmc->lock);
		error = usb_ep_queue(tmc->bulk_out_ep, req, GFP_ATOMIC);
		spin_lock(&tmc->lock);
		if (error) {
			printk("rx submit --> %d\n", error);
			list_add(&req->list, &tmc->rx_reqs);
			break;
		}
		else if(list_empty(&req->list))
		{
			list_add(&req->list, &tmc->rx_reqs_active);
		}
	}

	return;
}

static int tmc_function_fops_open(struct inode *inode, struct file *file)
{
	struct tmc_device *tmc;

	tmc = container_of(inode->i_cdev, struct tmc_device, cdev);
	file->private_data = tmc;

	return 0;
}

static int tmc_function_fops_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;

	return 0;
}

static ssize_t tmc_function_fops_write(struct file *file, const char __user *buf, size_t len, loff_t *offset)
{
	struct tmc_device *tmc = file->private_data;
	unsigned long flags;
	size_t size;
	size_t bytes_copied = 0;
	struct usb_request *req;
	int value;

	if (len == 0)
		return -EINVAL;

	mutex_lock(&tmc->lock_tmc_io);
	spin_lock_irqsave(&tmc->lock, flags);

	/* Check if there is any available write buffers */
	if (likely(list_empty(&tmc->tx_reqs))) {
		/* Turn interrupts back on before sleeping. */
		spin_unlock_irqrestore(&tmc->lock, flags);

		/*
		 * If write buffers are available check if this is
		 * a NON-Blocking call or not.
		 */
		if (file->f_flags & (O_NONBLOCK|O_NDELAY)) {
			mutex_unlock(&tmc->lock_tmc_io);
			return -EAGAIN;
		}

		/* Sleep until a write buffer is available */
		wait_event_interruptible(tmc->tx_wait, likely(!list_empty(&tmc->tx_reqs)));
		spin_lock_irqsave(&tmc->lock, flags);
	}

	while (likely(!list_empty(&tmc->tx_reqs)) && len) {

		if (len > TMC_BULK_ENDPOINT_SIZE)
			size = TMC_BULK_ENDPOINT_SIZE;
		else
			size = len;

		req = container_of(tmc->tx_reqs.next, struct usb_request, list);
		list_del_init(&req->list);

		req->complete = tmc_function_bulk_in_complete;
		req->length = size;

		/* Check if we need to send a zero length packet. */
		if (len > size)
			/* They will be more TX requests so no yet. */
			req->zero = 0;
		else
			/* If the data amount is not a multiple of the
			 * maxpacket size then send a zero length packet.
			 */
			req->zero = ((len % tmc->bulk_in_ep->maxpacket) == 0);

		/* Don't leave irqs off while doing memory copies */
		spin_unlock_irqrestore(&tmc->lock, flags);

		if (copy_from_user(req->buf, buf, size)) {
			list_add(&req->list, &tmc->tx_reqs);
			mutex_unlock(&tmc->lock_tmc_io);
			return bytes_copied;
		}

		bytes_copied += size;
		len -= size;
		buf += size;

		spin_lock_irqsave(&tmc->lock, flags);

		list_add(&req->list, &tmc->tx_reqs_active);

		/* here, we unlock, and only unlock, to avoid deadlock. */
		spin_unlock(&tmc->lock);
		value = usb_ep_queue(tmc->bulk_in_ep, req, GFP_ATOMIC);
		spin_lock(&tmc->lock);
		if (value) {
			list_move(&req->list, &tmc->tx_reqs);
			spin_unlock_irqrestore(&tmc->lock, flags);
			mutex_unlock(&tmc->lock_tmc_io);
			return -EAGAIN;
		}
	}

	spin_unlock_irqrestore(&tmc->lock, flags);
	mutex_unlock(&tmc->lock_tmc_io);

	if (bytes_copied)
		return bytes_copied;
	else
		return -EAGAIN;
}

static ssize_t tmc_function_fops_read(struct file * file, char __user *buf, size_t len, loff_t *offset)
{
	struct tmc_device *tmc  = file->private_data;
	size_t bytes_copied;
	size_t current_rx_bytes;
	u8 *current_rx_buf;
	struct usb_request *current_rx_req;
	struct usb_request *req;
	unsigned long flags = 0;
	size_t size;

	if(len == 0)
	{
		return -EINVAL;
	}

	mutex_lock(&tmc->lock_tmc_io);
	spin_lock_irqsave(&tmc->lock, flags);

	tmc_setup_bulk_out_reqs(tmc);

	bytes_copied = 0;
	current_rx_req = tmc->current_rx_req;
	current_rx_bytes = tmc->current_rx_bytes;
	current_rx_buf = tmc->current_rx_buf;
	tmc->current_rx_req = NULL;
	tmc->current_rx_bytes = 0;
	tmc->current_rx_buf = NULL;

	/* Check if there is any data in the read buffers. Please note that
	 * current_rx_bytes is the number of bytes in the current rx buffer.
	 * If it is zero then check if there are any other rx_buffers that
	 * are on the completed list. We are only out of data if all rx
	 * buffers are empty.
	 */
	if ((current_rx_bytes == 0) && (likely(list_empty(&tmc->rx_buffers)))) {
		/* Turn interrupts back on before sleeping. */
		spin_unlock_irqrestore(&tmc->lock, flags);

		/*
		 * If no data is available check if this is a NON-Blocking
		 * call or not.
		 */
		if (file->f_flags & (O_NONBLOCK|O_NDELAY)) {
			mutex_unlock(&tmc->lock_tmc_io);
			return -EAGAIN;
		}

		/* Sleep until data is available */
		wait_event_interruptible(tmc->rx_wait, (likely(!list_empty(&tmc->rx_buffers))));
		spin_lock_irqsave(&tmc->lock, flags);
	}

	/* We have data to return then copy it to the caller's buffer.*/
	while ((current_rx_bytes || likely(!list_empty(&tmc->rx_buffers))) && len)
	{
		if (current_rx_bytes == 0)
		{
			req = container_of(tmc->rx_buffers.next, struct usb_request, list);
			list_del_init(&req->list);

			if (req->actual && req->buf)
			{
				current_rx_req = req;
				current_rx_bytes = req->actual;
				current_rx_buf = req->buf;
			}
			else
			{
				list_add(&req->list, &tmc->rx_reqs);
				continue;
			}
		}

		/* Don't leave irqs off while doing memory copies */
		spin_unlock_irqrestore(&tmc->lock, flags);

		if (len > current_rx_bytes)
			size = current_rx_bytes;
		else
			size = len;

		size -= copy_to_user(buf, current_rx_buf, size);
		bytes_copied += size;
		len -= size;
		buf += size;

		spin_lock_irqsave(&tmc->lock, flags);

		/* If we aren't returning all the data left in this RX request
		 * buffer then adjust the amount of data left in the buffer.
		 * Othewise if we are done with this RX request buffer then
		 * requeue it to get any incoming data from the USB host.
		 */
		if (size < current_rx_bytes) {
			current_rx_bytes -= size;
			current_rx_buf += size;
		}
		else
		{
			list_add(&current_rx_req->list, &tmc->rx_reqs);
			current_rx_bytes = 0;
			current_rx_buf = NULL;
			current_rx_req = NULL;
		}
	}

	tmc->current_rx_req = current_rx_req;
	tmc->current_rx_buf = current_rx_buf;
	tmc->current_rx_bytes = current_rx_bytes;

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

	mutex_lock(&tmc->lock_tmc_io);
	spin_lock_irqsave(&tmc->lock, flags);

	tmc_setup_bulk_out_reqs(tmc);
	spin_unlock_irqrestore(&tmc->lock, flags);
	mutex_unlock(&tmc->lock_tmc_io);

	poll_wait(file, &tmc->rx_wait, wait);
	poll_wait(file, &tmc->tx_wait, wait);

	spin_lock_irqsave(&tmc->lock, flags);
	if (likely(!list_empty(&tmc->tx_reqs)))
		ret |= EPOLLOUT | EPOLLWRNORM;

	if (likely(tmc->current_rx_bytes) || likely(!list_empty(&tmc->rx_buffers)))
		ret |= EPOLLIN | EPOLLRDNORM;

	spin_unlock_irqrestore(&tmc->lock, flags);

	return ret;
}

static const struct file_operations f_tmc_fops = {
	.owner		= THIS_MODULE,
	.open		= tmc_function_fops_open,
	.release	= tmc_function_fops_release,
	.write		= tmc_function_fops_write,
	.read		= tmc_function_fops_read,
	.poll		= tmc_function_fops_poll,
	.llseek		= noop_llseek,
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
	struct usb_request *req;
	int ret = -EINVAL;

	opts = fi_to_f_tmc_opts(f->fi);

	mutex_init(&opts->lock);

	/* Get TMC/488 capabilities from ConfigFS */
	tmc->capabilities.bcdUSBTMC = opts->bcdUSBTMC;
	tmc->capabilities.bmInterfaceCapabilities = opts->bmInterfaceCapabilities;
	tmc->capabilities.bmDeviceCapabilities = opts->bmDeviceCapabilities;
	tmc->capabilities.bcdUSB488 = opts->bcdUSB488;
	tmc->capabilities.bmInterfaceCapabilities488 = opts->bmInterfaceCapabilities488;
	tmc->capabilities.bmDeviceCapabilities488 = opts->bmDeviceCapabilities488;

	/* Allocate interface IDs. */
	ret = usb_interface_id(c, f);
	if(ret < 0)
	{
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
	else
	{
		tmc->bulk_in_ep = ep;
	}

	ep = usb_ep_autoconfig(c->cdev->gadget, &tmc_bulk_out_ep_fs);
	if (!ep) {
		printk("Unable to allocate bulk out EP\n");
		return ret;
	}
	else
	{
		tmc->bulk_out_ep = ep;
	}

	ep = usb_ep_autoconfig(c->cdev->gadget, &tmc_interrupt_ep_fs);
	if (!ep) {
		printk("Unable to allocate interrupt EP\n");
		return ret;
	}
	else
	{
		tmc->interrupt_ep = ep;
	}

	/* All endpoints are dual speed */
	tmc_bulk_in_ep_hs.bEndpointAddress = tmc_bulk_in_ep_fs.bEndpointAddress;
	tmc_bulk_out_ep_hs.bEndpointAddress = tmc_bulk_out_ep_fs.bEndpointAddress;
	tmc_interrupt_ep_hs.bEndpointAddress = tmc_interrupt_ep_fs.bEndpointAddress;

	/* Copy descriptors */
	ret = usb_assign_descriptors(f, tmc_descriptors_fs, tmc_descriptors_hs, NULL, NULL);
	if(ret)
	{
		printk("Failed to assign descriptors\n");
		return ret;
	}

	ret = -ENOMEM;
	for (int i = 0; i < 32; i++) {
		req = tmc_req_alloc(tmc->bulk_in_ep, TMC_BULK_ENDPOINT_SIZE, GFP_KERNEL);
		if(!req)
			goto error_bulk_in_reqs;
		list_add(&req->list, &tmc->tx_reqs);
	}

	for (int i = 0; i < 32; i++) {
		req = tmc_req_alloc(tmc->bulk_out_ep, TMC_BULK_ENDPOINT_SIZE, GFP_KERNEL);
		if(!req)
			goto error_bulk_out_reqs;
		list_add(&req->list, &tmc->rx_reqs);
	}

	/* Allocate interrupt request and its buffer */
	tmc->interrupt_req = tmc_req_alloc(tmc->interrupt_ep, TMC_INTR_ENDPOINT_SIZE, GFP_KERNEL);
	if(!tmc->interrupt_req)
	{
		printk("Failed to allocate interrupt req\n");
		goto error_interrupt_req;
	}

	/* Create the char device */
	cdev_init(&tmc->cdev, &f_tmc_fops);
	ret = cdev_device_add(&tmc->cdev, &tmc->dev);
	if(ret)
	{
		printk("Unable to create character device\n");
		goto error_cdev;
	}

	return 0;

error_cdev:
	cdev_del(&tmc->cdev);

error_interrupt_req:
	tmc_req_free(tmc->interrupt_ep, tmc->interrupt_req);

error_bulk_out_reqs:
	while (!list_empty(&tmc->rx_reqs)) {
		req = container_of(tmc->rx_reqs.next, struct usb_request, list);
		list_del(&req->list);
		tmc_req_free(tmc->bulk_out_ep, req);
	}

error_bulk_in_reqs:
	while (!list_empty(&tmc->tx_reqs)) {
		req = container_of(tmc->tx_reqs.next, struct usb_request, list);
		list_del(&req->list);
		tmc_req_free(tmc->bulk_in_ep, req);
	}

	usb_free_all_descriptors(f);
	return ret;
}

static void tmc_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct tmc_device *tmc = func_to_tmc(f);
	struct usb_request *req;

	cdev_device_del(&tmc->cdev, &tmc->dev);

	/* Free all memory for this driver. */
	while (!list_empty(&tmc->tx_reqs)) {
		req = container_of(tmc->tx_reqs.next, struct usb_request,
				list);
		list_del(&req->list);
		tmc_req_free(tmc->bulk_in_ep, req);
	}

	if (tmc->current_rx_req != NULL)
		tmc_req_free(tmc->bulk_out_ep, tmc->current_rx_req);

	while (!list_empty(&tmc->rx_reqs)) {
		req = container_of(tmc->rx_reqs.next,
				struct usb_request, list);
		list_del(&req->list);
		tmc_req_free(tmc->bulk_out_ep, req);
	}

	while (!list_empty(&tmc->rx_buffers)) {
		req = container_of(tmc->rx_buffers.next,
				struct usb_request, list);
		list_del(&req->list);
		tmc_req_free(tmc->bulk_out_ep, req);
	}

	if(tmc->interrupt_req != NULL)
	{
		tmc_req_free(tmc->interrupt_ep, tmc->interrupt_req);
	}

	usb_free_all_descriptors(f);
}

static int set_tmc_interface(struct tmc_device *tmc)
{
	int			result = 0;

	tmc->bulk_in_ep->desc = ep_desc(tmc->gadget, &tmc_bulk_in_ep_fs, &tmc_bulk_in_ep_hs,
				NULL);
	tmc->bulk_in_ep->driver_data = tmc;

	tmc->bulk_out_ep->desc = ep_desc(tmc->gadget, &tmc_bulk_out_ep_fs,
				    &tmc_bulk_out_ep_hs, NULL);
	tmc->bulk_out_ep->driver_data = tmc;

	tmc->interrupt_ep->desc = ep_desc(tmc->gadget, &tmc_interrupt_ep_fs,
				    &tmc_interrupt_ep_hs, NULL);
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
	int			result = 0;

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

	if(!alt)
		ret = set_interface(tmc, interface);

	return ret;
}

static void tmc_function_disable(struct usb_function *f)
{
	struct tmc_device *tmc = func_to_tmc(f);

	tmc_reset_interface(tmc);
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

	if(ctrl->bRequestType == (USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_IN))
	{
		switch (ctrl->bRequest) {
			case USBTMC_REQUEST_INITIATE_CLEAR:
				// TODO: Need to stall the bulk out endpoint here
				value = usb_ep_set_halt(tmc->bulk_out_ep);
				if(value < 0)
				{
					printk("STALL on bulk out endpoint failed\n");
				}
				tmc->capabilities.USBTMC_status = USBTMC_STATUS_SUCCESS;
				value = sizeof(tmc->capabilities.USBTMC_status);
				memcpy(req->buf, (void *) &(tmc->capabilities.USBTMC_status), value);
				break;
			case USBTMC_REQUEST_CHECK_CLEAR_STATUS:
				tmc->capabilities.USBTMC_status = USBTMC_STATUS_SUCCESS;
				value = sizeof(tmc->capabilities.USBTMC_status);
				memcpy(req->buf, (void *) &(tmc->capabilities.USBTMC_status), value);
				break;
			case USBTMC_REQUEST_GET_CAPABILITIES:
				tmc->capabilities.USBTMC_status = USBTMC_STATUS_SUCCESS;
				if(w_length != sizeof(struct capability_response))
				{
					tmc->capabilities.USBTMC_status = USBTMC_STATUS_FAILED;
				}

				value = min_t(unsigned short, w_length, sizeof(struct capability_response));
				memcpy(req->buf, (void *) &(tmc->capabilities), value);
				break;
			case USBTMC_REQUEST_INDICATOR_PULSE:
				if(tmc->capabilities.bmInterfaceCapabilities & USBTMC488_CAPABILITY_488_DOT_2) // Is this the same as the indicator pulse?
				{
					tmc->capabilities.USBTMC_status = USBTMC_STATUS_SUCCESS;
					value = sizeof(tmc->capabilities.USBTMC_status);
					memcpy(req->buf, (void *) &(tmc->capabilities.USBTMC_status), value);
				}
				else
				{
					// TODO: Unhandled request
				}
				break;
			case USBTMC488_REQUEST_READ_STATUS_BYTE:
				// Attempt to send the status byte via the interrupt endpoint
				// per USBTMC 4.3.1.2
				if(tmc->capabilities.bmDeviceCapabilities488 & USBTMC488_CAPABILITY_SR1)
				{
					// USBTMC/USB488 3.4.2 Table 7
					tmc->interrupt.tag = (u8) (USB_DIR_IN | w_value);
					tmc->interrupt.status_byte = tmc->status_byte;

					value = sizeof(tmc->interrupt);

					tmc->interrupt_req->length = value;
					tmc->interrupt_req->zero = 0;
					tmc->interrupt_req->complete = tmc_function_req_complete;
					tmc->interrupt_req->context = tmc;
					memcpy(tmc->interrupt_req->buf, (void *) &(tmc->interrupt), value);

					// Attempt to send it
					value = usb_ep_queue(tmc->interrupt_ep, tmc->interrupt_req, GFP_ATOMIC);

					// Handle the control endpoint now
					if(!value)
					{
						tmc->status.USBTMC_status = USBTMC_STATUS_SUCCESS;
						tmc->status.tag = (u8) w_value;
						tmc->status.status_byte = 0;

						value = min_t(unsigned short, w_length, sizeof(struct status_response));
						memcpy(req->buf, (void *) &(tmc->status), value);
					}
				}
				break;
			case USBTMC488_REQUEST_REN_CONTROL:
				if(tmc->capabilities.bmDeviceCapabilities488 & USBTMC488_CAPABILITY_RL1)
				{
					tmc->ren = (w_value != 0) ? true : false;
					ren = tmc->ren;
					tmc->capabilities.USBTMC_status = USBTMC_STATUS_SUCCESS;
					value = sizeof(tmc->capabilities.USBTMC_status);
					memcpy(req->buf, (void *) &(tmc->capabilities.USBTMC_status), value);
				}
				else
				{
					// TODO: Need to stall here
				}
				break;
			case USBTMC488_REQUEST_GOTO_LOCAL:
				if(tmc->capabilities.bmDeviceCapabilities488 & USBTMC488_CAPABILITY_RL1)
				{
					u8 status = USBTMC_STATUS_SUCCESS;
					value = sizeof(status);
					memcpy(req->buf, (void *) &status, value);
				}
				else
				{
					// TODO: Stall the control endpoint
				}
				break;
			case USBTMC488_REQUEST_LOCAL_LOCKOUT:
				if(tmc->capabilities.bmDeviceCapabilities488 & USBTMC488_CAPABILITY_RL1)
				{
					u8 status = USBTMC_STATUS_SUCCESS;
					value = sizeof(status);
					memcpy(req->buf, (void *) &status, value);
				}
				break;
			default:
				printk("UHANDLED REQUEST");
				printk("wValue: %d, wIndex: %d, wLength: %d\n", w_value, w_index, w_length);
		}
	}
	else if(ctrl->bRequestType == (USB_TYPE_CLASS | USB_RECIP_ENDPOINT | USB_DIR_IN))
	{
		switch(ctrl->bRequest)
		{
			// TODO
			case USBTMC_REQUEST_INITIATE_ABORT_BULK_OUT:
				printk("INITIATE ABORT BULK OUT\n");
				printk("bTag: %d\n", ctrl->wValue);
				break;
			case USBTMC_REQUEST_CHECK_ABORT_BULK_OUT_STATUS:
				printk("INITIATE ABORT BULK OUT STATUS\n");
				printk("bTag: %d\n", ctrl->wValue);
				break;
			case USBTMC_REQUEST_INITIATE_ABORT_BULK_IN:
				printk("INITIATE ABORT BULK IN\n");
				printk("bTag: %d\n", ctrl->wValue);
				break;
			case USBTMC_REQUEST_CHECK_ABORT_BULK_IN_STATUS:
				printk("INITIATE ABORT BULK IN STATUS\n");
				printk("bTag: %d\n", ctrl->wValue);
				break;
			default:
				printk("UHANDLED REQUEST");
				printk("wValue: %d, wIndex: %d, wLength: %d\n", w_value, w_index, w_length);
		}
	}

	if(value >= 0)
	{
		req->zero = 0;
		req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if(value < 0)
		{
			printk("Error sending ctrl endpoint response\n");
		}
	}
	return value;
}

static bool tmc_req_match(struct usb_function *f, const struct usb_ctrlrequest *ctrl, bool config0)
{
	return true;
}

static void tmc_free_func(struct usb_function *f)
{
	struct tmc_device *tmc = func_to_tmc(f);
	kfree(tmc);
}

static struct usb_function *tmc_alloc_func(struct usb_function_instance *fi)
{
	struct tmc_device *tmc;
	int ret = -1;

	tmc = kzalloc(sizeof(*tmc), GFP_KERNEL);
	if (tmc == NULL)
	{
		printk("%s failed\n", __FUNCTION__);
		return ERR_PTR(-ENOMEM);
	}

	/* Initialize the char device */
	device_initialize(&tmc->dev);
	tmc->dev.release = tmc_function_device_release;
	tmc->dev.class = &tmc_class;
	tmc->dev.devt = MKDEV(major, minor);
	ret = dev_set_name(&tmc->dev, "tmc");
	if (ret)
	{
		printk("Unable to set device name\n");
		return ERR_PTR(ret);
	}

	memset(&tmc->status, 0, sizeof(struct status_response));
	memset(&tmc->interrupt, 0, sizeof(struct interrupt_response));

	tmc->status_byte = 2;	// TODO: Where would the status byte eventually come from? Maybe a configfs entry?

	/* TODO: Add this to the sysfs */
	tmc->state = TMC_STATE_DISCONNECTED;

	/* Register the function. */
	tmc->func.name = "tmc";
	tmc->func.strings = tmc_function_strings;
	tmc->func.bind = tmc_function_bind;
	tmc->func.unbind = tmc_function_unbind;
	tmc->func.set_alt = tmc_function_set_alt;
	tmc->func.disable = tmc_function_disable;
	tmc->func.setup = tmc_function_setup;
	tmc->func.free_func = tmc_free_func;
	tmc->func.req_match = tmc_req_match;

	INIT_LIST_HEAD(&tmc->tx_reqs);
	INIT_LIST_HEAD(&tmc->rx_reqs);
	INIT_LIST_HEAD(&tmc->rx_buffers);
	INIT_LIST_HEAD(&tmc->tx_reqs_active);
	INIT_LIST_HEAD(&tmc->rx_reqs_active);

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
	if(result)
	{
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
	if(result)
	{
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
	if(result)
	{
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
	if(result)
	{
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
	if(result)
	{
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
	if(result)
	{
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
	if(result)
	{
		mutex_unlock(&opts->lock);
		return result;
	}

	ren = (bool) num;
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
	if (status)
	{
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
