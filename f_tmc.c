/*
 * f_usbtmc.c
 *
 *  Created on: Jan 24, 2024
 *      Author: matt
 */

#include <linux/errno.h>
#include <linux/configfs.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/usb/g_tmc.h>

#include "u_tmc.h"

static const bool USE_POLL_FOR_HEADER = true;

static int32_t major = 0;
static int32_t minor = 0;

static u8 g_ren = 0;
static u8 g_status_byte = 0;
static u8 g_termchar = 0;
static enum tmc_gadget_remote_local_state g_rlstate = LOCS;

static struct usb_endpoint_descriptor tmc_gadget_bulk_in_ep_fs = {
	.bLength			= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor tmc_gadget_bulk_out_ep_fs = {
	.bLength			= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_OUT,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor tmc_gadget_interrupt_ep_fs = {
	.bLength			= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_XFER_INT,
	.bInterval			= 1,
};

static struct usb_interface_descriptor tmc_gadget_interface_descriptor = {
	.bLength				= USB_DT_INTERFACE_SIZE,
	.bDescriptorType		= USB_DT_INTERFACE,
	/* .bInterfaceNumber	= DYNAMIC, */
	.bNumEndpoints			= TMC_GADGET_NUM_ENDPOINTS,
	.bInterfaceClass		= USB_CLASS_APP_SPEC,
	.bInterfaceSubClass		= TMC_GADGET_SUBCLASS,
	.bInterfaceProtocol		= 1,
	/* .iInterface			= DYNAMIC, */
};

static struct usb_descriptor_header *tmc_gadget_descriptors_fs[] = {
		(struct usb_descriptor_header *) &tmc_gadget_interface_descriptor,
		(struct usb_descriptor_header *) &tmc_gadget_bulk_in_ep_fs,
		(struct usb_descriptor_header *) &tmc_gadget_bulk_out_ep_fs,
		(struct usb_descriptor_header *) &tmc_gadget_interrupt_ep_fs,
		NULL
};

static struct usb_endpoint_descriptor tmc_gadget_bulk_in_ep_hs = {
	.bLength			= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= cpu_to_le16(512),
};

static struct usb_endpoint_descriptor tmc_gadget_bulk_out_ep_hs = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= cpu_to_le16(512),
};

static struct usb_endpoint_descriptor tmc_gadget_interrupt_ep_hs = {
	.bLength			= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bmAttributes		= USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize		= cpu_to_le16(16),
	.bInterval			= 1,
};

static struct usb_descriptor_header *tmc_gadget_descriptors_hs[] = {
		(struct usb_descriptor_header *) &tmc_gadget_interface_descriptor,
		(struct usb_descriptor_header *) &tmc_gadget_bulk_in_ep_hs,
		(struct usb_descriptor_header *) &tmc_gadget_bulk_out_ep_hs,
		(struct usb_descriptor_header *) &tmc_gadget_interrupt_ep_hs,
		NULL
};

static struct usb_string tmc_gadget_en_us_strings[] = {
	{ }
};

static struct usb_gadget_strings tmc_gadget_string_table = {
	.language = 0x0409,	/* en-us */
	.strings = tmc_gadget_en_us_strings,
};

static struct usb_gadget_strings *tmc_gadget_strings[] = {
	&tmc_gadget_string_table,
	NULL,
};

static void tmc_gadget_req_free(struct usb_ep *ep, struct usb_request *req)
{
	if (ep != NULL && req != NULL)
	{
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static struct usb_request *tmc_gadget_req_alloc(struct usb_ep *ep, unsigned len, gfp_t gfp_flags)
{
	struct usb_request	*req;

	req = usb_ep_alloc_request(ep, gfp_flags);

	if (req != NULL)
	{
		req->length = len;
		req->buf = kmalloc(len, gfp_flags);
		if (req->buf == NULL)
		{
			usb_ep_free_request(ep, req);
			return NULL;
		}
	}

	return req;
}

static int tmc_gadget_try_halt_bulk_out_endpoint(struct tmc_device *tmc)
{

	uint16_t retry_count;
	int err;
	retry_count = 1000;
	err = 0;
	while (((err = usb_ep_set_halt(tmc->bulk_out_ep)) == -EAGAIN) && (retry_count > 0))
	{
		--retry_count;
	}

	return err;
}

static int tmc_gadget_ioctl_write_stb(struct tmc_device *tmc, void __user *arg)
{
	if (get_user(g_status_byte, (__u8 __user *)arg)) // @suppress("Type cannot be resolved")
	{
		return -EFAULT;
	}

	return 0;
}

static int tmc_gadget_ioctl_read_stb(struct tmc_device *tmc, void __user *arg)
{
	if (put_user(g_status_byte, (__u8 __user *)arg))// @suppress("Type cannot be resolved")
	{
		return -EFAULT;
	}

	return 0;
}

static int tmc_gadget_ioctl_get_header(struct tmc_device *tmc, void __user *arg)
{
	if (access_ok(arg, GADGET_TMC_HEADER_SIZE))
	{
		return copy_to_user(arg, &tmc->current_header, GADGET_TMC_HEADER_SIZE);
	}

	return -EFAULT;
}

static int tmc_gadget_rl_state_machine(struct tmc_device *tmc, enum tmc_gadget_rl_state_machine_event event)
{
	switch(g_rlstate)
	{
		case LOCS:
			if (g_ren)
			{
				switch(event)
				{
					case TMC_EVENT_INITIATE_CLEAR:
					case TMC_EVENT_TRIGGER:
					case TMC_EVENT_DEV_DEP_MSG_OUT:
						g_rlstate = REMS;
						break;
					case TMC_EVENT_LOCAL_LOCKOUT:
						g_rlstate = LWLS;
						break;
					default:
						break;
				}
			}
			break;
		case LWLS:
			switch(event)
			{
				case TMC_EVENT_INITIATE_CLEAR:
				case TMC_EVENT_TRIGGER:
				case TMC_EVENT_DEV_DEP_MSG_OUT:
					g_rlstate = RWLS;
					break;
				default:
					break;
			}
			break;
		case REMS:
			switch(event)
			{
				case TMC_EVENT_GOTO_LOCAL:
				case TMC_EVENT_BUS_ACTIVITY:
					g_rlstate = LOCS;
					break;
				case TMC_EVENT_LOCAL_LOCKOUT:
					g_rlstate = RWLS;
					break;
				default:
					break;
			}
			break;
		case RWLS:
			switch(event)
			{
				case TMC_EVENT_GOTO_LOCAL:
				case TMC_EVENT_BUS_ACTIVITY:
					g_rlstate = LWLS;
					break;
				default:
					break;
			}
			break;
	}

	return 0;
}

static void tmc_gadget_bulk_out_req_complete(struct usb_ep *ep, struct usb_request *req)
{

	struct tmc_device *tmc = (struct tmc_device *)ep->driver_data;
	int status = req->status;
	unsigned long flags = 0;

	spin_lock_irqsave(&tmc->lock, flags);

	switch(status)
	{
		case 0:
			if (req->actual > 0)
			{
				/*
				 * Parse the TMC Bulk OUT Header and verify the contents
				 * per USBTMC Spec. Section 3.2.2.3
				 */
				if (!tmc->current_rx_bytes_remaining)
				{

					if (req->actual < TMC_GADGET_HEADER_SIZE)
					{
						/*
						 * Halt the bulk out endpoint and discard any received data
						 * There will not be any requests queued up at this point
						 */
						usb_ep_set_halt(tmc->bulk_out_ep);
						dev_err(&tmc->dev, "%s: message length less than TMC header size", __func__);
						break;
					}

					if (tmc->new_header_required && !tmc->new_header_available)
					{
						wake_up(&tmc->header_wait);
					}
					tmc->new_header_required = false;
					tmc->new_header_available = true;
					memcpy(&tmc->current_header, req->buf, TMC_GADGET_HEADER_SIZE);

					/*
					 * Check the MsgID value to make sure it is recognized and supported
					 */
					if ((tmc->current_header.MsgID != TMC_DEV_DEP_MSG_OUT) &&
							(tmc->current_header.MsgID != TMC_REQUEST_DEV_DEP_MSG_IN) &&
							(tmc->current_header.MsgID != TMC_VENDOR_SPECIFIC_OUT) &&
							(tmc->current_header.MsgID != TMC_REQUEST_VENDOR_SPECIFIC_IN) &&
							(tmc->current_header.MsgID != TMC_488_TRIGGER))
					{
						/*
						 * Halt the bulk out endpoint and discard any received data
						 * There will not be any requests queued up at this point
						 */
						usb_ep_set_halt(tmc->bulk_out_ep);
						dev_err(&tmc->dev, "unrecognized header MsgID value (%u)\n", tmc->current_header.MsgID);
						break;
					}

					if ((tmc->current_header.MsgID == TMC_REQUEST_DEV_DEP_MSG_IN) ||
							(tmc->current_header.MsgID == TMC_REQUEST_VENDOR_SPECIFIC_IN))
					{

						if (USE_POLL_FOR_HEADER)
						{
							tmc->current_rx_bytes_remaining = 0;
							tmc->current_rx_bytes = 0;
							tmc->current_rx_buf = NULL;
							tmc->new_header_required = true;
						}
						else
						{
							tmc->current_rx_bytes_remaining = TMC_GADGET_HEADER_SIZE;
						}
					}
					else
					{
						if (USE_POLL_FOR_HEADER)
						{
							tmc->current_rx_bytes_remaining = tmc->current_header.TransferSize;
							tmc->current_rx_bytes = req->actual - TMC_GADGET_HEADER_SIZE;
							tmc->current_rx_buf = req->buf + TMC_GADGET_HEADER_SIZE;
						}
						else
						{
							tmc->current_rx_bytes_remaining = TMC_GADGET_HEADER_SIZE + tmc->current_header.TransferSize;
						}
					}

					/*
					 * Check the transfer attributes for an End of Message character or Termination Character
					 */
					if ((tmc->current_header.MsgID == TMC_REQUEST_DEV_DEP_MSG_IN) ||
							(tmc->current_header.MsgID == TMC_REQUEST_VENDOR_SPECIFIC_IN))
					{
						if(tmc->current_header.bmTransferAttributes & TMC_XFER_TERM_CHAR_ENABLED)
						{
							g_termchar = tmc->current_header.TermChar;
						}
						else
						{
							g_termchar = 0;
						}
					}
					/*
					 * The total number of bytes sent must be a multiple of 4 so check
					 * for alignment bytes here
					 */
					tmc->current_rx_bytes_remaining += (tmc->current_rx_bytes_remaining % 4)
									? (4 - (tmc->current_rx_bytes_remaining % 4))
									: 0;
				}
				else
				{
					if (USE_POLL_FOR_HEADER)
					{
						tmc->current_rx_bytes = req->actual;
						tmc->current_rx_buf = req->buf;
					}
				}
				tmc->rx_complete = true;
				tmc->bulk_out_queued = false;
			}
			else
			{
				tmc->rx_complete = false;
			}
			break;
		case -ECONNRESET:
			tmc->new_header_required = true;
			tmc->rx_complete = false;
			tmc->connection_reset = true;
			dev_err(&tmc->dev, "%s: ECONNRESET, status: %d\n", __func__, status);
			break;
		case -ESHUTDOWN:
			tmc->new_header_required = true;
			tmc->rx_complete = false;
			tmc->connection_reset = true;
			dev_err(&tmc->dev, "%s: ESHUTDOWN, status: %d\n", __func__, status);
			break;
		case -ECONNABORTED:
			dev_err(&tmc->dev, "%s: ECONNABORTED, status: %d\n", __func__, status);
			break;
		case -EOVERFLOW:
			dev_err(&tmc->dev, "%s: EOVERFLOW, status: %d\n", __func__, status);
			break;
		case -EPIPE:
			dev_err(&tmc->dev, "%s: EPIPE, status: %d\n", __func__, status);
			break;
		default:
			dev_err(&tmc->dev, "%s: default, status: %d\n", __func__, status);
			break;
	}

	wake_up_interruptible(&tmc->rx_wait);
	spin_unlock_irqrestore(&tmc->lock, flags);

	return;
}

static void tmc_gadget_bulk_in_req_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct tmc_device *tmc = ep->driver_data;
	int status = req->status;

	switch (req->status)
	{
		case 0:
			tmc->bulk_in_queued = false;
			break;
		case -ECONNRESET:		/* unlink */
			tmc->tx_pending = false;
			tmc->connection_reset = true;
			dev_err(&tmc->dev, "%s: ECONNRESET, status: %d\n", __func__, status);
			break;
		case -ESHUTDOWN:		/* disconnect etc */
			tmc->tx_pending = false;
			tmc->connection_reset = true;
			dev_err(&tmc->dev, "%s: ESHUTDOWN, status: %d\n", __func__, status);
			break;
		default:
			dev_err(&tmc->dev, "%s: default, status: %d\n", __func__, status);
			break;
	}

	spin_lock(&tmc->lock);
	tmc->tx_pending = false;
	wake_up_interruptible(&tmc->tx_wait);
	spin_unlock(&tmc->lock);

	return;
}

static void tmc_gadget_interrupt_req_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct tmc_device *tmc = ep->driver_data;
	tmc->intr_in_queued = false;
	return;
}

static int tmc_gadget_setup_bulk_out_req(struct tmc_device *tmc)
{
	struct usb_request *req;
	int error = 0;

	req = tmc->bulk_out_req;

	/* The USB Host sends us whatever amount of data it wants to
	 * so we always set the length field to the full buffer size.
	 */
	req->length = TMC_GADGET_BULK_ENDPOINT_SIZE;
	req->complete = tmc_gadget_bulk_out_req_complete;

	/* here, we unlock, and only unlock, to avoid deadlock. */
	spin_unlock(&tmc->lock);
	error = usb_ep_queue(tmc->bulk_out_ep, req, GFP_ATOMIC);
	spin_lock(&tmc->lock);

	if (error)
	{
		dev_err(&tmc->dev, "%s: could not queue bulk out endpoint (error %d)\n", __func__, error);
	}
	else
	{
		tmc->bulk_out_queued = true;
	}

	return error;
}

static int tmc_gadget_fops_open(struct inode *inode, struct file *file)
{
	struct tmc_device *tmc;

	tmc = container_of(inode->i_cdev, struct tmc_device, cdev);
	file->private_data = tmc;

	return 0;
}

static int tmc_gadget_fops_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;

	return 0;
}

static ssize_t tmc_gadget_fops_write(struct file *file, const char __user *buf, size_t len, loff_t *offset) // @suppress("Type cannot be resolved")
{
	struct tmc_device *tmc = file->private_data;
	unsigned long flags = 0;

	if (len == 0)
	{
		return -EINVAL;
	}

	if (tmc->is_shutdown)
	{
		return -ESHUTDOWN;
	}

	mutex_lock(&tmc->io_lock);
	spin_lock_irqsave(&tmc->lock, flags);

	if (tmc->tx_pending)
	{
		/* Turn interrupts back on before sleeping. */
		spin_unlock_irqrestore(&tmc->lock, flags);

		/*
		 * If write buffers are available check if this is
		 * a NON-Blocking call or not.
		 */
		if (file->f_flags & (O_NONBLOCK | O_NDELAY))
		{
			mutex_unlock(&tmc->io_lock);
			return -EAGAIN;
		}

		wait_event_interruptible(tmc->tx_wait, (false == tmc->tx_pending)); // @suppress("Type cannot be resolved")
		spin_lock_irqsave(&tmc->lock, flags);
	}

	ssize_t bytes_copied = 0;
	struct usb_request *req = NULL;
	bool send_term_char = false;
	bool header_required = true;

	tmc->current_tx_bytes_remaining = TMC_GADGET_HEADER_SIZE + len;
	tmc->current_tx_buf = (uint8_t *)buf;

	req = tmc->bulk_in_req;
	req->complete = tmc_gadget_bulk_in_req_complete;

	do
	{
		size_t adjustment = 0;
		uint8_t *response_ptr = req->buf;
		size_t room_left = TMC_GADGET_BULK_ENDPOINT_SIZE;
		size_t write_count = 0;

		if (header_required)
		{
			struct tmc_header response_header;
			memset(&response_header, 0, sizeof(struct tmc_header));

			response_header.MsgID = TMC_DEV_DEP_MSG_IN;
			response_header.bTag = tmc->current_header.bTag;
			response_header.bTagInverse = tmc->current_header.bTagInverse;
			if (tmc->current_header.bmTransferAttributes & GADGET_TMC_XFER_ATTRS_TERM_CHAR_ENABLED)
			{
				response_header.TransferSize = len + 1;
				response_header.TermChar = tmc->current_header.TermChar;
				response_header.bmTransferAttributes |= 2;
			}
			else
			{
				response_header.TransferSize = len;
				response_header.TermChar = 0;
			}

			if (tmc->current_tx_bytes_remaining % 4)
			{
				adjustment = 4 - (tmc->current_tx_bytes_remaining % 4);
				tmc->current_tx_bytes_remaining += adjustment;
			}

			response_header.bmTransferAttributes |= 1; // EOM

			memcpy(response_ptr, &response_header, TMC_GADGET_HEADER_SIZE);

			response_ptr += TMC_GADGET_HEADER_SIZE;
			write_count += TMC_GADGET_HEADER_SIZE;
			tmc->current_tx_bytes_remaining -= TMC_GADGET_HEADER_SIZE;
			room_left -= TMC_GADGET_HEADER_SIZE;
			header_required = false;
		}

		size_t copy_count = room_left;
		if (tmc->current_tx_bytes_remaining <= copy_count)
		{
			copy_count = tmc->current_tx_bytes_remaining;
		}

		/* Don't leave irqs off while doing memory copies */
		spin_unlock_irqrestore(&tmc->lock, flags);

		if (copy_from_user(response_ptr, tmc->current_tx_buf, copy_count))
		{
			tmc->tx_pending = false;
			mutex_unlock(&tmc->io_lock);
			return -EIO;
		}

		bytes_copied += copy_count;
		tmc->current_tx_bytes_remaining -= copy_count;
		tmc->current_tx_buf += copy_count;
		response_ptr += copy_count;
		write_count += copy_count;
		room_left -= copy_count;

		if ((tmc->current_tx_bytes_remaining == 0) &&
				(tmc->current_header.bmTransferAttributes & GADGET_TMC_XFER_ATTRS_TERM_CHAR_ENABLED))
		{
			if (room_left)
			{
				*(response_ptr - adjustment) = (tmc->current_header.TermChar);
			}
			else
			{
				// Need to send another transaction with the termchar
				send_term_char = true;
			}

			if ((adjustment == 0) && room_left)
			{
				write_count += 1;
			}
		}

		req->length = write_count;

		/* Check if we need to send a zero length packet. */
		if (write_count < TMC_GADGET_BULK_ENDPOINT_SIZE)
		{
			/* They will be more TX requests so no yet. */
			req->zero = 0;
		}
		else
		{
			/* If the data amount is not a multiple of the
			 * maxpacket size then send a zero length packet.
			 */
			req->zero = ((write_count % TMC_GADGET_BULK_ENDPOINT_SIZE) == 0);
		}

		spin_lock_irqsave(&tmc->lock, flags);

		/* here, we unlock, and only unlock, to avoid deadlock. */
		spin_unlock(&tmc->lock);
		tmc->previous_bulk_in_tag = tmc->current_header.bTag;
		int error = usb_ep_queue(tmc->bulk_in_ep, req, GFP_ATOMIC);
		spin_lock(&tmc->lock);
		if (error)
		{
			tmc->tx_pending = false;
			tmc->bulk_in_queued = false;
			spin_unlock_irqrestore(&tmc->lock, flags);
			mutex_unlock(&tmc->io_lock);
			return -EIO;
		}
		else
		{
			tmc->tx_pending = true;
			tmc->bulk_in_queued = true;
		}

		spin_unlock_irqrestore(&tmc->lock, flags);
		wait_event_interruptible(tmc->tx_wait, (false == tmc->tx_pending)); // @suppress("Type cannot be resolved")
		spin_lock_irqsave(&tmc->lock, flags);

	} while (tmc->current_tx_bytes_remaining);

	if (send_term_char)
	{
		// Need to send a term char as its own transfer
		uint8_t termchar_buffer[] = { g_termchar };
		memcpy((uint8_t *)req->buf, termchar_buffer, 1);
		req->length = 1;

		spin_unlock(&tmc->lock);
		int error = usb_ep_queue(tmc->bulk_in_ep, req, GFP_ATOMIC);
		spin_lock(&tmc->lock);
		if (error)
		{
			tmc->tx_pending = false;
			tmc->bulk_in_queued = false;
			spin_unlock_irqrestore(&tmc->lock, flags);
			mutex_unlock(&tmc->io_lock);
			return -EIO;
		}
		else
		{
			tmc->tx_pending = true;
			tmc->bulk_in_queued = true;
		}

		spin_unlock_irqrestore(&tmc->lock, flags);
		wait_event_interruptible(tmc->tx_wait, (false == tmc->tx_pending)); // @suppress("Type cannot be resolved")
		spin_lock_irqsave(&tmc->lock, flags);
	}

	spin_unlock_irqrestore(&tmc->lock, flags);
	mutex_unlock(&tmc->io_lock);

	return bytes_copied;
}

static ssize_t tmc_gadget_fops_read(struct file * file, char __user *buf, size_t len, loff_t *offset) // @suppress("Type cannot be resolved")
{
	struct tmc_device *tmc  = file->private_data;
	size_t bytes_copied;
	size_t current_rx_bytes;
	uint8_t *current_rx_buf;
	struct usb_request *req;
	unsigned long flags = 0;
	size_t size;

	if (len == 0)
	{
		return -EINVAL;
	}

	if (tmc->is_shutdown)
	{
		return -ESHUTDOWN;
	}

	mutex_lock(&tmc->io_lock);
	spin_lock_irqsave(&tmc->lock, flags);

	bytes_copied = 0;
	current_rx_bytes = tmc->current_rx_bytes;

	if (!current_rx_bytes && !tmc->new_header_available)
	{
		int err = tmc_gadget_setup_bulk_out_req(tmc);
		if (err)
		{
			return (ssize_t) err;
		}
	}

	if ((current_rx_bytes == 0) && !tmc->rx_complete)
	{
		/* Turn interrupts back on before sleeping. */
		spin_unlock_irqrestore(&tmc->lock, flags);

		/*
		 * If no data is available check if this is a NON-Blocking
		 * call or not.
		 */
		if (file->f_flags & (O_NONBLOCK | O_NDELAY))
		{
			mutex_unlock(&tmc->io_lock);
			return -EAGAIN;
		}

		/* Sleep until data is available */
		wait_event_interruptible(tmc->rx_wait, ((true == tmc->rx_complete) || (true == tmc->connection_reset))); // @suppress("Type cannot be resolved")
		spin_lock_irqsave(&tmc->lock, flags);
	}

	/* We have data to return then copy it to the caller's buffer.*/
	while ((current_rx_bytes || tmc->rx_complete) && len)
	{
		if (USE_POLL_FOR_HEADER)
		{
			req = tmc->bulk_out_req;
			if (current_rx_bytes == 0)
			{
				current_rx_bytes = req->actual;
				current_rx_buf = req->buf;
			}
			else
			{
				current_rx_bytes = tmc->current_rx_bytes;
				current_rx_buf = tmc->current_rx_buf;
			}
		}
		else
		{
			if (current_rx_bytes == 0)
			{
				req = tmc->bulk_out_req;

				if (req->actual && req->buf)
				{
					current_rx_bytes = req->actual;
					current_rx_buf = req->buf;
				}
			}
			else
			{
				current_rx_bytes = tmc->current_rx_bytes;
				current_rx_buf = tmc->current_rx_buf;
			}
		}

		/* Don't leave irqs off while doing memory copies */
		spin_unlock_irqrestore(&tmc->lock, flags);

		if (len > current_rx_bytes)
		{
			size = current_rx_bytes;
		}
		else
		{
			size = len;
		}

		size -= copy_to_user(buf, current_rx_buf, size);
		bytes_copied += size;
		len -= size;
		buf += size;

		tmc->current_rx_bytes_remaining -= bytes_copied;
		if (!tmc->current_rx_bytes_remaining)
		{
			tmc->new_header_required = true;
		}

		spin_lock_irqsave(&tmc->lock, flags);

		if (size < current_rx_bytes)
		{
			current_rx_bytes -= size;
			current_rx_buf += size;
		}
		else
		{
			current_rx_bytes = 0;
			current_rx_buf = NULL;
			tmc->rx_complete = false;
			tmc_gadget_req_free(tmc->bulk_out_ep, tmc->bulk_out_req);
			tmc->bulk_out_req = tmc_gadget_req_alloc(tmc->bulk_out_ep, TMC_GADGET_BULK_ENDPOINT_SIZE, GFP_KERNEL);
			tmc->previous_bulk_out_tag = tmc->current_header.bTag;
		}
	}

	tmc->current_rx_buf = current_rx_buf;
	tmc->current_rx_bytes = current_rx_bytes;

	spin_unlock_irqrestore(&tmc->lock, flags);
	mutex_unlock(&tmc->io_lock);

	if (tmc->connection_reset)
	{
		tmc->connection_reset = false;
		return -ECONNRESET;
	}

	if (bytes_copied)
	{
		return bytes_copied;
	}

	return -EAGAIN;
}

static __poll_t tmc_gadget_fops_poll(struct file *file, struct poll_table_struct *wait)
{
	unsigned long flags = 0;
	struct tmc_device *tmc  = file->private_data;
	__poll_t	ret = 0;

	mutex_lock(&tmc->io_lock);
	spin_lock_irqsave(&tmc->lock, flags);
	if (!tmc->current_rx_bytes_remaining && !tmc->new_header_available)
	{
		if (tmc_gadget_setup_bulk_out_req(tmc))
		{
			spin_unlock_irqrestore(&tmc->lock, flags);
			mutex_unlock(&tmc->io_lock);
			return ret;
		}
	}

	spin_unlock_irqrestore(&tmc->lock, flags);
	mutex_unlock(&tmc->io_lock);

	poll_wait(file, &tmc->header_wait, wait);

	if (tmc->new_header_available)
	{
		tmc->new_header_available = false;
		ret |= EPOLLIN;
	}

	return ret;
}

static long tmc_gadget_fops_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct tmc_device *tmc  = file->private_data;
	unsigned long flags = 0;
	long ret = 0;

	mutex_lock(&tmc->io_lock);
	spin_lock_irqsave(&tmc->lock, flags);
	switch(cmd)
	{
		case GADGET_TMC_IOCTL_ABORT_BULK_OUT:
			usb_ep_dequeue(tmc->bulk_out_ep, tmc->bulk_out_req);
			break;
		case GADGET_TMC_IOCTL_ABORT_BULK_IN:
			usb_ep_dequeue(tmc->bulk_in_ep, tmc->bulk_out_req);
			break;
		case GADGET_TMC488_IOCTL_GET_STB:
			ret = (long) tmc_gadget_ioctl_read_stb(tmc, (void __user *)arg);
			break;
		case GADGET_TMC488_IOCTL_SET_STB:
			ret = (long) tmc_gadget_ioctl_write_stb(tmc, (void __user *)arg);
			break;
		case GADGET_TMC_IOCTL_GET_HEADER:
			ret = (long) tmc_gadget_ioctl_get_header(tmc, (void __user *)arg);
			break;
		default:
			break;
	}
	spin_unlock_irqrestore(&tmc->lock, flags);
	mutex_unlock(&tmc->io_lock);
	return ret;
}

static const struct file_operations f_tmc_fops =
{
	.owner			= THIS_MODULE,
	.open			= tmc_gadget_fops_open,
	.release		= tmc_gadget_fops_release,
	.write			= tmc_gadget_fops_write,
	.read			= tmc_gadget_fops_read,
	.poll			= tmc_gadget_fops_poll,
	.unlocked_ioctl = tmc_gadget_fops_ioctl,
	.llseek			= noop_llseek,
};

static const struct class tmc_class = {
	.name = "tmc",
};

static int tmc_gadget_set_interface(struct tmc_device *tmc)
{
	int result = 0;

	tmc->bulk_in_ep->desc = tmc_gadget_get_ep_descriptor(tmc->gadget, &tmc_gadget_bulk_in_ep_fs, &tmc_gadget_bulk_in_ep_hs, NULL);
	tmc->bulk_in_ep->driver_data = tmc;

	tmc->bulk_out_ep->desc = tmc_gadget_get_ep_descriptor(tmc->gadget, &tmc_gadget_bulk_out_ep_fs, &tmc_gadget_bulk_out_ep_hs, NULL);
	tmc->bulk_out_ep->driver_data = tmc;

	tmc->interrupt_ep->desc = tmc_gadget_get_ep_descriptor(tmc->gadget, &tmc_gadget_interrupt_ep_fs, &tmc_gadget_interrupt_ep_hs, NULL);
	tmc->interrupt_ep->driver_data = tmc;

	result = usb_ep_enable(tmc->bulk_in_ep);
	if (result != 0)
	{
		dev_err(&tmc->dev, "%s: failed to enable bulk in ep", __func__);
		tmc->bulk_in_ep->desc = NULL;
		tmc->bulk_out_ep->desc = NULL;
		tmc->interrupt_ep->desc = NULL;
		return result;
	}

	result = usb_ep_enable(tmc->bulk_out_ep);
	if (result != 0)
	{
		dev_err(&tmc->dev, "%s: failed to enable bulk out ep", __func__);
		usb_ep_disable(tmc->bulk_in_ep);
		tmc->bulk_in_ep->desc = NULL;
		tmc->bulk_out_ep->desc = NULL;
		tmc->interrupt_ep->desc = NULL;
		return result;
	}

	result = usb_ep_enable(tmc->interrupt_ep);
	if (result != 0)
	{
		dev_err(&tmc->dev, "%s: failed to enable interrupt ep", __func__);
		usb_ep_disable(tmc->bulk_out_ep);
		usb_ep_disable(tmc->bulk_in_ep);
		tmc->bulk_in_ep->desc = NULL;
		tmc->bulk_out_ep->desc = NULL;
		tmc->interrupt_ep->desc = NULL;
		return result;
	}

	tmc->is_shutdown = false;

	return result;
}

static void tmc_gadget_reset_interface(struct tmc_device *tmc)
{
	unsigned long flags;

	if (tmc->interface < 0)
	{
		return;
	}

	if (tmc->bulk_in_ep->desc)
	{
		usb_ep_disable(tmc->bulk_in_ep);
	}

	if (tmc->bulk_out_ep->desc)
	{
		usb_ep_disable(tmc->bulk_out_ep);
	}

	if (tmc->interrupt_ep->desc)
	{
		usb_ep_disable(tmc->interrupt_ep);
	}

	spin_lock_irqsave(&tmc->lock, flags);
	tmc->bulk_in_ep->desc = NULL;
	tmc->bulk_out_ep->desc = NULL;
	tmc->interrupt_ep->desc = NULL;
	tmc->interface = -1;
	tmc->is_shutdown = true;
	spin_unlock_irqrestore(&tmc->lock, flags);
}

static int tmc_gadget_ctrl_req_initiate_clear(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
	int ret = 0;
	uint16_t w_length = le16_to_cpu(ctrl->wLength);
	uint8_t response[] = { GADGET_TMC_STATUS_SUCCESS };

	tmc_gadget_rl_state_machine(tmc, TMC_EVENT_INITIATE_CLEAR);

	// Dequeue the bulk out endpoint if necessary
	if (tmc->bulk_out_queued)
	{
		ret = usb_ep_dequeue(tmc->bulk_out_ep, tmc->bulk_out_req);
		if (ret)
		{
			dev_err(&tmc->dev, "%s, could not dequeue bulk out endpoint (%d)\n", __func__, ret);
		}
		tmc->bulk_out_queued = false;
	}

	// Dequeue the bulk in endpoint if necessary
	if (tmc->bulk_in_queued)
	{
		ret = usb_ep_dequeue(tmc->bulk_in_ep, tmc->bulk_in_req);
		if (ret)
		{
			dev_err(&tmc->dev, "%s, could not dequeue bulk in endpoint (%d)\n", __func__, ret);
		}
		tmc->bulk_in_queued = false;
	}

	// Halt the bulk out endpoint
	ret = tmc_gadget_try_halt_bulk_out_endpoint(tmc);
	if (ret)
	{
		dev_err(&tmc->dev, "error halting bulk out endpoint (%d)\n", ret);
		response[0] = GADGET_TMC_STATUS_FAILED;
		memcpy(req->buf, (void *) &response, 1);
		req->zero = 0;
		req->length = w_length;
		ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);

		if (ret)
		{
			dev_err(&tmc->dev, "error queueing initiate clear response on control endpoint (%d)\n", ret);
		}
	}
	else
	{
		// Queue the control endpoint response from USBTMC 1.00 Table 31
		memcpy(req->buf, (void *) &response, 1);
		req->zero = 0;
		req->length = w_length;
		ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);

		if (ret)
		{
			dev_err(&tmc->dev, "error queueing initiate clear response on control endpoint (%d)\n", ret);
		}
		else
		{
			// Clear all input and output buffers

			// Flush the bulk out FIFO if necessary
			if (usb_ep_fifo_status(tmc->bulk_out_ep) > 0)
			{
				usb_ep_fifo_flush(tmc->bulk_out_ep);
			}

			// Flush the bulk in FIFO if necessary
			if (usb_ep_fifo_status(tmc->bulk_in_ep) > 0)
			{
				usb_ep_fifo_flush(tmc->bulk_in_ep);
			}
		}
	}
	return ret;
}

static int tmc_gadget_ctrl_req_check_clear_status(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
	// TODO: handle the case where the bulk in FIFO cannot be emptied
	int ret = 0;
	uint16_t w_length = le16_to_cpu(ctrl->wLength);
	uint8_t response[] = { GADGET_TMC_STATUS_SUCCESS, 0 };

	memcpy(req->buf, (void *) &response, w_length);
	req->zero = 0;
	req->length = w_length;
	ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
	return ret;
}

static int tmc_gadget_ctrl_req_get_capabilities(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
	uint16_t w_length = le16_to_cpu(ctrl->wLength);
	struct capability_response response;

	memset(&response, 0, sizeof(struct capability_response));

	response.bcdUSBTMC = cpu_to_le16(tmc->device_capabilities.bcdUSBTMC);
	response.bmUSBTMCInterfaceCapabilities = tmc->device_capabilities.bmUSBTMCInterfaceCapabilities;
	response.bmUSBTMCDeviceCapabilities = tmc->device_capabilities.bmUSBTMCDeviceCapabilities;
	response.bcdUSB488 = cpu_to_le16(tmc->device_capabilities.bcdUSB488);
	response.bmUSB488InterfaceCapabilities = tmc->device_capabilities.bmUSB488InterfaceCapabilities;
	response.bmUSB488DeviceCapabilities = tmc->device_capabilities.bmUSB488DeviceCapabilities;

	response.USBTMC_status = GADGET_TMC_STATUS_SUCCESS;
	if (w_length != sizeof(struct capability_response))
	{
		response.USBTMC_status = GADGET_TMC_STATUS_FAILED;
	}
	int ret = min_t(unsigned short, w_length, sizeof(struct capability_response));
	memcpy(req->buf, (void *) &response, ret);

	req->zero = 0;
	req->length = ret;
	ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);

	return ret;
}

static int tmc_gadget_ctrl_req_indicator_pulse(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
	int ret = 0;

	if (tmc->device_capabilities.bmUSBTMCInterfaceCapabilities & GADGET_TMC_CAPABILITY_INDICATOR_PULSE)
	{
		uint16_t w_length = le16_to_cpu(ctrl->wLength);
		uint8_t response[] = { GADGET_TMC_STATUS_SUCCESS };

		memcpy(req->buf, (void *) &response, w_length);
		req->zero = 0;
		req->length = w_length;
		ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
	}
	else
	{
		// TODO: Unhandled request; return a STALL handshake packet?
	}
	return ret;
}

static int tmc_gadget_ctrl_req_read_status_byte(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
	struct status_byte_response response;
	int ret = 0;
	uint16_t w_value = le16_to_cpu(ctrl->wValue);
	uint16_t w_length = le16_to_cpu(ctrl->wLength);

	// Attempt to send the status byte via the interrupt endpoint
	// per USBTMC 4.3.1.2
	if (tmc->device_capabilities.bmUSB488DeviceCapabilities & GADGET_TMC488_CAPABILITY_SR1)
	{
		// USBTMC/USB488 3.4.2 Table 7
		response.tag = (uint8_t) (USB_DIR_IN | (0x7F & w_value));
		response.status_byte = g_status_byte;

		ret = sizeof(response);

		tmc->interrupt_req->length = ret;
		tmc->interrupt_req->zero = 0;
		tmc->interrupt_req->complete = tmc_gadget_interrupt_req_complete;
		tmc->interrupt_req->context = tmc;
		memcpy(tmc->interrupt_req->buf, (void *) &(response), ret);

		// Attempt to send it on the interrupt endpoint first
		ret = usb_ep_queue(tmc->interrupt_ep, tmc->interrupt_req, GFP_ATOMIC);
		if (!ret)
		{
			tmc->intr_in_queued = true;

			// Handle the control endpoint now
			struct status_response status;
			status.USBTMC_status = GADGET_TMC_STATUS_SUCCESS;
			status.tag = (uint8_t) w_value;
			status.status_byte = g_status_byte;

			ret = w_length;
			memcpy(req->buf, (void *) &status, ret);

			req->zero = 0;
			req->length = ret;
			ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		}
	}
	else
	{
		// When a device does not have an interrupt endpoint, the device must
		// queue a control endpoint response as shown in Table 12
		// per USBTMC 4.3.1.1
		struct status_response status;
		status.USBTMC_status = GADGET_TMC_STATUS_SUCCESS;
		status.tag = (uint8_t) w_value;
		status.status_byte = g_status_byte;

		ret = w_length;
		memcpy(req->buf, (void *) &status, ret);

		req->zero = 0;
		req->length = ret;
		ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
	}

	return ret;
}

static int tmc_gadget_ctrl_req_ren_control(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
	uint8_t response[] = { GADGET_TMC_STATUS_SUCCESS };
	int ret = 0;
	uint16_t w_value = le16_to_cpu(ctrl->wValue);
	uint16_t w_length = le16_to_cpu(ctrl->wLength);

	if (tmc->device_capabilities.bmUSB488DeviceCapabilities & GADGET_TMC488_CAPABILITY_REN_CONTROL)
	{
		g_ren = 0xFF & w_value;
		memcpy(req->buf, (void *) &response, w_length);
		req->zero = 0;
		req->length = w_length;
		ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
	}
	else
	{
		// Return a stall handshake packet
	}

	return ret;
}

static int tmc_gadget_ctrl_req_goto_local(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
	int ret = -EOPNOTSUPP;

	if (tmc->device_capabilities.bmUSB488DeviceCapabilities & GADGET_TMC488_CAPABILITY_GOTO_LOCAL)
	{
		// TODO:
	}
	else
	{
		// TODO: Stall the control endpoint
	}

	return ret;
}

static int tmc_gadget_ctrl_req_local_lockout(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
	int ret = -EOPNOTSUPP;

	if (tmc->device_capabilities.bmUSB488DeviceCapabilities & GADGET_TMC488_CAPABILITY_LOCAL_LOCKOUT)
	{
		// TODO:
	}

	return ret;
}

static int tmc_gadget_ctrl_req_initiate_abort_bulk_in(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
	int ret = 0;
	int status = 0;
	uint8_t current_tag = tmc->current_header.bTag;
	uint8_t bulk_in_tag = ctrl->wValue & 0xFF;
	uint8_t response[2] = { 0 };

	if (current_tag == bulk_in_tag)
	{
		status = GADGET_TMC_STATUS_SUCCESS;
		response[0] = status;
		response[1] = current_tag;

		ret = usb_ep_set_halt(tmc->bulk_in_ep);
		if (ret < 0)
		{
			dev_err(&tmc->dev, "usb_ep_set_halt failed on tmc->bulk_in_ep with error code %d\n", ret);
		}

		if (tmc->bulk_in_queued)
		{
			ret = usb_ep_dequeue(tmc->bulk_in_ep, tmc->bulk_in_req);
			if(ret < 0)
			{
				dev_err(&tmc->dev, "usb_ep_dequeue failed on tmc->bulk_in_ep with error code %d\n", ret);
			}
			else
			{
				tmc->bulk_in_queued = false;
			}
		}

		tmc->bulk_in_req->complete = tmc_gadget_bulk_in_req_complete;
		tmc->bulk_in_req->length = 0;
		tmc->bulk_in_req->zero = 1;
		ret = usb_ep_queue(tmc->bulk_in_ep, tmc->bulk_in_req, GFP_ATOMIC);
		if (ret < 0)
		{
			dev_err(&tmc->dev, "usb_ep_queue failed on tmc->bulk_in_ep with error code %d\n", ret);
		}
		else
		{
			tmc->current_tx_bytes = 0;
			tmc->tx_pending = false;
			tmc->bulk_in_queued = false;
			tmc->new_header_required = true;
			tmc->abort_bulk_in_complete = true;
		}
	}
	else if (tmc->bulk_in_queued || (usb_ep_fifo_status(tmc->bulk_out_ep) > 0))
	{
		status = GADGET_TMC_STATUS_TRANSFER_NOT_IN_PROGRESS;
		response[0] = status;
		response[1] = current_tag;
	}
	else
	{
		status = GADGET_TMC_STATUS_FAILED;
		response[0] = status;
		response[1] = tmc->previous_bulk_in_tag;
	}

	memcpy(req->buf, (void *) &response, 2);
	req->zero = 0;
	req->length = 2;

	ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
	if (ret < 0)
	{
		dev_err(&tmc->dev, "usb_ep_queue failed on gadget->ep0 with error code %d\n", ret);
	}

	return ret;
}

static int tmc_gadget_ctrl_req_check_abort_bulk_in_status(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
	int ret = 0;
	uint8_t response[8] = { 0 };

	if (tmc->abort_bulk_in_complete)
	{
		tmc->abort_bulk_in_complete = false;
		response[0] = GADGET_TMC_STATUS_SUCCESS;
		response[4] = cpu_to_le32(0);
	}

	memcpy(req->buf, (void *) &response, 8);
	req->zero = 0;
	req->length = 8;

	ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
	if (ret < 0)
	{
		dev_err(&tmc->dev, "usb_ep_queue failed on gadget->ep0 with error code %d\n", ret);
	}

	return ret;
}

static int tmc_gadget_ctrl_req_initiate_abort_bulk_out(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
	int ret = 0;
	int status = 0;
	uint8_t current_tag = tmc->current_header.bTag;
	uint8_t bulk_out_tag = ctrl->wValue & 0xFF;
	uint8_t response[2] = { 0 };

	tmc->new_header_required = true;
	tmc->current_rx_bytes_remaining = 0;
	tmc->current_rx_bytes = 0;
	tmc->current_rx_buf = NULL;

	if (current_tag == bulk_out_tag)
	{
		ret = usb_ep_set_halt(tmc->bulk_out_ep);
		if (ret < 0)
		{
			dev_err(&tmc->dev, "usb_ep_dequeue failed on tmc->bulk_out_ep with error code %d\n", ret);
		}

		status = GADGET_TMC_STATUS_SUCCESS;
		response[0] = status;
		response[1] = current_tag;
		memcpy(req->buf, (void *) &response, 2);
		req->zero = 0;
		req->length = 2;
		ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (ret < 0)
		{
			dev_err(&tmc->dev, "usb_ep_queue failed on gadget->ep0 with error code %d\n", ret);
		}

		if (usb_ep_fifo_status(tmc->bulk_in_ep) > 0)
		{
			usb_ep_fifo_flush(tmc->bulk_in_ep);
		}
	}
	else if (usb_ep_fifo_status(tmc->bulk_out_ep) > 0)
	{
		status = GADGET_TMC_STATUS_TRANSFER_NOT_IN_PROGRESS;
		response[0] = status;
		response[1] = current_tag;
		memcpy(req->buf, (void *) &response, 2);
		req->zero = 0;
		req->length = 2;
		ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
	}
	else
	{
		status = GADGET_TMC_STATUS_FAILED;
		response[0] = status;
		response[1] = tmc->previous_bulk_out_tag;
		memcpy(req->buf, (void *) &response, 2);
		req->zero = 0;
		req->length = 2;
		ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
	}
	return ret;
}

static int tmc_gadget_ctrl_req_check_abort_bulk_out_status(struct usb_composite_dev *cdev, struct tmc_device *tmc, const struct usb_ctrlrequest *ctrl, struct usb_request *req)
{
	int ret = 0;
	uint8_t response[8] = { 0 };

	response[0] = GADGET_TMC_STATUS_SUCCESS;
	response[4] = cpu_to_le32(0);
	memcpy(req->buf, (void *) &response, 2);
	req->zero = 0;
	req->length = 2;

	ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
	if (ret < 0)
	{
		dev_err(&tmc->dev, "usb_ep_queue failed on gadget->ep0 with error code %d\n", ret);
	}

	return ret;
}

static void tmc_gadget_device_release(struct device *dev)
{
	// TODO: tmc_function_device_release. What needs to be done in here?
	return;
}

static int tmc_gadget_function_bind(struct usb_configuration *config, struct usb_function *func)
{
	struct f_tmc_opts *opts;
	struct usb_ep *ep;
	struct tmc_device *tmc = tmc_gadget_function_to_tmc_device(func);

	opts = tmc_gadget_function_instance_to_f_tmc_opts(func->fi);

	mutex_init(&opts->io_lock);

	/* Allocate interface IDs. */
	int ret = usb_interface_id(config, func);
	if (ret < 0)
	{
		dev_err(&tmc->dev, "could not allocate interface ID's\n");
	}
	tmc_gadget_interface_descriptor.bInterfaceNumber = ret;

	/* finish hookup to lower layer ... */
	tmc->gadget = config->cdev->gadget;

	/* Initialize the char device */
	device_initialize(&tmc->dev);
	tmc->dev.release = tmc_gadget_device_release;
	tmc->dev.class = &tmc_class;
	tmc->dev.devt = MKDEV(major, minor);
	ret = dev_set_name(&tmc->dev, "tmc");
	if (ret)
	{
		dev_err(&tmc->dev, "%s: unable to set device name", __func__);
		return ret;
	}

	/* Allocate endpoints. */
	ret = -ENODEV;
	ep = usb_ep_autoconfig(config->cdev->gadget, &tmc_gadget_bulk_in_ep_fs);
	if (!ep)
	{
		goto ERROR_BULK_IN_EP_AUTOCONFIG_FAIL;
	}
	else
	{
		tmc->bulk_in_ep = ep;
	}

	ep = usb_ep_autoconfig(config->cdev->gadget, &tmc_gadget_bulk_out_ep_fs);
	if (!ep)
	{
		goto ERROR_BULK_OUT_EP_AUTOCONFIG_FAIL;
	}
	else
	{
		tmc->bulk_out_ep = ep;
	}

	ep = usb_ep_autoconfig(config->cdev->gadget, &tmc_gadget_interrupt_ep_fs);
	if (!ep)
	{
		usb_ep_autoconfig_release(tmc->bulk_out_ep);
		usb_ep_autoconfig_release(tmc->bulk_in_ep);
		goto ERROR_INTERRUPT_EP_AUTOCONFIG_FAIL;
	}
	else
	{
		tmc->interrupt_ep = ep;
	}

	/* All endpoints are dual speed */
	tmc_gadget_bulk_in_ep_hs.bEndpointAddress = tmc_gadget_bulk_in_ep_fs.bEndpointAddress;
	tmc_gadget_bulk_out_ep_hs.bEndpointAddress = tmc_gadget_bulk_out_ep_fs.bEndpointAddress;
	tmc_gadget_interrupt_ep_hs.bEndpointAddress = tmc_gadget_interrupt_ep_fs.bEndpointAddress;

	/* Copy descriptors */
	ret = usb_assign_descriptors(func, tmc_gadget_descriptors_fs, tmc_gadget_descriptors_hs, NULL, NULL);
	if (ret)
	{
		goto ERROR_ASSIGN_DESCRIPTORS_FAIL;
	}

	ret = -ENOMEM;
	tmc->bulk_in_req = tmc_gadget_req_alloc(tmc->bulk_in_ep, TMC_GADGET_BULK_ENDPOINT_SIZE, GFP_KERNEL);
	if (!tmc->bulk_in_req)
	{
		goto ERROR_BULK_IN_REQ_ALLOC_FAIL;
	}

	tmc->bulk_out_req = tmc_gadget_req_alloc(tmc->bulk_out_ep, TMC_GADGET_BULK_ENDPOINT_SIZE, GFP_KERNEL);
	if (!tmc->bulk_out_req)
	{
		goto ERROR_BULK_OUT_REQ_ALLOC_FAIL;
	}

	/* Allocate interrupt request and its buffer */
	tmc->interrupt_req = tmc_gadget_req_alloc(tmc->interrupt_ep, TMC_GADGET_INTR_ENDPOINT_SIZE, GFP_KERNEL);
	if (!tmc->interrupt_req)
	{
		goto ERROR_INTERRUPT_REQ_ALLOC_FAIL;
	}

	/* Create the char device */
	cdev_init(&tmc->cdev, &f_tmc_fops);
	ret = cdev_device_add(&tmc->cdev, &tmc->dev);
	if (ret)
	{
		goto ERROR_CDEV_DEVICE_ADD_FAIL;
	}

	tmc->interface = 0;
	tmc->is_shutdown = true;
	tmc->connection_reset = false;

	/*
	 * Get TMC/488 capabilities from ConfigFS
	 * These are user-configured values because the device using the driver may or may not
	 * have certain features. This allows a user to configure the driver to suit their device.
	 */
	tmc->device_capabilities.bcdUSBTMC = opts->bcdUSBTMC;
	tmc->device_capabilities.bmUSBTMCInterfaceCapabilities = opts->bmInterfaceCapabilities;
	tmc->device_capabilities.bmUSBTMCDeviceCapabilities = opts->bmDeviceCapabilities;
	tmc->device_capabilities.bcdUSB488 = opts->bcdUSB488;
	tmc->device_capabilities.bmUSB488InterfaceCapabilities = opts->bmInterfaceCapabilities488;
	tmc->device_capabilities.bmUSB488DeviceCapabilities = opts->bmDeviceCapabilities488;

	memset(&tmc->current_header, 0, sizeof(tmc->current_header));
	tmc->new_header_required = true;
	tmc->new_header_available = false;

	tmc->current_rx_bytes = 0;
	tmc->current_rx_bytes_remaining = 0;
	tmc->current_rx_buf = NULL;

	tmc->current_tx_bytes = 0;
	tmc->current_tx_bytes_remaining = 0;
	tmc->current_tx_buf = NULL;

	tmc->previous_bulk_out_tag = 0;
	tmc->previous_bulk_in_tag = 0;

	tmc->bulk_out_queued = false;
	tmc->bulk_in_queued = false;
	tmc->intr_in_queued = false;
	tmc->abort_bulk_in_complete = false;

	spin_lock_init(&tmc->lock);
	mutex_init(&tmc->io_lock);
	init_waitqueue_head(&tmc->header_wait);
	init_waitqueue_head(&tmc->rx_wait);
	init_waitqueue_head(&tmc->tx_wait);
	tmc->rx_complete = false;
	tmc->tx_pending = false;

	opts->tmc = tmc;

	return 0;

ERROR_CDEV_DEVICE_ADD_FAIL:
	cdev_del(&tmc->cdev);
	tmc_gadget_req_free(tmc->interrupt_ep, tmc->interrupt_req);

ERROR_INTERRUPT_REQ_ALLOC_FAIL:
	tmc_gadget_req_free(tmc->bulk_out_ep, tmc->bulk_out_req);

ERROR_BULK_OUT_REQ_ALLOC_FAIL:
	tmc_gadget_req_free(tmc->bulk_in_ep, tmc->bulk_in_req);

ERROR_BULK_IN_REQ_ALLOC_FAIL:
	usb_free_all_descriptors(func);

ERROR_ASSIGN_DESCRIPTORS_FAIL:
	usb_ep_autoconfig_release(tmc->interrupt_ep);

ERROR_INTERRUPT_EP_AUTOCONFIG_FAIL:
	usb_ep_autoconfig_release(tmc->bulk_out_ep);

ERROR_BULK_OUT_EP_AUTOCONFIG_FAIL:
	usb_ep_autoconfig_release(tmc->bulk_in_ep);

ERROR_BULK_IN_EP_AUTOCONFIG_FAIL:
	return ret;
}

static void tmc_gadget_function_unbind(struct usb_configuration *config, struct usb_function *func)
{
	struct tmc_device *tmc = tmc_gadget_function_to_tmc_device(func);

	cdev_device_del(&tmc->cdev, &tmc->dev);

	if (tmc->interrupt_req != NULL)
	{
		tmc_gadget_req_free(tmc->interrupt_ep, tmc->interrupt_req);
	}
	if (tmc->bulk_out_req != NULL)
	{
		tmc_gadget_req_free(tmc->bulk_out_ep, tmc->bulk_out_req);
	}
	if (tmc->bulk_in_req != NULL)
	{
		tmc_gadget_req_free(tmc->bulk_in_ep, tmc->bulk_in_req);
	}

	usb_free_all_descriptors(func);

	usb_ep_autoconfig_release(tmc->interrupt_ep);
	usb_ep_autoconfig_release(tmc->bulk_out_ep);
	usb_ep_autoconfig_release(tmc->bulk_in_ep);
}

static int tmc_gadget_function_set_alt(struct usb_function *f, unsigned interface, unsigned alt)
{
	struct tmc_device *tmc = tmc_gadget_function_to_tmc_device(f);
	int ret = -ENOTSUPP;

	if (!alt)
	{
		/* Free the current interface */
		tmc_gadget_reset_interface(tmc);

		ret = tmc_gadget_set_interface(tmc);
		if (ret)
		{
			tmc_gadget_reset_interface(tmc);
		}
		else
		{
			tmc->interface = interface;
		}
	}

	return ret;
}

static void tmc_gadget_function_disable(struct usb_function *f)
{

	struct tmc_device *tmc = tmc_gadget_function_to_tmc_device(f);

	tmc_gadget_reset_interface(tmc);
}

static int tmc_gadget_function_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct tmc_device *tmc = tmc_gadget_function_to_tmc_device(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request	*req = cdev->req;
	int value = -EOPNOTSUPP;
	uint16_t w_index = le16_to_cpu(ctrl->wIndex);
	uint16_t w_value = le16_to_cpu(ctrl->wValue);
	uint16_t w_length = le16_to_cpu(ctrl->wLength);

	switch (ctrl->bRequest)
	{
		case GADGET_TMC_REQ_INITIATE_ABORT_BULK_OUT:
			value = tmc_gadget_ctrl_req_initiate_abort_bulk_out(cdev, tmc, ctrl, req);
			break;
		case GADGET_TMC_REQ_CHECK_ABORT_BULK_OUT_STATUS:
			value = tmc_gadget_ctrl_req_check_abort_bulk_out_status(cdev, tmc, ctrl, req);
			break;
		case GADGET_TMC_REQ_INITIATE_ABORT_BULK_IN:
			value = tmc_gadget_ctrl_req_initiate_abort_bulk_in(cdev, tmc, ctrl, req);
			break;
		case GADGET_TMC_REQ_CHECK_ABORT_BULK_IN_STATUS:
			value = tmc_gadget_ctrl_req_check_abort_bulk_in_status(cdev, tmc, ctrl, req);
			break;
		case GADGET_TMC_REQ_INITIATE_CLEAR:
			value = tmc_gadget_ctrl_req_initiate_clear(cdev, tmc, ctrl, req);
			break;
		case GADGET_TMC_REQ_CHECK_CLEAR_STATUS:
			value = tmc_gadget_ctrl_req_check_clear_status(cdev, tmc, ctrl, req);
			break;
		case GADGET_TMC_REQ_GET_CAPABILITIES:
			value = tmc_gadget_ctrl_req_get_capabilities(cdev, tmc, ctrl, req);
			break;
		case GADGET_TMC_REQ_INDICATOR_PULSE:
			value = tmc_gadget_ctrl_req_indicator_pulse(cdev, tmc, ctrl, req);
			break;
		case GADGET_TMC488_REQ_READ_STATUS_BYTE:
			value = tmc_gadget_ctrl_req_read_status_byte(cdev, tmc, ctrl, req);
			break;
		case GADGET_TMC488_REQ_REN_CONTROL:
			value = tmc_gadget_ctrl_req_ren_control(cdev, tmc, ctrl, req);
			break;
		case GADGET_TMC488_REQ_GOTO_LOCAL:
			value = tmc_gadget_ctrl_req_goto_local(cdev, tmc, ctrl, req);
			break;
		case GADGET_TMC488_REQ_LOCAL_LOCKOUT:
			value = tmc_gadget_ctrl_req_local_lockout(cdev, tmc, ctrl, req);
			break;
		default:
			dev_err(&tmc->dev, "---- unhandled request (bRequest: %d)\n", ctrl->bRequest);
			dev_err(&tmc->dev, "---- wValue: %d, wIndex: %d, wLength: %d\n", w_value, w_index, w_length);
	}

	return value;
}

static void tmc_gadget_function_suspend(struct usb_function *f)
{
	return;
}

static void tmc_gadget_function_resume(struct usb_function *f)
{
	return;
}

static void tmc_gadget_function_free_func(struct usb_function *f)
{
	struct tmc_device *tmc = tmc_gadget_function_to_tmc_device(f);

	kfree(tmc);
}

static bool tmc_gadget_function_req_match(struct usb_function *f, const struct usb_ctrlrequest *ctrl, bool config0)
{
	bool ret = false;

	switch(ctrl->bRequest)
	{
		case GADGET_TMC_REQ_INITIATE_ABORT_BULK_OUT:
		case GADGET_TMC_REQ_CHECK_ABORT_BULK_OUT_STATUS:
		case GADGET_TMC_REQ_INITIATE_ABORT_BULK_IN:
		case GADGET_TMC_REQ_CHECK_ABORT_BULK_IN_STATUS:
		case GADGET_TMC_REQ_INITIATE_CLEAR:
		case GADGET_TMC_REQ_CHECK_CLEAR_STATUS:
		case GADGET_TMC_REQ_GET_CAPABILITIES:
		case GADGET_TMC_REQ_INDICATOR_PULSE:
			/* We are 488-compliant so we must handle the following requests as well */
		case GADGET_TMC488_REQ_READ_STATUS_BYTE:
		case GADGET_TMC488_REQ_REN_CONTROL:
		case GADGET_TMC488_REQ_GOTO_LOCAL:
		case GADGET_TMC488_REQ_LOCAL_LOCKOUT:
			ret = true;
			break;
	}
	return ret;
}

static void tmc_gadget_attr_release(struct config_item *item)
{
	struct f_tmc_opts *opts = tmc_gadget_config_item_to_f_tmc_opts(item);

	usb_put_function_instance(&opts->func_inst);
}

static struct configfs_item_operations tmc_item_ops = {
	.release = tmc_gadget_attr_release,
};

/* ConfigFS USB TMC Capabilities */
//cppcheck-suppress unusedFunction
static ssize_t f_tmc_bcdUSBTMC_show(struct config_item *item, char *page)
{
	struct f_tmc_opts *opts = tmc_gadget_config_item_to_f_tmc_opts(item);
	int result;

	mutex_lock(&opts->io_lock);
	result = sprintf(page, "%d", opts->bcdUSBTMC);
	mutex_unlock(&opts->io_lock);

	return result;
}

//cppcheck-suppress unusedFunction
static ssize_t f_tmc_bcdUSBTMC_store(struct config_item *item, const char *page, size_t len)
{
	struct f_tmc_opts *opts = tmc_gadget_config_item_to_f_tmc_opts(item);
	int result;
	uint16_t num;

	mutex_lock(&opts->io_lock);
	result = kstrtou16(page, 0, &num);
	if (result)
	{
		mutex_unlock(&opts->io_lock);
		return result;
	}

	opts->bcdUSBTMC = num;
	result = len;
	mutex_unlock(&opts->io_lock);

	return result;
}
CONFIGFS_ATTR(f_tmc_, bcdUSBTMC);

//cppcheck-suppress unusedFunction
static ssize_t f_tmc_bmUSBTMCInterfaceCapabilities_show(struct config_item *item, char *page)
{
	struct f_tmc_opts *opts = tmc_gadget_config_item_to_f_tmc_opts(item);
	int result;

	mutex_lock(&opts->io_lock);
	result = sprintf(page, "%d", opts->bmInterfaceCapabilities);
	mutex_unlock(&opts->io_lock);

	return result;
}

//cppcheck-suppress unusedFunction
static ssize_t f_tmc_bmUSBTMCInterfaceCapabilities_store(struct config_item *item, const char *page, size_t len)
{
	struct f_tmc_opts *opts = tmc_gadget_config_item_to_f_tmc_opts(item);
	int result;
	uint8_t num;

	mutex_lock(&opts->io_lock);
	result = kstrtou8(page, 0, &num);
	if (result)
	{
		mutex_unlock(&opts->io_lock);
		return result;
	}

	opts->bmInterfaceCapabilities = num;
	result = len;
	mutex_unlock(&opts->io_lock);

	return result;
}
CONFIGFS_ATTR(f_tmc_, bmUSBTMCInterfaceCapabilities);

//cppcheck-suppress unusedFunction
static ssize_t f_tmc_bmUSBTMCDeviceCapabilities_show(struct config_item *item, char *page)
{
	struct f_tmc_opts *opts = tmc_gadget_config_item_to_f_tmc_opts(item);
	int result;

	mutex_lock(&opts->io_lock);
	result = sprintf(page, "%d", opts->bmDeviceCapabilities);
	mutex_unlock(&opts->io_lock);

	return result;
}

//cppcheck-suppress unusedFunction
static ssize_t f_tmc_bmUSBTMCDeviceCapabilities_store(struct config_item *item, const char *page, size_t len)
{
	struct f_tmc_opts *opts = tmc_gadget_config_item_to_f_tmc_opts(item);
	int result;
	uint8_t num;

	mutex_lock(&opts->io_lock);
	result = kstrtou8(page, 0, &num);
	if (result)
	{
		mutex_unlock(&opts->io_lock);
		return result;
	}

	opts->bmDeviceCapabilities = num;
	result = len;
	mutex_unlock(&opts->io_lock);

	return result;
}
CONFIGFS_ATTR(f_tmc_, bmUSBTMCDeviceCapabilities);

//cppcheck-suppress unusedFunction
static ssize_t f_tmc_bcdUSB488_show(struct config_item *item, char *page)
{
	struct f_tmc_opts *opts = tmc_gadget_config_item_to_f_tmc_opts(item);
	int result;

	mutex_lock(&opts->io_lock);
	result = sprintf(page, "%d", opts->bcdUSB488);
	mutex_unlock(&opts->io_lock);

	return result;
}

//cppcheck-suppress unusedFunction
static ssize_t f_tmc_bcdUSB488_store(struct config_item *item, const char *page, size_t len)
{
	struct f_tmc_opts *opts = tmc_gadget_config_item_to_f_tmc_opts(item);
	int result;
	uint16_t num;

	mutex_lock(&opts->io_lock);
	result = kstrtou16(page, 0, &num);
	if (result)
	{
		mutex_unlock(&opts->io_lock);
		return result;
	}

	opts->bcdUSB488 = num;
	result = len;
	mutex_unlock(&opts->io_lock);

	return result;
}
CONFIGFS_ATTR(f_tmc_, bcdUSB488);

//cppcheck-suppress unusedFunction
static ssize_t f_tmc_bmUSB488InterfaceCapabilities_show(struct config_item *item, char *page)
{
	struct f_tmc_opts *opts = tmc_gadget_config_item_to_f_tmc_opts(item);
	int result;

	mutex_lock(&opts->io_lock);
	result = sprintf(page, "%d", opts->bmInterfaceCapabilities488);
	mutex_unlock(&opts->io_lock);

	return result;
}

//cppcheck-suppress unusedFunction
static ssize_t f_tmc_bmUSB488InterfaceCapabilities_store(struct config_item *item, const char *page, size_t len)
{
	struct f_tmc_opts *opts = tmc_gadget_config_item_to_f_tmc_opts(item);
	int result;
	uint8_t num;

	mutex_lock(&opts->io_lock);
	result = kstrtou8(page, 0, &num);
	if (result)
	{
		mutex_unlock(&opts->io_lock);
		return result;
	}

	opts->bmInterfaceCapabilities488 = num;
	result = len;
	mutex_unlock(&opts->io_lock);

	return result;
}
CONFIGFS_ATTR(f_tmc_, bmUSB488InterfaceCapabilities);

//cppcheck-suppress unusedFunction
static ssize_t f_tmc_bmUSB488DeviceCapabilities_show(struct config_item *item, char *page)
{
	struct f_tmc_opts *opts = tmc_gadget_config_item_to_f_tmc_opts(item);
	int result;

	mutex_lock(&opts->io_lock);
	result = sprintf(page, "%d", opts->bmDeviceCapabilities488);
	mutex_unlock(&opts->io_lock);

	return result;
}

//cppcheck-suppress unusedFunction
static ssize_t f_tmc_bmUSB488DeviceCapabilities_store(struct config_item *item, const char *page, size_t len)
{
	struct f_tmc_opts *opts = tmc_gadget_config_item_to_f_tmc_opts(item);
	int result;
	uint8_t num;

	mutex_lock(&opts->io_lock);
	result = kstrtou8(page, 0, &num);
	if (result)
	{
		mutex_unlock(&opts->io_lock);
		return result;
	}

	opts->bmDeviceCapabilities488 = num;
	result = len;
	mutex_unlock(&opts->io_lock);

	return result;
}
CONFIGFS_ATTR(f_tmc_, bmUSB488DeviceCapabilities);

static struct configfs_attribute *tmc_attrs[] = {
	&f_tmc_attr_bcdUSBTMC,
	&f_tmc_attr_bmUSBTMCInterfaceCapabilities,
	&f_tmc_attr_bmUSBTMCDeviceCapabilities,
	&f_tmc_attr_bcdUSB488,
	&f_tmc_attr_bmUSB488InterfaceCapabilities,
	&f_tmc_attr_bmUSB488DeviceCapabilities,
	NULL,
};

static const struct config_item_type tmc_func_type = {
	.ct_item_ops = &tmc_item_ops,
	.ct_attrs = tmc_attrs,
	.ct_owner = THIS_MODULE,
};

static struct usb_function *tmc_gadget_alloc_func(struct usb_function_instance *fi)
{
	struct tmc_device *tmc;

	tmc = kzalloc(sizeof(*tmc), GFP_KERNEL);
	if (tmc == NULL)
	{
		return ERR_PTR(-ENOMEM);
	}

	/* Register the function. */
	tmc->func.name = "tmc";
	tmc->func.strings = tmc_gadget_strings;
	tmc->func.bind = tmc_gadget_function_bind;
	tmc->func.unbind = tmc_gadget_function_unbind;
	tmc->func.set_alt = tmc_gadget_function_set_alt;
	tmc->func.disable = tmc_gadget_function_disable;
	tmc->func.setup = tmc_gadget_function_setup;
	tmc->func.suspend = tmc_gadget_function_suspend;
	tmc->func.resume = tmc_gadget_function_resume;
	tmc->func.free_func = tmc_gadget_function_free_func;
	tmc->func.req_match = tmc_gadget_function_req_match;

	return &tmc->func;
}

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
	{
		return ERR_PTR(-ENOMEM);
	}
	opts->func_inst.free_func_inst = tmc_free_instance;

	status = class_register(&tmc_class);
	if (status)
	{
		return ERR_PTR(-EFAULT);
	}

	status = alloc_chrdev_region(&dev, 0, 1, "tmc");
	if (status)
	{
		class_unregister(&tmc_class);
		return ERR_PTR(-EFAULT);
	}

	major = MAJOR(dev);
	minor = 0;

	config_group_init_type_name(&opts->func_inst.group, "", &tmc_func_type);

	return &opts->func_inst;
}

DECLARE_USB_FUNCTION_INIT(tmc, tmc_alloc_instance, tmc_gadget_alloc_func);
MODULE_LICENSE("GPL");
