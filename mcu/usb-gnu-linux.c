/*
 * usb-gnu-linux.c - USB Device Emulation by USBIP Protocol
 *
 * Copyright (C) 2017  Flying Stone Technology
 * Author: NIIBE Yutaka <gniibe@fsij.org>
 *
 * This file is a part of Chopstx, a thread library for embedded.
 *
 * Chopstx is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Chopstx is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <pthread.h>

#include <unistd.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <signal.h>

#include <usb_lld.h>
#include <usb_lld_common.h>

static pthread_t tid_main;
static pthread_t tid_usbip;

#define USBIP_PORT 3240

#define CMD_REQ_LIST   0x01118005
#define CMD_REQ_ATTACH 0x01118003
#define CMD_URB        0x00000001
#define CMD_DETACH     0x00000002

#define RET_URB        0x00000003

struct usbip_msg_head {
  uint32_t cmd;
  uint32_t seq;
};

struct usbip_usb_device {
  char path[256];
  char busid[32];

  uint32_t busnum;
  uint32_t devnum;
  uint32_t speed;

  uint16_t idVendor;
  uint16_t IdProduct;
  uint16_t bcdDevice;

  uint8_t bDeviceClass;
  uint8_t bDeviceSubClass;
  uint8_t bDeviceProtocol;
  uint8_t bConfigurationValue;
  uint8_t bNumConfigurations;
  uint8_t bNumInterfaces;
} __attribute__((packed));

/*
 * Only support a single device.
 */
static struct usbip_usb_device usbip_usb_device;

static void
refresh_usb_device (void)
{
  /* Issue device descriptor request, and fill usbip_usb_device.  */

  /* 1: size=18 (or follows more desc) */
  /* 1: DEVICE_DESCRIPTOR */
  /* 2: bcdUSB: ignore or use for speed? */
  /* 1: bDeviceClass */ /* COPY */
  /* 1: bDeviceSubClass */ /* COPY */
  /* 1: bDeviceProtocol */ /* COPY */
  /* 1: bMaxPacketSize: ignore */
  /* 2: idVendor */  /* COPY */
  /* 2: idProduct */ /* COPY */
  /* 2: bcdDevice */ /* COPY */
  /* 1: iManufacturer: ignore */
  /* 1: iProduct: ignore */
  /* 1: iSerialNumber: ignore */
  /* 1: bNumConfigurations */ /* COPY */
  /* ... */
}

#define USBIP_REPLY_HEADER_SIZE 8
#define DEVICE_INFO_SIZE        (256+32+12+6+6)
#define INTERFACE_INFO_SIZE     4
#define DEVICE_LIST_SIZE        (DEVICE_INFO_SIZE*1)

#define USBIP_REPLY_DEVICE_LIST "\x01\x11\x00\x02"
#define USBIP_REPLY_ATTACH      "\x01\x11\x00\x03"

#define NETWORK_UINT32_ZERO     "\x00\x00\x00\x00"
#define NETWORK_UINT32_ONE      "\x00\x00\x00\x01"
#define NETWORK_UINT32_TWO      "\x00\x00\x00\x02"
#define NETWORK_UINT16_FSIJ      "\x23\x4b"
#define NETWORK_UINT16_ZERO      "\x00\x00"
#define NETWORK_UINT16_ONE_ONE   "\x01\x01"

enum {
  USB_INTR_SETUP,
  USB_INTR_DATA_TRANSFER,
  USB_INTR_RESET,
  USB_INTR_SUSPEND,
};

struct usb_controller {
  uint8_t intr;
  uint8_t dir;
  uint8_t ep;
};

static struct usb_controller usbc;

static void
notify_device (uint8_t intr, uint8_t dir, uint8_t ep)
{
  usbc.intr = intr;
  usbc.dir = dir;
  usbc.ep = ep;
  pthread_kill (tid_main, SIGUSR1);
}

#define MY_BUS_ID "1-1\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000"

static char *
fill_device_info (char *p)
{
  memset (p, 0, 256);
  strcpy (p, "/sys/devices/pci0000:00/0000:00:01.1/usb1/1-1");
  p += 256;

  memcpy (p, MY_BUS_ID, 32);
  p += 32;

  memcpy (p, NETWORK_UINT32_ZERO, 4); /* Bus */
  p += 4;
  memcpy (p, NETWORK_UINT32_ZERO, 4); /* Dev */
  p += 4;
  memcpy (p, NETWORK_UINT32_ONE, 4); /* Speed */
  p += 4;
  memcpy (p, NETWORK_UINT16_FSIJ, 2);
  p += 2;
  memcpy (p, NETWORK_UINT16_ZERO, 2); /* Gnuk */
  p += 2;
  memcpy (p, NETWORK_UINT16_ONE_ONE, 2); /* USB 1.1 */
  p += 2;

  *p++ = 0; /* bDeviceClass        */
  *p++ = 0; /* bDeviceSubClass     */
  *p++ = 0; /* bDeviceProtocol     */

  *p++ = 0; /* bConfigurationValue */
  *p++ = 1; /* bNumConfigurations  */
  *p++ = 0; /* bNumInterfaces, little sense to fill (only for user output)*/

  return p;
}

static char *
list_devices (size_t *len_p)
{
  char *p0, *p;

  *len_p = 0;
  p0 = malloc (DEVICE_LIST_SIZE);
  if (p0 == NULL)
    return NULL;

  *len_p = DEVICE_LIST_SIZE;

  p = fill_device_info (p0);
  /* Don't send interface information, that's no problem for a single device.  */

  return p0;
}

static char *
attach_device (char busid[32], size_t *len_p)
{
  char *p0, *p;

  *len_p = 0;
  if (memcmp (busid, MY_BUS_ID, 32) != 0) 
    return NULL;

  notify_device (USB_INTR_RESET, 0, 0);

  p0 = malloc (DEVICE_INFO_SIZE);
  if (p0 == NULL)
    return NULL;

  p = fill_device_info (p0);

  *len_p = p - p0;
  return p0;
}

#define URB_DATA_SIZE 65536

#define USBIP_DIR_OUT 0
#define USBIP_DIR_IN  1

struct usbip_msg_ctl {
  uint32_t devid;
  uint32_t dir;
  uint32_t ep;
  uint32_t flags_status;
  uint32_t len;
  uint32_t rsvd[2];
  uint32_t err_cnt;
};

static struct usbip_msg_ctl msg_ctl;
static uint8_t usb_setup[8];
static char usb_data[URB_DATA_SIZE];
static char *usb_data_p;

static int control_setup_transaction (void);
static int control_write_data_transaction (char *buf, uint16_t count);
static int control_write_status_transaction (void);
static int control_read_data_transaction (char *buf);
static int control_read_status_transaction (void);
static int write_data_transaction (void);
static int read_data_transaction (void);


static int
handle_control_urb (void)
{
  int r;

  r = control_setup_transaction ();
  if (r < 0)
    return r;

  if (msg_ctl.dir == USBIP_DIR_OUT)
    {				/* Output from host to device.  */
      uint16_t remain = msg_ctl.len;
      uint16_t count;

      while (r == 0)
	{
	  if (remain > 64)
	    count = 64;
	  else
	    count = remain;

	  r = control_write_data_transaction (usb_data_p, count);
	  if (r < 0)
	    break;

	  usb_data_p += count;
	  if (count < 64)
	    break;
	}
      if (r >= 0)
	r = control_write_status_transaction ();
    }
  else
    {				/* Input from device to host.  */
      int count = 0;

      usb_data_p = usb_data;
      while (r == 0)
	{
	  r = control_read_data_transaction (usb_data_p);
	  if (r < 0)
	    break;

	  count += r;
	  usb_data_p += r;
	  if (r < 64)
	    break;

	  if (count == URB_DATA_SIZE)
	    {
	      perror ("control IN too long");
	      return -1;
	    }
	}
      msg_ctl.len = count;
      if (r > 0)
	r = control_read_status_transaction ();
    }

  return r;
}

static int
handle_data_urb  (void)
{
  int r;

  if (msg_ctl.dir == USBIP_DIR_OUT)
    {				/* Output from host to device.  */
      while (r == 0)
	r = write_data_transaction ();
    }
  else
    {				/* Input from device to host.  */
      while (r == 0)
	r = read_data_transaction ();
    }

  return r;
}

static void
handle_urb (int fd, uint32_t seq)
{
  int r = 0;

  if (recv (fd, (char *)&msg_ctl, sizeof (msg_ctl), 0) != sizeof (msg_ctl))
    {
      perror ("msg recv ctl");
      r = -1;
      goto leave;
    }

  msg_ctl.devid = ntohl (msg_ctl.devid);
  msg_ctl.dir = ntohl (msg_ctl.dir);
  msg_ctl.ep = ntohl (msg_ctl.ep);
  msg_ctl.flags_status = ntohl (msg_ctl.flags_status);
  msg_ctl.len = ntohl (msg_ctl.len);

  if (recv (fd, (char *)usb_setup, sizeof (usb_setup), 0) != sizeof (usb_setup))
    {
      perror ("msg recv setup");
      r = -1;
      goto leave;
    }

  usb_data_p = usb_data;

  if (msg_ctl.dir == USBIP_DIR_OUT && msg_ctl.len)
    {
      if (msg_ctl.len > URB_DATA_SIZE)
	{
	  perror ("msg len too long");
	  r = -1;
	  goto leave;
	}

      if (recv (fd, usb_data, msg_ctl.len, 0) != msg_ctl.len)
	{
	  perror ("msg recv data");
	  r = -1;
	  goto leave;
	}
    }

  if (msg_ctl.ep == 0)
    r = handle_control_urb ();
  else
    r = handle_data_urb ();

 leave:
  if (r < 0)
    msg_ctl.flags_status = htonl (1);
  else
    msg_ctl.flags_status = 0;

  {
    struct usbip_msg_head msg;
    int dir = msg_ctl.dir;
    int len = msg_ctl.len;

    msg.cmd = htonl (RET_URB);
    msg.seq = htonl (seq);

    if ((size_t)send (fd, &msg, sizeof (msg), 0) != sizeof (msg))
      {
	perror ("reply send");
      }

    msg_ctl.devid = htonl (msg_ctl.devid);
    msg_ctl.dir = htonl (msg_ctl.dir);
    msg_ctl.ep = htonl (msg_ctl.ep);
    msg_ctl.len = htonl (msg_ctl.len);

    if ((size_t)send (fd, &msg_ctl, sizeof (msg_ctl), 0) != sizeof (msg_ctl))
      {
	perror ("reply send");
      }

    if (dir == USBIP_DIR_IN && len)
      {
	if (send (fd, usb_data, len, 0) != len)
	  {
	    perror ("reply send");
	  }
      }
  }
}

static pthread_mutex_t comm_mutex;
static pthread_cond_t comm_cond;

static void
usbip_wait (void)
{
  pthread_mutex_lock (&comm_mutex);
  pthread_cond_wait (&comm_cond, &comm_mutex);
  pthread_mutex_unlock (&comm_mutex);
}

static void
notify_usbip (void)
{
  pthread_mutex_lock (&comm_mutex);
  pthread_cond_signal (&comm_cond);
  pthread_mutex_unlock (&comm_mutex);
}

static void
send_reply (int fd, char *reply, int ok)
{
  char buf[8];
  char *p = buf;

  memcpy (p, reply, 4);
  p += 4;
  if (ok)
    memcpy (p, NETWORK_UINT32_ZERO, 4);
  else
    memcpy (p, NETWORK_UINT32_ONE, 4);
  p += 4;

  if ((size_t)send (fd, buf, 8, 0) != 8)
    {
      perror ("reply send");
    }
}

/*
 * In the USBIP protocol, client sends URB (USB Request Block) to this
 * server.
 *
 * This server acts/emulates as a USB host controller, and transforms
 * URB into packets to device, transforms packets from device into
 * URB.
 */
static void *
run_server (void *arg)
{
  int sock;
  struct sockaddr_in v4addr;
  const int one = 1;

  pthread_mutex_init (&comm_mutex, NULL);
  pthread_cond_init (&comm_cond, NULL);

  (void)arg;
  if ((sock = socket (PF_INET, SOCK_STREAM, 0)) < 0)
    {
      perror ("socket");
      exit (1);
    }

  if (setsockopt (sock, SOL_SOCKET, SO_REUSEADDR,
		  (const char*)&one, sizeof (int)) < 0)
    perror ("WARN: setsockopt");

  memset (&v4addr, 0, sizeof (v4addr));
  v4addr.sin_family = AF_INET;
  v4addr.sin_addr.s_addr = htonl (INADDR_LOOPBACK);
  v4addr.sin_port = htons (USBIP_PORT);

  if (bind (sock, (const struct sockaddr *)&v4addr, sizeof (v4addr)) < 0)
    {
      perror ("bind");
      exit (1);
    }

  /* We only accept a connection from a single client.  */
  if (listen (sock, 1) < 0)
    {
      perror ("listen");
      exit (1);
    }

  for (;;)
    {
      int fd;
      int attached = 0;

      /* We don't care who is connecting.  */
      if ((fd = accept (sock, NULL, NULL)) < 0)
        {
          perror ("accept");
          exit (1);
        }

      for (;;)
        {
	  struct usbip_msg_head msg;
	  char busid[32];

	  if (recv (fd, (char *)&msg, sizeof (msg), 0) != sizeof (msg))
	    {
	      perror ("msg recv");
	      break;
	    }

	  msg.cmd = ntohl (msg.cmd);
	  msg.seq = ntohl (msg.seq);

	  if (msg.cmd == CMD_REQ_LIST)
	    {
	      char *device_list;
	      size_t device_list_size;

	      if (recv (fd, busid, 32, 0) != 32)
		{
		  perror ("msg recv");
		  break;
		}

	      if (attached)
		{
		  fprintf (stderr, "REQ list while attached\n");
		  break;
		}

	      device_list = list_devices (&device_list_size);

	      send_reply (fd, USBIP_REPLY_DEVICE_LIST, !!device_list);


	      if (send (fd, NETWORK_UINT32_ONE, 4, 0) != 4)
		{
		  perror ("list send");
		  break;
		}

	      if ((size_t)send (fd, device_list, device_list_size, 0)
		  != device_list_size)
		{
		  perror ("list send");
		  break;
		}

	      free (device_list);
	    }
	  else if (msg.cmd == CMD_REQ_ATTACH)
	    {
	      char *attach;
	      size_t attach_size;

	      if (attached)
		{
		  fprintf (stderr, "REQ attach while attached\n");
		  break;
		}
	      
	      if (recv (fd, busid, 32, 0) != 32)
		{
		  perror ("attach recv");
		  break;
		}

	      attach = attach_device (busid, &attach_size);
	      send_reply (fd, USBIP_REPLY_ATTACH, !!attach);
	      if (attach)
		{
		  if ((size_t)send (fd, attach, attach_size, 0) != attach_size)
		    {
		      perror ("list send");
		      break;
		    }
		  free (attach);
		  attached = 1;
		}
	    }
	  else if (msg.cmd == CMD_URB)
	    {
	      if (!attached)
		{
		  fprintf (stderr, "URB while attached\n");
		  break;
		}

	      handle_urb (fd, msg.seq);
	    }
	  else if (msg.cmd == CMD_DETACH)
	    {
	      if (!attached)
		{
		  fprintf (stderr, "DETACH while attached\n");
		  break;
		}

	      /* send reply??? */
	      break;
	    }
	  else
	    {
	      fprintf (stderr, "Unknown command %08x, disconnecting.\n", msg.cmd);
	      break;
	    }
	}

       close (fd);
    }

  return NULL;
}

enum {
  USB_STATE_STALL = 0,
  USB_STATE_NAK,
  USB_STATE_SETUP,
  USB_STATE_TX,
  USB_STATE_RX,
};

#define EP_TX_VALID (1<<0)
#define EP_RX_VALID (1<<1)

struct usb_control {
  uint8_t state;
  uint8_t buf[USB_MAX_PACKET_SIZE];
  uint16_t len;
  pthread_mutex_t mutex;
  pthread_cond_t cond;
};

static struct usb_control usbc_ep0;
static struct usb_control usbc_ep1_in;
static struct usb_control usbc_ep2_in;
static struct usb_control usbc_ep3_in;
static struct usb_control usbc_ep4_in;
static struct usb_control usbc_ep5_in;
static struct usb_control usbc_ep6_in;
static struct usb_control usbc_ep7_in;
static struct usb_control usbc_ep1_out;
static struct usb_control usbc_ep2_out;
static struct usb_control usbc_ep3_out;
static struct usb_control usbc_ep4_out;
static struct usb_control usbc_ep5_out;
static struct usb_control usbc_ep6_out;
static struct usb_control usbc_ep7_out;

/** Control setup transaction.
 *
 * -->[SETUP]-->[DATA0]-->{ACK}-> Success
 *                      \-------> Error
 */
static int
control_setup_transaction (void)
{
  int r;

  pthread_mutex_lock (&usbc_ep0.mutex);
  while (1)
    if (usbc_ep0.state == USB_STATE_NAK)
      pthread_cond_wait (&usbc_ep0.cond, &usbc_ep0.mutex); /* Timed wait??? */
    else if (usbc_ep0.state == USB_STATE_SETUP)
      {
	memcpy (usbc_ep0.buf, usb_setup, sizeof (usb_setup));
	usbc_ep0.len = sizeof (usb_setup);
	usbc_ep0.state = USB_STATE_NAK;
	notify_device (USB_INTR_SETUP, 0, msg_ctl.dir);
	if (msg_ctl.dir == USBIP_DIR_OUT
	    && usb_setup[6] == 0 && usb_setup[6] == 7)
	  /* Control Write Transfer with no data satage.  */
	  r = 1;
	else
	  r = 0;
	break;
      }
    else
      {
	r = -1;
	break;
      }
  pthread_mutex_unlock (&usbc_ep0.mutex);
  return r;
}

/** Control WRITE transaction.
 *
 * -->[OUT]-->[DATA0]---->{ACK}---> Success
 *                     \--{NAK}---> again
 *                     \--{STALL}-> Error
 *                     \----------> Error
 */
static int
control_write_data_transaction (char *buf, uint16_t count)
{
  int r;

  pthread_mutex_lock (&usbc_ep0.mutex);
  while (1)
    if (usbc_ep0.state == USB_STATE_NAK)
      pthread_cond_wait (&usbc_ep0.cond, &usbc_ep0.mutex); /* Timed wait??? */
    else if (usbc_ep0.state == USB_STATE_RX)
      {
	memcpy (usbc_ep0.buf, buf, count);
	usbc_ep0.len = count;
	usbc_ep0.state = USB_STATE_NAK;
	notify_device (USB_INTR_DATA_TRANSFER, msg_ctl.ep, msg_ctl.dir);
	r = 0;
	break;
      }
    else
      {
	r = -1;
	break;
      }
  pthread_mutex_unlock (&usbc_ep0.mutex);
  return r;
}

/** Status transaction for control WRITE.
 *            zero-len
 * -->[IN]--->{DATA0}->[ACK]---> Success
 *         \--{NAK}---> again
 *         \--{STALL}-> Error
 *         \----------> Error
 */
static int
control_write_status_transaction (void)
{
  int r;

  pthread_mutex_lock (&usbc_ep0.mutex);
  while (1)
    if (usbc_ep0.state == USB_STATE_NAK)
      pthread_cond_wait (&usbc_ep0.cond, &usbc_ep0.mutex); /* Timed wait??? */
    else if (usbc_ep0.state == USB_STATE_TX)
      {
	if (usbc_ep0.len != 0)
	  fprintf (stderr, "*** ACK length %d\n", usbc_ep0.len);
	usbc_ep0.state = USB_STATE_NAK;
	notify_device (USB_INTR_DATA_TRANSFER, msg_ctl.ep, msg_ctl.dir);
	r = 0;
	break;
      }
    else
      {
	r = -1;
	break;
      }
  pthread_mutex_unlock (&usbc_ep0.mutex);
  return r;
}

/** Control READ transaction.
 *
 * -->[IN]-->{DATAx}---->[ACK]---> Success
 *         |          \---------> Error
 *         \-{STALL}------------> Error
 *         \-{NAK}--------------> again
 */
static int
control_read_data_transaction (char *buf)
{
  int r;
  uint16_t count;

  pthread_mutex_lock (&usbc_ep0.mutex);
  while (1)
    if (usbc_ep0.state == USB_STATE_NAK)
      pthread_cond_wait (&usbc_ep0.cond, &usbc_ep0.mutex); /* Timed wait??? */
    else if (usbc_ep0.state == USB_STATE_TX)
      {
	if (usbc_ep0.len > 64)
	  {
	    fprintf (stderr, "*** length %d\n", usbc_ep0.len);
	    r = -1;
	  }
	else
	  {
	    count = usbc_ep0.len;
	    memcpy (buf, usbc_ep0.buf, count);
	    usbc_ep0.state = USB_STATE_NAK;
	    notify_device (USB_INTR_DATA_TRANSFER, msg_ctl.ep, msg_ctl.dir);
	    r = count;
	  }
	break;
      }
    else
      {
	r = -1;
	break;
      }
  pthread_mutex_unlock (&usbc_ep0.mutex);
  return r;
}

/* Status transaction for control READ.
 *            zero-len
 * -->[OUT]--->[DATA0]--->{ACK}---> Success
 *                     \--{NAK}---> again
 *                     \--{STALL}-> Error
 *                     \----------> Error
 */
static int
control_read_status_transaction (void)
{
  int r;

  pthread_mutex_lock (&usbc_ep0.mutex);
  while (1)
    if (usbc_ep0.state == USB_STATE_NAK)
      pthread_cond_wait (&usbc_ep0.cond, &usbc_ep0.mutex); /* Timed wait??? */
    else if (usbc_ep0.state == USB_STATE_RX)
      {
	usbc_ep0.len = 0;
	usbc_ep0.state = USB_STATE_NAK;
	notify_device (USB_INTR_DATA_TRANSFER, msg_ctl.ep, msg_ctl.dir);
	r = 0;
	break;
      }
    else
      {
	r = -1;
	break;
      }
  pthread_mutex_unlock (&usbc_ep0.mutex);
  return r;
}

static int
write_data_transaction (void)
{
  return 0;
}

static int
read_data_transaction (void)
{
  return 0;
}

void
usb_lld_init (struct usb_dev *dev, uint8_t feature)
{
  int r;

  /*
   * Launch the thread for USBIP.  This maps to usb_lld_sys_init where
   * we initialize USB controller on MCU.
   */
  tid_main = pthread_self ();
  r = pthread_create (&tid_usbip, NULL, run_server, NULL);
  if (r)
    {
      fprintf (stderr, "usb_lld_init: %s\n", strerror (r));
      exit (1);
    }

  dev->configuration = 0;
  dev->feature = feature;
  dev->state = WAIT_SETUP;

  pthread_mutex_init (&usbc_ep0.mutex, NULL);
  pthread_cond_init (&usbc_ep0.cond, NULL);
}


void
usb_lld_prepare_shutdown (void)
{
}

void
usb_lld_shutdown (void)
{
  /* 
   * Stop USBIP server thread.
   * This maps to usb_lld_sys_shutdown where we stop USB controller on MCU.
   */
  pthread_cancel (tid_usbip);
  pthread_join (tid_usbip, NULL);
}


int
usb_lld_ctrl_ack (struct usb_dev *dev)
{
  dev->state = WAIT_STATUS_IN;
  usbc_ep0.len = 0;
  usbc_ep0.state = USB_STATE_TX;
  notify_usbip ();
  return USB_EVENT_OK;
}

int
usb_lld_ctrl_recv (struct usb_dev *dev, void *p, size_t len)
{
  struct ctrl_data *data_p = &dev->ctrl_data;
  data_p->addr = p;
  data_p->len = len;
  dev->state = OUT_DATA;
  usbc_ep0.state = USB_STATE_RX;
  notify_usbip ();
  return USB_EVENT_OK;
}

int
usb_lld_ctrl_send (struct usb_dev *dev, const void *buf, size_t buflen)
{
  struct ctrl_data *data_p = &dev->ctrl_data;
  uint32_t len_asked = dev->dev_req.len;
  uint32_t len;

  data_p->addr = (void *)buf;
  data_p->len = buflen;

  /* Restrict the data length to be the one host asks for */
  if (data_p->len > len_asked)
    data_p->len = len_asked;

  if (data_p->len != 0 && (data_p->len % USB_MAX_PACKET_SIZE) == 0)
    data_p->require_zlp = 1;

  if (data_p->len < USB_MAX_PACKET_SIZE)
    {
      len = data_p->len;
      dev->state = LAST_IN_DATA;
    }
  else
    {
      len = USB_MAX_PACKET_SIZE;
      dev->state = IN_DATA;
    }

  if (len)
    {
      memcpy (usbc_ep0.buf, data_p->addr, len);
      data_p->len -= len;
      data_p->addr += len;
    }

  usbc_ep0.len = len;
  usbc_ep0.state = USB_STATE_TX;
  notify_usbip ();
  return USB_EVENT_OK;
}

uint8_t
usb_lld_current_configuration (struct usb_dev *dev)
{
  (void)dev;
  return 0;
}


void
usb_lld_ctrl_error (struct usb_dev *dev)
{
  dev->state = STALLED;
  notify_usbip ();
}

void
usb_lld_reset (struct usb_dev *dev, uint8_t feature)
{
  usb_lld_set_configuration (dev, 0);
  dev->feature = feature;
}

void
usb_lld_set_configuration (struct usb_dev *dev, uint8_t config)
{
  dev->configuration = config;
}


void
usb_lld_setup_endpoint (int ep_num, int ep_type, int ep_kind,
			int ep_rx_addr, int ep_tx_addr,
			int ep_rx_memory_size)
{
}

void
usb_lld_stall (int ep_num)
{
}


void
usb_lld_stall_tx (int ep_num)
{
  usb_lld_stall (ep_num);
}

void
usb_lld_stall_rx (int ep_num)
{
  usb_lld_stall (ep_num);
}
