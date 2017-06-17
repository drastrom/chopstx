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
#include <errno.h>

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
#define CMD_URB_SUBMIT 0x00000001
#define CMD_URB_UNLINK 0x00000002

#define REP_URB_SUBMIT 0x00000003
#define REP_URB_UNLINK 0x00000004

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
  uint16_t idProduct;
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

static const char *issue_get_desc (void);

#define MY_BUS_ID "1-1\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000"

#define USBIP_DIR_OUT 0
#define USBIP_DIR_IN  1

static void
refresh_usb_device (void)
{
  const char *desc;

  /* Issue device descriptor request, and fill usbip_usb_device.  */
  desc = issue_get_desc ();

  memset (usbip_usb_device.path, 0, 256);
  strcpy (usbip_usb_device.path,
	  "/sys/devices/pci0000:00/0000:00:01.1/usb1/1-1");
  memcpy (usbip_usb_device.busid, MY_BUS_ID, 32);

  usbip_usb_device.busnum = 0;
  usbip_usb_device.devnum = 0;
  usbip_usb_device.speed =  htonl (2); /* Full speed.  */

  /* USB descriptors are little endian.  USBIP is network order.  */

  /* 0: size=18 (or follows more desc) */
  /* 1: DEVICE_DESCRIPTOR */
  /* 2: bcdUSB: ignore or use for speed? */
  /* 4: bDeviceClass */
  /* 5: bDeviceSubClass */
  /* 6: bDeviceProtocol */
  /* 7: bMaxPacketSize: ignore */
  /* 8: idVendor */
  /* 10: idProduct */
  /* 12: bcdDevice */
  /* 14: iManufacturer: ignore */
  /* 15: iProduct: ignore */
  /* 16: iSerialNumber: ignore */
  /* 17: bNumConfigurations */
  /* ... */
  usbip_usb_device.idVendor = htons (((desc[9] << 8)|desc[8]));
  usbip_usb_device.idProduct = htons (((desc[11] << 8)|desc[10]));
  usbip_usb_device.bcdDevice = htons (((desc[13] << 8)|desc[12]));

  usbip_usb_device.bDeviceClass = desc[4];
  usbip_usb_device.bDeviceSubClass = desc[5];
  usbip_usb_device.bDeviceProtocol = desc[6];

  usbip_usb_device.bConfigurationValue = 0;
  usbip_usb_device.bNumConfigurations = desc[17];
  usbip_usb_device.bNumInterfaces = 0;
}

#define USBIP_REPLY_HEADER_SIZE 8
#define DEVICE_INFO_SIZE        (256+32+12+6+6)
#define INTERFACE_INFO_SIZE     4
#define DEVICE_LIST_SIZE        (DEVICE_INFO_SIZE*1)

#define USBIP_REPLY_DEVICE_LIST "\x01\x11\x00\x05"
#define USBIP_REPLY_ATTACH      "\x01\x11\x00\x03"

#define NETWORK_UINT32_ZERO     "\x00\x00\x00\x00"
#define NETWORK_UINT32_ONE      "\x00\x00\x00\x01"
#define NETWORK_UINT32_TWO      "\x00\x00\x00\x02"
#define NETWORK_UINT16_FSIJ      "\x23\x4b"
#define NETWORK_UINT16_ZERO      "\x00\x00"
#define NETWORK_UINT16_ONE_ONE   "\x01\x01"

enum {
  USB_INTR_NONE = 0,
  USB_INTR_SETUP,
  USB_INTR_DATA_TRANSFER,
  USB_INTR_RESET,
  USB_INTR_SUSPEND,
};

struct usb_controller {
  pthread_mutex_t mutex;
  pthread_cond_t cond;
  uint8_t intr;
  uint8_t dir;
  uint8_t ep_num;
};

static struct usb_controller usbc;

static void
notify_device (uint8_t intr, uint8_t ep_num, uint8_t dir)
{
  pthread_mutex_lock (&usbc.mutex);
  if (usbc.intr)
    pthread_cond_wait (&usbc.cond, &usbc.mutex);
  usbc.intr = intr;
  usbc.dir = (dir == USBIP_DIR_IN);
  usbc.ep_num = ep_num;
  pthread_kill (tid_main, SIGUSR1);
  pthread_mutex_unlock (&usbc.mutex);
}


static const char *
list_devices (size_t *len_p)
{
  refresh_usb_device ();
  *len_p = sizeof (usbip_usb_device);
  return (const char *)&usbip_usb_device;
}

static const char *
attach_device (char busid[32], size_t *len_p)
{
  *len_p = 0;
  if (memcmp (busid, MY_BUS_ID, 32) != 0) 
    return NULL;

  //  notify_device (USB_INTR_RESET, 0, 0);

  refresh_usb_device ();
  *len_p = sizeof (usbip_usb_device);
  return (const char *)&usbip_usb_device;
}

#define URB_DATA_SIZE 65536

struct usbip_msg_ctl {
  uint32_t devid;
  uint32_t dir;
  uint32_t ep;
  uint32_t status;
  uint32_t len;
  uint32_t rsvd[2];
  uint32_t err_cnt;
};

struct urb {
  struct urb *next;
  struct urb *prev;
  pthread_t tid;
  uint32_t seq;
  uint32_t devid;
  uint32_t dir;
  uint32_t ep;
  uint32_t len;
  uint8_t setup[8];
  char data[URB_DATA_SIZE];
  char *data_p;
};

static pthread_mutex_t urb_mutex;
static struct urb *urb_list;

static uint8_t usb_setup[8];

static int control_setup_transaction (struct urb *urb);
static int control_write_data_transaction (char *buf, uint16_t count);
static int control_write_status_transaction (void);
static int control_read_data_transaction (char *buf);
static int control_read_status_transaction (void);
static int write_data_transaction (int ep_num, char *buf, uint16_t count);
static int read_data_transaction (int ep_num, char *buf);

enum {
  USB_STATE_DISABLED = 0,
  USB_STATE_STALL,
  USB_STATE_NAK,
  USB_STATE_SETUP,
  USB_STATE_TX,
  USB_STATE_RX,
};

struct usb_control {
  uint8_t state;
  uint8_t *buf;
  uint16_t len;
  pthread_mutex_t mutex;
  pthread_cond_t cond;
};

static struct usb_control usbc_ep0;

static int
handle_control_urb (struct urb *urb)
{
  int r;

  puts ("hcu 0");
  r = control_setup_transaction (urb);
  if (r < 0)
    return r;

  puts ("hcu 1");
  if (urb->dir == USBIP_DIR_OUT)
    {				/* Output from host to device.  */
      uint16_t remain = urb->len;
      uint16_t count;

      printf ("hcu: %d\n", r);
      while (r == 0)
	{
	  if (remain > 64)
	    count = 64;
	  else
	    count = remain;

	  r = control_write_data_transaction (urb->data_p, count);
	  if (r < 0)
	    break;

	  urb->data_p += count;
	  if (count < 64)
	    break;
	}
      if (r >= 0)
	r = control_write_status_transaction ();
    }
  else
    {				/* Input from device to host.  */
      int count = 0;
      puts ("hcu 2");
      while (1)
	{
	  r = control_read_data_transaction (urb->data_p);
	  if (r < 0)
	    break;

	  puts ("hcu 3");
	  count += r;
	  urb->data_p += r;
	  if (r < 64)
	    break;

	  if (count == URB_DATA_SIZE)
	    {
	      perror ("control IN too long");
	      return -1;
	    }
	}
      puts ("hcu 4");
      if (r >= 0)
	{
	  puts ("hcu 5");
	  r = control_read_status_transaction ();
	  if (r >= 0)
	    r = count;
	}
      printf ("hcu 6: %d\n", r);
    }

  return r;
}

static int
handle_data_urb  (struct urb *urb)
{
  int r;

  puts ("hdu 0");
  if (urb->dir == USBIP_DIR_OUT)
    {				/* Output from host to device.  */
      uint16_t remain = urb->len;
      uint16_t count;

      puts ("hdu 1");
      while (1)
	{
	  if (remain > 64)
	    count = 64;
	  else
	    count = remain;

	  puts ("hdu 2");
	  r = write_data_transaction (urb->ep, urb->data_p, count);
	  if (r < 0)
	    break;

	  puts ("hdu 3");
	  urb->data_p += count;
	  if (count < 64)
	    {
	      r = 0;
	      break;
	    }
	}
    }
  else
    {				/* Input from device to host.  */
      int count = 0;

      puts ("hdu 4");
      while (1)
	{
	  puts ("hdu 5");
	  r = read_data_transaction (urb->ep, urb->data_p);
	  if (r < 0)
	    break;

	  puts ("hdu 6");
	  count += r;
	  urb->data_p += r;
	  if (r < 64)
	    break;

	  if (count == URB_DATA_SIZE)
	    {
	      perror ("IN too long");
	      return -1;
	    }
	}
      if (r >= 0)
	r = count;
    }

  return r;
}

#define USB_REQ_GET_DESCRIPTOR		0x06

static struct urb urb_getdesc;

static const char *
issue_get_desc (void)
{
  urb_getdesc.setup[0] = 0x80;		         /* Type: GET, Standard, DEVICE */
  urb_getdesc.setup[1] = USB_REQ_GET_DESCRIPTOR; /* Request */
  urb_getdesc.setup[2] = 0;			 /* Value L: desc_index */
  urb_getdesc.setup[3] = 1;			 /* Value H: desc_type */
  urb_getdesc.setup[4] = 0;              	 /* Index */
  urb_getdesc.setup[5] = 0;
  urb_getdesc.setup[6] = 64;		         /* Length */
  urb_getdesc.setup[7] = 0;
  urb_getdesc.data_p = urb_getdesc.data;
  urb_getdesc.seq = 0;
  urb_getdesc.devid = 0;
  urb_getdesc.dir = USBIP_DIR_IN;
  urb_getdesc.ep = 0;
  urb_getdesc.len = 64;
  handle_control_urb (&urb_getdesc);
  return (const char *)urb_getdesc.data;
}

static pthread_mutex_t fd_mutex;

static void
unlink_urb (struct urb *urb)
{
  pthread_mutex_lock (&urb_mutex);
  urb->next->prev = urb->prev;
  urb->prev->next = urb->next;
  if (urb_list == urb)
    {
      if (urb->next == urb)
	urb_list = NULL;
      else
	urb_list = urb->next;
    }
  pthread_mutex_unlock (&urb_mutex);
}

static void * handle_urb_next (void *arg);

static int fd;

static void
handle_urb (uint32_t seq)
{
  int r = 0;
  struct usbip_msg_head msg;
  struct usbip_msg_ctl msg_ctl;
  struct urb *urb;
  const char zeros[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  pthread_attr_t attr;

  pthread_attr_init(&attr);
  pthread_attr_setdetachstate (&attr, PTHREAD_CREATE_DETACHED);

  urb = malloc (sizeof (struct urb));
  if (urb == NULL)
    {
      perror ("URB alloc");
      exit (1);
    }

  pthread_mutex_lock (&urb_mutex);
  if (urb_list == NULL)
    {
      urb_list = urb;
      urb->next = urb->prev = urb;
    }
  else
    {
      urb->next = urb_list;
      urb->prev = urb_list->prev;
      urb_list->prev = urb;
      urb_list = urb;
      urb->next = urb->prev = urb;
    }
  pthread_mutex_unlock (&urb_mutex);

  urb->seq = seq;
  if (recv (fd, (char *)&msg_ctl, sizeof (msg_ctl), 0) != sizeof (msg_ctl))
    {
      perror ("msg recv ctl");
      r = -EINVAL;
      goto leave;
    }

  urb->tid = 0;
  urb->devid = ntohl (msg_ctl.devid);
  urb->dir = ntohl (msg_ctl.dir);
  urb->ep = ntohl (msg_ctl.ep);
  urb->len = ntohl (msg_ctl.len);
  urb->data_p = urb->data;

  if (urb->len > URB_DATA_SIZE)
    {
      perror ("msg len too long");
      r = -EINVAL;
      goto leave;
    }

  printf ("URB: dir=%s, ep=%d, len=%d\n", urb->dir==USBIP_DIR_IN? "IN": "OUT",
	  urb->ep, urb->len);

  if (recv (fd, (char *)urb->setup, sizeof (urb->setup), 0) != sizeof (urb->setup))
    {
      perror ("msg recv setup");
      r = -EINVAL;
      goto leave;
    }

  if (urb->ep == 0)
    printf ("URB: %02x %02x %02x %02x %02x %02x %02x %02x\n",
	    urb->setup[0], urb->setup[1], urb->setup[2], urb->setup[3],
	    urb->setup[4], urb->setup[5], urb->setup[6], urb->setup[7]);

  if (urb->dir == USBIP_DIR_OUT && urb->len)
    {
      if (recv (fd, urb->data, urb->len, 0) != urb->len)
	{
	  perror ("msg recv data");
	  r = -EINVAL;
	  goto leave;
	}
    }

  r = pthread_create (&urb->tid, &attr, handle_urb_next, urb);
  pthread_attr_destroy (&attr);
  if (r == 0)
    {
      return;
    }

  r = -r;

 leave:
  msg.cmd = htonl (REP_URB_SUBMIT);
  msg.seq = htonl (urb->seq);

  msg_ctl.devid = 0;
  msg_ctl.dir = 0;
  msg_ctl.ep = 0;
  msg_ctl.status = htonl (r);
  msg_ctl.len = 0;
  msg_ctl.rsvd[0] = msg_ctl.rsvd[1] = 0;
  msg_ctl.err_cnt = 0;
  
  pthread_mutex_lock (&fd_mutex);
  if ((size_t)send (fd, &msg, sizeof (msg), 0) != sizeof (msg))
    {
      perror ("reply send");
    }

  if ((size_t)send (fd, &msg_ctl, sizeof (msg_ctl), 0) != sizeof (msg_ctl))
    {
      perror ("reply send");
    }

  if ((size_t)send (fd, zeros, sizeof (zeros), 0) != sizeof (zeros))
    {
      perror ("reply send");
    }

  pthread_mutex_unlock (&fd_mutex);
  unlink_urb (urb);
  free (urb);
}

static void *
handle_urb_next (void *arg)
{
  struct urb *urb = arg;
  int r = 0;
  struct usbip_msg_head msg;
  struct usbip_msg_ctl msg_ctl;
  const char zeros[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

  puts ("call h_u");
  if (urb->ep == 0)
    r = handle_control_urb (urb);
  else
    r = handle_data_urb (urb);

  printf ("hu-next: %d (%d)\n", r, urb->seq);
  if (urb->dir == USBIP_DIR_IN)
    {
      if (r >= 0)
	urb->len = r;
      else
	urb->len = 0;
    }
  else
    urb->len = 0;

  msg.cmd = htonl (REP_URB_SUBMIT);
  msg.seq = htonl (urb->seq);

  msg_ctl.devid = 0;
  msg_ctl.dir = 0;
  msg_ctl.ep = 0;
  msg_ctl.len = htonl (urb->len);
  msg_ctl.rsvd[0] = msg_ctl.rsvd[1] = 0;
  msg_ctl.err_cnt = 0;

  if (r < 0)
    msg_ctl.status = htonl (r);
  else
    msg_ctl.status = 0;

  pthread_mutex_lock (&fd_mutex);
  if ((size_t)send (fd, &msg, sizeof (msg), 0) != sizeof (msg))
    {
      perror ("reply send");
    }

  if ((size_t)send (fd, &msg_ctl, sizeof (msg_ctl), 0) != sizeof (msg_ctl))
    {
      perror ("reply send");
    }

  if ((size_t)send (fd, zeros, sizeof (zeros), 0) != sizeof (zeros))
    {
      perror ("reply send");
    }

  if (urb->dir == USBIP_DIR_IN && urb->len)
    {
      if (send (fd, urb->data, urb->len, 0) != urb->len)
	{
	  perror ("reply send");
	}
    }
  pthread_mutex_unlock (&fd_mutex);

  unlink_urb (urb);
  free (urb);
  return NULL;
}


static void
send_reply (char *reply, int ok)
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
  int attached = 0;

  pthread_mutex_init (&usbc.mutex, NULL);
  pthread_cond_init (&usbc.cond, NULL);

  pthread_mutex_init (&fd_mutex, NULL);
  pthread_mutex_init (&urb_mutex, NULL);

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

 again:
  /* We don't care who is connecting.  */
  if ((fd = accept (sock, NULL, NULL)) < 0)
    {
      perror ("accept");
      exit (1);
    }

  for (;;)
    {
      struct usbip_msg_head msg;

      if (recv (fd, (char *)&msg, sizeof (msg), 0) != sizeof (msg))
	{
	  perror ("msg recv");
	  break;
	}

      msg.cmd = ntohl (msg.cmd);
      msg.seq = ntohl (msg.seq);

      if (msg.cmd == CMD_REQ_LIST)
	{
	  const char *device_list;
	  size_t device_list_size;

	  printf ("Device List\n");
	  if (attached)
	    {
	      fprintf (stderr, "REQ list while attached\n");
	      break;
	    }

	  device_list = list_devices (&device_list_size);

	  pthread_mutex_lock (&fd_mutex);
	  send_reply (USBIP_REPLY_DEVICE_LIST, !!device_list);

	  if (send (fd, NETWORK_UINT32_ONE, 4, 0) == 4
	      && (size_t)send (fd, device_list, device_list_size, 0) == device_list_size)
	    pthread_mutex_unlock (&fd_mutex);
	  else
	    {
	      pthread_mutex_unlock (&fd_mutex);
	      perror ("list send");
	      break;
	    }

	  close (fd);
	  goto again;
	}
      else if (msg.cmd == CMD_REQ_ATTACH)
	{
	  const char *attach;
	  size_t attach_size;
	  char busid[32];

	  printf ("Attach device\n");
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

	  pthread_mutex_lock (&fd_mutex);
	  send_reply (USBIP_REPLY_ATTACH, !!attach);
	  if (attach
	      && (size_t)send (fd, attach, attach_size, 0) == attach_size)
	    {
	      printf ("Attach device!\n");
	      attached = 1;
	      pthread_mutex_unlock (&fd_mutex);
	    }
	  else
	    {
	      pthread_mutex_unlock (&fd_mutex);
	      perror ("attach send");
	      break;
	    }
	}
      else if (msg.cmd == CMD_URB_SUBMIT)
	{
	  if (!attached)
	    {
	      fprintf (stderr, "SUBMIT while not attached\n");
	      break;
	    }

	  printf ("URB SUBMIT! %d\n", msg.seq);
	  handle_urb (msg.seq);
	}
      else if (msg.cmd == CMD_URB_UNLINK)
	{
	  struct usbip_msg_ctl msg_ctl;
	  const char zeros[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	  uint32_t seq;
	  struct urb *urb;
	  char buf[8];

	  if (!attached)
	    {
	      fprintf (stderr, "UNLINK while not attached\n");
	      break;
	    }

	  if (recv (fd, (char *)&msg_ctl, sizeof (msg_ctl), 0) != sizeof (msg_ctl))
	    {
	      perror ("msg recv ctl");
	      break;
	    }

	  if (recv (fd, buf, sizeof (buf), 0) != sizeof (buf))
	    {
	      perror ("msg recv setup");
	      break;
	    }

	  seq = ntohl (msg_ctl.status);
	  printf ("URB UNLINK! %d\n", seq);

	  pthread_mutex_lock (&urb_mutex);
	  if (urb_list)
	    {
	      int found = 0;

	      urb = urb_list;
	      do
		{
		  if (urb->seq == seq)
		    {
		      found = 1;
		      break;
		    }
		  urb = urb->next;
		}
	      while (urb != urb_list);

	      if (found)
		{
		  pthread_cancel (urb->tid);
		  urb->next->prev = urb->prev;
		  urb->prev->next = urb->next;
		  if (urb_list == urb)
		    {
		      if (urb->next == urb)
			urb_list = NULL;
		      else
			urb_list = urb->next;
		    }
		}
	    }
	  pthread_mutex_unlock (&urb_mutex);

	  msg.cmd = htonl (REP_URB_UNLINK);
	  msg.seq = htonl (msg.seq);

	  msg_ctl.devid = 0;
	  msg_ctl.dir = 0;
	  msg_ctl.ep = 0;
	  msg_ctl.status = 0;

	  pthread_mutex_lock (&fd_mutex);
	  if ((size_t)send (fd, &msg, sizeof (msg), 0) != sizeof (msg))
	    {
	      perror ("reply send");
	    }

	  if ((size_t)send (fd, &msg_ctl, sizeof (msg_ctl), 0) != sizeof (msg_ctl))
	    {
	      perror ("reply send");
	    }

	  if ((size_t)send (fd, zeros, sizeof (zeros), 0) != sizeof (zeros))
	    {
	      perror ("reply send");
	    }
	  pthread_mutex_unlock (&fd_mutex);
	}
      else
	{
	  fprintf (stderr, "Unknown command %08x, disconnecting.\n", msg.cmd);
	  break;
	}
    }

  close (fd);
  close (sock);

  return NULL;
}

static struct usb_control usbc_ep_in[7];
static struct usb_control usbc_ep_out[7];

/** Control setup transaction.
 *
 * -->[SETUP]-->[DATA0]-->{ACK}-> Success
 *                      \-------> Error
 */
static int
control_setup_transaction (struct urb *urb)
{
  int r;

  pthread_mutex_lock (&usbc_ep0.mutex);
  while (1)
    if (usbc_ep0.state == USB_STATE_NAK)
      pthread_cond_wait (&usbc_ep0.cond, &usbc_ep0.mutex);
    else if (usbc_ep0.state == USB_STATE_SETUP)
      {
	if (urb->dir == USBIP_DIR_OUT
	    && urb->setup[6] == 0 && urb->setup[7] == 0)
	  /* Control Write Transfer with no data stage.  */
	  r = 1;
	else
	  r = 0;

	usbc_ep0.state = USB_STATE_NAK;
	memcpy (usb_setup, urb->setup, sizeof (usb_setup));
	notify_device (USB_INTR_SETUP, 0, urb->dir);
	break;
      }
    else
      {
	printf ("cst error %d\n", usbc_ep0.state);
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
      pthread_cond_wait (&usbc_ep0.cond, &usbc_ep0.mutex);
    else if (usbc_ep0.state == USB_STATE_RX)
      {
	if (usbc_ep0.len < count)
	  {
	    fprintf (stderr, "*** usbc_ep0.len < count");
	    r = -1;
	    usbc_ep0.state = USB_STATE_STALL;
	  }
	else
	  {
	    r = 0;
	    usbc_ep0.state = USB_STATE_NAK;
	    memcpy (usbc_ep0.buf, buf, count);
	    usbc_ep0.len = count;
	    notify_device (USB_INTR_DATA_TRANSFER, 0, USBIP_DIR_OUT);
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
      pthread_cond_wait (&usbc_ep0.cond, &usbc_ep0.mutex);
    else if (usbc_ep0.state == USB_STATE_TX)
      {
	puts ("control_write_status_transaction");
	if (usbc_ep0.len != 0)
	  fprintf (stderr, "*** ACK length %d\n", usbc_ep0.len);
	usbc_ep0.state = USB_STATE_NAK;
	notify_device (USB_INTR_DATA_TRANSFER, 0, USBIP_DIR_IN);
	r = 0;
	break;
      }
    else
      {
	r = -EPIPE;
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
      pthread_cond_wait (&usbc_ep0.cond, &usbc_ep0.mutex);
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
	    notify_device (USB_INTR_DATA_TRANSFER, 0, USBIP_DIR_IN);
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
      pthread_cond_wait (&usbc_ep0.cond, &usbc_ep0.mutex);
    else if (usbc_ep0.state == USB_STATE_RX)
      {
	usbc_ep0.len = 0;
	usbc_ep0.state = USB_STATE_NAK;
	notify_device (USB_INTR_DATA_TRANSFER, 0, USBIP_DIR_OUT);
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

#define USB_URB_TIMEOUT 1000000 /* nanosecond */

/* WRITE transaction
 * -->[OUT]--->[DATAx]--->{ACK}---> Success
 *                     \--{NAK}---> again
 *                     \--{STALL}-> Error
 *                     \----------> Error
 */
static int
write_data_transaction (int ep_num, char *buf, uint16_t count)
{
  int r;
  struct usb_control *usbc_p = &usbc_ep_out[ep_num];

  pthread_mutex_lock (&usbc_p->mutex);
  while (1)
    if (usbc_p->state == USB_STATE_NAK)
      pthread_cond_wait (&usbc_p->cond, &usbc_p->mutex);
    else if (usbc_p->state == USB_STATE_RX)
      {
	if (usbc_p->len < count)
	  {
	    r = -1;
	    usbc_p->state = USB_STATE_STALL;
	  }
	else
	  {
	    r = 0;
	    usbc_p->state = USB_STATE_NAK;
	    memcpy (usbc_p->buf, buf, count);
	    usbc_p->len = count;
	    notify_device (USB_INTR_DATA_TRANSFER, ep_num, USBIP_DIR_OUT);
	  }
	break;
      }
    else
      {
	r = -1;
	break;
      }
  pthread_mutex_unlock (&usbc_p->mutex);
  return r;
}

/** READ transaction.
 *
 * -->[IN]-->{DATAx}---->[ACK]---> Success
 *         |          \---------> Error
 *         \-{STALL}------------> Error
 *         \-{NAK}--------------> again
 */
static int
read_data_transaction (int ep_num, char *buf)
{
  int r;
  uint16_t count;
  struct usb_control *usbc_p = &usbc_ep_in[ep_num];

  pthread_mutex_lock (&usbc_p->mutex);
  while (1)
    if (usbc_p->state == USB_STATE_NAK)
      pthread_cond_wait (&usbc_p->cond, &usbc_p->mutex);
    else if (usbc_p->state == USB_STATE_TX)
      {
	if (usbc_p->len > 64)
	  {
	    fprintf (stderr, "*** length %d\n", usbc_p->len);
	    r = -1;
	  }
	else
	  {
	    count = usbc_p->len;
	    memcpy (buf, usbc_p->buf, count);
	    usbc_p->state = USB_STATE_NAK;
	    notify_device (USB_INTR_DATA_TRANSFER, 0, USBIP_DIR_IN);
	    r = count;
	  }
	break;
      }
    else
      {
	r = -1;
	break;
      }
  pthread_mutex_unlock (&usbc_p->mutex);
  return r;
}

void chx_handle_intr (uint32_t irq_num);

#define INTR_REQ_USB SIGUSR1

static void
usb_intr (int signum, siginfo_t *siginfo, void *arg)
{
  extern void chx_sigmask (ucontext_t *uc);

  ucontext_t *uc = arg;
  (void)signum;
  (void)siginfo;
  chx_sigmask (uc);
  chx_handle_intr (INTR_REQ_USB);
}


void
usb_lld_init (struct usb_dev *dev, uint8_t feature)
{
  int r;
  sigset_t sigset;
  struct sigaction act;

  sigemptyset (&sigset);
  sigaddset (&sigset, SIGUSR1);
  sigaddset (&sigset, SIGALRM);

  /*
   * Launch the thread for USBIP.  This maps to usb_lld_sys_init where
   * we initialize USB controller on MCU.
   */
  tid_main = pthread_self ();

  pthread_sigmask (SIG_BLOCK, &sigset, NULL);

  r = pthread_create (&tid_usbip, NULL, run_server, NULL);
  if (r)
    {
      fprintf (stderr, "usb_lld_init: %s\n", strerror (r));
      exit (1);
    }

  act.sa_sigaction = usb_intr;
  sigfillset (&act.sa_mask);
  act.sa_flags = SA_SIGINFO;
  sigaction (SIGUSR1, &act, NULL);

  pthread_sigmask (SIG_UNBLOCK, &sigset, NULL);

  pthread_mutex_init (&usbc_ep0.mutex, NULL);
  pthread_cond_init (&usbc_ep0.cond, NULL);

  dev->configuration = 0;
  dev->feature = feature;
  dev->state = WAIT_SETUP;

  usbc_ep0.state = USB_STATE_SETUP;
}


void
usb_lld_prepare_shutdown (void)
{
  sigset_t sigset;

  sigemptyset (&sigset);
  sigaddset (&sigset, SIGUSR1);
  pthread_sigmask (SIG_BLOCK, &sigset, NULL);
}

void
usb_lld_shutdown (void)
{
  /* 
   * Stop USBIP server thread.
   */
  pthread_cancel (tid_usbip);
  pthread_join (tid_usbip, NULL);
  /* FIXME: Cancel all URB threads here.  */
}

#define USB_MAKE_EV(event) (event<<24)
#define USB_MAKE_TXRX(ep_num,txrx,len) ((txrx? (1<<23):0)|(ep_num<<16)|len)

static int handle_setup0 (struct usb_dev *dev);
static int usb_handle_transfer (struct usb_dev *dev, uint8_t dir, uint8_t ep_num);

int
usb_lld_event_handler (struct usb_dev *dev)
{
  uint8_t intr;
  uint8_t dir;
  uint8_t ep_num;

  pthread_mutex_lock (&usbc.mutex);
  intr = usbc.intr;
  dir = usbc.dir;
  ep_num = usbc.ep_num;
  usbc.intr = USB_INTR_NONE;
  pthread_cond_signal (&usbc.cond);
  pthread_mutex_unlock (&usbc.mutex);

  if (intr == USB_INTR_RESET)
    return USB_MAKE_EV (USB_EVENT_DEVICE_RESET);
  else if (intr == USB_INTR_SETUP)
    return USB_MAKE_EV (handle_setup0 (dev));
  else if (intr == USB_INTR_DATA_TRANSFER)
    return usb_handle_transfer (dev, dir, ep_num);

  return USB_EVENT_OK;
}

static void handle_datastage_out (struct usb_dev *dev)
{
  struct ctrl_data *data_p = &dev->ctrl_data;
  uint32_t len;

  pthread_mutex_lock (&usbc_ep0.mutex);
  len = usbc_ep0.len;
  data_p->len -= len;
  data_p->addr += len;

  len = data_p->len;
  if (len > USB_MAX_PACKET_SIZE)
    len = USB_MAX_PACKET_SIZE;

  if (dev->ctrl_data.len == 0)
    {
      dev->state = WAIT_STATUS_IN;
      usbc_ep0.buf = usb_setup;
      usbc_ep0.len = 0;
      usbc_ep0.state = USB_STATE_TX;
    }
  else
    {
      dev->state = OUT_DATA;
      usbc_ep0.buf = data_p->addr;
      usbc_ep0.len = len;
      usbc_ep0.state = USB_STATE_RX;
    }

  pthread_cond_signal (&usbc_ep0.cond);
  pthread_mutex_unlock (&usbc_ep0.mutex);
}

static void handle_datastage_in (struct usb_dev *dev)
{
  struct ctrl_data *data_p = &dev->ctrl_data;
  uint32_t len = USB_MAX_PACKET_SIZE;

  if ((data_p->len == 0) && (dev->state == LAST_IN_DATA))
    {
      pthread_mutex_lock (&usbc_ep0.mutex);

      if (data_p->require_zlp)
	{
	  data_p->require_zlp = 0;

	  /* No more data to send.  Send empty packet */
	  usbc_ep0.buf = usb_setup;
	  usbc_ep0.len = 0;
	  usbc_ep0.state = USB_STATE_TX;
	}
      else
	{
	  /* No more data to send, proceed to receive OUT acknowledge.  */
	  dev->state = WAIT_STATUS_OUT;
	  usbc_ep0.buf = usb_setup;
	  usbc_ep0.len = 0;
	  usbc_ep0.state = USB_STATE_RX;
	}

      pthread_cond_signal (&usbc_ep0.cond);
      pthread_mutex_unlock (&usbc_ep0.mutex);
      return;
    }

  dev->state = (data_p->len <= len) ? LAST_IN_DATA : IN_DATA;

  if (len > data_p->len)
    len = data_p->len;

  data_p->len -= len;
  data_p->addr += len;

  pthread_mutex_lock (&usbc_ep0.mutex);
  usbc_ep0.buf = data_p->addr;
  usbc_ep0.len = len;
  usbc_ep0.state = USB_STATE_TX;
  pthread_cond_signal (&usbc_ep0.cond);
  pthread_mutex_unlock (&usbc_ep0.mutex);
}

typedef int (*HANDLER) (struct usb_dev *dev);

static int std_none (struct usb_dev *dev)
{
  (void)dev;
  return -1;
}

static uint16_t status_info;

static int std_get_status (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = (arg->type & RECIPIENT);

  if (arg->value != 0 || arg->len != 2 || (arg->index >> 8) != 0
      || USB_SETUP_SET (arg->type))
    return -1;

  if (rcp == DEVICE_RECIPIENT)
    {
      if (arg->index == 0)
	{
	  /* Get Device Status */
	  uint8_t feature = dev->feature;

	  /* Remote Wakeup enabled */
	  if ((feature & (1 << 5)))
	    status_info |= 2;
	  else
	    status_info &= ~2;

	  /* Bus-powered */
	  if ((feature & (1 << 6)))
	    status_info |= 1;
	  else /* Self-powered */
	    status_info &= ~1;

	  return usb_lld_ctrl_send (dev, &status_info, 2);
	}
    }
  else if (rcp == INTERFACE_RECIPIENT)
    {
      if (dev->configuration == 0)
	return -1;

      return USB_EVENT_GET_STATUS_INTERFACE;
    }
  else if (rcp == ENDPOINT_RECIPIENT)
    {
      uint8_t ep_num = (arg->index & 0x0f);

      if ((arg->index & 0x70) || ep_num == ENDP0)
	return -1;

      if ((arg->index & 0x80))
	{
	  if (usbc_ep_in[ep_num].state == USB_STATE_DISABLED)
	    return -1;
	  else if (usbc_ep_in[ep_num].state == USB_STATE_STALL)
	    status_info |= 1;
	}
      else
	{
	  if (usbc_ep_out[ep_num].state == USB_STATE_DISABLED)
	    return -1;
	  else if (usbc_ep_out[ep_num].state == USB_STATE_STALL)
	    status_info |= 1;
	}

      return usb_lld_ctrl_send (dev, &status_info, 2);
    }

  return -1;
}

static int std_clear_feature (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (USB_SETUP_GET (arg->type))
    return -1;

  if (rcp == DEVICE_RECIPIENT)
    {
      if (arg->len != 0 || arg->index != 0)
	return -1;

      if (arg->value == FEATURE_DEVICE_REMOTE_WAKEUP)
	{
	  dev->feature &= ~(1 << 5);
	  return USB_EVENT_CLEAR_FEATURE_DEVICE;
	}
    }
  else if (rcp == ENDPOINT_RECIPIENT)
    {
      uint8_t ep_num = (arg->index & 0x0f);

      if (dev->configuration == 0)
	return -1;

      if (arg->len != 0 || (arg->index >> 8) != 0
	  || arg->value != FEATURE_ENDPOINT_HALT || ep_num == ENDP0)
	return -1;

      if ((arg->index & 0x80))
	{
	  if (usbc_ep_in[ep_num].state == USB_STATE_DISABLED)
	    return -1;

	  usbc_ep_in[ep_num].state = USB_STATE_NAK;
	}
      else
	{
	  if (usbc_ep_out[ep_num].state == USB_STATE_DISABLED)
	    return -1;

	  usbc_ep_out[ep_num].state = USB_STATE_NAK;
	}

      return USB_EVENT_CLEAR_FEATURE_ENDPOINT;
    }

  return -1;
}

static int std_set_feature (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (USB_SETUP_GET (arg->type))
    return -1;

  if (rcp == DEVICE_RECIPIENT)
    {
      if (arg->len != 0 || arg->index != 0)
	return -1;

      if (arg->value == FEATURE_DEVICE_REMOTE_WAKEUP)
	{
	  dev->feature |= 1 << 5;
	  return USB_EVENT_SET_FEATURE_DEVICE;
	}
    }
  else if (rcp == ENDPOINT_RECIPIENT)
    {
      uint8_t ep_num = (arg->index & 0x0f);

      if (dev->configuration == 0)
	return -1;

      if (arg->len != 0 || (arg->index >> 8) != 0
	  || arg->value != FEATURE_ENDPOINT_HALT || ep_num == ENDP0)
	return -1;

      if ((arg->index & 0x80))
	{
	  if (usbc_ep_in[ep_num].state == USB_STATE_DISABLED)
	    return -1;

	  usbc_ep_in[ep_num].state = USB_STATE_STALL;
	}
      else
	{
	  if (usbc_ep_out[ep_num].state == USB_STATE_DISABLED)
	    return -1;

	  usbc_ep_out[ep_num].state = USB_STATE_STALL;
	}

      return USB_EVENT_SET_FEATURE_ENDPOINT;
    }

  return -1;
}

static int std_set_address (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (USB_SETUP_GET (arg->type))
    return -1;

  if (rcp == DEVICE_RECIPIENT && arg->len == 0 && arg->value <= 127
      && arg->index == 0 && dev->configuration == 0)
    return usb_lld_ctrl_ack (dev);

  return -1;
}

static int std_get_descriptor (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  if (USB_SETUP_SET (arg->type))
    return -1;

  return USB_EVENT_GET_DESCRIPTOR;
}

static int std_get_configuration (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (USB_SETUP_SET (arg->type))
    return -1;

  if (arg->value != 0 || arg->index != 0 || arg->len != 1)
    return -1;

  if (rcp == DEVICE_RECIPIENT)
    return usb_lld_ctrl_send (dev, &dev->configuration, 1);

  return -1;
}

static int std_set_configuration (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (USB_SETUP_GET (arg->type))
    return -1;

  if (arg->index != 0 || arg->len != 0)
    return -1;

  if (rcp == DEVICE_RECIPIENT)
    return USB_EVENT_SET_CONFIGURATION;

  return -1;
}

static int std_get_interface (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (USB_SETUP_SET (arg->type))
    return -1;

  if (arg->value != 0 || (arg->index >> 8) != 0 || arg->len != 1)
    return -1;

  if (dev->configuration == 0)
    return -1;

  if (rcp == INTERFACE_RECIPIENT)
    return USB_EVENT_GET_INTERFACE;

  return -1;
}

static int std_set_interface (struct usb_dev *dev)
{
  struct device_req *arg = &dev->dev_req;
  uint8_t rcp = arg->type & RECIPIENT;

  if (USB_SETUP_GET (arg->type) || rcp != INTERFACE_RECIPIENT
      || arg->len != 0 || (arg->index >> 8) != 0
      || (arg->value >> 8) != 0 || dev->configuration == 0)
    return -1;

  return USB_EVENT_SET_INTERFACE;
}


static int
handle_setup0 (struct usb_dev *dev)
{
  uint8_t req_no;
  HANDLER handler;

  dev->dev_req.type = usb_setup[0];
  dev->dev_req.request = req_no = usb_setup[1];
  dev->dev_req.value = (usb_setup[3] << 8) + usb_setup[2];
  dev->dev_req.index = (usb_setup[5] << 8) + usb_setup[4];
  dev->dev_req.len = (usb_setup[7] << 8) + usb_setup[6];

  dev->ctrl_data.addr = NULL;
  dev->ctrl_data.len = 0;
  dev->ctrl_data.require_zlp = 0;

  if ((dev->dev_req.type & REQUEST_TYPE) == STANDARD_REQUEST)
    {
      int r;

      switch (req_no)
	{
	case 0: handler = std_get_status;  break;
	case 1: handler = std_clear_feature;  break;
	case 3: handler = std_set_feature;  break;
	case 5: handler = std_set_address;  break;
	case 6: handler = std_get_descriptor;  break;
	case 8: handler = std_get_configuration;  break;
	case 9: handler = std_set_configuration;  break;
	case 10: handler = std_get_interface;  break;
	case 11: handler = std_set_interface;  break;
	default: handler = std_none;  break;
	}

      if ((r = (*handler) (dev)) < 0)
	{
	  usb_lld_ctrl_error (dev);
	  return USB_EVENT_OK;
	}
      else
	return r;
    }
  else
    return USB_EVENT_CTRL_REQUEST;
}

static int handle_in0 (struct usb_dev *dev)
{
  int r = 0;

  if (dev->state == IN_DATA || dev->state == LAST_IN_DATA)
    handle_datastage_in (dev);
  else if (dev->state == WAIT_STATUS_IN)
    {
      if ((dev->dev_req.request == SET_ADDRESS) &&
	  ((dev->dev_req.type & (REQUEST_TYPE | RECIPIENT))
	   == (STANDARD_REQUEST | DEVICE_RECIPIENT)))
	{
	  /* XXX: record the assigned address of this device??? */
	  printf ("Set Address: %d\n", dev->dev_req.value);
	  r = USB_EVENT_DEVICE_ADDRESSED;
	}
      else
	r = USB_EVENT_CTRL_WRITE_FINISH;
      dev->state = WAIT_SETUP;
      pthread_mutex_lock (&usbc_ep0.mutex);
      usbc_ep0.buf = usb_setup;
      usbc_ep0.len = 8;
      usbc_ep0.state = USB_STATE_SETUP;
      pthread_mutex_unlock (&usbc_ep0.mutex);
    }
  else
    {
      puts ("handle_in0 error");
      dev->state = STALLED;
      pthread_mutex_lock (&usbc_ep0.mutex);
      usbc_ep0.state = USB_STATE_STALL;
      pthread_cond_signal (&usbc_ep0.cond);
      pthread_mutex_unlock (&usbc_ep0.mutex);
    }

  return r;
}

static void handle_out0 (struct usb_dev *dev)
{
  if (dev->state == OUT_DATA)
    /* Usual case.  */
    handle_datastage_out (dev);
  else if (dev->state == WAIT_STATUS_OUT)
    {
      /*
       * Control READ transfer finished by ZLP.
       * Leave ENDP0 status RX_NAK, TX_NAK.
       */
      dev->state = WAIT_SETUP;
      pthread_mutex_lock (&usbc_ep0.mutex);
      usbc_ep0.buf = usb_setup;
      usbc_ep0.len = 8;
      usbc_ep0.state = USB_STATE_SETUP;
      pthread_mutex_unlock (&usbc_ep0.mutex);
    }
  else
    {
      /*
       * dev->state == IN_DATA || dev->state == LAST_IN_DATA
       * (Host aborts the transfer before finish)
       * Or else, unexpected state.
       * STALL the endpoint, until we receive the next SETUP token.
       */
      puts ("handle_out0 error");
      dev->state = STALLED;
      pthread_mutex_lock (&usbc_ep0.mutex);
      usbc_ep0.state = USB_STATE_STALL;
      pthread_cond_signal (&usbc_ep0.cond);
      pthread_mutex_unlock (&usbc_ep0.mutex);
    }
}


static int
usb_handle_transfer (struct usb_dev *dev, uint8_t dir, uint8_t ep_num)
{
  if (ep_num == 0)
    {
      if (dir)
	return USB_MAKE_EV (handle_in0 (dev));
      else
	{
	  handle_out0 (dev);
	  return USB_EVENT_OK;
	}
    }
  else
    {
      uint16_t len;

      if (dir)
	{
	  len = usbc_ep_in[ep_num].len;
	  return USB_MAKE_TXRX (ep_num, 0, len);
	}
      else
	{
	  len = usbc_ep_out[ep_num].len;
	  return  USB_MAKE_TXRX (ep_num, 1, len);
	}
    }

  return USB_EVENT_OK;
}

int
usb_lld_ctrl_ack (struct usb_dev *dev)
{
  dev->state = WAIT_STATUS_IN;
  pthread_mutex_lock (&usbc_ep0.mutex);
  usbc_ep0.buf = usb_setup;
  usbc_ep0.len = 0;
  usbc_ep0.state = USB_STATE_TX;
  pthread_cond_signal (&usbc_ep0.cond);
  pthread_mutex_unlock (&usbc_ep0.mutex);
  return USB_EVENT_OK;
}

int
usb_lld_ctrl_recv (struct usb_dev *dev, void *p, size_t len)
{
  struct ctrl_data *data_p = &dev->ctrl_data;
  data_p->addr = p;
  data_p->len = len;
  dev->state = OUT_DATA;
  if (len > USB_MAX_PACKET_SIZE)
    len = USB_MAX_PACKET_SIZE;
  pthread_mutex_lock (&usbc_ep0.mutex);
  usbc_ep0.state = USB_STATE_RX;
  usbc_ep0.buf = p;
  usbc_ep0.len = len;
  pthread_cond_signal (&usbc_ep0.cond);
  pthread_mutex_unlock (&usbc_ep0.mutex);
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

  pthread_mutex_lock (&usbc_ep0.mutex);
  usbc_ep0.buf = data_p->addr;
  usbc_ep0.len = len;
  usbc_ep0.state = USB_STATE_TX;
  data_p->len -= len;
  data_p->addr += len;
  pthread_cond_signal (&usbc_ep0.cond);
  pthread_mutex_unlock (&usbc_ep0.mutex);
  return USB_EVENT_OK;
}

uint8_t
usb_lld_current_configuration (struct usb_dev *dev)
{
  return dev->configuration;
}


void
usb_lld_ctrl_error (struct usb_dev *dev)
{
  puts ("ctrl_error");
  dev->state = STALLED;
  pthread_mutex_lock (&usbc_ep0.mutex);
  usbc_ep0.state = USB_STATE_STALL;
  pthread_cond_signal (&usbc_ep0.cond);
  pthread_mutex_unlock (&usbc_ep0.mutex);
}

void
usb_lld_reset (struct usb_dev *dev, uint8_t feature)
{
  usb_lld_set_configuration (dev, 0);
  dev->feature = feature;
  usbc_ep0.state = USB_STATE_SETUP;
}

void
usb_lld_set_configuration (struct usb_dev *dev, uint8_t config)
{
  dev->configuration = config;
}


void
usb_lld_setup_endp (struct usb_dev *dev, int ep_num, int rx_en, int tx_en)
{
  (void)dev;

  if (ep_num == 0)
    return;

  if (rx_en)
    {
      usbc_ep_out[ep_num].buf = NULL;
      usbc_ep_out[ep_num].len = 0;
      usbc_ep_out[ep_num].state = USB_STATE_NAK;
      pthread_mutex_init (&usbc_ep_out[ep_num].mutex, NULL);
      pthread_cond_init (&usbc_ep_out[ep_num].cond, NULL);
    }

  if (tx_en)
    {
      usbc_ep_in[ep_num].buf = NULL;
      usbc_ep_in[ep_num].len = 0;
      usbc_ep_in[ep_num].state = USB_STATE_NAK;
      pthread_mutex_init (&usbc_ep_in[ep_num].mutex, NULL);
      pthread_cond_init (&usbc_ep_in[ep_num].cond, NULL);
    }
}


void
usb_lld_stall_tx (int ep_num)
{
  printf ("stall tx %d", ep_num);
  usbc_ep_in[ep_num].state = USB_STATE_STALL;
}

void
usb_lld_stall_rx (int ep_num)
{
  printf ("stall rx %d", ep_num);
  usbc_ep_out[ep_num].state = USB_STATE_STALL;
}

void
usb_lld_rx_enable_buf (int ep_num, void *buf, size_t len)
{
  struct usb_control *usbc_p = &usbc_ep_out[ep_num];

  pthread_mutex_lock (&usbc_p->mutex);
  usbc_p->state = USB_STATE_RX;
  usbc_p->buf = buf;
  usbc_p->len = len;
  pthread_cond_signal (&usbc_p->cond);
  pthread_mutex_unlock (&usbc_p->mutex);
}


void
usb_lld_tx_enable_buf (int ep_num, const void *buf, size_t len)
{
  struct usb_control *usbc_p = &usbc_ep_in[ep_num];

  pthread_mutex_lock (&usbc_p->mutex);
  usbc_p->state = USB_STATE_TX;
  usbc_p->buf = (void *)buf;
  usbc_p->len = len;
  pthread_cond_signal (&usbc_p->cond);
  pthread_mutex_unlock (&usbc_p->mutex);
}
