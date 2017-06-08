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

struct usbip_msg_head {
  uint32_t cmd;
  uint32_t seq;
};

#define USBIP_REPLY_HEADER_SIZE 12
#define DEVICE_INFO_SIZE        (256+32+12+6+6)
#define INTERFACE_INFO_SIZE     4
#define DEVICE_LIST_SIZE        (USBIP_REPLY_HEADER_SIZE+DEVICE_INFO_SIZE*1+INTERFACE_INFO_SIZE*1)

#define USBIP_REPLY_DEVICE_LIST "\x01\x11\x00\x05"
#define NETWORK_UINT32_ZERO     "\x00\x00\x00\x00"
#define NETWORK_UINT32_ONE      "\x00\x00\x00\x01"
#define NETWORK_UINT32_TWO      "\x00\x00\x00\x02"
#define NETWORK_UINT16_FSIJ      "\x23\x4b"
#define NETWORK_UINT16_ZERO      "\x00\x00"
#define NETWORK_UINT16_ONE_ONE   "\x01\x01"

static void
notify_app (void)
{
  pthread_kill (tid_main, SIGUSR1);
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

  p = p0;
  memcpy (p, USBIP_REPLY_DEVICE_LIST, 4);
  p += 4;
  memcpy (p, NETWORK_UINT32_ZERO, 4);
  p += 4;
  memcpy (p, NETWORK_UINT32_ONE, 4);
  p += 4;
  memset (p, 0, 256);
  strcpy (p, "/sys/devices/pci0000:00/0000:00:01.1/usb1/1-1");
  p += 256;
  memset (p, 0, 32);
  strcpy (p, "1-1");
  p += 32;
  memcpy (p, NETWORK_UINT32_ONE, 4); /* Bus */
  p += 4;
  memcpy (p, NETWORK_UINT32_TWO, 4); /* Dev */
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
  *p++ = 1; /* bConfigurationValue */
  *p++ = 1; /* bNumInterfaces      */

  *p++ = 11; /* bInterfaceClass    */
  *p++ = 0;  /* bInterfaceSubClass */
  *p++ = 0;  /* bInterfaceProtocol */
  *p++ = 0;  /* ----pad----------- */

  return p0;
}

static char *
attach_device (char busid[32], size_t *len_p)
{
  (void)busid;

  *len_p = 0;
  return NULL;
}

struct usbip_msg_ctl {
  uint32_t devid;
  uint32_t dir;
  uint32_t ep;
  uint32_t flags;
  uint32_t len;
  uint32_t start_frame;		/* Only for ISO, INTERRUPT */
  uint32_t num_packets;		/* Only for ISO            */
  uint32_t interval;		/* Only for ISO, INTERRUPT */
  uint8_t setup[8];		/* Only for CONTROL        */
};


static int
handle_control_urb  (int fd, uint32_t seq, struct usbip_msg_ctl *cp)
{
  int r;

  r = setup_transaction (fd, seq, cp);
  if (r)
    return r;

  if (cp->dir)
    {				/* Output from host to device.  */
      while (r == 0)
	r = control_out_data_transaction ();
      if (r > 0)
	r = control_out_ack_transaction ();
    }
  else
    {				/* Input from device to host.  */
      while (r == 0)
	r = control_in_data_transaction ();
      if (r > 0)
	r = control_in_ack_transaction ();
    }

  return r;
}

static int
handle_data_urb  (int fd, uint32_t seq, struct usbip_msg_ctl *cp)
{
  if (cp->dir)
    {				/* Output from host to device.  */
      while (r == 0)
	r = out_data_transaction ();
    }
  else
    {				/* Input from device to host.  */
      while (r == 0)
	r = in_data_transaction ();
    }

  return r;
}

static int
handle_urb (int fd, uint32_t seq)
{
  struct usbip_msg_ctl msg_ctl;

  if (recv (fd, (char *)&msg_ctl, sizeof (msg_ctl), 0) != sizeof (msg_ctl))
    {
      perror ("msg recv ctl");
      return -1;
    }

  msg_ctl.devid = ntohl (msg_ctl.devid);
  msg_ctl.dir = ntohl (msg_ctl.dir);
  msg_ctl.ep = ntohl (msg_ctl.ep);
  msg_ctl.flags = ntohl (msg_ctl.flags);
  msg_ctl.len = ntohl (msg_ctl.);
  msg_ctl.start_frame = ntohl (msg_ctl.start_frame);
  msg_ctl.num_packets = ntohl (msg_ctl.num_packets);
  msg_ctl.interval = ntohl (msg_ctl.interval);

  if (ep == 0)
    return handle_control_urb (fd, seq, &msg_ctl);
  else
    return handle_data_urb (fd, seq, &msg_ctl);
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

/*
 * In the USBIP protocol, it sends URB (USB Request Block) to this server. 
 *
 * This server acts/emulates as a USB host controller, and
 * transforms URB into packets and packets into URB.
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

	      if (attached)
		{
		  fprintf (stderr, "REQ list while attached\n");
		  break;
		}

	      device_list = list_devices (&device_list_size);

	      if ((size_t)send (fd, device_list, device_list_size, 0) != device_list_size)
		{
		  perror ("list send");
		  break;
		}

	      free (device_list);
	    }
	  else if (msg.cmd == CMD_REQ_ATTACH)
	    {
	      char busid[32];
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
	      if ((size_t)send (fd, attach, attach_size, 0) != attach_size)
		{
		  perror ("list send");
		  break;
		}

	      free (attach);
	      attached = 1;
	    }
	  else if (msg.cmd == CMD_URB)
	    {
	      if (!attached)
		{
		  fprintf (stderr, "URB while attached\n");
		  break;
		}

	      if (handle_urb (fd, msg.seq) < 0)
		{
		  fprintf (stderr, "URB handling failed\n");
		  break;
		}
	    }
	  else if(msg.cmd == CMD_DETACH)
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

#define EP_TX_VALID (1<<0)
#define EP_RX_VALID (1<<1)

struct usb_control {
  uint16_t tx_count;
  uint16_t rx_count;
  uint8_t  status_flag;
  uint8_t tx_data[USB_MAX_PACKET_SIZE];
  uint8_t rx_data[USB_MAX_PACKET_SIZE];
};

static struct usb_control usbc_ep0;
static struct usb_control usbc_ep1;
static struct usb_control usbc_ep2;
static struct usb_control usbc_ep3;
static struct usb_control usbc_ep5;
static struct usb_control usbc_ep6;
static struct usb_control usbc_ep7;

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
  usbc_ep0.tx_count = 0;
  usbc_ep0.status_flag = EP_TX_VALID;
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
  usbc_ep0.status_flag = EP_RX_VALID;
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
      memcpy (usbc_ep0.tx_data, data_p->addr, len);
      data_p->len -= len;
      data_p->addr += len;
    }

  usbc_ep0.status_flag = EP_TX_VALID;
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
