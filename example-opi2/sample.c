#include <stdint.h>
#include <stdlib.h>
#include <chopstx.h>

struct PORT {
  volatile uint32_t CFG0;	/* Port 0-7 */
  volatile uint32_t CFG1;	/* Port 8-15 */
  volatile uint32_t CFG2;	/* Port 16-23 */
  volatile uint32_t CFG3;	/* Port 24-31 */
  volatile uint32_t DATA;
  volatile uint32_t LEVEL0;
  volatile uint32_t LEVEL1;
  volatile uint32_t PULL0;
  volatile uint32_t PULL1;
};

static struct PORT *const PORTA = (struct PORT *const)0x01c20800;
static struct PORT *const PORTL = (struct PORT *const)0x01f02c00;

void
set_led (uint32_t on_off)
{
  uint32_t v = PORTL->DATA;

  if (on_off)
    v |= (1 << 10);
  else
    v &= ~(1 << 10);
  PORTL->DATA = v;
}

void
set_led_red (uint32_t on_off)
{
  uint32_t v = PORTA->DATA;

  if (on_off)
    v |= (1 << 15);
  else
    v &= ~(1 << 15);
  PORTA->DATA = v;
}


static chopstx_mutex_t mtx;
static chopstx_cond_t cnd0;
static chopstx_cond_t cnd1;

static uint8_t u, v;
static uint8_t m;		/* 0..100 */

static void
wait_for (uint32_t usec)
{
  chopstx_usec_wait (usec);
}

static void *
pwm (void *arg)
{
  (void)arg;

  chopstx_mutex_lock (&mtx);
  chopstx_cond_wait (&cnd0, &mtx);
  chopstx_mutex_unlock (&mtx);

  while (1)
    {
      set_led (u&v);
      wait_for (m);
      set_led (0);
      wait_for (100-m);
    }

  return NULL;
}

static void *
blk (void *arg)
{
  (void)arg;

  chopstx_mutex_lock (&mtx);
  chopstx_cond_wait (&cnd1, &mtx);
  chopstx_mutex_unlock (&mtx);

  while (1)
    {
      v = 0;
      wait_for (200*1000);
      v = 1;
      wait_for (200*1000);
    }

  return NULL;
}

#define PRIO_PWM 3
#define PRIO_BLK 2

extern uint8_t __process1_stack_base__, __process1_stack_size__;
extern uint8_t __process2_stack_base__, __process2_stack_size__;

const uint32_t __stackaddr_pwm = (uint32_t)&__process1_stack_base__;
const size_t __stacksize_pwm = (size_t)&__process1_stack_size__;

const uint32_t __stackaddr_blk = (uint32_t)&__process2_stack_base__;
const size_t __stacksize_blk = (size_t)&__process2_stack_size__;


int
main (int argc, const char *argv[])
{
  (void)argc;
  (void)argv;

  set_led (0);
  set_led_red (0);

  chopstx_mutex_init (&mtx);
  chopstx_cond_init (&cnd0);
  chopstx_cond_init (&cnd1);

  m = 10;

  chopstx_create (PRIO_PWM, __stackaddr_pwm, __stacksize_pwm, pwm, NULL);
  chopstx_create (PRIO_BLK, __stackaddr_blk, __stacksize_blk, blk, NULL);

  chopstx_usec_wait (200*1000);

  chopstx_mutex_lock (&mtx);
  chopstx_cond_signal (&cnd0);
  chopstx_cond_signal (&cnd1);
  chopstx_mutex_unlock (&mtx);

  while (1)
    {
      u ^= 1;

      set_led_red (1);
      wait_for (100*1000*6);
      set_led_red (0);
      wait_for (100*1000*6);
    }

  return 0;
}
