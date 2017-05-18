#include <stdint.h>
#include <stdlib.h>
#include <chopstx.h>

#include <unistd.h>
#include <stdio.h>

static void
set_led (int on)
{
#if 1
  if (on)
    write (1, "********\x08\x08\x08\x08\x08\x08\x08\x08", 16);
  else
    write (1, "        \x08\x08\x08\x08\x08\x08\x08\x08", 16);
#else
  if (on)
    puts ("!");
  else
    puts ("");
#endif
}


static chopstx_mutex_t mtx;
static chopstx_cond_t cnd0;
static chopstx_cond_t cnd1;

static uint8_t u, v;
static uint8_t m;		/* 0..100 */

static void
wait_for (uint32_t usec)
{
#if defined(BUSY_LOOP)
  uint32_t count = usec * 6;
  uint32_t i;

  for (i = 0; i < count; i++)
    asm volatile ("" : : "r" (i) : "memory");
#else
  chopstx_usec_wait (usec);
#endif
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

#if defined(BUSY_LOOP)
#define PRIO_PWM (CHOPSTX_SCHED_RR|1)
#define PRIO_BLK (CHOPSTX_SCHED_RR|1)
#else
#define PRIO_PWM 3
#define PRIO_BLK 2
#endif

static char __process1_stack_base__[4096];
static char __process2_stack_base__[4096];

#define STACK_ADDR_PWM ((uintptr_t)__process1_stack_base__)
#define STACK_SIZE_PWM (sizeof __process1_stack_base__)

#define STACK_ADDR_BLK ((uintptr_t)__process2_stack_base__)
#define STACK_SIZE_BLK (sizeof __process2_stack_base__)


void entry (void);

int
main (int argc, const char *argv[])
{
  (void)argc;
  (void)argv;
  entry ();
}

int
emu_main (int argc, const char *argv[])
{
  (void)argc;
  (void)argv;

  chopstx_mutex_init (&mtx);
  chopstx_cond_init (&cnd0);
  chopstx_cond_init (&cnd1);

  m = 10;

  chopstx_create (PRIO_PWM, STACK_ADDR_PWM, STACK_SIZE_PWM, pwm, NULL);
  chopstx_create (PRIO_BLK, STACK_ADDR_BLK, STACK_SIZE_BLK, blk, NULL);

  chopstx_usec_wait (200*1000);

  chopstx_mutex_lock (&mtx);
  chopstx_cond_signal (&cnd0);
  chopstx_cond_signal (&cnd1);
  chopstx_mutex_unlock (&mtx);

  while (1)
    {
      u ^= 1;
      wait_for (200*1000*6);
    }

  return 0;
}
