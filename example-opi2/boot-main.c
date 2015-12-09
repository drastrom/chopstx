#include <stdint.h>
#include <chx_hard_regs.h>
#include "io.h"

void
set_led_pwr (uint32_t on_off)
{
  uint32_t v = PORTL->DATA;

  if (on_off)
    v |= (1 << 10);
  else
    v &= ~(1 << 10);
  PORTL->DATA = v;
}

static void
set_led_sts (uint32_t on_off)
{
  uint32_t v = PORTA->DATA;

  if (on_off)
    v |= (1 << 15);
  else
    v &= ~(1 << 15);
  PORTA->DATA = v;
}

void
boot_main (void)
{
  while (1)
    {
      set_led_sts (1);
      delay (10000000);
      set_led_sts (0);
      delay (10000000);
    }
}
