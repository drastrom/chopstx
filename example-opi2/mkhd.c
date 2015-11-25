#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

struct header {
  uint32_t jump_inst;
  uint8_t magic[8];
  uint32_t chksum;
  uint32_t len;
};

int
calculate_chksum (struct header *p, uint32_t len)
{
  uint32_t sum = 0;
  uint32_t *buf = (uint32_t *)p;
  uint32_t i;

  if ((len & 3))
    return -1;

  p->chksum = 0x5f0a6c39;
  p->len = len;
  len >>= 2;

  for (i = 0; i < len; i++)
    sum += buf[i];

  p->chksum = sum;
  return 0;
}

#define BLOCK_MASK (1024-1)

int
main (int argc, char *argv[])
{
  struct header *buf;
  uint32_t len;
  FILE *f;

  if (argc != 3)
    {
      puts ("Usage: mkhd input-file output-file");
      exit (1);
    }
  
  if ((f = fopen (argv[1], "rb")) == NULL)
    {
      perror ("input");
      exit (1);
    }
  fseek (f, 0, SEEK_END);
  len = ftell (f);
  rewind (f);

  buf = calloc ((len + BLOCK_MASK)& ~BLOCK_MASK, 1);
  fread (buf, 1, len, f);
  fclose (f);
  len = (len + BLOCK_MASK) & ~BLOCK_MASK;

  if (calculate_chksum (buf, len) < 0)
    {
      free (buf);
      fputs ("unaligend\n", stderr);
      exit (1);
    }

  if ((f = fopen (argv[2], "wb")) == NULL)
    {
      free (buf);
      perror ("output");
      exit (1);
    }

  fwrite (buf, 1, len, f);
  fclose (f);
  free (buf);

  return 0;
}
