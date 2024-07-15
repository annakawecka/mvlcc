#include <mvlcc_readout_config.h>
#include <mvlcc_wrap.h>
#include <stdio.h>

int main(int argc, char *argv[]){
  mvlcc_t mvlc;
  int ec;
  int i;
  int tell = 1;

  uint32_t vmeBase;
  uint16_t regAddr;
  uint32_t readValue;


  if (argc <= 1)
    {
      fprintf(stderr, "Device not specified.\n");
      return 1;
    }

  mvlc = mvlcc_make_mvlc(argv[1]);

  if (ec = mvlcc_connect(mvlc))
    {
      fprintf(stderr, "Could not connect.\n");
      return 1;
    }
  
  vmeBase = 0x21000000u;
  regAddr = 0x6008u;
  readValue = 0u;

  for (i = 0; i < 100000; i++)
    {

      ec = mvlcc_single_vme_read(mvlc, vmeBase + regAddr, &readValue, 32, 16);

      if (ec)
	{
	  fprintf(stderr, "Could not read @ 0x%08x.\n", vmeBase + regAddr);
	  return 1;
	}

      regAddr = 0x6004u;
  
      ec = mvlcc_single_vme_write(mvlc, vmeBase + regAddr, 9, 32, 16);

      if (ec)
	{
	  fprintf(stderr, "Could not write @ 0x%08x.\n", vmeBase + regAddr);
	  return 1;
	}

      ec = mvlcc_single_vme_read(mvlc, vmeBase + regAddr, &readValue, 32, 16);

      if (ec)
	{
	  fprintf(stderr, "Could not read @ 0x%08x.\n", vmeBase + regAddr);
	  return 1;
	}

      if (i == tell)
	{
	  printf ("%d  ", i);
	  fflush(stdout);
	  if (tell < 5000)
	    tell = tell * 2;
	  else
	    tell += 5000;
	}
    }
  

  mvlcc_disconnect(mvlc);

  mvlcc_free_mvlc(mvlc);

  return 0;
}
