MEMORY
{
  /* https://www.st.com/resource/en/reference_manual/dm00355726.pdf */
  FLASH : ORIGIN = 0x08000000, LENGTH = 512K
  RAM   : ORIGIN = 0x20000000, LENGTH = 96K
  CCRAM : ORIGIN = 0x10000000, LENGTH = 16K
}

_stack_start = ORIGIN(CCRAM) + LENGTH(CCRAM);
