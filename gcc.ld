
/* Linker script to configure memory regions. */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 512k
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 128k
}

ENTRY(Reset_Handler)

SECTIONS
{
  .isr_vector :
  {
  	. = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > FLASH

	.text :
	{
  	. = ALIGN(4);
		*(.text*)
		*(.rodata*)
		KEEP(*(.init))
		KEEP(*(.fini))
		/* Exception handling (required by GCC) */
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    *(.ARM.exidx*)
		. = ALIGN(4);
		_etext = .;
	} > FLASH

	.data : AT (_etext)
	{
		. = ALIGN(4);
		_sdata = .;                  /* start of .data in RAM */
		_sidata = LOADADDR(.data);   /* start of .data in FLASH */
    *(.data*)
    . = ALIGN(4);
    _edata = .;                  /* end of .data in RAM */
	} > RAM

	.bss :
	{
	  . = ALIGN(4);
		_sbss = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
	} > RAM

	.heap :
	{
	  . = ALIGN(8);
	  _heap_start = .;
	  /* Newlib expects 'end' symbol for sbrk */
    end = .;   /* start of heap */
	  . = . + 32k;
	  . = ALIGN(8);
	  _heap_end = .;
	} > RAM

	.stack :
	{
	  . = ALIGN(8);
	  _sstack = .;
	  . = ALIGN(8);
	  . = ORIGIN(RAM) + LENGTH(RAM);
	  _estack = .;
	} > RAM
	PROVIDE(__stack = _estack);
}

