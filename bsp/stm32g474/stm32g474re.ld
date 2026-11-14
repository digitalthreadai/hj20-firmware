/*
 * Linker script for the STM32G474RE (512 KB flash, 96 KB SRAM1+SRAM2,
 * 32 KB CCM SRAM) as used on the HJ-20 controller board.
 *
 * Copyright (c) 2026 Helix Robotics (fictional demonstration company, DigitalEng.ai).
 * All rights reserved.
 */

ENTRY(Reset_Handler)

MEMORY
{
    FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
    RAM    (rwx) : ORIGIN = 0x20000000, LENGTH = 96K   /* SRAM1 80K + SRAM2 16K, contiguous */
    CCMRAM (rwx) : ORIGIN = 0x10000000, LENGTH = 32K   /* zero-wait, I+D bus: control state  */
}

_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Stack_Size = 0x1000;   /* 4 KB main stack (ISRs nest on MSP) */

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
        *(.text .text.*)
        *(.rodata .rodata.*)
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > FLASH

    /* Initialised data: stored in flash, copied to RAM by Reset_Handler. */
    _sidata = LOADADDR(.data);
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data .data.*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    /* Zero-initialised hot state for the 20 kHz loop (HJ20_CCM attribute). */
    .ccmbss (NOLOAD) :
    {
        . = ALIGN(4);
        _sccmbss = .;
        *(.ccmbss .ccmbss.*)
        . = ALIGN(4);
        _eccmbss = .;
    } > CCMRAM

    /* Fails the link if static data leaves less than the minimum stack. */
    ._stack_reserve (NOLOAD) :
    {
        . = ALIGN(8);
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } > RAM

    /DISCARD/ :
    {
        *(.comment)
        *(.note*)
    }
}
