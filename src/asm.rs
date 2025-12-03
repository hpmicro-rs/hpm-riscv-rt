//! Assembly entry point and startup code for HPMicro RISC-V MCUs.
//!
//! This module provides the `_start` entry point that:
//! 1. Initializes global pointer and stack pointer
//! 2. Calls `__pre_init` hook
//! 3. Initializes .data and .bss sections
//! 4. Calls `_setup_interrupts`
//! 5. Jumps to `main`

use core::arch::global_asm;

// Entry point of all programs (_start)
// Initializes stack pointer, global pointer, then calls _start_rust
global_asm!(
    r#"
    .section .init, "ax"
    .global _hpm_start
    .type _hpm_start, @function

_hpm_start:
    /* Initialize global pointer */
    .option push
    .option norelax
    la gp, __global_pointer$
    .option pop

    /* Initialize stack pointer */
    la sp, _sstack

    /* Set pre-init trap handler (simple infinite loop) */
    la t0, _pre_init_trap
    csrw mtvec, t0

    /* Clear mstatus */
    csrw mstatus, zero

    /* Disable interrupts */
    csrw mie, zero

    /* Call pre-init hook (before RAM is initialized) */
    call __pre_init

    /* Initialize .data section */
    la a0, _sdata
    la a1, _edata
    la a2, _sidata
    bgeu a0, a1, 2f
1:
    lw t0, 0(a2)
    sw t0, 0(a0)
    addi a0, a0, 4
    addi a2, a2, 4
    bltu a0, a1, 1b
2:

    /* Initialize .bss section */
    la a0, _sbss
    la a1, _ebss
    bgeu a0, a1, 4f
3:
    sw zero, 0(a0)
    addi a0, a0, 4
    bltu a0, a1, 3b
4:

    /* Initialize .fast section (ILM) */
    la a0, _sfast
    la a1, _efast
    la a2, _sifast
    bgeu a0, a1, 6f
5:
    lw t0, 0(a2)
    sw t0, 0(a0)
    addi a0, a0, 4
    addi a2, a2, 4
    bltu a0, a1, 5b
6:

    /* Initialize .fast.data section (DLM) */
    la a0, __fast_data_start__
    la a1, __fast_data_end__
    la a2, __fast_data_load_addr__
    bgeu a0, a1, 8f
7:
    lw t0, 0(a2)
    sw t0, 0(a0)
    addi a0, a0, 4
    addi a2, a2, 4
    bltu a0, a1, 7b
8:

    /* Initialize .fast.bss section */
    la a0, __fast_bss_start__
    la a1, __fast_bss_end__
    bgeu a0, a1, 10f
9:
    sw zero, 0(a0)
    addi a0, a0, 4
    bltu a0, a1, 9b
10:

    /* Call Rust startup code */
    call _hpm_start_rust

    /* Should not return, but if it does, loop forever */
    j _pre_init_trap

    .size _hpm_start, . - _hpm_start
"#
);

// Pre-init trap handler - simple infinite loop
// Used during early boot before the real trap handler is set up
global_asm!(
    r#"
    .section .init, "ax"
    .global _pre_init_trap
    .type _pre_init_trap, @function
    .balign 4

_pre_init_trap:
    j _pre_init_trap

    .size _pre_init_trap, . - _pre_init_trap
"#
);

// Default pre-init function (does nothing)
global_asm!(
    r#"
    .section .init, "ax"
    .weak default_pre_init
    .type default_pre_init, @function

default_pre_init:
    ret

    .size default_pre_init, . - default_pre_init
"#
);

// Default mp_hook (single-hart: always returns true)
global_asm!(
    r#"
    .section .init, "ax"
    .weak default_mp_hook
    .type default_mp_hook, @function

default_mp_hook:
    li a0, 1
    ret

    .size default_mp_hook, . - default_mp_hook
"#
);

// Default setup_interrupts (does nothing, real implementation in lib.rs)
global_asm!(
    r#"
    .section .init, "ax"
    .weak default_setup_interrupts
    .type default_setup_interrupts, @function

default_setup_interrupts:
    ret

    .size default_setup_interrupts, . - default_setup_interrupts
"#
);

// Default start_trap (simple trap handler for early boot)
global_asm!(
    r#"
    .section .trap, "ax"
    .weak default_start_trap
    .type default_start_trap, @function
    .balign 4

default_start_trap:
    j default_start_trap

    .size default_start_trap, . - default_start_trap
"#
);

