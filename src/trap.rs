//! Trap handling for HPMicro RISC-V MCUs.
//!
//! This module provides the CORE_LOCAL handler (vector table entry 0)
//! for handling exceptions and core interrupts in PLIC vectored mode.
//!
//! In HPMicro's PLIC vectored mode:
//! - mtvec points to the vector table in ILM
//! - Entry 0 (CORE_LOCAL) handles exceptions and core interrupts
//! - Entries 1+ are direct jump targets for PLIC external interrupts

use core::arch::global_asm;

use riscv::register::mcause;

use crate::TrapFrame;

// ============ Exception Handlers ============

extern "C" {
    fn InstructionMisaligned(trap_frame: &TrapFrame);
    fn InstructionFault(trap_frame: &TrapFrame);
    fn IllegalInstruction(trap_frame: &TrapFrame);
    fn Breakpoint(trap_frame: &TrapFrame);
    fn LoadMisaligned(trap_frame: &TrapFrame);
    fn LoadFault(trap_frame: &TrapFrame);
    fn StoreMisaligned(trap_frame: &TrapFrame);
    fn StoreFault(trap_frame: &TrapFrame);
    fn UserEnvCall(trap_frame: &TrapFrame);
    fn SupervisorEnvCall(trap_frame: &TrapFrame);
    fn MachineEnvCall(trap_frame: &TrapFrame);
    fn InstructionPageFault(trap_frame: &TrapFrame);
    fn LoadPageFault(trap_frame: &TrapFrame);
    fn StorePageFault(trap_frame: &TrapFrame);
    fn ExceptionHandler(trap_frame: &TrapFrame);
}

/// Exception dispatch table.
#[doc(hidden)]
#[no_mangle]
pub static __HPM_EXCEPTIONS: [Option<unsafe extern "C" fn(&TrapFrame)>; 16] = [
    Some(InstructionMisaligned), // 0
    Some(InstructionFault),      // 1
    Some(IllegalInstruction),    // 2
    Some(Breakpoint),            // 3
    Some(LoadMisaligned),        // 4
    Some(LoadFault),             // 5
    Some(StoreMisaligned),       // 6
    Some(StoreFault),            // 7
    Some(UserEnvCall),           // 8
    Some(SupervisorEnvCall),     // 9
    None,                        // 10 (reserved)
    Some(MachineEnvCall),        // 11
    Some(InstructionPageFault),  // 12
    Some(LoadPageFault),         // 13
    None,                        // 14 (reserved)
    Some(StorePageFault),        // 15
];

// ============ Core Interrupt Handlers ============

extern "C" {
    fn SupervisorSoft();
    fn MachineSoft();
    fn SupervisorTimer();
    fn MachineTimer();
    fn SupervisorExternal();
    fn MachineExternal();
    fn DefaultHandler();
}

/// Core interrupt dispatch table.
#[doc(hidden)]
#[no_mangle]
pub static __HPM_CORE_INTERRUPTS: [Option<unsafe extern "C" fn()>; 14] = [
    None,                     // 0 (reserved)
    Some(SupervisorSoft),     // 1
    None,                     // 2 (reserved)
    Some(MachineSoft),        // 3 - PLICSW
    None,                     // 4 (reserved)
    Some(SupervisorTimer),    // 5
    None,                     // 6 (reserved)
    Some(MachineTimer),       // 7 - MCHTMR
    None,                     // 8 (reserved)
    Some(SupervisorExternal), // 9
    None,                     // 10 (reserved)
    Some(MachineExternal),    // 11
    None,                     // 12 (Coprocessor, reserved)
    None,                     // 13 (Host, reserved)
];

// ============ CORE_LOCAL Handler ============

/// Rust handler for CORE_LOCAL (vector table entry 0).
///
/// This function dispatches exceptions and core interrupts to their handlers.
#[no_mangle]
#[link_section = ".trap.rust"]
unsafe extern "C" fn _start_rust_CORE_LOCAL(trap_frame: *const TrapFrame) {
    let cause = mcause::read();
    let code = cause.code();

    // Debug: log every CORE_LOCAL invocation
    // defmt::trace!("CORE_LOCAL: is_exception={}, code={}", cause.is_exception(), code);

    if cause.is_exception() {
        // Some Andes cores can report a legal instruction as illegal with mtval=0.
        #[cfg(feature = "ignore-illegal-mtval-zero")]
        if code == 2 && riscv::register::mtval::read() == 0 {
            return;
        }

        let trap_frame = &*trap_frame;
        if let Some(Some(handler)) = __HPM_EXCEPTIONS.get(code) {
            handler(trap_frame);
        }
        // Always call ExceptionHandler for unhandled exceptions
        ExceptionHandler(trap_frame);
    } else if let Some(Some(handler)) = __HPM_CORE_INTERRUPTS.get(code) {
        handler();
    } else {
        DefaultHandler();
    }
}

// CORE_LOCAL assembly handler.
// Saves the interrupted integer context, calls Rust handler, restores registers.
global_asm!(
    r#"
    .section .trap.rust, "ax"
    .global CORE_LOCAL
    .type CORE_LOCAL, @function
    .balign 4

CORE_LOCAL:
    /* Save integer registers. Keep the stack 16-byte aligned for the Rust call. */
    addi sp, sp, -128
    sw ra, 0(sp)
    sw t0, 4(sp)
    sw t1, 8(sp)
    sw t2, 12(sp)
    sw t3, 16(sp)
    sw t4, 20(sp)
    sw t5, 24(sp)
    sw t6, 28(sp)
    sw a0, 32(sp)
    sw a1, 36(sp)
    sw a2, 40(sp)
    sw a3, 44(sp)
    sw a4, 48(sp)
    sw a5, 52(sp)
    sw a6, 56(sp)
    sw a7, 60(sp)
    sw gp, 64(sp)
    sw tp, 68(sp)
    sw s0, 72(sp)
    sw s1, 76(sp)
    sw s2, 80(sp)
    sw s3, 84(sp)
    sw s4, 88(sp)
    sw s5, 92(sp)
    sw s6, 96(sp)
    sw s7, 100(sp)
    sw s8, 104(sp)
    sw s9, 108(sp)
    sw s10, 112(sp)
    sw s11, 116(sp)

    /* Call Rust handler with trap frame pointer */
    mv a0, sp
    call _start_rust_CORE_LOCAL

    /* Restore integer registers */
    lw ra, 0(sp)
    lw t0, 4(sp)
    lw t1, 8(sp)
    lw t2, 12(sp)
    lw t3, 16(sp)
    lw t4, 20(sp)
    lw t5, 24(sp)
    lw t6, 28(sp)
    lw a0, 32(sp)
    lw a1, 36(sp)
    lw a2, 40(sp)
    lw a3, 44(sp)
    lw a4, 48(sp)
    lw a5, 52(sp)
    lw a6, 56(sp)
    lw a7, 60(sp)
    lw gp, 64(sp)
    lw tp, 68(sp)
    lw s0, 72(sp)
    lw s1, 76(sp)
    lw s2, 80(sp)
    lw s3, 84(sp)
    lw s4, 88(sp)
    lw s5, 92(sp)
    lw s6, 96(sp)
    lw s7, 100(sp)
    lw s8, 104(sp)
    lw s9, 108(sp)
    lw s10, 112(sp)
    lw s11, 116(sp)
    addi sp, sp, 128

    mret

    .size CORE_LOCAL, . - CORE_LOCAL
"#
);
