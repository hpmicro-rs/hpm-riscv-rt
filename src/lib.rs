//! HPMicro RISC-V Runtime
//!
//! This crate provides complete runtime support for HPMicro RISC-V MCUs,
//! with HPMicro-specific customizations for PLIC vectored interrupt mode.
//!
//! ## Usage
//!
//! Add this crate as a dependency:
//!
//! ```toml
//! [dependencies]
//! hpm-riscv-rt = "0.2"
//! ```
//!
//! Configure linker scripts in `.cargo/config.toml`:
//!
//! ```toml
//! rustflags = [
//!     "-C", "link-arg=-Tmemory.x",   # User-provided memory layout
//!     "-C", "link-arg=-Tdevice.x",   # From hpm-metapac (__INTERRUPTS)
//!     "-C", "link-arg=-Tlink.x",     # From hpm-riscv-rt
//! ]
//! ```
//!
//! Use the provided macros:
//!
//! ```ignore
//! use hpm_riscv_rt::{entry, pre_init, fast};
//!
//! #[pre_init]
//! unsafe fn before_main() {
//!     // Called before RAM is initialized
//! }
//!
//! #[entry]
//! fn main() -> ! {
//!     loop {}
//! }
//!
//! #[fast]
//! fn critical_function() {
//!     // Runs from ILM for better performance
//! }
//! ```

#![no_std]

mod asm;
pub mod trap;

use andes_riscv::{
    plic::{Plic, PlicExt},
    register,
};
use riscv::register::{
    mcounteren, mie, mstatus,
    mtvec::{self, Mtvec, TrapMode},
};

// Re-export macros
pub use hpm_riscv_rt_macros::{entry, external_interrupt, fast, pre_init};

/// HPMicro PLIC base address (same for all series)
const PLIC_BASE: usize = 0xE400_0000;
#[cfg(feature = "diagnostics")]
const PLIC_CLAIM_OFFSET: usize = 0x0020_0004;

#[cfg(any(feature = "hpm67-fix", feature = "pma-noncacheable"))]
const PMP_RWX_NAPOT: u32 = 0x1F;
#[cfg(any(feature = "hpm67-fix", feature = "pma-noncacheable"))]
const PMA_NAPOT_MEM_NC_BUF: u32 = 0x0F;

// ============ TrapFrame ============

/// Registers saved during a trap.
///
/// This struct mirrors the integer context saved by the CORE_LOCAL trap entry.
#[repr(C)]
pub struct TrapFrame {
    /// Return address
    pub ra: usize,
    /// Temporary register t0
    pub t0: usize,
    /// Temporary register t1
    pub t1: usize,
    /// Temporary register t2
    pub t2: usize,
    /// Temporary register t3
    pub t3: usize,
    /// Temporary register t4
    pub t4: usize,
    /// Temporary register t5
    pub t5: usize,
    /// Temporary register t6
    pub t6: usize,
    /// Argument/return register a0
    pub a0: usize,
    /// Argument register a1
    pub a1: usize,
    /// Argument register a2
    pub a2: usize,
    /// Argument register a3
    pub a3: usize,
    /// Argument register a4
    pub a4: usize,
    /// Argument register a5
    pub a5: usize,
    /// Argument register a6
    pub a6: usize,
    /// Argument register a7
    pub a7: usize,
    /// Global pointer
    pub gp: usize,
    /// Thread pointer
    pub tp: usize,
    /// Saved register s0/frame pointer
    pub s0: usize,
    /// Saved register s1
    pub s1: usize,
    /// Saved register s2
    pub s2: usize,
    /// Saved register s3
    pub s3: usize,
    /// Saved register s4
    pub s4: usize,
    /// Saved register s5
    pub s5: usize,
    /// Saved register s6
    pub s6: usize,
    /// Saved register s7
    pub s7: usize,
    /// Saved register s8
    pub s8: usize,
    /// Saved register s9
    pub s9: usize,
    /// Saved register s10
    pub s10: usize,
    /// Saved register s11
    pub s11: usize,
}

// ============ Rust Startup Code ============

/// Rust startup function called from assembly after RAM is initialized.
///
/// This function:
/// 1. Enables FPU
/// 2. Enables L1 Cache
/// 3. Initializes non-cacheable sections
/// 4. Sets up interrupts (PLIC vectored mode)
/// 5. Calls `main`
///
/// # Safety
///
/// This function must run exactly once from the reset entry after the linker
/// symbols and machine-mode execution environment have been established.
#[no_mangle]
pub unsafe extern "C" fn _hpm_start_rust() -> ! {
    extern "Rust" {
        fn main() -> !;
    }

    extern "C" {
        fn _setup_interrupts();
    }

    #[cfg(feature = "diagnostics")]
    clear_trap_record();

    // 1. Enable FPU (all HPMicro MCUs have FPU)
    mstatus::set_fs(mstatus::FS::Initial);

    // 2. Enable L1 Cache
    andes_riscv::l1c::ic_enable();
    // A flash algorithm can leave stale XPI-backed instruction-cache state.
    // HPM's fence.i operation invalidates the full L1I before execution
    // continues from the freshly programmed image.
    core::arch::asm!("fence.i", options(nostack));
    andes_riscv::l1c::dc_enable();
    andes_riscv::l1c::dc_invalidate_all();

    // 2.5. Configure PMA entries for non-cacheable regions
    // HPM67xx needs both RTT fix and noncacheable region configured together
    #[cfg(all(feature = "hpm67-fix", feature = "pma-noncacheable"))]
    configure_pma_hpm67();

    // Only RTT fix (no noncacheable region)
    #[cfg(all(feature = "hpm67-fix", not(feature = "pma-noncacheable")))]
    configure_rtt_noncacheable();

    // Only noncacheable region (non-HPM67 chips)
    #[cfg(all(feature = "pma-noncacheable", not(feature = "hpm67-fix")))]
    configure_noncacheable_pma();
    // 3. Initialize non-cacheable sections
    init_noncacheable_sections();

    // 4. Setup interrupts (PLIC vectored mode)
    _setup_interrupts();

    // 5. Jump to main
    main()
}

#[cfg(feature = "diagnostics")]
unsafe fn clear_trap_record() {
    extern "C" {
        static mut __hpm_preinit_trap_record: u32;
    }

    let base = core::ptr::addr_of_mut!(__hpm_preinit_trap_record);
    for offset in 0..8 {
        base.add(offset).write_volatile(0);
    }
}

/// Initialize non-cacheable data and bss sections.
#[inline(always)]
unsafe fn init_noncacheable_sections() {
    extern "C" {
        static mut __noncacheable_data_start__: u32;
        static mut __noncacheable_data_end__: u32;
        static __noncacheable_data_load_addr__: u32;
        static mut __noncacheable_bss_start__: u32;
        static mut __noncacheable_bss_end__: u32;
    }

    // Copy .noncacheable.data
    let mut src = core::ptr::addr_of!(__noncacheable_data_load_addr__);
    let mut dst = core::ptr::addr_of_mut!(__noncacheable_data_start__);
    let end = core::ptr::addr_of!(__noncacheable_data_end__);
    while (dst as *const u32) < end {
        dst.write_volatile(src.read_volatile());
        src = src.add(1);
        dst = dst.add(1);
    }

    // Zero .noncacheable.bss
    let mut dst = core::ptr::addr_of_mut!(__noncacheable_bss_start__);
    let end = core::ptr::addr_of!(__noncacheable_bss_end__);
    while (dst as *const u32) < end {
        dst.write_volatile(0);
        dst = dst.add(1);
    }
}

/// Configure PMA for HPM67xx: both RTT fix and noncacheable region in one call.
///
/// This avoids potential issues with separate pmacfg0 modifications.
/// - Entry 0: RTT control block (4KB)
/// - Entry 1: REGION_NONCACHEABLE_RAM
#[cfg(all(feature = "hpm67-fix", feature = "pma-noncacheable"))]
unsafe fn configure_pma_hpm67() {
    // RTT symbol (weak, 0 if not linked)
    extern "C" {
        #[link_name = "_SEGGER_RTT"]
        static SEGGER_RTT: u8;
        static __noncacheable_start__: u32;
        static __noncacheable_end__: u32;
    }

    let rtt_addr = core::ptr::addr_of!(SEGGER_RTT) as u32;
    let nc_start = core::ptr::addr_of!(__noncacheable_start__) as u32;
    let nc_end = core::ptr::addr_of!(__noncacheable_end__) as u32;

    let mut pmacfg0_val: u32 = 0;
    let mut pmpcfg0_val: u32 = PMP_RWX_NAPOT;

    set_pmp_addr0(0xFFFF_FFFF);

    // Entry 1: RTT (4KB)
    if rtt_addr != 0 {
        let aligned_addr = rtt_addr & !0xFFF;
        let size = 0x1000u32; // 4KB
        let napot_addr = (aligned_addr + (size >> 1) - 1) >> 2;
        set_pmp_addr1(napot_addr);
        register::pmaaddr1::write(napot_addr as usize);
        pmpcfg0_val |= PMP_RWX_NAPOT << 8;
        pmacfg0_val |= PMA_NAPOT_MEM_NC_BUF << 8;
    }

    // Entry 2: Noncacheable region
    if nc_end > nc_start {
        let length = nc_end - nc_start;
        let napot_addr = (nc_start + (length >> 1) - 1) >> 2;
        set_pmp_addr2(napot_addr);
        register::pmaaddr2::write(napot_addr as usize);
        pmpcfg0_val |= PMP_RWX_NAPOT << 16;
        pmacfg0_val |= PMA_NAPOT_MEM_NC_BUF << 16;
    }

    write_pmpcfg0(pmpcfg0_val);
    write_pmacfg0(pmacfg0_val);

    // Fence to ensure PMA takes effect
    core::arch::asm!("fence.i");
}

/// Configure PMA to make RTT control block non-cacheable (HPM67xx D-cache fix).
///
/// This function detects if defmt-rtt is linked by checking if `_SEGGER_RTT` symbol
/// exists (linker provides 0 if not defined). If linked, it configures PMA entry 0
/// to make that region non-cacheable, solving the D-cache coherency issue with probe-rs RTT.
///
/// PMA configuration:
/// - Entry type: NAPOT (Naturally Aligned Power Of Two)
/// - Memory type: Non-cacheable, Bufferable
/// - Region size: 4KB (aligned down from _SEGGER_RTT address)
#[cfg(all(feature = "hpm67-fix", not(feature = "pma-noncacheable")))]
unsafe fn configure_rtt_noncacheable() {
    // Weak symbol - will be null/zero if defmt-rtt is not linked
    extern "C" {
        #[link_name = "_SEGGER_RTT"]
        static SEGGER_RTT: u8;
    }

    // Get the address of _SEGGER_RTT
    let rtt_addr = core::ptr::addr_of!(SEGGER_RTT) as u32;

    // Skip if symbol doesn't exist (address would be 0 or invalid)
    if rtt_addr == 0 {
        return;
    }

    // Align down to 4KB boundary (PMA NAPOT minimum practical granularity)
    let aligned_addr = rtt_addr & !0xFFF;
    let size = 0x1000u32; // 4KB

    // NAPOT address format: (base + size/2 - 1) >> 2
    let napot_addr = (aligned_addr + (size >> 1) - 1) >> 2;

    // Configure PMA entry 1 to make RTT region non-cacheable.
    set_pmp_addr0(0xFFFF_FFFF);
    set_pmp_addr1(napot_addr);
    register::pmaaddr1::write(napot_addr as usize);
    write_pmpcfg0(PMP_RWX_NAPOT | (PMP_RWX_NAPOT << 8));
    write_pmacfg0(PMA_NAPOT_MEM_NC_BUF << 8);

    // Fence to ensure PMA takes effect
    core::arch::asm!("fence.i");
}

/// Configure PMA to make REGION_NONCACHEABLE_RAM actually non-cacheable.
///
/// This function reads the linker-provided `__noncacheable_start__` and `__noncacheable_end__`
/// symbols and configures PMA entry 1 to make that region non-cacheable.
///
/// Required for HPM5E/62/63 series. For HPM67xx, use configure_pma_hpm67() instead.
#[cfg(all(feature = "pma-noncacheable", not(feature = "hpm67-fix")))]
unsafe fn configure_noncacheable_pma() {
    extern "C" {
        static __noncacheable_start__: u32;
        static __noncacheable_end__: u32;
    }

    let start = core::ptr::addr_of!(__noncacheable_start__) as u32;
    let end = core::ptr::addr_of!(__noncacheable_end__) as u32;

    // Skip if noncacheable region is empty (e.g., HPM5300)
    if end <= start {
        return;
    }

    let length = end - start;

    // Verify alignment requirements (must be power of 2 aligned)
    debug_assert!(
        (length & (length - 1)) == 0,
        "noncacheable region length must be power of 2"
    );
    debug_assert!(
        (start & (length - 1)) == 0,
        "noncacheable region start must be aligned to its size"
    );

    // NAPOT address format: (base + size/2 - 1) >> 2
    let napot_addr = (start + (length >> 1) - 1) >> 2;

    // Match the HPM SDK board_init_pmp() layout:
    // Entry 0 grants RWX over the full physical address space.
    // Entry 1 makes REGION_NONCACHEABLE_RAM uncached for DMA descriptors/buffers.
    set_pmp_addr0(0xFFFF_FFFF);
    set_pmp_addr1(napot_addr);
    register::pmaaddr1::write(napot_addr as usize);
    write_pmpcfg0(PMP_RWX_NAPOT | (PMP_RWX_NAPOT << 8));
    write_pmacfg0(PMA_NAPOT_MEM_NC_BUF << 8);

    // Fence to ensure PMA takes effect
    core::arch::asm!("fence.i");
}

#[inline(always)]
#[cfg(any(feature = "hpm67-fix", feature = "pma-noncacheable"))]
unsafe fn set_pmp_addr0(value: u32) {
    core::arch::asm!("csrw pmpaddr0, {0}", in(reg) value, options(nomem, nostack));
}

#[inline(always)]
#[cfg(any(feature = "hpm67-fix", feature = "pma-noncacheable"))]
unsafe fn set_pmp_addr1(value: u32) {
    core::arch::asm!("csrw pmpaddr1, {0}", in(reg) value, options(nomem, nostack));
}

#[inline(always)]
#[cfg(all(feature = "hpm67-fix", feature = "pma-noncacheable"))]
unsafe fn set_pmp_addr2(value: u32) {
    core::arch::asm!("csrw pmpaddr2, {0}", in(reg) value, options(nomem, nostack));
}

#[inline(always)]
#[cfg(any(feature = "hpm67-fix", feature = "pma-noncacheable"))]
unsafe fn write_pmpcfg0(value: u32) {
    core::arch::asm!("csrw pmpcfg0, {0}", in(reg) value, options(nomem, nostack));
}

#[inline(always)]
#[cfg(any(feature = "hpm67-fix", feature = "pma-noncacheable"))]
unsafe fn write_pmacfg0(value: u32) {
    core::arch::asm!("csrw 0xBC0, {0}", in(reg) value, options(nomem, nostack));
}

// ============ Interrupt Setup ============

/// Setup interrupts for HPMicro MCUs.
///
/// This function:
/// 1. Cleans up PLIC state
/// 2. Enables mcycle counter
/// 3. Configures mtvec to point to the vector table
/// 4. Enables PLIC vectored mode via MMISC_CTL
/// 5. Enables global interrupts
///
/// # Safety
///
/// The caller must have exclusive access to the machine interrupt CSRs and
/// PLIC target context while this function resets and configures them.
#[export_name = "_setup_interrupts"]
pub unsafe fn setup_interrupts() {
    extern "C" {
        // Vector table generated by hpm-metapac
        // Entry 0: CORE_LOCAL (exceptions and core interrupts)
        // Entry 1+: PLIC external interrupt handlers
        static __INTERRUPTS: u32;
    }

    let plic = Plic::from_ptr(PLIC_BASE as *mut ());

    // 1. Clean up PLIC state. CLAIM/COMPLETE is side-effectful, so each
    // completion uses a direct write and never a read-modify-write.
    plic.set_threshold(0);
    for i in 0..128 {
        plic.targetconfig(0)
            .claim()
            .write(|w| w.set_interrupt_id(i as u16));
    }
    // Clear all interrupt enables
    for i in 0..4 {
        plic.targetint(0).inten(i).write(|w| w.0 = 0);
    }

    // 2. Enable mcycle counter
    mcounteren::set_cy();

    // 3. Set vector table address. TrapMode is ignored when
    // MMISC_CTL.VEC_PLIC is enabled.
    let vector_addr = core::ptr::addr_of!(__INTERRUPTS) as usize;
    mtvec::write(Mtvec::new(vector_addr, TrapMode::Direct));

    // 4. Enable PLIC vectored mode (Andes-specific)
    plic.feature().modify(|w| w.set_vectored(true));
    register::mmisc_ctl::set_vec_plic();

    // 5. Enable global interrupts
    mstatus::set_mie();
    mstatus::set_sie();
    mie::set_mext();
}

// ============ Default Handlers ============

/// Default exception handler - loops forever.
#[no_mangle]
#[allow(non_snake_case)]
pub extern "C" fn DefaultExceptionHandler(trap_frame: &TrapFrame) -> ! {
    #[cfg(feature = "diagnostics")]
    unsafe {
        record_default_exception(trap_frame as *const TrapFrame as usize);
    }

    #[cfg(not(feature = "diagnostics"))]
    let _ = trap_frame;

    loop {
        core::hint::spin_loop();
    }
}

/// Default interrupt handler - loops forever.
#[no_mangle]
#[allow(non_snake_case)]
pub extern "C" fn DefaultInterruptHandler() {
    #[cfg(feature = "diagnostics")]
    unsafe {
        record_default_interrupt();
    }

    loop {
        core::hint::spin_loop();
    }
}

#[inline(always)]
#[cfg(feature = "diagnostics")]
unsafe fn record_default_interrupt() {
    extern "C" {
        static mut __hpm_preinit_trap_record: u32;
    }

    let base = core::ptr::addr_of_mut!(__hpm_preinit_trap_record);
    let claim_addr = (PLIC_BASE + PLIC_CLAIM_OFFSET) as *const u32;
    core::arch::asm!(
        "lw t0, 0({claim_addr})",
        "csrr t1, mtvec",
        "csrr t2, mstatus",
        "csrr t3, mie",
        "csrr t4, mip",
        "sw {magic}, 0({base})",
        "sw t0, 4({base})",
        "sw t1, 8({base})",
        "sw t2, 12({base})",
        "sw t3, 16({base})",
        "sw t4, 20({base})",
        "sw sp, 24({base})",
        base = in(reg) base,
        claim_addr = in(reg) claim_addr,
        magic = in(reg) 0x4850_4d49u32,
        out("t0") _,
        out("t1") _,
        out("t2") _,
        out("t3") _,
        out("t4") _,
        options(nostack),
    );
}

#[inline(always)]
#[cfg(feature = "diagnostics")]
unsafe fn record_default_exception(trap_frame: usize) {
    extern "C" {
        static mut __hpm_preinit_trap_record: u32;
    }

    let base = core::ptr::addr_of_mut!(__hpm_preinit_trap_record);
    core::arch::asm!(
        "csrr t0, mcause",
        "csrr t1, mepc",
        "csrr t2, mtval",
        "csrr t3, mtvec",
        "csrr t4, mstatus",
        "sw {magic}, 0({base})",
        "sw t0, 4({base})",
        "sw t1, 8({base})",
        "sw t2, 12({base})",
        "sw {trap_frame}, 16({base})",
        "sw sp, 20({base})",
        "sw t3, 24({base})",
        "sw t4, 28({base})",
        base = in(reg) base,
        magic = in(reg) 0x4850_4d45u32,
        trap_frame = in(reg) trap_frame,
        out("t0") _,
        out("t1") _,
        out("t2") _,
        out("t3") _,
        out("t4") _,
        options(nostack),
    );
}
