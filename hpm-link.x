/* HPMicro RISC-V Link Script
 *
 * This is the main linker script for HPMicro RISC-V MCUs.
 * It handles:
 *   - Standard sections (.text, .rodata, .data, .bss)
 *   - Fast sections in ILM/DLM (.fast.text, .fast.data, .fast.bss)
 *   - Vector table placed in ILM (512-byte aligned for PLIC vectored mode)
 *   - Non-cacheable sections
 *
 * Required MEMORY regions (defined in memory.x):
 *   REGION_TEXT, REGION_RODATA, REGION_DATA, REGION_BSS
 *   REGION_HEAP, REGION_STACK
 *   REGION_FASTTEXT (ILM), REGION_FASTDATA (DLM)
 *
 * Optional regions:
 *   REGION_NONCACHEABLE_RAM, AHB_SRAM, REGION_CAN
 */

ENTRY(_hpm_start);

/* Stack configuration */
PROVIDE(_stack_size = 0x4000);
PROVIDE(_heap_size = 0);

/* Multi-hart configuration (single-hart by default) */
PROVIDE(_max_hart_id = 0);
PROVIDE(_hart_stack_size = 2K);

/* Text start address */
PROVIDE(_stext = ORIGIN(REGION_TEXT));
PROVIDE(_stack_start = ORIGIN(REGION_STACK) + LENGTH(REGION_STACK));

/* RTT support: provide 0 if defmt-rtt is not linked */
PROVIDE(_SEGGER_RTT = 0);

/* Non-cacheable region: provide 0 if not defined in memory.x */
PROVIDE(__noncacheable_start__ = 0);
PROVIDE(__noncacheable_end__ = 0);

/* ============ Exception Handlers ============ */
/* Default to ExceptionHandler if not defined */
PROVIDE(InstructionMisaligned = ExceptionHandler);
PROVIDE(InstructionFault = ExceptionHandler);
PROVIDE(IllegalInstruction = ExceptionHandler);
PROVIDE(Breakpoint = ExceptionHandler);
PROVIDE(LoadMisaligned = ExceptionHandler);
PROVIDE(LoadFault = ExceptionHandler);
PROVIDE(StoreMisaligned = ExceptionHandler);
PROVIDE(StoreFault = ExceptionHandler);
PROVIDE(UserEnvCall = ExceptionHandler);
PROVIDE(SupervisorEnvCall = ExceptionHandler);
PROVIDE(MachineEnvCall = ExceptionHandler);
PROVIDE(InstructionPageFault = ExceptionHandler);
PROVIDE(LoadPageFault = ExceptionHandler);
PROVIDE(StorePageFault = ExceptionHandler);

/* ============ Core Interrupt Handlers ============ */
/* Default to DefaultHandler if not defined */
PROVIDE(SupervisorSoft = DefaultHandler);
PROVIDE(MachineSoft = DefaultHandler);
PROVIDE(SupervisorTimer = DefaultHandler);
PROVIDE(MachineTimer = DefaultHandler);
PROVIDE(SupervisorExternal = DefaultHandler);
PROVIDE(MachineExternal = DefaultHandler);

/* ============ Default Handlers ============ */
PROVIDE(DefaultHandler = DefaultInterruptHandler);
PROVIDE(ExceptionHandler = DefaultExceptionHandler);

/* ============ riscv-rt Compatibility Symbols ============ */
/* abort function for riscv-rt */
PROVIDE(abort = DefaultExceptionHandler);
/* hal_main alias for main */
PROVIDE(hal_main = main);

/* ============ Startup Hooks ============ */
/* Pre-initialization function (called before RAM init, interrupts disabled) */
PROVIDE(__pre_init = default_pre_init);

/* Interrupt setup function (called after RAM init) */
PROVIDE(_setup_interrupts = default_setup_interrupts);

/* Multi-processor hook (returns true for primary hart) */
PROVIDE(_mp_hook = default_mp_hook);

/* Start trap handler (used during startup before vector table is ready) */
PROVIDE(_start_trap = default_start_trap);

/* ============ SECTIONS ============ */
SECTIONS
{
    /* Dummy section to make _stext work */
    .text.dummy (NOLOAD) :
    {
        . = ABSOLUTE(_stext);
    } > REGION_TEXT

    /* Code section */
    .text _stext :
    {
        /* Reset handler first */
        KEEP(*(.init));
        KEEP(*(.init.rust));
        . = ALIGN(4);

        /* Trap handlers (startup trap, not PLIC vector table) */
        *(.trap);
        *(.trap.rust);

        /* Abort handler */
        *(.text.abort);

        /* All other code */
        *(.text .text.*);

        . = ALIGN(4);
    } > REGION_TEXT

    /* Vector table and fast code - placed in ILM */
    .fast : ALIGN(512)
    {
        _sifast = LOADADDR(.fast);
        _sfast = .;

        /* Vector table must be 512-byte aligned for PLIC vectored mode */
        __vector_ram_start__ = .;
        /*
         * CAUTION: ILM starts at 0x00000000.
         * Using address 0 as an IRQ handler results in `None` when cast to `Option<fn()>`.
         * The __INTERRUPTS table from hpm-metapac handles this by using CORE_LOCAL at entry 0.
         */
        KEEP(*(.vector_table.interrupts));
        __vector_ram_end__ = .;
        . = ALIGN(8);

        /* Fast text section */
        __fast_text_start__ = .;
        *(.trap.rust);
        *(.fast .fast.* .fast.text .fast.text.*);
        . = ALIGN(4);
        __fast_text_end__ = .;

        _efast = .;
    } > REGION_FASTTEXT AT > REGION_RODATA

    __vector_load_addr__ = LOADADDR(.fast);
    __fast_text_load_addr__ = _sifast;

    /* Read-only data */
    .rodata : ALIGN(4)
    {
        *(.srodata .srodata.*);
        *(.rodata .rodata.*);
        . = ALIGN(4);
    } > REGION_RODATA

    /* Initialized data */
    .data : ALIGN(4)
    {
        _sidata = LOADADDR(.data);
        _sdata = .;
        __sdata = .;  /* riscv-rt compatibility */
        /* Global pointer for linker relaxations */
        PROVIDE(__global_pointer$ = . + 0x800);
        *(.sdata .sdata.* .sdata2 .sdata2.*);
        *(.data .data.*);
        . = ALIGN(4);
        _edata = .;
        __edata = .;  /* riscv-rt compatibility */
    } > REGION_DATA AT > REGION_RODATA

    __sidata = LOADADDR(.data);  /* riscv-rt compatibility */

    /* Uninitialized data */
    .bss (NOLOAD) : ALIGN(4)
    {
        _sbss = .;
        __sbss = .;  /* riscv-rt compatibility */
        *(.sbss .sbss.* .bss .bss.*);
        . = ALIGN(4);
        _ebss = .;
        __ebss = .;  /* riscv-rt compatibility */
    } > REGION_BSS

    /* Fast data section - placed in DLM */
    .fast.data : ALIGN(4)
    {
        __fast_data_start__ = .;
        *(.fast.data .fast.data.*);
        . = ALIGN(4);
        __fast_data_end__ = .;
    } > REGION_FASTDATA AT > REGION_RODATA

    __fast_data_load_addr__ = LOADADDR(.fast.data);

    /* Fast BSS section - placed in DLM */
    .fast.bss (NOLOAD) : ALIGN(4)
    {
        __fast_bss_start__ = .;
        *(.fast.bss .fast.bss.*);
        . = ALIGN(4);
        __fast_bss_end__ = .;
    } > REGION_FASTDATA

    /* Non-cacheable data (optional) */
    .noncacheable.data : ALIGN(8)
    {
        __noncacheable_data_start__ = .;
        KEEP(*(.noncacheable.data .noncacheable.data.*));
        . = ALIGN(8);
        __noncacheable_data_end__ = .;
    } > REGION_NONCACHEABLE_RAM AT > REGION_RODATA

    __noncacheable_data_load_addr__ = LOADADDR(.noncacheable.data);

    .noncacheable.bss (NOLOAD) : ALIGN(8)
    {
        __noncacheable_bss_start__ = .;
        KEEP(*(.noncacheable .noncacheable.*));
        KEEP(*(.noncacheable.bss .noncacheable.bss.*));
        . = ALIGN(8);
        __noncacheable_bss_end__ = .;
    } > REGION_NONCACHEABLE_RAM

    /* Heap */
    .heap (NOLOAD) :
    {
        _sheap = .;
        . += _heap_size;
        . = ALIGN(4);
        _eheap = .;
    } > REGION_HEAP

    /* Stack */
    .stack (NOLOAD) :
    {
        _estack = .;
        . = ABSOLUTE(_stack_start);
        _sstack = .;
    } > REGION_STACK

    /* AHB SRAM (optional) */
    .ahb_sram (NOLOAD) :
    {
        KEEP(*(.ahb_sram .ahb_sram.*));
    } > AHB_SRAM

    /* CAN message buffers (optional)
     * User should define REGION_CAN in memory.x if using CAN
     * Example: REGION_ALIAS("REGION_CAN", AHB_SRAM);
     */
    /* .can section removed - define in user memory.x if needed */

    /* GOT section - should be empty */
    .got (INFO) :
    {
        KEEP(*(.got .got.*));
    }

    /* Exception handling (for panic unwinding) */
    .eh_frame : { KEEP(*(.eh_frame)) } > REGION_TEXT
    .eh_frame_hdr : { *(.eh_frame_hdr) } > REGION_TEXT
}

/* ============ ASSERTIONS ============ */

ASSERT(ORIGIN(REGION_TEXT) % 4 == 0, "
ERROR(hpm-riscv-rt): REGION_TEXT must be 4-byte aligned");

ASSERT(ORIGIN(REGION_RODATA) % 4 == 0, "
ERROR(hpm-riscv-rt): REGION_RODATA must be 4-byte aligned");

ASSERT(ORIGIN(REGION_DATA) % 4 == 0, "
ERROR(hpm-riscv-rt): REGION_DATA must be 4-byte aligned");

ASSERT(ORIGIN(REGION_HEAP) % 4 == 0, "
ERROR(hpm-riscv-rt): REGION_HEAP must be 4-byte aligned");

ASSERT(ORIGIN(REGION_STACK) % 4 == 0, "
ERROR(hpm-riscv-rt): REGION_STACK must be 4-byte aligned");

ASSERT(_stext % 4 == 0, "
ERROR(hpm-riscv-rt): _stext must be 4-byte aligned");

ASSERT(_sdata % 4 == 0 && _edata % 4 == 0, "
BUG(hpm-riscv-rt): .data is not 4-byte aligned");

ASSERT(_sidata % 4 == 0, "
BUG(hpm-riscv-rt): LMA of .data is not 4-byte aligned");

ASSERT(_sbss % 4 == 0 && _ebss % 4 == 0, "
BUG(hpm-riscv-rt): .bss is not 4-byte aligned");

ASSERT(_sheap % 4 == 0, "
BUG(hpm-riscv-rt): start of .heap is not 4-byte aligned");

ASSERT(_stext + SIZEOF(.text) < ORIGIN(REGION_TEXT) + LENGTH(REGION_TEXT), "
ERROR(hpm-riscv-rt): .text section exceeds REGION_TEXT");

ASSERT(SIZEOF(.stack) > (_max_hart_id + 1) * _hart_stack_size, "
ERROR(hpm-riscv-rt): .stack too small for all harts.
Consider changing `_max_hart_id` or `_hart_stack_size`.");

ASSERT(SIZEOF(.got) == 0, "
ERROR(hpm-riscv-rt): .got section detected. Dynamic relocations not supported.
If linking C code via `cc` crate, compile without -fPIC flag.");
