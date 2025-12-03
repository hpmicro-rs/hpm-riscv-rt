# hpm-riscv-rt

Runtime support for HPMicro RISC-V MCUs with Andes PLIC vectored interrupt mode.

## Features

- Complete startup code (`_hpm_start`) with RAM initialization
- PLIC vectored interrupt support (hardware-accelerated interrupt dispatch)
- L1 Cache initialization (I-Cache, D-Cache)
- Fast code/data sections for ILM/DLM placement
- Non-cacheable memory section support
- Compatible with `riscv-rt` memory.x format (with HPMicro extensions)

## Usage

### 1. Add dependency

```toml
[dependencies]
hpm-riscv-rt = "0.2"
```

### 2. Configure linker scripts

In `.cargo/config.toml`:

```toml
[target.riscv32imafc-unknown-none-elf]
rustflags = [
    "-C", "link-arg=-Tmemory.x",     # User-provided memory layout
    "-C", "link-arg=-Tdevice.x",     # From hpm-metapac (__INTERRUPTS)
    "-C", "link-arg=-Thpm-link.x",   # From hpm-riscv-rt
]
```

### 3. Create `memory.x`

```ld
MEMORY
{
    XPI0_APP    : ORIGIN = 0x80003000, LENGTH = 1024K - 0x3000
    ILM         : ORIGIN = 0x00000000, LENGTH = 128K
    DLM         : ORIGIN = 0x00080000, LENGTH = 128K
    AHB_SRAM    : ORIGIN = 0xF0400000, LENGTH = 32K
}

/* Standard regions (riscv-rt compatible) */
REGION_ALIAS("REGION_TEXT", XPI0_APP);
REGION_ALIAS("REGION_RODATA", XPI0_APP);
REGION_ALIAS("REGION_DATA", DLM);
REGION_ALIAS("REGION_BSS", DLM);
REGION_ALIAS("REGION_HEAP", DLM);
REGION_ALIAS("REGION_STACK", DLM);

/* HPMicro extensions */
REGION_ALIAS("REGION_FASTTEXT", ILM);      /* Vector table + fast code */
REGION_ALIAS("REGION_FASTDATA", DLM);      /* Fast data */
REGION_ALIAS("REGION_NONCACHEABLE_RAM", DLM);
```

### 4. Write your application

```rust
#![no_std]
#![no_main]

use hpm_riscv_rt::{entry, pre_init, fast};

#[entry]
fn main() -> ! {
    // Your application code
    loop {}
}
```

## Memory Regions

| Region | Required | Description |
|--------|----------|-------------|
| `REGION_TEXT` | Yes | Code (.text) |
| `REGION_RODATA` | Yes | Read-only data (.rodata) |
| `REGION_DATA` | Yes | Initialized data (.data) |
| `REGION_BSS` | Yes | Uninitialized data (.bss) |
| `REGION_HEAP` | Yes | Heap memory |
| `REGION_STACK` | Yes | Stack memory |
| `REGION_FASTTEXT` | Yes | ILM - Vector table + fast code |
| `REGION_FASTDATA` | Yes | DLM - Fast data |
| `REGION_NONCACHEABLE_RAM` | Yes | Non-cacheable memory |
| `AHB_SRAM` | Optional | AHB SRAM for DMA buffers |

## Macros

### `#[entry]`

Declares the program entry point. Function must have signature `fn() -> !`.

```rust
#[entry]
fn main() -> ! {
    loop {}
}
```

### `#[pre_init]`

Declares a function to run before RAM initialization. Useful for disabling watchdog or configuring external RAM.

```rust
#[pre_init]
unsafe fn setup() {
    // Runs before .data/.bss are initialized
    // Stack is valid, interrupts are disabled
}
```

### `#[fast]`

Places functions in ILM (.fast.text) or statics in DLM (.fast.data/.fast.bss) for better performance.

```rust
#[fast]
fn time_critical_function() {
    // Runs from ILM (zero wait state)
}

#[fast]
static mut BUFFER: [u8; 1024] = [0; 1024];  // In .fast.data (DLM)

#[fast]
static UNINIT: MaybeUninit<[u8; 4096]> = MaybeUninit::uninit();  // In .fast.bss
```

### `#[external_interrupt]`

Declares an external interrupt handler for PLIC.

```rust
use hpm_riscv_rt::external_interrupt;

#[external_interrupt(pac::interrupt::UART0)]
fn uart0_handler() {
    // Handle UART0 interrupt
}
```

## Interrupt Handling

HPMicro uses Andes PLIC vectored mode:

- **Vector table** at 512-byte aligned address in ILM
- **Entry 0** (`CORE_LOCAL`): Handles exceptions and core interrupts (MachineTimer, MachineSoft, etc.)
- **Entry 1+**: Direct jump to PLIC external interrupt handlers

Core interrupt handlers can be defined by exporting symbols:

```rust
#[no_mangle]
extern "C" fn MachineTimer() {
    // Handle machine timer interrupt
}

#[no_mangle]
extern "C" fn MachineSoft() {
    // Handle machine software interrupt (PLICSW)
}
```

## Startup Sequence

1. `_hpm_start` (assembly entry point)
   - Initialize global pointer and stack pointer
   - Set pre-init trap handler
   - Call `__pre_init` hook
   - Initialize .data, .bss, .fast sections
2. `_hpm_start_rust` (Rust startup)
   - Enable FPU
   - Enable L1 Cache (I-Cache, D-Cache)
   - Initialize non-cacheable sections
   - Call `_setup_interrupts` (configure PLIC vectored mode)
   - Jump to `main()`

## Compatibility

This crate is designed to work alongside `riscv-rt` (pulled in by `hpm-metapac/rt`). Symbol conflicts are avoided by using `_hpm_` prefix for startup symbols.

## License

MIT OR Apache-2.0
