//! Procedural macros for hpm-riscv-rt
//!
//! This crate provides:
//! - `#[entry]` - Define the program entry point
//! - `#[pre_init]` - Define a pre-initialization function
//! - `#[fast]` - Place functions/statics in ILM/DLM
//! - `#[external_interrupt]` - Define PLIC external interrupt handlers

use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, spanned::Spanned, Expr, Item, ItemFn, parse::Parse, parse::ParseStream};

/// Attribute to declare the entry point of the program.
///
/// The function must have the signature `fn() -> !` (never returns).
///
/// # Example
///
/// ```ignore
/// #[entry]
/// fn main() -> ! {
///     loop {}
/// }
/// ```
#[proc_macro_attribute]
pub fn entry(_args: TokenStream, input: TokenStream) -> TokenStream {
    let f = parse_macro_input!(input as ItemFn);

    let fn_attrs = &f.attrs;
    let fn_vis = &f.vis;
    let fn_sig = &f.sig;
    let fn_block = &f.block;

    quote!(
        #(#fn_attrs)*
        #[unsafe(export_name = "main")]
        #fn_vis #fn_sig #fn_block
    )
    .into()
}

/// Attribute to declare a function that runs before RAM is initialized.
///
/// The function must have the signature `unsafe fn()`.
/// At this point:
/// - Stack is valid
/// - .data and .bss are NOT initialized
/// - Interrupts are disabled
///
/// # Example
///
/// ```ignore
/// #[pre_init]
/// unsafe fn setup_watchdog() {
///     // Disable watchdog before RAM init
/// }
/// ```
#[proc_macro_attribute]
pub fn pre_init(_args: TokenStream, input: TokenStream) -> TokenStream {
    let f = parse_macro_input!(input as ItemFn);

    let fn_attrs = &f.attrs;
    let fn_vis = &f.vis;
    let fn_sig = &f.sig;
    let fn_block = &f.block;

    quote!(
        #(#fn_attrs)*
        #[unsafe(export_name = "__pre_init")]
        #fn_vis #fn_sig #fn_block
    )
    .into()
}

/// Place a function or static into fast memory (ILM/DLM).
///
/// Functions are placed into `.fast.text` section (ILM).
/// Statics are placed into `.fast.data` or `.fast.bss` section (DLM).
///
/// # Example
///
/// ```ignore
/// use hpm_riscv_rt::fast;
///
/// #[fast]
/// fn critical_function() {
///     // This function runs from ILM
/// }
///
/// #[fast]
/// static BUFFER: [u8; 1024] = [0; 1024];
/// ```
#[proc_macro_attribute]
pub fn fast(_args: TokenStream, input: TokenStream) -> TokenStream {
    let item = parse_macro_input!(input as Item);

    match item {
        Item::Fn(f) => {
            quote!(
                #[unsafe(link_section = ".fast.text")]
                #[inline(never)]
                #f
            )
            .into()
        }
        Item::Static(item) => {
            // Check if it's uninitialized (MaybeUninit::uninit())
            let section = if is_uninit_expr(&item.expr) {
                quote!(#[unsafe(link_section = ".fast.bss")])
            } else {
                quote!(#[unsafe(link_section = ".fast.data")])
            };

            quote!(
                #section
                #item
            )
            .into()
        }
        _ => {
            let span = item.span();
            syn::Error::new(span, "#[fast] can only be applied to functions or statics")
                .to_compile_error()
                .into()
        }
    }
}

fn is_uninit_expr(expr: &Expr) -> bool {
    if let Expr::Call(call) = expr {
        let s = quote!(#call).to_string();
        s.contains("MaybeUninit") && (s.contains("uninit()") || s.contains("uninit_array()"))
    } else {
        false
    }
}

/// Argument for the external_interrupt attribute.
struct ExternalInterruptArg {
    interrupt: syn::Path,
}

impl Parse for ExternalInterruptArg {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        Ok(ExternalInterruptArg {
            interrupt: input.parse()?,
        })
    }
}

/// Define an external interrupt handler for HPMicro PLIC.
///
/// This macro generates an interrupt handler function that will be called
/// when the specified PLIC interrupt occurs. The function is exported with
/// the interrupt name so it can be placed in the vector table.
///
/// # Example
///
/// ```ignore
/// use hpm_riscv_rt::external_interrupt;
/// use hpm_pac::interrupt;
///
/// #[external_interrupt(interrupt::UART0)]
/// fn uart0_handler() {
///     // Handle UART0 interrupt
/// }
/// ```
///
/// # Safety
///
/// The handler function runs in interrupt context. It must:
/// - Not block or wait
/// - Complete quickly
/// - Handle the interrupt source to prevent re-triggering
#[proc_macro_attribute]
pub fn external_interrupt(args: TokenStream, input: TokenStream) -> TokenStream {
    let args = parse_macro_input!(args as ExternalInterruptArg);
    let f = parse_macro_input!(input as ItemFn);

    let interrupt_path = &args.interrupt;
    let fn_name = &f.sig.ident;
    let fn_body = &f.block;
    let fn_attrs = &f.attrs;
    let fn_vis = &f.vis;

    // Get the interrupt name from the path (last segment)
    let interrupt_name = interrupt_path
        .segments
        .last()
        .map(|s| &s.ident)
        .expect("interrupt path should have at least one segment");

    quote!(
        #(#fn_attrs)*
        #[unsafe(no_mangle)]
        #fn_vis unsafe extern "riscv-interrupt-m" fn #interrupt_name() {
            // The original function body wrapped in unsafe
            #[inline(always)]
            unsafe fn #fn_name() #fn_body

            #fn_name()
        }
    )
    .into()
}
