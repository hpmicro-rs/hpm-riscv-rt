use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    // Copy hpm-link.x to output directory
    println!("cargo:rerun-if-changed=hpm-link.x");
    fs::copy("hpm-link.x", out_dir.join("hpm-link.x")).unwrap();

    // Add linker search path
    println!("cargo:rustc-link-search={}", out_dir.display());

    // Note: The user's .cargo/config.toml should specify the linker scripts:
    //   -Tmemory.x    (user-provided memory layout)
    //   -Tdevice.x    (from hpm-metapac, provides __INTERRUPTS)
    //   -Thpm-link.x  (from hpm-riscv-rt)
}
