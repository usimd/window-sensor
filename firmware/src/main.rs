#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_std)]
#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_main)]
#![allow(dead_code)] // Driver register constants and board type aliases are reference documentation

#[cfg(all(target_arch = "arm", target_os = "none"))]
include!("main_embedded.rs");

#[cfg(not(all(target_arch = "arm", target_os = "none")))]
fn main() {}
