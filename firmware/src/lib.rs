//! Window Sensor firmware library — host-testable core logic.
//!
//! This lib exposes the pure-logic modules (classifier, BTHome encoder, etc.)
//! for unit testing on x86_64 without hardware dependencies.

#![no_std]
#![allow(dead_code)]

pub mod advertising;
pub mod battery;
pub mod bthome;
pub mod classifier;
pub mod diagnostics;
pub mod gesture;
pub mod mems;
pub mod mems_button;
pub mod mems_tuning;
pub mod ota;
pub mod ota_boot;
pub mod partition;
pub mod settings;
pub mod setup;
pub mod telemetry;
pub mod window_tuning;
