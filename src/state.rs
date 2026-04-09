use core::sync::atomic::{AtomicBool, AtomicU8};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal, watch::Watch};

// event notify definitions
pub static VALVE_OPEN: Signal<CriticalSectionRawMutex, bool> = Signal::new();
pub static EXECUTE_IGNITION: Signal<CriticalSectionRawMutex, bool> = Signal::new();
pub static OPEN_O2: Signal<CriticalSectionRawMutex, bool> = Signal::new();
pub static TO_IGNITION: Signal<CriticalSectionRawMutex, bool> = Signal::new();
pub static PANEL_RX_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static VALVE_RX_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub static IGNITION_ACTIVE: AtomicBool = AtomicBool::new(false);
pub static TIMEOUT_FLAG: AtomicBool = AtomicBool::new(false);
pub static STATE_RESET: AtomicBool = AtomicBool::new(false);
pub static BUTTON_WATCH: Watch<CriticalSectionRawMutex, u8, 2> = Watch::new();
pub static SEQUENCE_STATE: AtomicU8 = AtomicU8::new(0);
