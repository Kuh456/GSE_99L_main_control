#![no_std]
use core::sync::atomic::{AtomicBool, AtomicU8};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal, watch::Watch};
pub mod tasks;
// --- CAN ID Definitions ---
pub const CAN_ID_FROM_CONTROL_PANEL: u16 = 0x101;
pub const CAN_ID_TO_CTRL_MAIN_STATE: u16 = 0x103;
pub const CAN_ID_TO_MAIN_VALVE: u16 = 0x105;
pub const CAN_ID_TO_MAIN_VALVE_ACK: u16 = 0x200;
pub const CAN_ID_FROM_MAIN_VALVE_ACK: u16 = 0x201;

// --- Time Definitions
pub const IGNITION_WAIT_MS: u64 = 20000; // 点火ボタンを押し続けてからイグナイターをONにするまでの時間 (ms)
pub const IGNITION_SEQUENCE_TIMEOUT_MS: u64 = 10000; // シーケンスのタイムアウト (ms)
pub const MAIN_VALVE_OPEN_DELAY_MS: u64 = 2000; // イグナイター点火後,メインバルブを開くまでの時間 (ms)
pub const COMMUNICATION_TIMEOUT_MS: u64 = 3000; // メインバルブ、コントロールパネルとの通信タイムアウト (ms)

// event notify definitions
pub static VALVE_OPEN: Signal<CriticalSectionRawMutex, bool> = Signal::new();
pub static EXECUTE_IGNITION: Signal<CriticalSectionRawMutex, bool> = Signal::new();
pub static TO_IGNITION: Signal<CriticalSectionRawMutex, bool> = Signal::new();
pub static PANEL_RX_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static VALVE_RX_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

// status flag definitions
pub static IGNITION_ACTIVE: AtomicBool = AtomicBool::new(false);
pub static TIMEOUT_FLAG: AtomicBool = AtomicBool::new(false);
pub static STATE_RESET: AtomicBool = AtomicBool::new(false);
pub static BUTTON_WATCH: Watch<CriticalSectionRawMutex, u8, 2> = Watch::new();
pub static SEQUENCE_STATE: AtomicU8 = AtomicU8::new(0);
pub static OPEN_O2: Watch<CriticalSectionRawMutex, bool, 2> = Watch::new();

#[derive(Clone, Copy)]
pub enum STATE {
    IDLE,     // 通常状態.
    IGNITION, // 点火シーケンス実行.
    TIMEOUT,  // cannot operate fire_pin
    CANERROR, // CAN error
}
impl From<STATE> for u8 {
    fn from(state: STATE) -> Self {
        match state {
            STATE::IDLE => 0,
            STATE::IGNITION => 1,
            STATE::TIMEOUT => 2,
            STATE::CANERROR => 3,
        }
    }
}
