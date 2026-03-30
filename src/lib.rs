#![no_std]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]
// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
// --- CAN ID Definitions ---
pub const CAN_ID_FROM_CONTROL_PANEL: u16 = 0x101;
pub const CAN_ID_TO_CTRL_ACK: u16 = 0x103;
pub const CAN_ID_TO_MAIN_VALVE: u16 = 0x105;
// --- Time Definitions
pub const IGNITION_WAIT_MS: u64 = 20000; // 点火ボタンを押し続けてからイグナイターをONにするまでの時間 (ms)
pub const IGNITION_SEQUENCE_TIMEOUT_MS: u64 = 10000; // シーケンスのタイムアウト (ms)
pub const MAIN_VALVE_OPEN_DELAY_MS: u64 = 3500; // 点火シーケンス開始後,メインバルブを開くまでの時間 (ms)
pub const CTRL_PANEL_TIMEOUT_MS: u64 = 3000; // コントロールパネルの通信タイムアウト (ms)

#[derive(Clone, Copy)]
pub enum STATE {
    IDLE,     // 通常状態.
    IGNITION, // 点火シーケンス実行.
    TIMEOUT,  // cannot operate fire_pin
    CANERROR, // CAN error
}
