#![no_std]
// --- CAN ID Definitions ---
pub const CAN_ID_FROM_CONTROL_PANEL: u16 = 0x101;
pub const CAN_ID_TO_CTRL_MAIN_STATE: u16 = 0x103;
pub const CAN_ID_TO_MAIN_VALVE: u16 = 0x105;
pub const CAN_ID_FROM_CONTROL_ACK: u16 = 0x10a;
pub const CAN_ID_TO_MAIN_VALVE_ACK: u16 = 0x200;
pub const CAN_ID_FROM_MAIN_VALVE_ACK: u16 = 0x201;

// --- Time Definitions
pub const IGNITION_WAIT_MS: u64 = 20000; // 点火ボタンを押し続けてからイグナイターをONにするまでの時間 (ms)
pub const IGNITION_SEQUENCE_TIMEOUT_MS: u64 = 10000; // シーケンスのタイムアウト (ms)
pub const MAIN_VALVE_OPEN_DELAY_MS: u64 = 2000; // 点火シーケンス開始後,メインバルブを開くまでの時間 (ms)
pub const COMMUNICATION_TIMEOUT_MS: u64 = 3000; // メインバルブ、コントロールパネルとの通信タイムアウト (ms)

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
