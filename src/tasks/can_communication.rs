use crate::{
    BUTTON_WATCH, CAN_ID_FROM_CONTROL_PANEL, CAN_ID_FROM_MAIN_VALVE_ACK, CAN_ID_TO_CTRL_MAIN_STATE,
    CAN_ID_TO_MAIN_VALVE, CAN_ID_TO_MAIN_VALVE_ACK, PANEL_RX_SIGNAL, SEQUENCE_STATE, VALVE_OPEN,
    VALVE_RX_SIGNAL,
};
use core::sync::atomic::Ordering;
use embassy_futures::{select::{Either, select}, yield_now};
use embassy_time::{Duration, Ticker, Timer, with_timeout};
use embedded_can::{Frame, Id};
use esp_hal::{
    Async,
    twai::{self, EspTwaiFrame, StandardId},
};
use esp_println::println;

#[embassy_executor::task]
pub async fn can_transmit_task(mut tx: twai::TwaiTx<'static, Async>) {
    let mut ticker = Ticker::every(Duration::from_millis(50));
    loop {
        let open_fut = VALVE_OPEN.wait();

        // どれか1つが完了するのを待つ
        match select(ticker.next(), open_fut).await {
            //  500ms経過した（通常処理）.
            Either::First(_) => {
                let main_state = { SEQUENCE_STATE.load(Ordering::Relaxed) }; // 送信する前に現在の状態を読み込む
                send_can_message(&mut tx, CAN_ID_TO_CTRL_MAIN_STATE, &[main_state]).await;
                send_can_message(&mut tx, CAN_ID_TO_MAIN_VALVE_ACK, &[main_state]).await;
            }
            // valve close:0 , valve open:1
            Either::Second(valve_flag) => {
                let kind = valve_flag as u8;
                send_can_message(&mut tx, CAN_ID_TO_MAIN_VALVE, &[kind]).await;
            }
        }
    }
}

#[embassy_executor::task]
pub async fn can_receive_task(mut rx: twai::TwaiRx<'static, Async>) {
    let button_sender = BUTTON_WATCH.sender();
    button_sender.send(0); // 未初期化状態を防ぐために初期値0を送信
    loop {
        if let Some(payload) = receive_can_message(&mut rx, 3).await {
            {
                match payload.id() {
                    Id::Standard(s_id) if s_id.as_raw() == CAN_ID_FROM_CONTROL_PANEL => {
                        if payload.data().len() > 0 {
                            PANEL_RX_SIGNAL.signal(());
                            button_sender.send(payload.data()[0]);
                        }
                    }
                    Id::Standard(s_id) if s_id.as_raw() == CAN_ID_FROM_MAIN_VALVE_ACK => {
                        VALVE_RX_SIGNAL.signal(());
                    }
                    _ => {}
                }
            }
        }
        yield_now().await;
    }
}

// --- CAN Helper Functions ---
/// CANメッセージを送信するラッパー関数
async fn send_can_message(tx: &mut twai::TwaiTx<'_, Async>, id: u16, data: &[u8]) {
    let std_id = match StandardId::new(id) {
        // 11bitの標準IDとして有効かチェック
        Some(id) => id,
        None => {
            esp_println::println!("Error: Invalid CAN ID (0x{:x})", id);
            return;
        }
    };

    let frame = EspTwaiFrame::new(std_id, data).unwrap();
    let result = tx.transmit_async(&frame).await;
    match result {
        Ok(()) => {} // 送信成功
        Err(e) => {
            // esp_println::println!("CAN Transmit Error (ID: 0x{:x}): {:?}", id, e);
        }
    }
}

/// CANメッセージを受信するラッパー関数
async fn receive_can_message(
    rx: &mut twai::TwaiRx<'_, Async>,
    timeout_secs: u64,
) -> Option<EspTwaiFrame> {
    let result = with_timeout(Duration::from_secs(timeout_secs), rx.receive_async()).await;

    match result {
        Ok(Ok(frame)) => Some(frame), // 受信成功
        Ok(Err(e)) => {
            // println!("CAN Receive Error: {:?}", e);
            Timer::after(Duration::from_millis(50)).await;
            None
        }
        Err(_) => {
            // タイムアウトは通常の待機状態なのでエラーログは出さない
            Timer::after(Duration::from_millis(50)).await;
            None
        }
    }
}
