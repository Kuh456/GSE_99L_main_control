use crate::{
    EXECUTE_IGNITION, IGNITION_ACTIVE, IGNITION_SEQUENCE_TIMEOUT_MS, IGNITION_WAIT_MS,
    MAIN_VALVE_OPEN_DELAY_MS, OPEN_O2, TIMEOUT_FLAG, VALVE_OPEN,
};
use core::sync::atomic::Ordering;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Timer};
use esp_hal::gpio::{Input, Output};
use esp_println::println;

#[embassy_executor::task]
pub async fn execute_ignition_task(mut fire: Output<'static>, mut emergency_sw: Input<'static>) {
    loop {
        // 点火要求、または緊急停止を待機する
        match select(EXECUTE_IGNITION.wait(), emergency_sw.wait_for_high()).await {
            Either::First(true) => {
                // 点火要求を受信して進行
            }
            Either::First(false) => continue, // 誤信号などは無視
            Either::Second(_) => {
                println!("Emergency switch pressed during IDLE.");
                continue; // 待機中の緊急停止。安全なのでループの先頭に戻る
            }
        }

        //  タスクがどう抜け出しても安全に終了
        struct IgnitionGuard;
        impl Drop for IgnitionGuard {
            fn drop(&mut self) {
                IGNITION_ACTIVE.store(false, Ordering::Relaxed);
                OPEN_O2.sender().send(true); // O2開放指示
                TIMEOUT_FLAG.store(true, Ordering::Relaxed); // 必要に応じてメインのステートを更新
            }
        }
        let _guard = IgnitionGuard;
        IGNITION_ACTIVE.store(true, Ordering::Relaxed);

        // 待機フェーズ (緊急スイッチを常に監視)
        match select(
            Timer::after(Duration::from_millis(IGNITION_WAIT_MS)),
            emergency_sw.wait_for_high(),
        )
        .await
        {
            Either::First(_) => { /* 待機完了、次へ */ }
            Either::Second(_) => {
                println!("Aborted: Emergency switch activated during ignition wait time.");
                continue; // ガードがドロップされ、安全状態に戻ってループ先頭へ
            }
        }

        // 点火フェーズ
        fire.set_high();
        match select(
            Timer::after(Duration::from_millis(MAIN_VALVE_OPEN_DELAY_MS)),
            emergency_sw.wait_for_high(),
        )
        .await
        {
            Either::First(_) => { /* ディレイ完了、バルブオープンへ */ }
            Either::Second(_) => {
                println!("Aborted: Emergency switch activated during ignition.");
                fire.set_low();
                continue;
            }
        }

        // バルブ開放と最終タイムアウト
        VALVE_OPEN.signal(true);
        OPEN_O2.sender().send(true); // O2開放指示
        Timer::after(Duration::from_millis(IGNITION_SEQUENCE_TIMEOUT_MS)).await;

        fire.set_low();
        // ループの先頭に戻り、次の点火シーケンスに備える
    }
}
