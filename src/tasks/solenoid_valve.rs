use crate::{BUTTON_WATCH, IGNITION_ACTIVE, OPEN_O2, STATE_RESET, TO_IGNITION, VALVE_OPEN};
use core::sync::atomic::Ordering;
use embassy_futures::select::{Either, Either3, select, select3};
use embassy_time::{Duration, Instant, Timer};
use esp_hal::gpio::Output;
use esp_println::println;

#[embassy_executor::task]
pub async fn solenoid_valve_task(
    mut dump: Output<'static>,
    mut fill: Output<'static>,
    mut separate: Output<'static>,
    mut o2: Output<'static>,
) {
    let Some(mut button_receiver) = BUTTON_WATCH.receiver() else {
        println!("Failed to create button state receiver: Maximum receivers reached.");
        return;
    };
    let Some(mut o2_receiver) = OPEN_O2.receiver() else {
        println!("Failed to create O2 state receiver: Maximum receivers reached.");
        return;
    };

    let mut current_button_state = 0;
    let mut current_o2_flag = false;

    // O2テストのための状態管理変数
    let mut o2_test_end_time: Option<Instant> = None;
    let mut prev_o2_test_flag = false; // エッジ検出（0→1の変化）用

    loop {
        let button_fut = button_receiver.changed();
        let o2_fut = o2_receiver.changed();

        // タイマー稼働中かどうかで分ける
        match o2_test_end_time {
            Some(end_time) => {
                match select3(button_fut, o2_fut, Timer::at(end_time)).await {
                    Either3::First(new_state) => {
                        current_button_state = new_state;
                    }
                    Either3::Second(new_o2_flag) => {
                        current_o2_flag = new_o2_flag;
                    }
                    Either3::Third(_) => {
                        // 3秒経過してタイマーが発火した
                        o2_test_end_time = None;
                    }
                }
            }
            None => {
                // タイマー停止中: ボタンとシグナルの2つだけを待つ
                match select(button_fut, o2_fut).await {
                    Either::First(new_state) => {
                        current_button_state = new_state;
                    }
                    Either::Second(new_o2_flag) => {
                        current_o2_flag = new_o2_flag;
                    }
                }
            }
        }

        // ボタン状態からフラグをパース
        let dump_flag = (current_button_state & 1) == 1;
        let fire_flag = ((current_button_state >> 1) & 1) == 1;
        let fill_flag = ((current_button_state >> 2) & 1) == 1;
        let separate_flag = ((current_button_state >> 3) & 1) == 1;
        let valve_set_flag = ((current_button_state >> 4) & 1) == 1;
        let o2_test_flag = ((current_button_state >> 5) & 1) == 1;
        let main_reset_flag = ((current_button_state >> 6) & 1) == 1;
        // println!(
        //     "Button state: dump={}, fire={}, fill={}, separate={}, valve_set={}, o2_test={}, main_reset={}",
        //     dump_flag,
        //     fire_flag,
        //     fill_flag,
        //     separate_flag,
        //     valve_set_flag,
        //     o2_test_flag,
        //     main_reset_flag
        // );

        dump.set_level(dump_flag.into());
        fill.set_level(fill_flag.into());
        separate.set_level(separate_flag.into());

        if fire_flag {
            TO_IGNITION.signal(true);
        }
        if valve_set_flag {
            VALVE_OPEN.signal(false);
        }
        if main_reset_flag {
            STATE_RESET.store(true, Ordering::Relaxed);
        }

        //  3秒間だけ O2 を HIGH にする
        let is_ignition_running = IGNITION_ACTIVE.load(Ordering::Relaxed);
        if is_ignition_running {
            o2_test_end_time = None;
        } else {
            if o2_test_flag && !prev_o2_test_flag {
                // fireがLOW で、かつ o2_test_flagが立ち上がった時だけテストを開始
                // 現在時刻から3秒後を終了時刻としてセット
                o2_test_end_time = Some(Instant::now() + Duration::from_secs(3));
            }
        }

        // 次回の比較のために状態を保存
        prev_o2_test_flag = o2_test_flag;

        // O2電磁弁の実際の出力状態を決定
        // 通常のシーケンス開要求 または テストタイマー稼働中 ならHIGH
        let is_o2_open = current_o2_flag || o2_test_end_time.is_some();
        o2.set_level(is_o2_open.into());
    }
}
