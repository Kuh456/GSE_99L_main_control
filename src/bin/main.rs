#![no_std]
#![no_main]
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
use C99L_main_control::*;
use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, Either3, Either4, select, select3, select4};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal, watch::Watch};
use embassy_time::with_timeout;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_can::{Frame, Id};
use esp_backtrace as _;
use esp_hal::{
    Async,
    clock::CpuClock,
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    interrupt::software::SoftwareInterruptControl,
    system::Stack,
    timer::timg::TimerGroup,
    twai::{self, BaudRate, EspTwaiFrame, StandardId, TwaiMode, filter::SingleStandardFilter},
};
use esp_println::println;
use esp_rtos::embassy::Executor;
use static_cell::StaticCell;

// event notify definitions
static VALVE_OPEN: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static EXECUTE_IGNITION: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static OPEN_O2: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static TO_IGNITION: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static PANEL_RX_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static VALVE_RX_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

static IGNITION_ACTIVE: AtomicBool = AtomicBool::new(false);
static TIMEOUT_FLAG: AtomicBool = AtomicBool::new(false);
static STATE_RESET: AtomicBool = AtomicBool::new(false);
static BUTTON_WATCH: Watch<CriticalSectionRawMutex, u8, 2> = Watch::new();
static SEQUENCE_STATE: AtomicU8 = AtomicU8::new(0);

#[embassy_executor::task]
async fn can_transmit_task(mut tx: twai::TwaiTx<'static, Async>) {
    let mut ticker = Ticker::every(Duration::from_millis(500));
    loop {
        let open_fut = VALVE_OPEN.wait();

        // どれか1つが完了するのを待つ
        match select(ticker.next(), open_fut).await {
            //  500ms経過した（通常処理）.
            Either::First(_) => {
                let main_state = { SEQUENCE_STATE.load(Ordering::Relaxed) }; // 送信する前に現在の状態を読み込む
                let frame_to_valve = EspTwaiFrame::new(
                    StandardId::new(CAN_ID_TO_MAIN_VALVE_ACK).unwrap(),
                    &[main_state],
                )
                .unwrap();
                let frame_to_ctrl = EspTwaiFrame::new(
                    StandardId::new(CAN_ID_TO_CTRL_MAIN_STATE).unwrap(),
                    &[main_state],
                )
                .unwrap();
                let result1 =
                    with_timeout(Duration::from_millis(50), tx.transmit_async(&frame_to_ctrl))
                        .await;
                match result1 {
                    //  指定時間内に終わらなかった (外側のResultがErr)
                    Err(_timeout_error) => {}

                    //  時間内に終わったが、通信エラーまたは切断が発生した (内側のResultがErr)
                    Ok(Err(can_err)) => {
                        println!("can_tx_err: {:?}", can_err);
                    }
                    //  時間内に正常に受信完了 (両方ともOk)
                    Ok(Ok(())) => {}
                }
                let result2 = with_timeout(
                    Duration::from_millis(50),
                    tx.transmit_async(&frame_to_valve),
                )
                .await;
                match result2 {
                    //  指定時間内に終わらなかった (外側のResultがErr)
                    Err(_timeout_error) => {}

                    //  時間内に終わったが、通信エラーまたは切断が発生した (内側のResultがErr)
                    Ok(Err(can_err)) => {
                        println!("can_tx_err: {:?}", can_err);
                    }
                    //  時間内に正常に受信完了 (両方ともOk)
                    Ok(Ok(())) => {}
                }
            }

            // valve close:0 , valve open:1
            Either::Second(valve_flag) => {
                let kind = valve_flag as u8;
                let valve_angle_kind =
                    EspTwaiFrame::new(StandardId::new(CAN_ID_TO_MAIN_VALVE).unwrap(), &[kind])
                        .unwrap();
                let result = with_timeout(
                    Duration::from_millis(50),
                    tx.transmit_async(&valve_angle_kind),
                )
                .await;
                match result {
                    //  指定時間内に終わらなかった (外側のResultがErr)
                    Err(_timeout_error) => {}

                    //  時間内に終わったが、通信エラーまたは切断が発生した (内側のResultがErr)
                    Ok(Err(can_err)) => {
                        println!("can_tx_err: {:?}", can_err);
                    }
                    //  時間内に正常に受信完了 (両方ともOk)
                    Ok(Ok(())) => {}
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn can_receive_task(mut rx: twai::TwaiRx<'static, Async>) {
    let button_sender = BUTTON_WATCH.sender();
    button_sender.send(0); // 未初期化状態を防ぐために初期値0を送信
    loop {
        let result = with_timeout(Duration::from_secs(3), rx.receive_async()).await;
        match result {
            Ok(frame_result) => match frame_result {
                Ok(payload) => {
                    let message = payload.data()[0];
                    match payload.id() {
                        Id::Standard(s_id) if s_id.as_raw() == CAN_ID_FROM_CONTROL_PANEL => {
                            PANEL_RX_SIGNAL.signal(());
                            button_sender.send(message);
                        }
                        Id::Standard(s_id) if s_id.as_raw() == CAN_ID_FROM_MAIN_VALVE_ACK => {
                            VALVE_RX_SIGNAL.signal(());
                        }
                        _ => {}
                    }
                }
                Err(can_err) => {
                    println!("can_rx_err: {:?}", can_err);
                }
            },
            Err(_) => {
                // CAN受信エラー(TIMEOUT).
            }
        }
    }
}

#[embassy_executor::task]
async fn execute_ignition_task(mut fire: Output<'static>, emergency_sw: Input<'static>) {
    // 1度きりの実行
    if EXECUTE_IGNITION.wait().await || emergency_sw.is_low() {
        struct IgnitionGuard;
        impl Drop for IgnitionGuard {
            fn drop(&mut self) {
                IGNITION_ACTIVE.store(false, Ordering::Relaxed);
                OPEN_O2.signal(false);
                TIMEOUT_FLAG.store(true, Ordering::Relaxed);
            }
        }
        let _guard = IgnitionGuard;
        Timer::after(Duration::from_millis(IGNITION_WAIT_MS)).await;
        if emergency_sw.is_high() {
            println!(
                "Emergency switch activated during ignition wait time. Aborting ignition sequence."
            );
            return;
        };
        fire.set_high();

        Timer::after(Duration::from_millis(MAIN_VALVE_OPEN_DELAY_MS)).await;
        if emergency_sw.is_high() {
            println!(
                "Emergency switch activated during ignition wait time. Aborting ignition sequence."
            );
            fire.set_low();
            return;
        };
        VALVE_OPEN.signal(true);
        OPEN_O2.signal(false);

        Timer::after(Duration::from_millis(IGNITION_SEQUENCE_TIMEOUT_MS)).await;
        fire.set_low();
        TIMEOUT_FLAG.store(true, Ordering::Relaxed);
    }
}

#[embassy_executor::task]
async fn solenoid_valve_task(
    mut dump: Output<'static>,
    mut fill: Output<'static>,
    mut separate: Output<'static>,
    mut o2: Output<'static>,
) {
    let Some(mut button_receiver) = BUTTON_WATCH.receiver() else {
        println!("Failed to create button state receiver: Maximum receivers reached.");
        return;
    };

    let mut current_button_state = 0;
    let mut current_o2_flag = false;

    // O2テストのための状態管理変数
    let mut o2_test_end_time: Option<Instant> = None;
    let mut prev_o2_test_flag = false; // エッジ検出（0→1の変化）用

    loop {
        let button_fut = button_receiver.changed();
        let o2_fut = OPEN_O2.wait();

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
        println!(
            "Button state: dump={}, fire={}, fill={}, separate={}, valve_set={}, o2_test={}, main_reset={}",
            dump_flag,
            fire_flag,
            fill_flag,
            separate_flag,
            valve_set_flag,
            o2_test_flag,
            main_reset_flag
        );

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

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.2.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    static APP_CORE_STACK: StaticCell<Stack<8192>> = StaticCell::new();
    let app_core_stack = APP_CORE_STACK.init(Stack::new());
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    // --- GPIO Definitions
    let dump = Output::new(peripherals.GPIO14, Level::Low, OutputConfig::default()); // sch: SW1 14
    let ignition = Output::new(peripherals.GPIO32, Level::Low, OutputConfig::default()); // sch: ESP_IGNITER
    let o2 = Output::new(peripherals.GPIO27, Level::Low, OutputConfig::default()); // sch: SW2 27
    let fill = Output::new(peripherals.GPIO26, Level::Low, OutputConfig::default()); // sch: SW3 26
    let separate = Output::new(peripherals.GPIO25, Level::Low, OutputConfig::default()); // sch: SW4 25
    // let mut spare = Output::new(peripherals.GPIO33, Level::Low, OutputConfig::default()); // sch: SW5  33
    let emergency_sw = Input::new(peripherals.GPIO4, InputConfig::default()); // sch: emergency_sw 4
    let can_tx = Output::new(peripherals.GPIO23, Level::Low, OutputConfig::default());
    let can_rx = Input::new(peripherals.GPIO22, InputConfig::default());
    // let dump_12p = Input::new(peripherals.GPIO34, InputConfig::default());
    // let dump_12b = Input::new(peripherals.GPIO35, InputConfig::default());
    // let igni_12p = Input::new(peripherals.GPIO36, InputConfig::default());
    // let igni_12b = Input::new(peripherals.GPIO39, InputConfig::default());
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    // TODO: Spawn some tasks
    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        #[cfg(target_arch = "xtensa")]
        sw_int.software_interrupt0,
        sw_int.software_interrupt1,
        app_core_stack,
        move || {
            static EXECUTOR: StaticCell<Executor> = StaticCell::new();
            let executor = EXECUTOR.init(Executor::new());
            executor.run(|spawner| {
                // CAN設定
                const TWAI_BAUDRATE: twai::BaudRate = BaudRate::B125K;
                let mut can_config = twai::TwaiConfiguration::new(
                    peripherals.TWAI0,
                    can_rx,
                    can_tx,
                    TWAI_BAUDRATE,
                    TwaiMode::Normal,
                )
                .into_async();
                // Partially filter the incoming messages to reduce overhead of receiving
                // undesired messages
                can_config.set_filter(
        const { SingleStandardFilter::new(b"0xxxxxxxxxx", b"x", [b"xxxxxxxx", b"xxxxxxxx"]) },
    );
                let can = can_config.start();
                let (rx, tx) = can.split();
                spawner.spawn(can_receive_task(rx)).ok();
                spawner.spawn(can_transmit_task(tx)).ok();
            });
        },
    );
    spawner
        .spawn(solenoid_valve_task(dump, fill, separate, o2))
        .ok();
    spawner
        .spawn(execute_ignition_task(ignition, emergency_sw))
        .ok();
    // --- State Definitions ---
    let mut state = STATE::IDLE;
    let mut has_timed_out = false;
    let timeout_duration = Duration::from_millis(COMMUNICATION_TIMEOUT_MS);
    let mut panel_deadline = Instant::now() + timeout_duration;
    let mut valve_deadline = Instant::now() + timeout_duration;
    // main loop controls the system state
    loop {
        let next_timeout = panel_deadline.min(valve_deadline);
        SEQUENCE_STATE.store(state.into(), Ordering::Relaxed);
        let state_reset_flag = STATE_RESET.load(Ordering::Relaxed);
        if state_reset_flag && matches!(state, STATE::TIMEOUT) {
            state = STATE::IDLE;
            has_timed_out = false;
            STATE_RESET.store(false, Ordering::Relaxed);
        }
        if TIMEOUT_FLAG.load(Ordering::Relaxed) {
            state = STATE::TIMEOUT;
            has_timed_out = true;
            TIMEOUT_FLAG.store(false, Ordering::Relaxed);
        }
        match select4(
            TO_IGNITION.wait(),
            VALVE_RX_SIGNAL.wait(),
            PANEL_RX_SIGNAL.wait(),
            Timer::at(next_timeout), // <--- デッドライン時刻までスリープ
        )
        .await
        {
            //  点火リクエスト受信
            // IDLE -> IGNITIONに移るときだけ点火シーケンスを実行.
            // state = TIMEOUTのときはメインバルブのみを操作.
            Either4::First(_) => {
                if matches!(state, STATE::IDLE) {
                    IGNITION_ACTIVE.store(true, Ordering::Relaxed);
                    OPEN_O2.signal(true);
                    state = STATE::IGNITION;
                    EXECUTE_IGNITION.signal(true);
                } else if matches!(state, STATE::TIMEOUT) {
                    VALVE_OPEN.signal(true);
                }
            }

            //  CAN通信でバルブ状態の受信
            Either4::Second(_) => {
                valve_deadline = Instant::now() + timeout_duration;
                if matches!(state, STATE::CANERROR) {
                    // 両方とも正常なら復帰
                    let now = Instant::now();
                    if panel_deadline > now && valve_deadline > now {
                        state = if has_timed_out {
                            STATE::TIMEOUT
                        } else {
                            STATE::IDLE
                        };
                    }
                }
            }

            //  CAN通信のイベントを受信
            Either4::Third(_) => {
                panel_deadline = Instant::now() + timeout_duration;
                // エラー状態からの自動復帰ロジック
                if matches!(state, STATE::CANERROR) {
                    // 両方とも正常なら復帰
                    let now = Instant::now();
                    if panel_deadline > now && valve_deadline > now {
                        state = if has_timed_out {
                            STATE::TIMEOUT
                        } else {
                            STATE::IDLE
                        };
                    }
                }
            }

            // 通信のタイムアウト
            Either4::Fourth(_) => {
                println!("Communication Timeout Error!");
                state = STATE::CANERROR;
            }
        }

        // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples
    }
}
