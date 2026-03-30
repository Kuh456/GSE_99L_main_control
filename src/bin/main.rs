#![no_std]
#![no_main]
use C99L_main_control::*;
use core::sync::atomic::{AtomicU8, Ordering};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, Either3, Either4, select, select3, select4};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, signal::Signal,
};
use embassy_time::with_timeout;
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};
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
static can_error: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static valve_open: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static excute_ignition: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static open_o2: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static timeout: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static to_ignition: Signal<CriticalSectionRawMutex, bool> = Signal::new();

static BUTTON_STATE: AtomicU8 = AtomicU8::new(0);
static SEQUENCE_STATE: AtomicU8 = AtomicU8::new(0);

#[embassy_executor::task]
async fn can_transmit_task(mut tx: twai::TwaiTx<'static, Async>) {
    let mut ticker = Ticker::every(Duration::from_millis(500));
    loop {
        let open_fut = valve_open.wait();

        // どれか1つが完了するのを待つ
        match select(ticker.next(), open_fut).await {
            //  500ms経過した（通常処理）.
            Either::First(_) => {
                let ack = { SEQUENCE_STATE.load(Ordering::Relaxed) };
                let frame_to_send =
                    EspTwaiFrame::new(StandardId::new(CAN_ID_TO_CTRL_ACK).unwrap(), &[ack])
                        .unwrap();
                let result =
                    with_timeout(Duration::from_millis(50), tx.transmit_async(&frame_to_send))
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
    loop {
        let frame = rx.receive_async().await;
        match frame {
            Ok(payload) => {
                let message = payload.data()[0];
                can_error.signal(false); // 正常にCAN受信したらエラー解除.
                match payload.id() {
                    Id::Standard(s_id) if s_id.as_raw() == CAN_ID_FROM_CONTROL_PANEL => {
                        BUTTON_STATE.store(message, Ordering::Relaxed);
                    }
                    _ => {} // ignore the others
                }
            }
            Err(e) => {
                // CAN受信エラー.
                println!("CAN receive error: {:?}", e);
                can_error.signal(true);
            }
        }
    }
}

#[embassy_executor::task]
async fn excute_ignition_task(mut fire: Output<'static>) {
    // 1度きりの実行
    if excute_ignition.wait().await {
        Timer::after(Duration::from_millis(IGNITION_WAIT_MS)).await;
        fire.set_high();

        Timer::after(Duration::from_millis(MAIN_VALVE_OPEN_DELAY_MS)).await;
        valve_open.signal(true);
        open_o2.signal(false);

        Timer::after(Duration::from_millis(IGNITION_SEQUENCE_TIMEOUT_MS)).await;
        fire.set_low();
        timeout.signal(true);
    }
}

#[embassy_executor::task]
async fn solenoid_valve_task(
    mut dump: Output<'static>,
    mut fill: Output<'static>,
    mut separate: Output<'static>,
    mut o2: Output<'static>,
) {
    loop {
        let o2_flag = open_o2.wait().await;
        o2.set_level(o2_flag.into());
        let button_state = { BUTTON_STATE.load(Ordering::Relaxed) };
        let dump_flag = (button_state & 1) == 1;
        let fire_flag = ((button_state >> 1) & 1) == 1;
        let fill_flag = ((button_state >> 2) & 1) == 1;
        let separate_flag = ((button_state >> 3) & 1) == 1;
        let valve_set_flag = ((button_state >> 4) & 1) == 1;
        let o2_test_flag = ((button_state >> 5) & 1) == 1;
        dump.set_level(dump_flag.into());
        fill.set_level(fill_flag.into());
        separate.set_level(separate_flag.into());

        // 点火シーケンスの開始(stateがIDLEのとき)
        // o2電磁弁の操作だけ行う(stateがTIMEOUTのとき)
        if fire_flag {
            to_ignition.signal(true);
        }
        if valve_set_flag {
            valve_open.signal(false)
        };
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
    let mut dump = Output::new(peripherals.GPIO14, Level::Low, OutputConfig::default()); // sch: SW1 14
    let mut ignition = Output::new(peripherals.GPIO32, Level::Low, OutputConfig::default()); // sch: ESP_IGNITER
    let mut o2 = Output::new(peripherals.GPIO27, Level::Low, OutputConfig::default()); // sch: SW2 27
    let mut fill = Output::new(peripherals.GPIO26, Level::Low, OutputConfig::default()); // sch: SW3 26
    let mut separate = Output::new(peripherals.GPIO25, Level::Low, OutputConfig::default()); // sch: SW4 25
    // let mut spare = Output::new(peripherals.GPIO33, Level::Low, OutputConfig::default()); // sch: SW5  33
    let can_tx = Output::new(peripherals.GPIO23, Level::Low, OutputConfig::default());
    let can_rx = Input::new(peripherals.GPIO22, InputConfig::default());
    let dump_12p = Input::new(peripherals.GPIO34, InputConfig::default());
    let dump_12b = Input::new(peripherals.GPIO35, InputConfig::default());
    let igni_12p = Input::new(peripherals.GPIO36, InputConfig::default());
    let igni_12b = Input::new(peripherals.GPIO39, InputConfig::default());
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
    spawner.spawn(excute_ignition_task(ignition)).ok();
    // --- State Definitions ---
    let mut state = STATE::IDLE;
    let mut ticker = Ticker::every(Duration::from_millis(60));
    // main loop controls the system state
    loop {
        match select4(
            can_error.wait(),
            timeout.wait(),
            to_ignition.wait(),
            ticker.next(),
        )
        .await
        {
            /// CANエラー.
            Either4::First(can_flag) => {
                if can_flag {
                    state = STATE::CANERROR;
                } else if !matches!(state, STATE::TIMEOUT) {
                    state = STATE::IDLE;
                }
            }

            /// 点火シーケンスタイムアウト.
            Either4::Second(_) => {
                state = STATE::TIMEOUT;
            }

            /// 点火リクエスト受信.
            /// IDLE -> IGNITIONに移るときだけ点火シーケンスを実行.
            /// state = TIMEOUTのときは酸素電磁弁のみを操作.
            Either4::Third(_) => {
                if matches!(state, STATE::IDLE) {
                    open_o2.signal(true);
                    state = STATE::IGNITION;
                    excute_ignition.signal(true);
                } else if matches!(state, STATE::TIMEOUT) {
                    open_o2.signal(true);
                }
            }
            Either4::Fourth(_) => {
                // queueがあふれたら捨てる
                SEQUENCE_STATE.store(state as u8, Ordering::Relaxed);
            }
        }
    }
    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples
}
