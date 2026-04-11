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

use c99l_main_control::{
    tasks::{can_communication::*, execute_ignition::*, solenoid_valve::*}, // 各タスクをインポート
    *,                                                                     // 定数をインポート
};
#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
use core::sync::atomic::Ordering;
use embassy_executor::Spawner;
use embassy_futures::select::{Either4, select4};
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    interrupt::software::SoftwareInterruptControl,
    system::Stack,
    timer::timg::TimerGroup,
    twai::{self, BaudRate, TwaiMode, filter::SingleStandardFilter},
};
use esp_println::println;
use esp_rtos::embassy::Executor;
use static_cell::StaticCell;
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
                        const {
                            SingleStandardFilter::new(
                                b"0xxxxxxxxxx",
                                b"x",
                                [b"xxxxxxxx", b"xxxxxxxx"],
                            )
                        },
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
    let mut last_panel_rx = Instant::now();
    let mut last_valve_rx = Instant::now();

    // main loop controls the system state
    loop {
        // 次のタイムアウト時刻の計算
        let now = Instant::now();
        let panel_alive = now < last_panel_rx + timeout_duration;
        let valve_alive = now < last_valve_rx + timeout_duration;

        if panel_alive && valve_alive {
            // 両方とも正常
            if matches!(state, STATE::CANERROR) {
                // CANERRORからの自動復帰
                state = if has_timed_out {
                    STATE::TIMEOUT
                } else {
                    STATE::IDLE
                };
            }
        } else {
            // どちらか（または両方）がタイムアウトしている
            if !matches!(state, STATE::CANERROR) {
                // println!("Communication Timeout Error!");
                state = STATE::CANERROR;
            }
        }

        let mut next_wakeup = now + embassy_time::Duration::from_secs(3600);
        if panel_alive {
            next_wakeup = next_wakeup.min(last_panel_rx + timeout_duration);
        }
        if valve_alive {
            next_wakeup = next_wakeup.min(last_valve_rx + timeout_duration);
        }

        //  外部からの状態リセット・タイムアウトフラグの処理
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

        // 現在のステートを全体に共有
        SEQUENCE_STATE.store(state.into(), Ordering::Relaxed);

        match select4(
            TO_IGNITION.wait(),
            VALVE_RX_SIGNAL.wait(),
            PANEL_RX_SIGNAL.wait(),
            Timer::at(next_wakeup),
        )
        .await
        {
            // 点火リクエスト受信
            Either4::First(_) => {
                if matches!(state, STATE::IDLE) {
                    OPEN_O2.sender().send(true); // O2開放指示 
                    state = STATE::IGNITION;
                    EXECUTE_IGNITION.signal(true);
                } else if matches!(state, STATE::TIMEOUT) {
                    VALVE_OPEN.signal(true);
                }
            }
            // VALVEからのCAN通信受信
            Either4::Second(_) => {
                last_valve_rx = Instant::now();
            }
            // PANELからのCAN通信受信
            Either4::Third(_) => {
                last_panel_rx = Instant::now();
            }
            // 通信のタイムアウト
            Either4::Fourth(_) => {}
        }
    }
}
