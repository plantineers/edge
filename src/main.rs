#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(byte_slice_trim_ascii)]
extern crate alloc;

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::vec;
use alloc::vec::Vec;
use embassy_executor::_export::StaticCell;
use embassy_futures::select::{select, Either};

use embassy_executor::Executor;
use embassy_time::{Duration, Ticker};
use esp_backtrace as _;
use esp_println::logger::init_logger;
use esp_println::println;
use esp_wifi::binary::include::{
    esp_wifi_get_protocol, esp_wifi_set_protocol, wifi_interface_t_WIFI_IF_STA, WIFI_PROTOCOL_LR,
};
use esp_wifi::esp_now::{EspNow, PeerInfo, BROADCAST_ADDRESS};
use esp_wifi::initialize;
use futures_util::StreamExt;
use hal::clock::{ClockControl, CpuClock};
use hal::system::SystemExt;
use hal::systimer::SystemTimer;
use hal::{embassy, Rng};
use hal::{peripherals::Peripherals, prelude::*, timer::TimerGroup, Rtc};

// Executor and allocator
#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();
static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[derive(serde::Deserialize, serde::Serialize, Debug)]
struct SensorData {
    device_id: u32,
    sensors: Vec<(String, f32)>,
}
impl SensorData {
    fn new(device_id: u32) -> Self {
        Self {
            device_id,
            sensors: vec![
                ("temperature".to_string(), 23.5),
                ("humidity".to_string(), 42.0),
            ],
        }
    }
}
fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;

    extern "C" {
        static mut _heap_start: u32;
    }
    unsafe {
        let heap_start = &_heap_start as *const _ as usize;
        ALLOCATOR.init(heap_start as *mut u8, HEAP_SIZE);
    }
}

#[embassy_executor::task]
async fn run(mut esp_now: EspNow<'static>) {
    unsafe {
        let protocol: *mut u8 = &mut *Box::new(0);
        esp_wifi_get_protocol(wifi_interface_t_WIFI_IF_STA, protocol);
        if *protocol as u32 == WIFI_PROTOCOL_LR {
            println!("Protocol: LR");
        } else {
            println!("Protocol: {:?}", *protocol);
        }
    }
    let mut ticker = Ticker::every(Duration::from_secs(5));
    loop {
        let res = select(ticker.next(), async {
            let r = esp_now.receive_async().await;
            // Cut off the null bytes
            let my_data = r.get_data();
            println!("Received {:?}", my_data);
            println!(
                "Received {:?}",
                serde_json::from_slice::<SensorData>(my_data).unwrap()
            );
            if r.info.dst_address == BROADCAST_ADDRESS {
                if !esp_now.peer_exists(&r.info.src_address).unwrap() {
                    esp_now
                        .add_peer(PeerInfo {
                            peer_address: r.info.src_address,
                            lmk: None,
                            channel: None,
                            encrypt: false,
                        })
                        .unwrap();
                }
                esp_now
                    .send(
                        &r.info.src_address,
                        serde_json::to_vec(&SensorData::new(2)).unwrap().as_slice(),
                    )
                    .unwrap();
            }
        })
        .await;

        match res {
            Either::First(_) => {
                println!("Send");
                esp_now
                    .send(
                        &BROADCAST_ADDRESS,
                        serde_json::to_vec(&SensorData::new(2)).unwrap().as_slice(),
                    )
                    .unwrap();
            }
            Either::Second(_) => (),
        }
    }
}

#[entry]
fn main() -> ! {
    init_logger(log::LevelFilter::Info);
    esp_wifi::init_heap();
    init_heap();

    let peripherals = Peripherals::take();

    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock160MHz).freeze();
    let mut rtc = {
        let mut rtc = Rtc::new(peripherals.RTC_CNTL);
        rtc.swd.disable();

        rtc.rwdt.disable();
        rtc
    };
    let timer = SystemTimer::new(peripherals.SYSTIMER).alarm0;
    initialize(
        timer,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();

    let (wifi, _) = peripherals.RADIO.split();
    unsafe {
        esp_wifi_set_protocol(
            wifi_interface_t_WIFI_IF_STA,
            WIFI_PROTOCOL_LR.try_into().unwrap(),
        );
    }
    let esp_now = EspNow::new(wifi).unwrap();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timer_group0.timer0);
    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(run(esp_now)).ok();
    });
}
