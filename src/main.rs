#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
extern crate alloc;
#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

use alloc::string::String;
use alloc::vec;

use esp_backtrace as _;
use esp_println::logger::init_logger;
use esp_println::println;
use esp_wifi::current_millis;
use esp_wifi::esp_now::{PeerInfo, BROADCAST_ADDRESS};
use esp_wifi::initialize;
use hal::clock::{ClockControl, CpuClock};
use hal::system::SystemExt;
use hal::systimer::SystemTimer;
use hal::Rng;
use hal::{peripherals::Peripherals, prelude::*, Rtc};
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
#[derive(serde::Deserialize, serde::Serialize, Debug)]
struct SensorData {
    device_id: u32,
    temperature: f32,
    humidity: f32,
}
impl SensorData {
    fn new(device_id: u32, temperature: f32, humidity: f32) -> Self {
        Self {
            device_id,
            temperature,
            humidity,
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
    let _rtc = {
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
    let mut esp_now = esp_wifi::esp_now::EspNow::new(wifi).unwrap();

    println!("esp-now version {}", esp_now.get_version().unwrap());

    let mut next_send_time = current_millis() + 5 * 1000;
    loop {
        let r = esp_now.receive();
        if let Some(r) = r {
            println!("Received {:x?}", r);
            println!("Message: {}", String::from_utf8_lossy(&r.data));

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
                esp_now.send(&r.info.src_address, b"Hello Peer").unwrap();
            }
        }

        if current_millis() >= next_send_time {
            next_send_time = current_millis() + 5 * 1000;
            println!("Send");
            esp_now.send(&BROADCAST_ADDRESS, b"0123456789").unwrap();
        }
    }
}
