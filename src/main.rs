#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(byte_slice_trim_ascii)]
// Sensors
#[cfg(feature = "dht11")]
mod dht11;
#[cfg(feature = "hw390")]
mod hw390;

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
use hal::adc::{AdcConfig, Attenuation, ADC, ADC1};
use hal::clock::{ClockControl, CpuClock};
use hal::system::SystemExt;
use hal::systimer::SystemTimer;
use hal::{embassy, Delay, Rng, IO};
use hal::{peripherals::Peripherals, prelude::*, timer::TimerGroup, Rtc};
use postcard::to_vec;

// Executor and allocator
#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();
static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[derive(serde::Deserialize, serde::Serialize, Debug)]
struct SensorData {
    controller: [char; 32],
    sensors: Vec<Data>,
}
#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct Data {
    r#type: String,
    value: f32,
}
impl Data {
    fn new(r#type: String, value: f32) -> Self {
        Self { r#type, value }
    }
}
impl SensorData {
    fn new(controller: [char; 32]) -> Self {
        Self {
            controller,
            sensors: vec![],
        }
    }
    fn add_data(&mut self, data: Data) {
        self.sensors.push(data);
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
    let mut ticker = Ticker::every(Duration::from_secs(60 * 5));
    loop {
        let sensor_data = SensorData::new(['a'; 32]);
        println!("Sending data...");
        esp_now
            .send(
                &BROADCAST_ADDRESS,
                to_vec::<SensorData, 200>(&SensorData::new(['a'; 32]))
                    .unwrap()
                    .as_slice(),
            )
            .unwrap();
        ticker.next().await;
    }
}

#[entry]
fn main() -> ! {
    init_logger(log::LevelFilter::Info);
    init_heap();

    let peripherals = Peripherals::take();

    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock160MHz).freeze();
    let mut rtc = {
        let mut rtc = Rtc::new(peripherals.RTC_CNTL);
        rtc.swd.disable();

        rtc.rwdt.disable();
        rtc
    };
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    #[cfg(feature = "dht11")]
    {
        // Testing DHT11
        let mut delay = Delay::new(&clocks);
        let pin = io.pins.gpio7.into_open_drain_output();
        let data = dht11::poll_sensor(pin, &mut delay);
        println!("DHT11: {:?}", data);
    }
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
