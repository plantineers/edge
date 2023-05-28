#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
// Sensors
#[cfg(feature = "dht11")]
mod dht11;
#[cfg(feature = "hw390")]
mod hw390;
#[cfg(feature = "hw390")]
use crate::hw390::Hw390;

extern crate alloc;
use alloc::boxed::Box;
use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;
use embassy_executor::_export::StaticCell;

use embassy_executor::Executor;
use embassy_time::{Duration, Ticker};
use esp_backtrace as _;
use esp_println::logger::init_logger;
use esp_println::println;
use esp_wifi::binary::include::{
    esp_wifi_get_protocol, esp_wifi_set_protocol, wifi_interface_t_WIFI_IF_STA, WIFI_PROTOCOL_LR,
};
use esp_wifi::esp_now::{EspNow, BROADCAST_ADDRESS};
use esp_wifi::initialize;
use futures_util::StreamExt;
use hal::adc::{AdcConfig, Attenuation, ADC, ADC1};
use hal::clock::{ClockControl, CpuClock};
use hal::peripherals::APB_SARADC;
use hal::system::{PeripheralClockControl, SystemExt};
use hal::systimer::SystemTimer;
use hal::{embassy, Rng, IO};
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
/// This initializes the heap to be used by the allocator.
/// DANGER: If something doesn't work for no apparent reason, try decreasing the heap size if you're not using it all.
fn init_heap() {
    const HEAP_SIZE: usize = 24 * 1024;

    extern "C" {
        static mut _heap_start: u32;
    }
    unsafe {
        let heap_start = &_heap_start as *const _ as usize;
        ALLOCATOR.init(heap_start as *mut u8, HEAP_SIZE);
    }
}

#[embassy_executor::task]
async fn run(
    mut esp_now: EspNow<'static>,
    io: IO,
    mut adc1: AdcConfig<ADC1>,
    mut peripheral_cc: PeripheralClockControl,
    adc: APB_SARADC,
) {
    let mut ticker = Ticker::every(Duration::from_secs(60 * 1));
    #[cfg(feature = "hw390")]
    // Create hw390 instance with gpio2
    let mut hw390 = {
        let analog = adc.split();
        let config = AdcConfig::new();
        let pin = adc1.enable_pin(io.pins.gpio2.into_analog(), Attenuation::Attenuation11dB);
        let adc = ADC::<ADC1>::adc(&mut peripheral_cc, analog.adc1, config).unwrap();
        Hw390 { adc, pin }
    };
    loop {
        // TODO: Generate the UUID on start, storing it in the flash
        let mut sensor_data = SensorData::new(['a'; 32]);
        #[cfg(feature = "hw390")]
        {
            sensor_data.add_data(hw390.read());
        }
        println!("Sending data... {:?}", sensor_data);
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

    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock160MHz).freeze();
    #[allow(unused_variables, unused_mut)]
    let mut rtc = {
        let mut rtc = Rtc::new(peripherals.RTC_CNTL);
        rtc.swd.disable();

        rtc.rwdt.disable();
        rtc
    };
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
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

    let mut peripheral_cc = system.peripheral_clock_control;
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks, &mut peripheral_cc);
    embassy::init(&clocks, timer_group0.timer0);
    let executor = EXECUTOR.init(Executor::new());
    let adc = peripherals.APB_SARADC;
    let adc_config = AdcConfig::new();
    executor.run(|spawner| {
        spawner
            .spawn(run(esp_now, io, adc_config, peripheral_cc, adc))
            .ok();
    });
}
