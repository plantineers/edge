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
use alloc::string::{String, ToString};
use alloc::vec;
use alloc::vec::Vec;
use embassy_executor::_export::StaticCell;

use embassy_executor::Executor;
use embassy_time::{Duration, Ticker};
use esp_backtrace as _;
use esp_println::logger::init_logger;
use esp_println::println;
use esp_wifi::binary::include::{
    esp_wifi_set_protocol, wifi_interface_t_WIFI_IF_STA, WIFI_PROTOCOL_LR,
};
use esp_wifi::esp_now::{EspNow, BROADCAST_ADDRESS};
use esp_wifi::initialize;
use hal::adc::{AdcConfig, ADC1};
#[allow(unused_imports)]
use hal::adc::{Attenuation, ADC};
use hal::clock::{ClockControl, Clocks, CpuClock};
use hal::i2c::I2C;
use hal::peripherals::{APB_SARADC, I2C0};
use hal::system::{PeripheralClockControl, SystemExt};
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
    const HEAP_SIZE: usize = 4 * 1024;

    extern "C" {
        static mut _heap_start: u32;
    }
    unsafe {
        let heap_start = &_heap_start as *const _ as usize;
        ALLOCATOR.init(heap_start as *mut u8, HEAP_SIZE);
    }
}

#[embassy_executor::task]
async fn main_loop(
    mut esp_now: EspNow<'static>,
    io: IO,
    #[allow(unused)] mut adc1: AdcConfig<ADC1>,
    #[allow(unused)] mut peripheral_cc: PeripheralClockControl,
    #[allow(unused)] mut adc: APB_SARADC,
    #[allow(unused)] mut i2c0: I2C0,
    #[allow(unused)] clocks: Clocks<'static>,
    #[allow(unused)] mut delay: Delay,
) {
    let mut ticker = Ticker::every(Duration::from_secs(10 * 1));
    #[cfg(feature = "hw390")]
    // Create hw390 instance with gpio2
    let mut hw390 = {
        let analog = adc.split();
        let mut adc1_config = AdcConfig::new();
        let mut pin =
            adc1_config.enable_pin(io.pins.gpio2.into_analog(), Attenuation::Attenuation11dB);
        let mut adc1 = ADC::<ADC1>::adc(&mut peripheral_cc, analog.adc1, adc1_config).unwrap();
        Hw390 { adc: adc1, pin }
    };
    #[cfg(feature = "tsl2591")]
    // TODO: Move to seperate module
    let mut tsl = {
        let mut i2c = {
            I2C::new(
                i2c0,
                // I2C on the xiao ESP32c3 is on D5 and D6, meaning gpio6, gpio7
                io.pins.gpio6,
                io.pins.gpio7,
                400u32.kHz(),
                &mut peripheral_cc,
                &clocks,
            )
        };
        let mut t = tsl2591::Driver::new(i2c).unwrap();
        t.enable().unwrap();
        t.set_timing(None).unwrap();
        t.set_gain(None).unwrap();
        t
    };
    loop {
        // TODO: Generate the UUID on start, storing it in the flash
        let mut sensor_data = SensorData::new(['a'; 32]);
        #[cfg(feature = "hw390")]
        {
            sensor_data.add_data(hw390.read());
        }
        #[cfg(feature = "tsl2591")]
        {
            sensor_data.add_data(Data::new("light".to_string(), {
                // I really don't know why the library doesn't abstract this away. Maybe we should make a PR?
                // TODO: Think about normalizing the lux values, though it's probably not needed
                let (ch_0, ch_1) = tsl.get_channel_data(&mut delay).unwrap();
                tsl.calculate_lux(ch_0, ch_1).unwrap()
            }));
        }
        println!("Sending data: {:?}", sensor_data);
        esp_now
            .send(
                &BROADCAST_ADDRESS,
                to_vec::<SensorData, 200>(&sensor_data).unwrap().as_slice(),
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
    let i2c0 = peripherals.I2C0;
    let delay = Delay::new(&clocks);
    executor.run(|spawner| {
        spawner
            .spawn(main_loop(
                esp_now,
                io,
                adc_config,
                peripheral_cc,
                adc,
                i2c0,
                clocks,
                delay,
            ))
            .ok();
    });
}
