use crate::Data;
use alloc::string::ToString;
use core::any::Any;
use core::marker::PhantomData;
use embedded_hal::adc::OneShot;
use hal::adc::{AdcConfig, AdcPin, Attenuation, ADC, ADC1};
use hal::analog::SarAdcExt;
use hal::gpio::{
    Analog, Bank0GpioRegisterAccess, Gpio2, Gpio2Signals, GpioPin, InputOutputAnalogPinType,
    SingleCoreInteruptStatusRegisterAccessBank0,
};
use hal::peripherals::Peripherals;
use hal::system::{SystemExt, SystemParts};
use hal::IO;
pub struct Hw390<'a> {
    pub adc: ADC<'a, ADC1>,
    pub pin: AdcPin<
        GpioPin<
            Analog,
            Bank0GpioRegisterAccess,
            SingleCoreInteruptStatusRegisterAccessBank0,
            InputOutputAnalogPinType,
            Gpio2Signals,
            2,
        >,
        ADC1,
    >,
}

impl<'a> Hw390<'a> {
    pub fn read(&mut self) -> Data {
        let readout = hal::prelude::nb::block!(self.adc.read(&mut self.pin)).unwrap();
        Data {
            r#type: "humidity".to_string(),
            value: normalise_sensor_data(readout),
        }
    }
}

fn get_humidity(peripherals: Peripherals, io: IO, mut system: SystemParts) -> Data {
    // To use a hw390 capacitive moisture sensor, connect the sensor data cable to GPIO2(A0)
    let analog = peripherals.APB_SARADC.split();
    let mut adc1_config = AdcConfig::new();
    let mut pin = adc1_config.enable_pin(io.pins.gpio2.into_analog(), Attenuation::Attenuation11dB);
    let mut adc1 = ADC::<ADC1>::adc(
        &mut system.peripheral_clock_control,
        analog.adc1,
        adc1_config,
    )
    .unwrap();
    let readout = adc1.read(&mut pin).unwrap();
    Data {
        r#type: "humidity".to_string(),
        value: normalise_sensor_data(readout),
    }
}

/// The hw390 moisture sensor returns a value between 2459 and 4095
/// From our measurements the sensor was in water at 2459 and in air at 4095
/// We want to normalise the values to be between 0 and 1
fn normalise_sensor_data(readout: u16) -> f32 {
    (readout - 2459) as f32 / (4095 - 2459) as f32
}
