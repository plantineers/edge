use crate::Data;
use alloc::string::ToString;
use embedded_hal::adc::OneShot;
use esp_println::println;
use hal::adc::{AdcPin, ADC, ADC1};
use hal::gpio::{
    Analog, Bank0GpioRegisterAccess, Gpio2Signals, GpioPin, InputOutputAnalogPinType,
    SingleCoreInteruptStatusRegisterAccessBank0,
};
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
            r#type: "soil-moisture".to_string(),
            value: normalise_humidity_data(readout),
        }
    }
}

/// The hw390 moisture sensor returns a value between 3000 and 4095
/// From our measurements the sensor was in water at 3000 and in air at 4095
/// We want to normalise the values to be between 0 and 1, so that 1 is in water and 0 is in air
fn normalise_humidity_data(readout: u16) -> f32 {
    println!("HW390 readout: {}", readout);
    let min_value = 3000;
    let max_value = 4095;
    let normalized_value =
        (readout.saturating_sub(min_value)) as f32 / (max_value - min_value) as f32;
    // And now invert the value
    1.0 - normalized_value
}
