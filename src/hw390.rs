use hal::adc::{AdcConfig, Attenuation, ADC, ADC1};
use hal::analog::SarAdcExt;
use hal::peripherals::Peripherals;
use hal::system::{SystemExt, SystemParts};
use hal::IO;

fn get_humidity(peripherals: Peripherals, io: IO, mut system: SystemParts) {
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
}
