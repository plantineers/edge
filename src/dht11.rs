use dht11::{Dht11, Measurement};
use embedded_hal::digital::v2::{InputPin, OutputPin};
use hal::prelude::_embedded_hal_blocking_delay_DelayUs;
use hal::Delay;

pub fn poll_sensor<P, E>(pin: P, delay: &mut Delay) -> Result<Measurement, dht11::Error<E>>
where
    P: InputPin<Error = E> + OutputPin<Error = E>,
{
    let mut dht11 = Dht11::new(pin);
    dht11.perform_measurement(delay)
}
