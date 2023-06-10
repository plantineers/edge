/// Convert our Iterator into a `[char; 32]` array without a heap allocation
pub(crate) fn convert_to_chars<I, const N: usize>(iter: I) -> [char; N]
where
    I: IntoIterator<Item = char>,
{
    let mut array = ['a'; N];
    for (i, c) in iter.into_iter().enumerate() {
        array[i] = c;
    }
    array
}
/// Convert an array of chars into an array of u8s. Useful for storing the uuid in flash and more
/// efficient than storing the intermediate result in a vector
pub(crate) fn convert_to_u8s<const N: usize>(array: [char; N]) -> [u8; N] {
    let mut u8_array = [0; N];
    for (i, c) in array.iter().enumerate() {
        u8_array[i] = *c as u8;
    }
    u8_array
}
