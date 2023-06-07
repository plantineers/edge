# Plantbuddy-edge

[![Build Status](https://img.shields.io/github/actions/workflow/status/plantineers/edge/ci.yml?style=flat-square)](Build Status)
[![License](https://img.shields.io/github/license/plantineers/edge?style=flat-square)](License)

## Introduction

Plantbuddy is a solution for monitoring your plants. This is the repository containing the aggregator Firmware for the ESP32C3. Is is responsible for collecting data from the sensors and sending it to the [esp-gateway](https://github.com/plantineers/esp-gateway) via ESP-Now to ensure long range communication. It is written in bare-metal Rust running via [esp-hal](https://github.com/esp-rs/esp-hal) and [embassy](https://github.com/embassy-rs/embassy).

## Getting Started

### Prerequisites

- [Rust](https://rustup.rs/)


### Installation

1. Clone the repository

```sh
git clone https://github.com/plantineers/edge.git
```

2. Install Rust nightly and the required target

```sh
rustup toolchain install nightly
rustup target add riscv32imc-unknown-none-elf
rustup component add rust-src
```

3. Install cargo-espflash

```sh
cargo install espflash --git https://github.com/esp-rs/espflash/
```

4. After plugging in the ESP32C3, allow access to the serial port and flash the firmware

```sh
sudo chmod a+rw /dev/<serial-port> # To permanently allow access, add your user to the dialout or uucp group depending on the distro
cargo run --release --features "<features, comma, seperated>" # See below for available features
```

## Features

For every sensor there is a feature that can be enabled to include it in the build. The following features are available:
| Feature   | Description                                      | Output                                                 |
| --------- | ------------------------------------------------ | ------------------------------------------------------ |
| `tsl2591` | Enable the TSL2591 light sensor                  | `{"light": <lux>}`                                     |
| `hw390`   | Enable the HW390 soil moisture sensor            | `{"soil-moisture": <percentage>}`                      |
| `dht11`   | Enable the DHT11 temperature and humidity sensor | `{"temperature": <celsius>, "humidity": <percentage>}` |
| `dht22`   | Enable the DHT22 temperature and humidity sensor | `{"temperature": <celsius>, "humidity": <percentage>}` |

The dht11 and dht22 features are mutually exclusive. Only one of them can be enabled at a time.
