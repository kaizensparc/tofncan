//! Blinks an LED
//!
//! This assumes that an LED is connected to pb0

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_halt as _;

use rtt_target::{rprintln, rtt_init_print};

use cortex_m_rt::entry;
use gd32e103_hal::{delay::Delay, i2c, pac, prelude::*};
use sht3x::Sht3x;
use veml7700::Veml7700;

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw rcu and flash devices and convert them into the corresponding HAL
    // structs.
    let mut flash = dp.FMC.constrain();
    let mut rcu = dp.RCU.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcu
        .cfgr
        .use_hxtal(8.mhz())
        .hclk(72.mhz())
        .sysclk(72.mhz())
        .pclk1(24.mhz())
        .freeze(&mut flash.ws);

    rtt_init_print!();
    rprintln!("Clock conf:");
    rprintln!("- HCLK: {:?}", clocks.hclk());
    rprintln!("- PCLK1: {:?}", clocks.pclk1());
    rprintln!("- PCLK2: {:?}", clocks.pclk2());
    rprintln!("- SYSCLK: {:?}", clocks.sysclk());
    rprintln!("- ADCCLK: {:?}", clocks.adcclk());

    // Acquire the GPIOB peripheral
    let mut gpiob = dp.GPIOB.split(&mut rcu.apb2);

    // Configure gpio B pin 0 as a push-pull output. The `crl` register is passed to the function
    // in order to configure the port. For pins 8+, crh should be passed instead.
    let mut led = gpiob.pb0.into_push_pull_output(&mut gpiob.crl);
    // Configure the syst timer to trigger an update every second
    let mut delay = Delay::new(cp.SYST, &clocks);
    rprintln!("ahb: {}", clocks.hclk().0);

    let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
    let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);

    cp.DWT.enable_cycle_counter();
    let i2c_dev = i2c::BlockingI2c::i2c0(
        dp.I2C0,
        scl,
        sda,
        i2c::Mode::standard(40.khz()),
        &clocks,
        &mut rcu.apb1,
        1000,
        3,
        1000,
        1000,
    );

    let bus = shared_bus::BusManagerSimple::new(i2c_dev);

    let mut sht30 = Sht3x::new(bus.acquire_i2c(), sht3x::Address::Low);
    let status = sht30.status(&mut delay).unwrap();
    rprintln!("SHT30: Current status {:?}", status);
    let m = sht30
        .measure(
            sht3x::ClockStretch::Enabled,
            sht3x::Repeatability::Medium,
            &mut delay,
        )
        .unwrap();
    rprintln!("SHT30: {}C, {}%H", m.temperature, m.humidity);

    let mut veml = Veml7700::new(bus.acquire_i2c());
    veml.enable().unwrap();
    let white = veml.read_white().unwrap();
    let lux = veml.read_lux().unwrap();
    rprintln!("VEML7700: {} white, {} lux", white, lux);

    let mut tof = vl53l1::Device::default();
    let mut tof_i2c = bus.acquire_i2c();
    vl53l1::software_reset(&mut tof, &mut tof_i2c, &mut delay).unwrap();

    vl53l1::data_init(&mut tof, &mut tof_i2c).unwrap();
    vl53l1::static_init(&mut tof).unwrap();

    let roi = vl53l1::UserRoi {
        bot_right_x: 10,
        bot_right_y: 6,
        top_left_x: 6,
        top_left_y: 10,
    };
    vl53l1::set_user_roi(&mut tof, roi).unwrap();
    vl53l1::set_measurement_timing_budget_micro_seconds(&mut tof, 50_000).unwrap();
    vl53l1::set_inter_measurement_period_milli_seconds(&mut tof, 60).unwrap();

    vl53l1::start_measurement(&mut tof, &mut tof_i2c).unwrap();
    vl53l1::wait_measurement_data_ready(&mut tof, &mut tof_i2c, &mut delay).unwrap();

    // Wait for the timer to trigger an update and change the state of the LED
    loop {
        delay.delay_ms(500u32);
        led.set_high();
        delay.delay_ms(500u32);
        led.set_low();
    }
}
