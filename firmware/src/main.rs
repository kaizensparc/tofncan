//! Welcome to ToFnCAN firmware documentation!
//!
//! # CAN bus communication
//!
//! ## Frame IDs
//!
//! Note: all CAN addresses below are offset by [`FRAME_ID_BASE`] unless for firmware
//! which is at address 1 (maximum priority) [TODO]
//!
//! The bus addressing for this node is the following:
//!
//! - 0 (emitter): Welcome message, sent at boot only
//! - 1 (receiver): system configuration registers
//! - 2 (receiver): TOF general configuration registers
//! - 3 (receiver): TOF timing registers
//! - 4 (emitter): SHT30 results
//! - 5 (emitter): VEML results
//! - 6 (emitter): VL53 rates readings
//! - 7 (emitter): VL53 effective SPAD, sigma and range readings
//! - 8 (emitter): VL53 stream count and range status
//!
//! ## Messages
//!
//! All messages are send in network order (Big Endian)
//!
//! ### Welcome message
//!
//! - Boot reason (4 bytes)
//!
//! ### System configuration registers
//!
//! - restart (1 byte)
//! - automatic retransmit (1 byte)
//! - diffusion interval (ms, 4 bytes)
//!
//! ### ToF general configuration registers
//!
//! - [distance mode](`vl53l1::DistanceMode`) (1 byte)
//! - roi (bot_right_x, bot_right_y, top_left_x, top_left_y, 4 bytes total)
//!
//! ### ToF timing registers
//!
//! - measurement timing budget (us, 4 bytes)
//! - inter measurement period (ms, 4 bytes)
//!
//! ### SHT30 Results
//!
//! - Status (2 bytes)
//! - Humidity (2 bytes)
//! - Temperature (4 bytes)
//!
//! ### VEML Results
//!
//! - White (2 bytes)
//! - Lux (4 bytes)
//!
//! ### VL53
//!
//! #### Rates readings
//!
//! - Signal rate (MCPS 16.16 fix point value)
//! - Ambient rate (MCPS 16.16 fix point value)
//!
//! #### Effective SPAD, sigma and range readings
//!
//! - Effective SPAD count (2 bytes, real value is /256)
//! - Sigma value in mm (16.16 fix point value)
//! - Range distance in mm (2 bytes)
//!
//! #### Stream count and range status
//!
//! - Stream count (1 byte)
//! - [Range status](`vl53l1::RangeStatus`) (1 byte)
//!

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_reset as _;

use rtt_target::{rprint, rprintln, rtt_init_print};

use cortex_m_rt::entry;
use gd32e103_hal::{can, delay::Delay, i2c, pac, prelude::*, timer, watchdog};
use sht3x::Sht3x;
use veml7700::Veml7700;

/// Using 11bit CAN addressing, please do not go after 0x7FF - 5
pub const FRAME_ID_BASE: u16 = 0x20;

#[repr(u16)]
pub enum FrameIds {
    Welcome = FRAME_ID_BASE,
    SystemConfiguration = FRAME_ID_BASE + 1,
    ToFGeneralConfiguration = FRAME_ID_BASE + 2,
    ToFTiming = FRAME_ID_BASE + 3,
    Sht30 = FRAME_ID_BASE + 4,
    Veml = FRAME_ID_BASE + 5,
    ToFRates = FRAME_ID_BASE + 6,
    ToFSPAD = FRAME_ID_BASE + 7,
    ToFStream = FRAME_ID_BASE + 8,
}

fn send_welcome<I>(can_bus: &mut bxcan::Can<I>, boot_reason: u32)
where
    I: bxcan::Instance,
{
    rprint!("Sending welcome... ");
    let frame = bxcan::Frame::new_data(
        bxcan::StandardId::new(FrameIds::Welcome as u16).unwrap(),
        boot_reason.to_be_bytes(),
    );
    can_bus.transmit(&frame).unwrap();
    while !can_bus.is_transmitter_idle() {}
    rprintln!("Done.");
}

fn parse_can_msg<I, T>(
    frame: &bxcan::Frame,
    can_bus: &mut bxcan::Can<I>,
    cdt: &mut T,
    tof: &mut vl53l1::Device,
) where
    I: bxcan::Instance,
    T: embedded_hal::timer::CountDown,
    T::Time: From<gd32e103_hal::time::Hertz>,
{
    let id = {
        match frame.id() {
            bxcan::Id::Standard(x) => x.as_raw(),
            bxcan::Id::Extended(_) => return,
        }
    };

    let data = match frame.data() {
        Some(x) => x,
        None => return,
    };

    if id == FrameIds::SystemConfiguration as u16 {
        if data.len() != 6 {
            return;
        }
        if data[0] != 0 {
            cortex_m::peripheral::SCB::sys_reset();
        }
        can_bus
            .modify_config()
            .set_automatic_retransmit(data[1] != 0)
            .enable();
        let t_ms = u32::from_be_bytes(data[2..5].try_into().unwrap());
        cdt.start((1000 / t_ms).hz());
    } else if id == FrameIds::ToFGeneralConfiguration as u16 {
        if data.len() != 5 {
            return;
        }
        let distance_mode = match data[0] {
            1 => vl53l1::DistanceMode::Short,
            2 => vl53l1::DistanceMode::Medium,
            3 => vl53l1::DistanceMode::Long,
            _ => return,
        };
        vl53l1::set_distance_mode(tof, distance_mode).unwrap();
        let roi = vl53l1::UserRoi {
            bot_right_x: data[1],
            bot_right_y: data[2],
            top_left_x: data[3],
            top_left_y: data[4],
        };
        vl53l1::set_user_roi(tof, roi).unwrap();
    } else if id == FrameIds::ToFTiming as u16 {
        if data.len() != 8 {
            return;
        }
        vl53l1::set_measurement_timing_budget_micro_seconds(
            tof,
            u32::from_be_bytes(data[..3].try_into().unwrap()),
        )
        .unwrap();
        vl53l1::set_inter_measurement_period_milli_seconds(
            tof,
            u32::from_be_bytes(data[4..7].try_into().unwrap()),
        )
        .unwrap();
    }
}

fn wait_for_system_config<I, T>(
    can_bus: &mut bxcan::Can<I>,
    wdt: &mut watchdog::FreeWatchdog,
    delay: &mut Delay,
    cdt: &mut T,
    tof: &mut vl53l1::Device,
) where
    I: bxcan::Instance,
    T: embedded_hal::timer::CountDown,
    T::Time: From<gd32e103_hal::time::Hertz>,
{
    rprint!("Waiting for configuration... ");
    let mut iter = 0u32;
    loop {
        wdt.feed();
        match can_bus.receive() {
            Ok(rx) => {
                parse_can_msg(&rx, can_bus, cdt, tof);
                if rx.id()
                    == bxcan::Id::Standard(
                        bxcan::StandardId::new(FrameIds::SystemConfiguration as u16).unwrap(),
                    )
                {
                    rprint!("{:?}", rx.data());
                    break;
                }
            }
            Err(nb::Error::Other(_)) => rprint!("Overrun error"),
            Err(nb::Error::WouldBlock) => {}
        };
        iter = iter.saturating_add(1);
        if iter > 1000 {
            rprint!("No configuration received, using defaults...");
            break;
        }
        delay.delay_ms(5u32);
    }
    rprintln!("Done.");
}

fn read_sht30<I2C, E>(sht30: &mut sht3x::Sht3x<I2C>, delay: &mut Delay) -> bxcan::Frame
where
    I2C: embedded_hal::blocking::i2c::Read<Error = E>
        + embedded_hal::blocking::i2c::Write<Error = E>
        + embedded_hal::blocking::i2c::WriteRead<Error = E>,
    E: core::fmt::Debug,
{
    let sht30_status = sht30.status(delay).unwrap().bits();
    let sht30_readings = sht30
        .measure(
            sht3x::ClockStretch::Enabled,
            sht3x::Repeatability::Medium,
            delay,
        )
        .unwrap();

    bxcan::Frame::new_data(
        bxcan::StandardId::new(FrameIds::Sht30 as u16).unwrap(),
        [
            sht30_status.to_be_bytes()[0],
            sht30_status.to_be_bytes()[1],
            sht30_readings.humidity.to_be_bytes()[0],
            sht30_readings.humidity.to_be_bytes()[1],
            sht30_readings.temperature.to_be_bytes()[0],
            sht30_readings.temperature.to_be_bytes()[1],
            sht30_readings.temperature.to_be_bytes()[2],
            sht30_readings.temperature.to_be_bytes()[3],
        ],
    )
}

fn read_veml<I2C, E>(veml: &mut Veml7700<I2C>) -> bxcan::Frame
where
    I2C: embedded_hal::blocking::i2c::Read<Error = E>
        + embedded_hal::blocking::i2c::Write<Error = E>
        + embedded_hal::blocking::i2c::WriteRead<Error = E>,
    E: core::fmt::Debug,
{
    let veml_white = veml.read_white().unwrap();
    let veml_lux = veml.read_lux().unwrap();
    bxcan::Frame::new_data(
        bxcan::StandardId::new(FrameIds::Veml as u16).unwrap(),
        [
            veml_white.to_be_bytes()[0],
            veml_white.to_be_bytes()[1],
            veml_lux.to_be_bytes()[0],
            veml_lux.to_be_bytes()[1],
            veml_lux.to_be_bytes()[2],
            veml_lux.to_be_bytes()[3],
        ],
    )
}

fn read_send_vl53<I, I2C, E>(
    tof: &mut vl53l1::Device,
    tof_i2c: &mut I2C,
    delay: &mut Delay,
    can_bus: &mut bxcan::Can<I>,
) where
    I: bxcan::Instance,
    I2C: embedded_hal::blocking::i2c::Read<Error = E>
        + embedded_hal::blocking::i2c::Write<Error = E>
        + embedded_hal::blocking::i2c::WriteRead<Error = E>,
    E: core::fmt::Debug,
{
    let vl53_readings = {
        vl53l1::wait_measurement_data_ready(tof, tof_i2c, delay).unwrap();
        let r = vl53l1::get_ranging_measurement_data(tof, tof_i2c).unwrap();
        vl53l1::clear_interrupt_and_start_measurement(tof, tof_i2c, delay).unwrap();
        r
    };
    let frame = bxcan::Frame::new_data(
        bxcan::StandardId::new(FrameIds::ToFRates as u16).unwrap(),
        [
            vl53_readings.signal_rate_rtn_mega_cps.to_be_bytes()[0],
            vl53_readings.signal_rate_rtn_mega_cps.to_be_bytes()[1],
            vl53_readings.signal_rate_rtn_mega_cps.to_be_bytes()[2],
            vl53_readings.signal_rate_rtn_mega_cps.to_be_bytes()[3],
            vl53_readings.ambient_rate_rtn_mega_cps.to_be_bytes()[0],
            vl53_readings.ambient_rate_rtn_mega_cps.to_be_bytes()[1],
            vl53_readings.ambient_rate_rtn_mega_cps.to_be_bytes()[2],
            vl53_readings.ambient_rate_rtn_mega_cps.to_be_bytes()[3],
        ],
    );
    while !can_bus.is_transmitter_idle() {}
    can_bus.transmit(&frame).unwrap();

    let frame = bxcan::Frame::new_data(
        bxcan::StandardId::new(FrameIds::ToFSPAD as u16).unwrap(),
        [
            vl53_readings.effective_spad_rtn_count.to_be_bytes()[0],
            vl53_readings.effective_spad_rtn_count.to_be_bytes()[1],
            vl53_readings.sigma_milli_meter.to_be_bytes()[0],
            vl53_readings.sigma_milli_meter.to_be_bytes()[1],
            vl53_readings.sigma_milli_meter.to_be_bytes()[2],
            vl53_readings.sigma_milli_meter.to_be_bytes()[3],
            vl53_readings.range_milli_meter.to_be_bytes()[0],
            vl53_readings.range_milli_meter.to_be_bytes()[1],
        ],
    );
    while !can_bus.is_transmitter_idle() {}
    can_bus.transmit(&frame).unwrap();

    let frame = bxcan::Frame::new_data(
        bxcan::StandardId::new(FrameIds::ToFStream as u16).unwrap(),
        [vl53_readings.stream_count, vl53_readings.range_status as u8],
    );
    while !can_bus.is_transmitter_idle() {}
    can_bus.transmit(&frame).unwrap();
}

fn check_incoming<I, T>(can_bus: &mut bxcan::Can<I>, cdt: &mut T, tof: &mut vl53l1::Device)
where
    I: bxcan::Instance,
    T: embedded_hal::timer::CountDown,
    T::Time: From<gd32e103_hal::time::Hertz>,
{
    loop {
        match can_bus.rx0().receive() {
            Ok(rx) => parse_can_msg(&rx, can_bus, cdt, tof),
            Err(nb::Error::Other(_)) => rprint!("Overrun error"),
            Err(nb::Error::WouldBlock) => break,
        };
    }
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("TOFnCAN loading...");

    // Get access to the core peripherals from the cortex-m crate
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let mut dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw rcu and flash devices and convert them into the corresponding HAL
    // structs.
    let mut flash = dp.FMC.constrain();
    let mut rcu = dp.RCU.constrain();

    // Get reset reason
    let reason = rcu.rstsck.reason();
    rprintln!("Reset reason: {}", reason);

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies.
    rprint!("Initializing clocks... ");
    let clocks = rcu
        .cfgr
        .use_hxtal(8.mhz())
        .sysclk(8.mhz())
        .hclk(8.mhz())
        .pclk1(8.mhz())
        .freeze(&mut flash.ws);
    rprintln!("Done.");

    // Initialize the watchdog timer
    let mut wdt = watchdog::FreeWatchdog::new(dp.FWDGT);
    wdt.start(5000.ms());
    wdt.feed();

    // Acquire the GPIO peripherals
    rprint!("Initializing internal peripherals... ");
    let mut gpioa = dp.GPIOA.split(&mut rcu.apb2);
    let mut gpiob = dp.GPIOB.split(&mut rcu.apb2);

    // Configure gpio B pin 0 as a push-pull output. The `crl` register is passed to the function
    // in order to configure the port. For pins 8+, crh should be passed instead.
    // let mut led = gpiob.pb0.into_push_pull_output(&mut gpiob.crl);

    // Configure the syst timer
    let mut delay = Delay::new(cp.SYST, &clocks);

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
    rprintln!("Done.");

    let bus = shared_bus::BusManagerSimple::new(i2c_dev);

    rprint!("Initializing SHT30 sensor... ");
    wdt.feed();
    let mut sht30 = Sht3x::new(bus.acquire_i2c(), sht3x::Address::Low);
    let _ = sht30.reset(&mut delay).unwrap();
    let _ = sht30.status(&mut delay).unwrap();
    rprintln!("Done.");

    rprint!("Initializing VEML7700 sensor... ");
    wdt.feed();
    let mut veml = Veml7700::new(bus.acquire_i2c());
    veml.enable().unwrap();
    rprintln!("Done.");

    rprint!("Initializing VL53L1 sensor... ");
    wdt.feed();
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
    vl53l1::set_distance_mode(&mut tof, vl53l1::DistanceMode::Medium).unwrap();
    vl53l1::set_measurement_timing_budget_micro_seconds(&mut tof, 50_000).unwrap();
    vl53l1::set_inter_measurement_period_milli_seconds(&mut tof, 60).unwrap();

    vl53l1::clear_interrupt_and_start_measurement(&mut tof, &mut tof_i2c, &mut delay).unwrap();
    vl53l1::wait_measurement_data_ready(&mut tof, &mut tof_i2c, &mut delay).unwrap();
    vl53l1::get_ranging_measurement_data(&mut tof, &mut tof_i2c).unwrap();
    rprintln!("Done.");

    rprint!("Initializing CAN bus... ");
    wdt.feed();
    let pa12 = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);
    let pa11 = gpioa.pa11.into_floating_input(&mut gpioa.crh);
    let can_bus = can::Can::new(dp.CAN0, &mut rcu.apb1);
    can_bus.assign_pins((pa12, pa11), &mut dp.AFIO);
    // Bit timing is defined via http://www.bittiming.can-wiki.info/?CLK=8&ctype=bxCAN&calc=1
    let mut can_bus = bxcan::Can::builder(can_bus)
        .set_automatic_retransmit(true)
        .set_bit_timing(0x001c0001)
        .enable();
    can_bus
        .modify_filters()
        .clear()
        .enable_bank(
            0,
            bxcan::Fifo::Fifo0,
            bxcan::filter::Mask32::frames_with_std_id(
                bxcan::StandardId::new(1u16).unwrap(),
                bxcan::StandardId::MAX,
            ),
        )
        .enable_bank(
            1,
            bxcan::Fifo::Fifo0,
            bxcan::filter::Mask32::frames_with_std_id(
                bxcan::StandardId::new(FrameIds::SystemConfiguration as u16).unwrap(),
                bxcan::StandardId::MAX,
            ),
        )
        .enable_bank(
            2,
            bxcan::Fifo::Fifo0,
            bxcan::filter::Mask32::frames_with_std_id(
                bxcan::StandardId::new(FrameIds::ToFGeneralConfiguration as u16).unwrap(),
                bxcan::StandardId::MAX,
            ),
        )
        .enable_bank(
            3,
            bxcan::Fifo::Fifo0,
            bxcan::filter::Mask32::frames_with_std_id(
                bxcan::StandardId::new(FrameIds::ToFTiming as u16).unwrap(),
                bxcan::StandardId::MAX,
            ),
        );
    rprintln!("Done.");

    // Initialize the timer used for diffusion intervals
    let mut cdt = timer::Timer::timer0(dp.TIMER0, &clocks, &mut rcu.apb2).start_count_down(1.hz());

    // Send the welcome message and wait for a start/timeout before looping
    wdt.feed();
    send_welcome(&mut can_bus, reason);
    wait_for_system_config(&mut can_bus, &mut wdt, &mut delay, &mut cdt, &mut tof);

    loop {
        if cdt.wait() == Err(nb::Error::WouldBlock) {
            check_incoming(&mut can_bus, &mut cdt, &mut tof);
            wdt.feed();
            continue;
        }

        rprint!("Sending readings... ");
        wdt.feed();

        rprint!("SHT30 ");
        let frame = read_sht30(&mut sht30, &mut delay);
        while !can_bus.is_transmitter_idle() {}
        can_bus.transmit(&frame).unwrap();

        rprint!("| VEML ");
        let frame = read_veml(&mut veml);
        while !can_bus.is_transmitter_idle() {}
        can_bus.transmit(&frame).unwrap();

        rprint!("| VL53 ");
        read_send_vl53(&mut tof, &mut tof_i2c, &mut delay, &mut can_bus);

        // Wait until the end of transmission before continuing
        while !can_bus.is_transmitter_idle() {}

        wdt.feed();
        rprintln!("Done.");
    }
}
