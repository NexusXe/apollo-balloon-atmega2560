#![no_std]
#![no_main]
#![feature(core_intrinsics)]
#![feature(abi_avr_interrupt)]
#![feature(asm_experimental_arch)]
#![feature(nonzero_ops)]
#![feature(asm_const)]

use core::intrinsics::*;

extern crate libm;
use panic_halt as _;
use core::arch::asm;
use arduino_hal::{simple_pwm::{self, IntoPwmPin}, delay_ms, prelude::*};

extern crate apollo; // takes up roughly 8.7K of flash
use apollo::{generate_packet, telemetry::BlockStackData, parameters::TOTAL_MESSAGE_LENGTH_BYTES};

mod sensors;


#[macro_export]
macro_rules! transmit_packet {
    ($packet: expr, $txpin: expr, $ledpin: expr, $serial: expr) => {
        $txpin.set_duty(127);
        for mut _byte in $packet.iter() {
            for _bit in 0..8 {
                let mut _w: u8 = 2;
                match _byte & (1 << _bit) { 
                    0 => {
                        $txpin.disable();
                        $ledpin.set_low();
                        _w = 0;
                        //ufmt::uwrite!(&mut $serial, "0").void_unwrap();
                        //delay_ms(1000u16/parameters::BAUDRATE);
                        $txpin.disable();
                        $ledpin.set_low();
                    }
                    _ => {
                        $txpin.enable();
                        $ledpin.set_high();
                        _w = 1;
                        //ufmt::uwrite!(&mut $serial, "1").void_unwrap();
                        //delay_ms(1000u16/parameters::BAUDRATE);
                        $txpin.disable();
                        $ledpin.set_low();
                    }
                }
                ufmt::uwrite!(&mut $serial, "{}", _w).void_unwrap();
            }
        }
        ufmt::uwriteln!(&mut $serial, "").void_unwrap();
    };
}


// #[inline(never)]
fn _no_operation(count: usize) {
    for _ in 0..count {
        unsafe { asm!("nop"); }
    }
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let timer1 = simple_pwm::Timer1Pwm::new(dp.TC1, simple_pwm::Prescaler::Direct);
    let mut txpin = pins.d11.into_output().into_pwm(&timer1);
    let mut ledpin = pins.d13.into_output().downgrade();

    // let ready_indicator = pins.d52.into_floating_input();

    let mut serial = arduino_hal::default_serial!(dp, pins, 115200);
    
    // let i2c = arduino_hal::I2c::new(
    //     dp.TWI,
    //     pins.d20.into_pull_up_input(),
    //     pins.d21.into_pull_up_input(),
    //     50000,
    // );

    delay_ms(1000);
    // wait until the ready indicator pin is pulled low
    // while ready_indicator.is_high() {
    //     delay_ms(10);
    // }


    loop {
        let location = sensors::get_location();
        let mut _packet = generate_packet(BlockStackData { data_arr: [sensors::get_altitude(), sensors::get_voltage(), sensors::get_temperature(), location.0, location.1] });
        for x in _packet.iter() {
            ufmt::uwrite!(&mut serial, "{:x} ", *x).void_unwrap();
        }
        ufmt::uwriteln!(&mut serial, "").void_unwrap();
        transmit_packet!(_packet, txpin, ledpin, serial);
        // let dense_bools = expand_transmit_ident(0b11001101);
        // for i in dense_bools {
        //     ufmt::uwriteln!(&mut serial, "{:?}", i).void_unwrap();
        // }
    }
 }

fn _format_f32(_data: f32) -> (u32, u32) {
    unsafe {
        let _data_whole_number: f32 = floorf32(_data);
        let _data_decimal: f32 = _data - _data_whole_number;

        let _data: u32 = fmul_fast(_data, 10000.0) as u32;
        return (unchecked_div(_data, 10000), unchecked_div(_data, 10000)); // trollface
    }
}
