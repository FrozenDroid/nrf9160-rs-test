#![feature(lang_items)]
#![no_main]
#![no_std]

use alloc_cortex_m::CortexMHeap;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[no_mangle]
extern crate tinyrlibc;

#[macro_use]
extern crate cortex_m_rt;

use nrf91::{interrupt, Peripherals};
use core::panic::PanicInfo;
use cortex_m_rt::{entry, ExceptionFrame};
use core::borrow::BorrowMut;
use nrf91::p0_ns::dir::PIN2W;
use nrf91::gpiote0_s::config::PSELR;
use nrf91::clock_ns::lfclkrun::STATUSR;
use nrf91::clock_ns::HFCLKSTAT;
use nrf91::clock_ns::hfclkstat::STATER;
use core::alloc::Layout;
use cortex_m::peripheral::NVIC;
use core::ops::Add;
use core::ptr::{null, null_mut};
use embedded_hal::timer::CountDown;
use core::fmt::Write;
use nrf91::UARTE0_NS;
use nrf9160_dk_bsp::{Board, hal::Timer, prelude::*};
use nrf9160_hal::{Uarte};

static mut dev_per: Option<Peripherals> = None;
static mut core_per: Option<cortex_m::Peripherals> = None;

fn device_peripherals() -> &'static mut Peripherals {
    unsafe {
        dev_per.as_mut().unwrap()
    }
}

fn core_peripherals() -> &'static mut cortex_m::Peripherals {
    unsafe {
        core_per.as_mut().unwrap()
    }
}


static mut CDC_UART: Option<Uarte<UARTE0_NS>> = None;

fn cdc_uart() -> &'static mut Uarte<UARTE0_NS> {
    unsafe {
        CDC_UART.as_mut().unwrap()
    }
}

#[entry]
fn main() -> ! {
    let mut board = Board::take().unwrap();
    let mut timer = Timer::new(board.TIMER0_NS);

    unsafe {
        CDC_UART = Some(board.cdc_uart);
    }

    // Initialize the allocator BEFORE you use it
    let start = cortex_m_rt::heap_start() as usize;
    let size = 1024; // in bytes
    unsafe { ALLOCATOR.init(start, size) }

    board.POWER_NS.tasks_constlat.write(|f| f.tasks_constlat().trigger());

    let mut bsdlib = nrf91_bsdlib::init(&mut board.NVIC).unwrap();

    writeln!(cdc_uart(), "BSD initialized");

    let mut socket = bsdlib.create_socket(nrf91_bsdlib::ProtocolFamily::LTE, nrf91_bsdlib::ProtocolType::None, nrf91_bsdlib::TransportProtocol::AT).unwrap();
    writeln!(cdc_uart(), "AT socket fd: {:?}", socket);

    let mut buffer = [0u8; 128];



    let recv = socket.send_command("AT%XSYSTEMMODE=1,0,0,0", &mut buffer).unwrap();
    writeln!(cdc_uart(), "recv: {:?}", unsafe { core::str::from_utf8_unchecked(&buffer[..recv as usize]) });

    let recv = socket.send_command("AT+CGDCONT=1,\"IP\",\"ibasis.iot\"", &mut buffer).unwrap();
    writeln!(cdc_uart(), "recv: {:?}", unsafe { core::str::from_utf8_unchecked(&buffer[..recv as usize]) });

//    let recv = socket.send_command("AT+CPSMS=", &mut buffer).unwrap();
//    writeln!(cdc_uart(), "recv: {:?}", unsafe { core::str::from_utf8_unchecked(&buffer[..recv as usize]) });
//
//    let recv = socket.send_command("AT+CIND?", &mut buffer).unwrap();
//    writeln!(cdc_uart(), "recv: {:?}", unsafe { core::str::from_utf8_unchecked(&buffer[..recv as usize]) });
//
//    let recv = socket.send_command("AT+CEDRXS=1,4,\"1000\"", &mut buffer).unwrap();
//    writeln!(cdc_uart(), "recv: {:?}", unsafe { core::str::from_utf8_unchecked(&buffer[..recv as usize]) });
//
//    let recv = socket.send_command("AT%XBANDLOCK=2,\"1000000010000001100010001110\"", &mut buffer).unwrap();
//    writeln!(cdc_uart(), "recv: {:?}", unsafe { core::str::from_utf8_unchecked(&buffer[..recv as usize]) });
//
    let recv = socket.send_command("AT+CEREG=5", &mut buffer).unwrap();
    writeln!(cdc_uart(), "recv: {:?}", unsafe { core::str::from_utf8_unchecked(&buffer[..recv as usize]) });

    let recv = socket.send_command("AT+CFUN=1", &mut buffer).unwrap();
    writeln!(cdc_uart(), "recv: {:?}", unsafe { core::str::from_utf8_unchecked(&buffer[..recv as usize]) });

    let recv = socket.wait_for_response("+CEREG:", &mut buffer, None).unwrap();
    writeln!(cdc_uart(), "recv: {:?}", unsafe { core::str::from_utf8_unchecked(&buffer[..recv as usize]) });

    let recv = socket.wait_for_response("+CEREG:", &mut buffer, None).unwrap();
    writeln!(cdc_uart(), "recv: {:?}", unsafe { core::str::from_utf8_unchecked(&buffer[..recv as usize]) });
//
//    let recv = socket.send_command("AT+CESQ", &mut buffer).unwrap();
//    writeln!(cdc_uart(), "recv: {:?}", unsafe { core::str::from_utf8_unchecked(&buffer[..recv as usize]) });

    timer.start(1_000_000u32);

    nb::block!(timer.wait()).unwrap();

    let tcp_socket = bsdlib.create_socket(nrf91_bsdlib::ProtocolFamily::Inet, nrf91_bsdlib::ProtocolType::Stream, nrf91_bsdlib::TransportProtocol::None);
    writeln!(cdc_uart(), "TCP socket fd: {:?}", tcp_socket);

    board.leds.led_1.enable();

    loop {
//        writeln!(cdc_uart(), "hellooo!!!!");
        writeln!(cdc_uart(), "ip: {:?}", bsdlib.resolve_hostname("www.google.com"));
        board.leds.led_1.enable();
        cortex_m::asm::delay(32000000);
        board.leds.led_1.disable();
        cortex_m::asm::delay(32000000);
    }
}

#[panic_handler]
fn panic(panic: &PanicInfo) -> ! {
    unsafe {
        writeln!(cdc_uart(), "{:?}", panic);
    }
    loop { }
}

#[no_mangle]
#[exception]
fn DefaultHandler(irqn: i16) {
    writeln!(cdc_uart(), "{:?}", irqn);
//     custom default handler
}

#[no_mangle]
#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    cortex_m::asm::bkpt();

    loop {}
}

#[no_mangle]
#[interrupt]
fn UARTE0_SPIM0_SPIS0_TWIM0_TWIS0() {
    device_peripherals().UARTE0_NS.events_txdrdy.modify(|r, f| {
        if r.events_txdrdy().is_generated() {
            return f.events_txdrdy().not_generated();
        }
        f
    });
    device_peripherals().UARTE0_NS.events_endtx.modify(|r, f| {
        if r.events_endtx().is_generated() {
           return f.events_endtx().not_generated()
        }
        f
    });
}

#[lang = "oom"]
#[no_mangle]
pub fn rust_oom(layout: Layout) -> ! {
    // ..
    loop { }
}

