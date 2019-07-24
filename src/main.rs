#![no_main]
#![no_std]


#[no_mangle]
extern crate tinyrlibc;


#[no_mangle]
extern "C" fn bsd_os_init() {

}

#[no_mangle]
extern "C" fn bsd_os_errno_set(err: nrfxlib_sys::ctypes::c_int) {

}
//
#[no_mangle]
extern "C" fn bsd_os_timedwait(context: u32, p_timeout: *mut i32) -> i32 {
    0
}

#[no_mangle]
extern crate nrfxlib_sys;
//
#[no_mangle]
extern "C" fn bsd_os_application_irq_clear() {

}

#[no_mangle]
extern "C" fn bsd_os_application_irq_set() {

}

#[no_mangle]
extern "C" fn bsd_os_trace_irq_set() {

}

#[no_mangle]
extern "C" fn bsd_os_trace_irq_clear() {

}

#[no_mangle]
extern "C" fn bsd_os_trace_put(p_buffer: *const u8, buf_len: u32) -> i32 {
    0
}

//#[no_mangle]
//fn bsd_os_application_irq_handler() {
//}

//#[no_mangle]
//fn bsd_os_trace_irq_handler() {
//
//}

#[macro_use]
extern crate cortex_m_rt;

use nrf91::{interrupt, Peripherals};
use core::panic::PanicInfo;
use cortex_m_rt::entry;
use core::borrow::BorrowMut;
use nrf91::p0_ns::dir::PIN2W;
use nrf91::gpiote0_s::config::PSELR;
use nrf91::clock_ns::lfclkrun::STATUSR;
use nrf91::clock_ns::HFCLKSTAT;
use nrf91::clock_ns::hfclkstat::STATER;

static mut per: Option<Peripherals> = None;

fn peripherals() -> &'static mut Peripherals {
    unsafe {
        per.as_mut().unwrap()
    }
}

#[entry]
fn main() -> ! {
    unsafe {
        per = nrf91::Peripherals::take();
    }

    let mut core_per = cortex_m::Peripherals::take().expect("could not take core peripherals");

//    peripherals().RTC0_S.events_tick.write(|f| f.events_tick().)
//    peripherals().RTC0_S.evten.write(|f| f.tick().enabled());
//    peripherals().RTC0_S.prescaler.write(|f| unsafe { f.prescaler().bits(0) });
//    peripherals().RTC0_S.intenset.write(|f| f.tick().set());
//    peripherals().RTC0_S.tasks_start.write(|f| f.tasks_start().trigger());

    peripherals().GPIOTE0_S.config[0].reset();
//    peripherals().GPIOTE0_S.config[1].reset();
//    peripherals().GPIOTE0_S.config[2].reset();
//    peripherals().GPIOTE0_S.config[3].reset();
    peripherals().GPIOTE0_S.config[0].write(|f| unsafe { f.mode().task().psel().bits(2).polarity().toggle() });
//    peripherals().GPIOTE0_S.config[1].write(|f| unsafe { f.mode().task().psel().bits(3).polarity().toggle() });
//    peripherals().GPIOTE0_S.config[2].write(|f| unsafe { f.mode().task().psel().bits(4).polarity().toggle() });
//    peripherals().GPIOTE0_S.config[3].write(|f| unsafe { f.mode().task().psel().bits(5).polarity().toggle() });

    core_per.NVIC.enable(nrf91::interrupt::RTC0);
    core_per.NVIC.enable(nrf91::interrupt::EGU0);
    core_per.NVIC.enable(nrf91::interrupt::CLOCK_POWER);
//    peripherals().CLOCK_S.inten.write(|f| f.hfclkstarted().enabled().lfclkstarted().enabled());

//    peripherals().CLOCK_S.tasks_hfclkstop.write(|f| f.tasks_hfclkstop().trigger());
//    peripherals().CLOCK_S.tasks_lfclkstop.write(|f| f.tasks_lfclkstop().trigger());

//    peripherals().CLOCK_S.tasks_lfclkstart.write(|f| f.tasks_lfclkstart().trigger());

    loop {
        peripherals().GPIOTE0_S.tasks_out[0].write(|f| f.tasks_out().trigger());
        cortex_m::asm::delay(32000000);
    }
}

#[panic_handler]
fn panic(panic: &PanicInfo) -> ! {
    unsafe {
        peripherals().P0_S.dir.write(|f| f.pin2().output());
        peripherals().P0_S.dir.write(|f| f.pin3().output());
        peripherals().P0_S.dir.write(|f| f.pin4().output());
        peripherals().P0_S.dir.write(|f| f.pin5().output());
        peripherals().P0_S.out.write(|f| f.pin2().high());
        peripherals().P0_S.out.write(|f| f.pin3().high());
        peripherals().P0_S.out.write(|f| f.pin4().high());
        peripherals().P0_S.out.write(|f| f.pin5().high());
    }
    loop { }
}

#[interrupt]
fn EGU1() {

}

#[interrupt]
fn EGU2() {

}

#[interrupt]
fn RTC0() {
    peripherals().GPIOTE0_S.tasks_out[2].write(|f| f.tasks_out().trigger());
}

#[interrupt]
fn CLOCK_POWER() {
    let r = peripherals().CLOCK_S.events_hfclkstarted.read();
    if r.events_hfclkstarted().is_generated() {
        peripherals().GPIOTE0_S.tasks_set[0].write(|f| f.tasks_set().trigger());
    }
    let r = peripherals().CLOCK_S.events_lfclkstarted.read();
    if r.events_lfclkstarted().is_generated() {
        peripherals().GPIOTE0_S.tasks_set[1].write(|f| f.tasks_set().trigger());
    }
}

