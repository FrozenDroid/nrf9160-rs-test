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

use nrf91::interrupt;
use core::panic::PanicInfo;
use cortex_m_rt::entry;
use core::borrow::BorrowMut;
use nrf91::p0_ns::dir::PIN2W;
use nrf91::gpiote0_s::config::PSELR;

#[entry]
fn main() -> ! {
    let per = nrf91::Peripherals::take().expect("could not take peripherals");
    let mut core_per = cortex_m::Peripherals::take().expect("could not take core peripherals");

//    per.P0_S.dir.write(|f| f.pin2().output());

//    per.DPPIC_S.chen.


    per.DPPIC_S.chen.write(|f| f.ch0().enabled());
    per.DPPIC_S.chenset.write(|f| f.ch0().set());

    per.GPIOTE0_S.config[0].write(|f| unsafe { f.mode().task().psel().bits(3).polarity().lo_to_hi().outinit().low() });

    unsafe {
//        core_per.NVIC.enable(nrf91::interrupt::GPIOTE0);
//        core_per.NVIC.enable(nrf91::interrupt::EGU1);
//        core_per.NVIC.enable(nrf91::interrupt::EGU2);
//        nrfxlib_sys::bsd_init();
    }

    loop {
        per.GPIOTE0_S.tasks_out[0].write(|f| f.tasks_out().trigger());
//        per.GPIOTE0_S.tasks_set[0].write(|f| f.tasks_set().trigger());
//        per.P0_S.out.modify(|r, w| w.pin2().bit(r.pin2().is_low()));
        cortex_m::asm::delay(64000000);
    }
}

#[panic_handler]
fn panic(panic: &PanicInfo) -> ! {
    unsafe {
        let per = nrf91::Peripherals::steal();
        per.P0_S.dir.write(|f| f.pin2().output());
        per.P0_S.dir.write(|f| f.pin3().output());
        per.P0_S.dir.write(|f| f.pin4().output());
        per.P0_S.dir.write(|f| f.pin5().output());
        per.P0_S.out.write(|f| f.pin2().high());
        per.P0_S.out.write(|f| f.pin3().high());
        per.P0_S.out.write(|f| f.pin4().high());
        per.P0_S.out.write(|f| f.pin5().high());
    }
    loop { }
}

#[interrupt]
fn EGU1() {

}

#[interrupt]
fn EGU2() {

}
