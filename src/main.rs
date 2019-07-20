#![no_main]
#![no_std]

#[no_mangle]
extern fn bsd_os_init() {

}

#[no_mangle]
extern fn bsd_os_errno_set(err: nrfxlib_sys::ctypes::c_int) {

}

#[no_mangle]
extern fn bsd_os_timedwait(context: u32, p_timeout: *mut i32) -> i32 {
    0
}

#[no_mangle]
extern crate tinyrlibc;

extern crate nrfxlib_sys;

use core::panic::PanicInfo;
use cortex_m_rt::entry;
use core::borrow::BorrowMut;

#[entry]
fn main() -> ! {
    let per = nrf91::Peripherals::take().expect("could not take peripherals");
    let core_per = cortex_m::Peripherals::take().expect("could not take core peripherals");

    per.P0_S.dir.write(|f| f.pin2().output());

    unsafe {
        nrfxlib_sys::bsd_init();
    }
//    per.IPC_S.

//    per.P0_NS.
    loop {
        per.P0_S.out.modify(|r, w| w.pin2().bit(r.pin2().is_low()));
        cortex_m::asm::delay(64000000);
    }
}

#[panic_handler]
fn panic(panic: &PanicInfo) -> ! {
    loop { }
}