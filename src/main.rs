#![feature(lang_items)]
#![no_main]
#![no_std]

use alloc_cortex_m::CortexMHeap;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[no_mangle]
extern crate tinyrlibc;


extern "C" {
    fn IPC_IRQHandler();
}

#[no_mangle]
extern "C" fn bsd_os_init() {

}

#[no_mangle]
extern "C" fn bsd_os_errno_set(err: nrfxlib_sys::ctypes::c_int) {
    cortex_m::asm::bkpt();
}

#[no_mangle]
extern "C" fn bsd_irrecoverable_error_handler(error: u32) {
    cortex_m::asm::bkpt();
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
    NVIC::unpend(nrf91::interrupt::EGU1);
}

#[no_mangle]
extern "C" fn bsd_os_application_irq_set() {
    NVIC::pend(nrf91::interrupt::EGU1);
}

#[no_mangle]
extern "C" fn bsd_os_trace_irq_set() {
    NVIC::pend(nrf91::interrupt::EGU2);
}

#[no_mangle]
extern "C" fn bsd_os_trace_irq_clear() {
    NVIC::unpend(nrf91::interrupt::EGU2);
}

#[no_mangle]
extern "C" fn bsd_os_trace_put(p_buffer: *const u8, buf_len: u32) -> i32 {
    0
}

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

use nrf91::UARTE1_NS;

#[entry]
fn main() -> ! {
    // Initialize the allocator BEFORE you use it
    let start = cortex_m_rt::heap_start() as usize;
    let size = 1024; // in bytes
    unsafe { ALLOCATOR.init(start, size) }

    unsafe {
        dev_per = nrf91::Peripherals::take();
        core_per = cortex_m::Peripherals::take();
    }

    device_peripherals().UARTE1_NS.config.write(|f| f.hwfc().enabled().parity().excluded().stop().one());
    device_peripherals().UARTE1_NS.baudrate.write(|f| f.baudrate().baud115200());

    device_peripherals().P0_NS.outset.write(|f| f.pin29().set());
    device_peripherals().P0_NS.dir.write(|f| f.pin29().output().pin2().output());

    device_peripherals().UARTE1_NS.psel.txd.write(|f| unsafe { f.connect().bit(true).pin().bits(29) });
    device_peripherals().UARTE1_NS.psel.rxd.write(|f| unsafe { f.connect().bit(true).pin().bits(28) });
    device_peripherals().UARTE1_NS.psel.rts.write(|f| unsafe { f.connect().bit(true).pin().bits(27) });
    device_peripherals().UARTE1_NS.psel.cts.write(|f| unsafe { f.connect().bit(true).pin().bits(26) });

    device_peripherals().UARTE1_NS.events_endrx.write(|f| f.events_endrx().clear_bit());
    device_peripherals().UARTE1_NS.events_endtx.write(|f| f.events_endtx().clear_bit());
    device_peripherals().UARTE1_NS.events_error.write(|f| f.events_error().clear_bit());
    device_peripherals().UARTE1_NS.events_rxto.write(|f| f.events_rxto().clear_bit());
    device_peripherals().UARTE1_NS.events_txstopped.write(|f| f.events_txstopped().clear_bit());
    device_peripherals().UARTE1_NS.inten.write(|f|
        f.endrx().enabled()
            .endtx().enabled()
            .error().enabled()
            .rxto().enabled()
            .txstopped().enabled()
    );

    device_peripherals().UARTE1_NS.enable.write(|f| f.enable().enabled());

//    device_peripherals().UARTE1_NS.tasks_startrx.write(|f| f.tasks_startrx().set_bit());

//    device_peripherals().P0_NS.dir.modify(|r, f| f.pin2().output());

    unsafe {
        core_peripherals().NVIC.set_priority(nrf91::interrupt::EGU1, 6);
        core_peripherals().NVIC.set_priority(nrf91::interrupt::EGU2, 6);
    }

//    core_peripherals().NVIC.enable(nrf91::interrupt::UARTE0_SPIM0_SPIS0_TWIM0_TWIS0);
    core_peripherals().NVIC.enable(nrf91::interrupt::EGU1);
    core_peripherals().NVIC.enable(nrf91::interrupt::EGU2);
//    core_peripherals().NVIC.enable(nrf91::interrupt::IPC);

    unsafe {
//        let ret = nrfxlib_sys::bsd_init();
//        let sock = nrfxlib_sys::nrf_socket(nrfxlib_sys::NRF_AF_INET as i32, nrfxlib_sys::NRF_SOCK_STREAM as i32, 0);
//
//        let mut a: *mut *mut nrfxlib_sys::nrf_addrinfo = null_mut();
//        let mut local_addr = nrfxlib_sys::nrf_sockaddr_in { sin_addr: nrfxlib_sys::nrf_in_addr { s_addr: 0 }, sin_family: 0, sin_len: 0, sin_port: 0, };
//
//        let addr_ret = nrfxlib_sys::nrf_getaddrinfo("google.com".as_ptr(), null(), null(), a);

//        let mut sock_addr = *core::mem::transmute::<_, *mut nrfxlib_sys::nrf_sockaddr_in>(*(**a).ai_addr);
//        sock_addr.sin_port = (1337 as u16).to_be();
//        sock_addr.sin_len = core::mem::size_of::<nrfxlib_sys::nrf_sockaddr_in>() as u8;

//        local_addr.sin_family = nrfxlib_sys::NRF_AF_INET as i32;


//        cortex_m::asm::bkpt();
    }

    let mut dma_tx = [0u8; 1];

    loop {
        dma_tx[0] = b'\n';
        device_peripherals().UARTE1_NS.txd.ptr.write(|f| unsafe { f.ptr().bits(dma_tx.as_ptr() as u32) });
        device_peripherals().UARTE1_NS.txd.maxcnt.write(|f| unsafe { f.maxcnt().bits(1) });
        device_peripherals().UARTE1_NS.tasks_starttx.write(|f| f.tasks_starttx().trigger());
        device_peripherals().P0_NS.out.modify(|r, w| w.pin2().bit(r.pin2().is_low()));
        cortex_m::asm::delay(64000000);
    }
}

#[panic_handler]
fn panic(panic: &PanicInfo) -> ! {
    unsafe {
        device_peripherals().P0_S.dir.write(|f| f.pin2().output());
        device_peripherals().P0_S.dir.write(|f| f.pin3().output());
        device_peripherals().P0_S.dir.write(|f| f.pin4().output());
        device_peripherals().P0_S.dir.write(|f| f.pin5().output());
        device_peripherals().P0_S.out.write(|f| f.pin2().high());
        device_peripherals().P0_S.out.write(|f| f.pin3().high());
        device_peripherals().P0_S.out.write(|f| f.pin4().high());
        device_peripherals().P0_S.out.write(|f| f.pin5().high());
    }
    loop { }
}

#[no_mangle]
#[exception]
fn DefaultHandler(irqn: i16) {
    irqn;
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

}

#[no_mangle]
#[interrupt]
fn IPC() {
    unsafe {
        IPC_IRQHandler();
//        nrfxlib_sys::bsd_os_application_irq_handler();
    }
}

#[no_mangle]
#[interrupt]
fn EGU1() {
    unsafe {
        nrfxlib_sys::bsd_os_application_irq_handler();
    }
}

#[no_mangle]
#[interrupt]
fn EGU2() {
    unsafe {
//        cortex_m::asm::bkpt();awe
        nrfxlib_sys::bsd_os_trace_irq_handler();
    }
}

#[lang = "oom"]
#[no_mangle]
pub fn rust_oom(layout: Layout) -> ! {
    // ..
    loop { }
}

