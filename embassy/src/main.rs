#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(naked_functions)]

use ch32_hal as hal;
use core::future::poll_fn;
use core::task::Poll;
use embassy_executor::Spawner;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::Timer;
use hal::pac;
use hal::{i2c::I2c, peripherals, prelude::Hertz, println};
use qingke::riscv;
use qingke_rt::interrupt;

#[interrupt]
fn I2C2_EV() {
    println!("I2C2_EV");
    pac::I2C2.ctlr2().modify(|w| {
        w.set_itevten(false);
    });
}

#[interrupt]
fn I2C2_ER() {
    println!("I2C2_ER");
    pac::I2C2.ctlr2().modify(|w| {
        w.set_iterren(false);
    });
}

fn enable_interrupts() {
    pac::I2C2.ctlr2().modify(|w| {
        w.set_iterren(true);
        w.set_itevten(true);
    });
}

#[embassy_executor::task]
async fn i2c_task(mut i2c: I2c<'static, peripherals::I2C2, hal::mode::Blocking>) {
    let waker = AtomicWaker::new();
    
    loop {
        // Request BMP device ID
        i2c.blocking_write(0x77, &[0xD0]).unwrap();

        // Send start condition
        pac::I2C2.ctlr1().modify(|reg| {
            reg.set_start(true);
            reg.set_ack(true);
        });

        // Wait for start condition to be generated
        while !pac::I2C2.star1().read().sb() {}

        println!("start condition generated.");

        // Set device address
        pac::I2C2
            .datar()
            .write(|reg| reg.set_datar((0x77 << 1) + 1));

        // Wait for address to be acknowledged
        while !pac::I2C2.star1().read().addr() {}

        println!("address acknowledged.");

        // Clear condition
        pac::I2C2.star2().read();

        enable_interrupts();

        // only one byte
        pac::I2C2.ctlr1().modify(|w| {
            w.set_ack(false);
            w.set_stop(true);
        });

        let id = poll_fn(|cx| {
            waker.register(cx.waker());

            match pac::I2C2.star1().read().rx_ne() {
                true => Poll::Ready(pac::I2C2.datar().read().0),
                false => {
                    enable_interrupts();
                    Poll::Pending
                }
            }
        })
        .await;

        pac::I2C2.ctlr1().modify(|w| {
            w.set_ack(false);
            w.set_stop(true);
        });
        
        println!("id: {id}");

        Timer::after_secs(5).await;
    }
}

#[embassy_executor::task]
async fn long_running() {
    loop {
        println!("Hello from long running task");

        Timer::after_secs(2).await;
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) {
    hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_96MHZ_HSI;
    let p = hal::init(config);
    hal::embassy::init();

    riscv::asm::delay(1000000);

    println!("Embassy initialized");

    let scl = p.PB10;
    let sda = p.PB11;
    let i2c = I2c::new_blocking(p.I2C2, scl, sda, Hertz::hz(400_000), Default::default());

    spawner.spawn(i2c_task(i2c)).unwrap();
    spawner.spawn(long_running()).unwrap();
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("\nPanic: {info}");

    loop {}
}
