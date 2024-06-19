#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::println;
use qingke::riscv;

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
    let config = hal::Config::default();
    hal::init(config);
    hal::embassy::init();

    riscv::asm::delay(1000000);

    println!("Embassy initialized");

    spawner.spawn(long_running()).unwrap();
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("\nPanic: {info}");

    loop {}
}
