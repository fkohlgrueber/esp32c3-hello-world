#![no_std]
#![no_main]

use core::fmt::Write;

use esp_hal_common::{gpio::OutputPin, types::OutputSignal};

use esp32c3_hal::{gpio::IO, pac::{self, Peripherals}, prelude::*, Delay, RtcCntl, Serial, Timer};
use panic_halt as _;
use riscv_rt::entry;

const MORSE_DATA: [Item; 13] = init_morse_data();

#[derive(Debug, Clone, Copy)]
struct Item(u32);

impl Item {
    const fn new(duration0: u16, level0: bool, duration1: u16, level1: bool) -> Self {
        Item(
            duration0 as u32
                | ((level0 as u32) << 15)
                | ((duration1 as u32) << 16)
                | ((level1 as u32) << 31),
        )
    }
}

const fn init_morse_data() -> [Item; 13] {
    let dot = Item::new(32767, true, 32767, false);
    let long = Item::new(32767, true, 32767, true);
    let space = Item::new(32767, false, 32767, false);
    let end = Item::new(0, true, 0, false);
    let morse_esp = [
        // E : dot
        dot, 
        space, 
        // S : dot, dot, dot
        dot, 
        dot, 
        dot, 
        space, 
        // P : dot, dash, dash, dot
        dot, 
        long, dot, 
        long, dot, 
        dot, 
        // RMT end marker
        end,
    ];
    morse_esp
}

pub struct RMT0 {
    rmt: pac::RMT,
    data: &'static mut [Item],
}

impl RMT0 {
    pub fn new(mut channel: impl OutputPin<OutputSignal = OutputSignal>, rmt: pac::RMT, system: &pac::SYSTEM, clk_div: u8, idle: Option<bool>) -> Self {
        // activate RMT in SYSTEM registers
        let perip_rst_en0 = &system.perip_rst_en0;
        perip_rst_en0.modify(|_, w| w.rmt_rst().set_bit());
        perip_rst_en0.modify(|_, w| w.rmt_rst().clear_bit());
        let perip_clk_en0 = &system.perip_clk_en0;
        perip_clk_en0.modify(|_, w| w.rmt_clk_en().set_bit());
        perip_rst_en0.modify(|_, w| w.rmt_rst().clear_bit());


        
        let sys_conf = &rmt.sys_conf;
        let ch0conf0 = &rmt.ch0conf0;
    
        // enable memory access
        sys_conf.modify(|_, w| w.apb_fifo_mask().set_bit());

        // setup clock and divider
        sys_conf.modify(|_r, w| w.rmt_sclk_active().clear_bit());
        sys_conf.modify(|_r, w| unsafe {
            w
                .rmt_sclk_sel().bits(1)
                .rmt_sclk_div_num().bits(clk_div)
                .rmt_sclk_div_a().bits(0)
                .rmt_sclk_div_b().bits(0)
                .rmt_sclk_active().set_bit()
        });

        // setup idle level
        match idle {
            Some(lv) => {
                ch0conf0.modify(|_, w| w.idle_out_en_ch0().set_bit().idle_out_lv_ch0().bit(lv));
            }
            None => {
                ch0conf0.modify(|_, w| w.idle_out_en_ch0().clear_bit().idle_out_lv_ch0().clear_bit());
            }
        }

        // route to output
        channel.connect_peripheral_to_output(OutputSignal::RMT_SIG_0);
        
        // calculate data slice for the channel's memory
        let rmt_base_address = esp32c3_hal::pac::RMT::PTR;
        let ch0_data: &'static mut [Item] = unsafe {
            let ch0_data_start_address = (rmt_base_address as *mut u8).add(0x400);
            core::slice::from_raw_parts_mut::<'static>(ch0_data_start_address as *mut Item, 48)
        };

        RMT0 {
            rmt,
            data: ch0_data,
        }
    }

    pub fn start_tx(&self, reset_rd_pos: bool) {
        let ch0conf0 = &self.rmt.ch0conf0;
        if reset_rd_pos {
            // reset
            ch0conf0.modify(|_, w| w.mem_rd_rst_ch0().set_bit());
            ch0conf0.modify(|_, w| w.mem_rd_rst_ch0().clear_bit());
            ch0conf0.modify(|_, w| w.apb_mem_rst_ch0().set_bit());
            ch0conf0.modify(|_, w| w.apb_mem_rst_ch0().clear_bit());
        }
        // start
        ch0conf0.modify(|_, w| w.reg_conf_update_ch0().set_bit());
        ch0conf0.modify(|_, w| w.tx_start_ch0().set_bit());
    }
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();

    // Disable the watchdog timers. For the ESP32-C3, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);
    let mut timer0 = Timer::new(peripherals.TIMG0);
    let mut timer1 = Timer::new(peripherals.TIMG1);

    rtc_cntl.set_super_wdt_enable(false);
    rtc_cntl.set_wdt_enable(false);
    timer0.disable();
    timer1.disable();

    // serial port
    let mut serial0 = Serial::new(peripherals.UART0).unwrap();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let mut delay = Delay::new(peripherals.SYSTIMER);

    let rmt0 = RMT0::new(io.pins.gpio6, peripherals.RMT, &peripherals.SYSTEM, 255, Some(false));

    rmt0.data[..MORSE_DATA.len()].copy_from_slice(&MORSE_DATA);

    loop {
        writeln!(serial0, "Hello world!").unwrap();

        rmt0.start_tx(true);

        delay.delay_ms(10000u32);
    }
}
