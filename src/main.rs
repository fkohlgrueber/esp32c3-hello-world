#![no_std]
#![no_main]

use core::{cell::RefCell, fmt::Write};

use esp_hal_common::{gpio::OutputPin, interrupt, pac::UART0, types::OutputSignal, Cpu};

use esp32c3_hal::{gpio::IO, pac::Peripherals, prelude::*, Delay, RtcCntl, Serial, Timer};
use panic_halt as _;
use riscv_rt::entry;

#[derive(Debug, Clone, Copy)]
struct Item(u32);

impl Item {
    fn new(duration0: u16, level0: bool, duration1: u16, level1: bool) -> Self {
        Item(
            duration0 as u32
                | ((level0 as u32) << 15)
                | ((duration1 as u32) << 16)
                | ((level1 as u32) << 31),
        )
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

    // Set GPIO5 as an output, and set its state high initially.
    let mut io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = io.pins.gpio5.into_push_pull_output();

    led.set_high().unwrap();

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let mut delay = Delay::new(peripherals.SYSTIMER);

    let dot = Item::new(32767, true, 32767, false);
    let long = Item::new(32767, true, 32767, true);
    let space = Item::new(32767, false, 32767, false);
    let end = Item::new(0, true, 0, false);
    let morse_esp = [
        // E : dot
        dot, space, // S : dot, dot, dot
        dot, dot, dot, space, // P : dot, dash, dash, dot
        dot, long, dot, long, dot, dot, // RMT end marker
        end,
    ];

    let sys_conf = &peripherals.RMT.sys_conf;
    let ch0conf0 = &peripherals.RMT.ch0conf0;

    // rmt_config
    {
        // rmt_module_enable
        {
            // periph_module_reset
            let perip_rst_en0 = &peripherals.SYSTEM.perip_rst_en0;
            perip_rst_en0.modify(|_, w| w.rmt_rst().set_bit());
            perip_rst_en0.modify(|_, w| w.rmt_rst().clear_bit());

            // periph_module_enable
            let perip_clk_en0 = &peripherals.SYSTEM.perip_clk_en0;
            perip_clk_en0.modify(|_, w| w.rmt_clk_en().set_bit());
            perip_rst_en0.modify(|_, w| w.rmt_rst().clear_bit());
        }

        // rmt_set_pin
        // skipped for now...

        // rmt_internal_config
        {
            // rmt_ll_enable_mem_access
            sys_conf.modify(|_, w| w.apb_fifo_mask().set_bit());

            // rmt_ll_set_counter_clock_src
            sys_conf.modify(|_r, w| w.rmt_sclk_active().clear_bit());
            sys_conf.modify(|_r, w| unsafe {
                w
                    .rmt_sclk_sel().bits(1)
                    .rmt_sclk_div_num().bits(255)
                    .rmt_sclk_div_a().bits(0)
                    .rmt_sclk_div_b().bits(0)
                    .rmt_sclk_active().set_bit()
            });

            // rmt_ll_tx_reset_pointer
            ch0conf0.modify(|_, w| w.mem_rd_rst_ch0().set_bit());
            ch0conf0.modify(|_, w| w.mem_rd_rst_ch0().clear_bit());
            ch0conf0.modify(|_, w| w.apb_mem_rst_ch0().set_bit());
            ch0conf0.modify(|_, w| w.apb_mem_rst_ch0().clear_bit());

            // enable idle at low level
            ch0conf0.modify(|_, w| w.idle_out_en_ch0().set_bit().idle_out_lv_ch0().clear_bit());
        }
    }

    let mut out = io.pins.gpio6.into_push_pull_output();
    // out.enable_output(true);
    out.connect_peripheral_to_output(OutputSignal::RMT_SIG_0);
    // out.enable_output(true);
    {
        // default clock: APB_CLK which is equal to CPU_CLK which is 20 MHz
        // divider: RMT_SCLK_DIV_NUM + 1 + RMT_SCLK_DIV_A/RMT_SCLK_DIV_B
        // -> target 200
        // rmt_sclk is enabled at 100 KHz

        // ch0 memory:  48 x 32-bit block
        // start address: RMT base address + 0x400 + (n - 1) x 48
        let rmt_base_address = esp32c3_hal::pac::RMT::PTR;
        let ch0_data = unsafe {
            let ch0_data_start_address = (rmt_base_address as *mut u8).add(0x400);
            core::slice::from_raw_parts_mut(ch0_data_start_address as *mut u32, 48)
        };

        for i in 0..morse_esp.len() {
            ch0_data[i] = morse_esp[i].0;
        }
    }

    loop {
        let reader = peripherals.RMT.ch0status.read();
        let state = reader.state_ch0();
        writeln!(serial0, "Hello world! State: {:?}", state.bits()).unwrap();

        // write data
        // rmt_fill_tx_items

        // rmt_tx_start
        // reset
        ch0conf0.modify(|_, w| w.mem_rd_rst_ch0().set_bit());
        ch0conf0.modify(|_, w| w.mem_rd_rst_ch0().clear_bit());
        ch0conf0.modify(|_, w| w.apb_mem_rst_ch0().set_bit());
        ch0conf0.modify(|_, w| w.apb_mem_rst_ch0().clear_bit());
        // start
        ch0conf0.modify(|_, w| w.reg_conf_update_ch0().set_bit());
        ch0conf0.modify(|_, w| w.tx_start_ch0().set_bit());

        led.toggle().unwrap();
        delay.delay_ms(10000u32);
    }
}
