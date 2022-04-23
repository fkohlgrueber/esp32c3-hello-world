#![no_std]
#![no_main]

use core::fmt::Write;

use esp_hal_common::{gpio::OutputPin, types::OutputSignal};

use esp32c3_hal::{
    gpio::IO,
    pac::{self, Peripherals},
    prelude::*,
    Delay, RtcCntl, Serial, Timer,
};
use panic_halt as _;
use riscv_rt::entry;

const WS2812_T0H_NS: u32 = 400;
const WS2812_T0L_NS: u32 = 800;
const WS2812_T1H_NS: u32 = 850;
const WS2812_T1L_NS: u32 = 450;
const WS2812_RESET: u32 = 75000;
// const WS2812_TOLERANCE: u32 = 150;

#[derive(Debug, Clone, Copy)]
struct Item(u32);

impl Item {
    const fn new(duration0: u16, level0: bool, duration1: u16, level1: bool) -> Self {
        assert!(duration0 < (1 << 15));
        assert!(duration1 < (1 << 15));
        Item(
            duration0 as u32
                | ((level0 as u32) << 15)
                | ((duration1 as u32) << 16)
                | ((level1 as u32) << 31),
        )
    }
}

pub struct RMT0 {
    rmt: pac::RMT,
    data: &'static mut [Item],
    counter_clk_hz: u32,
}

impl RMT0 {
    pub fn new(
        mut channel: impl OutputPin<OutputSignal = OutputSignal>,
        rmt: pac::RMT,
        system: &pac::SYSTEM,
        clk_div: u8,
        idle: Option<bool>,
    ) -> Self {
        // activate RMT in SYSTEM registers
        let perip_rst_en0 = &system.perip_rst_en0;
        perip_rst_en0.modify(|_, w| w.rmt_rst().set_bit());
        perip_rst_en0.modify(|_, w| w.rmt_rst().clear_bit());
        let perip_clk_en0 = &system.perip_clk_en0;
        perip_clk_en0.modify(|_, w| w.rmt_clk_en().set_bit());
        perip_rst_en0.modify(|_, w| w.rmt_rst().clear_bit());

        // aliases for quick access to the most relevant registers
        let sys_conf = &rmt.sys_conf;
        let ch0conf0 = &rmt.ch0conf0;

        // enable memory access
        sys_conf.modify(|_, w| w.apb_fifo_mask().set_bit());

        // setup clock and divider
        sys_conf.modify(|_r, w| w.rmt_sclk_active().clear_bit());
        sys_conf.modify(|_r, w| unsafe {
            w.rmt_sclk_sel().bits(1) // use APB_CLK
                .rmt_sclk_div_num().bits(0)
                .rmt_sclk_div_a().bits(0)
                .rmt_sclk_div_b().bits(0)
                .rmt_sclk_active().set_bit()
        });
        ch0conf0
            .modify(|_, w| unsafe { w.div_cnt_ch0().bits(clk_div).carrier_en_ch0().clear_bit() });

        // setup idle level
        match idle {
            Some(lv) => {
                ch0conf0.modify(|_, w| w.idle_out_en_ch0().set_bit().idle_out_lv_ch0().bit(lv));
            }
            None => {
                ch0conf0.modify(|_, w| {
                    w.idle_out_en_ch0()
                        .clear_bit()
                        .idle_out_lv_ch0()
                        .clear_bit()
                });
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
            counter_clk_hz: 80_000_000 / clk_div as u32  // TODO: this needs to change when other clocks / clock frequencies are used.
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

    pub fn wait_tx_done(&self) -> nb::Result<(), ()> {
        // TODO: find documentation about the meaning of that state flag. What's
        // written below seems to work, but it'd be better to know for sure
        if self.rmt.ch0status.read().state_ch0().bits() == 0 {
            return Ok(());
        }
        return Err(nb::Error::WouldBlock);
    }

    pub fn counter_clk_hz(&self) -> u32 {
        self.counter_clk_hz
    }
}


pub struct Led {
    rmt: RMT0,
    bit0: Item,
    bit1: Item,
    end: Item,
}

impl Led {
    pub fn new(rmt: RMT0) -> Self {
        let ratio = rmt.counter_clk_hz() as f32 / 1e9;
        let ws2812_t0h_ticks = (ratio * WS2812_T0H_NS as f32) as u16; 
        let ws2812_t0l_ticks = (ratio * WS2812_T0L_NS as f32) as u16; 
        let ws2812_t1h_ticks = (ratio * WS2812_T1H_NS as f32) as u16; 
        let ws2812_t1l_ticks = (ratio * WS2812_T1L_NS as f32) as u16;
        // TODO: Check that accurracy of the timing is within allowed range
        let ws2812_reset = ratio * WS2812_RESET as f32;
        if ws2812_reset >= (1<<15) as f32 {
            // TODO: proper error handling
            panic!("Reset not supported by current clock settings");
        }
        Led {
            rmt, 
            bit1: Item::new(ws2812_t1h_ticks, true, ws2812_t1l_ticks, false),
            bit0: Item::new(ws2812_t0h_ticks, true, ws2812_t0l_ticks, false),
            end: Item::new(ws2812_reset as u16, false, 0, false),
        }
    }

    pub fn write_color(&mut self, color: [u8; 3]) {
        
        const NUM_BITS: usize = 24;
    
        let all_bits = u32::from_be_bytes([0, color[1], color[0], color[2]]);
        for i in 0..NUM_BITS {
            self.rmt.data[i] = if all_bits & (1 << (NUM_BITS - 1 - i)) != 0 {
                self.bit1
            } else {
                self.bit0
            };
        }
        // end marker
        self.rmt.data[NUM_BITS] = self.end;
    
        // start transmission
        self.rmt.start_tx(true);

        // wait until transmission is done
        nb::block!(self.rmt.wait_tx_done()).unwrap();
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

    let rmt0 = RMT0::new(
        io.pins.gpio8,
        peripherals.RMT,
        &peripherals.SYSTEM,
        1,
        Some(false),
    );

    let mut led = Led::new(rmt0);

    const L: u8 = 0;
    const H: u8 = 10;
    let colors = [
        [H, L, L],
        [H, H, L],
        [L, H, L],
        [L, H, H],
        [L, L, H],
        [H, L, H],
    ];

    writeln!(serial0, "Hello world!").unwrap();

    loop {
        for color in colors {
            led.write_color(color);
            nb::block!(led.rmt.wait_tx_done()).unwrap();
            delay.delay_ms(500u32);
        }
    }
}
