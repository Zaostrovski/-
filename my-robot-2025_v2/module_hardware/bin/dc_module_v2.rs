#![feature(generic_const_exprs)]
#![feature(inline_const_pat)]
#![allow(incomplete_features)]
extern crate core;

use anyhow::bail;
use std::net::{Ipv4Addr, ToSocketAddrs, UdpSocket};
// use bstr::ByteSlice;
// use chrono::{DateTime, Local, NaiveDateTime, TimeDelta, Timelike, Utc};
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::geometry::{AnchorPoint, Point};
use embedded_graphics::image::{Image, ImageRaw};
use embedded_graphics::mono_font::ascii::{FONT_10X20, FONT_4X6, FONT_5X7, FONT_6X9, FONT_7X13_BOLD, FONT_8X13_BOLD, FONT_9X15_BOLD};
use embedded_graphics::mono_font::iso_8859_14::FONT_6X10;
use embedded_graphics::mono_font::iso_8859_16::FONT_9X18_BOLD;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::raw::{ByteOrder, LittleEndian};
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::Primitive;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::line::StyledPixelsIterator;
use embedded_graphics::primitives::{
    Circle, Line, PrimitiveStyle, Rectangle, StrokeAlignment, StyledDrawable,
};
use embedded_graphics::text::renderer::{TextMetrics, TextRenderer};
use embedded_graphics::text::{Alignment, Text, TextStyleBuilder};
use embedded_graphics::text::{Baseline, TextStyle};
use embedded_graphics::Drawable;
use embedded_hal::delay::DelayNs;
use embedded_layout::layout::linear::{FixedMargin, LinearLayout};
use embedded_layout::prelude::*;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::delay::{Ets, FreeRtos, BLOCK};
use esp_idf_svc::hal::gpio::{
    AnyIOPin, AnyInputPin, AnyOutputPin, IOPin, Input, Level, Output, OutputPin, PinDriver, Pull,
};
use esp_idf_svc::hal::prelude::{FromValueType, Peripherals};
use esp_idf_svc::hal::spi::{Dma, SpiBusDriver, SpiConfig, SpiDriver, SpiDriverConfig};
use esp_idf_svc::hal::{gpio, peripheral, spi};
use esp_idf_svc::io::EspIOError;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};
use esp_idf_svc::timer::EspTimerService;
use esp_idf_svc::wifi::{
    AccessPointInfo, AuthMethod, ClientConfiguration as WifiClientConfig,
    Configuration as WifiConfig,
};

use log::{info, warn, LevelFilter};

use embedded_hal::i2c::NoAcknowledgeSource::Address;
use esp_idf_svc::hal::ledc::{LedcDriver, LedcTimerDriver};
use esp_idf_svc::hal::units::{Hertz, KiloHertz};
use esp_idf_svc::tls::X509;
use std::time::{Duration, SystemTime};
use embedded_hal::pwm::SetDutyCycle;
use esp_idf_svc::hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_svc::hal::uart::UartDriver;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use module_common::*;
use crate::encoder::Encoder;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("init");

    let p = Peripherals::take().unwrap();
    let mut delay = Ets;
    
    let sys_loop = EspSystemEventLoop::take().unwrap();
    let nvs_partition = EspDefaultNvsPartition::take().unwrap();

    // Serial
    let config = esp_idf_svc::hal::uart::config::Config::new().baudrate(Hertz(115_200));
    let tx = p.pins.gpio1;
    let rx = p.pins.gpio3;
    let uart = UartDriver::new(
        p.uart0,
        tx,
        rx,
        Option::<gpio::Gpio0>::None,
        Option::<gpio::Gpio1>::None,
        &config,
    ).unwrap();


    let ledc_timer = LedcTimerDriver::new(
        p.ledc.timer0,
        &esp_idf_svc::hal::ledc::config::TimerConfig::new().frequency(KiloHertz(25).into()),
    )
        .unwrap();

    let mut in1_a = PinDriver::output(p.pins.gpio32).unwrap();
    let mut in2_a = PinDriver::output(p.pins.gpio33).unwrap();
    let mut pwm_a = LedcDriver::new(p.ledc.channel0, &ledc_timer, p.pins.gpio25).unwrap();

    let mut in1_b = PinDriver::output(p.pins.gpio26).unwrap();
    let mut in2_b = PinDriver::output(p.pins.gpio27).unwrap();
    let mut pwm_b = LedcDriver::new(p.ledc.channel1, &ledc_timer, p.pins.gpio14).unwrap();

    let max_duty = pwm_a.get_max_duty();

    let mut self_ip: Ipv4Addr = Ipv4Addr::new(0, 0, 0, 0);
    let mut inc_msg: Option<IncMsgDcModule> = None;

    let tick_per_round = 8246;

    let mut enc_a_1 = p.pins.gpio19;
    let mut enc_a_2 = p.pins.gpio18;
    let encoder_a = Encoder::new(p.pcnt1, &mut enc_a_1, &mut enc_a_2).unwrap();

    let mut enc_b_1 = p.pins.gpio4;
    let mut enc_b_2 = p.pins.gpio23;
    let encoder_b = Encoder::new(p.pcnt0, &mut enc_b_1, &mut enc_b_2).unwrap();

    let mut buff = [0u8; 512];
    let mut buff_read: Vec<u8> = Vec::new();
    let mut is_buff_ready = false;
    let mut iter: u32 = 0;
    loop {

        if let Ok(size) = uart.read(&mut buff, BLOCK) {
            for ch in &buff[0..size] {
                if *ch == b'\n'{
                    is_buff_ready = true;
                    break;
                }else{
                    buff_read.push(*ch);
                }
            }
        }
        if is_buff_ready {
            if let Ok(inc_msg_1) = serde_json::from_slice::<IncMsgDcModule>(buff_read.as_slice()) {
                inc_msg = Some(inc_msg_1);
            }
            let out_msg = OutMsgDcModule::Ok;
            is_buff_ready = false;
            buff_read.clear();
        }

        if let Some(msg) = &inc_msg {
            match msg {

                IncMsgDcModule::SetPwmA { pwm } => {
                    let pwm = pwm.clamp(-100.0, 100.0).round();
                    if pwm > 0.0 {
                        let _ = in1_a.set_high();
                        let _ = in2_a.set_low();
                    }else{
                        let _ = in1_a.set_low();
                        let _ = in2_a.set_high();
                    }
                    let _ = pwm_a.set_duty_cycle_percent(pwm.abs() as u8);
                }
                IncMsgDcModule::SetPwmB { pwm } => {
                    let pwm = pwm.clamp(-100.0, 100.0).round();
                    if pwm > 0.0 {
                        let _ = in1_b.set_high();
                        let _ = in2_b.set_low();
                    }else{
                        let _ = in1_b.set_low();
                        let _ = in2_b.set_high();
                    }
                    let _ = pwm_b.set_duty_cycle_percent(pwm.abs() as u8);
                }
            }
        }
        inc_msg = None;

        if iter >= u32::MAX {
            iter = 0;
        } else {
            iter += 1;
        }
    }
}

mod encoder {
    use std::cmp::min;
    use std::sync::atomic::AtomicI32;
    use std::sync::atomic::Ordering;
    use std::sync::Arc;
    use esp_idf_svc::hal::pcnt::{PcntEvent, PcntEventType};
    use esp_idf_svc::sys::EspError;
    use esp_idf_svc::hal::gpio::AnyInputPin;
    use esp_idf_svc::hal::gpio::InputPin;
    use esp_idf_svc::hal::pcnt::*;
    use esp_idf_svc::hal::peripheral::Peripheral;

    const LOW_LIMIT: i16 = -100;
    const HIGH_LIMIT: i16 = 100;

    pub struct Encoder<'d> {
        unit: PcntDriver<'d>,
        approx_value: Arc<AtomicI32>,
    }

    impl<'d> Encoder<'d> {
        pub fn new<PCNT: Pcnt>(
            pcnt: impl Peripheral<P=PCNT> + 'd,
            pin_a: impl Peripheral<P=impl InputPin> + 'd,
            pin_b: impl Peripheral<P=impl InputPin> + 'd,
        ) -> Result<Self, EspError> {
            let mut unit = PcntDriver::new(
                pcnt,
                Some(pin_a),
                Some(pin_b),
                Option::<AnyInputPin>::None,
                Option::<AnyInputPin>::None,
            )?;
            unit.channel_config(
                PcntChannel::Channel0,
                PinIndex::Pin0,
                PinIndex::Pin1,
                &PcntChannelConfig {
                    lctrl_mode: PcntControlMode::Reverse,
                    hctrl_mode: PcntControlMode::Keep,
                    pos_mode: PcntCountMode::Decrement,
                    neg_mode: PcntCountMode::Increment,
                    counter_h_lim: HIGH_LIMIT,
                    counter_l_lim: LOW_LIMIT,
                },
            )?;
            unit.channel_config(
                PcntChannel::Channel1,
                PinIndex::Pin1,
                PinIndex::Pin0,
                &PcntChannelConfig {
                    lctrl_mode: PcntControlMode::Reverse,
                    hctrl_mode: PcntControlMode::Keep,
                    pos_mode: PcntCountMode::Increment,
                    neg_mode: PcntCountMode::Decrement,
                    counter_h_lim: HIGH_LIMIT,
                    counter_l_lim: LOW_LIMIT,
                },
            )?;

            unit.set_filter_value(min(10 * 80, 1023))?;
            unit.filter_enable()?;

            let approx_value = Arc::new(AtomicI32::new(0));
            // unsafe interrupt code to catch the upper and lower limits from the encoder
            // and track the overflow in `value: Arc<AtomicI32>` - I plan to use this for
            // a wheeled robot's odomerty
            unsafe {
                let approx_value = approx_value.clone();
                unit.subscribe(move |status| {
                    let status = PcntEventType::from_repr_truncated(status);
                    if status.contains(PcntEvent::HighLimit) {
                        approx_value.fetch_add(HIGH_LIMIT as i32, Ordering::SeqCst);
                    }
                    if status.contains(PcntEvent::LowLimit) {
                        approx_value.fetch_add(LOW_LIMIT as i32, Ordering::SeqCst);
                    }
                })?;
            }
            unit.event_enable(PcntEvent::HighLimit)?;
            unit.event_enable(PcntEvent::LowLimit)?;
            unit.counter_pause()?;
            unit.counter_clear()?;
            unit.counter_resume()?;

            Ok(Self { unit, approx_value })
        }

        pub fn get_value(&self) -> Result<i32, EspError> {
            let value =
                self.approx_value.load(Ordering::Relaxed) + self.unit.get_counter_value()? as i32;
            Ok(value)
        }
    }
}

