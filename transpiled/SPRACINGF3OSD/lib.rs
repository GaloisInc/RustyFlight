#![allow(dead_code)]
#![allow(mutable_transmutes)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]
#![allow(unused_assignments)]
#![allow(unused_mut)]
#![feature(asm)]
#![feature(c_variadic)]
#![feature(const_raw_ptr_to_usize_cast)]
#![feature(const_transmute)]
#![feature(extern_types)]
#![feature(main)]
#![feature(ptr_wrapping_offset_from)]
#![feature(register_tool)]
#![register_tool(c2rust)]


#[macro_use]
extern crate c2rust_bitfields;
extern crate libc;



pub mod src {
pub mod lib {
pub mod main {
pub mod CMSIS {
pub mod DSP {
pub mod Source {
pub mod BasicMathFunctions {
pub mod arm_mult_f32;
} // mod BasicMathFunctions
pub mod CommonTables {
pub mod arm_common_tables;
} // mod CommonTables
pub mod ComplexMathFunctions {
pub mod arm_cmplx_mag_f32;
} // mod ComplexMathFunctions
pub mod StatisticsFunctions {
pub mod arm_max_f32;
} // mod StatisticsFunctions
pub mod TransformFunctions {
pub mod arm_cfft_f32;
pub mod arm_cfft_radix8_f32;
pub mod arm_rfft_fast_f32;
pub mod arm_rfft_fast_init_f32;
} // mod TransformFunctions
} // mod Source
} // mod DSP
} // mod CMSIS
pub mod STM32F3 {
pub mod Drivers {
pub mod STM32F30x_StdPeriph_Driver {
pub mod src {
pub mod stm32f30x_adc;
pub mod stm32f30x_comp;
pub mod stm32f30x_dac;
pub mod stm32f30x_dbgmcu;
pub mod stm32f30x_dma;
pub mod stm32f30x_exti;
pub mod stm32f30x_flash;
pub mod stm32f30x_gpio;
pub mod stm32f30x_hrtim;
pub mod stm32f30x_i2c;
pub mod stm32f30x_iwdg;
pub mod stm32f30x_misc;
pub mod stm32f30x_opamp;
pub mod stm32f30x_pwr;
pub mod stm32f30x_rcc;
pub mod stm32f30x_rtc;
pub mod stm32f30x_spi;
pub mod stm32f30x_syscfg;
pub mod stm32f30x_tim;
pub mod stm32f30x_usart;
pub mod stm32f30x_wwdg;
} // mod src
} // mod STM32F30x_StdPeriph_Driver
} // mod Drivers
} // mod STM32F3
pub mod STM32_USB_FS_Device_Driver {
pub mod src {
pub mod usb_core;
pub mod usb_init;
pub mod usb_int;
pub mod usb_mem;
pub mod usb_regs;
pub mod usb_sil;
} // mod src
} // mod STM32_USB_FS_Device_Driver
} // mod main
} // mod lib
pub mod src {
pub mod main {
pub mod build {
pub mod build_config;
pub mod debug;
pub mod version;
} // mod build
pub mod common {
pub mod bitarray;
pub mod colorconversion;
pub mod crc;
pub mod encoding;
pub mod explog_approx;
pub mod filter;
pub mod gps_conversion;
pub mod huffman;
pub mod huffman_table;
pub mod maths;
pub mod printf;
pub mod streambuf;
pub mod string_light;
pub mod strtol;
pub mod time;
pub mod typeconversion;
} // mod common
pub mod config {
pub mod config_eeprom;
pub mod config_streamer;
pub mod feature;
} // mod config
pub mod drivers {
pub mod accgyro {
pub mod accgyro_fake;
} // mod accgyro
pub mod adc;
pub mod adc_stm32f30x;
pub mod buf_writer;
pub mod bus;
pub mod bus_i2c_busdev;
pub mod bus_i2c_config;
pub mod bus_i2c_soft;
pub mod bus_i2c_stm32f30x;
pub mod bus_spi;
pub mod bus_spi_config;
pub mod bus_spi_pinconfig;
pub mod bus_spi_stdperiph;
pub mod buttons;
pub mod display;
pub mod dma;
pub mod exti;
pub mod io;
pub mod light_led;
pub mod light_ws2811strip_stdperiph;
pub mod max7456;
pub mod pinio;
pub mod pwm_output_dshot;
pub mod rcc;
pub mod resource;
pub mod sdcard;
pub mod sdcard_standard;
pub mod serial;
pub mod serial_pinconfig;
pub mod serial_uart;
pub mod serial_uart_init;
pub mod serial_uart_pinconfig;
pub mod serial_uart_stm32f30x;
pub mod serial_usb_vcp;
pub mod sound_beeper;
pub mod stack_check;
pub mod system;
pub mod system_stm32f30x;
pub mod timer;
pub mod timer_common;
pub mod timer_stm32f30x;
pub mod transponder_ir_arcitimer;
pub mod transponder_ir_erlt;
pub mod transponder_ir_ilap;
pub mod transponder_ir_io_stdperiph;
pub mod usb_io;
} // mod drivers
pub mod fc {
pub mod board_info;
pub mod config;
pub mod fc_dispatch;
pub mod fc_hardfaults;
pub mod fc_tasks;
pub mod runtime_config;
} // mod fc
pub mod interface {
pub mod msp;
pub mod msp_box;
pub mod smartaudio_protocol;
pub mod tramp_protocol;
} // mod interface
pub mod io {
pub mod asyncfatfs {
pub mod asyncfatfs;
pub mod fat_standard;
} // mod asyncfatfs
pub mod beeper;
pub mod displayport_max7456;
pub mod osd_slave;
pub mod piniobox;
pub mod serial;
pub mod statusindicator;
pub mod transponder_ir;
pub mod usb_cdc_hid;
pub mod usb_msc;
} // mod io
pub mod msp {
pub mod msp_serial;
} // mod msp
pub mod osd_slave {
pub mod osd_slave_init;
} // mod osd_slave
pub mod pg {
pub mod adc;
pub mod beeper;
pub mod beeper_dev;
pub mod board;
pub mod bus_i2c;
pub mod bus_spi;
pub mod dashboard;
pub mod flash;
pub mod max7456;
pub mod pg;
pub mod pinio;
pub mod piniobox;
pub mod rcdevice;
pub mod rx;
pub mod rx_pwm;
pub mod rx_spi;
pub mod sdcard;
pub mod sdio;
pub mod timerio;
pub mod usb;
pub mod vcd;
} // mod pg
pub mod scheduler {
pub mod scheduler;
} // mod scheduler
pub mod sensors {
pub mod adcinternal;
pub mod battery;
pub mod current;
pub mod voltage;
} // mod sensors
pub mod target {
pub mod SPRACINGF3OSD {
pub mod config;
pub mod target;
} // mod SPRACINGF3OSD
pub mod config_helper;
pub mod system_stm32f30x;
} // mod target
pub mod vcp {
pub mod hw_config;
pub mod stm32_it;
pub mod usb_desc;
pub mod usb_endp;
pub mod usb_istr;
pub mod usb_prop;
pub mod usb_pwr;
} // mod vcp
} // mod main
} // mod src
} // mod src

