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
} // mod main
} // mod lib
pub mod src {
pub mod main {
pub mod blackbox {
pub mod blackbox;
pub mod blackbox_encoding;
pub mod blackbox_io;
} // mod blackbox
pub mod build {
pub mod build_config;
pub mod debug;
pub mod version;
} // mod build
pub mod cms {
pub mod cms;
pub mod cms_menu_blackbox;
pub mod cms_menu_builtin;
pub mod cms_menu_imu;
pub mod cms_menu_ledstrip;
pub mod cms_menu_misc;
pub mod cms_menu_osd;
pub mod cms_menu_power;
pub mod cms_menu_vtx_rtc6705;
pub mod cms_menu_vtx_smartaudio;
pub mod cms_menu_vtx_tramp;
} // mod cms
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
pub mod accgyro_mpu;
pub mod accgyro_mpu6500;
pub mod accgyro_spi_mpu6500;
pub mod gyro_sync;
} // mod accgyro
pub mod adc;
pub mod adc_stm32f30x;
pub mod barometer {
pub mod barometer_bmp085;
pub mod barometer_bmp280;
pub mod barometer_ms5611;
} // mod barometer
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
pub mod camera_control;
pub mod compass {
pub mod compass_ak8975;
pub mod compass_hmc5883l;
pub mod compass_qmc5883l;
} // mod compass
pub mod display;
pub mod display_ug2864hsweg01;
pub mod dma;
pub mod exti;
pub mod flash;
pub mod flash_m25p16;
pub mod flash_w25m;
pub mod io;
pub mod light_led;
pub mod light_ws2811strip;
pub mod light_ws2811strip_stdperiph;
pub mod pinio;
pub mod pwm_esc_detect;
pub mod pwm_output;
pub mod pwm_output_dshot;
pub mod rangefinder {
pub mod rangefinder_hcsr04;
pub mod rangefinder_lidartf;
} // mod rangefinder
pub mod rcc;
pub mod resource;
pub mod rx {
pub mod rx_pwm;
pub mod rx_spi;
pub mod rx_xn297;
} // mod rx
pub mod serial;
pub mod serial_escserial;
pub mod serial_pinconfig;
pub mod serial_softserial;
pub mod serial_uart;
pub mod serial_uart_init;
pub mod serial_uart_pinconfig;
pub mod serial_uart_stm32f30x;
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
pub mod vtx_common;
} // mod drivers
pub mod fc {
pub mod board_info;
pub mod config;
pub mod controlrate_profile;
pub mod fc_core;
pub mod fc_dispatch;
pub mod fc_hardfaults;
pub mod fc_init;
pub mod fc_rc;
pub mod fc_tasks;
pub mod rc_adjustments;
pub mod rc_controls;
pub mod rc_modes;
pub mod runtime_config;
} // mod fc
pub mod flight {
pub mod failsafe;
pub mod gps_rescue;
pub mod imu;
pub mod mixer;
pub mod mixer_tricopter;
pub mod pid;
pub mod position;
pub mod servos;
pub mod servos_tricopter;
} // mod flight
pub mod interface {
pub mod cli;
pub mod msp;
pub mod msp_box;
pub mod settings;
pub mod smartaudio_protocol;
pub mod tramp_protocol;
} // mod interface
pub mod io {
pub mod beeper;
pub mod dashboard;
pub mod displayport_crsf;
pub mod displayport_max7456;
pub mod displayport_msp;
pub mod displayport_oled;
pub mod displayport_srxl;
pub mod flashfs;
pub mod gps;
pub mod ledstrip;
pub mod osd;
pub mod pidaudio;
pub mod piniobox;
pub mod rcdevice;
pub mod rcdevice_cam;
pub mod serial;
pub mod serial_4way;
pub mod serial_4way_avrootloader;
pub mod serial_4way_stk500v2;
pub mod spektrum_rssi;
pub mod spektrum_vtx_control;
pub mod statusindicator;
pub mod transponder_ir;
pub mod usb_cdc_hid;
pub mod usb_msc;
pub mod vtx;
pub mod vtx_control;
pub mod vtx_rtc6705;
pub mod vtx_smartaudio;
pub mod vtx_string;
pub mod vtx_tramp;
} // mod io
pub mod msp {
pub mod msp_serial;
} // mod msp
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
pub mod rx {
pub mod crsf;
pub mod fport;
pub mod ibus;
pub mod jetiexbus;
pub mod msp;
pub mod pwm;
pub mod rx;
pub mod rx_spi;
pub mod sbus;
pub mod sbus_channels;
pub mod spektrum;
pub mod sumd;
pub mod sumh;
pub mod xbus;
} // mod rx
pub mod scheduler {
pub mod scheduler;
} // mod scheduler
pub mod sensors {
pub mod acceleration;
pub mod adcinternal;
pub mod barometer;
pub mod battery;
pub mod boardalignment;
pub mod compass;
pub mod current;
pub mod esc_sensor;
pub mod gyro;
pub mod gyroanalyse;
pub mod initialisation;
pub mod rangefinder;
pub mod voltage;
} // mod sensors
pub mod target {
pub mod SPRACINGF3 {
pub mod target;
} // mod SPRACINGF3
pub mod config_helper;
pub mod system_stm32f30x;
} // mod target
pub mod telemetry {
pub mod crsf;
pub mod frsky_hub;
pub mod hott;
pub mod ibus;
pub mod ibus_shared;
pub mod jetiexbus;
pub mod ltm;
pub mod mavlink;
pub mod msp_shared;
pub mod smartport;
pub mod srxl;
pub mod telemetry;
} // mod telemetry
} // mod main
} // mod src
} // mod src

