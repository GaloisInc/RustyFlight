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
pub mod STM32F7 {
pub mod Drivers {
pub mod STM32F7xx_HAL_Driver {
pub mod Src {
pub mod stm32f7xx_hal;
pub mod stm32f7xx_hal_adc;
pub mod stm32f7xx_hal_adc_ex;
pub mod stm32f7xx_hal_cortex;
pub mod stm32f7xx_hal_dac;
pub mod stm32f7xx_hal_dac_ex;
pub mod stm32f7xx_hal_dma;
pub mod stm32f7xx_hal_dma_ex;
pub mod stm32f7xx_hal_flash;
pub mod stm32f7xx_hal_flash_ex;
pub mod stm32f7xx_hal_gpio;
pub mod stm32f7xx_hal_i2c;
pub mod stm32f7xx_hal_i2c_ex;
pub mod stm32f7xx_hal_pcd;
pub mod stm32f7xx_hal_pcd_ex;
pub mod stm32f7xx_hal_pwr;
pub mod stm32f7xx_hal_pwr_ex;
pub mod stm32f7xx_hal_rcc;
pub mod stm32f7xx_hal_rcc_ex;
pub mod stm32f7xx_hal_spi;
pub mod stm32f7xx_hal_tim;
pub mod stm32f7xx_hal_tim_ex;
pub mod stm32f7xx_hal_uart;
pub mod stm32f7xx_hal_usart;
pub mod stm32f7xx_ll_dma;
pub mod stm32f7xx_ll_dma2d;
pub mod stm32f7xx_ll_gpio;
pub mod stm32f7xx_ll_rcc;
pub mod stm32f7xx_ll_spi;
pub mod stm32f7xx_ll_tim;
pub mod stm32f7xx_ll_usb;
pub mod stm32f7xx_ll_utils;
} // mod Src
} // mod STM32F7xx_HAL_Driver
} // mod Drivers
pub mod Middlewares {
pub mod ST {
pub mod STM32_USB_Device_Library {
pub mod Class {
pub mod CDC {
pub mod Src {
pub mod usbd_cdc;
} // mod Src
} // mod CDC
pub mod CDC_HID {
pub mod Src {
pub mod usbd_cdc_hid;
} // mod Src
} // mod CDC_HID
pub mod HID {
pub mod Src {
pub mod usbd_hid;
} // mod Src
} // mod HID
pub mod MSC {
pub mod Src {
pub mod usbd_msc;
pub mod usbd_msc_bot;
pub mod usbd_msc_data;
pub mod usbd_msc_scsi;
} // mod Src
} // mod MSC
} // mod Class
pub mod Core {
pub mod Src {
pub mod usbd_core;
pub mod usbd_ctlreq;
pub mod usbd_ioreq;
} // mod Src
} // mod Core
} // mod STM32_USB_Device_Library
} // mod ST
} // mod Middlewares
} // mod STM32F7
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
pub mod accgyro_spi_mpu6000;
pub mod accgyro_spi_mpu6500;
pub mod gyro_sync;
} // mod accgyro
pub mod adc;
pub mod adc_stm32f7xx;
pub mod audio_stm32f7xx;
pub mod barometer {
pub mod barometer_bmp280;
pub mod barometer_ms5611;
} // mod barometer
pub mod buf_writer;
pub mod bus;
pub mod bus_i2c_busdev;
pub mod bus_i2c_config;
pub mod bus_i2c_hal;
pub mod bus_i2c_soft;
pub mod bus_spi;
pub mod bus_spi_config;
pub mod bus_spi_ll;
pub mod bus_spi_pinconfig;
pub mod buttons;
pub mod camera_control;
pub mod compass {
pub mod compass_hmc5883l;
pub mod compass_qmc5883l;
} // mod compass
pub mod display;
pub mod display_ug2864hsweg01;
pub mod dma_stm32f7xx;
pub mod exti;
pub mod flash;
pub mod flash_m25p16;
pub mod flash_w25m;
pub mod io;
pub mod light_led;
pub mod light_ws2811strip;
pub mod light_ws2811strip_hal;
pub mod max7456;
pub mod pinio;
pub mod pwm_esc_detect;
pub mod pwm_output;
pub mod pwm_output_dshot_hal;
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
pub mod serial_uart_hal;
pub mod serial_uart_pinconfig;
pub mod serial_uart_stm32f7xx;
pub mod serial_usb_vcp;
pub mod sound_beeper;
pub mod stack_check;
pub mod system;
pub mod system_stm32f7xx;
pub mod timer_common;
pub mod timer_hal;
pub mod timer_stm32f7xx;
pub mod transponder_ir_arcitimer;
pub mod transponder_ir_erlt;
pub mod transponder_ir_ilap;
pub mod transponder_ir_io_hal;
pub mod usb_io;
pub mod usb_msc_f7xx;
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
pub mod msc {
pub mod emfat;
pub mod emfat_file;
pub mod usbd_storage;
pub mod usbd_storage_emfat;
} // mod msc
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
pub mod OMNIBUSF7 {
pub mod config;
pub mod target;
} // mod OMNIBUSF7
pub mod config_helper;
pub mod system_stm32f7xx;
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
pub mod vcp_hal {
pub mod usbd_cdc_interface;
pub mod usbd_conf;
pub mod usbd_desc;
} // mod vcp_hal
} // mod main
} // mod src
} // mod src

