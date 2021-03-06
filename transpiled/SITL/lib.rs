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




pub mod src {
pub mod lib {
pub mod main {
pub mod dyad {
pub mod dyad;
} // mod dyad
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
pub mod accgyro_fake;
pub mod gyro_sync;
} // mod accgyro
pub mod barometer {
pub mod barometer_fake;
} // mod barometer
pub mod buf_writer;
pub mod bus;
pub mod bus_i2c_busdev;
pub mod bus_i2c_soft;
pub mod buttons;
pub mod camera_control;
pub mod compass {
pub mod compass_fake;
} // mod compass
pub mod display;
pub mod exti;
pub mod io;
pub mod light_led;
pub mod light_ws2811strip;
pub mod pinio;
pub mod pwm_esc_detect;
pub mod rangefinder {
pub mod rangefinder_hcsr04;
pub mod rangefinder_lidartf;
} // mod rangefinder
pub mod resource;
pub mod rx {
pub mod rx_pwm;
pub mod rx_spi;
} // mod rx
pub mod serial;
pub mod serial_softserial;
pub mod serial_tcp;
pub mod sound_beeper;
pub mod stack_check;
pub mod timer_common;
pub mod transponder_ir_arcitimer;
pub mod transponder_ir_erlt;
pub mod transponder_ir_ilap;
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
pub mod displayport_srxl;
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
pub mod SITL {
pub mod target;
pub mod udplink;
} // mod SITL
pub mod config_helper;
} // mod target
pub mod telemetry {
pub mod frsky_hub;
pub mod hott;
pub mod ibus;
pub mod ibus_shared;
pub mod jetiexbus;
pub mod ltm;
pub mod mavlink;
pub mod msp_shared;
pub mod smartport;
pub mod telemetry;
} // mod telemetry
} // mod main
} // mod src
} // mod src

