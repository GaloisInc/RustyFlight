use core;
use libc;
extern "C" {
    pub type _IO_wide_data;
    pub type _IO_codecvt;
    pub type _IO_marker;
    pub type gyroDev_s;
    /*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
    pub type accDev_s;
    #[no_mangle]
    static mut stderr: *mut FILE;
    #[no_mangle]
    fn fclose(__stream: *mut FILE) -> libc::c_int;
    #[no_mangle]
    fn fopen(_: *const libc::c_char, _: *const libc::c_char) -> *mut FILE;
    #[no_mangle]
    fn fprintf(_: *mut FILE, _: *const libc::c_char, _: ...) -> libc::c_int;
    #[no_mangle]
    fn printf(_: *const libc::c_char, _: ...) -> libc::c_int;
    #[no_mangle]
    fn fread(_: *mut libc::c_void, _: libc::c_ulong, _: libc::c_ulong,
             _: *mut FILE) -> libc::c_ulong;
    #[no_mangle]
    fn fwrite(_: *const libc::c_void, _: libc::c_ulong, _: libc::c_ulong,
              _: *mut FILE) -> libc::c_ulong;
    #[no_mangle]
    fn fseek(__stream: *mut FILE, __off: libc::c_long, __whence: libc::c_int)
     -> libc::c_int;
    #[no_mangle]
    fn ftell(__stream: *mut FILE) -> libc::c_long;
    #[no_mangle]
    fn rewind(__stream: *mut FILE);
    #[no_mangle]
    fn exit(_: libc::c_int) -> !;
    #[no_mangle]
    fn strerror(_: libc::c_int) -> *mut libc::c_char;
    #[no_mangle]
    fn __errno_location() -> *mut libc::c_int;
    #[no_mangle]
    fn nanosleep(__requested_time: *const timespec,
                 __remaining: *mut timespec) -> libc::c_int;
    #[no_mangle]
    fn clock_gettime(__clock_id: clockid_t, __tp: *mut timespec)
     -> libc::c_int;
    #[no_mangle]
    fn pthread_create(__newthread: *mut pthread_t,
                      __attr: *const pthread_attr_t,
                      __start_routine:
                          Option<unsafe extern "C" fn(_: *mut libc::c_void)
                                     -> *mut libc::c_void>,
                      __arg: *mut libc::c_void) -> libc::c_int;
    #[no_mangle]
    fn pthread_join(__th: pthread_t, __thread_return: *mut *mut libc::c_void)
     -> libc::c_int;
    #[no_mangle]
    fn pthread_mutex_init(__mutex: *mut pthread_mutex_t,
                          __mutexattr: *const pthread_mutexattr_t)
     -> libc::c_int;
    #[no_mangle]
    fn pthread_mutex_trylock(__mutex: *mut pthread_mutex_t) -> libc::c_int;
    #[no_mangle]
    fn pthread_mutex_unlock(__mutex: *mut pthread_mutex_t) -> libc::c_int;
    #[no_mangle]
    fn dyad_init();
    #[no_mangle]
    fn dyad_update();
    #[no_mangle]
    fn dyad_shutdown();
    #[no_mangle]
    fn dyad_setTickInterval(seconds: libc::c_double);
    #[no_mangle]
    fn dyad_setUpdateTimeout(seconds: libc::c_double);
    #[no_mangle]
    static mut fakeAccDev: *mut accDev_s;
    #[no_mangle]
    fn fakeAccSet(acc: *mut accDev_s, x: int16_t, y: int16_t, z: int16_t);
    #[no_mangle]
    static mut fakeGyroDev: *mut gyroDev_s;
    #[no_mangle]
    fn fakeGyroSet(gyro: *mut gyroDev_s, x: int16_t, y: int16_t, z: int16_t);
    // in deg
    #[no_mangle]
    fn imuSetAttitudeQuat(w: libc::c_float, x: libc::c_float,
                          y: libc::c_float, z: libc::c_float);
    #[no_mangle]
    fn feature(mask: uint32_t) -> bool;
    #[no_mangle]
    fn rescheduleTask(taskId: cfTaskId_e, newPeriodMicros: uint32_t);
    /* *
 * Copyright (c) 2017 cs8425
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the MIT license.
 */
    #[no_mangle]
    fn udpInit(link: *mut udpLink_t, addr: *const libc::c_char,
               port: libc::c_int, isServer: bool) -> libc::c_int;
    #[no_mangle]
    fn udpRecv(link: *mut udpLink_t, data: *mut libc::c_void, size: size_t,
               timeout_ms: uint32_t) -> libc::c_int;
    #[no_mangle]
    fn udpSend(link: *mut udpLink_t, data: *const libc::c_void, size: size_t)
     -> libc::c_int;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type __uint64_t = libc::c_ulong;
pub type __off_t = libc::c_long;
pub type __off64_t = libc::c_long;
pub type __time_t = libc::c_long;
pub type __clockid_t = libc::c_int;
pub type __syscall_slong_t = libc::c_long;
pub type int8_t = __int8_t;
pub type int16_t = __int16_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
pub type uintptr_t = libc::c_ulong;
pub type size_t = libc::c_ulong;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct _IO_FILE {
    pub _flags: libc::c_int,
    pub _IO_read_ptr: *mut libc::c_char,
    pub _IO_read_end: *mut libc::c_char,
    pub _IO_read_base: *mut libc::c_char,
    pub _IO_write_base: *mut libc::c_char,
    pub _IO_write_ptr: *mut libc::c_char,
    pub _IO_write_end: *mut libc::c_char,
    pub _IO_buf_base: *mut libc::c_char,
    pub _IO_buf_end: *mut libc::c_char,
    pub _IO_save_base: *mut libc::c_char,
    pub _IO_backup_base: *mut libc::c_char,
    pub _IO_save_end: *mut libc::c_char,
    pub _markers: *mut _IO_marker,
    pub _chain: *mut _IO_FILE,
    pub _fileno: libc::c_int,
    pub _flags2: libc::c_int,
    pub _old_offset: __off_t,
    pub _cur_column: libc::c_ushort,
    pub _vtable_offset: libc::c_schar,
    pub _shortbuf: [libc::c_char; 1],
    pub _lock: *mut libc::c_void,
    pub _offset: __off64_t,
    pub _codecvt: *mut _IO_codecvt,
    pub _wide_data: *mut _IO_wide_data,
    pub _freeres_list: *mut _IO_FILE,
    pub _freeres_buf: *mut libc::c_void,
    pub __pad5: size_t,
    pub _mode: libc::c_int,
    pub _unused2: [libc::c_char; 20],
}
pub type _IO_lock_t = ();
pub type FILE = _IO_FILE;
pub type clockid_t = __clockid_t;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct timespec {
    pub tv_sec: __time_t,
    pub tv_nsec: __syscall_slong_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct __pthread_internal_list {
    pub __prev: *mut __pthread_internal_list,
    pub __next: *mut __pthread_internal_list,
}
pub type __pthread_list_t = __pthread_internal_list;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct __pthread_mutex_s {
    pub __lock: libc::c_int,
    pub __count: libc::c_uint,
    pub __owner: libc::c_int,
    pub __nusers: libc::c_uint,
    pub __kind: libc::c_int,
    pub __spins: libc::c_short,
    pub __elision: libc::c_short,
    pub __list: __pthread_list_t,
}
pub type pthread_t = libc::c_ulong;
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union pthread_mutexattr_t {
    pub __size: [libc::c_char; 4],
    pub __align: libc::c_int,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union pthread_attr_t {
    pub __size: [libc::c_char; 56],
    pub __align: libc::c_long,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union pthread_mutex_t {
    pub __data: __pthread_mutex_s,
    pub __size: [libc::c_char; 40],
    pub __align: libc::c_long,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct TIM_TypeDef {
    pub test: *mut libc::c_void,
}
pub type FLASH_Status = libc::c_uint;
pub const FLASH_TIMEOUT: FLASH_Status = 5;
pub const FLASH_COMPLETE: FLASH_Status = 4;
pub const FLASH_ERROR_WRP: FLASH_Status = 3;
pub const FLASH_ERROR_PG: FLASH_Status = 2;
pub const FLASH_BUSY: FLASH_Status = 1;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct fdm_packet {
    pub timestamp: libc::c_double,
    pub imu_angular_velocity_rpy: [libc::c_double; 3],
    pub imu_linear_acceleration_xyz: [libc::c_double; 3],
    pub imu_orientation_quat: [libc::c_double; 4],
    pub velocity_xyz: [libc::c_double; 3],
    pub position_xyz: [libc::c_double; 3],
    // meters, NED from origin
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct servo_packet {
    pub motor_speed: [libc::c_float; 4],
    // normal: [0.0, 1.0], 3D: [-1.0, 1.0]
}
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
pub type ioTag_t = uint8_t;
// packet tag to specify IO pin
pub type IO_t = *mut libc::c_void;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct serialPinConfig_s {
    pub ioTagTx: [ioTag_t; 10],
    pub ioTagRx: [ioTag_t; 10],
    pub ioTagInverter: [ioTag_t; 10],
}
pub type serialPinConfig_t = serialPinConfig_s;
pub type sa_family_t = libc::c_ushort;
pub type in_port_t = uint16_t;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct sockaddr_in {
    pub sin_family: sa_family_t,
    pub sin_port: in_port_t,
    pub sin_addr: in_addr,
    pub sin_zero: [libc::c_uchar; 8],
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct in_addr {
    pub s_addr: in_addr_t,
}
pub type in_addr_t = uint32_t;
pub type cfTaskId_e = libc::c_uint;
pub const TASK_SELF: cfTaskId_e = 19;
pub const TASK_NONE: cfTaskId_e = 18;
pub const TASK_COUNT: cfTaskId_e = 18;
pub const TASK_PINIOBOX: cfTaskId_e = 17;
pub const TASK_RCDEVICE: cfTaskId_e = 16;
pub const TASK_TELEMETRY: cfTaskId_e = 15;
pub const TASK_ALTITUDE: cfTaskId_e = 14;
pub const TASK_BARO: cfTaskId_e = 13;
pub const TASK_COMPASS: cfTaskId_e = 12;
pub const TASK_GPS: cfTaskId_e = 11;
pub const TASK_BATTERY_ALERTS: cfTaskId_e = 10;
pub const TASK_BATTERY_CURRENT: cfTaskId_e = 9;
pub const TASK_BATTERY_VOLTAGE: cfTaskId_e = 8;
pub const TASK_DISPATCH: cfTaskId_e = 7;
pub const TASK_SERIAL: cfTaskId_e = 6;
pub const TASK_RX: cfTaskId_e = 5;
pub const TASK_ATTITUDE: cfTaskId_e = 4;
pub const TASK_ACCEL: cfTaskId_e = 3;
pub const TASK_GYROPID: cfTaskId_e = 2;
pub const TASK_MAIN: cfTaskId_e = 1;
pub const TASK_SYSTEM: cfTaskId_e = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct udpLink_t {
    pub fd: libc::c_int,
    pub si: sockaddr_in,
    pub recv: sockaddr_in,
    pub port: libc::c_int,
    pub addr: *mut libc::c_char,
    pub isServer: bool,
}
pub type failureMode_e = libc::c_uint;
pub const FAILURE_GYRO_INIT_FAILED: failureMode_e = 6;
pub const FAILURE_FLASH_WRITE_FAILED: failureMode_e = 5;
pub const FAILURE_INVALID_EEPROM_CONTENTS: failureMode_e = 4;
pub const FAILURE_ACC_INCOMPATIBLE: failureMode_e = 3;
pub const FAILURE_ACC_INIT: failureMode_e = 2;
pub const FAILURE_MISSING_ACC: failureMode_e = 1;
pub const FAILURE_DEVELOPER: failureMode_e = 0;
// 16 bit on both 103 and 303, just register access must be 32bit sometimes (use timCCR_t)
pub type timCCR_t = uint32_t;
pub type timerUsageFlag_e = libc::c_uint;
pub const TIM_USE_BEEPER: timerUsageFlag_e = 64;
pub const TIM_USE_TRANSPONDER: timerUsageFlag_e = 32;
pub const TIM_USE_LED: timerUsageFlag_e = 16;
pub const TIM_USE_SERVO: timerUsageFlag_e = 8;
pub const TIM_USE_MOTOR: timerUsageFlag_e = 4;
pub const TIM_USE_PWM: timerUsageFlag_e = 2;
pub const TIM_USE_PPM: timerUsageFlag_e = 1;
pub const TIM_USE_NONE: timerUsageFlag_e = 0;
pub const TIM_USE_ANY: timerUsageFlag_e = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct timerHardware_s {
    pub tim: *mut TIM_TypeDef,
    pub tag: ioTag_t,
    pub channel: uint8_t,
    pub usageFlags: timerUsageFlag_e,
    pub output: uint8_t,
}
pub type timerHardware_t = timerHardware_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct timerChannel_t {
    pub ccr: *mut timCCR_t,
    pub tim: *mut TIM_TypeDef,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct pwmOutputPort_t {
    pub channel: timerChannel_t,
    pub pulseScale: libc::c_float,
    pub pulseOffset: libc::c_float,
    pub forceOverflow: bool,
    pub enabled: bool,
    pub io: IO_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct motorDevConfig_s {
    pub motorPwmRate: uint16_t,
    pub motorPwmProtocol: uint8_t,
    pub motorPwmInversion: uint8_t,
    pub useUnsyncedPwm: uint8_t,
    pub useBurstDshot: uint8_t,
    pub ioTags: [ioTag_t; 8],
}
pub type motorDevConfig_t = motorDevConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct servoDevConfig_s {
    pub servoCenterPulse: uint16_t,
    pub servoPwmRate: uint16_t,
    pub ioTags: [ioTag_t; 8],
}
pub type servoDevConfig_t = servoDevConfig_s;
pub const FEATURE_3D: C2RustUnnamed = 4096;
pub type C2RustUnnamed = libc::c_uint;
pub const FEATURE_DYNAMIC_FILTER: C2RustUnnamed = 536870912;
pub const FEATURE_ANTI_GRAVITY: C2RustUnnamed = 268435456;
pub const FEATURE_ESC_SENSOR: C2RustUnnamed = 134217728;
pub const FEATURE_SOFTSPI: C2RustUnnamed = 67108864;
pub const FEATURE_RX_SPI: C2RustUnnamed = 33554432;
pub const FEATURE_AIRMODE: C2RustUnnamed = 4194304;
pub const FEATURE_TRANSPONDER: C2RustUnnamed = 2097152;
pub const FEATURE_CHANNEL_FORWARDING: C2RustUnnamed = 1048576;
pub const FEATURE_OSD: C2RustUnnamed = 262144;
pub const FEATURE_DASHBOARD: C2RustUnnamed = 131072;
pub const FEATURE_LED_STRIP: C2RustUnnamed = 65536;
pub const FEATURE_RSSI_ADC: C2RustUnnamed = 32768;
pub const FEATURE_RX_MSP: C2RustUnnamed = 16384;
pub const FEATURE_RX_PARALLEL_PWM: C2RustUnnamed = 8192;
pub const FEATURE_TELEMETRY: C2RustUnnamed = 1024;
pub const FEATURE_RANGEFINDER: C2RustUnnamed = 512;
pub const FEATURE_GPS: C2RustUnnamed = 128;
pub const FEATURE_SOFTSERIAL: C2RustUnnamed = 64;
pub const FEATURE_SERVO_TILT: C2RustUnnamed = 32;
pub const FEATURE_MOTOR_STOP: C2RustUnnamed = 16;
pub const FEATURE_RX_SERIAL: C2RustUnnamed = 8;
pub const FEATURE_INFLIGHT_ACC_CAL: C2RustUnnamed = 4;
pub const FEATURE_RX_PPM: C2RustUnnamed = 1;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct rxConfig_s {
    pub rcmap: [uint8_t; 8],
    pub serialrx_provider: uint8_t,
    pub serialrx_inverted: uint8_t,
    pub halfDuplex: uint8_t,
    pub spektrum_bind_pin_override_ioTag: ioTag_t,
    pub spektrum_bind_plug_ioTag: ioTag_t,
    pub spektrum_sat_bind: uint8_t,
    pub spektrum_sat_bind_autoreset: uint8_t,
    pub rssi_channel: uint8_t,
    pub rssi_scale: uint8_t,
    pub rssi_invert: uint8_t,
    pub midrc: uint16_t,
    pub mincheck: uint16_t,
    pub maxcheck: uint16_t,
    pub rcInterpolation: uint8_t,
    pub rcInterpolationChannels: uint8_t,
    pub rcInterpolationInterval: uint8_t,
    pub fpvCamAngleDegrees: uint8_t,
    pub airModeActivateThreshold: uint8_t,
    pub rx_min_usec: uint16_t,
    pub rx_max_usec: uint16_t,
    pub max_aux_channel: uint8_t,
    pub rssi_src_frame_errors: uint8_t,
    pub rssi_offset: int8_t,
    pub rc_smoothing_type: uint8_t,
    pub rc_smoothing_input_cutoff: uint8_t,
    pub rc_smoothing_derivative_cutoff: uint8_t,
    pub rc_smoothing_debug_axis: uint8_t,
    pub rc_smoothing_input_type: uint8_t,
    pub rc_smoothing_derivative_type: uint8_t,
}
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
pub type rxConfig_t = rxConfig_s;
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
#[no_mangle]
pub static mut SystemCoreClock: uint32_t = 0;
// mapping of radio channels to internal RPYTA+ order
// type of UART-based receiver (0 = spek 10, 1 = spek 11, 2 = sbus). Must be enabled by FEATURE_RX_SERIAL first.
// invert the serial RX protocol compared to it's default setting
// allow rx to operate in half duplex mode on F4, ignored for F1 and F3.
// number of bind pulses for Spektrum satellite receivers
// whenever we will reset (exit) binding mode after hard reboot
// Some radios have not a neutral point centered on 1500. can be changed here
// minimum rc end
// maximum rc end
// Camera angle to be scaled into rc commands
// Throttle setpoint percent where airmode gets activated
// true to use frame drop flags in the rx protocol
// offset applied to the RSSI value before it is returned
// Determines the smoothing algorithm to use: INTERPOLATION or FILTER
// Filter cutoff frequency for the input filter (0 = auto)
// Filter cutoff frequency for the setpoint weight derivative filter (0 = auto)
// Axis to log as debug values when debug_mode = RC_SMOOTHING
// Input filter type (0 = PT1, 1 = BIQUAD)
// Derivative filter type (0 = OFF, 1 = PT1, 2 = BIQUAD)
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
#[no_mangle]
pub static mut timerHardware: [timerHardware_t; 1] =
    [timerHardware_t{tim: 0 as *const TIM_TypeDef as *mut TIM_TypeDef,
                     tag: 0,
                     channel: 0,
                     usageFlags: TIM_USE_ANY,
                     output: 0,}; 1];
// unused
static mut fdmPkt: fdm_packet =
    fdm_packet{timestamp: 0.,
               imu_angular_velocity_rpy: [0.; 3],
               imu_linear_acceleration_xyz: [0.; 3],
               imu_orientation_quat: [0.; 4],
               velocity_xyz: [0.; 3],
               position_xyz: [0.; 3],};
static mut pwmPkt: servo_packet = servo_packet{motor_speed: [0.; 4],};
static mut start_time: timespec = timespec{tv_sec: 0, tv_nsec: 0,};
static mut simRate: libc::c_double = 1.0f64;
static mut tcpWorker: pthread_t = 0;
static mut udpWorker: pthread_t = 0;
static mut workerRunning: bool = 1i32 != 0;
static mut stateLink: udpLink_t =
    udpLink_t{fd: 0,
              si:
                  sockaddr_in{sin_family: 0,
                              sin_port: 0,
                              sin_addr: in_addr{s_addr: 0,},
                              sin_zero: [0; 8],},
              recv:
                  sockaddr_in{sin_family: 0,
                              sin_port: 0,
                              sin_addr: in_addr{s_addr: 0,},
                              sin_zero: [0; 8],},
              port: 0,
              addr: 0 as *const libc::c_char as *mut libc::c_char,
              isServer: false,};
static mut pwmLink: udpLink_t =
    udpLink_t{fd: 0,
              si:
                  sockaddr_in{sin_family: 0,
                              sin_port: 0,
                              sin_addr: in_addr{s_addr: 0,},
                              sin_zero: [0; 8],},
              recv:
                  sockaddr_in{sin_family: 0,
                              sin_port: 0,
                              sin_addr: in_addr{s_addr: 0,},
                              sin_zero: [0; 8],},
              port: 0,
              addr: 0 as *const libc::c_char as *mut libc::c_char,
              isServer: false,};
static mut updateLock: pthread_mutex_t =
    pthread_mutex_t{__data:
                        __pthread_mutex_s{__lock: 0,
                                          __count: 0,
                                          __owner: 0,
                                          __nusers: 0,
                                          __kind: 0,
                                          __spins: 0,
                                          __elision: 0,
                                          __list:
                                              __pthread_list_t{__prev:
                                                                   0 as
                                                                       *const __pthread_internal_list
                                                                       as
                                                                       *mut __pthread_internal_list,
                                                               __next:
                                                                   0 as
                                                                       *const __pthread_internal_list
                                                                       as
                                                                       *mut __pthread_internal_list,},},};
static mut mainLoopLock: pthread_mutex_t =
    pthread_mutex_t{__data:
                        __pthread_mutex_s{__lock: 0,
                                          __count: 0,
                                          __owner: 0,
                                          __nusers: 0,
                                          __kind: 0,
                                          __spins: 0,
                                          __elision: 0,
                                          __list:
                                              __pthread_list_t{__prev:
                                                                   0 as
                                                                       *const __pthread_internal_list
                                                                       as
                                                                       *mut __pthread_internal_list,
                                                               __next:
                                                                   0 as
                                                                       *const __pthread_internal_list
                                                                       as
                                                                       *mut __pthread_internal_list,},},};
#[no_mangle]
pub unsafe extern "C" fn lockMainPID() -> libc::c_int {
    return pthread_mutex_trylock(&mut mainLoopLock); // in seconds
}
#[no_mangle]
pub unsafe extern "C" fn sendMotorUpdate() {
    udpSend(&mut pwmLink,
            &mut pwmPkt as *mut servo_packet as *const libc::c_void,
            ::core::mem::size_of::<servo_packet>() as libc::c_ulong); // in uS
}
#[no_mangle]
pub unsafe extern "C" fn updateState(mut pkt: *const fdm_packet) {
    static mut last_timestamp: libc::c_double =
        0i32 as libc::c_double; // last packet
    static mut last_realtime: uint64_t = 0i32 as uint64_t;
    static mut last_ts: timespec = timespec{tv_sec: 0, tv_nsec: 0,};
    let mut now_ts: timespec = timespec{tv_sec: 0, tv_nsec: 0,};
    clock_gettime(1i32, &mut now_ts);
    let realtime_now: uint64_t = micros64_real();
    if realtime_now as libc::c_double >
           last_realtime as libc::c_double + 500i32 as libc::c_double * 1e3f64
       {
        // 500ms timeout
        last_timestamp = (*pkt).timestamp; // in seconds
        last_realtime = realtime_now;
        sendMotorUpdate();
        return
    }
    let deltaSim: libc::c_double = (*pkt).timestamp - last_timestamp;
    if deltaSim < 0i32 as libc::c_double {
        // don't use old packet
        return
    }
    let mut x: int16_t = 0;
    let mut y: int16_t = 0;
    let mut z: int16_t = 0;
    x =
        constrain((-(*pkt).imu_linear_acceleration_xyz[0] *
                       (256i32 as libc::c_double / 9.80665f64)) as
                      libc::c_int, -32767i32, 32767i32) as int16_t;
    y =
        constrain((-(*pkt).imu_linear_acceleration_xyz[1] *
                       (256i32 as libc::c_double / 9.80665f64)) as
                      libc::c_int, -32767i32, 32767i32) as int16_t;
    z =
        constrain((-(*pkt).imu_linear_acceleration_xyz[2] *
                       (256i32 as libc::c_double / 9.80665f64)) as
                      libc::c_int, -32767i32, 32767i32) as int16_t;
    fakeAccSet(fakeAccDev, x, y, z);
    //    printf("[acc]%lf,%lf,%lf\n", pkt->imu_linear_acceleration_xyz[0], pkt->imu_linear_acceleration_xyz[1], pkt->imu_linear_acceleration_xyz[2]);
    x =
        constrain(((*pkt).imu_angular_velocity_rpy[0] * 16.4f64 *
                       (180.0f64 / 3.14159265358979323846f64)) as libc::c_int,
                  -32767i32, 32767i32) as int16_t;
    y =
        constrain((-(*pkt).imu_angular_velocity_rpy[1] * 16.4f64 *
                       (180.0f64 / 3.14159265358979323846f64)) as libc::c_int,
                  -32767i32, 32767i32) as int16_t;
    z =
        constrain((-(*pkt).imu_angular_velocity_rpy[2] * 16.4f64 *
                       (180.0f64 / 3.14159265358979323846f64)) as libc::c_int,
                  -32767i32, 32767i32) as int16_t;
    fakeGyroSet(fakeGyroDev, x, y, z);
    //    printf("[gyr]%lf,%lf,%lf\n", pkt->imu_angular_velocity_rpy[0], pkt->imu_angular_velocity_rpy[1], pkt->imu_angular_velocity_rpy[2]);
    imuSetAttitudeQuat((*pkt).imu_orientation_quat[0] as libc::c_float,
                       (*pkt).imu_orientation_quat[1] as libc::c_float,
                       (*pkt).imu_orientation_quat[2] as libc::c_float,
                       (*pkt).imu_orientation_quat[3] as libc::c_float);
    if deltaSim < 0.02f64 && deltaSim > 0i32 as libc::c_double {
        // simulator should run faster than 50Hz
        //        simRate = simRate * 0.5 + (1e6 * deltaSim / (realtime_now - last_realtime)) * 0.5;
        let mut out_ts: timespec = timespec{tv_sec: 0, tv_nsec: 0,};
        timeval_sub(&mut out_ts, &mut now_ts, &mut last_ts);
        simRate =
            deltaSim /
                (out_ts.tv_sec as libc::c_double +
                     1e-9f64 * out_ts.tv_nsec as libc::c_double)
    }
    //    printf("simRate = %lf, millis64 = %lu, millis64_real = %lu, deltaSim = %lf\n", simRate, millis64(), millis64_real(), deltaSim*1e6);
    last_timestamp = (*pkt).timestamp;
    last_realtime = micros64_real();
    last_ts.tv_sec = now_ts.tv_sec;
    last_ts.tv_nsec = now_ts.tv_nsec;
    pthread_mutex_unlock(&mut updateLock);
    // can send PWM output now
}
unsafe extern "C" fn udpThread(mut data: *mut libc::c_void)
 -> *mut libc::c_void {
    let mut n: libc::c_int = 0i32;
    while workerRunning {
        n =
            udpRecv(&mut stateLink,
                    &mut fdmPkt as *mut fdm_packet as *mut libc::c_void,
                    ::core::mem::size_of::<fdm_packet>() as libc::c_ulong,
                    100i32 as uint32_t);
        if n as libc::c_ulong ==
               ::core::mem::size_of::<fdm_packet>() as libc::c_ulong {
            //            printf("[data]new fdm %d\n", n);
            updateState(&mut fdmPkt);
        }
    }
    printf(b"udpThread end!!\n\x00" as *const u8 as *const libc::c_char);
    return 0 as *mut libc::c_void;
}
unsafe extern "C" fn tcpThread(mut data: *mut libc::c_void)
 -> *mut libc::c_void {
    dyad_init();
    dyad_setTickInterval(0.2f32 as libc::c_double);
    dyad_setUpdateTimeout(0.5f32 as libc::c_double);
    while workerRunning { dyad_update(); }
    dyad_shutdown();
    printf(b"tcpThread end!!\n\x00" as *const u8 as *const libc::c_char);
    return 0 as *mut libc::c_void;
}
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
// system
#[no_mangle]
pub unsafe extern "C" fn systemInit() {
    let mut ret: libc::c_int = 0; // fake 500MHz
    clock_gettime(1i32, &mut start_time);
    printf(b"[system]Init...\n\x00" as *const u8 as *const libc::c_char);
    SystemCoreClock = (500i32 as libc::c_double * 1e6f64) as uint32_t;
    FLASH_Unlock();
    if pthread_mutex_init(&mut updateLock, 0 as *const pthread_mutexattr_t) !=
           0i32 {
        printf(b"Create updateLock error!\n\x00" as *const u8 as
                   *const libc::c_char);
        exit(1i32);
    }
    if pthread_mutex_init(&mut mainLoopLock, 0 as *const pthread_mutexattr_t)
           != 0i32 {
        printf(b"Create mainLoopLock error!\n\x00" as *const u8 as
                   *const libc::c_char);
        exit(1i32);
    }
    ret =
        pthread_create(&mut tcpWorker, 0 as *const pthread_attr_t,
                       Some(tcpThread as
                                unsafe extern "C" fn(_: *mut libc::c_void)
                                    -> *mut libc::c_void),
                       0 as *mut libc::c_void);
    if ret != 0i32 {
        printf(b"Create tcpWorker error!\n\x00" as *const u8 as
                   *const libc::c_char);
        exit(1i32);
    }
    ret =
        udpInit(&mut pwmLink,
                b"127.0.0.1\x00" as *const u8 as *const libc::c_char, 9002i32,
                0i32 != 0);
    printf(b"init PwnOut UDP link...%d\n\x00" as *const u8 as
               *const libc::c_char, ret);
    ret =
        udpInit(&mut stateLink, 0 as *const libc::c_char, 9003i32, 1i32 != 0);
    printf(b"start UDP server...%d\n\x00" as *const u8 as *const libc::c_char,
           ret);
    ret =
        pthread_create(&mut udpWorker, 0 as *const pthread_attr_t,
                       Some(udpThread as
                                unsafe extern "C" fn(_: *mut libc::c_void)
                                    -> *mut libc::c_void),
                       0 as *mut libc::c_void);
    if ret != 0i32 {
        printf(b"Create udpWorker error!\n\x00" as *const u8 as
                   *const libc::c_char);
        exit(1i32);
    }
    // serial can't been slow down
    rescheduleTask(TASK_SERIAL, 1i32 as uint32_t);
}
// bootloader/IAP
#[no_mangle]
pub unsafe extern "C" fn systemReset() {
    printf(b"[system]Reset!\n\x00" as *const u8 as *const libc::c_char);
    workerRunning = 0i32 != 0;
    pthread_join(tcpWorker, 0 as *mut *mut libc::c_void);
    pthread_join(udpWorker, 0 as *mut *mut libc::c_void);
    exit(0i32);
}
#[no_mangle]
pub unsafe extern "C" fn systemResetToBootloader() {
    printf(b"[system]ResetToBootloader!\n\x00" as *const u8 as
               *const libc::c_char);
    workerRunning = 0i32 != 0;
    pthread_join(tcpWorker, 0 as *mut *mut libc::c_void);
    pthread_join(udpWorker, 0 as *mut *mut libc::c_void);
    exit(0i32);
}
#[no_mangle]
pub unsafe extern "C" fn timerInit() {
    printf(b"[timer]Init...\n\x00" as *const u8 as *const libc::c_char);
}
#[no_mangle]
pub unsafe extern "C" fn timerStart() { }
#[no_mangle]
pub unsafe extern "C" fn failureMode(mut mode: failureMode_e) {
    printf(b"[failureMode]!!! %d\n\x00" as *const u8 as *const libc::c_char,
           mode as libc::c_uint);
    loop  { };
}
// failure
#[no_mangle]
pub unsafe extern "C" fn indicateFailure(mut mode: failureMode_e,
                                         mut repeatCount: libc::c_int) {
    printf(b"Failure LED flash for: [failureMode]!!! %d\n\x00" as *const u8 as
               *const libc::c_char, mode as libc::c_uint);
}
// Time part
// Thanks ArduPilot
#[no_mangle]
pub unsafe extern "C" fn nanos64_real() -> uint64_t {
    let mut ts: timespec = timespec{tv_sec: 0, tv_nsec: 0,};
    clock_gettime(1i32, &mut ts);
    return (ts.tv_sec as libc::c_double * 1e9f64 +
                ts.tv_nsec as libc::c_double -
                (start_time.tv_sec as libc::c_double * 1e9f64 +
                     start_time.tv_nsec as libc::c_double)) as uint64_t;
}
#[no_mangle]
pub unsafe extern "C" fn micros64_real() -> uint64_t {
    let mut ts: timespec = timespec{tv_sec: 0, tv_nsec: 0,};
    clock_gettime(1i32, &mut ts);
    return (1.0e6f64 *
                (ts.tv_sec as libc::c_double +
                     ts.tv_nsec as libc::c_double * 1.0e-9f64 -
                     (start_time.tv_sec as libc::c_double +
                          start_time.tv_nsec as libc::c_double * 1.0e-9f64)))
               as uint64_t;
}
#[no_mangle]
pub unsafe extern "C" fn millis64_real() -> uint64_t {
    let mut ts: timespec = timespec{tv_sec: 0, tv_nsec: 0,};
    clock_gettime(1i32, &mut ts);
    return (1.0e3f64 *
                (ts.tv_sec as libc::c_double +
                     ts.tv_nsec as libc::c_double * 1.0e-9f64 -
                     (start_time.tv_sec as libc::c_double +
                          start_time.tv_nsec as libc::c_double * 1.0e-9f64)))
               as uint64_t;
}
#[no_mangle]
pub unsafe extern "C" fn micros64() -> uint64_t {
    static mut last: uint64_t = 0i32 as uint64_t;
    static mut out: uint64_t = 0i32 as uint64_t;
    let mut now: uint64_t = nanos64_real();
    out =
        (out as libc::c_double +
             now.wrapping_sub(last) as libc::c_double * simRate) as uint64_t;
    last = now;
    return (out as libc::c_double * 1e-3f64) as uint64_t;
    //    return micros64_real();
}
#[no_mangle]
pub unsafe extern "C" fn millis64() -> uint64_t {
    static mut last: uint64_t = 0i32 as uint64_t;
    static mut out: uint64_t = 0i32 as uint64_t;
    let mut now: uint64_t = nanos64_real();
    out =
        (out as libc::c_double +
             now.wrapping_sub(last) as libc::c_double * simRate) as uint64_t;
    last = now;
    return (out as libc::c_double * 1e-6f64) as uint64_t;
    //    return millis64_real();
}
#[no_mangle]
pub unsafe extern "C" fn micros() -> uint32_t {
    return (micros64() & 0xffffffffu32 as libc::c_ulong) as uint32_t;
}
#[no_mangle]
pub unsafe extern "C" fn millis() -> uint32_t {
    return (millis64() & 0xffffffffu32 as libc::c_ulong) as uint32_t;
}
#[no_mangle]
pub unsafe extern "C" fn microsleep(mut usec: uint32_t) {
    let mut ts: timespec = timespec{tv_sec: 0, tv_nsec: 0,};
    ts.tv_sec = 0i32 as __time_t;
    ts.tv_nsec =
        (usec as libc::c_ulong).wrapping_mul(1000u64) as __syscall_slong_t;
    while nanosleep(&mut ts, &mut ts) == -1i32 && *__errno_location() == 4i32
          {
    };
}
#[no_mangle]
pub unsafe extern "C" fn delayMicroseconds(mut us: uint32_t) {
    microsleep((us as libc::c_double / simRate) as uint32_t);
}
#[no_mangle]
pub unsafe extern "C" fn delayMicroseconds_real(mut us: uint32_t) {
    microsleep(us);
}
#[no_mangle]
pub unsafe extern "C" fn delay(mut ms: uint32_t) {
    let mut start: uint64_t = millis64();
    while millis64().wrapping_sub(start) < ms as libc::c_ulong {
        microsleep(1000i32 as uint32_t);
    };
}
// Subtract the ‘struct timespec’ values X and Y,  storing the result in RESULT.
// Return 1 if the difference is negative, otherwise 0.
// result = x - y
// from: http://www.gnu.org/software/libc/manual/html_node/Elapsed-Time.html
#[no_mangle]
pub unsafe extern "C" fn timeval_sub(mut result: *mut timespec,
                                     mut x: *mut timespec,
                                     mut y: *mut timespec) -> libc::c_int {
    let mut s_carry: libc::c_uint = 0i32 as libc::c_uint;
    let mut ns_carry: libc::c_uint = 0i32 as libc::c_uint;
    // Perform the carry for the later subtraction by updating y.
    if (*x).tv_nsec < (*y).tv_nsec {
        let mut nsec: libc::c_int =
            (((*y).tv_nsec - (*x).tv_nsec) / 1000000000i32 as libc::c_long +
                 1i32 as libc::c_long) as libc::c_int;
        ns_carry =
            ns_carry.wrapping_add((1000000000i32 * nsec) as libc::c_uint);
        s_carry = s_carry.wrapping_add(nsec as libc::c_uint)
    }
    // Compute the time remaining to wait. tv_usec is certainly positive.
    (*result).tv_sec = (*x).tv_sec - (*y).tv_sec - s_carry as libc::c_long;
    (*result).tv_nsec =
        (*x).tv_nsec - (*y).tv_nsec + ns_carry as libc::c_long;
    // Return 1 if result is negative.
    return ((*x).tv_sec < (*y).tv_sec) as libc::c_int;
}
// PWM part
static mut pwmMotorsEnabled: bool = 0i32 != 0;
static mut motors: [pwmOutputPort_t; 8] =
    [pwmOutputPort_t{channel:
                         timerChannel_t{ccr:
                                            0 as *const timCCR_t as
                                                *mut timCCR_t,
                                        tim:
                                            0 as *const TIM_TypeDef as
                                                *mut TIM_TypeDef,},
                     pulseScale: 0.,
                     pulseOffset: 0.,
                     forceOverflow: false,
                     enabled: false,
                     io: 0 as *const libc::c_void as *mut libc::c_void,}; 8];
static mut servos: [pwmOutputPort_t; 8] =
    [pwmOutputPort_t{channel:
                         timerChannel_t{ccr:
                                            0 as *const timCCR_t as
                                                *mut timCCR_t,
                                        tim:
                                            0 as *const TIM_TypeDef as
                                                *mut TIM_TypeDef,},
                     pulseScale: 0.,
                     pulseOffset: 0.,
                     forceOverflow: false,
                     enabled: false,
                     io: 0 as *const libc::c_void as *mut libc::c_void,}; 8];
// real value to send
static mut motorsPwm: [int16_t; 8] = [0; 8];
static mut servosPwm: [int16_t; 8] = [0; 8];
static mut idlePulse: int16_t = 0;
#[no_mangle]
pub unsafe extern "C" fn motorDevInit(mut motorConfig:
                                          *const motorDevConfig_t,
                                      mut _idlePulse: uint16_t,
                                      mut motorCount: uint8_t) {
    idlePulse = _idlePulse as int16_t;
    let mut motorIndex: libc::c_int = 0i32;
    while motorIndex < 8i32 && motorIndex < motorCount as libc::c_int {
        motors[motorIndex as usize].enabled = 1i32 != 0;
        motorIndex += 1
    }
    pwmMotorsEnabled = 1i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn servoDevInit(mut servoConfig:
                                          *const servoDevConfig_t) {
    let mut servoIndex: uint8_t = 0i32 as uint8_t;
    while (servoIndex as libc::c_int) < 8i32 {
        servos[servoIndex as usize].enabled = 1i32 != 0;
        servoIndex = servoIndex.wrapping_add(1)
    };
}
#[no_mangle]
pub unsafe extern "C" fn pwmGetMotors() -> *mut pwmOutputPort_t {
    return motors.as_mut_ptr();
}
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
/*
  DshotSettingRequest (KISS24). Spin direction, 3d and save Settings reqire 10 requests.. and the TLM Byte must always be high if 1-47 are used to send settings

  3D Mode:
  0 = stop
  48   (low) - 1047 (high) -> negative direction
  1048 (low) - 2047 (high) -> positive direction
 */
// V2 includes settings
// Currently not implemented
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// KISS audio Stream mode on/Off
// KISS silent Mode on/Off
/* resolution + frame reset (2us) */
/* resolution + frame reset (2us) */
// function pointer used to write motors
// function pointer used after motors are written
//CAVEAT: This is used in the `motorConfig_t` parameter group, so the parameter group constraints apply
// The update rate of motor outputs (50-498Hz)
// Pwm Protocol
// Active-High vs Active-Low. Useful for brushed FCs converted for brushless operation
// PWM values, in milliseconds, common range is 1000-2000 (1ms to 2ms)
// This is the value for servos when they should be in the middle. e.g. 1500.
// The update rate of servo outputs (50-498Hz)
#[no_mangle]
pub unsafe extern "C" fn pwmEnableMotors() { pwmMotorsEnabled = 1i32 != 0; }
#[no_mangle]
pub unsafe extern "C" fn pwmAreMotorsEnabled() -> bool {
    return pwmMotorsEnabled;
}
#[no_mangle]
pub unsafe extern "C" fn isMotorProtocolDshot() -> bool { return 0i32 != 0; }
#[no_mangle]
pub unsafe extern "C" fn pwmWriteMotor(mut index: uint8_t,
                                       mut value: libc::c_float) {
    motorsPwm[index as usize] =
        (value - idlePulse as libc::c_int as libc::c_float) as int16_t;
}
#[no_mangle]
pub unsafe extern "C" fn pwmShutdownPulsesForAllMotors(mut motorCount:
                                                           uint8_t) {
    pwmMotorsEnabled = 0i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn pwmCompleteMotorUpdate(mut motorCount: uint8_t) {
    // send to simulator
    // for gazebo8 ArduCopterPlugin remap, normal range = [0.0, 1.0], 3D rang = [-1.0, 1.0]
    let mut outScale: libc::c_double = 1000.0f64;
    if feature(FEATURE_3D as libc::c_int as uint32_t) { outScale = 500.0f64 }
    pwmPkt.motor_speed[3] =
        (motorsPwm[0] as libc::c_int as libc::c_double / outScale) as
            libc::c_float;
    pwmPkt.motor_speed[0] =
        (motorsPwm[1] as libc::c_int as libc::c_double / outScale) as
            libc::c_float;
    pwmPkt.motor_speed[1] =
        (motorsPwm[2] as libc::c_int as libc::c_double / outScale) as
            libc::c_float;
    pwmPkt.motor_speed[2] =
        (motorsPwm[3] as libc::c_int as libc::c_double / outScale) as
            libc::c_float;
    // get one "fdm_packet" can only send one "servo_packet"!!
    if pthread_mutex_trylock(&mut updateLock) != 0i32 { return }
    udpSend(&mut pwmLink,
            &mut pwmPkt as *mut servo_packet as *const libc::c_void,
            ::core::mem::size_of::<servo_packet>() as libc::c_ulong);
    //    printf("[pwm]%u:%u,%u,%u,%u\n", idlePulse, motorsPwm[0], motorsPwm[1], motorsPwm[2], motorsPwm[3]);
}
#[no_mangle]
pub unsafe extern "C" fn pwmWriteServo(mut index: uint8_t,
                                       mut value: libc::c_float) {
    servosPwm[index as usize] = value as int16_t;
}
// ADC part
#[no_mangle]
pub unsafe extern "C" fn adcGetChannel(mut channel: uint8_t) -> uint16_t {
    return 0i32 as uint16_t;
}
// stack part
#[no_mangle]
pub static mut _estack: libc::c_char = 0;
#[no_mangle]
pub static mut _Min_Stack_Size: libc::c_char = 0;
// fake EEPROM
static mut eepromFd: *mut FILE = 0 as *const FILE as *mut FILE;
#[no_mangle]
pub static mut eepromData: [uint8_t; 32768] = [0; 32768];
#[no_mangle]
pub unsafe extern "C" fn FLASH_Unlock() {
    if !eepromFd.is_null() {
        fprintf(stderr,
                b"[FLASH_Unlock] eepromFd != NULL\n\x00" as *const u8 as
                    *const libc::c_char);
        return
    }
    // open or create
    eepromFd =
        fopen(b"eeprom.bin\x00" as *const u8 as *const libc::c_char,
              b"r+\x00" as *const u8 as *const libc::c_char);
    if !eepromFd.is_null() {
        // obtain file size:
        fseek(eepromFd, 0i32 as libc::c_long, 2i32);
        let mut lSize: size_t = ftell(eepromFd) as size_t;
        rewind(eepromFd);
        let mut n: size_t =
            fread(eepromData.as_mut_ptr() as *mut libc::c_void,
                  1i32 as libc::c_ulong,
                  ::core::mem::size_of::<[uint8_t; 32768]>() as libc::c_ulong,
                  eepromFd);
        if n == lSize {
            printf(b"[FLASH_Unlock] loaded \'%s\', size = %ld / %ld\n\x00" as
                       *const u8 as *const libc::c_char,
                   b"eeprom.bin\x00" as *const u8 as *const libc::c_char,
                   lSize,
                   ::core::mem::size_of::<[uint8_t; 32768]>() as
                       libc::c_ulong);
        } else {
            fprintf(stderr,
                    b"[FLASH_Unlock] failed to load \'%s\'\n\x00" as *const u8
                        as *const libc::c_char,
                    b"eeprom.bin\x00" as *const u8 as *const libc::c_char);
            return
        }
    } else {
        printf(b"[FLASH_Unlock] created \'%s\', size = %ld\n\x00" as *const u8
                   as *const libc::c_char,
               b"eeprom.bin\x00" as *const u8 as *const libc::c_char,
               ::core::mem::size_of::<[uint8_t; 32768]>() as libc::c_ulong);
        eepromFd =
            fopen(b"eeprom.bin\x00" as *const u8 as *const libc::c_char,
                  b"w+\x00" as *const u8 as *const libc::c_char);
        if eepromFd.is_null() {
            fprintf(stderr,
                    b"[FLASH_Unlock] failed to create \'%s\'\n\x00" as
                        *const u8 as *const libc::c_char,
                    b"eeprom.bin\x00" as *const u8 as *const libc::c_char);
            return
        }
        if fwrite(eepromData.as_mut_ptr() as *const libc::c_void,
                  ::core::mem::size_of::<[uint8_t; 32768]>() as libc::c_ulong,
                  1i32 as libc::c_ulong, eepromFd) != 1i32 as libc::c_ulong {
            fprintf(stderr,
                    b"[FLASH_Unlock] write failed: %s\n\x00" as *const u8 as
                        *const libc::c_char, strerror(*__errno_location()));
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn FLASH_Lock() {
    // flush & close
    if !eepromFd.is_null() {
        fseek(eepromFd, 0i32 as libc::c_long, 0i32);
        fwrite(eepromData.as_mut_ptr() as *const libc::c_void,
               1i32 as libc::c_ulong,
               ::core::mem::size_of::<[uint8_t; 32768]>() as libc::c_ulong,
               eepromFd);
        fclose(eepromFd);
        eepromFd = 0 as *mut FILE;
        printf(b"[FLASH_Lock] saved \'%s\'\n\x00" as *const u8 as
                   *const libc::c_char,
               b"eeprom.bin\x00" as *const u8 as *const libc::c_char);
    } else {
        fprintf(stderr,
                b"[FLASH_Lock] eeprom is not unlocked\n\x00" as *const u8 as
                    *const libc::c_char);
    };
}
#[no_mangle]
pub unsafe extern "C" fn FLASH_ErasePage(mut Page_Address: uintptr_t)
 -> FLASH_Status {
    //    printf("[FLASH_ErasePage]%x\n", Page_Address);
    return FLASH_COMPLETE;
}
#[no_mangle]
pub unsafe extern "C" fn FLASH_ProgramWord(mut addr: uintptr_t,
                                           mut value: uint32_t)
 -> FLASH_Status {
    if addr >= eepromData.as_mut_ptr() as uintptr_t &&
           addr <
               &mut *eepromData.as_mut_ptr().offset((::core::mem::size_of::<[uint8_t; 32768]>()
                                                         as
                                                         libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>()
                                                                                         as
                                                                                         libc::c_ulong)
                                                        as isize) as
                   *mut uint8_t as uintptr_t {
        *(addr as *mut uint32_t) = value;
        printf(b"[FLASH_ProgramWord]%p = %08x\n\x00" as *const u8 as
                   *const libc::c_char, addr as *mut libc::c_void,
               *(addr as *mut uint32_t));
    } else {
        printf(b"[FLASH_ProgramWord]%p out of range!\n\x00" as *const u8 as
                   *const libc::c_char, addr as *mut libc::c_void);
    }
    return FLASH_COMPLETE;
}
#[no_mangle]
pub unsafe extern "C" fn uartPinConfigure(mut pSerialPinConfig:
                                              *const serialPinConfig_t) {
    printf(b"uartPinConfigure\x00" as *const u8 as *const libc::c_char);
}
#[no_mangle]
pub unsafe extern "C" fn spektrumBind(mut rxConfig: *mut rxConfig_t) {
    printf(b"spektrumBind\x00" as *const u8 as *const libc::c_char);
}
