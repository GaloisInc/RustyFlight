use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
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
    static mut debug: [int16_t; 4];
    #[no_mangle]
    static mut debugMode: uint8_t;
    #[no_mangle]
    fn biquadFilterInitLPF(filter: *mut biquadFilter_t,
                           filterFreq: libc::c_float, refreshRate: uint32_t);
    #[no_mangle]
    fn getVrefMv() -> uint16_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type C2RustUnnamed = libc::c_uint;
pub const DEBUG_COUNT: C2RustUnnamed = 44;
pub const DEBUG_ANTI_GRAVITY: C2RustUnnamed = 43;
pub const DEBUG_RC_SMOOTHING_RATE: C2RustUnnamed = 42;
pub const DEBUG_RX_SIGNAL_LOSS: C2RustUnnamed = 41;
pub const DEBUG_RC_SMOOTHING: C2RustUnnamed = 40;
pub const DEBUG_ACRO_TRAINER: C2RustUnnamed = 39;
pub const DEBUG_ITERM_RELAX: C2RustUnnamed = 38;
pub const DEBUG_RTH: C2RustUnnamed = 37;
pub const DEBUG_SMARTAUDIO: C2RustUnnamed = 36;
pub const DEBUG_USB: C2RustUnnamed = 35;
pub const DEBUG_CURRENT: C2RustUnnamed = 34;
pub const DEBUG_SDIO: C2RustUnnamed = 33;
pub const DEBUG_RUNAWAY_TAKEOFF: C2RustUnnamed = 32;
pub const DEBUG_CORE_TEMP: C2RustUnnamed = 31;
pub const DEBUG_LIDAR_TF: C2RustUnnamed = 30;
pub const DEBUG_RANGEFINDER_QUALITY: C2RustUnnamed = 29;
pub const DEBUG_RANGEFINDER: C2RustUnnamed = 28;
pub const DEBUG_FPORT: C2RustUnnamed = 27;
pub const DEBUG_SBUS: C2RustUnnamed = 26;
pub const DEBUG_MAX7456_SPICLOCK: C2RustUnnamed = 25;
pub const DEBUG_MAX7456_SIGNAL: C2RustUnnamed = 24;
pub const DEBUG_DUAL_GYRO_DIFF: C2RustUnnamed = 23;
pub const DEBUG_DUAL_GYRO_COMBINE: C2RustUnnamed = 22;
pub const DEBUG_DUAL_GYRO_RAW: C2RustUnnamed = 21;
pub const DEBUG_DUAL_GYRO: C2RustUnnamed = 20;
pub const DEBUG_GYRO_RAW: C2RustUnnamed = 19;
pub const DEBUG_RX_FRSKY_SPI: C2RustUnnamed = 18;
pub const DEBUG_FFT_FREQ: C2RustUnnamed = 17;
pub const DEBUG_FFT_TIME: C2RustUnnamed = 16;
pub const DEBUG_FFT: C2RustUnnamed = 15;
pub const DEBUG_ALTITUDE: C2RustUnnamed = 14;
pub const DEBUG_ESC_SENSOR_TMP: C2RustUnnamed = 13;
pub const DEBUG_ESC_SENSOR_RPM: C2RustUnnamed = 12;
pub const DEBUG_STACK: C2RustUnnamed = 11;
pub const DEBUG_SCHEDULER: C2RustUnnamed = 10;
pub const DEBUG_ESC_SENSOR: C2RustUnnamed = 9;
pub const DEBUG_ANGLERATE: C2RustUnnamed = 8;
pub const DEBUG_RC_INTERPOLATION: C2RustUnnamed = 7;
pub const DEBUG_GYRO_SCALED: C2RustUnnamed = 6;
pub const DEBUG_PIDLOOP: C2RustUnnamed = 5;
pub const DEBUG_ACCELEROMETER: C2RustUnnamed = 4;
pub const DEBUG_GYRO_FILTERED: C2RustUnnamed = 3;
pub const DEBUG_BATTERY: C2RustUnnamed = 2;
pub const DEBUG_CYCLETIME: C2RustUnnamed = 1;
pub const DEBUG_NONE: C2RustUnnamed = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct biquadFilter_s {
    pub b0: libc::c_float,
    pub b1: libc::c_float,
    pub b2: libc::c_float,
    pub a1: libc::c_float,
    pub a2: libc::c_float,
    pub x1: libc::c_float,
    pub x2: libc::c_float,
    pub y1: libc::c_float,
    pub y2: libc::c_float,
}
/* this holds the data required to update samples thru a filter */
pub type biquadFilter_t = biquadFilter_s;
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
pub type pgn_t = uint16_t;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed_0 = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed_0 = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed_0 = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed_0 = 4095;
// function that resets a single parameter group instance
pub type pgResetFunc
    =
    unsafe extern "C" fn(_: *mut libc::c_void, _: libc::c_int) -> ();
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pgRegistry_s {
    pub pgn: pgn_t,
    pub size: uint16_t,
    pub address: *mut uint8_t,
    pub copy: *mut uint8_t,
    pub ptr: *mut *mut uint8_t,
    pub reset: C2RustUnnamed_1,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_1 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<pgResetFunc>,
}
pub type pgRegistry_t = pgRegistry_s;
pub type currentMeterId_e = libc::c_uint;
pub const CURRENT_METER_ID_MSP_2: currentMeterId_e = 91;
pub const CURRENT_METER_ID_MSP_1: currentMeterId_e = 90;
pub const CURRENT_METER_ID_VIRTUAL_2: currentMeterId_e = 81;
pub const CURRENT_METER_ID_VIRTUAL_1: currentMeterId_e = 80;
pub const CURRENT_METER_ID_ESC_MOTOR_20: currentMeterId_e = 79;
pub const CURRENT_METER_ID_ESC_MOTOR_12: currentMeterId_e = 71;
pub const CURRENT_METER_ID_ESC_MOTOR_11: currentMeterId_e = 70;
pub const CURRENT_METER_ID_ESC_MOTOR_10: currentMeterId_e = 69;
pub const CURRENT_METER_ID_ESC_MOTOR_9: currentMeterId_e = 68;
pub const CURRENT_METER_ID_ESC_MOTOR_8: currentMeterId_e = 67;
pub const CURRENT_METER_ID_ESC_MOTOR_7: currentMeterId_e = 66;
pub const CURRENT_METER_ID_ESC_MOTOR_6: currentMeterId_e = 65;
pub const CURRENT_METER_ID_ESC_MOTOR_5: currentMeterId_e = 64;
pub const CURRENT_METER_ID_ESC_MOTOR_4: currentMeterId_e = 63;
pub const CURRENT_METER_ID_ESC_MOTOR_3: currentMeterId_e = 62;
pub const CURRENT_METER_ID_ESC_MOTOR_2: currentMeterId_e = 61;
pub const CURRENT_METER_ID_ESC_MOTOR_1: currentMeterId_e = 60;
pub const CURRENT_METER_ID_ESC_COMBINED_10: currentMeterId_e = 59;
pub const CURRENT_METER_ID_ESC_COMBINED_1: currentMeterId_e = 50;
pub const CURRENT_METER_ID_12V_10: currentMeterId_e = 49;
pub const CURRENT_METER_ID_12V_2: currentMeterId_e = 41;
pub const CURRENT_METER_ID_12V_1: currentMeterId_e = 40;
pub const CURRENT_METER_ID_9V_10: currentMeterId_e = 39;
pub const CURRENT_METER_ID_9V_2: currentMeterId_e = 31;
pub const CURRENT_METER_ID_9V_1: currentMeterId_e = 30;
pub const CURRENT_METER_ID_5V_10: currentMeterId_e = 29;
pub const CURRENT_METER_ID_5V_2: currentMeterId_e = 21;
pub const CURRENT_METER_ID_5V_1: currentMeterId_e = 20;
pub const CURRENT_METER_ID_BATTERY_10: currentMeterId_e = 19;
pub const CURRENT_METER_ID_BATTERY_2: currentMeterId_e = 11;
pub const CURRENT_METER_ID_BATTERY_1: currentMeterId_e = 10;
pub const CURRENT_METER_ID_NONE: currentMeterId_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct currentMeter_s {
    pub amperage: int32_t,
    pub amperageLatest: int32_t,
    pub mAhDrawn: int32_t,
}
pub type currentMeter_t = currentMeter_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct currentMeterMAhDrawnState_s {
    pub mAhDrawn: int32_t,
    pub mAhDrawnF: libc::c_float,
}
/* base */
/* size */
// The parameter group number, the top 4 bits are reserved for version
// Size of the group in RAM, the top 4 bits are reserved for flags
// Address of the group in RAM.
// Address of the copy in RAM.
// The pointer to update after loading the record into ram.
// Pointer to init template
// Pointer to pgResetFunc
// current read by current sensor in centiampere (1/100th A)
// current read by current sensor in centiampere (1/100th A) (unfiltered)
// milliampere hours drawn from the battery since start
// WARNING - do not mix usage of CURRENT_SENSOR_* and CURRENT_METER_*, they are separate concerns.
pub type currentMeterMAhDrawnState_t = currentMeterMAhDrawnState_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct currentMeterADCState_s {
    pub mahDrawnState: currentMeterMAhDrawnState_t,
    pub amperage: int32_t,
    pub amperageLatest: int32_t,
}
// milliampere hours drawn from the battery since start
//
// ADC
//
pub type currentMeterADCState_t = currentMeterADCState_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct currentSensorADCConfig_s {
    pub scale: int16_t,
    pub offset: int16_t,
}
pub type currentSensorADCConfig_t = currentSensorADCConfig_s;
#[inline]
unsafe extern "C" fn currentSensorADCConfig()
 -> *const currentSensorADCConfig_t {
    return &mut currentSensorADCConfig_System;
}
// current read by current sensor in centiampere (1/100th A)
// current read by current sensor in centiampere (1/100th A) (unfiltered)
// scale the current sensor output voltage to milliamps. Value in mV/10A
// offset of the current sensor in mA
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
pub static mut currentMeterSourceNames: [*const libc::c_char; 5] =
    [b"NONE\x00" as *const u8 as *const libc::c_char,
     b"ADC\x00" as *const u8 as *const libc::c_char,
     b"VIRTUAL\x00" as *const u8 as *const libc::c_char,
     b"ESC\x00" as *const u8 as *const libc::c_char,
     b"MSP\x00" as *const u8 as *const libc::c_char];
#[no_mangle]
pub static mut currentMeterIds: [uint8_t; 1] =
    [CURRENT_METER_ID_BATTERY_1 as libc::c_int as uint8_t];
// Initialized in run_static_initializers
#[no_mangle]
pub static mut supportedCurrentMeterCount: uint8_t = 0;
//
// Current Meter API
//
//
// ADC/Virtual/ESC/MSP shared
//
#[no_mangle]
pub unsafe extern "C" fn currentMeterReset(mut meter: *mut currentMeter_t) {
    (*meter).amperage = 0 as libc::c_int;
    (*meter).amperageLatest = 0 as libc::c_int;
    (*meter).mAhDrawn = 0 as libc::c_int;
}
static mut adciBatFilter: biquadFilter_t =
    biquadFilter_t{b0: 0.,
                   b1: 0.,
                   b2: 0.,
                   a1: 0.,
                   a2: 0.,
                   x1: 0.,
                   x2: 0.,
                   y1: 0.,
                   y2: 0.,};
// for Allegro ACS758LCB-100U (40mV/A)
#[no_mangle]
pub static mut currentSensorADCConfig_Copy: currentSensorADCConfig_t =
    currentSensorADCConfig_t{scale: 0, offset: 0,};
#[no_mangle]
pub static mut currentSensorADCConfig_System: currentSensorADCConfig_t =
    currentSensorADCConfig_t{scale: 0, offset: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut currentSensorADCConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (256 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<currentSensorADCConfig_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &currentSensorADCConfig_System as
                                     *const currentSensorADCConfig_t as
                                     *mut currentSensorADCConfig_t as
                                     *mut uint8_t,
                             copy:
                                 &currentSensorADCConfig_Copy as
                                     *const currentSensorADCConfig_t as
                                     *mut currentSensorADCConfig_t as
                                     *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_1{ptr:
                                                     &pgResetTemplate_currentSensorADCConfig
                                                         as
                                                         *const currentSensorADCConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_currentSensorADCConfig:
           currentSensorADCConfig_t =
    {
        let mut init =
            currentSensorADCConfig_s{scale: 400 as libc::c_int as int16_t,
                                     offset: 0 as libc::c_int as int16_t,};
        init
    };
unsafe extern "C" fn currentMeterADCToCentiamps(src: uint16_t) -> int32_t {
    let mut config: *const currentSensorADCConfig_t =
        currentSensorADCConfig();
    let mut millivolts: int32_t =
        (src as
             uint32_t).wrapping_mul(getVrefMv() as
                                        libc::c_uint).wrapping_div(4096 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_uint)
            as int32_t;
    // y=x/m+b m is scale in (mV/10A) and b is offset in (mA)
    let mut centiAmps: int32_t =
        (millivolts * 10000 as libc::c_int / (*config).scale as int32_t +
             (*config).offset as int32_t) / 10 as libc::c_int;
    if debugMode as libc::c_int == DEBUG_CURRENT as libc::c_int {
        debug[0 as libc::c_int as usize] = millivolts as int16_t
    }
    if debugMode as libc::c_int == DEBUG_CURRENT as libc::c_int {
        debug[1 as libc::c_int as usize] = centiAmps as int16_t
    }
    return centiAmps;
    // Returns Centiamps to maintain compatability with the rest of the code
}
//
// ADC
//
#[no_mangle]
pub static mut currentMeterADCState: currentMeterADCState_t =
    currentMeterADCState_t{mahDrawnState:
                               currentMeterMAhDrawnState_t{mAhDrawn: 0,
                                                           mAhDrawnF: 0.,},
                           amperage: 0,
                           amperageLatest: 0,};
#[no_mangle]
pub unsafe extern "C" fn currentMeterADCInit() {
    memset(&mut currentMeterADCState as *mut currentMeterADCState_t as
               *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<currentMeterADCState_t>() as libc::c_ulong);
    biquadFilterInitLPF(&mut adciBatFilter, 0.4f32,
                        (1000000 as libc::c_int / 50 as libc::c_int) as
                            uint32_t);
}
#[no_mangle]
pub unsafe extern "C" fn currentMeterADCRefresh(mut lastUpdateAt: int32_t) {
    currentMeterADCState.amperageLatest = 0 as libc::c_int;
    currentMeterADCState.amperage = 0 as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn currentMeterADCRead(mut meter: *mut currentMeter_t) {
    (*meter).amperageLatest = currentMeterADCState.amperageLatest;
    (*meter).amperage = currentMeterADCState.amperage;
    (*meter).mAhDrawn = currentMeterADCState.mahDrawnState.mAhDrawn;
    if debugMode as libc::c_int == DEBUG_CURRENT as libc::c_int {
        debug[2 as libc::c_int as usize] = (*meter).amperageLatest as int16_t
    }
    if debugMode as libc::c_int == DEBUG_CURRENT as libc::c_int {
        debug[3 as libc::c_int as usize] = (*meter).mAhDrawn as int16_t
    };
}
//
// API for reading current meters by id.
//
//
// VIRTUAL
//
//
// ESC
//
//
// API for current meters using IDs
//
// This API is used by MSP, for configuration/status.
//
#[no_mangle]
pub unsafe extern "C" fn currentMeterRead(mut id: currentMeterId_e,
                                          mut meter: *mut currentMeter_t) {
    if id as libc::c_uint ==
           CURRENT_METER_ID_BATTERY_1 as libc::c_int as libc::c_uint {
        currentMeterADCRead(meter);
    } else { currentMeterReset(meter); };
}
unsafe extern "C" fn run_static_initializers() {
    supportedCurrentMeterCount =
        (::core::mem::size_of::<[uint8_t; 1]>() as
             libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>() as
                                             libc::c_ulong) as uint8_t
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
