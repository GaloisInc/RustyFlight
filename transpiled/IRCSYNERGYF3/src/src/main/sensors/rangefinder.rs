use ::libc;
extern "C" {
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
    fn quickMedianFilter5(v: *mut int32_t) -> int32_t;
    #[no_mangle]
    fn cos_approx(x: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn millis() -> timeMs_t;
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
    fn hcsr04Detect(dev: *mut rangefinderDev_t,
                    sonarConfig_0: *const sonarConfig_t) -> bool;
    #[no_mangle]
    fn sensorsSet(mask: uint32_t);
    #[no_mangle]
    fn sensorsClear(mask: uint32_t);
    #[no_mangle]
    static mut requestedSensors: [uint8_t; 5];
    #[no_mangle]
    static mut detectedSensors: [uint8_t; 5];
    #[no_mangle]
    fn rescheduleTask(taskId: cfTaskId_e, newPeriodMicros: uint32_t);
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
/* base */
/* size */
// The parameter group number, the top 4 bits are reserved for version
// Size of the group in RAM, the top 4 bits are reserved for flags
// Address of the group in RAM.
// Address of the copy in RAM.
// The pointer to update after loading the record into ram.
// Pointer to init template
// Pointer to pgResetFunc
// millisecond time
pub type timeMs_t = uint32_t;
// microsecond time
pub type timeUs_t = uint32_t;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rangefinderDev_s {
    pub delayMs: timeMs_t,
    pub maxRangeCm: int16_t,
    pub detectionConeDeciDegrees: int16_t,
    pub detectionConeExtendedDeciDegrees: int16_t,
    pub init: rangefinderOpInitFuncPtr,
    pub update: rangefinderOpStartFuncPtr,
    pub read: rangefinderOpReadFuncPtr,
}
pub type rangefinderOpReadFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut rangefinderDev_s) -> int32_t>;
pub type rangefinderOpStartFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut rangefinderDev_s) -> ()>;
pub type rangefinderOpInitFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut rangefinderDev_s) -> ()>;
pub type rangefinderDev_t = rangefinderDev_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct sonarConfig_s {
    pub triggerTag: ioTag_t,
    pub echoTag: ioTag_t,
}
pub type sonarConfig_t = sonarConfig_s;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const SENSOR_INDEX_COUNT: C2RustUnnamed_2 = 5;
pub const SENSOR_INDEX_RANGEFINDER: C2RustUnnamed_2 = 4;
pub const SENSOR_INDEX_MAG: C2RustUnnamed_2 = 3;
pub const SENSOR_INDEX_BARO: C2RustUnnamed_2 = 2;
pub const SENSOR_INDEX_ACC: C2RustUnnamed_2 = 1;
pub const SENSOR_INDEX_GYRO: C2RustUnnamed_2 = 0;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_3 = 64;
pub const SENSOR_GPS: C2RustUnnamed_3 = 32;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_3 = 16;
pub const SENSOR_SONAR: C2RustUnnamed_3 = 16;
pub const SENSOR_MAG: C2RustUnnamed_3 = 8;
pub const SENSOR_BARO: C2RustUnnamed_3 = 4;
pub const SENSOR_ACC: C2RustUnnamed_3 = 2;
pub const SENSOR_GYRO: C2RustUnnamed_3 = 1;
pub type rangefinderType_e = libc::c_uint;
pub const RANGEFINDER_UIB: rangefinderType_e = 7;
pub const RANGEFINDER_VL53L0X: rangefinderType_e = 6;
pub const RANGEFINDER_HCSR04I2C: rangefinderType_e = 5;
pub const RANGEFINDER_SRF10: rangefinderType_e = 4;
pub const RANGEFINDER_TF02: rangefinderType_e = 3;
pub const RANGEFINDER_TFMINI: rangefinderType_e = 2;
pub const RANGEFINDER_HCSR04: rangefinderType_e = 1;
pub const RANGEFINDER_NONE: rangefinderType_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rangefinderConfig_s {
    pub rangefinder_hardware: uint8_t,
}
pub type rangefinderConfig_t = rangefinderConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rangefinder_s {
    pub dev: rangefinderDev_t,
    pub maxTiltCos: libc::c_float,
    pub rawAltitude: int32_t,
    pub calculatedAltitude: int32_t,
    pub lastValidResponseTimeMs: timeMs_t,
    pub snrThresholdReached: bool,
    pub dynamicDistanceThreshold: int32_t,
    pub snr: int16_t,
}
pub type rangefinder_t = rangefinder_s;
pub type cfTaskId_e = libc::c_uint;
pub const TASK_SELF: cfTaskId_e = 23;
pub const TASK_NONE: cfTaskId_e = 22;
pub const TASK_COUNT: cfTaskId_e = 22;
pub const TASK_PINIOBOX: cfTaskId_e = 21;
pub const TASK_RCDEVICE: cfTaskId_e = 20;
pub const TASK_CAMCTRL: cfTaskId_e = 19;
pub const TASK_VTXCTRL: cfTaskId_e = 18;
pub const TASK_CMS: cfTaskId_e = 17;
pub const TASK_ESC_SENSOR: cfTaskId_e = 16;
pub const TASK_TELEMETRY: cfTaskId_e = 15;
pub const TASK_ALTITUDE: cfTaskId_e = 14;
pub const TASK_RANGEFINDER: cfTaskId_e = 13;
pub const TASK_BARO: cfTaskId_e = 12;
pub const TASK_COMPASS: cfTaskId_e = 11;
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
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
#[inline]
unsafe extern "C" fn sonarConfig() -> *const sonarConfig_t {
    return &mut sonarConfig_System;
}
#[inline]
unsafe extern "C" fn rangefinderConfig() -> *const rangefinderConfig_t {
    return &mut rangefinderConfig_System;
}
// these are full detection cone angles, maximum tilt is half of this
// detection cone angle as in device spec
// device spec is conservative, in practice have slightly larger detection cone
// function pointers
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
//#include "uav_interconnect/uav_interconnect.h"
// XXX Interface to CF/BF legacy(?) altitude estimation code.
// XXX Will be gone once iNav's estimator is ported.
#[no_mangle]
pub static mut rangefinderMaxRangeCm: int16_t = 0;
#[no_mangle]
pub static mut rangefinderMaxAltWithTiltCm: int16_t = 0;
#[no_mangle]
pub static mut rangefinderCfAltCm: int16_t = 0;
// Complimentary Filter altitude
#[no_mangle]
pub static mut rangefinder: rangefinder_t =
    rangefinder_t{dev:
                      rangefinderDev_t{delayMs: 0,
                                       maxRangeCm: 0,
                                       detectionConeDeciDegrees: 0,
                                       detectionConeExtendedDeciDegrees: 0,
                                       init: None,
                                       update: None,
                                       read: None,},
                  maxTiltCos: 0.,
                  rawAltitude: 0,
                  calculatedAltitude: 0,
                  lastValidResponseTimeMs: 0,
                  snrThresholdReached: false,
                  dynamicDistanceThreshold: 0,
                  snr: 0,};
#[no_mangle]
pub static mut rangefinderConfig_Copy: rangefinderConfig_t =
    rangefinderConfig_t{rangefinder_hardware: 0,};
#[no_mangle]
pub static mut rangefinderConfig_System: rangefinderConfig_t =
    rangefinderConfig_t{rangefinder_hardware: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut rangefinderConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (527 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<rangefinderConfig_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &rangefinderConfig_System as
                                     *const rangefinderConfig_t as
                                     *mut rangefinderConfig_t as *mut uint8_t,
                             copy:
                                 &rangefinderConfig_Copy as
                                     *const rangefinderConfig_t as
                                     *mut rangefinderConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_1{ptr:
                                                     &pgResetTemplate_rangefinderConfig
                                                         as
                                                         *const rangefinderConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_rangefinderConfig: rangefinderConfig_t =
    {
        let mut init =
            rangefinderConfig_s{rangefinder_hardware:
                                    RANGEFINDER_NONE as libc::c_int as
                                        uint8_t,};
        init
    };
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut sonarConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (516 as libc::c_int |
                                      (1 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<sonarConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &sonarConfig_System as *const sonarConfig_t
                                     as *mut sonarConfig_t as *mut uint8_t,
                             copy:
                                 &sonarConfig_Copy as *const sonarConfig_t as
                                     *mut sonarConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_1{ptr:
                                                     &pgResetTemplate_sonarConfig
                                                         as
                                                         *const sonarConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut sonarConfig_System: sonarConfig_t =
    sonarConfig_t{triggerTag: 0, echoTag: 0,};
#[no_mangle]
pub static mut sonarConfig_Copy: sonarConfig_t =
    sonarConfig_t{triggerTag: 0, echoTag: 0,};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_sonarConfig: sonarConfig_t =
    {
        let mut init =
            sonarConfig_s{triggerTag:
                              ((1 as libc::c_int + 1 as libc::c_int) <<
                                   4 as libc::c_int | 0 as libc::c_int) as
                                  ioTag_t,
                          echoTag:
                              ((1 as libc::c_int + 1 as libc::c_int) <<
                                   4 as libc::c_int | 1 as libc::c_int) as
                                  ioTag_t,};
        init
    };
/*
 * Detect which rangefinder is present
 */
unsafe extern "C" fn rangefinderDetect(mut dev: *mut rangefinderDev_t,
                                       mut rangefinderHardwareToUse: uint8_t)
 -> bool {
    let mut rangefinderHardware: rangefinderType_e = RANGEFINDER_NONE;
    requestedSensors[SENSOR_INDEX_RANGEFINDER as libc::c_int as usize] =
        rangefinderHardwareToUse;
    match rangefinderHardwareToUse as libc::c_int {
        1 => {
            if hcsr04Detect(dev, sonarConfig()) {
                // FIXME: Do actual detection if HC-SR04 is plugged in
                rangefinderHardware = RANGEFINDER_HCSR04;
                rescheduleTask(TASK_RANGEFINDER,
                               (70 as libc::c_int * 1000 as libc::c_int) as
                                   uint32_t);
            }
        }
        0 => { rangefinderHardware = RANGEFINDER_NONE }
        4 | 5 | 6 | 7 | 2 | 3 | _ => { }
    }
    if rangefinderHardware as libc::c_uint ==
           RANGEFINDER_NONE as libc::c_int as libc::c_uint {
        sensorsClear(SENSOR_RANGEFINDER as libc::c_int as uint32_t);
        return 0 as libc::c_int != 0
    }
    detectedSensors[SENSOR_INDEX_RANGEFINDER as libc::c_int as usize] =
        rangefinderHardware as uint8_t;
    sensorsSet(SENSOR_RANGEFINDER as libc::c_int as uint32_t);
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn rangefinderResetDynamicThreshold() {
    rangefinder.snrThresholdReached = 0 as libc::c_int != 0;
    rangefinder.dynamicDistanceThreshold = 0 as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn rangefinderInit() -> bool {
    if !rangefinderDetect(&mut rangefinder.dev,
                          (*rangefinderConfig()).rangefinder_hardware) {
        return 0 as libc::c_int != 0
    }
    rangefinder.dev.init.expect("non-null function pointer")(&mut rangefinder.dev);
    rangefinder.rawAltitude = -(1 as libc::c_int);
    rangefinder.calculatedAltitude = -(1 as libc::c_int);
    rangefinder.maxTiltCos =
        cos_approx(rangefinder.dev.detectionConeExtendedDeciDegrees as
                       libc::c_int as libc::c_float / 2.0f32 / 10.0f32 *
                       0.0174532925f32);
    rangefinder.lastValidResponseTimeMs = millis();
    rangefinder.snr = 0 as libc::c_int as int16_t;
    rangefinderResetDynamicThreshold();
    // XXX Interface to CF/BF legacy(?) altitude estimation code.
    // XXX Will be gone once iNav's estimator is ported.
    rangefinderMaxRangeCm =
        rangefinder.dev.maxRangeCm; // Complimentary Filter altitude
    rangefinderMaxAltWithTiltCm =
        (rangefinderMaxRangeCm as libc::c_int as libc::c_float *
             rangefinder.maxTiltCos) as int16_t;
    rangefinderCfAltCm =
        (rangefinder.dev.maxRangeCm as libc::c_int / 2 as libc::c_int) as
            int16_t;
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn applyMedianFilter(mut newReading: int32_t) -> int32_t {
    static mut filterSamples: [int32_t; 5] = [0; 5];
    static mut filterSampleIndex: libc::c_int = 0 as libc::c_int;
    static mut medianFilterReady: bool = 0 as libc::c_int != 0;
    if newReading > -(1 as libc::c_int) {
        // only accept samples that are in range
        filterSamples[filterSampleIndex as usize] = newReading;
        filterSampleIndex += 1;
        if filterSampleIndex == 5 as libc::c_int {
            filterSampleIndex = 0 as libc::c_int;
            medianFilterReady = 1 as libc::c_int != 0
        }
    }
    return if medianFilterReady as libc::c_int != 0 {
               quickMedianFilter5(filterSamples.as_mut_ptr())
           } else { newReading };
}
unsafe extern "C" fn computePseudoSnr(mut newReading: int32_t) -> int16_t {
    static mut snrSamples: [int16_t; 5] = [0; 5];
    static mut snrSampleIndex: uint8_t = 0 as libc::c_int as uint8_t;
    static mut previousReading: int32_t = -(1 as libc::c_int);
    static mut snrReady: bool = 0 as libc::c_int != 0;
    let mut pseudoSnr: int16_t = 0 as libc::c_int as int16_t;
    let delta: libc::c_int = newReading - previousReading;
    snrSamples[snrSampleIndex as usize] =
        constrain(delta * delta / 10 as libc::c_int, 0 as libc::c_int,
                  6400 as libc::c_int) as int16_t;
    snrSampleIndex = snrSampleIndex.wrapping_add(1);
    if snrSampleIndex as libc::c_int == 5 as libc::c_int {
        snrSampleIndex = 0 as libc::c_int as uint8_t;
        snrReady = 1 as libc::c_int != 0
    }
    previousReading = newReading;
    if snrReady {
        let mut i: uint8_t = 0 as libc::c_int as uint8_t;
        while (i as libc::c_int) < 5 as libc::c_int {
            pseudoSnr =
                (pseudoSnr as libc::c_int +
                     snrSamples[i as usize] as libc::c_int) as int16_t;
            i = i.wrapping_add(1)
        }
        return constrain(pseudoSnr as libc::c_int, 0 as libc::c_int,
                         32000 as libc::c_int) as int16_t
    } else { return -(1 as libc::c_int) as int16_t };
}
/*
 * This is called periodically by the scheduler
 */
// XXX Returns timeDelta_t for iNav for pseudo-RT scheduling.
#[no_mangle]
pub unsafe extern "C" fn rangefinderUpdate(mut currentTimeUs: timeUs_t) {
    if rangefinder.dev.update.is_some() {
        rangefinder.dev.update.expect("non-null function pointer")(&mut rangefinder.dev);
    };
    // return rangefinder.dev.delayMs * 1000;  // to microseconds XXX iNav only
}
#[no_mangle]
pub unsafe extern "C" fn isSurfaceAltitudeValid() -> bool {
    /*
     * Preconditions: raw and calculated altidude > 0
     * SNR lower than threshold
     */
    if rangefinder.calculatedAltitude > 0 as libc::c_int &&
           rangefinder.rawAltitude > 0 as libc::c_int &&
           (rangefinder.snr as libc::c_int) < 600 as libc::c_int {
        /*
         * When critical altitude was determined, distance reported by rangefinder
         * has to be lower than it to assume healthy readout
         */
        if rangefinder.snrThresholdReached {
            return rangefinder.rawAltitude <
                       rangefinder.dynamicDistanceThreshold
        } else { return 1 as libc::c_int != 0 }
    } else { return 0 as libc::c_int != 0 };
}
/* *
 * Get the last distance measured by the sonar in centimeters. When the ground is too far away, RANGEFINDER_OUT_OF_RANGE is returned.
 */
#[no_mangle]
pub unsafe extern "C" fn rangefinderProcess(mut cosTiltAngle: libc::c_float)
 -> bool {
    if rangefinder.dev.read.is_some() {
        let distance: int32_t =
            rangefinder.dev.read.expect("non-null function pointer")(&mut rangefinder.dev);
        // If driver reported no new measurement - don't do anything
        if distance == -(3 as libc::c_int) { return 0 as libc::c_int != 0 }
        if distance >= 0 as libc::c_int {
            rangefinder.lastValidResponseTimeMs = millis();
            rangefinder.rawAltitude = applyMedianFilter(distance)
        } else if distance == -(1 as libc::c_int) {
            rangefinder.lastValidResponseTimeMs = millis();
            rangefinder.rawAltitude = -(1 as libc::c_int)
        } else {
            // Invalid response / hardware failure
            rangefinder.rawAltitude = -(2 as libc::c_int)
        }
        rangefinder.snr = computePseudoSnr(distance);
        if rangefinder.snrThresholdReached as libc::c_int == 0 as libc::c_int
               && rangefinder.rawAltitude > 0 as libc::c_int {
            if (rangefinder.snr as libc::c_int) < 600 as libc::c_int &&
                   rangefinder.dynamicDistanceThreshold <
                       rangefinder.rawAltitude {
                rangefinder.dynamicDistanceThreshold =
                    rangefinder.rawAltitude * 75 as libc::c_int /
                        100 as libc::c_int
            }
            if rangefinder.snr as libc::c_int >= 600 as libc::c_int {
                rangefinder.snrThresholdReached = 1 as libc::c_int != 0
            }
        }
        if debugMode as libc::c_int == DEBUG_RANGEFINDER as libc::c_int {
            debug[3 as libc::c_int as usize] = rangefinder.snr
        }
        if debugMode as libc::c_int ==
               DEBUG_RANGEFINDER_QUALITY as libc::c_int {
            debug[0 as libc::c_int as usize] =
                rangefinder.rawAltitude as int16_t
        }
        if debugMode as libc::c_int ==
               DEBUG_RANGEFINDER_QUALITY as libc::c_int {
            debug[1 as libc::c_int as usize] =
                rangefinder.snrThresholdReached as int16_t
        }
        if debugMode as libc::c_int ==
               DEBUG_RANGEFINDER_QUALITY as libc::c_int {
            debug[2 as libc::c_int as usize] =
                rangefinder.dynamicDistanceThreshold as int16_t
        }
        if debugMode as libc::c_int ==
               DEBUG_RANGEFINDER_QUALITY as libc::c_int {
            debug[3 as libc::c_int as usize] =
                isSurfaceAltitudeValid() as int16_t
        }
    } else {
        // Bad configuration
        rangefinder.rawAltitude = -(1 as libc::c_int)
    }
    /* *
    * Apply tilt correction to the given raw sonar reading in order to compensate for the tilt of the craft when estimating
    * the altitude. Returns the computed altitude in centimeters.
    *
    * When the ground is too far away or the tilt is too large, RANGEFINDER_OUT_OF_RANGE is returned.
    */
    if cosTiltAngle < rangefinder.maxTiltCos ||
           rangefinder.rawAltitude < 0 as libc::c_int {
        rangefinder.calculatedAltitude = -(1 as libc::c_int)
    } else {
        rangefinder.calculatedAltitude =
            (rangefinder.rawAltitude as libc::c_float * cosTiltAngle) as
                int32_t
    }
    if debugMode as libc::c_int == DEBUG_RANGEFINDER as libc::c_int {
        debug[1 as libc::c_int as usize] = rangefinder.rawAltitude as int16_t
    }
    if debugMode as libc::c_int == DEBUG_RANGEFINDER as libc::c_int {
        debug[2 as libc::c_int as usize] =
            rangefinder.calculatedAltitude as int16_t
    }
    return 1 as libc::c_int != 0;
}
/* *
 * Get the latest altitude that was computed, or RANGEFINDER_OUT_OF_RANGE if sonarCalculateAltitude
 * has never been called.
 */
#[no_mangle]
pub unsafe extern "C" fn rangefinderGetLatestAltitude() -> int32_t {
    return rangefinder.calculatedAltitude;
}
#[no_mangle]
pub unsafe extern "C" fn rangefinderGetLatestRawAltitude() -> int32_t {
    return rangefinder.rawAltitude;
}
#[no_mangle]
pub unsafe extern "C" fn rangefinderIsHealthy() -> bool {
    return millis().wrapping_sub(rangefinder.lastValidResponseTimeMs) <
               500 as libc::c_int as libc::c_uint;
}
