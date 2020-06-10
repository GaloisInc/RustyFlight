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
    static mut armingFlags: uint8_t;
    #[no_mangle]
    static mut stateFlags: uint8_t;
    #[no_mangle]
    fn sensors(mask: uint32_t) -> bool;
    #[no_mangle]
    static mut gpsSol: gpsSolutionData_t;
    #[no_mangle]
    fn isBaroCalibrationComplete() -> bool;
    #[no_mangle]
    fn performBaroCalibrationCycle();
    #[no_mangle]
    fn baroCalculateAltitude() -> int32_t;
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
pub type C2RustUnnamed_0 = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed_0 = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed_0 = 2;
pub const ARMED: C2RustUnnamed_0 = 1;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const FIXED_WING: C2RustUnnamed_1 = 16;
pub const SMALL_ANGLE: C2RustUnnamed_1 = 8;
pub const CALIBRATE_MAG: C2RustUnnamed_1 = 4;
pub const GPS_FIX: C2RustUnnamed_1 = 2;
pub const GPS_FIX_HOME: C2RustUnnamed_1 = 1;
pub type timeUs_t = uint32_t;
pub type gpsSolutionData_t = gpsSolutionData_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsSolutionData_s {
    pub llh: gpsLocation_t,
    pub groundSpeed: uint16_t,
    pub groundCourse: uint16_t,
    pub hdop: uint16_t,
    pub numSat: uint8_t,
}
// speed in 0.1m/s
// degrees * 10
// generic HDOP value (*100)
/* LLH Location in NEU axis system */
pub type gpsLocation_t = gpsLocation_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsLocation_s {
    pub lat: int32_t,
    pub lon: int32_t,
    pub alt: int32_t,
}
pub const SENSOR_GPS: C2RustUnnamed_2 = 32;
pub const SENSOR_BARO: C2RustUnnamed_2 = 4;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_2 = 64;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_2 = 16;
pub const SENSOR_SONAR: C2RustUnnamed_2 = 16;
pub const SENSOR_MAG: C2RustUnnamed_2 = 8;
pub const SENSOR_ACC: C2RustUnnamed_2 = 2;
pub const SENSOR_GYRO: C2RustUnnamed_2 = 1;
#[no_mangle]
pub static mut SystemCoreClock: uint32_t = 0;
// latitude * 1e+7
// longitude * 1e+7
// altitude in 0.01m
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
static mut estimatedAltitude: int32_t = 0 as libc::c_int;
static mut altitudeOffsetSet: bool = 0 as libc::c_int != 0;
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
pub unsafe extern "C" fn calculateEstimatedAltitude(mut currentTimeUs:
                                                        timeUs_t) {
    static mut previousTimeUs: timeUs_t =
        0 as libc::c_int as timeUs_t; //conservative default
    static mut baroAltOffset: int32_t = 0 as libc::c_int;
    static mut gpsAltOffset: int32_t = 0 as libc::c_int;
    let dTime: uint32_t = currentTimeUs.wrapping_sub(previousTimeUs);
    if dTime < (1000 as libc::c_int * 25 as libc::c_int) as libc::c_uint {
        return
    }
    previousTimeUs = currentTimeUs;
    let mut baroAlt: int32_t = 0 as libc::c_int;
    let mut gpsAlt: int32_t = 0 as libc::c_int;
    let mut gpsTrust: libc::c_float = 0.3f64 as libc::c_float;
    let mut haveBaroAlt: bool = 0 as libc::c_int != 0;
    let mut haveGpsAlt: bool = 0 as libc::c_int != 0;
    if sensors(SENSOR_BARO as libc::c_int as uint32_t) {
        if !isBaroCalibrationComplete() {
            performBaroCalibrationCycle();
        } else {
            baroAlt = baroCalculateAltitude();
            haveBaroAlt = 1 as libc::c_int != 0
        }
    }
    if sensors(SENSOR_GPS as libc::c_int as uint32_t) as libc::c_int != 0 &&
           stateFlags as libc::c_int & GPS_FIX as libc::c_int != 0 {
        gpsAlt = gpsSol.llh.alt;
        haveGpsAlt = 1 as libc::c_int != 0;
        if gpsSol.hdop as libc::c_int != 0 as libc::c_int {
            gpsTrust =
                (100.0f64 / gpsSol.hdop as libc::c_int as libc::c_double) as
                    libc::c_float
        }
        // always use at least 10% of other sources besides gps if available
        gpsTrust =
            ({
                 let mut _a: libc::c_float = gpsTrust;
                 let mut _b: libc::c_float = 0.9f32;
                 if _a < _b { _a } else { _b }
             })
    }
    if armingFlags as libc::c_int & ARMED as libc::c_int != 0 &&
           !altitudeOffsetSet {
        baroAltOffset = baroAlt;
        gpsAltOffset = gpsAlt;
        altitudeOffsetSet = 1 as libc::c_int != 0
    } else if armingFlags as libc::c_int & ARMED as libc::c_int == 0 &&
                  altitudeOffsetSet as libc::c_int != 0 {
        altitudeOffsetSet = 0 as libc::c_int != 0
    }
    baroAlt -= baroAltOffset;
    gpsAlt -= gpsAltOffset;
    if haveGpsAlt as libc::c_int != 0 && haveBaroAlt as libc::c_int != 0 {
        estimatedAltitude =
            (gpsAlt as libc::c_float * gpsTrust +
                 baroAlt as libc::c_float *
                     (1 as libc::c_int as libc::c_float - gpsTrust)) as
                int32_t
    } else if haveGpsAlt {
        estimatedAltitude = gpsAlt
    } else if haveBaroAlt { estimatedAltitude = baroAlt }
    if debugMode as libc::c_int == DEBUG_ALTITUDE as libc::c_int {
        debug[0 as libc::c_int as usize] =
            (100 as libc::c_int as libc::c_float * gpsTrust) as int32_t as
                int16_t
    }
    if debugMode as libc::c_int == DEBUG_ALTITUDE as libc::c_int {
        debug[1 as libc::c_int as usize] = baroAlt as int16_t
    }
    if debugMode as libc::c_int == DEBUG_ALTITUDE as libc::c_int {
        debug[2 as libc::c_int as usize] = gpsAlt as int16_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn isAltitudeOffset() -> bool {
    return altitudeOffsetSet;
}
#[no_mangle]
pub unsafe extern "C" fn getEstimatedAltitude() -> int32_t {
    return estimatedAltitude;
}
// This should be removed or fixed, but it would require changing a lot of other things to get rid of.
#[no_mangle]
pub unsafe extern "C" fn getEstimatedVario() -> int16_t {
    return 0 as libc::c_int as int16_t;
}
