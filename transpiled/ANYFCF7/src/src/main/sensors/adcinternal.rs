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
    fn adcInternalReadTempsensor() -> uint16_t;
    #[no_mangle]
    static mut adcVREFINTCAL: uint16_t;
    #[no_mangle]
    static mut adcTSCAL1: uint16_t;
    #[no_mangle]
    static mut adcTSSlopeK: uint16_t;
    #[no_mangle]
    fn adcInternalIsBusy() -> bool;
    #[no_mangle]
    fn adcInternalStartConversion();
    #[no_mangle]
    fn adcInternalReadVrefint() -> uint16_t;
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
pub type timeUs_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct movingAverageStateUint16_s {
    pub sum: uint32_t,
    pub values: *mut uint16_t,
    pub size: uint8_t,
    pub pos: uint8_t,
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
pub type movingAverageStateUint16_t = movingAverageStateUint16_s;
#[no_mangle]
pub unsafe extern "C" fn updateMovingAverageUint16(mut state:
                                                       *mut movingAverageStateUint16_t,
                                                   mut newValue: uint16_t)
 -> uint16_t {
    (*state).sum =
        ((*state).sum as
             libc::c_uint).wrapping_sub(*(*state).values.offset((*state).pos
                                                                    as isize)
                                            as libc::c_uint) as uint32_t as
            uint32_t;
    *(*state).values.offset((*state).pos as isize) = newValue;
    (*state).sum =
        ((*state).sum as libc::c_uint).wrapping_add(newValue as libc::c_uint)
            as uint32_t as uint32_t;
    (*state).pos =
        (((*state).pos as libc::c_int + 1 as libc::c_int) %
             (*state).size as libc::c_int) as uint8_t;
    return (*state).sum.wrapping_div((*state).size as libc::c_uint) as
               uint16_t;
}
static mut adcVrefintValue: uint16_t = 0;
static mut adcVrefintValues: [uint16_t; 8] = [0; 8];
#[no_mangle]
pub static mut adcVrefintAverageState: movingAverageStateUint16_t =
    unsafe {
        {
            let mut init =
                movingAverageStateUint16_s{sum: 0 as libc::c_int as uint32_t,
                                           values:
                                               adcVrefintValues.as_ptr() as
                                                   *mut _,
                                           size: 8 as libc::c_int as uint8_t,
                                           pos: 0 as libc::c_int as uint8_t,};
            init
        }
    };
static mut adcTempsensorValue: uint16_t = 0;
static mut adcTempsensorValues: [uint16_t; 8] = [0; 8];
#[no_mangle]
pub static mut adcTempsensorAverageState: movingAverageStateUint16_t =
    unsafe {
        {
            let mut init =
                movingAverageStateUint16_s{sum: 0 as libc::c_int as uint32_t,
                                           values:
                                               adcTempsensorValues.as_ptr() as
                                                   *mut _,
                                           size: 8 as libc::c_int as uint8_t,
                                           pos: 0 as libc::c_int as uint8_t,};
            init
        }
    };
static mut coreTemperature: int16_t = 0;
#[no_mangle]
pub unsafe extern "C" fn getVrefMv() -> uint16_t {
    return (3300 as libc::c_int * adcVrefintValue as libc::c_int /
                adcVREFINTCAL as libc::c_int) as uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn getCoreTemperatureCelsius() -> int16_t {
    return coreTemperature;
}
#[no_mangle]
pub unsafe extern "C" fn adcInternalProcess(mut currentTimeUs: timeUs_t) {
    if adcInternalIsBusy() { return }
    let mut vrefintSample: uint16_t = adcInternalReadVrefint();
    let mut tempsensorSample: uint16_t = adcInternalReadTempsensor();
    adcVrefintValue =
        updateMovingAverageUint16(&mut adcVrefintAverageState, vrefintSample);
    adcTempsensorValue =
        updateMovingAverageUint16(&mut adcTempsensorAverageState,
                                  tempsensorSample);
    let mut adcTempsensorAdjusted: int32_t =
        adcTempsensorValue as int32_t * 3300 as libc::c_int /
            getVrefMv() as libc::c_int;
    coreTemperature =
        (((adcTempsensorAdjusted - adcTSCAL1 as libc::c_int) *
              adcTSSlopeK as libc::c_int +
              30 as libc::c_int * 1000 as libc::c_int + 500 as libc::c_int) /
             1000 as libc::c_int) as int16_t;
    if debugMode as libc::c_int == DEBUG_CORE_TEMP as libc::c_int {
        debug[0 as libc::c_int as usize] = coreTemperature
    }
    adcInternalStartConversion();
    // Start next conversion
}
#[no_mangle]
pub unsafe extern "C" fn adcInternalInit() {
    // Call adcInternalProcess repeatedly to fill moving average array
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 9 as libc::c_int {
        // empty
        while adcInternalIsBusy() { }
        adcInternalProcess(0 as libc::c_int as timeUs_t);
        i += 1
    };
}
