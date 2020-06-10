use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
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
// time difference, 32 bits always sufficient
// millisecond time
// microsecond time
pub type timeUs_t = uint32_t;
pub type beeperMode_e = libc::c_uint;
pub const BEEPER_ALL: beeperMode_e = 24;
pub const BEEPER_RC_SMOOTHING_INIT_FAIL: beeperMode_e = 23;
pub const BEEPER_CAM_CONNECTION_CLOSE: beeperMode_e = 22;
pub const BEEPER_CAM_CONNECTION_OPEN: beeperMode_e = 21;
pub const BEEPER_CRASH_FLIP_MODE: beeperMode_e = 20;
pub const BEEPER_BLACKBOX_ERASE: beeperMode_e = 19;
pub const BEEPER_USB: beeperMode_e = 18;
pub const BEEPER_SYSTEM_INIT: beeperMode_e = 17;
pub const BEEPER_ARMED: beeperMode_e = 16;
pub const BEEPER_DISARM_REPEAT: beeperMode_e = 15;
pub const BEEPER_MULTI_BEEPS: beeperMode_e = 14;
pub const BEEPER_READY_BEEP: beeperMode_e = 13;
pub const BEEPER_ACC_CALIBRATION_FAIL: beeperMode_e = 12;
pub const BEEPER_ACC_CALIBRATION: beeperMode_e = 11;
pub const BEEPER_RX_SET: beeperMode_e = 10;
pub const BEEPER_GPS_STATUS: beeperMode_e = 9;
pub const BEEPER_BAT_LOW: beeperMode_e = 8;
pub const BEEPER_BAT_CRIT_LOW: beeperMode_e = 7;
pub const BEEPER_ARMING_GPS_FIX: beeperMode_e = 6;
pub const BEEPER_ARMING: beeperMode_e = 5;
pub const BEEPER_DISARMING: beeperMode_e = 4;
pub const BEEPER_RX_LOST_LANDING: beeperMode_e = 3;
pub const BEEPER_RX_LOST: beeperMode_e = 2;
pub const BEEPER_GYRO_CALIBRATED: beeperMode_e = 1;
pub const BEEPER_SILENCE: beeperMode_e = 0;
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
//size limit for 'beep_multiBeeps[]'
// Stub out beeper functions if #BEEPER not defined
#[no_mangle]
pub unsafe extern "C" fn beeper(mut mode: beeperMode_e) { }
#[no_mangle]
pub unsafe extern "C" fn beeperSilence() { }
#[no_mangle]
pub unsafe extern "C" fn beeperConfirmationBeeps(mut beepCount: uint8_t) { }
#[no_mangle]
pub unsafe extern "C" fn beeperWarningBeeps(mut beepCount: uint8_t) { }
#[no_mangle]
pub unsafe extern "C" fn beeperUpdate(mut currentTimeUs: timeUs_t) { }
#[no_mangle]
pub unsafe extern "C" fn getArmingBeepTimeMicros() -> uint32_t {
    return 0 as libc::c_int as uint32_t;
}
#[no_mangle]
pub unsafe extern "C" fn beeperModeForTableIndex(mut idx: libc::c_int)
 -> beeperMode_e {
    return BEEPER_SILENCE;
}
#[no_mangle]
pub unsafe extern "C" fn beeperModeMaskForTableIndex(mut idx: libc::c_int)
 -> uint32_t {
    return 0 as libc::c_int as uint32_t;
}
#[no_mangle]
pub unsafe extern "C" fn beeperNameForTableIndex(mut idx: libc::c_int)
 -> *const libc::c_char {
    return 0 as *const libc::c_char;
}
#[no_mangle]
pub unsafe extern "C" fn beeperTableEntryCount() -> libc::c_int {
    return 0 as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn isBeeperOn() -> bool {
    return 0 as libc::c_int != 0;
}
