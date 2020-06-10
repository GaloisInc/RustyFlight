use ::libc;
extern "C" {
    #[no_mangle]
    fn beeperConfirmationBeeps(beepCount: uint8_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type armingDisableFlags_e = libc::c_uint;
pub const ARMING_DISABLED_ARM_SWITCH: armingDisableFlags_e = 524288;
pub const ARMING_DISABLED_GPS: armingDisableFlags_e = 262144;
pub const ARMING_DISABLED_PARALYZE: armingDisableFlags_e = 131072;
pub const ARMING_DISABLED_MSP: armingDisableFlags_e = 65536;
pub const ARMING_DISABLED_BST: armingDisableFlags_e = 32768;
pub const ARMING_DISABLED_OSD_MENU: armingDisableFlags_e = 16384;
pub const ARMING_DISABLED_CMS_MENU: armingDisableFlags_e = 8192;
pub const ARMING_DISABLED_CLI: armingDisableFlags_e = 4096;
pub const ARMING_DISABLED_CALIBRATING: armingDisableFlags_e = 2048;
pub const ARMING_DISABLED_LOAD: armingDisableFlags_e = 1024;
pub const ARMING_DISABLED_NOPREARM: armingDisableFlags_e = 512;
pub const ARMING_DISABLED_BOOT_GRACE_TIME: armingDisableFlags_e = 256;
pub const ARMING_DISABLED_ANGLE: armingDisableFlags_e = 128;
pub const ARMING_DISABLED_THROTTLE: armingDisableFlags_e = 64;
pub const ARMING_DISABLED_RUNAWAY_TAKEOFF: armingDisableFlags_e = 32;
pub const ARMING_DISABLED_BOXFAILSAFE: armingDisableFlags_e = 16;
pub const ARMING_DISABLED_BAD_RX_RECOVERY: armingDisableFlags_e = 8;
pub const ARMING_DISABLED_RX_FAILSAFE: armingDisableFlags_e = 4;
pub const ARMING_DISABLED_FAILSAFE: armingDisableFlags_e = 2;
pub const ARMING_DISABLED_NO_GYRO: armingDisableFlags_e = 1;
pub type flightModeFlags_e = libc::c_uint;
pub const GPS_RESCUE_MODE: flightModeFlags_e = 2048;
pub const FAILSAFE_MODE: flightModeFlags_e = 1024;
pub const PASSTHRU_MODE: flightModeFlags_e = 256;
pub const HEADFREE_MODE: flightModeFlags_e = 64;
pub const GPS_HOLD_MODE: flightModeFlags_e = 32;
pub const GPS_HOME_MODE: flightModeFlags_e = 16;
pub const BARO_MODE: flightModeFlags_e = 8;
pub const MAG_MODE: flightModeFlags_e = 4;
pub const HORIZON_MODE: flightModeFlags_e = 2;
pub const ANGLE_MODE: flightModeFlags_e = 1;
#[no_mangle]
pub static mut SystemCoreClock: uint32_t = 0;
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
pub static mut armingFlags: uint8_t = 0 as libc::c_int as uint8_t;
#[no_mangle]
pub static mut stateFlags: uint8_t = 0 as libc::c_int as uint8_t;
#[no_mangle]
pub static mut flightModeFlags: uint16_t = 0 as libc::c_int as uint16_t;
static mut enabledSensors: uint32_t = 0 as libc::c_int as uint32_t;
// Must be shorter than OSD_WARNINGS_MAX_SIZE (11) to be displayed fully in OSD
#[no_mangle]
pub static mut armingDisableFlagNames: [*const libc::c_char; 20] =
    [b"NOGYRO\x00" as *const u8 as *const libc::c_char,
     b"FAILSAFE\x00" as *const u8 as *const libc::c_char,
     b"RXLOSS\x00" as *const u8 as *const libc::c_char,
     b"BADRX\x00" as *const u8 as *const libc::c_char,
     b"BOXFAILSAFE\x00" as *const u8 as *const libc::c_char,
     b"RUNAWAY\x00" as *const u8 as *const libc::c_char,
     b"THROTTLE\x00" as *const u8 as *const libc::c_char,
     b"ANGLE\x00" as *const u8 as *const libc::c_char,
     b"BOOTGRACE\x00" as *const u8 as *const libc::c_char,
     b"NOPREARM\x00" as *const u8 as *const libc::c_char,
     b"LOAD\x00" as *const u8 as *const libc::c_char,
     b"CALIB\x00" as *const u8 as *const libc::c_char,
     b"CLI\x00" as *const u8 as *const libc::c_char,
     b"CMS\x00" as *const u8 as *const libc::c_char,
     b"OSD\x00" as *const u8 as *const libc::c_char,
     b"BST\x00" as *const u8 as *const libc::c_char,
     b"MSP\x00" as *const u8 as *const libc::c_char,
     b"PARALYZE\x00" as *const u8 as *const libc::c_char,
     b"GPS\x00" as *const u8 as *const libc::c_char,
     b"ARMSWITCH\x00" as *const u8 as *const libc::c_char];
static mut armingDisableFlags: armingDisableFlags_e =
    0 as armingDisableFlags_e;
#[no_mangle]
pub unsafe extern "C" fn setArmingDisabled(mut flag: armingDisableFlags_e) {
    armingDisableFlags =
        (armingDisableFlags as libc::c_uint | flag as libc::c_uint) as
            armingDisableFlags_e;
}
#[no_mangle]
pub unsafe extern "C" fn unsetArmingDisabled(mut flag: armingDisableFlags_e) {
    armingDisableFlags =
        (armingDisableFlags as libc::c_uint & !(flag as libc::c_uint)) as
            armingDisableFlags_e;
}
#[no_mangle]
pub unsafe extern "C" fn isArmingDisabled() -> bool {
    return armingDisableFlags as u64 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn getArmingDisableFlags() -> armingDisableFlags_e {
    return armingDisableFlags;
}
/* *
 * Enables the given flight mode.  A beep is sounded if the flight mode
 * has changed.  Returns the new 'flightModeFlags' value.
 */
#[no_mangle]
pub unsafe extern "C" fn enableFlightMode(mut mask: flightModeFlags_e)
 -> uint16_t {
    let mut oldVal: uint16_t = flightModeFlags;
    flightModeFlags =
        (flightModeFlags as libc::c_uint | mask as libc::c_uint) as uint16_t;
    if flightModeFlags as libc::c_int != oldVal as libc::c_int {
        beeperConfirmationBeeps(1 as libc::c_int as uint8_t);
    }
    return flightModeFlags;
}
/* *
 * Disables the given flight mode.  A beep is sounded if the flight mode
 * has changed.  Returns the new 'flightModeFlags' value.
 */
#[no_mangle]
pub unsafe extern "C" fn disableFlightMode(mut mask: flightModeFlags_e)
 -> uint16_t {
    let mut oldVal: uint16_t = flightModeFlags;
    flightModeFlags =
        (flightModeFlags as libc::c_uint & !(mask as libc::c_uint)) as
            uint16_t;
    if flightModeFlags as libc::c_int != oldVal as libc::c_int {
        beeperConfirmationBeeps(1 as libc::c_int as uint8_t);
    }
    return flightModeFlags;
}
#[no_mangle]
pub unsafe extern "C" fn sensors(mut mask: uint32_t) -> bool {
    return enabledSensors & mask != 0;
}
#[no_mangle]
pub unsafe extern "C" fn sensorsSet(mut mask: uint32_t) {
    enabledSensors |= mask;
}
#[no_mangle]
pub unsafe extern "C" fn sensorsClear(mut mask: uint32_t) {
    enabledSensors &= !mask;
}
#[no_mangle]
pub unsafe extern "C" fn sensorsMask() -> uint32_t { return enabledSensors; }
