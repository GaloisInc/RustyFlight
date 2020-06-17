use core;
use libc;
extern "C" {
    #[no_mangle]
    fn strlen(_: *const libc::c_char) -> libc::c_ulong;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct displayPortVTable_s {
    pub grab: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                         -> libc::c_int>,
    pub release: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                            -> libc::c_int>,
    pub clearScreen: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                                -> libc::c_int>,
    pub drawScreen: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                               -> libc::c_int>,
    pub screenSize: Option<unsafe extern "C" fn(_: *const displayPort_t)
                               -> libc::c_int>,
    pub writeString: Option<unsafe extern "C" fn(_: *mut displayPort_t,
                                                 _: uint8_t, _: uint8_t,
                                                 _: *const libc::c_char)
                                -> libc::c_int>,
    pub writeChar: Option<unsafe extern "C" fn(_: *mut displayPort_t,
                                               _: uint8_t, _: uint8_t,
                                               _: uint8_t) -> libc::c_int>,
    pub isTransferInProgress: Option<unsafe extern "C" fn(_:
                                                              *const displayPort_t)
                                         -> bool>,
    pub heartbeat: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                              -> libc::c_int>,
    pub resync: Option<unsafe extern "C" fn(_: *mut displayPort_t) -> ()>,
    pub isSynced: Option<unsafe extern "C" fn(_: *const displayPort_t)
                             -> bool>,
    pub txBytesFree: Option<unsafe extern "C" fn(_: *const displayPort_t)
                                -> uint32_t>,
}
pub type displayPort_t = displayPort_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct displayPort_s {
    pub vTable: *const displayPortVTable_s,
    pub device: *mut libc::c_void,
    pub rows: uint8_t,
    pub cols: uint8_t,
    pub posX: uint8_t,
    pub posY: uint8_t,
    pub cleared: bool,
    pub cursorRow: int8_t,
    pub grabCount: int8_t,
}
pub type displayPortVTable_t = displayPortVTable_s;
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
// SITL (software in the loop) simulator
// use simulatior's attitude directly
// disable this if wants to test AHRS algorithm
//#define SIMULATOR_ACC_SYNC
//#define SIMULATOR_GYRO_SYNC
//#define SIMULATOR_IMU_SYNC
//#define SIMULATOR_GYROPID_SYNC
// file name to save config
//#define USE_SOFTSERIAL1
//#define USE_SOFTSERIAL2
// I think SITL don't need this
// suppress 'no pins defined' warning
// belows are internal stuff
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
pub unsafe extern "C" fn displayClearScreen(mut instance:
                                                *mut displayPort_t) {
    (*(*instance).vTable).clearScreen.expect("non-null function pointer")(instance);
    (*instance).cleared = 1i32 != 0;
    (*instance).cursorRow = -1i32 as int8_t;
}
#[no_mangle]
pub unsafe extern "C" fn displayDrawScreen(mut instance: *mut displayPort_t) {
    (*(*instance).vTable).drawScreen.expect("non-null function pointer")(instance);
}
#[no_mangle]
pub unsafe extern "C" fn displayScreenSize(mut instance: *const displayPort_t)
 -> libc::c_int {
    return (*(*instance).vTable).screenSize.expect("non-null function pointer")(instance);
}
// Note: displayPortProfile_t used as a parameter group for CMS over CRSF (io/displayport_crsf)
#[no_mangle]
pub unsafe extern "C" fn displayGrab(mut instance: *mut displayPort_t) {
    (*(*instance).vTable).grab.expect("non-null function pointer")(instance);
    (*(*instance).vTable).clearScreen.expect("non-null function pointer")(instance);
    (*instance).grabCount += 1;
}
#[no_mangle]
pub unsafe extern "C" fn displayRelease(mut instance: *mut displayPort_t) {
    (*(*instance).vTable).release.expect("non-null function pointer")(instance);
    (*instance).grabCount -= 1;
}
#[no_mangle]
pub unsafe extern "C" fn displayReleaseAll(mut instance: *mut displayPort_t) {
    (*(*instance).vTable).release.expect("non-null function pointer")(instance);
    (*instance).grabCount = 0i32 as int8_t;
}
#[no_mangle]
pub unsafe extern "C" fn displayIsGrabbed(mut instance: *const displayPort_t)
 -> bool {
    // can be called before initialised
    return !instance.is_null() && (*instance).grabCount as libc::c_int > 0i32;
}
#[no_mangle]
pub unsafe extern "C" fn displaySetXY(mut instance: *mut displayPort_t,
                                      mut x: uint8_t, mut y: uint8_t) {
    (*instance).posX = x;
    (*instance).posY = y;
}
#[no_mangle]
pub unsafe extern "C" fn displayWrite(mut instance: *mut displayPort_t,
                                      mut x: uint8_t, mut y: uint8_t,
                                      mut s: *const libc::c_char)
 -> libc::c_int {
    (*instance).posX =
        (x as libc::c_ulong).wrapping_add(strlen(s)) as uint8_t;
    (*instance).posY = y;
    return (*(*instance).vTable).writeString.expect("non-null function pointer")(instance,
                                                                                 x,
                                                                                 y,
                                                                                 s);
}
#[no_mangle]
pub unsafe extern "C" fn displayWriteChar(mut instance: *mut displayPort_t,
                                          mut x: uint8_t, mut y: uint8_t,
                                          mut c: uint8_t) -> libc::c_int {
    (*instance).posX = (x as libc::c_int + 1i32) as uint8_t;
    (*instance).posY = y;
    return (*(*instance).vTable).writeChar.expect("non-null function pointer")(instance,
                                                                               x,
                                                                               y,
                                                                               c);
}
#[no_mangle]
pub unsafe extern "C" fn displayIsTransferInProgress(mut instance:
                                                         *const displayPort_t)
 -> bool {
    return (*(*instance).vTable).isTransferInProgress.expect("non-null function pointer")(instance);
}
#[no_mangle]
pub unsafe extern "C" fn displayIsSynced(mut instance: *const displayPort_t)
 -> bool {
    return (*(*instance).vTable).isSynced.expect("non-null function pointer")(instance);
}
#[no_mangle]
pub unsafe extern "C" fn displayHeartbeat(mut instance: *mut displayPort_t) {
    (*(*instance).vTable).heartbeat.expect("non-null function pointer")(instance);
}
#[no_mangle]
pub unsafe extern "C" fn displayResync(mut instance: *mut displayPort_t) {
    (*(*instance).vTable).resync.expect("non-null function pointer")(instance);
}
#[no_mangle]
pub unsafe extern "C" fn displayTxBytesFree(mut instance:
                                                *const displayPort_t)
 -> uint16_t {
    return (*(*instance).vTable).txBytesFree.expect("non-null function pointer")(instance)
               as uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn displayInit(mut instance: *mut displayPort_t,
                                     mut vTable: *const displayPortVTable_t) {
    (*instance).vTable = vTable;
    (*(*instance).vTable).clearScreen.expect("non-null function pointer")(instance);
    (*instance).cleared = 1i32 != 0;
    (*instance).grabCount = 0i32 as int8_t;
    (*instance).cursorRow = -1i32 as int8_t;
}
