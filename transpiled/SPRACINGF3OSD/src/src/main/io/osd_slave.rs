use ::libc;
extern "C" {
    // Disabling this, in favour of tfp_format to be used in cli.c
//int tfp_printf(const char *fmt, ...);
    #[no_mangle]
    fn tfp_sprintf(s: *mut libc::c_char, fmt: *const libc::c_char, _: ...)
     -> libc::c_int;
    #[no_mangle]
    fn displayClearScreen(instance: *mut displayPort_t);
    #[no_mangle]
    fn displayDrawScreen(instance: *mut displayPort_t);
    #[no_mangle]
    fn displayWrite(instance: *mut displayPort_t, x: uint8_t, y: uint8_t,
                    s: *const libc::c_char) -> libc::c_int;
    #[no_mangle]
    fn displayWriteChar(instance: *mut displayPort_t, x: uint8_t, y: uint8_t,
                        c: uint8_t) -> libc::c_int;
    #[no_mangle]
    fn displayIsTransferInProgress(instance: *const displayPort_t) -> bool;
    #[no_mangle]
    fn displayResync(instance: *mut displayPort_t);
    #[no_mangle]
    fn displayIsSynced(instance: *const displayPort_t) -> bool;
    #[no_mangle]
    fn delay(ms: timeMs_t);
    #[no_mangle]
    fn micros() -> timeUs_t;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
#[derive(Copy, Clone)]
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
pub type displayPort_t = displayPort_s;
// CMS state
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
#[derive(Copy, Clone)]
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
pub type timeDelta_t = int32_t;
// millisecond time
pub type timeMs_t = uint32_t;
// microsecond time
pub type timeUs_t = uint32_t;
#[inline]
unsafe extern "C" fn cmp32(mut a: uint32_t, mut b: uint32_t) -> int32_t {
    return a.wrapping_sub(b) as int32_t;
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
 Created by Dominic Clifton
  */
//#define OSD_SLAVE_DEBUG
// when locked the system ignores requests to enter cli or bootloader mode via serial connection.
#[no_mangle]
pub static mut osdSlaveIsLocked: bool = 0 as libc::c_int != 0;
static mut osdDisplayPort: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
unsafe extern "C" fn osdDrawLogo(mut x: libc::c_int, mut y: libc::c_int) {
    let mut fontOffset: libc::c_char = 160 as libc::c_int as libc::c_char;
    let mut row: libc::c_int = 0 as libc::c_int;
    while row < 4 as libc::c_int {
        let mut column: libc::c_int = 0 as libc::c_int;
        while column < 24 as libc::c_int {
            if fontOffset as libc::c_int != 255 as libc::c_int {
                // FIXME magic number
                let fresh0 = fontOffset;
                fontOffset = fontOffset + 1;
                displayWriteChar(osdDisplayPort, (x + column) as uint8_t,
                                 (y + row) as uint8_t, fresh0 as uint8_t);
            }
            column += 1
        }
        row += 1
    };
}
#[no_mangle]
pub static mut displayDrawScreenQueued: bool = 0 as libc::c_int != 0;
#[no_mangle]
pub static mut receivingScreen: bool = 0 as libc::c_int != 0;
#[no_mangle]
pub static mut stalled: bool = 0 as libc::c_int != 0;
#[no_mangle]
pub unsafe extern "C" fn osdSlaveDrawScreen() {
    displayDrawScreenQueued = 1 as libc::c_int != 0;
}
static mut timeoutAt: uint32_t = 0 as libc::c_int as uint32_t;
#[no_mangle]
pub unsafe extern "C" fn osdSlaveClearScreen() {
    displayClearScreen(osdDisplayPort);
    receivingScreen = 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn osdSlaveWriteChar(x: uint8_t, y: uint8_t,
                                           c: uint8_t) {
    displayWriteChar(osdDisplayPort, x, y, c);
}
#[no_mangle]
pub unsafe extern "C" fn osdSlaveWrite(x: uint8_t, y: uint8_t,
                                       mut s: *const libc::c_char) {
    displayWrite(osdDisplayPort, x, y, s);
}
// msp api
#[no_mangle]
pub unsafe extern "C" fn osdSlaveHeartbeat() {
    timeoutAt =
        micros().wrapping_add((1000 as libc::c_int * 1000 as libc::c_int) as
                                  libc::c_uint);
    if stalled {
        stalled = 0 as libc::c_int != 0;
        displayResync(osdDisplayPort);
    };
}
// init
#[no_mangle]
pub unsafe extern "C" fn osdSlaveInit(mut osdDisplayPortToUse:
                                          *mut displayPort_t) {
    if osdDisplayPortToUse.is_null() {
        return
    } // need max7456 to be ready before using the displayPort API further.
    osdDisplayPort =
        osdDisplayPortToUse; // wait a little for video to stabilise
    delay(100 as libc::c_int as timeMs_t);
    displayClearScreen(osdDisplayPort);
    displayResync(osdDisplayPort);
    delay(100 as libc::c_int as timeMs_t);
    osdDrawLogo(3 as libc::c_int, 1 as libc::c_int);
    let mut string_buffer: [libc::c_char; 30] = [0; 30];
    tfp_sprintf(string_buffer.as_mut_ptr(),
                b"V%s\x00" as *const u8 as *const libc::c_char,
                b"2.5.0\x00" as *const u8 as *const libc::c_char);
    displayWrite(osdDisplayPort, 20 as libc::c_int as uint8_t,
                 6 as libc::c_int as uint8_t, string_buffer.as_mut_ptr());
    displayWrite(osdDisplayPort, 13 as libc::c_int as uint8_t,
                 6 as libc::c_int as uint8_t,
                 b"OSD\x00" as *const u8 as *const libc::c_char);
    displayResync(osdDisplayPort);
}
#[no_mangle]
pub unsafe extern "C" fn osdSlaveInitialized() -> bool {
    return !osdDisplayPort.is_null();
}
// task api
#[no_mangle]
pub unsafe extern "C" fn osdSlaveCheck(mut currentTimeUs: timeUs_t,
                                       mut currentDeltaTimeUs: timeDelta_t)
 -> bool {
    if !stalled && cmp32(currentTimeUs, timeoutAt) > 0 as libc::c_int {
        stalled = 1 as libc::c_int != 0;
        displayWrite(osdDisplayPort, 8 as libc::c_int as uint8_t,
                     12 as libc::c_int as uint8_t,
                     b"WAITING FOR FC\x00" as *const u8 as
                         *const libc::c_char);
        displayResync(osdDisplayPort);
    }
    if !receivingScreen && !displayIsSynced(osdDisplayPort) {
        // queue a screen draw to ensure any remaining characters not written to the screen yet
        // remember that displayDrawScreen() may return WITHOUT having fully updated the screen.
        displayDrawScreenQueued = 1 as libc::c_int != 0
    }
    return receivingScreen as libc::c_int != 0 ||
               displayDrawScreenQueued as libc::c_int != 0;
}
/*
 * Called periodically by the scheduler
 */
#[no_mangle]
pub unsafe extern "C" fn osdSlaveUpdate(mut currentTimeUs: timeUs_t) {
    // don't touch buffers if DMA transaction is in progress
    if displayIsTransferInProgress(osdDisplayPort) { return }
    // MAX7456_DMA_CHANNEL_TX
    if displayDrawScreenQueued {
        displayDrawScreen(osdDisplayPort);
        displayDrawScreenQueued = 0 as libc::c_int != 0;
        receivingScreen = 0 as libc::c_int != 0
    };
}
// OSD_SLAVE
