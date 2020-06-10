use ::libc;
extern "C" {
    #[no_mangle]
    fn displayInit(instance: *mut displayPort_t,
                   vTable: *const displayPortVTable_t);
    // Airware 1.20
    //#define SPEKTRUM_SRXL_TEXTGEN_BUFFER_COLS 13 // Airware 1.21
    #[no_mangle]
    fn spektrumTmTextGenPutChar(col: uint8_t, row: uint8_t, c: libc::c_char)
     -> libc::c_int;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
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
// CMS state
// displayPort_t is used as a parameter group in 'displayport_msp.h' and 'displayport_max7456`.h'. Treat accordingly!
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
// Device management
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
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
pub static mut srxlDisplayPort: displayPort_t =
    displayPort_t{vTable: 0 as *const displayPortVTable_s,
                  device: 0 as *const libc::c_void as *mut libc::c_void,
                  rows: 0,
                  cols: 0,
                  posX: 0,
                  posY: 0,
                  cleared: false,
                  cursorRow: 0,
                  grabCount: 0,};
unsafe extern "C" fn srxlDrawScreen(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    return 0 as libc::c_int;
}
unsafe extern "C" fn srxlScreenSize(mut displayPort: *const displayPort_t)
 -> libc::c_int {
    return (*displayPort).rows as libc::c_int *
               (*displayPort).cols as libc::c_int;
}
unsafe extern "C" fn srxlWriteChar(mut displayPort: *mut displayPort_t,
                                   mut col: uint8_t, mut row: uint8_t,
                                   mut c: uint8_t) -> libc::c_int {
    return spektrumTmTextGenPutChar(col, row, c as libc::c_char);
}
unsafe extern "C" fn srxlWriteString(mut displayPort: *mut displayPort_t,
                                     mut col: uint8_t, mut row: uint8_t,
                                     mut s: *const libc::c_char)
 -> libc::c_int {
    while *s != 0 {
        let fresh0 = col;
        col = col.wrapping_add(1);
        let fresh1 = s;
        s = s.offset(1);
        srxlWriteChar(displayPort, fresh0, row, *fresh1 as uint8_t);
    }
    return 0 as libc::c_int;
}
unsafe extern "C" fn srxlClearScreen(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    let mut row: libc::c_int = 0 as libc::c_int;
    while row < 9 as libc::c_int {
        let mut col: libc::c_int = 0 as libc::c_int;
        while col < 12 as libc::c_int {
            srxlWriteChar(displayPort, col as uint8_t, row as uint8_t,
                          ' ' as i32 as uint8_t);
            col += 1
        }
        row += 1
    }
    srxlWriteString(displayPort, 1 as libc::c_int as uint8_t,
                    0 as libc::c_int as uint8_t,
                    b"BETAFLIGHT\x00" as *const u8 as *const libc::c_char);
    if (*displayPort).grabCount as libc::c_int == 0 as libc::c_int {
        srxlWriteString(displayPort, 0 as libc::c_int as uint8_t,
                        2 as libc::c_int as uint8_t,
                        b"MENU:THR MID\x00" as *const u8 as
                            *const libc::c_char);
        srxlWriteString(displayPort, 2 as libc::c_int as uint8_t,
                        3 as libc::c_int as uint8_t,
                        b"+ YAW LEFT\x00" as *const u8 as
                            *const libc::c_char);
        srxlWriteString(displayPort, 2 as libc::c_int as uint8_t,
                        4 as libc::c_int as uint8_t,
                        b"+ PITCH UP\x00" as *const u8 as
                            *const libc::c_char);
    }
    return 0 as libc::c_int;
}
unsafe extern "C" fn srxlIsTransferInProgress(mut displayPort:
                                                  *const displayPort_t)
 -> bool {
    return 0 as libc::c_int != 0;
}
unsafe extern "C" fn srxlIsSynced(mut displayPort: *const displayPort_t)
 -> bool {
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn srxlHeartbeat(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    return 0 as libc::c_int;
}
unsafe extern "C" fn srxlResync(mut displayPort: *mut displayPort_t) { }
unsafe extern "C" fn srxlTxBytesFree(mut displayPort: *const displayPort_t)
 -> uint32_t {
    return 4294967295 as libc::c_uint;
}
unsafe extern "C" fn srxlGrab(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    (*displayPort).grabCount = 1 as libc::c_int as int8_t;
    return (*displayPort).grabCount as libc::c_int;
}
unsafe extern "C" fn srxlRelease(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    (*displayPort).grabCount = 0 as libc::c_int as int8_t;
    let mut cnt: libc::c_int = (*displayPort).grabCount as libc::c_int;
    srxlClearScreen(displayPort);
    return cnt;
}
static mut srxlVTable: displayPortVTable_t =
    unsafe {
        {
            let mut init =
                displayPortVTable_s{grab:
                                        Some(srxlGrab as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> libc::c_int),
                                    release:
                                        Some(srxlRelease as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> libc::c_int),
                                    clearScreen:
                                        Some(srxlClearScreen as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> libc::c_int),
                                    drawScreen:
                                        Some(srxlDrawScreen as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> libc::c_int),
                                    screenSize:
                                        Some(srxlScreenSize as
                                                 unsafe extern "C" fn(_:
                                                                          *const displayPort_t)
                                                     -> libc::c_int),
                                    writeString:
                                        Some(srxlWriteString as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t,
                                                                      _:
                                                                          uint8_t,
                                                                      _:
                                                                          uint8_t,
                                                                      _:
                                                                          *const libc::c_char)
                                                     -> libc::c_int),
                                    writeChar:
                                        Some(srxlWriteChar as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t,
                                                                      _:
                                                                          uint8_t,
                                                                      _:
                                                                          uint8_t,
                                                                      _:
                                                                          uint8_t)
                                                     -> libc::c_int),
                                    isTransferInProgress:
                                        Some(srxlIsTransferInProgress as
                                                 unsafe extern "C" fn(_:
                                                                          *const displayPort_t)
                                                     -> bool),
                                    heartbeat:
                                        Some(srxlHeartbeat as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> libc::c_int),
                                    resync:
                                        Some(srxlResync as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> ()),
                                    isSynced:
                                        Some(srxlIsSynced as
                                                 unsafe extern "C" fn(_:
                                                                          *const displayPort_t)
                                                     -> bool),
                                    txBytesFree:
                                        Some(srxlTxBytesFree as
                                                 unsafe extern "C" fn(_:
                                                                          *const displayPort_t)
                                                     -> uint32_t),};
            init
        }
    };
#[no_mangle]
pub unsafe extern "C" fn displayPortSrxlInit() -> *mut displayPort_t {
    srxlDisplayPort.device = 0 as *mut libc::c_void;
    displayInit(&mut srxlDisplayPort, &srxlVTable);
    srxlDisplayPort.rows = 9 as libc::c_int as uint8_t;
    srxlDisplayPort.cols = 12 as libc::c_int as uint8_t;
    return &mut srxlDisplayPort;
}
