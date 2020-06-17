use core;
use libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn strlen(_: *const libc::c_char) -> libc::c_ulong;
    #[no_mangle]
    fn displayInit(instance: *mut displayPort_t,
                   vTable: *const displayPortVTable_t);
    #[no_mangle]
    fn mspSerialTxBytesFree() -> uint32_t;
    #[no_mangle]
    fn mspSerialPush(cmd: uint8_t, data: *mut uint8_t, datalen: libc::c_int,
                     direction: mspDirection_e) -> libc::c_int;
    #[no_mangle]
    static mut cliMode: uint8_t;
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
pub type pgn_t = uint16_t;
pub type C2RustUnnamed = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed = 4095;
// function that resets a single parameter group instance
pub type pgResetFunc
    =
    unsafe extern "C" fn(_: *mut libc::c_void, _: libc::c_int) -> ();
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct pgRegistry_s {
    pub pgn: pgn_t,
    pub size: uint16_t,
    pub address: *mut uint8_t,
    pub copy: *mut uint8_t,
    pub ptr: *mut *mut uint8_t,
    pub reset: C2RustUnnamed_0,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union C2RustUnnamed_0 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<unsafe extern "C" fn(_: *mut libc::c_void,
                                          _: libc::c_int) -> ()>,
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct displayPortProfile_s {
    pub colAdjust: int8_t,
    pub rowAdjust: int8_t,
    pub invert: bool,
    pub blackBrightness: uint8_t,
    pub whiteBrightness: uint8_t,
}
pub type displayPortProfile_t = displayPortProfile_s;
pub type mspDirection_e = libc::c_uint;
pub const MSP_DIRECTION_REQUEST: mspDirection_e = 1;
pub const MSP_DIRECTION_REPLY: mspDirection_e = 0;
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
#[inline]
unsafe extern "C" fn displayPortProfileMsp() -> *const displayPortProfile_t {
    return &mut displayPortProfileMsp_System;
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
// no template required since defaults are zero
#[no_mangle]
pub static mut displayPortProfileMsp_System: displayPortProfile_t =
    displayPortProfile_t{colAdjust: 0,
                         rowAdjust: 0,
                         invert: false,
                         blackBrightness: 0,
                         whiteBrightness: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut displayPortProfileMsp_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn: (512i32 | 0i32 << 12i32) as pgn_t,
                             size:
                                 (::core::mem::size_of::<displayPortProfile_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &displayPortProfileMsp_System as
                                     *const displayPortProfile_t as
                                     *mut displayPortProfile_t as
                                     *mut uint8_t,
                             copy:
                                 &displayPortProfileMsp_Copy as
                                     *const displayPortProfile_t as
                                     *mut displayPortProfile_t as
                                     *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{ptr:
                                                     0 as *const libc::c_void
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut displayPortProfileMsp_Copy: displayPortProfile_t =
    displayPortProfile_t{colAdjust: 0,
                         rowAdjust: 0,
                         invert: false,
                         blackBrightness: 0,
                         whiteBrightness: 0,};
static mut mspDisplayPort: displayPort_t =
    displayPort_t{vTable: 0 as *const displayPortVTable_s,
                  device: 0 as *const libc::c_void as *mut libc::c_void,
                  rows: 0,
                  cols: 0,
                  posX: 0,
                  posY: 0,
                  cleared: false,
                  cursorRow: 0,
                  grabCount: 0,};
unsafe extern "C" fn output(mut displayPort: *mut displayPort_t,
                            mut cmd: uint8_t, mut buf: *mut uint8_t,
                            mut len: libc::c_int) -> libc::c_int {
    // FIXME There should be no dependency on the CLI but mspSerialPush doesn't check for cli mode, and can't because it also shouldn't have a dependency on the CLI.
    if cliMode != 0 { return 0i32 }
    return mspSerialPush(cmd, buf, len, MSP_DIRECTION_REPLY);
}
unsafe extern "C" fn heartbeat(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    let mut subcmd: [uint8_t; 1] = [0i32 as uint8_t];
    // heartbeat is used to:
    // a) ensure display is not released by MW OSD software
    // b) prevent OSD Slave boards from displaying a 'disconnected' status.
    return output(displayPort, 182i32 as uint8_t, subcmd.as_mut_ptr(),
                  ::core::mem::size_of::<[uint8_t; 1]>() as libc::c_ulong as
                      libc::c_int);
}
unsafe extern "C" fn grab(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    return heartbeat(displayPort);
}
unsafe extern "C" fn release(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    let mut subcmd: [uint8_t; 1] = [1i32 as uint8_t];
    return output(displayPort, 182i32 as uint8_t, subcmd.as_mut_ptr(),
                  ::core::mem::size_of::<[uint8_t; 1]>() as libc::c_ulong as
                      libc::c_int);
}
unsafe extern "C" fn clearScreen(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    let mut subcmd: [uint8_t; 1] = [2i32 as uint8_t];
    return output(displayPort, 182i32 as uint8_t, subcmd.as_mut_ptr(),
                  ::core::mem::size_of::<[uint8_t; 1]>() as libc::c_ulong as
                      libc::c_int);
}
unsafe extern "C" fn drawScreen(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    let mut subcmd: [uint8_t; 1] = [4i32 as uint8_t];
    return output(displayPort, 182i32 as uint8_t, subcmd.as_mut_ptr(),
                  ::core::mem::size_of::<[uint8_t; 1]>() as libc::c_ulong as
                      libc::c_int);
}
unsafe extern "C" fn screenSize(mut displayPort: *const displayPort_t)
 -> libc::c_int {
    return (*displayPort).rows as libc::c_int *
               (*displayPort).cols as libc::c_int;
}
unsafe extern "C" fn writeString(mut displayPort: *mut displayPort_t,
                                 mut col: uint8_t, mut row: uint8_t,
                                 mut string: *const libc::c_char)
 -> libc::c_int {
    // FIXME move this
    let mut buf: [uint8_t; 34] = [0; 34];
    let mut len: libc::c_int = strlen(string) as libc::c_int;
    if len >= 30i32 { len = 30i32 }
    buf[0] = 3i32 as uint8_t;
    buf[1] = row;
    buf[2] = col;
    buf[3] = 0i32 as uint8_t;
    memcpy(&mut *buf.as_mut_ptr().offset(4) as *mut uint8_t as
               *mut libc::c_void, string as *const libc::c_void,
           len as libc::c_ulong);
    return output(displayPort, 182i32 as uint8_t, buf.as_mut_ptr(),
                  len + 4i32);
}
unsafe extern "C" fn writeChar(mut displayPort: *mut displayPort_t,
                               mut col: uint8_t, mut row: uint8_t,
                               mut c: uint8_t) -> libc::c_int {
    let mut buf: [libc::c_char; 2] = [0; 2];
    buf[0] = c as libc::c_char;
    buf[1] = 0i32 as libc::c_char;
    return writeString(displayPort, col, row, buf.as_mut_ptr());
    // !!TODO - check if there is a direct MSP command to do this
}
unsafe extern "C" fn isTransferInProgress(mut displayPort:
                                              *const displayPort_t) -> bool {
    return 0i32 != 0; // XXX Will reflect NTSC/PAL in the future
}
unsafe extern "C" fn isSynced(mut displayPort: *const displayPort_t) -> bool {
    return 1i32 != 0;
}
unsafe extern "C" fn resync(mut displayPort: *mut displayPort_t) {
    (*displayPort).rows =
        (13i32 + (*displayPortProfileMsp()).rowAdjust as libc::c_int) as
            uint8_t;
    (*displayPort).cols =
        (30i32 + (*displayPortProfileMsp()).colAdjust as libc::c_int) as
            uint8_t;
    drawScreen(displayPort);
}
unsafe extern "C" fn txBytesFree(mut displayPort: *const displayPort_t)
 -> uint32_t {
    return mspSerialTxBytesFree();
}
static mut mspDisplayPortVTable: displayPortVTable_t =
    unsafe {
        {
            let mut init =
                displayPortVTable_s{grab:
                                        Some(grab as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> libc::c_int),
                                    release:
                                        Some(release as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> libc::c_int),
                                    clearScreen:
                                        Some(clearScreen as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> libc::c_int),
                                    drawScreen:
                                        Some(drawScreen as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> libc::c_int),
                                    screenSize:
                                        Some(screenSize as
                                                 unsafe extern "C" fn(_:
                                                                          *const displayPort_t)
                                                     -> libc::c_int),
                                    writeString:
                                        Some(writeString as
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
                                        Some(writeChar as
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
                                        Some(isTransferInProgress as
                                                 unsafe extern "C" fn(_:
                                                                          *const displayPort_t)
                                                     -> bool),
                                    heartbeat:
                                        Some(heartbeat as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> libc::c_int),
                                    resync:
                                        Some(resync as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> ()),
                                    isSynced:
                                        Some(isSynced as
                                                 unsafe extern "C" fn(_:
                                                                          *const displayPort_t)
                                                     -> bool),
                                    txBytesFree:
                                        Some(txBytesFree as
                                                 unsafe extern "C" fn(_:
                                                                          *const displayPort_t)
                                                     -> uint32_t),};
            init
        }
    };
#[no_mangle]
pub unsafe extern "C" fn displayPortMspInit() -> *mut displayPort_s {
    displayInit(&mut mspDisplayPort, &mspDisplayPortVTable);
    resync(&mut mspDisplayPort);
    return &mut mspDisplayPort;
}
// USE_MSP_DISPLAYPORT
