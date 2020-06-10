use ::libc;
extern "C" {
    #[no_mangle]
    fn displayInit(instance: *mut displayPort_t,
                   vTable: *const displayPortVTable_t);
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
    /* * PAL or NTSC, value is number of chars total */
    #[no_mangle]
    static mut maxScreenSize: uint16_t;
    #[no_mangle]
    fn max7456Init(max7456Config_0: *const max7456Config_s,
                   vcdProfile: *const vcdProfile_s, cpuOverclock: bool)
     -> bool;
    #[no_mangle]
    fn max7456Invert(invert: bool);
    #[no_mangle]
    fn max7456Brightness(black: uint8_t, white: uint8_t);
    #[no_mangle]
    fn max7456DrawScreen();
    #[no_mangle]
    fn max7456GetRowsCount() -> uint8_t;
    #[no_mangle]
    fn max7456Write(x: uint8_t, y: uint8_t, buff: *const libc::c_char);
    #[no_mangle]
    fn max7456WriteChar(x: uint8_t, y: uint8_t, c: uint8_t);
    #[no_mangle]
    fn max7456ClearScreen();
    #[no_mangle]
    fn max7456RefreshAll();
    #[no_mangle]
    fn max7456DmaInProgress() -> bool;
    #[no_mangle]
    fn max7456BuffersSynced() -> bool;
    #[no_mangle]
    static mut systemConfig_System: systemConfig_t;
    #[no_mangle]
    static mut resumeRefreshAt: timeUs_t;
    #[no_mangle]
    fn osdResetAlarms();
    #[no_mangle]
    static mut max7456Config_System: max7456Config_t;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct displayPortProfile_s {
    pub colAdjust: int8_t,
    pub rowAdjust: int8_t,
    pub invert: bool,
    pub blackBrightness: uint8_t,
    pub whiteBrightness: uint8_t,
}
pub type displayPortProfile_t = displayPortProfile_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct vcdProfile_s {
    pub video_system: uint8_t,
    pub h_offset: int8_t,
    pub v_offset: int8_t,
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct max7456Config_s {
    pub clockConfig: uint8_t,
    pub csTag: ioTag_t,
    pub spiDevice: uint8_t,
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
// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
pub type ioTag_t = uint8_t;
pub type pgn_t = uint16_t;
pub type C2RustUnnamed = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed = 4095;
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
    pub reset: C2RustUnnamed_0,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_0 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<pgResetFunc>,
}
pub type pgRegistry_t = pgRegistry_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct systemConfig_s {
    pub pidProfileIndex: uint8_t,
    pub activeRateProfile: uint8_t,
    pub debug_mode: uint8_t,
    pub task_statistics: uint8_t,
    pub rateProfile6PosSwitch: uint8_t,
    pub cpu_overclock: uint8_t,
    pub powerOnArmingGraceTime: uint8_t,
    pub boardIdentifier: [libc::c_char; 6],
}
pub type systemConfig_t = systemConfig_s;
pub type vcdProfile_t = vcdProfile_s;
// in seconds
// microsecond time
pub type timeUs_t = uint32_t;
pub type max7456Config_t = max7456Config_s;
#[inline]
unsafe extern "C" fn systemConfig() -> *const systemConfig_t {
    return &mut systemConfig_System;
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
#[inline]
unsafe extern "C" fn displayPortProfileMax7456()
 -> *const displayPortProfile_t {
    return &mut displayPortProfileMax7456_System;
}
#[inline]
unsafe extern "C" fn max7456Config() -> *const max7456Config_t {
    return &mut max7456Config_System;
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
#[no_mangle]
pub static mut max7456DisplayPort: displayPort_t =
    displayPort_t{vTable: 0 as *const displayPortVTable_s,
                  device: 0 as *const libc::c_void as *mut libc::c_void,
                  rows: 0,
                  cols: 0,
                  posX: 0,
                  posY: 0,
                  cleared: false,
                  cursorRow: 0,
                  grabCount: 0,};
#[no_mangle]
pub static mut displayPortProfileMax7456_System: displayPortProfile_t =
    displayPortProfile_t{colAdjust: 0,
                         rowAdjust: 0,
                         invert: false,
                         blackBrightness: 0,
                         whiteBrightness: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut displayPortProfileMax7456_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (513 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<displayPortProfile_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &displayPortProfileMax7456_System as
                                     *const displayPortProfile_t as
                                     *mut displayPortProfile_t as
                                     *mut uint8_t,
                             copy:
                                 &displayPortProfileMax7456_Copy as
                                     *const displayPortProfile_t as
                                     *mut displayPortProfile_t as
                                     *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut displayPortProfile_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<pgResetFunc>>(Some(pgResetFn_displayPortProfileMax7456
                                                                                                            as
                                                                                                            unsafe extern "C" fn(_:
                                                                                                                                     *mut displayPortProfile_t)
                                                                                                                ->
                                                                                                                    ())),},};
            init
        }
    };
#[no_mangle]
pub static mut displayPortProfileMax7456_Copy: displayPortProfile_t =
    displayPortProfile_t{colAdjust: 0,
                         rowAdjust: 0,
                         invert: false,
                         blackBrightness: 0,
                         whiteBrightness: 0,};
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_displayPortProfileMax7456(mut displayPortProfile:
                                                                 *mut displayPortProfile_t) {
    (*displayPortProfile).colAdjust = 0 as libc::c_int as int8_t;
    (*displayPortProfile).rowAdjust = 0 as libc::c_int as int8_t;
    // Set defaults as per MAX7456 datasheet
    (*displayPortProfile).invert = 0 as libc::c_int != 0;
    (*displayPortProfile).blackBrightness = 0 as libc::c_int as uint8_t;
    (*displayPortProfile).whiteBrightness = 2 as libc::c_int as uint8_t;
}
unsafe extern "C" fn grab(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    // FIXME this should probably not have a dependency on the OSD or OSD slave code
    osdResetAlarms();
    resumeRefreshAt = 0 as libc::c_int as timeUs_t;
    return 0 as libc::c_int;
}
unsafe extern "C" fn release(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    return 0 as libc::c_int;
}
unsafe extern "C" fn clearScreen(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    max7456Invert((*displayPortProfileMax7456()).invert);
    max7456Brightness((*displayPortProfileMax7456()).blackBrightness,
                      (*displayPortProfileMax7456()).whiteBrightness);
    max7456ClearScreen();
    return 0 as libc::c_int;
}
unsafe extern "C" fn drawScreen(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    max7456DrawScreen();
    return 0 as libc::c_int;
}
unsafe extern "C" fn screenSize(mut displayPort: *const displayPort_t)
 -> libc::c_int {
    return maxScreenSize as libc::c_int;
}
unsafe extern "C" fn writeString(mut displayPort: *mut displayPort_t,
                                 mut x: uint8_t, mut y: uint8_t,
                                 mut s: *const libc::c_char) -> libc::c_int {
    max7456Write(x, y, s);
    return 0 as libc::c_int;
}
unsafe extern "C" fn writeChar(mut displayPort: *mut displayPort_t,
                               mut x: uint8_t, mut y: uint8_t, mut c: uint8_t)
 -> libc::c_int {
    max7456WriteChar(x, y, c);
    return 0 as libc::c_int;
}
unsafe extern "C" fn isTransferInProgress(mut displayPort:
                                              *const displayPort_t) -> bool {
    return max7456DmaInProgress();
}
unsafe extern "C" fn isSynced(mut displayPort: *const displayPort_t) -> bool {
    return max7456BuffersSynced();
}
unsafe extern "C" fn resync(mut displayPort: *mut displayPort_t) {
    max7456RefreshAll();
    (*displayPort).rows =
        (max7456GetRowsCount() as libc::c_int +
             (*displayPortProfileMax7456()).rowAdjust as libc::c_int) as
            uint8_t;
    (*displayPort).cols =
        (30 as libc::c_int +
             (*displayPortProfileMax7456()).colAdjust as libc::c_int) as
            uint8_t;
}
unsafe extern "C" fn heartbeat(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    return 0 as libc::c_int;
}
unsafe extern "C" fn txBytesFree(mut displayPort: *const displayPort_t)
 -> uint32_t {
    return 4294967295 as libc::c_uint;
}
static mut max7456VTable: displayPortVTable_t =
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
pub unsafe extern "C" fn max7456DisplayPortInit(mut vcdProfile:
                                                    *const vcdProfile_t)
 -> *mut displayPort_t {
    if !max7456Init(max7456Config(), vcdProfile,
                    (*systemConfig()).cpu_overclock != 0) {
        return 0 as *mut displayPort_t
    }
    displayInit(&mut max7456DisplayPort, &max7456VTable);
    resync(&mut max7456DisplayPort);
    return &mut max7456DisplayPort;
}
// USE_MAX7456
