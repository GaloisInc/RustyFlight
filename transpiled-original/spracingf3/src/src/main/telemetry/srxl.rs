use core;
use libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn memcmp(_: *const libc::c_void, _: *const libc::c_void,
              _: libc::c_ulong) -> libc::c_int;
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
    static mut cmsInMenu: bool;
    #[no_mangle]
    fn crc16_ccitt_sbuf_append(dst: *mut sbuf_s, start: *mut uint8_t);
    #[no_mangle]
    fn sbufWriteU8(dst: *mut sbuf_t, val: uint8_t);
    #[no_mangle]
    fn sbufWriteU16(dst: *mut sbuf_t, val: uint16_t);
    #[no_mangle]
    fn sbufWriteU16BigEndian(dst: *mut sbuf_t, val: uint16_t);
    #[no_mangle]
    fn sbufFill(dst: *mut sbuf_t, data: uint8_t, len: libc::c_int);
    #[no_mangle]
    fn sbufWriteData(dst: *mut sbuf_t, data: *const libc::c_void,
                     len: libc::c_int);
    #[no_mangle]
    fn sbufBytesRemaining(buf: *mut sbuf_t) -> libc::c_int;
    #[no_mangle]
    fn sbufPtr(buf: *mut sbuf_t) -> *mut uint8_t;
    #[no_mangle]
    fn sbufSwitchToReader(buf: *mut sbuf_t, base: *mut uint8_t);
    #[no_mangle]
    fn srxlRxWriteTelemetryData(data: *const libc::c_void, len: libc::c_int);
    #[no_mangle]
    fn srxlRxIsActive() -> bool;
    #[no_mangle]
    static spek2commonBand: [uint8_t; 5];
    #[no_mangle]
    static vtxTrampPi: [uint8_t; 8];
    #[no_mangle]
    static vtxSaPi: [uint8_t; 8];
    #[no_mangle]
    fn getBatteryVoltage() -> uint16_t;
    #[no_mangle]
    fn getAmperage() -> int32_t;
    #[no_mangle]
    fn getMAhDrawn() -> int32_t;
    #[no_mangle]
    fn vtxCommonDevice() -> *mut vtxDevice_t;
    #[no_mangle]
    fn vtxCommonGetDeviceType(vtxDevice: *const vtxDevice_t) -> vtxDevType_e;
    #[no_mangle]
    fn vtxCommonGetBandAndChannel(vtxDevice: *const vtxDevice_t,
                                  pBand: *mut uint8_t, pChannel: *mut uint8_t)
     -> bool;
    #[no_mangle]
    fn vtxCommonGetPowerIndex(vtxDevice: *const vtxDevice_t,
                              pIndex: *mut uint8_t) -> bool;
    #[no_mangle]
    fn vtxCommonGetPitMode(vtxDevice: *const vtxDevice_t,
                           pOnOff: *mut uint8_t) -> bool;
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
    //min freq in MHz
    //max freq in MHz
    #[no_mangle]
    static trampPowerTable: [uint16_t; 5];
    #[no_mangle]
    static mut saPowerTable: [saPowerTable_t; 0];
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct sbuf_s {
    pub ptr: *mut uint8_t,
    pub end: *mut uint8_t,
}
pub type sbuf_t = sbuf_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct spektrumVtx_t {
    pub band: uint8_t,
    pub channel: uint8_t,
    pub power: uint8_t,
    pub region: uint8_t,
    pub pitMode: uint8_t,
    pub powerValue: uint16_t,
}
pub type srxlScheduleFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut sbuf_t, _: timeUs_t) -> bool>;
pub type vtxDevice_t = vtxDevice_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct vtxDevice_s {
    pub vTable: *const vtxVTable_s,
    pub capability: vtxDeviceCapability_t,
    pub frequencyTable: *mut uint16_t,
    pub bandNames: *mut *mut libc::c_char,
    pub channelNames: *mut *mut libc::c_char,
    pub powerNames: *mut *mut libc::c_char,
    pub frequency: uint16_t,
    pub band: uint8_t,
    pub channel: uint8_t,
    pub powerIndex: uint8_t,
    pub pitMode: uint8_t,
}
// VTX magic numbers
// RTC6705 RF Power index "---", 25 or 200 mW
// SmartAudio "---", 25, 200, 500, 800 mW
// Tramp "---", 25, 100, 200, 400, 600 mW
pub type vtxDeviceCapability_t = vtxDeviceCapability_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct vtxDeviceCapability_s {
    pub bandCount: uint8_t,
    pub channelCount: uint8_t,
    pub powerCount: uint8_t,
    pub filler: uint8_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct vtxVTable_s {
    pub process: Option<unsafe extern "C" fn(_: *mut vtxDevice_t, _: timeUs_t)
                            -> ()>,
    pub getDeviceType: Option<unsafe extern "C" fn(_: *const vtxDevice_t)
                                  -> vtxDevType_e>,
    pub isReady: Option<unsafe extern "C" fn(_: *const vtxDevice_t) -> bool>,
    pub setBandAndChannel: Option<unsafe extern "C" fn(_: *mut vtxDevice_t,
                                                       _: uint8_t, _: uint8_t)
                                      -> ()>,
    pub setPowerByIndex: Option<unsafe extern "C" fn(_: *mut vtxDevice_t,
                                                     _: uint8_t) -> ()>,
    pub setPitMode: Option<unsafe extern "C" fn(_: *mut vtxDevice_t,
                                                _: uint8_t) -> ()>,
    pub setFrequency: Option<unsafe extern "C" fn(_: *mut vtxDevice_t,
                                                  _: uint16_t) -> ()>,
    pub getBandAndChannel: Option<unsafe extern "C" fn(_: *const vtxDevice_t,
                                                       _: *mut uint8_t,
                                                       _: *mut uint8_t)
                                      -> bool>,
    pub getPowerIndex: Option<unsafe extern "C" fn(_: *const vtxDevice_t,
                                                   _: *mut uint8_t) -> bool>,
    pub getPitMode: Option<unsafe extern "C" fn(_: *const vtxDevice_t,
                                                _: *mut uint8_t) -> bool>,
    pub getFrequency: Option<unsafe extern "C" fn(_: *const vtxDevice_t,
                                                  _: *mut uint16_t) -> bool>,
}
pub type vtxDevType_e = libc::c_uint;
pub const VTXDEV_UNKNOWN: vtxDevType_e = 255;
pub const VTXDEV_TRAMP: vtxDevType_e = 4;
pub const VTXDEV_SMARTAUDIO: vtxDevType_e = 3;
pub const VTXDEV_RTC6705: vtxDevType_e = 1;
pub const VTXDEV_UNSUPPORTED: vtxDevType_e = 0;
pub type saPowerTable_t = saPowerTable_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct saPowerTable_s {
    pub rfpower: libc::c_int,
    pub valueV1: int16_t,
    pub valueV2: int16_t,
}
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
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
static mut srxlTelemetryEnabled: bool = false;
static mut srxlFrame: [uint8_t; 21] = [0; 21];
static mut srxlTelemetryNow: bool = 0i32 != 0;
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
pub unsafe extern "C" fn srxlCollectTelemetryNow() {
    srxlTelemetryNow =
        1i32 !=
            0; // start at byte 3, since CRC does not include device address and packet length
}
unsafe extern "C" fn srxlInitializeFrame(mut dst: *mut sbuf_t) {
    (*dst).ptr = srxlFrame.as_mut_ptr();
    (*dst).end =
        &mut *srxlFrame.as_mut_ptr().offset((::core::mem::size_of::<[uint8_t; 21]>()
                                                 as
                                                 libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>()
                                                                                 as
                                                                                 libc::c_ulong)
                                                as isize) as *mut uint8_t;
    sbufWriteU8(dst, 0xa5i32 as uint8_t);
    sbufWriteU8(dst, 0x80i32 as uint8_t);
    sbufWriteU8(dst, 0x15i32 as uint8_t);
}
unsafe extern "C" fn srxlFinalize(mut dst: *mut sbuf_t) {
    crc16_ccitt_sbuf_append(dst, &mut *srxlFrame.as_mut_ptr().offset(3));
    sbufSwitchToReader(dst, srxlFrame.as_mut_ptr());
    // write the telemetry frame to the receiver.
    srxlRxWriteTelemetryData(sbufPtr(dst) as *const libc::c_void,
                             sbufBytesRemaining(dst)); // Clear remainder
}
#[no_mangle]
pub unsafe extern "C" fn srxlFrameQos(mut dst: *mut sbuf_t,
                                      mut currentTimeUs: timeUs_t) -> bool {
    sbufWriteU8(dst, 0x7fi32 as uint8_t); // pulse leading edges
    sbufWriteU8(dst, 0i32 as uint8_t); // vbat is in units of 0.1V
    sbufFill(dst, 0xffi32 as uint8_t, 14i32); // temperature
    return 1i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn srxlFrameRpm(mut dst: *mut sbuf_t,
                                      mut currentTimeUs: timeUs_t) -> bool {
    sbufWriteU8(dst, 0x7ei32 as uint8_t);
    sbufWriteU8(dst, 0i32 as uint8_t);
    sbufWriteU16BigEndian(dst, 0xffffi32 as uint16_t);
    sbufWriteU16BigEndian(dst,
                          (getBatteryVoltage() as libc::c_int * 10i32) as
                              uint16_t);
    sbufWriteU16BigEndian(dst, 0x7fffi32 as uint16_t);
    sbufFill(dst, 0xffi32 as uint8_t, 8i32);
    return 1i32 != 0;
}
// 2s
#[no_mangle]
pub unsafe extern "C" fn srxlFrameFlightPackCurrent(mut dst: *mut sbuf_t,
                                                    mut currentTimeUs:
                                                        timeUs_t) -> bool {
    let mut amps: uint16_t = (getAmperage() / 10i32) as uint16_t; // temp A
    let mut mah: uint16_t = getMAhDrawn() as uint16_t; // Amps B
    static mut sentAmps: uint16_t = 0; // mAH B
    static mut sentMah: uint16_t = 0; // temp B
    static mut lastTimeSentFPmAh: timeUs_t = 0i32 as timeUs_t;
    let mut keepAlive: timeUs_t =
        currentTimeUs.wrapping_sub(lastTimeSentFPmAh);
    if amps as libc::c_int != sentAmps as libc::c_int ||
           mah as libc::c_int != sentMah as libc::c_int ||
           keepAlive > 2000000i32 as libc::c_uint {
        sbufWriteU8(dst, 0x34i32 as uint8_t);
        sbufWriteU8(dst, 0i32 as uint8_t);
        sbufWriteU16(dst, amps);
        sbufWriteU16(dst, mah);
        sbufWriteU16(dst, 0x7fffi32 as uint16_t);
        sbufWriteU16(dst, 0x7fffi32 as uint16_t);
        sbufWriteU16(dst, 0x7fffi32 as uint16_t);
        sbufWriteU16(dst, 0x7fffi32 as uint16_t);
        sbufWriteU16(dst, 0xffffi32 as uint16_t);
        sentAmps = amps;
        sentMah = mah;
        lastTimeSentFPmAh = currentTimeUs;
        return 1i32 != 0
    }
    return 0i32 != 0;
}
// Text Generator COLS
/*
typedef struct
{
    UINT8       identifier;
    UINT8       sID;               // Secondary ID
    UINT8       lineNumber;        // Line number to display (0 = title, 1-8 for general, 254 = Refresh backlight, 255 = Erase all text on screen)
    char        text[13];          // 0-terminated text when < 13 chars
} STRU_SPEKTRUM_SRXL_TEXTGEN;
*/
static mut srxlTextBuff: [[libc::c_char; 13]; 9] = [[0; 13]; 9];
static mut lineSent: [bool; 9] = [false; 9];
// Airware 1.20
//#define SPEKTRUM_SRXL_TEXTGEN_BUFFER_COLS 13 // Airware 1.21
//**************************************************************************
// API Running in external client task context. E.g. in the CMS task
#[no_mangle]
pub unsafe extern "C" fn spektrumTmTextGenPutChar(mut col: uint8_t,
                                                  mut row: uint8_t,
                                                  mut c: libc::c_char)
 -> libc::c_int {
    if (row as libc::c_int) < 9i32 && (col as libc::c_int) < 12i32 {
        // Only update and force a tm transmision if something has actually changed.
        if srxlTextBuff[row as usize][col as usize] as libc::c_int !=
               c as libc::c_int {
            srxlTextBuff[row as usize][col as usize] = c;
            lineSent[row as usize] = 0i32 != 0
        }
    }
    return 0i32;
}
//**************************************************************************
#[no_mangle]
pub unsafe extern "C" fn srxlFrameText(mut dst: *mut sbuf_t,
                                       mut currentTimeUs: timeUs_t) -> bool {
    static mut lineNo: uint8_t = 0i32 as uint8_t;
    let mut lineCount: libc::c_int = 0i32;
    // Skip already sent lines...
    while lineSent[lineNo as usize] as libc::c_int != 0 && lineCount < 9i32 {
        lineNo = ((lineNo as libc::c_int + 1i32) % 9i32) as uint8_t;
        lineCount += 1
    }
    sbufWriteU8(dst, 0xci32 as uint8_t);
    sbufWriteU8(dst, 0i32 as uint8_t);
    sbufWriteU8(dst, lineNo);
    sbufWriteData(dst,
                  srxlTextBuff[lineNo as usize].as_mut_ptr() as
                      *const libc::c_void, 13i32);
    lineSent[lineNo as usize] = 1i32 != 0;
    lineNo = ((lineNo as libc::c_int + 1i32) % 9i32) as uint8_t;
    // Always send something, Always one user frame after the two mandatory frames
    // I.e. All of the three frame prep routines QOS, RPM, TEXT should always return true
    // too keep the "Waltz" sequence intact.
    return 1i32 != 0;
}
static mut vtxDeviceType: uint8_t = 0;
unsafe extern "C" fn collectVtxTmData(mut vtx: *mut spektrumVtx_t) {
    let mut vtxDevice: *const vtxDevice_t = vtxCommonDevice();
    vtxDeviceType = vtxCommonGetDeviceType(vtxDevice) as uint8_t;
    // Collect all data from VTX, if VTX is ready
    if vtxDevice.is_null() ||
           !(vtxCommonGetBandAndChannel(vtxDevice, &mut (*vtx).band,
                                        &mut (*vtx).channel) as libc::c_int !=
                 0 &&
                 vtxCommonGetPitMode(vtxDevice, &mut (*vtx).pitMode) as
                     libc::c_int != 0 &&
                 vtxCommonGetPowerIndex(vtxDevice, &mut (*vtx).power) as
                     libc::c_int != 0) {
        (*vtx).band = 0i32 as uint8_t;
        (*vtx).channel = 0i32 as uint8_t;
        (*vtx).power = 0i32 as uint8_t;
        (*vtx).pitMode = 0i32 as uint8_t
    }
    (*vtx).powerValue = 0i32 as uint16_t;
    (*vtx).region = 0xffi32 as uint8_t;
}
// Reverse lookup, device power index to Spektrum power range index.
unsafe extern "C" fn convertVtxPower(mut vtx: *mut spektrumVtx_t) {
    let mut powerIndexTable: *const uint8_t =
        0 as
            *const uint8_t; // Lookup the device power value, 0-based table vs 1-based index. Doh.
    match vtxDeviceType as libc::c_int {
        4 => {
            powerIndexTable =
                vtxTrampPi.as_ptr(); // Translate device power index to Spektrum power index.
            (*vtx).powerValue =
                trampPowerTable[((*vtx).power as libc::c_int - 1i32) as usize]
        }
        3 => {
            powerIndexTable = vtxSaPi.as_ptr();
            (*vtx).powerValue =
                (*saPowerTable.as_mut_ptr().offset(((*vtx).power as
                                                        libc::c_int - 1i32) as
                                                       isize)).rfpower as
                    uint16_t
        }
        255 | 0 | _ => { }
    }
    if !powerIndexTable.is_null() {
        let mut i: libc::c_int = 0i32;
        while i < 8i32 {
            if *powerIndexTable.offset(i as isize) as libc::c_int >=
                   (*vtx).power as libc::c_int {
                (*vtx).power = i as uint8_t;
                break ;
            } else { i += 1 }
        }
    };
}
unsafe extern "C" fn convertVtxTmData(mut vtx: *mut spektrumVtx_t) {
    // Convert from internal band indexes to Spektrum indexes
    let mut i: libc::c_int = 0i32;
    while i < 5i32 {
        if spek2commonBand[i as usize] as libc::c_int ==
               (*vtx).band as libc::c_int {
            (*vtx).band = i as uint8_t;
            break ;
        } else { i += 1 }
    }
    // De-bump channel no 1 based interally, 0-based in Spektrum.
    (*vtx).channel = (*vtx).channel.wrapping_sub(1);
    // Convert Power index to Spektrum ranges, different per brand.
    convertVtxPower(vtx);
}
// uS
unsafe extern "C" fn srxlFrameVTX(mut dst: *mut sbuf_t,
                                  mut currentTimeUs: timeUs_t) -> bool {
    static mut lastTimeSentVtx: timeUs_t = 0i32 as timeUs_t;
    static mut vtxSent: spektrumVtx_t =
        spektrumVtx_t{band: 0,
                      channel: 0,
                      power: 0,
                      region: 0,
                      pitMode: 0,
                      powerValue: 0,};
    let mut vtx: spektrumVtx_t =
        spektrumVtx_t{band: 0,
                      channel: 0,
                      power: 0,
                      region: 0,
                      pitMode: 0,
                      powerValue: 0,};
    collectVtxTmData(&mut vtx);
    if vtxDeviceType as libc::c_int != VTXDEV_UNKNOWN as libc::c_int &&
           vtxDeviceType as libc::c_int != VTXDEV_UNSUPPORTED as libc::c_int {
        convertVtxTmData(&mut vtx);
        if memcmp(&mut vtxSent as *mut spektrumVtx_t as *const libc::c_void,
                  &mut vtx as *mut spektrumVtx_t as *const libc::c_void,
                  ::core::mem::size_of::<spektrumVtx_t>() as libc::c_ulong) !=
               0i32 ||
               currentTimeUs.wrapping_sub(lastTimeSentVtx) >
                   2000000i32 as libc::c_uint {
            // Fill in the VTX tm structure
            sbufWriteU8(dst, 0xdi32 as uint8_t);
            sbufWriteU8(dst, 0i32 as uint8_t);
            sbufWriteU8(dst, vtx.band);
            sbufWriteU8(dst, vtx.channel);
            sbufWriteU8(dst, vtx.pitMode);
            sbufWriteU8(dst, vtx.power);
            sbufWriteU16(dst, vtx.powerValue);
            sbufWriteU8(dst, vtx.region);
            sbufFill(dst, 0xffi32 as uint8_t, 7i32);
            memcpy(&mut vtxSent as *mut spektrumVtx_t as *mut libc::c_void,
                   &mut vtx as *mut spektrumVtx_t as *const libc::c_void,
                   ::core::mem::size_of::<spektrumVtx_t>() as libc::c_ulong);
            lastTimeSentVtx = currentTimeUs;
            return 1i32 != 0
        }
    }
    return 0i32 != 0;
}
#[no_mangle]
pub static mut srxlScheduleFuncs: [srxlScheduleFnPtr; 5] =
    unsafe {
        [Some(srxlFrameQos as
                  unsafe extern "C" fn(_: *mut sbuf_t, _: timeUs_t) -> bool),
         Some(srxlFrameRpm as
                  unsafe extern "C" fn(_: *mut sbuf_t, _: timeUs_t) -> bool),
         Some(srxlFrameFlightPackCurrent as
                  unsafe extern "C" fn(_: *mut sbuf_t, _: timeUs_t) -> bool),
         Some(srxlFrameVTX as
                  unsafe extern "C" fn(_: *mut sbuf_t, _: timeUs_t) -> bool),
         Some(srxlFrameText as
                  unsafe extern "C" fn(_: *mut sbuf_t, _: timeUs_t) -> bool)]
    };
unsafe extern "C" fn processSrxl(mut currentTimeUs: timeUs_t) {
    static mut srxlScheduleIndex: uint8_t = 0i32 as uint8_t;
    static mut srxlScheduleUserIndex: uint8_t = 0i32 as uint8_t;
    let mut srxlPayloadBuf: sbuf_t =
        sbuf_t{ptr: 0 as *mut uint8_t, end: 0 as *mut uint8_t,};
    let mut dst: *mut sbuf_t = &mut srxlPayloadBuf;
    let mut srxlFnPtr: srxlScheduleFnPtr = None;
    if (srxlScheduleIndex as libc::c_int) < 2i32 {
        srxlFnPtr = srxlScheduleFuncs[srxlScheduleIndex as usize]
    } else {
        srxlFnPtr =
            srxlScheduleFuncs[(srxlScheduleIndex as libc::c_int +
                                   srxlScheduleUserIndex as libc::c_int) as
                                  usize];
        srxlScheduleUserIndex =
            ((srxlScheduleUserIndex as libc::c_int + 1i32) %
                 (1i32 + 1i32 + 1i32)) as uint8_t;
        // Boost CMS performance by sending nothing else but CMS Text frames when in a CMS menu.
        // Sideeffect, all other reports are still not sent if user leaves CMS without a proper EXIT.
        if cmsInMenu as libc::c_int != 0 &&
               pCurrentDisplay == &mut srxlDisplayPort as *mut displayPort_t {
            srxlFnPtr =
                Some(srxlFrameText as
                         unsafe extern "C" fn(_: *mut sbuf_t, _: timeUs_t)
                             -> bool)
        }
    }
    if srxlFnPtr.is_some() {
        srxlInitializeFrame(dst);
        if srxlFnPtr.expect("non-null function pointer")(dst, currentTimeUs) {
            srxlFinalize(dst);
        }
    }
    srxlScheduleIndex =
        ((srxlScheduleIndex as libc::c_int + 1i32) % (2i32 + 1i32)) as
            uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn initSrxlTelemetry() {
    // check if there is a serial port open for SRXL telemetry (ie opened by the SRXL RX)
    // and feature is enabled, if so, set SRXL telemetry enabled
    srxlTelemetryEnabled = srxlRxIsActive();
}
#[no_mangle]
pub unsafe extern "C" fn checkSrxlTelemetryState() -> bool {
    return srxlTelemetryEnabled;
}
/*
 * Called periodically by the scheduler
 */
#[no_mangle]
pub unsafe extern "C" fn handleSrxlTelemetry(mut currentTimeUs: timeUs_t) {
    if !srxlTelemetryNow { return }
    srxlTelemetryNow = 0i32 != 0;
    processSrxl(currentTimeUs);
}
