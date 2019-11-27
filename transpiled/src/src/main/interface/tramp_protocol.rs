use core;
use libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
}
pub type size_t = libc::c_ulong;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
#[derive ( Copy, Clone )]
#[repr(C, packed)]
pub struct trampSettings_s {
    pub frequency: uint16_t,
    pub power: uint16_t,
    pub raceModeEnabled: uint8_t,
    pub pitModeEnabled: uint8_t,
}
pub type trampSettings_t = trampSettings_s;
#[derive ( Copy, Clone )]
#[repr(C, packed)]
pub struct trampFrameHeader_s {
    pub syncStart: uint8_t,
    pub command: uint8_t,
}
pub type trampFrameHeader_t = trampFrameHeader_s;
#[derive ( Copy, Clone )]
#[repr(C, packed)]
pub struct trampFrameFooter_s {
    pub crc: uint8_t,
    pub syncStop: uint8_t,
}
pub type trampFrameFooter_t = trampFrameFooter_s;
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union trampPayload_u {
    pub buf: [uint8_t; 12],
    pub settings: trampSettings_t,
    pub frequency: uint16_t,
    pub power: uint16_t,
    pub active: uint8_t,
}
pub type trampPayload_t = trampPayload_u;
#[derive ( Copy, Clone )]
#[repr(C, packed)]
pub struct trampFrame_s {
    pub header: trampFrameHeader_t,
    pub payload: trampPayload_t,
    pub footer: trampFrameFooter_t,
}
pub type trampFrame_t = trampFrame_s;
// 0x76
unsafe extern "C" fn trampCrc(mut frame: *const trampFrame_t) -> uint8_t {
    let mut crc: uint8_t = 0i32 as uint8_t;
    let mut p: *const uint8_t = frame as *const uint8_t;
    let mut pEnd: *const uint8_t =
        p.offset((::core::mem::size_of::<trampFrameHeader_t>() as
                      libc::c_ulong).wrapping_add(12i32 as libc::c_ulong) as
                     isize);
    while p != pEnd {
        crc = (crc as libc::c_int + *p as libc::c_int) as uint8_t;
        p = p.offset(1)
    }
    return crc;
}
unsafe extern "C" fn trampFrameInit(mut frameType: uint8_t,
                                    mut frame: *mut trampFrame_t) {
    (*frame).header.syncStart = 0xfi32 as uint8_t;
    (*frame).header.command = frameType;
    let emptyPayload: [uint8_t; 12] =
        [0i32 as uint8_t, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    memcpy((*frame).payload.buf.as_mut_ptr() as *mut libc::c_void,
           emptyPayload.as_ptr() as *const libc::c_void,
           ::core::mem::size_of::<[uint8_t; 12]>() as libc::c_ulong);
}
unsafe extern "C" fn trampFrameClose(mut frame: *mut trampFrame_t) {
    (*frame).footer.crc = trampCrc(frame);
    (*frame).footer.syncStop = 0i32 as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn trampFrameGetSettings(mut frame: *mut trampFrame_t) {
    trampFrameInit('v' as i32 as uint8_t, frame);
    trampFrameClose(frame);
}
#[no_mangle]
pub unsafe extern "C" fn trampFrameSetFrequency(mut frame: *mut trampFrame_t,
                                                frequency: uint16_t) {
    trampFrameInit('F' as i32 as uint8_t, frame);
    (*frame).payload.frequency = frequency;
    trampFrameClose(frame);
}
#[no_mangle]
pub unsafe extern "C" fn trampFrameSetPower(mut frame: *mut trampFrame_t,
                                            power: uint16_t) {
    trampFrameInit('P' as i32 as uint8_t, frame);
    (*frame).payload.power = power;
    trampFrameClose(frame);
}
#[no_mangle]
pub unsafe extern "C" fn trampFrameSetActiveState(mut frame:
                                                      *mut trampFrame_t,
                                                  active: bool) {
    trampFrameInit('I' as i32 as uint8_t, frame);
    (*frame).payload.active = active as uint8_t;
    trampFrameClose(frame);
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
pub unsafe extern "C" fn trampParseResponseBuffer(mut settings:
                                                      *mut trampSettings_t,
                                                  mut buffer: *const uint8_t,
                                                  mut bufferLen: size_t)
 -> bool {
    if bufferLen != ::core::mem::size_of::<trampFrame_t>() as libc::c_ulong {
        return 0i32 != 0
    }
    let mut frame: *const trampFrame_t = buffer as *const trampFrame_t;
    let crc: uint8_t = trampCrc(frame);
    if crc as libc::c_int != (*frame).footer.crc as libc::c_int {
        return 0i32 != 0
    }
    memcpy(settings as *mut libc::c_void,
           &(*frame).payload.settings as *const trampSettings_t as
               *const libc::c_void,
           ::core::mem::size_of::<trampSettings_t>() as libc::c_ulong);
    return 1i32 != 0;
}
