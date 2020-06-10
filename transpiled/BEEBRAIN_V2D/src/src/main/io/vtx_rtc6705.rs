use ::libc;
extern "C" {
    #[no_mangle]
    fn rtc6705SetFrequency(freq: uint16_t);
    #[no_mangle]
    fn rtc6705SetRFPower(rf_power: uint8_t);
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
    static vtx58frequencyTable: [[uint16_t; 8]; 5];
    #[no_mangle]
    static vtx58BandNames: [*const libc::c_char; 0];
    #[no_mangle]
    static vtx58ChannelNames: [*const libc::c_char; 0];
    #[no_mangle]
    fn vtxCommonSetDevice(vtxDevice: *mut vtxDevice_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type timeUs_t = uint32_t;
pub type vtxDevice_t = vtxDevice_s;
#[derive(Copy, Clone)]
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
pub type vtxDeviceCapability_t = vtxDeviceCapability_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct vtxDeviceCapability_s {
    pub bandCount: uint8_t,
    pub channelCount: uint8_t,
    pub powerCount: uint8_t,
    pub filler: uint8_t,
}
#[derive(Copy, Clone)]
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
pub type vtxVTable_t = vtxVTable_s;
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
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
/* Created by jflyper */
#[no_mangle]
pub static mut rtc6705PowerNames: [*const libc::c_char; 3] =
    [b"OFF\x00" as *const u8 as *const libc::c_char,
     b"MIN\x00" as *const u8 as *const libc::c_char,
     b"MAX\x00" as *const u8 as *const libc::c_char];
// Forward
static mut vtxRTC6705: vtxDevice_t =
    unsafe {
        {
            let mut init =
                vtxDevice_s{vTable:
                                &rtc6705VTable as *const vtxVTable_t as
                                    *mut vtxVTable_t,
                            capability:
                                {
                                    let mut init =
                                        vtxDeviceCapability_s{bandCount:
                                                                  (5 as
                                                                       libc::c_int
                                                                       -
                                                                       1 as
                                                                           libc::c_int
                                                                       +
                                                                       1 as
                                                                           libc::c_int)
                                                                      as
                                                                      uint8_t,
                                                              channelCount:
                                                                  (8 as
                                                                       libc::c_int
                                                                       -
                                                                       1 as
                                                                           libc::c_int
                                                                       +
                                                                       1 as
                                                                           libc::c_int)
                                                                      as
                                                                      uint8_t,
                                                              powerCount:
                                                                  3 as
                                                                      libc::c_int
                                                                      as
                                                                      uint8_t,
                                                              filler: 0,};
                                    init
                                },
                            frequencyTable:
                                0 as *const uint16_t as *mut uint16_t,
                            bandNames:
                                vtx58BandNames.as_ptr() as
                                    *mut *mut libc::c_char,
                            channelNames:
                                vtx58ChannelNames.as_ptr() as
                                    *mut *mut libc::c_char,
                            powerNames:
                                rtc6705PowerNames.as_ptr() as
                                    *mut *mut libc::c_char,
                            frequency: 0,
                            band: 0,
                            channel: 0,
                            powerIndex: 0,
                            pitMode: 0,};
            init
        }
    };
#[no_mangle]
pub unsafe extern "C" fn vtxRTC6705Init() -> bool {
    vtxCommonSetDevice(&mut vtxRTC6705);
    return 1 as libc::c_int != 0;
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
pub unsafe extern "C" fn vtxRTC6705CanUpdate() -> bool {
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn vtxRTC6705Process(mut vtxDevice: *mut vtxDevice_t,
                                       mut now: timeUs_t) {
}
// Interface to common VTX API
unsafe extern "C" fn vtxRTC6705GetDeviceType(mut vtxDevice:
                                                 *const vtxDevice_t)
 -> vtxDevType_e {
    return VTXDEV_RTC6705;
}
unsafe extern "C" fn vtxRTC6705IsReady(mut vtxDevice: *const vtxDevice_t)
 -> bool {
    return !vtxDevice.is_null();
}
unsafe extern "C" fn vtxRTC6705SetBandAndChannel(mut vtxDevice:
                                                     *mut vtxDevice_t,
                                                 mut band: uint8_t,
                                                 mut channel: uint8_t) {
    while !vtxRTC6705CanUpdate() { }
    if band as libc::c_int >= 1 as libc::c_int &&
           band as libc::c_int <=
               5 as libc::c_int - 1 as libc::c_int + 1 as libc::c_int &&
           channel as libc::c_int >= 1 as libc::c_int &&
           channel as libc::c_int <=
               8 as libc::c_int - 1 as libc::c_int + 1 as libc::c_int {
        if (*vtxDevice).powerIndex as libc::c_int > 0 as libc::c_int {
            (*vtxDevice).band = band;
            (*vtxDevice).channel = channel;
            vtxRTC6705SetFrequency(vtxDevice,
                                   vtx58frequencyTable[(band as libc::c_int -
                                                            1 as libc::c_int)
                                                           as
                                                           usize][(channel as
                                                                       libc::c_int
                                                                       -
                                                                       1 as
                                                                           libc::c_int)
                                                                      as
                                                                      usize]);
        }
    };
}
unsafe extern "C" fn vtxRTC6705SetPowerByIndex(mut vtxDevice:
                                                   *mut vtxDevice_t,
                                               mut index: uint8_t) {
    while !vtxRTC6705CanUpdate() { }
    (*vtxDevice).powerIndex =
        ({
             let mut _a: uint8_t = index;
             let mut _b: libc::c_int = 1 as libc::c_int;
             if _a as libc::c_int > _b { _a as libc::c_int } else { _b }
         }) as uint8_t;
    rtc6705SetRFPower(index);
}
unsafe extern "C" fn vtxRTC6705SetPitMode(mut vtxDevice: *mut vtxDevice_t,
                                          mut onoff: uint8_t) {
}
unsafe extern "C" fn vtxRTC6705SetFrequency(mut vtxDevice: *mut vtxDevice_t,
                                            mut frequency: uint16_t) {
    if frequency as libc::c_int >= 5600 as libc::c_int &&
           frequency as libc::c_int <= 5950 as libc::c_int {
        frequency =
            constrain(frequency as libc::c_int, 5600 as libc::c_int,
                      5950 as libc::c_int) as uint16_t;
        (*vtxDevice).frequency = frequency;
        rtc6705SetFrequency(frequency);
    };
}
unsafe extern "C" fn vtxRTC6705GetBandAndChannel(mut vtxDevice:
                                                     *const vtxDevice_t,
                                                 mut pBand: *mut uint8_t,
                                                 mut pChannel: *mut uint8_t)
 -> bool {
    *pBand = (*vtxDevice).band;
    *pChannel = (*vtxDevice).channel;
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn vtxRTC6705GetPowerIndex(mut vtxDevice:
                                                 *const vtxDevice_t,
                                             mut pIndex: *mut uint8_t)
 -> bool {
    *pIndex = (*vtxDevice).powerIndex;
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn vtxRTC6705GetPitMode(mut vtxDevice: *const vtxDevice_t,
                                          mut pOnOff: *mut uint8_t) -> bool {
    return 0 as libc::c_int != 0;
}
unsafe extern "C" fn vtxRTC6705GetFreq(mut vtxDevice: *const vtxDevice_t,
                                       mut pFrequency: *mut uint16_t)
 -> bool {
    *pFrequency = (*vtxDevice).frequency;
    return 1 as libc::c_int != 0;
}
static mut rtc6705VTable: vtxVTable_t =
    unsafe {
        {
            let mut init =
                vtxVTable_s{process:
                                Some(vtxRTC6705Process as
                                         unsafe extern "C" fn(_:
                                                                  *mut vtxDevice_t,
                                                              _: timeUs_t)
                                             -> ()),
                            getDeviceType:
                                Some(vtxRTC6705GetDeviceType as
                                         unsafe extern "C" fn(_:
                                                                  *const vtxDevice_t)
                                             -> vtxDevType_e),
                            isReady:
                                Some(vtxRTC6705IsReady as
                                         unsafe extern "C" fn(_:
                                                                  *const vtxDevice_t)
                                             -> bool),
                            setBandAndChannel:
                                Some(vtxRTC6705SetBandAndChannel as
                                         unsafe extern "C" fn(_:
                                                                  *mut vtxDevice_t,
                                                              _: uint8_t,
                                                              _: uint8_t)
                                             -> ()),
                            setPowerByIndex:
                                Some(vtxRTC6705SetPowerByIndex as
                                         unsafe extern "C" fn(_:
                                                                  *mut vtxDevice_t,
                                                              _: uint8_t)
                                             -> ()),
                            setPitMode:
                                Some(vtxRTC6705SetPitMode as
                                         unsafe extern "C" fn(_:
                                                                  *mut vtxDevice_t,
                                                              _: uint8_t)
                                             -> ()),
                            setFrequency:
                                Some(vtxRTC6705SetFrequency as
                                         unsafe extern "C" fn(_:
                                                                  *mut vtxDevice_t,
                                                              _: uint16_t)
                                             -> ()),
                            getBandAndChannel:
                                Some(vtxRTC6705GetBandAndChannel as
                                         unsafe extern "C" fn(_:
                                                                  *const vtxDevice_t,
                                                              _: *mut uint8_t,
                                                              _: *mut uint8_t)
                                             -> bool),
                            getPowerIndex:
                                Some(vtxRTC6705GetPowerIndex as
                                         unsafe extern "C" fn(_:
                                                                  *const vtxDevice_t,
                                                              _: *mut uint8_t)
                                             -> bool),
                            getPitMode:
                                Some(vtxRTC6705GetPitMode as
                                         unsafe extern "C" fn(_:
                                                                  *const vtxDevice_t,
                                                              _: *mut uint8_t)
                                             -> bool),
                            getFrequency:
                                Some(vtxRTC6705GetFreq as
                                         unsafe extern "C" fn(_:
                                                                  *const vtxDevice_t,
                                                              _:
                                                                  *mut uint16_t)
                                             -> bool),};
            init
        }
    };
// VTX_RTC6705
// VTX_COMMON
