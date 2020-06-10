use ::libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type timeUs_t = uint32_t;
pub type vtxDevType_e = libc::c_uint;
pub const VTXDEV_UNKNOWN: vtxDevType_e = 255;
pub const VTXDEV_TRAMP: vtxDevType_e = 4;
pub const VTXDEV_SMARTAUDIO: vtxDevType_e = 3;
pub const VTXDEV_RTC6705: vtxDevType_e = 1;
pub const VTXDEV_UNSUPPORTED: vtxDevType_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct vtxDeviceCapability_s {
    pub bandCount: uint8_t,
    pub channelCount: uint8_t,
    pub powerCount: uint8_t,
    pub filler: uint8_t,
}
// VTX magic numbers
// RTC6705 RF Power index "---", 25 or 200 mW
// SmartAudio "---", 25, 200, 500, 800 mW
// Tramp "---", 25, 100, 200, 400, 600 mW
pub type vtxDeviceCapability_t = vtxDeviceCapability_s;
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
// Array of [bandCount][channelCount]
// char *bandNames[bandCount]
// char *channelNames[channelCount]
// char *powerNames[powerCount]
// Band = 1, 1-based
// CH1 = 1, 1-based
// Lowest/Off = 0
// 0 = non-PIT, 1 = PIT
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
static mut vtxDevice: *mut vtxDevice_t =
    0 as *const vtxDevice_t as *mut vtxDevice_t;
// 3.1.0
// PIT mode is defined as LOWEST POSSIBLE RF POWER.
// - It can be a dedicated mode, or lowest RF power possible.
// - It is *NOT* RF on/off control ?
#[no_mangle]
pub unsafe extern "C" fn vtxCommonInit() { }
#[no_mangle]
pub unsafe extern "C" fn vtxCommonSetDevice(mut pDevice: *mut vtxDevice_t) {
    vtxDevice = pDevice;
}
#[no_mangle]
pub unsafe extern "C" fn vtxCommonDevice() -> *mut vtxDevice_t {
    return vtxDevice;
}
#[no_mangle]
pub unsafe extern "C" fn vtxCommonGetDeviceType(mut vtxDevice_0:
                                                    *const vtxDevice_t)
 -> vtxDevType_e {
    if vtxDevice_0.is_null() { return VTXDEV_UNKNOWN }
    return (*(*vtxDevice_0).vTable).getDeviceType.expect("non-null function pointer")(vtxDevice_0);
}
// VTable functions
#[no_mangle]
pub unsafe extern "C" fn vtxCommonProcess(mut vtxDevice_0: *mut vtxDevice_t,
                                          mut currentTimeUs: timeUs_t) {
    if !vtxDevice_0.is_null() {
        (*(*vtxDevice_0).vTable).process.expect("non-null function pointer")(vtxDevice_0,
                                                                             currentTimeUs);
    };
}
// band and channel are 1 origin
#[no_mangle]
pub unsafe extern "C" fn vtxCommonSetBandAndChannel(mut vtxDevice_0:
                                                        *mut vtxDevice_t,
                                                    mut band: uint8_t,
                                                    mut channel: uint8_t) {
    if band as libc::c_int <=
           (*vtxDevice_0).capability.bandCount as libc::c_int &&
           channel as libc::c_int <=
               (*vtxDevice_0).capability.channelCount as libc::c_int {
        (*(*vtxDevice_0).vTable).setBandAndChannel.expect("non-null function pointer")(vtxDevice_0,
                                                                                       band,
                                                                                       channel);
    };
}
// index is zero origin, zero = power off completely
#[no_mangle]
pub unsafe extern "C" fn vtxCommonSetPowerByIndex(mut vtxDevice_0:
                                                      *mut vtxDevice_t,
                                                  mut index: uint8_t) {
    if index as libc::c_int <=
           (*vtxDevice_0).capability.powerCount as libc::c_int {
        (*(*vtxDevice_0).vTable).setPowerByIndex.expect("non-null function pointer")(vtxDevice_0,
                                                                                     index);
    };
}
// on = 1, off = 0
#[no_mangle]
pub unsafe extern "C" fn vtxCommonSetPitMode(mut vtxDevice_0:
                                                 *mut vtxDevice_t,
                                             mut onOff: uint8_t) {
    (*(*vtxDevice_0).vTable).setPitMode.expect("non-null function pointer")(vtxDevice_0,
                                                                            onOff);
}
#[no_mangle]
pub unsafe extern "C" fn vtxCommonSetFrequency(mut vtxDevice_0:
                                                   *mut vtxDevice_t,
                                               mut frequency: uint16_t) {
    (*(*vtxDevice_0).vTable).setFrequency.expect("non-null function pointer")(vtxDevice_0,
                                                                              frequency);
}
#[no_mangle]
pub unsafe extern "C" fn vtxCommonGetBandAndChannel(mut vtxDevice_0:
                                                        *const vtxDevice_t,
                                                    mut pBand: *mut uint8_t,
                                                    mut pChannel:
                                                        *mut uint8_t)
 -> bool {
    return (*(*vtxDevice_0).vTable).getBandAndChannel.expect("non-null function pointer")(vtxDevice_0,
                                                                                          pBand,
                                                                                          pChannel);
}
#[no_mangle]
pub unsafe extern "C" fn vtxCommonGetPowerIndex(mut vtxDevice_0:
                                                    *const vtxDevice_t,
                                                mut pIndex: *mut uint8_t)
 -> bool {
    return (*(*vtxDevice_0).vTable).getPowerIndex.expect("non-null function pointer")(vtxDevice_0,
                                                                                      pIndex);
}
#[no_mangle]
pub unsafe extern "C" fn vtxCommonGetPitMode(mut vtxDevice_0:
                                                 *const vtxDevice_t,
                                             mut pOnOff: *mut uint8_t)
 -> bool {
    return (*(*vtxDevice_0).vTable).getPitMode.expect("non-null function pointer")(vtxDevice_0,
                                                                                   pOnOff);
}
#[no_mangle]
pub unsafe extern "C" fn vtxCommonGetFrequency(mut vtxDevice_0:
                                                   *const vtxDevice_t,
                                               mut pFrequency: *mut uint16_t)
 -> bool {
    return (*(*vtxDevice_0).vTable).getFrequency.expect("non-null function pointer")(vtxDevice_0,
                                                                                     pFrequency);
}
#[no_mangle]
pub unsafe extern "C" fn vtxCommonGetDeviceCapability(mut vtxDevice_0:
                                                          *const vtxDevice_t,
                                                      mut pDeviceCapability:
                                                          *mut vtxDeviceCapability_t)
 -> bool {
    memcpy(pDeviceCapability as *mut libc::c_void,
           &(*vtxDevice_0).capability as *const vtxDeviceCapability_t as
               *const libc::c_void,
           ::core::mem::size_of::<vtxDeviceCapability_t>() as libc::c_ulong);
    return 1 as libc::c_int != 0;
}
