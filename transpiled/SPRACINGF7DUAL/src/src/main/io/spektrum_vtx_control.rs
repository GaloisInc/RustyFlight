use ::libc;
extern "C" {
    #[no_mangle]
    fn memcmp(_: *const libc::c_void, _: *const libc::c_void,
              _: libc::c_ulong) -> libc::c_int;
    #[no_mangle]
    fn saveConfigAndNotify();
    #[no_mangle]
    fn vtxCommonDevice() -> *mut vtxDevice_t;
    #[no_mangle]
    fn vtxCommonSetPitMode(vtxDevice: *mut vtxDevice_t, onoff: uint8_t);
    #[no_mangle]
    fn vtxCommonGetPitMode(vtxDevice: *const vtxDevice_t,
                           pOnOff: *mut uint8_t) -> bool;
    #[no_mangle]
    fn vtxCommonGetDeviceType(vtxDevice: *const vtxDevice_t) -> vtxDevType_e;
    #[no_mangle]
    static mut vtxSettingsConfig_System: vtxSettingsConfig_t;
    #[no_mangle]
    fn vtx58_Bandchan2Freq(band: uint8_t, channel: uint8_t) -> uint16_t;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct vtxSettingsConfig_s {
    pub band: uint8_t,
    pub channel: uint8_t,
    pub power: uint8_t,
    pub freq: uint16_t,
    pub pitModeFreq: uint16_t,
    pub lowPowerDisarm: uint8_t,
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
pub type vtxSettingsConfig_t = vtxSettingsConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct spektrumVtx_t {
    pub band: uint8_t,
    pub channel: uint8_t,
    pub power: uint8_t,
    pub region: uint8_t,
    pub pitMode: uint8_t,
    pub powerValue: uint16_t,
}
#[inline]
unsafe extern "C" fn vtxSettingsConfig() -> *const vtxSettingsConfig_t {
    return &mut vtxSettingsConfig_System;
}
#[inline]
unsafe extern "C" fn vtxSettingsConfigMutable() -> *mut vtxSettingsConfig_t {
    return &mut vtxSettingsConfig_System;
}
// 1=A, 2=B, 3=E, 4=F(Airwaves/Fatshark), 5=Raceband
// 1-8
// 0 = lowest
// sets freq in MHz if band=0
// sets out-of-range pitmode frequency
// min power while disarmed
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
// We can not use the common set/get-frequncy API.
// Some VTX devices do not support it.
//#define USE_VTX_COMMON_FREQ_API
// Translation table, Spektrum bands to BF internal vtx_common bands
#[no_mangle]
pub static mut spek2commonBand: [uint8_t; 5] =
    [4 as libc::c_int as uint8_t, 5 as libc::c_int as uint8_t,
     3 as libc::c_int as uint8_t, 2 as libc::c_int as uint8_t,
     1 as libc::c_int as uint8_t];
// RF Power Index translation tables. No generic power API available.....
// Tramp "---", 25, 200, 400. 600 mW
#[no_mangle]
pub static mut vtxTrampPi: [uint8_t; 8] =
    [0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     1 as libc::c_int as uint8_t, 2 as libc::c_int as uint8_t,
     3 as libc::c_int as uint8_t, 4 as libc::c_int as uint8_t,
     5 as libc::c_int as uint8_t, 3 as libc::c_int as uint8_t];
// USE_VTX_TRAMP
// RTC6705 "---", 25 or 200 mW
#[no_mangle]
pub static mut vtxRTC6705Pi: [uint8_t; 8] =
    [0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     1 as libc::c_int as uint8_t, 1 as libc::c_int as uint8_t,
     2 as libc::c_int as uint8_t, 2 as libc::c_int as uint8_t,
     2 as libc::c_int as uint8_t, 2 as libc::c_int as uint8_t];
//USE_VTX_RTC6705
// SmartAudio "---", 25, 200, 500. 800 mW
#[no_mangle]
pub static mut vtxSaPi: [uint8_t; 8] =
    [0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     1 as libc::c_int as uint8_t, 1 as libc::c_int as uint8_t,
     2 as libc::c_int as uint8_t, 3 as libc::c_int as uint8_t,
     4 as libc::c_int as uint8_t, 2 as libc::c_int as uint8_t];
// USE_VTX_SMARTAUDIO
#[no_mangle]
pub unsafe extern "C" fn convertSpektrumVtxPowerIndex(mut sPower: uint8_t)
 -> uint8_t {
    let mut devicePower: uint8_t = 0 as libc::c_int as uint8_t;
    let mut vtxDevice: *const vtxDevice_t = vtxCommonDevice();
    match vtxCommonGetDeviceType(vtxDevice) as libc::c_uint {
        1 => { devicePower = vtxRTC6705Pi[sPower as usize] }
        3 => {
            // USE_VTX_RTC6705
            devicePower = vtxSaPi[sPower as usize]
        }
        4 => {
            // USE_VTX_SMARTAUDIO
            devicePower = vtxTrampPi[sPower as usize]
        }
        255 | 0 | _ => { }
    }
    return devicePower;
}
// Mark an inital invalid VTX ctrl frame to force first VTX settings cheange to actually come from Tx/Rx.
static mut vtxControl_ipc: uint32_t = !(0xe000e000 as libc::c_uint);
// ############ RX task ######################
#[no_mangle]
pub unsafe extern "C" fn spektrumHandleVtxControl(mut vtxCntrl: uint32_t) {
    vtxControl_ipc = vtxCntrl;
}
// ###########################################
// ############ VTX_CONTROL task #############
#[no_mangle]
pub unsafe extern "C" fn spektrumVtxControl() {
    static mut prevVtxControl: uint32_t = 0 as libc::c_int as uint32_t;
    let mut vtxControl: uint32_t = 0;
    // Check for invalid VTX ctrl frames
    if vtxControl_ipc & 0xf000f000 as libc::c_uint !=
           0xe000e000 as libc::c_uint {
        return
    }
    vtxControl = vtxControl_ipc;
    vtxControl_ipc = 0 as libc::c_int as uint32_t;
    if prevVtxControl == vtxControl { return }
    prevVtxControl = vtxControl;
    let mut vtx: spektrumVtx_t =
        {
            let mut init =
                spektrumVtx_t{band:
                                  ((vtxControl &
                                        0xe00000 as libc::c_int as
                                            libc::c_uint) >>
                                       21 as libc::c_int) as uint8_t,
                              channel:
                                  ((vtxControl &
                                        0xf0000 as libc::c_int as
                                            libc::c_uint) >>
                                       16 as libc::c_int) as uint8_t,
                              power:
                                  ((vtxControl &
                                        0x7 as libc::c_int as libc::c_uint) >>
                                       0 as libc::c_int) as uint8_t,
                              region:
                                  ((vtxControl &
                                        0x8 as libc::c_int as libc::c_uint) >>
                                       3 as libc::c_int) as uint8_t,
                              pitMode:
                                  ((vtxControl &
                                        0x10 as libc::c_int as libc::c_uint)
                                       >> 4 as libc::c_int) as uint8_t,
                              powerValue: 0,};
            init
        };
    let prevSettings: vtxSettingsConfig_t =
        {
            let mut init =
                vtxSettingsConfig_s{band: (*vtxSettingsConfig()).band,
                                    channel: (*vtxSettingsConfig()).channel,
                                    power: (*vtxSettingsConfig()).power,
                                    freq: (*vtxSettingsConfig()).freq,
                                    pitModeFreq: 0,
                                    lowPowerDisarm:
                                        (*vtxSettingsConfig()).lowPowerDisarm,};
            init
        };
    let mut newSettings: vtxSettingsConfig_t = prevSettings;
    let mut vtxDevice: *mut vtxDevice_t = vtxCommonDevice();
    if !vtxDevice.is_null() {
        // Convert to the internal Common Band index
        let band: uint8_t =
            spek2commonBand[vtx.band as usize]; // 0 based to 1 based
        let channel: uint8_t =
            (vtx.channel as libc::c_int + 1 as libc::c_int) as uint8_t;
        if prevSettings.band as libc::c_int != band as libc::c_int ||
               prevSettings.channel as libc::c_int != channel as libc::c_int {
            newSettings.band = band;
            newSettings.channel = channel;
            newSettings.freq = vtx58_Bandchan2Freq(band, channel)
        }
        // Seems to be no unified internal VTX API standard for power levels/indexes, VTX device brand specific.
        let power: uint8_t = convertSpektrumVtxPowerIndex(vtx.power);
        if prevSettings.power as libc::c_int != power as libc::c_int {
            newSettings.power = power
        }
        // Everyone seems to agree on what PIT ON/OFF means
        let mut currentPitMode: uint8_t = 0 as libc::c_int as uint8_t;
        if vtxCommonGetPitMode(vtxDevice, &mut currentPitMode) {
            if currentPitMode as libc::c_int != vtx.pitMode as libc::c_int {
                vtxCommonSetPitMode(vtxDevice, vtx.pitMode);
            }
        }
    }
    if memcmp(&prevSettings as *const vtxSettingsConfig_t as
                  *const libc::c_void,
              &mut newSettings as *mut vtxSettingsConfig_t as
                  *const libc::c_void,
              ::core::mem::size_of::<vtxSettingsConfig_t>() as libc::c_ulong)
           != 0 {
        (*vtxSettingsConfigMutable()).band = newSettings.band;
        (*vtxSettingsConfigMutable()).channel = newSettings.channel;
        (*vtxSettingsConfigMutable()).power = newSettings.power;
        (*vtxSettingsConfigMutable()).freq = newSettings.freq;
        saveConfigAndNotify();
    };
}
// USE_SPEKTRUM_VTX_CONTROL && USE_VTX_COMMON
