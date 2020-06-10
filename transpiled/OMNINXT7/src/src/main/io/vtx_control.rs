use ::libc;
extern "C" {
    #[no_mangle]
    fn vtxCommonDevice() -> *mut vtxDevice_t;
    #[no_mangle]
    fn vtxCommonGetBandAndChannel(vtxDevice: *const vtxDevice_t,
                                  pBand: *mut uint8_t, pChannel: *mut uint8_t)
     -> bool;
    #[no_mangle]
    fn vtxCommonGetPowerIndex(vtxDevice: *const vtxDevice_t,
                              pIndex: *mut uint8_t) -> bool;
    #[no_mangle]
    fn vtxCommonGetDeviceCapability(vtxDevice: *const vtxDevice_t,
                                    pDeviceCapability:
                                        *mut vtxDeviceCapability_t) -> bool;
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    fn spektrumVtxControl();
    #[no_mangle]
    static mut vtxSettingsConfig_System: vtxSettingsConfig_t;
    #[no_mangle]
    fn isRangeActive(auxChannelIndex: uint8_t, range: *const channelRange_t)
     -> bool;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
// microsecond time
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
pub type C2RustUnnamed_1 = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed_1 = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed_1 = 2;
pub const ARMED: C2RustUnnamed_1 = 1;
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
pub struct channelRange_s {
    pub startStep: uint8_t,
    pub endStep: uint8_t,
}
// 1=A, 2=B, 3=E, 4=F(Airwaves/Fatshark), 5=Raceband
// 1-8
// 0 = lowest
// sets freq in MHz if band=0
// sets out-of-range pitmode frequency
// min power while disarmed
// steps are 25 apart
// a value of 0 corresponds to a channel value of 900 or less
// a value of 48 corresponds to a channel value of 2100 or more
// 48 steps between 900 and 2100
pub type channelRange_t = channelRange_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct vtxChannelActivationCondition_s {
    pub auxChannelIndex: uint8_t,
    pub band: uint8_t,
    pub channel: uint8_t,
    pub range: channelRange_t,
}
pub type vtxChannelActivationCondition_t = vtxChannelActivationCondition_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct vtxConfig_s {
    pub vtxChannelActivationConditions: [vtxChannelActivationCondition_t; 10],
    pub halfDuplex: uint8_t,
}
pub type vtxConfig_t = vtxConfig_s;
#[inline]
unsafe extern "C" fn vtxSettingsConfigMutable() -> *mut vtxSettingsConfig_t {
    return &mut vtxSettingsConfig_System;
}
#[inline]
unsafe extern "C" fn vtxConfig() -> *const vtxConfig_t {
    return &mut vtxConfig_System;
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
#[link_section = ".pg_registry"]
#[used]
pub static mut vtxConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (515 as libc::c_int |
                                      (1 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<vtxConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &vtxConfig_System as *const vtxConfig_t as
                                     *mut vtxConfig_t as *mut uint8_t,
                             copy:
                                 &vtxConfig_Copy as *const vtxConfig_t as
                                     *mut vtxConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{ptr:
                                                     &pgResetTemplate_vtxConfig
                                                         as *const vtxConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut vtxConfig_System: vtxConfig_t =
    vtxConfig_t{vtxChannelActivationConditions:
                    [vtxChannelActivationCondition_t{auxChannelIndex: 0,
                                                     band: 0,
                                                     channel: 0,
                                                     range:
                                                         channelRange_t{startStep:
                                                                            0,
                                                                        endStep:
                                                                            0,},};
                        10],
                halfDuplex: 0,};
#[no_mangle]
pub static mut vtxConfig_Copy: vtxConfig_t =
    vtxConfig_t{vtxChannelActivationConditions:
                    [vtxChannelActivationCondition_t{auxChannelIndex: 0,
                                                     band: 0,
                                                     channel: 0,
                                                     range:
                                                         channelRange_t{startStep:
                                                                            0,
                                                                        endStep:
                                                                            0,},};
                        10],
                halfDuplex: 0,};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_vtxConfig: vtxConfig_t =
    {
        let mut init =
            vtxConfig_s{vtxChannelActivationConditions:
                            [vtxChannelActivationCondition_t{auxChannelIndex:
                                                                 0,
                                                             band: 0,
                                                             channel: 0,
                                                             range:
                                                                 channelRange_t{startStep:
                                                                                    0,
                                                                                endStep:
                                                                                    0,},};
                                10],
                        halfDuplex: 1 as libc::c_int as uint8_t,};
        init
    };
static mut locked: uint8_t = 0 as libc::c_int as uint8_t;
#[no_mangle]
pub unsafe extern "C" fn vtxControlInit() {
    // NOTHING TO DO
}
#[no_mangle]
pub unsafe extern "C" fn vtxControlInputPoll() {
    // Check variuos input sources for VTX config updates
    // Get VTX updates
    spektrumVtxControl();
}
unsafe extern "C" fn vtxUpdateBandAndChannel(mut bandStep: uint8_t,
                                             mut channelStep: uint8_t) {
    if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
        locked = 1 as libc::c_int as uint8_t
    }
    if locked == 0 && !vtxCommonDevice().is_null() {
        let ref mut fresh0 = (*vtxSettingsConfigMutable()).band;
        *fresh0 =
            (*fresh0 as libc::c_int + bandStep as libc::c_int) as uint8_t;
        let ref mut fresh1 = (*vtxSettingsConfigMutable()).channel;
        *fresh1 =
            (*fresh1 as libc::c_int + channelStep as libc::c_int) as uint8_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn vtxIncrementBand() {
    vtxUpdateBandAndChannel(1 as libc::c_int as uint8_t,
                            0 as libc::c_int as uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn vtxDecrementBand() {
    vtxUpdateBandAndChannel(-(1 as libc::c_int) as uint8_t,
                            0 as libc::c_int as uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn vtxIncrementChannel() {
    vtxUpdateBandAndChannel(0 as libc::c_int as uint8_t,
                            1 as libc::c_int as uint8_t);
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
pub unsafe extern "C" fn vtxDecrementChannel() {
    vtxUpdateBandAndChannel(0 as libc::c_int as uint8_t,
                            -(1 as libc::c_int) as uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn vtxUpdateActivatedChannel() {
    if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
        locked = 1 as libc::c_int as uint8_t
    }
    if locked == 0 && !vtxCommonDevice().is_null() {
        static mut lastIndex: uint8_t = -(1 as libc::c_int) as uint8_t;
        let mut index: uint8_t = 0 as libc::c_int as uint8_t;
        while (index as libc::c_int) < 10 as libc::c_int {
            let mut vtxChannelActivationCondition:
                    *const vtxChannelActivationCondition_t =
                &*(*(vtxConfig as
                         unsafe extern "C" fn()
                             ->
                                 *const vtxConfig_t)()).vtxChannelActivationConditions.as_ptr().offset(index
                                                                                                           as
                                                                                                           isize)
                    as *const vtxChannelActivationCondition_t;
            if isRangeActive((*vtxChannelActivationCondition).auxChannelIndex,
                             &(*vtxChannelActivationCondition).range) as
                   libc::c_int != 0 &&
                   index as libc::c_int != lastIndex as libc::c_int {
                lastIndex = index;
                (*vtxSettingsConfigMutable()).band =
                    (*vtxChannelActivationCondition).band;
                (*vtxSettingsConfigMutable()).channel =
                    (*vtxChannelActivationCondition).channel;
                break ;
            } else { index = index.wrapping_add(1) }
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn vtxCycleBandOrChannel(bandStep: uint8_t,
                                               channelStep: uint8_t) {
    let mut vtxDevice: *const vtxDevice_t = vtxCommonDevice();
    if !vtxDevice.is_null() {
        let mut band: uint8_t = 0 as libc::c_int as uint8_t;
        let mut channel: uint8_t = 0 as libc::c_int as uint8_t;
        let mut capability: vtxDeviceCapability_t =
            vtxDeviceCapability_t{bandCount: 0,
                                  channelCount: 0,
                                  powerCount: 0,
                                  filler: 0,};
        let haveAllNeededInfo: bool =
            vtxCommonGetBandAndChannel(vtxDevice, &mut band, &mut channel) as
                libc::c_int != 0 &&
                vtxCommonGetDeviceCapability(vtxDevice, &mut capability) as
                    libc::c_int != 0;
        if !haveAllNeededInfo { return }
        let mut newChannel: libc::c_int =
            channel as libc::c_int + channelStep as libc::c_int;
        if newChannel > capability.channelCount as libc::c_int {
            newChannel = 1 as libc::c_int
        } else if newChannel < 1 as libc::c_int {
            newChannel = capability.channelCount as libc::c_int
        }
        let mut newBand: libc::c_int =
            band as libc::c_int + bandStep as libc::c_int;
        if newBand > capability.bandCount as libc::c_int {
            newBand = 1 as libc::c_int
        } else if newBand < 1 as libc::c_int {
            newBand = capability.bandCount as libc::c_int
        }
        (*vtxSettingsConfigMutable()).band = newBand as uint8_t;
        (*vtxSettingsConfigMutable()).channel = newChannel as uint8_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn vtxCyclePower(powerStep: uint8_t) {
    let mut vtxDevice: *const vtxDevice_t = vtxCommonDevice();
    if !vtxDevice.is_null() {
        let mut power: uint8_t = 0 as libc::c_int as uint8_t;
        let mut capability: vtxDeviceCapability_t =
            vtxDeviceCapability_t{bandCount: 0,
                                  channelCount: 0,
                                  powerCount: 0,
                                  filler: 0,};
        let haveAllNeededInfo: bool =
            vtxCommonGetPowerIndex(vtxDevice, &mut power) as libc::c_int != 0
                &&
                vtxCommonGetDeviceCapability(vtxDevice, &mut capability) as
                    libc::c_int != 0;
        if !haveAllNeededInfo { return }
        let mut newPower: libc::c_int =
            power as libc::c_int + powerStep as libc::c_int;
        if newPower >= capability.powerCount as libc::c_int {
            newPower = 0 as libc::c_int
        } else if newPower < 0 as libc::c_int {
            newPower = capability.powerCount as libc::c_int
        }
        (*vtxSettingsConfigMutable()).power = newPower as uint8_t
    };
}
/* *
 * Allow VTX channel/band/rf power/on-off and save via a single button.
 *
 * The LED1 flashes a set number of times, followed by a short pause, one per second.  The amount of flashes decreases over time while
 * the button is held to indicate the action that will be performed upon release.
 * The actions are ordered by most-frequently used action.  i.e. you change channel more frequently than band.
 *
 * The vtx settings can be changed while the VTX is OFF.
 * If the VTX is OFF when the settings are saved the VTX will be OFF on the next boot, likewise
 * If the VTX is ON when the settings are saved the VTX will be ON on the next boot.
 *
 * Future: It would be nice to re-use the code in statusindicator.c and blink-codes but target a different LED instead of the simple timed
 * behaviour of the LED1 here.
 *
 * Future: Blink out the state after changing it.
 */
#[no_mangle]
pub unsafe extern "C" fn handleVTXControlButton() { }
