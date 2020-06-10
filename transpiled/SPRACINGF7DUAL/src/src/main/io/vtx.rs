use ::libc;
extern "C" {
    #[no_mangle]
    fn memcmp(_: *const libc::c_void, _: *const libc::c_void,
              _: libc::c_ulong) -> libc::c_int;
    #[no_mangle]
    fn vtxCommonDevice() -> *mut vtxDevice_t;
    // VTable functions
    #[no_mangle]
    fn vtxCommonProcess(vtxDevice: *mut vtxDevice_t, currentTimeUs: timeUs_t);
    #[no_mangle]
    fn vtxCommonSetBandAndChannel(vtxDevice: *mut vtxDevice_t, band: uint8_t,
                                  channel: uint8_t);
    #[no_mangle]
    fn vtxCommonSetPowerByIndex(vtxDevice: *mut vtxDevice_t, level: uint8_t);
    #[no_mangle]
    fn vtxCommonSetPitMode(vtxDevice: *mut vtxDevice_t, onoff: uint8_t);
    #[no_mangle]
    fn vtxCommonSetFrequency(vtxDevice: *mut vtxDevice_t, freq: uint16_t);
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
    #[no_mangle]
    fn vtxCommonGetFrequency(vtxDevice: *const vtxDevice_t,
                             pFreq: *mut uint16_t) -> bool;
    #[no_mangle]
    fn saveConfigAndNotify();
    #[no_mangle]
    fn IS_RC_MODE_ACTIVE(boxId: boxId_e) -> bool;
    #[no_mangle]
    fn isModeActivationConditionPresent(modeId: boxId_e) -> bool;
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    fn failsafeIsActive() -> bool;
    #[no_mangle]
    fn vtx58_Bandchan2Freq(band: uint8_t, channel: uint8_t) -> uint16_t;
    #[no_mangle]
    fn vtxControlInputPoll();
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
    static mut cliMode: uint8_t;
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
pub type boxId_e = libc::c_uint;
pub const CHECKBOX_ITEM_COUNT: boxId_e = 41;
pub const BOXACROTRAINER: boxId_e = 40;
pub const BOXPIDAUDIO: boxId_e = 39;
pub const BOXUSER4: boxId_e = 38;
pub const BOXUSER3: boxId_e = 37;
pub const BOXUSER2: boxId_e = 36;
pub const BOXUSER1: boxId_e = 35;
pub const BOXPARALYZE: boxId_e = 34;
pub const BOXVTXPITMODE: boxId_e = 33;
pub const BOXBEEPGPSCOUNT: boxId_e = 32;
pub const BOXPREARM: boxId_e = 31;
pub const BOXFLIPOVERAFTERCRASH: boxId_e = 30;
pub const BOXCAMERA3: boxId_e = 29;
pub const BOXCAMERA2: boxId_e = 28;
pub const BOXCAMERA1: boxId_e = 27;
pub const BOXBLACKBOXERASE: boxId_e = 26;
pub const BOXFPVANGLEMIX: boxId_e = 25;
pub const BOX3D: boxId_e = 24;
pub const BOXAIRMODE: boxId_e = 23;
pub const BOXBLACKBOX: boxId_e = 22;
pub const BOXSERVO3: boxId_e = 21;
pub const BOXSERVO2: boxId_e = 20;
pub const BOXSERVO1: boxId_e = 19;
pub const BOXTELEMETRY: boxId_e = 18;
pub const BOXOSD: boxId_e = 17;
pub const BOXCALIB: boxId_e = 16;
pub const BOXLEDLOW: boxId_e = 15;
pub const BOXBEEPERON: boxId_e = 14;
pub const BOXCAMSTAB: boxId_e = 13;
pub const BOXHEADADJ: boxId_e = 12;
pub const BOXANTIGRAVITY: boxId_e = 11;
pub const BOXID_FLIGHTMODE_LAST: boxId_e = 10;
pub const BOXGPSRESCUE: boxId_e = 10;
pub const BOXFAILSAFE: boxId_e = 9;
pub const BOXPASSTHRU: boxId_e = 8;
pub const BOXHEADFREE: boxId_e = 7;
pub const BOXGPSHOLD: boxId_e = 6;
pub const BOXGPSHOME: boxId_e = 5;
pub const BOXBARO: boxId_e = 4;
pub const BOXMAG: boxId_e = 3;
pub const BOXHORIZON: boxId_e = 2;
pub const BOXANGLE: boxId_e = 1;
pub const BOXARM: boxId_e = 0;
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
pub const VTX_PARAM_COUNT: C2RustUnnamed_2 = 4;
pub const VTX_PARAM_CONFIRM: C2RustUnnamed_2 = 3;
pub const VTX_PARAM_PITMODE: C2RustUnnamed_2 = 2;
pub const VTX_PARAM_BANDCHAN: C2RustUnnamed_2 = 1;
pub const VTX_PARAM_POWER: C2RustUnnamed_2 = 0;
pub type C2RustUnnamed_2 = libc::c_uint;
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
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut vtxSettingsConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (259 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<vtxSettingsConfig_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &vtxSettingsConfig_System as
                                     *const vtxSettingsConfig_t as
                                     *mut vtxSettingsConfig_t as *mut uint8_t,
                             copy:
                                 &vtxSettingsConfig_Copy as
                                     *const vtxSettingsConfig_t as
                                     *mut vtxSettingsConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{ptr:
                                                     &pgResetTemplate_vtxSettingsConfig
                                                         as
                                                         *const vtxSettingsConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut vtxSettingsConfig_System: vtxSettingsConfig_t =
    vtxSettingsConfig_t{band: 0,
                        channel: 0,
                        power: 0,
                        freq: 0,
                        pitModeFreq: 0,
                        lowPowerDisarm: 0,};
#[no_mangle]
pub static mut vtxSettingsConfig_Copy: vtxSettingsConfig_t =
    vtxSettingsConfig_t{band: 0,
                        channel: 0,
                        power: 0,
                        freq: 0,
                        pitModeFreq: 0,
                        lowPowerDisarm: 0,};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_vtxSettingsConfig: vtxSettingsConfig_t =
    {
        let mut init =
            vtxSettingsConfig_s{band: 4 as libc::c_int as uint8_t,
                                channel: 1 as libc::c_int as uint8_t,
                                power: 1 as libc::c_int as uint8_t,
                                freq: 5740 as libc::c_int as uint16_t,
                                pitModeFreq: 0 as libc::c_int as uint16_t,
                                lowPowerDisarm: 0 as libc::c_int as uint8_t,};
        init
    };
#[no_mangle]
pub unsafe extern "C" fn vtxInit() {
    let mut settingsUpdated: bool = 0 as libc::c_int != 0;
    // sync frequency in parameter group when band/channel are specified
    let freq: uint16_t =
        vtx58_Bandchan2Freq((*vtxSettingsConfig()).band,
                            (*vtxSettingsConfig()).channel);
    if (*vtxSettingsConfig()).band as libc::c_int != 0 &&
           freq as libc::c_int != (*vtxSettingsConfig()).freq as libc::c_int {
        (*vtxSettingsConfigMutable()).freq = freq;
        settingsUpdated = 1 as libc::c_int != 0
    }
    // constrain pit mode frequency
    if (*vtxSettingsConfig()).pitModeFreq != 0 {
        let constrainedPitModeFreq: uint16_t =
            ({
                 let _a: uint16_t = (*vtxSettingsConfig()).pitModeFreq;
                 let mut _b: libc::c_int = 5000 as libc::c_int;
                 if _a as libc::c_int > _b { _a as libc::c_int } else { _b }
             }) as uint16_t;
        if constrainedPitModeFreq as libc::c_int !=
               (*vtxSettingsConfig()).pitModeFreq as libc::c_int {
            (*vtxSettingsConfigMutable()).pitModeFreq =
                constrainedPitModeFreq;
            settingsUpdated = 1 as libc::c_int != 0
        }
    }
    if settingsUpdated { saveConfigAndNotify(); };
}
unsafe extern "C" fn vtxGetSettings() -> vtxSettingsConfig_t {
    let mut settings: vtxSettingsConfig_t =
        {
            let mut init =
                vtxSettingsConfig_s{band: (*vtxSettingsConfig()).band,
                                    channel: (*vtxSettingsConfig()).channel,
                                    power: (*vtxSettingsConfig()).power,
                                    freq: (*vtxSettingsConfig()).freq,
                                    pitModeFreq:
                                        (*vtxSettingsConfig()).pitModeFreq,
                                    lowPowerDisarm:
                                        (*vtxSettingsConfig()).lowPowerDisarm,};
            init
        };
    if IS_RC_MODE_ACTIVE(BOXVTXPITMODE) as libc::c_int != 0 &&
           isModeActivationConditionPresent(BOXVTXPITMODE) as libc::c_int != 0
           && settings.pitModeFreq as libc::c_int != 0 {
        settings.band = 0 as libc::c_int as uint8_t;
        settings.freq = settings.pitModeFreq;
        settings.power = 1 as libc::c_int as uint8_t
    }
    if armingFlags as libc::c_int & ARMED as libc::c_int == 0 &&
           settings.lowPowerDisarm as libc::c_int != 0 && !failsafeIsActive()
       {
        settings.power = 1 as libc::c_int as uint8_t
    }
    return settings;
}
unsafe extern "C" fn vtxProcessBandAndChannel(mut vtxDevice: *mut vtxDevice_t)
 -> bool {
    if armingFlags as libc::c_int & ARMED as libc::c_int == 0 {
        let mut vtxBand: uint8_t = 0;
        let mut vtxChan: uint8_t = 0;
        if vtxCommonGetBandAndChannel(vtxDevice, &mut vtxBand, &mut vtxChan) {
            let settings: vtxSettingsConfig_t = vtxGetSettings();
            if vtxBand as libc::c_int != settings.band as libc::c_int ||
                   vtxChan as libc::c_int != settings.channel as libc::c_int {
                vtxCommonSetBandAndChannel(vtxDevice, settings.band,
                                           settings.channel);
                return 1 as libc::c_int != 0
            }
        }
    }
    return 0 as libc::c_int != 0;
}
unsafe extern "C" fn vtxProcessFrequency(mut vtxDevice: *mut vtxDevice_t)
 -> bool {
    if armingFlags as libc::c_int & ARMED as libc::c_int == 0 {
        let mut vtxFreq: uint16_t = 0;
        if vtxCommonGetFrequency(vtxDevice, &mut vtxFreq) {
            let settings: vtxSettingsConfig_t = vtxGetSettings();
            if vtxFreq as libc::c_int != settings.freq as libc::c_int {
                vtxCommonSetFrequency(vtxDevice, settings.freq);
                return 1 as libc::c_int != 0
            }
        }
    }
    return 0 as libc::c_int != 0;
}
unsafe extern "C" fn vtxProcessPower(mut vtxDevice: *mut vtxDevice_t)
 -> bool {
    let mut vtxPower: uint8_t = 0;
    if vtxCommonGetPowerIndex(vtxDevice, &mut vtxPower) {
        let settings: vtxSettingsConfig_t = vtxGetSettings();
        if vtxPower as libc::c_int != settings.power as libc::c_int {
            vtxCommonSetPowerByIndex(vtxDevice, settings.power);
            return 1 as libc::c_int != 0
        }
    }
    return 0 as libc::c_int != 0;
}
unsafe extern "C" fn vtxProcessPitMode(mut vtxDevice: *mut vtxDevice_t)
 -> bool {
    let mut pitOnOff: uint8_t = 0;
    let mut currPmSwitchState: bool = false;
    static mut prevPmSwitchState: bool = 0 as libc::c_int != 0;
    if armingFlags as libc::c_int & ARMED as libc::c_int == 0 &&
           vtxCommonGetPitMode(vtxDevice, &mut pitOnOff) as libc::c_int != 0 {
        currPmSwitchState = IS_RC_MODE_ACTIVE(BOXVTXPITMODE);
        if currPmSwitchState as libc::c_int !=
               prevPmSwitchState as libc::c_int {
            prevPmSwitchState = currPmSwitchState;
            if currPmSwitchState {
                if (*vtxSettingsConfig()).pitModeFreq != 0 {
                    return 0 as libc::c_int != 0
                }
                if isModeActivationConditionPresent(BOXVTXPITMODE) {
                    if pitOnOff == 0 {
                        vtxCommonSetPitMode(vtxDevice,
                                            1 as libc::c_int as uint8_t);
                        return 1 as libc::c_int != 0
                    }
                }
            } else if pitOnOff != 0 {
                vtxCommonSetPitMode(vtxDevice, 0 as libc::c_int as uint8_t);
                return 1 as libc::c_int != 0
            }
        }
    }
    return 0 as libc::c_int != 0;
}
unsafe extern "C" fn vtxProcessStateUpdate(mut vtxDevice: *mut vtxDevice_t)
 -> bool {
    let vtxSettingsState: vtxSettingsConfig_t = vtxGetSettings();
    let mut vtxState: vtxSettingsConfig_t = vtxSettingsState;
    if vtxSettingsState.band != 0 {
        vtxCommonGetBandAndChannel(vtxDevice, &mut vtxState.band,
                                   &mut vtxState.channel);
    } else { vtxCommonGetFrequency(vtxDevice, &mut vtxState.freq); }
    vtxCommonGetPowerIndex(vtxDevice, &mut vtxState.power);
    return memcmp(&vtxSettingsState as *const vtxSettingsConfig_t as
                      *const libc::c_void,
                  &mut vtxState as *mut vtxSettingsConfig_t as
                      *const libc::c_void,
                  ::core::mem::size_of::<vtxSettingsConfig_t>() as
                      libc::c_ulong) != 0;
}
#[no_mangle]
pub unsafe extern "C" fn vtxUpdate(mut currentTimeUs: timeUs_t) {
    static mut currentSchedule: uint8_t = 0 as libc::c_int as uint8_t;
    if cliMode != 0 { return }
    let mut vtxDevice: *mut vtxDevice_t = vtxCommonDevice();
    if !vtxDevice.is_null() {
        // Check input sources for config updates
        vtxControlInputPoll();
        let startingSchedule: uint8_t = currentSchedule;
        let mut vtxUpdatePending: bool = 0 as libc::c_int != 0;
        loop  {
            match currentSchedule as libc::c_int {
                0 => { vtxUpdatePending = vtxProcessPower(vtxDevice) }
                1 => {
                    if vtxGetSettings().band != 0 {
                        vtxUpdatePending = vtxProcessBandAndChannel(vtxDevice)
                    } else {
                        vtxUpdatePending = vtxProcessFrequency(vtxDevice)
                    }
                }
                2 => { vtxUpdatePending = vtxProcessPitMode(vtxDevice) }
                3 => { vtxUpdatePending = vtxProcessStateUpdate(vtxDevice) }
                _ => { }
            }
            currentSchedule =
                ((currentSchedule as libc::c_int + 1 as libc::c_int) %
                     VTX_PARAM_COUNT as libc::c_int) as uint8_t;
            if !(!vtxUpdatePending &&
                     currentSchedule as libc::c_int !=
                         startingSchedule as libc::c_int) {
                break ;
            }
        }
        if armingFlags as libc::c_int & ARMED as libc::c_int == 0 ||
               vtxUpdatePending as libc::c_int != 0 {
            vtxCommonProcess(vtxDevice, currentTimeUs);
        }
    };
}
