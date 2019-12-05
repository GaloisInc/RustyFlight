use core;
use libc;
extern "C" {
    #[no_mangle]
    fn memcmp(_: *const libc::c_void, _: *const libc::c_void,
              _: libc::c_ulong) -> libc::c_int;
    // Disabling this, in favour of tfp_format to be used in cli.c
//int tfp_printf(const char *fmt, ...);
    #[no_mangle]
    fn tfp_sprintf(s: *mut libc::c_char, fmt: *const libc::c_char, _: ...)
     -> libc::c_int;
    #[no_mangle]
    fn cmsMenuChange(pPort: *mut displayPort_t, ptr: *const libc::c_void)
     -> libc::c_long;
    #[no_mangle]
    fn saveConfigAndNotify();
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
    fn vtx58_Bandchan2Freq(band: uint8_t, channel: uint8_t) -> uint16_t;
    #[no_mangle]
    static mut saDevice: smartAudioDevice_t;
    #[no_mangle]
    static mut saPowerTable: [saPowerTable_t; 0];
    #[no_mangle]
    static saPowerNames: [*const libc::c_char; 0];
    #[no_mangle]
    static mut saDeferred: bool;
    #[no_mangle]
    fn saDacToPowerIndex(dac: libc::c_int) -> libc::c_int;
    #[no_mangle]
    fn saSetBandAndChannel(band: uint8_t, channel: uint8_t);
    #[no_mangle]
    fn saSetMode(mode: libc::c_int);
    #[no_mangle]
    fn saSetPitFreq(freq: uint16_t);
    #[no_mangle]
    static mut vtxSettingsConfig_System: vtxSettingsConfig_t;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int16_t = __int16_t;
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
// CMS state
// displayPort_t is used as a parameter group in 'displayport_msp.h' and 'displayport_max7456`.h'. Treat accordingly!
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
pub type OSD_MenuElement = libc::c_uint;
pub const OME_MAX: OSD_MenuElement = 15;
pub const OME_MENU: OSD_MenuElement = 15;
pub const OME_END: OSD_MenuElement = 14;
pub const OME_TAB: OSD_MenuElement = 13;
pub const OME_VISIBLE: OSD_MenuElement = 12;
pub const OME_FLOAT: OSD_MenuElement = 11;
pub const OME_String: OSD_MenuElement = 10;
pub const OME_INT16: OSD_MenuElement = 9;
pub const OME_UINT16: OSD_MenuElement = 8;
pub const OME_UINT8: OSD_MenuElement = 7;
pub const OME_INT8: OSD_MenuElement = 6;
pub const OME_Bool: OSD_MenuElement = 5;
pub const OME_Funcall: OSD_MenuElement = 4;
pub const OME_Submenu: OSD_MenuElement = 3;
pub const OME_OSD_Exit: OSD_MenuElement = 2;
pub const OME_Back: OSD_MenuElement = 1;
pub const OME_Label: OSD_MenuElement = 0;
pub type CMSEntryFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut displayPort_t, _: *const libc::c_void)
               -> libc::c_long>;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct OSD_Entry {
    pub text: *const libc::c_char,
    pub type_0: OSD_MenuElement,
    pub func: CMSEntryFuncPtr,
    pub data: *mut libc::c_void,
    pub flags: uint8_t,
}
// (Temporary) Flag for OME_Submenu, indicating func should be called to get a string to display.
pub type CMSMenuFuncPtr = Option<unsafe extern "C" fn() -> libc::c_long>;
// Causes automatic cmsMenuBack
/*
onExit function is called with self:
(1) Pointer to an OSD_Entry when cmsMenuBack() was called.
    Point to an OSD_Entry with type == OME_Back if BACK was selected.
(2) NULL if called from menu exit (forced exit at top level).
*/
pub type CMSMenuOnExitPtr
    =
    Option<unsafe extern "C" fn(_: *const OSD_Entry) -> libc::c_long>;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct CMS_Menu {
    pub onEnter: CMSMenuFuncPtr,
    pub onExit: CMSMenuOnExitPtr,
    pub entries: *mut OSD_Entry,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct OSD_UINT16_t {
    pub val: *mut uint16_t,
    pub min: uint16_t,
    pub max: uint16_t,
    pub step: uint16_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct OSD_TAB_t {
    pub val: *mut uint8_t,
    pub max: uint8_t,
    pub names: *const *const libc::c_char,
}
// Immediate
// ~UNLOCK
// SetFrequency flags, for pit mode frequency manipulation
// For generic API use, but here for now
pub type smartAudioDevice_t = smartAudioDevice_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct smartAudioDevice_s {
    pub version: int8_t,
    pub channel: int8_t,
    pub power: int8_t,
    pub mode: int8_t,
    pub freq: uint16_t,
    pub orfreq: uint16_t,
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
pub type vtxSettingsConfig_t = vtxSettingsConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct vtxSettingsConfig_s {
    pub band: uint8_t,
    pub channel: uint8_t,
    pub power: uint8_t,
    pub freq: uint16_t,
    pub pitModeFreq: uint16_t,
    pub lowPowerDisarm: uint8_t,
}
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
// Race model: Power up in pit mode
#[no_mangle]
pub static mut saCmsOpmodel: uint8_t = 0i32 as uint8_t;
#[no_mangle]
pub static mut saCmsRFState: uint8_t = 0;
// RF state; ACTIVE, PIR, POR XXX Not currently used
#[no_mangle]
pub static mut saCmsBand: uint8_t = 0i32 as uint8_t;
#[no_mangle]
pub static mut saCmsChan: uint8_t = 0i32 as uint8_t;
// Frequency derived from channel table (used for reference in band/channel mode)
#[no_mangle]
pub static mut saCmsFreqRef: uint16_t = 0i32 as uint16_t;
#[no_mangle]
pub static mut saCmsDeviceFreq: uint16_t = 0i32 as uint16_t;
#[no_mangle]
pub static mut saCmsDeviceStatus: uint8_t = 0i32 as uint8_t;
#[no_mangle]
pub static mut saCmsPower: uint8_t = 0i32 as uint8_t;
#[no_mangle]
pub static mut saCmsPitFMode: uint8_t = 0;
// Undef(0), In-Range(1) or Out-Range(2)
#[no_mangle]
pub static mut saCmsFselMode: uint8_t = 0;
// Channel(0) or User defined(1)
#[no_mangle]
pub static mut saCmsFselModeNew: uint8_t = 0;
// Channel(0) or User defined(1)
#[no_mangle]
pub static mut saCmsORFreq: uint16_t = 0i32 as uint16_t;
// POR frequency
#[no_mangle]
pub static mut saCmsORFreqNew: uint16_t = 0;
// POR frequency
#[no_mangle]
pub static mut saCmsUserFreq: uint16_t = 0i32 as uint16_t;
// User defined frequency
#[no_mangle]
pub static mut saCmsUserFreqNew: uint16_t = 0;
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
// User defined frequency
#[no_mangle]
pub unsafe extern "C" fn saCmsUpdate() {
    // XXX Take care of pit mode update somewhere???
    if saCmsOpmodel as libc::c_int == 0i32 {
        // This is a first valid response to GET_SETTINGS.
        saCmsOpmodel =
            if saDevice.mode as libc::c_int & 2i32 != 0 { 2i32 } else { 1i32 }
                as uint8_t;
        saCmsFselMode =
            if saDevice.mode as libc::c_int & 1i32 != 0 { 1i32 } else { 0i32 }
                as uint8_t;
        saCmsBand = (*vtxSettingsConfig()).band;
        saCmsChan = (*vtxSettingsConfig()).channel;
        saCmsFreqRef = (*vtxSettingsConfig()).freq;
        saCmsDeviceFreq = saCmsFreqRef;
        if saDevice.mode as libc::c_int & 2i32 == 0i32 {
            saCmsRFState = 3i32 as uint8_t
        } else if saDevice.mode as libc::c_int & 4i32 != 0 {
            saCmsRFState = 2i32 as uint8_t
        } else { saCmsRFState = 1i32 as uint8_t }
        saCmsPower = (*vtxSettingsConfig()).power;
        //init mode for menu
        if saCmsFselMode as libc::c_int != 0 &&
               (*vtxSettingsConfig()).freq as libc::c_int != 0 {
            saCmsUserFreq = (*vtxSettingsConfig()).freq
        }
        saCmsFselModeNew = saCmsFselMode
    }
    saUpdateStatusString();
}
#[no_mangle]
pub static mut saCmsStatusString: [libc::c_char; 31] =
    [45, 32, 45, 45, 32, 45, 45, 45, 45, 32, 45, 45, 45, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
#[no_mangle]
pub unsafe extern "C" fn saUpdateStatusString() {
    if saDevice.version as libc::c_int == 0i32 { return }
    // if user-freq mode then track possible change
    // XXX These should be done somewhere else
    if saCmsDeviceStatus as libc::c_int == 0i32 &&
           saDevice.version as libc::c_int != 0i32 {
        saCmsDeviceStatus = saDevice.version as uint8_t
    }
    if saCmsORFreq as libc::c_int == 0i32 &&
           saDevice.orfreq as libc::c_int != 0i32 {
        saCmsORFreq = saDevice.orfreq
    }
    if saCmsUserFreq as libc::c_int == 0i32 &&
           saDevice.freq as libc::c_int != 0i32 {
        saCmsUserFreq = saDevice.freq
    }
    if saDevice.version as libc::c_int == 2i32 {
        if saDevice.mode as libc::c_int & 8i32 != 0 {
            saCmsPitFMode = 1i32 as uint8_t
        } else { saCmsPitFMode = 0i32 as uint8_t }
    }
    saCmsStatusString[0] =
        (*::core::mem::transmute::<&[u8; 4],
                                   &[libc::c_char; 4]>(b"-FR\x00"))[saCmsOpmodel
                                                                        as
                                                                        usize];
    if saCmsFselMode as libc::c_int == 0i32 {
        saCmsStatusString[2] =
            (*::core::mem::transmute::<&[u8; 6],
                                       &[libc::c_char; 6]>(b"ABEFR\x00"))[(saDevice.channel
                                                                               as
                                                                               libc::c_int
                                                                               /
                                                                               8i32)
                                                                              as
                                                                              usize];
        saCmsStatusString[3] =
            ('1' as i32 + saDevice.channel as libc::c_int % 8i32) as
                libc::c_char
    } else {
        saCmsStatusString[2] = 'U' as i32 as libc::c_char;
        saCmsStatusString[3] = 'F' as i32 as libc::c_char
    }
    if saDevice.mode as libc::c_int & 2i32 != 0 &&
           saDevice.mode as libc::c_int & 8i32 != 0 {
        tfp_sprintf(&mut *saCmsStatusString.as_mut_ptr().offset(5) as
                        *mut libc::c_char,
                    b"%4d\x00" as *const u8 as *const libc::c_char,
                    saDevice.orfreq as libc::c_int);
    } else if saDevice.mode as libc::c_int & 1i32 != 0 {
        tfp_sprintf(&mut *saCmsStatusString.as_mut_ptr().offset(5) as
                        *mut libc::c_char,
                    b"%4d\x00" as *const u8 as *const libc::c_char,
                    saDevice.freq as libc::c_int);
    } else {
        tfp_sprintf(&mut *saCmsStatusString.as_mut_ptr().offset(5) as
                        *mut libc::c_char,
                    b"%4d\x00" as *const u8 as *const libc::c_char,
                    vtx58frequencyTable[(saDevice.channel as libc::c_int /
                                             8i32) as
                                            usize][(saDevice.channel as
                                                        libc::c_int % 8i32) as
                                                       usize] as libc::c_int);
    }
    saCmsStatusString[9] = ' ' as i32 as libc::c_char;
    if saDevice.mode as libc::c_int & 2i32 != 0 {
        saCmsStatusString[10] = 'P' as i32 as libc::c_char;
        if saDevice.mode as libc::c_int & 4i32 != 0 {
            saCmsStatusString[11] = 'I' as i32 as libc::c_char
        } else { saCmsStatusString[11] = 'O' as i32 as libc::c_char }
        saCmsStatusString[12] = 'R' as i32 as libc::c_char;
        saCmsStatusString[13] = 0i32 as libc::c_char
    } else {
        tfp_sprintf(&mut *saCmsStatusString.as_mut_ptr().offset(10) as
                        *mut libc::c_char,
                    b"%3d\x00" as *const u8 as *const libc::c_char,
                    if saDevice.version as libc::c_int == 2i32 {
                        (*saPowerTable.as_mut_ptr().offset(saDevice.power as
                                                               isize)).rfpower
                    } else {
                        (*saPowerTable.as_mut_ptr().offset(saDacToPowerIndex(saDevice.power
                                                                                 as
                                                                                 libc::c_int)
                                                               as
                                                               isize)).rfpower
                    });
    };
}
#[no_mangle]
pub unsafe extern "C" fn saCmsResetOpmodel() {
    // trigger data refresh in 'saCmsUpdate()'
    saCmsOpmodel = 0i32 as uint8_t;
}
unsafe extern "C" fn saCmsConfigBandByGvar(mut pDisp: *mut displayPort_t,
                                           mut self_0: *const libc::c_void)
 -> libc::c_long {
    if saDevice.version as libc::c_int == 0i32 {
        // Bounce back; not online yet
        saCmsBand = 0i32 as uint8_t;
        return 0i32 as libc::c_long
    }
    if saCmsBand as libc::c_int == 0i32 {
        // Bouce back, no going back to undef state
        saCmsBand = 1i32 as uint8_t;
        return 0i32 as libc::c_long
    }
    if saCmsOpmodel as libc::c_int == 1i32 && !saDeferred {
        saSetBandAndChannel((saCmsBand as libc::c_int - 1i32) as uint8_t,
                            (saCmsChan as libc::c_int - 1i32) as uint8_t);
    }
    saCmsFreqRef =
        vtx58frequencyTable[(saCmsBand as libc::c_int - 1i32) as
                                usize][(saCmsChan as libc::c_int - 1i32) as
                                           usize];
    return 0i32 as libc::c_long;
}
unsafe extern "C" fn saCmsConfigChanByGvar(mut pDisp: *mut displayPort_t,
                                           mut self_0: *const libc::c_void)
 -> libc::c_long {
    if saDevice.version as libc::c_int == 0i32 {
        // Bounce back; not online yet
        saCmsChan = 0i32 as uint8_t;
        return 0i32 as libc::c_long
    }
    if saCmsChan as libc::c_int == 0i32 {
        // Bounce back; no going back to undef state
        saCmsChan = 1i32 as uint8_t;
        return 0i32 as libc::c_long
    }
    if saCmsOpmodel as libc::c_int == 1i32 && !saDeferred {
        saSetBandAndChannel((saCmsBand as libc::c_int - 1i32) as uint8_t,
                            (saCmsChan as libc::c_int - 1i32) as uint8_t);
    }
    saCmsFreqRef =
        vtx58frequencyTable[(saCmsBand as libc::c_int - 1i32) as
                                usize][(saCmsChan as libc::c_int - 1i32) as
                                           usize];
    return 0i32 as libc::c_long;
}
unsafe extern "C" fn saCmsConfigPowerByGvar(mut pDisp: *mut displayPort_t,
                                            mut self_0: *const libc::c_void)
 -> libc::c_long {
    if saDevice.version as libc::c_int == 0i32 {
        // Bounce back; not online yet
        saCmsPower = 0i32 as uint8_t;
        return 0i32 as libc::c_long
    }
    if saCmsPower as libc::c_int == 0i32 {
        // Bouce back; no going back to undef state
        saCmsPower = 1i32 as uint8_t;
        return 0i32 as libc::c_long
    }
    if saCmsOpmodel as libc::c_int == 1i32 && !saDeferred {
        (*vtxSettingsConfigMutable()).power = saCmsPower
    }
    return 0i32 as libc::c_long;
}
unsafe extern "C" fn saCmsConfigPitFModeByGvar(mut pDisp: *mut displayPort_t,
                                               mut self_0:
                                                   *const libc::c_void)
 -> libc::c_long {
    if saDevice.version as libc::c_int == 1i32 {
        // V1 device doesn't support PIT mode; bounce back.
        saCmsPitFMode = 0i32 as uint8_t;
        return 0i32 as libc::c_long
    }
    if saCmsPitFMode as libc::c_int == 0i32 {
        // Bounce back
        saCmsPitFMode = 1i32 as uint8_t;
        return 0i32 as libc::c_long
    }
    if saCmsPitFMode as libc::c_int == 1i32 {
        saSetMode(1i32);
    } else { saSetMode(2i32); }
    return 0i32 as libc::c_long;
}
//                            m bc ffff ppp
//                            0123456789012
// Forward
unsafe extern "C" fn saCmsConfigOpmodelByGvar(mut pDisp: *mut displayPort_t,
                                              mut self_0: *const libc::c_void)
 -> libc::c_long {
    if saDevice.version as libc::c_int == 1i32 {
        if saCmsOpmodel as libc::c_int != 1i32 {
            saCmsOpmodel = 1i32 as uint8_t
        }
        return 0i32 as libc::c_long
    }
    let mut opmodel: uint8_t = saCmsOpmodel;
    if opmodel as libc::c_int == 1i32 {
        // VTX should power up transmitting.
        // Turn off In-Range and Out-Range bits
        saSetMode(0i32);
    } else if opmodel as libc::c_int == 2i32 {
        // VTX should power up in pit mode.
        // Default PitFMode is in-range to prevent users without
        // out-range receivers from getting blinded.
        saCmsPitFMode = 0i32 as uint8_t;
        saCmsConfigPitFModeByGvar(pDisp, self_0);
        // Direct frequency mode is not available in RACE opmodel
        saCmsFselModeNew = 0i32 as uint8_t;
        saCmsConfigFreqModeByGvar(pDisp, self_0);
    } else {
        // Trying to go back to unknown state; bounce back
        saCmsOpmodel = (0i32 + 1i32) as uint8_t
    }
    return 0i32 as libc::c_long;
}
/* USE_EXTENDED_CMS_MENUS */
static mut saCmsEntBand: OSD_TAB_t =
    unsafe {
        {
            let mut init =
                OSD_TAB_t{val: &saCmsBand as *const uint8_t as *mut uint8_t,
                          max: (5i32 - 1i32 + 1i32) as uint8_t,
                          names: vtx58BandNames.as_ptr(),};
            init
        }
    };
static mut saCmsEntChan: OSD_TAB_t =
    unsafe {
        {
            let mut init =
                OSD_TAB_t{val: &saCmsChan as *const uint8_t as *mut uint8_t,
                          max: (8i32 - 1i32 + 1i32) as uint8_t,
                          names: vtx58ChannelNames.as_ptr(),};
            init
        }
    };
static mut saCmsEntPower: OSD_TAB_t =
    unsafe {
        {
            let mut init =
                OSD_TAB_t{val: &saCmsPower as *const uint8_t as *mut uint8_t,
                          max: 4i32 as uint8_t,
                          names: saPowerNames.as_ptr(),};
            init
        }
    };
static mut saCmsEntFreqRef: OSD_UINT16_t =
    unsafe {
        {
            let mut init =
                OSD_UINT16_t{val:
                                 &saCmsFreqRef as *const uint16_t as
                                     *mut uint16_t,
                             min: 5600i32 as uint16_t,
                             max: 5900i32 as uint16_t,
                             step: 0i32 as uint16_t,};
            init
        }
    };
static mut saCmsOpmodelNames: [*const libc::c_char; 3] =
    [b"----\x00" as *const u8 as *const libc::c_char,
     b"FREE\x00" as *const u8 as *const libc::c_char,
     b"RACE\x00" as *const u8 as *const libc::c_char];
static mut saCmsFselModeNames: [*const libc::c_char; 2] =
    [b"CHAN\x00" as *const u8 as *const libc::c_char,
     b"USER\x00" as *const u8 as *const libc::c_char];
static mut saCmsPitFModeNames: [*const libc::c_char; 3] =
    [b"---\x00" as *const u8 as *const libc::c_char,
     b"PIR\x00" as *const u8 as *const libc::c_char,
     b"POR\x00" as *const u8 as *const libc::c_char];
static mut saCmsEntPitFMode: OSD_TAB_t =
    unsafe {
        {
            let mut init =
                OSD_TAB_t{val:
                              &saCmsPitFMode as *const uint8_t as
                                  *mut uint8_t,
                          max: 1i32 as uint8_t,
                          names: saCmsPitFModeNames.as_ptr(),};
            init
        }
    };
// Forward
unsafe extern "C" fn saCmsConfigFreqModeByGvar(mut pDisp: *mut displayPort_t,
                                               mut self_0:
                                                   *const libc::c_void)
 -> libc::c_long {
    // if trying to do user frequency mode in RACE opmodel then
    // revert because user-freq only available in FREE opmodel
    if saCmsFselModeNew as libc::c_int != 0i32 &&
           saCmsOpmodel as libc::c_int != 1i32 {
        saCmsFselModeNew = 0i32 as uint8_t
    }
    // don't call 'saSetBandAndChannel()' / 'saSetFreq()' here,
    // wait until SET / 'saCmsCommence()' is activated
    sacms_SetupTopMenu();
    return 0i32 as libc::c_long;
}
unsafe extern "C" fn saCmsCommence(mut pDisp: *mut displayPort_t,
                                   mut self_0: *const libc::c_void)
 -> libc::c_long {
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
    if saCmsOpmodel as libc::c_int == 2i32 {
        // Race model
        // Setup band, freq and power.
        newSettings.band = saCmsBand;
        newSettings.channel = saCmsChan;
        newSettings.freq = vtx58_Bandchan2Freq(saCmsBand, saCmsChan);
        // If in pit mode, cancel it.
        if saCmsPitFMode as libc::c_int == 0i32 {
            saSetMode(4i32 | 1i32);
        } else { saSetMode(4i32 | 2i32); }
    } else if saCmsFselModeNew as libc::c_int == 0i32 {
        newSettings.band = saCmsBand;
        newSettings.channel = saCmsChan;
        newSettings.freq = vtx58_Bandchan2Freq(saCmsBand, saCmsChan)
    } else {
        // Freestyle model
        // Setup band and freq / user freq
        saSetMode(0i32); //make sure FREE mode is setup
        newSettings.band = 0i32 as uint8_t;
        newSettings.freq = saCmsUserFreq
    }
    newSettings.power = saCmsPower;
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
    }
    return -1i32 as libc::c_long;
}
unsafe extern "C" fn saCmsSetPORFreqOnEnter() -> libc::c_long {
    if saDevice.version as libc::c_int == 1i32 {
        return -1i32 as libc::c_long
    }
    saCmsORFreqNew = saCmsORFreq;
    return 0i32 as libc::c_long;
}
unsafe extern "C" fn saCmsSetPORFreq(mut pDisp: *mut displayPort_t,
                                     mut self_0: *const libc::c_void)
 -> libc::c_long {
    saSetPitFreq(saCmsORFreqNew);
    return 0i32 as libc::c_long;
}
unsafe extern "C" fn saCmsORFreqGetString() -> *mut libc::c_char {
    static mut pbuf: [libc::c_char; 5] = [0; 5];
    tfp_sprintf(pbuf.as_mut_ptr(),
                b"%4d\x00" as *const u8 as *const libc::c_char,
                saCmsORFreq as libc::c_int);
    return pbuf.as_mut_ptr();
}
unsafe extern "C" fn saCmsUserFreqGetString() -> *mut libc::c_char {
    static mut pbuf: [libc::c_char; 5] = [0; 5];
    tfp_sprintf(pbuf.as_mut_ptr(),
                b"%4d\x00" as *const u8 as *const libc::c_char,
                saCmsUserFreq as libc::c_int);
    return pbuf.as_mut_ptr();
}
unsafe extern "C" fn saCmsSetUserFreqOnEnter() -> libc::c_long {
    saCmsUserFreqNew = saCmsUserFreq;
    return 0i32 as libc::c_long;
}
unsafe extern "C" fn saCmsConfigUserFreq(mut pDisp: *mut displayPort_t,
                                         mut self_0: *const libc::c_void)
 -> libc::c_long {
    saCmsUserFreq = saCmsUserFreqNew;
    return -1i32 as libc::c_long;
}
static mut cur_freq_data: OSD_UINT16_t =
    unsafe {
        {
            let mut init =
                OSD_UINT16_t{val:
                                 &saCmsORFreq as *const uint16_t as
                                     *mut uint16_t,
                             min: 5000i32 as uint16_t,
                             max: 5999i32 as uint16_t,
                             step: 0i32 as uint16_t,};
            init
        }
    };
static mut new_freq_data: OSD_UINT16_t =
    unsafe {
        {
            let mut init =
                OSD_UINT16_t{val:
                                 &saCmsORFreqNew as *const uint16_t as
                                     *mut uint16_t,
                             min: 5000i32 as uint16_t,
                             max: 5999i32 as uint16_t,
                             step: 1i32 as uint16_t,};
            init
        }
    };
static mut saCmsMenuPORFreqEntries: [OSD_Entry; 6] =
    unsafe {
        [{
             let mut init =
                 OSD_Entry{text:
                               b"- POR FREQ -\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Label,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"CUR FREQ\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT16,
                           func: None,
                           data:
                               &cur_freq_data as *const OSD_UINT16_t as
                                   *mut OSD_UINT16_t as *mut libc::c_void,
                           flags: 0x4i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"NEW FREQ\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT16,
                           func: None,
                           data:
                               &new_freq_data as *const OSD_UINT16_t as
                                   *mut OSD_UINT16_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"SET\x00" as *const u8 as *const libc::c_char,
                           type_0: OME_Funcall,
                           func:
                               Some(saCmsSetPORFreq as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"BACK\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Back,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text: 0 as *const libc::c_char,
                           type_0: OME_END,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         }]
    };
static mut saCmsMenuPORFreq: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter:
                             Some(saCmsSetPORFreqOnEnter as
                                      unsafe extern "C" fn() -> libc::c_long),
                         onExit: None,
                         entries:
                             saCmsMenuPORFreqEntries.as_ptr() as *mut _,};
            init
        }
    };
static mut user_cur_freq_data: OSD_UINT16_t =
    unsafe {
        {
            let mut init =
                OSD_UINT16_t{val:
                                 &saCmsUserFreq as *const uint16_t as
                                     *mut uint16_t,
                             min: 5000i32 as uint16_t,
                             max: 5999i32 as uint16_t,
                             step: 0i32 as uint16_t,};
            init
        }
    };
static mut user_new_freq_data: OSD_UINT16_t =
    unsafe {
        {
            let mut init =
                OSD_UINT16_t{val:
                                 &saCmsUserFreqNew as *const uint16_t as
                                     *mut uint16_t,
                             min: 5000i32 as uint16_t,
                             max: 5999i32 as uint16_t,
                             step: 1i32 as uint16_t,};
            init
        }
    };
static mut saCmsMenuUserFreqEntries: [OSD_Entry; 6] =
    unsafe {
        [{
             let mut init =
                 OSD_Entry{text:
                               b"- USER FREQ -\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Label,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"CUR FREQ\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT16,
                           func: None,
                           data:
                               &user_cur_freq_data as *const OSD_UINT16_t as
                                   *mut OSD_UINT16_t as *mut libc::c_void,
                           flags: 0x4i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"NEW FREQ\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT16,
                           func: None,
                           data:
                               &user_new_freq_data as *const OSD_UINT16_t as
                                   *mut OSD_UINT16_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"SET\x00" as *const u8 as *const libc::c_char,
                           type_0: OME_Funcall,
                           func:
                               Some(saCmsConfigUserFreq as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"BACK\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Back,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text: 0 as *const libc::c_char,
                           type_0: OME_END,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         }]
    };
static mut saCmsMenuUserFreq: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter:
                             Some(saCmsSetUserFreqOnEnter as
                                      unsafe extern "C" fn() -> libc::c_long),
                         onExit: None,
                         entries:
                             saCmsMenuUserFreqEntries.as_ptr() as *mut _,};
            init
        }
    };
static mut saCmsEntFselMode: OSD_TAB_t =
    unsafe {
        {
            let mut init =
                OSD_TAB_t{val:
                              &saCmsFselModeNew as *const uint8_t as
                                  *mut uint8_t,
                          max: 1i32 as uint8_t,
                          names: saCmsFselModeNames.as_ptr(),};
            init
        }
    };
static mut op_model_data: OSD_TAB_t =
    unsafe {
        {
            let mut init =
                OSD_TAB_t{val:
                              &saCmsOpmodel as *const uint8_t as *mut uint8_t,
                          max: 2i32 as uint8_t,
                          names: saCmsOpmodelNames.as_ptr(),};
            init
        }
    };
static mut saCmsMenuConfigEntries: [OSD_Entry; 7] =
    unsafe {
        [{
             let mut init =
                 OSD_Entry{text:
                               b"- SA CONFIG -\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Label,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"OP MODEL\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_TAB,
                           func:
                               Some(saCmsConfigOpmodelByGvar as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data:
                               &op_model_data as *const OSD_TAB_t as
                                   *mut OSD_TAB_t as *mut libc::c_void,
                           flags: 0x4i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"FSEL MODE\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_TAB,
                           func:
                               Some(saCmsConfigFreqModeByGvar as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data:
                               &saCmsEntFselMode as *const OSD_TAB_t as
                                   *mut OSD_TAB_t as *mut libc::c_void,
                           flags: 0x4i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"PIT FMODE\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_TAB,
                           func:
                               Some(saCmsConfigPitFModeByGvar as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data:
                               &saCmsEntPitFMode as *const OSD_TAB_t as
                                   *mut OSD_TAB_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"POR FREQ\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Submenu,
                           func:
                               ::core::mem::transmute::<Option<unsafe extern "C" fn()
                                                                   ->
                                                                       *mut libc::c_char>,
                                                        CMSEntryFuncPtr>(Some(saCmsORFreqGetString
                                                                                  as
                                                                                  unsafe extern "C" fn()
                                                                                      ->
                                                                                          *mut libc::c_char)),
                           data:
                               &saCmsMenuPORFreq as *const CMS_Menu as
                                   *mut CMS_Menu as *mut libc::c_void,
                           flags: 0x8i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"BACK\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Back,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text: 0 as *const libc::c_char,
                           type_0: OME_END,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         }]
    };
static mut saCmsMenuConfig: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter: None,
                         onExit: None,
                         entries: saCmsMenuConfigEntries.as_ptr() as *mut _,};
            init
        }
    };
static mut saCmsMenuCommenceEntries: [OSD_Entry; 4] =
    unsafe {
        [{
             let mut init =
                 OSD_Entry{text:
                               b"CONFIRM\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Label,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"YES\x00" as *const u8 as *const libc::c_char,
                           type_0: OME_Funcall,
                           func:
                               Some(saCmsCommence as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"BACK\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Back,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text: 0 as *const libc::c_char,
                           type_0: OME_END,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         }]
    };
static mut saCmsMenuCommence: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter: None,
                         onExit: None,
                         entries:
                             saCmsMenuCommenceEntries.as_ptr() as *mut _,};
            init
        }
    };
static mut saCmsMenuFreqModeEntries: [OSD_Entry; 8] =
    unsafe {
        [{
             let mut init =
                 OSD_Entry{text:
                               b"- SMARTAUDIO -\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Label,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text: b"\x00" as *const u8 as *const libc::c_char,
                           type_0: OME_Label,
                           func: None,
                           data:
                               saCmsStatusString.as_ptr() as *mut _ as
                                   *mut libc::c_void,
                           flags: 0x4i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"FREQ\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Submenu,
                           func:
                               ::core::mem::transmute::<Option<unsafe extern "C" fn()
                                                                   ->
                                                                       *mut libc::c_char>,
                                                        CMSEntryFuncPtr>(Some(saCmsUserFreqGetString
                                                                                  as
                                                                                  unsafe extern "C" fn()
                                                                                      ->
                                                                                          *mut libc::c_char)),
                           data:
                               &saCmsMenuUserFreq as *const CMS_Menu as
                                   *mut CMS_Menu as *mut libc::c_void,
                           flags: 0x8i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"POWER\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_TAB,
                           func:
                               Some(saCmsConfigPowerByGvar as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data:
                               &saCmsEntPower as *const OSD_TAB_t as
                                   *mut OSD_TAB_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"SET\x00" as *const u8 as *const libc::c_char,
                           type_0: OME_Submenu,
                           func:
                               Some(cmsMenuChange as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data:
                               &saCmsMenuCommence as *const CMS_Menu as
                                   *mut CMS_Menu as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"CONFIG\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Submenu,
                           func:
                               Some(cmsMenuChange as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data:
                               &saCmsMenuConfig as *const CMS_Menu as
                                   *mut CMS_Menu as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"BACK\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Back,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text: 0 as *const libc::c_char,
                           type_0: OME_END,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         }]
    };
static mut saCmsMenuChanModeEntries: [OSD_Entry; 10] =
    unsafe {
        [{
             let mut init =
                 OSD_Entry{text:
                               b"- SMARTAUDIO -\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Label,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text: b"\x00" as *const u8 as *const libc::c_char,
                           type_0: OME_Label,
                           func: None,
                           data:
                               saCmsStatusString.as_ptr() as *mut _ as
                                   *mut libc::c_void,
                           flags: 0x4i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"BAND\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_TAB,
                           func:
                               Some(saCmsConfigBandByGvar as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data:
                               &saCmsEntBand as *const OSD_TAB_t as
                                   *mut OSD_TAB_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"CHAN\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_TAB,
                           func:
                               Some(saCmsConfigChanByGvar as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data:
                               &saCmsEntChan as *const OSD_TAB_t as
                                   *mut OSD_TAB_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"(FREQ)\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT16,
                           func: None,
                           data:
                               &saCmsEntFreqRef as *const OSD_UINT16_t as
                                   *mut OSD_UINT16_t as *mut libc::c_void,
                           flags: 0x4i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"POWER\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_TAB,
                           func:
                               Some(saCmsConfigPowerByGvar as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data:
                               &saCmsEntPower as *const OSD_TAB_t as
                                   *mut OSD_TAB_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"SET\x00" as *const u8 as *const libc::c_char,
                           type_0: OME_Submenu,
                           func:
                               Some(cmsMenuChange as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data:
                               &saCmsMenuCommence as *const CMS_Menu as
                                   *mut CMS_Menu as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"CONFIG\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Submenu,
                           func:
                               Some(cmsMenuChange as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data:
                               &saCmsMenuConfig as *const CMS_Menu as
                                   *mut CMS_Menu as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"BACK\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Back,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text: 0 as *const libc::c_char,
                           type_0: OME_END,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         }]
    };
static mut saCmsMenuOfflineEntries: [OSD_Entry; 4] =
    unsafe {
        [{
             let mut init =
                 OSD_Entry{text:
                               b"- VTX SMARTAUDIO -\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Label,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text: b"\x00" as *const u8 as *const libc::c_char,
                           type_0: OME_Label,
                           func: None,
                           data:
                               saCmsStatusString.as_ptr() as *mut _ as
                                   *mut libc::c_void,
                           flags: 0x4i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"BACK\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Back,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text: 0 as *const libc::c_char,
                           type_0: OME_END,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         }]
    };
#[no_mangle]
pub static mut cmsx_menuVtxSmartAudio: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter:
                             Some(sacms_SetupTopMenu as
                                      unsafe extern "C" fn() -> libc::c_long),
                         onExit: None,
                         entries:
                             saCmsMenuOfflineEntries.as_ptr() as *mut _,};
            init
        }
    };
unsafe extern "C" fn sacms_SetupTopMenu() -> libc::c_long {
    if saCmsDeviceStatus != 0 {
        if saCmsFselModeNew as libc::c_int == 0i32 {
            cmsx_menuVtxSmartAudio.entries =
                saCmsMenuChanModeEntries.as_mut_ptr()
        } else {
            cmsx_menuVtxSmartAudio.entries =
                saCmsMenuFreqModeEntries.as_mut_ptr()
        }
    } else {
        cmsx_menuVtxSmartAudio.entries = saCmsMenuOfflineEntries.as_mut_ptr()
    }
    return 0i32 as libc::c_long;
}
// CMS