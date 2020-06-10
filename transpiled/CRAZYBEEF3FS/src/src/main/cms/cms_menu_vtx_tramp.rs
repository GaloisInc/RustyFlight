use ::libc;
extern "C" {
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
    static vtx58BandLetter: [libc::c_char; 0];
    #[no_mangle]
    fn vtx58_Bandchan2Freq(band: uint8_t, channel: uint8_t) -> uint16_t;
    //min freq in MHz
    //max freq in MHz
    #[no_mangle]
    static trampPowerTable: [uint16_t; 5];
    #[no_mangle]
    static trampPowerNames: [*const libc::c_char; 6];
    #[no_mangle]
    static mut trampBand: uint8_t;
    #[no_mangle]
    static mut trampChannel: uint8_t;
    #[no_mangle]
    static mut trampPower: uint16_t;
    // Actual transmitting power
    #[no_mangle]
    static mut trampPitMode: uint8_t;
    #[no_mangle]
    static mut trampCurFreq: uint32_t;
    #[no_mangle]
    static mut trampConfiguredPower: uint16_t;
    // Configured transmitting power
    #[no_mangle]
    static mut trampTemperature: int16_t;
    #[no_mangle]
    fn trampCommitChanges() -> bool;
    #[no_mangle]
    fn trampSetPitMode(onoff: uint8_t);
    #[no_mangle]
    fn trampSetBandAndChannel(band: uint8_t, channel: uint8_t);
    #[no_mangle]
    fn trampSetRFPower(level: uint16_t);
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct OSD_Entry {
    pub text: *const libc::c_char,
    pub type_0: OSD_MenuElement,
    pub func: CMSEntryFuncPtr,
    pub data: *mut libc::c_void,
    pub flags: uint8_t,
}
// Value should be updated dynamically
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct CMS_Menu {
    pub onEnter: CMSMenuFuncPtr,
    pub onExit: CMSMenuOnExitPtr,
    pub entries: *mut OSD_Entry,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct OSD_INT16_t {
    pub val: *mut int16_t,
    pub min: int16_t,
    pub max: int16_t,
    pub step: int16_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct OSD_UINT16_t {
    pub val: *mut uint16_t,
    pub min: uint16_t,
    pub max: uint16_t,
    pub step: uint16_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct OSD_TAB_t {
    pub val: *mut uint8_t,
    pub max: uint8_t,
    pub names: *const *const libc::c_char,
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
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
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
pub static mut trampCmsStatusString: [libc::c_char; 31] =
    [45, 32, 45, 45, 32, 45, 45, 45, 45, 32, 45, 45, 45, 45, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
//                               m bc ffff tppp
//                               01234567890123
#[no_mangle]
pub unsafe extern "C" fn trampCmsUpdateStatusString() {
    trampCmsStatusString[0 as libc::c_int as usize] =
        '*' as i32 as libc::c_char;
    trampCmsStatusString[1 as libc::c_int as usize] =
        ' ' as i32 as libc::c_char;
    trampCmsStatusString[2 as libc::c_int as usize] =
        *vtx58BandLetter.as_ptr().offset(trampBand as isize);
    trampCmsStatusString[3 as libc::c_int as usize] =
        *(*vtx58ChannelNames.as_ptr().offset(trampChannel as
                                                 isize)).offset(0 as
                                                                    libc::c_int
                                                                    as isize);
    trampCmsStatusString[4 as libc::c_int as usize] =
        ' ' as i32 as libc::c_char;
    if trampCurFreq != 0 {
        tfp_sprintf(&mut *trampCmsStatusString.as_mut_ptr().offset(5 as
                                                                       libc::c_int
                                                                       as
                                                                       isize)
                        as *mut libc::c_char,
                    b"%4d\x00" as *const u8 as *const libc::c_char,
                    trampCurFreq);
    } else {
        tfp_sprintf(&mut *trampCmsStatusString.as_mut_ptr().offset(5 as
                                                                       libc::c_int
                                                                       as
                                                                       isize)
                        as *mut libc::c_char,
                    b"----\x00" as *const u8 as *const libc::c_char);
    }
    if trampPower != 0 {
        tfp_sprintf(&mut *trampCmsStatusString.as_mut_ptr().offset(9 as
                                                                       libc::c_int
                                                                       as
                                                                       isize)
                        as *mut libc::c_char,
                    b" %c%3d\x00" as *const u8 as *const libc::c_char,
                    if trampPower as libc::c_int ==
                           trampConfiguredPower as libc::c_int {
                        ' ' as i32
                    } else { '*' as i32 }, trampPower as libc::c_int);
    } else {
        tfp_sprintf(&mut *trampCmsStatusString.as_mut_ptr().offset(9 as
                                                                       libc::c_int
                                                                       as
                                                                       isize)
                        as *mut libc::c_char,
                    b" ----\x00" as *const u8 as *const libc::c_char);
    };
}
#[no_mangle]
pub static mut trampCmsPitMode: uint8_t = 0 as libc::c_int as uint8_t;
#[no_mangle]
pub static mut trampCmsBand: uint8_t = 1 as libc::c_int as uint8_t;
#[no_mangle]
pub static mut trampCmsChan: uint8_t = 1 as libc::c_int as uint8_t;
#[no_mangle]
pub static mut trampCmsFreqRef: uint16_t = 0;
static mut trampCmsEntBand: OSD_TAB_t =
    unsafe {
        {
            let mut init =
                OSD_TAB_t{val:
                              &trampCmsBand as *const uint8_t as *mut uint8_t,
                          max:
                              (5 as libc::c_int - 1 as libc::c_int +
                                   1 as libc::c_int) as uint8_t,
                          names: vtx58BandNames.as_ptr(),};
            init
        }
    };
static mut trampCmsEntChan: OSD_TAB_t =
    unsafe {
        {
            let mut init =
                OSD_TAB_t{val:
                              &trampCmsChan as *const uint8_t as *mut uint8_t,
                          max:
                              (8 as libc::c_int - 1 as libc::c_int +
                                   1 as libc::c_int) as uint8_t,
                          names: vtx58ChannelNames.as_ptr(),};
            init
        }
    };
static mut trampCmsEntFreqRef: OSD_UINT16_t =
    unsafe {
        {
            let mut init =
                OSD_UINT16_t{val:
                                 &trampCmsFreqRef as *const uint16_t as
                                     *mut uint16_t,
                             min: 5600 as libc::c_int as uint16_t,
                             max: 5900 as libc::c_int as uint16_t,
                             step: 0 as libc::c_int as uint16_t,};
            init
        }
    };
static mut trampCmsPower: uint8_t = 1 as libc::c_int as uint8_t;
static mut trampCmsEntPower: OSD_TAB_t =
    unsafe {
        {
            let mut init =
                OSD_TAB_t{val:
                              &trampCmsPower as *const uint8_t as
                                  *mut uint8_t,
                          max:
                              ::core::mem::size_of::<[uint16_t; 5]>() as
                                  libc::c_ulong as uint8_t,
                          names: trampPowerNames.as_ptr(),};
            init
        }
    };
unsafe extern "C" fn trampCmsUpdateFreqRef() {
    if trampCmsBand as libc::c_int > 0 as libc::c_int &&
           trampCmsChan as libc::c_int > 0 as libc::c_int {
        trampCmsFreqRef =
            vtx58frequencyTable[(trampCmsBand as libc::c_int -
                                     1 as libc::c_int) as
                                    usize][(trampCmsChan as libc::c_int -
                                                1 as libc::c_int) as usize]
    };
}
unsafe extern "C" fn trampCmsConfigBand(mut pDisp: *mut displayPort_t,
                                        mut self_0: *const libc::c_void)
 -> libc::c_long {
    if trampCmsBand as libc::c_int == 0 as libc::c_int {
        // Bounce back
        trampCmsBand = 1 as libc::c_int as uint8_t
    } else { trampCmsUpdateFreqRef(); }
    return 0 as libc::c_int as libc::c_long;
}
unsafe extern "C" fn trampCmsConfigChan(mut pDisp: *mut displayPort_t,
                                        mut self_0: *const libc::c_void)
 -> libc::c_long {
    if trampCmsChan as libc::c_int == 0 as libc::c_int {
        // Bounce back
        trampCmsChan = 1 as libc::c_int as uint8_t
    } else { trampCmsUpdateFreqRef(); }
    return 0 as libc::c_int as libc::c_long;
}
unsafe extern "C" fn trampCmsConfigPower(mut pDisp: *mut displayPort_t,
                                         mut self_0: *const libc::c_void)
 -> libc::c_long {
    if trampCmsPower as libc::c_int == 0 as libc::c_int {
        // Bounce back
        trampCmsPower = 1 as libc::c_int as uint8_t
    }
    return 0 as libc::c_int as libc::c_long;
}
static mut trampCmsEntTemp: OSD_INT16_t =
    unsafe {
        {
            let mut init =
                OSD_INT16_t{val:
                                &trampTemperature as *const int16_t as
                                    *mut int16_t,
                            min: -(100 as libc::c_int) as int16_t,
                            max: 300 as libc::c_int as int16_t,
                            step: 0 as libc::c_int as int16_t,};
            init
        }
    };
static mut trampCmsPitModeNames: [*const libc::c_char; 3] =
    [b"---\x00" as *const u8 as *const libc::c_char,
     b"OFF\x00" as *const u8 as *const libc::c_char,
     b"ON \x00" as *const u8 as *const libc::c_char];
static mut trampCmsEntPitMode: OSD_TAB_t =
    unsafe {
        {
            let mut init =
                OSD_TAB_t{val:
                              &trampCmsPitMode as *const uint8_t as
                                  *mut uint8_t,
                          max: 2 as libc::c_int as uint8_t,
                          names: trampCmsPitModeNames.as_ptr(),};
            init
        }
    };
unsafe extern "C" fn trampCmsSetPitMode(mut pDisp: *mut displayPort_t,
                                        mut self_0: *const libc::c_void)
 -> libc::c_long {
    if trampCmsPitMode as libc::c_int == 0 as libc::c_int {
        // Bouce back
        trampCmsPitMode = 1 as libc::c_int as uint8_t
    } else {
        trampSetPitMode((trampCmsPitMode as libc::c_int - 1 as libc::c_int) as
                            uint8_t);
    }
    return 0 as libc::c_int as libc::c_long;
}
unsafe extern "C" fn trampCmsCommence(mut pDisp: *mut displayPort_t,
                                      mut self_0: *const libc::c_void)
 -> libc::c_long {
    trampSetBandAndChannel(trampCmsBand, trampCmsChan);
    trampSetRFPower(trampPowerTable[(trampCmsPower as libc::c_int -
                                         1 as libc::c_int) as usize]);
    // If it fails, the user should retry later
    trampCommitChanges();
    // update'vtx_' settings
    (*vtxSettingsConfigMutable()).band = trampCmsBand;
    (*vtxSettingsConfigMutable()).channel = trampCmsChan;
    (*vtxSettingsConfigMutable()).power = trampCmsPower;
    (*vtxSettingsConfigMutable()).freq =
        vtx58_Bandchan2Freq(trampCmsBand, trampCmsChan);
    saveConfigAndNotify();
    return -(1 as libc::c_int) as libc::c_long;
}
unsafe extern "C" fn trampCmsInitSettings() {
    if trampBand as libc::c_int > 0 as libc::c_int {
        trampCmsBand = trampBand
    }
    if trampChannel as libc::c_int > 0 as libc::c_int {
        trampCmsChan = trampChannel
    }
    trampCmsUpdateFreqRef();
    trampCmsPitMode =
        (trampPitMode as libc::c_int + 1 as libc::c_int) as uint8_t;
    if trampConfiguredPower as libc::c_int > 0 as libc::c_int {
        let mut i: uint8_t = 0 as libc::c_int as uint8_t;
        while (i as libc::c_int) < 5 as libc::c_int {
            if trampConfiguredPower as libc::c_int <=
                   trampPowerTable[i as usize] as libc::c_int {
                trampCmsPower =
                    (i as libc::c_int + 1 as libc::c_int) as uint8_t;
                break ;
            } else { i = i.wrapping_add(1) }
        }
    };
}
unsafe extern "C" fn trampCmsOnEnter() -> libc::c_long {
    trampCmsInitSettings();
    return 0 as libc::c_int as libc::c_long;
}
static mut trampCmsMenuCommenceEntries: [OSD_Entry; 4] =
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
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"YES\x00" as *const u8 as *const libc::c_char,
                           type_0: OME_Funcall,
                           func:
                               Some(trampCmsCommence as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
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
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text: 0 as *const libc::c_char,
                           type_0: OME_END,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         }]
    };
static mut trampCmsMenuCommence: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter: None,
                         onExit: None,
                         entries:
                             trampCmsMenuCommenceEntries.as_ptr() as *mut _,};
            init
        }
    };
static mut trampMenuEntries: [OSD_Entry; 11] =
    unsafe {
        [{
             let mut init =
                 OSD_Entry{text:
                               b"- TRAMP -\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Label,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text: b"\x00" as *const u8 as *const libc::c_char,
                           type_0: OME_Label,
                           func: None,
                           data:
                               trampCmsStatusString.as_ptr() as *mut _ as
                                   *mut libc::c_void,
                           flags: 0x4 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"PIT\x00" as *const u8 as *const libc::c_char,
                           type_0: OME_TAB,
                           func:
                               Some(trampCmsSetPitMode as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data:
                               &trampCmsEntPitMode as *const OSD_TAB_t as
                                   *mut OSD_TAB_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"BAND\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_TAB,
                           func:
                               Some(trampCmsConfigBand as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data:
                               &trampCmsEntBand as *const OSD_TAB_t as
                                   *mut OSD_TAB_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"CHAN\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_TAB,
                           func:
                               Some(trampCmsConfigChan as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data:
                               &trampCmsEntChan as *const OSD_TAB_t as
                                   *mut OSD_TAB_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
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
                               &trampCmsEntFreqRef as *const OSD_UINT16_t as
                                   *mut OSD_UINT16_t as *mut libc::c_void,
                           flags: 0x4 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"POWER\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_TAB,
                           func:
                               Some(trampCmsConfigPower as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data:
                               &trampCmsEntPower as *const OSD_TAB_t as
                                   *mut OSD_TAB_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"T(C)\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_INT16,
                           func: None,
                           data:
                               &trampCmsEntTemp as *const OSD_INT16_t as
                                   *mut OSD_INT16_t as *mut libc::c_void,
                           flags: 0x4 as libc::c_int as uint8_t,};
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
                               &trampCmsMenuCommence as *const CMS_Menu as
                                   *mut CMS_Menu as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
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
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text: 0 as *const libc::c_char,
                           type_0: OME_END,
                           func: None,
                           data:
                               0 as *const libc::c_void as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         }]
    };
#[no_mangle]
pub static mut cmsx_menuVtxTramp: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter:
                             Some(trampCmsOnEnter as
                                      unsafe extern "C" fn() -> libc::c_long),
                         onExit: None,
                         entries: trampMenuEntries.as_ptr() as *mut _,};
            init
        }
    };
