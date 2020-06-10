use ::libc;
extern "C" {
    #[no_mangle]
    fn saveConfigAndNotify();
    #[no_mangle]
    static vtx58BandNames: [*const libc::c_char; 0];
    #[no_mangle]
    fn vtx58_Bandchan2Freq(band: uint8_t, channel: uint8_t) -> uint16_t;
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
    static rtc6705PowerNames: [*const libc::c_char; 0];
    #[no_mangle]
    static mut vtxSettingsConfig_System: vtxSettingsConfig_t;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
pub const OME_MAX: OSD_MenuElement = 14;
pub const OME_MENU: OSD_MenuElement = 14;
pub const OME_END: OSD_MenuElement = 13;
pub const OME_TAB: OSD_MenuElement = 12;
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
// Bits in flags
// Value has been changed, need to redraw
// Text label should be printed
// Value should be updated dynamically
// (Temporary) Flag for OME_Submenu, indicating func should be called to get a string to display.
pub type CMSMenuFuncPtr = Option<unsafe extern "C" fn() -> libc::c_long>;
// Special return value(s) for function chaining by CMSMenuFuncPtr
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
pub struct OSD_UINT8_t {
    pub val: *mut uint8_t,
    pub min: uint8_t,
    pub max: uint8_t,
    pub step: uint8_t,
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
#[inline]
unsafe extern "C" fn vtxSettingsConfig() -> *const vtxSettingsConfig_t {
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
static mut cmsx_vtxBand: uint8_t = 0;
static mut cmsx_vtxChannel: uint8_t = 0;
static mut cmsx_vtxPower: uint8_t = 0;
// Initialized in run_static_initializers
static mut entryVtxBand: OSD_TAB_t =
    OSD_TAB_t{val: 0 as *const uint8_t as *mut uint8_t,
              max: 0,
              names: 0 as *const *const libc::c_char,};
static mut entryVtxChannel: OSD_UINT8_t =
    unsafe {
        {
            let mut init =
                OSD_UINT8_t{val:
                                &cmsx_vtxChannel as *const uint8_t as
                                    *mut uint8_t,
                            min: 1 as libc::c_int as uint8_t,
                            max:
                                (8 as libc::c_int - 1 as libc::c_int +
                                     1 as libc::c_int) as uint8_t,
                            step: 1 as libc::c_int as uint8_t,};
            init
        }
    };
// Initialized in run_static_initializers
static mut entryVtxPower: OSD_TAB_t =
    OSD_TAB_t{val: 0 as *const uint8_t as *mut uint8_t,
              max: 0,
              names: 0 as *const *const libc::c_char,};
unsafe extern "C" fn cmsx_Vtx_ConfigRead() {
    cmsx_vtxBand =
        ((*vtxSettingsConfig()).band as libc::c_int - 1 as libc::c_int) as
            uint8_t;
    cmsx_vtxChannel = (*vtxSettingsConfig()).channel;
    cmsx_vtxPower =
        ((*vtxSettingsConfig()).power as libc::c_int - 1 as libc::c_int) as
            uint8_t;
}
unsafe extern "C" fn cmsx_Vtx_ConfigWriteback() {
    // update vtx_ settings
    (*vtxSettingsConfigMutable()).band =
        (cmsx_vtxBand as libc::c_int + 1 as libc::c_int) as uint8_t;
    (*vtxSettingsConfigMutable()).channel = cmsx_vtxChannel;
    (*vtxSettingsConfigMutable()).power =
        (cmsx_vtxPower as libc::c_int + 1 as libc::c_int) as uint8_t;
    (*vtxSettingsConfigMutable()).freq =
        vtx58_Bandchan2Freq((cmsx_vtxBand as libc::c_int + 1 as libc::c_int)
                                as uint8_t, cmsx_vtxChannel);
    saveConfigAndNotify();
}
unsafe extern "C" fn cmsx_Vtx_onEnter() -> libc::c_long {
    cmsx_Vtx_ConfigRead();
    return 0 as libc::c_int as libc::c_long;
}
unsafe extern "C" fn cmsx_Vtx_onExit(mut self_0: *const OSD_Entry)
 -> libc::c_long {
    cmsx_Vtx_ConfigWriteback();
    return 0 as libc::c_int as libc::c_long;
}
static mut cmsx_menuVtxEntries: [OSD_Entry; 6] =
    unsafe {
        [{
             let mut init =
                 OSD_Entry{text:
                               b"--- VTX ---\x00" as *const u8 as
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
                               b"BAND\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_TAB,
                           func: None,
                           data:
                               &entryVtxBand as *const OSD_TAB_t as
                                   *mut OSD_TAB_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"CHANNEL\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT8,
                           func: None,
                           data:
                               &entryVtxChannel as *const OSD_UINT8_t as
                                   *mut OSD_UINT8_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"POWER\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_TAB,
                           func: None,
                           data:
                               &entryVtxPower as *const OSD_TAB_t as
                                   *mut OSD_TAB_t as *mut libc::c_void,
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
pub static mut cmsx_menuVtxRTC6705: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter:
                             Some(cmsx_Vtx_onEnter as
                                      unsafe extern "C" fn() -> libc::c_long),
                         onExit:
                             Some(cmsx_Vtx_onExit as
                                      unsafe extern "C" fn(_:
                                                               *const OSD_Entry)
                                          -> libc::c_long),
                         entries: cmsx_menuVtxEntries.as_ptr() as *mut _,};
            init
        }
    };
unsafe extern "C" fn run_static_initializers() {
    entryVtxBand =
        {
            let mut init =
                OSD_TAB_t{val: &mut cmsx_vtxBand,
                          max:
                              (5 as libc::c_int - 1 as libc::c_int) as
                                  uint8_t,
                          names:
                              &*vtx58BandNames.as_ptr().offset(1 as
                                                                   libc::c_int
                                                                   as isize)
                                  as *const *const libc::c_char,};
            init
        };
    entryVtxPower =
        {
            let mut init =
                OSD_TAB_t{val: &mut cmsx_vtxPower,
                          max:
                              (3 as libc::c_int - 1 as libc::c_int -
                                   1 as libc::c_int) as uint8_t,
                          names:
                              &*rtc6705PowerNames.as_ptr().offset(1 as
                                                                      libc::c_int
                                                                      as
                                                                      isize)
                                  as *const *const libc::c_char,};
            init
        }
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
// CMS
