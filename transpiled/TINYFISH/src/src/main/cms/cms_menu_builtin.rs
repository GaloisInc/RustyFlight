use ::libc;
extern "C" {
    #[no_mangle]
    static shortGitRevision: *const libc::c_char;
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
    // Device management
    // For main.c and scheduler
    #[no_mangle]
    fn cmsMenuChange(pPort: *mut displayPort_t, ptr: *const libc::c_void)
     -> libc::c_long;
    #[no_mangle]
    fn cmsMenuExit(pPort: *mut displayPort_t, ptr: *const libc::c_void)
     -> libc::c_long;
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
    static mut cmsx_menuImu: CMS_Menu;
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
    static mut cmsx_menuBlackbox: CMS_Menu;
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
    static mut cmsx_menuLedstrip: CMS_Menu;
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
    static mut cmsx_menuMisc: CMS_Menu;
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
    static mut cmsx_menuPower: CMS_Menu;
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
    static mut cmsx_menuVtxSmartAudio: CMS_Menu;
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
    static mut cmsx_menuVtxTramp: CMS_Menu;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
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
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
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
//
// Built-in menu contents and support functions
//
// Sub menus
// VTX supplied menus
// Info
static mut infoGitRev: [libc::c_char; 8] = [0; 8];
static mut infoTargetName: [libc::c_char; 9] =
    [84, 73, 78, 89, 70, 73, 83, 72, 0];
unsafe extern "C" fn cmsx_InfoInit() -> libc::c_long {
    let mut i: libc::c_int = 0; // Terminate string
    i = 0 as libc::c_int;
    while i < 7 as libc::c_int {
        if *shortGitRevision.offset(i as isize) as libc::c_int >= 'a' as i32
               &&
               *shortGitRevision.offset(i as isize) as libc::c_int <=
                   'f' as i32 {
            infoGitRev[i as usize] =
                (*shortGitRevision.offset(i as isize) as libc::c_int -
                     'a' as i32 + 'A' as i32) as libc::c_char
        } else {
            infoGitRev[i as usize] = *shortGitRevision.offset(i as isize)
        }
        i += 1
    }
    infoGitRev[i as usize] = 0 as libc::c_int as libc::c_char;
    return 0 as libc::c_int as libc::c_long;
}
static mut menuInfoEntries: [OSD_Entry; 7] =
    unsafe {
        [{
             let mut init =
                 OSD_Entry{text:
                               b"--- INFO ---\x00" as *const u8 as
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
                               b"FWID\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_String,
                           func: None,
                           data:
                               b"CLFL\x00" as *const u8 as *const libc::c_char
                                   as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"FWVER\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_String,
                           func: None,
                           data:
                               b"2.5.0\x00" as *const u8 as
                                   *const libc::c_char as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"GITREV\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_String,
                           func: None,
                           data:
                               infoGitRev.as_ptr() as *mut _ as
                                   *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"TARGET\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_String,
                           func: None,
                           data:
                               infoTargetName.as_ptr() as *mut _ as
                                   *mut libc::c_void,
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
static mut menuInfo: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter:
                             Some(cmsx_InfoInit as
                                      unsafe extern "C" fn() -> libc::c_long),
                         onExit: None,
                         entries: menuInfoEntries.as_ptr() as *mut _,};
            init
        }
    };
// Features
static mut menuFeaturesEntries: [OSD_Entry; 8] =
    unsafe {
        [{
             let mut init =
                 OSD_Entry{text:
                               b"--- FEATURES ---\x00" as *const u8 as
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
                               b"BLACKBOX\x00" as *const u8 as
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
                               &cmsx_menuBlackbox as *const CMS_Menu as
                                   *mut CMS_Menu as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"VTX SA\x00" as *const u8 as
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
                               &cmsx_menuVtxSmartAudio as *const CMS_Menu as
                                   *mut CMS_Menu as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"VTX TR\x00" as *const u8 as
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
                               &cmsx_menuVtxTramp as *const CMS_Menu as
                                   *mut CMS_Menu as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"LED STRIP\x00" as *const u8 as
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
                               &cmsx_menuLedstrip as *const CMS_Menu as
                                   *mut CMS_Menu as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"POWER\x00" as *const u8 as
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
                               &cmsx_menuPower as *const CMS_Menu as
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
static mut menuFeatures: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter: None,
                         onExit: None,
                         entries: menuFeaturesEntries.as_ptr() as *mut _,};
            init
        }
    };
// Main
static mut menuMainEntries: [OSD_Entry; 9] =
    unsafe {
        [{
             let mut init =
                 OSD_Entry{text:
                               b"-- MAIN --\x00" as *const u8 as
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
                               b"PROFILE\x00" as *const u8 as
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
                               &cmsx_menuImu as *const CMS_Menu as
                                   *mut CMS_Menu as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"FEATURES\x00" as *const u8 as
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
                               &menuFeatures as *const CMS_Menu as
                                   *mut CMS_Menu as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"FC&FW INFO\x00" as *const u8 as
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
                               &menuInfo as *const CMS_Menu as *mut CMS_Menu
                                   as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"MISC\x00" as *const u8 as
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
                               &cmsx_menuMisc as *const CMS_Menu as
                                   *mut CMS_Menu as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"EXIT\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_OSD_Exit,
                           func:
                               Some(cmsMenuExit as
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
                               b"SAVE&EXIT\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_OSD_Exit,
                           func:
                               Some(cmsMenuExit as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data: 1 as libc::c_int as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"SAVE&REBOOT\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_OSD_Exit,
                           func:
                               Some(cmsMenuExit as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data: 2 as libc::c_int as *mut libc::c_void,
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
pub static mut menuMain: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter: None,
                         onExit: None,
                         entries: menuMainEntries.as_ptr() as *mut _,};
            init
        }
    };
