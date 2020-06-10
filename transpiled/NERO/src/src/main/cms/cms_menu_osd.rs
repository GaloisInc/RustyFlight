use ::libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn cmsMenuChange(pPort: *mut displayPort_t, ptr: *const libc::c_void)
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
    static osdTimerSourceNames: [*const libc::c_char; 3];
    #[no_mangle]
    static mut osdConfig_System: osdConfig_t;
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
pub type osdConfig_t = osdConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct osdConfig_s {
    pub item_pos: [uint16_t; 43],
    pub cap_alarm: uint16_t,
    pub alt_alarm: uint16_t,
    pub rssi_alarm: uint8_t,
    pub units: osd_unit_e,
    pub timers: [uint16_t; 2],
    pub enabledWarnings: uint16_t,
    pub ahMaxPitch: uint8_t,
    pub ahMaxRoll: uint8_t,
    pub enabled_stats: uint32_t,
    pub esc_temp_alarm: int8_t,
    pub esc_rpm_alarm: int16_t,
    pub esc_current_alarm: int16_t,
    pub core_temp_alarm: uint8_t,
}
pub type osd_unit_e = libc::c_uint;
pub const OSD_UNIT_METRIC: osd_unit_e = 1;
pub const OSD_UNIT_IMPERIAL: osd_unit_e = 0;
pub const OSD_TIMER_2: C2RustUnnamed_0 = 1;
pub const OSD_TIMER_PREC_COUNT: osd_timer_precision_e = 2;
pub type osd_timer_precision_e = libc::c_uint;
pub const OSD_TIMER_PREC_HUNDREDTHS: osd_timer_precision_e = 1;
pub const OSD_TIMER_PREC_SECOND: osd_timer_precision_e = 0;
pub const OSD_TIMER_SRC_COUNT: osd_timer_source_e = 3;
pub type osd_timer_source_e = libc::c_uint;
pub const OSD_TIMER_SRC_LAST_ARMED: osd_timer_source_e = 2;
pub const OSD_TIMER_SRC_TOTAL_ARMED: osd_timer_source_e = 1;
pub const OSD_TIMER_SRC_ON: osd_timer_source_e = 0;
pub const OSD_TIMER_1: C2RustUnnamed_0 = 0;
pub const OSD_TIMER_COUNT: C2RustUnnamed_0 = 2;
pub const OSD_G_FORCE: C2RustUnnamed = 42;
pub const OSD_NUMERICAL_VARIO: C2RustUnnamed = 33;
pub const OSD_NUMERICAL_HEADING: C2RustUnnamed = 32;
pub const OSD_ROLL_ANGLE: C2RustUnnamed = 27;
pub const OSD_PITCH_ANGLE: C2RustUnnamed = 26;
pub const OSD_DISARMED: C2RustUnnamed = 29;
pub const OSD_WARNINGS: C2RustUnnamed = 21;
pub const OSD_DEBUG: C2RustUnnamed = 25;
pub const OSD_PIDRATE_PROFILE: C2RustUnnamed = 20;
pub const OSD_YAW_PIDS: C2RustUnnamed = 18;
pub const OSD_PITCH_PIDS: C2RustUnnamed = 17;
pub const OSD_ROLL_PIDS: C2RustUnnamed = 16;
pub const OSD_POWER: C2RustUnnamed = 19;
pub const OSD_ALTITUDE: C2RustUnnamed = 15;
pub const OSD_COMPASS_BAR: C2RustUnnamed = 34;
pub const OSD_HOME_DIST: C2RustUnnamed = 31;
pub const OSD_HOME_DIR: C2RustUnnamed = 30;
pub const OSD_GPS_LON: C2RustUnnamed = 23;
pub const OSD_GPS_LAT: C2RustUnnamed = 24;
pub const OSD_GPS_SATS: C2RustUnnamed = 14;
pub const OSD_GPS_SPEED: C2RustUnnamed = 13;
pub const OSD_MAH_DRAWN: C2RustUnnamed = 12;
pub const OSD_CURRENT_DRAW: C2RustUnnamed = 11;
pub const OSD_VTX_CHANNEL: C2RustUnnamed = 10;
pub const OSD_THROTTLE_POS: C2RustUnnamed = 9;
pub const OSD_CRAFT_NAME: C2RustUnnamed = 8;
pub const OSD_FLYMODE: C2RustUnnamed = 7;
pub const OSD_REMAINING_TIME_ESTIMATE: C2RustUnnamed = 37;
pub const OSD_ITEM_TIMER_2: C2RustUnnamed = 6;
pub const OSD_ITEM_TIMER_1: C2RustUnnamed = 5;
pub const OSD_HORIZON_SIDEBARS: C2RustUnnamed = 4;
pub const OSD_ARTIFICIAL_HORIZON: C2RustUnnamed = 3;
pub const OSD_CROSSHAIRS: C2RustUnnamed = 2;
pub const OSD_AVG_CELL_VOLTAGE: C2RustUnnamed = 22;
pub const OSD_MAIN_BATT_USAGE: C2RustUnnamed = 28;
pub const OSD_MAIN_BATT_VOLTAGE: C2RustUnnamed = 1;
pub const OSD_RSSI_VALUE: C2RustUnnamed = 0;
pub const OSD_ITEM_COUNT: C2RustUnnamed = 43;
pub type C2RustUnnamed = libc::c_uint;
pub const OSD_ANTI_GRAVITY: C2RustUnnamed = 41;
pub const OSD_CORE_TEMPERATURE: C2RustUnnamed = 40;
pub const OSD_ADJUSTMENT_RANGE: C2RustUnnamed = 39;
pub const OSD_RTC_DATETIME: C2RustUnnamed = 38;
pub const OSD_ESC_RPM: C2RustUnnamed = 36;
pub const OSD_ESC_TMP: C2RustUnnamed = 35;
pub type C2RustUnnamed_0 = libc::c_uint;
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
#[inline]
unsafe extern "C" fn osdConfigMutable() -> *mut osdConfig_t {
    return &mut osdConfig_System;
}
#[inline]
unsafe extern "C" fn osdConfig() -> *const osdConfig_t {
    return &mut osdConfig_System;
}
// Alarms
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
static mut osdConfig_item_pos: [uint16_t; 43] = [0; 43];
unsafe extern "C" fn menuOsdActiveElemsOnEnter() -> libc::c_long {
    memcpy(&mut *osdConfig_item_pos.as_mut_ptr().offset(0 as libc::c_int as
                                                            isize) as
               *mut uint16_t as *mut libc::c_void,
           &*(*(osdConfig as
                    unsafe extern "C" fn()
                        ->
                            *const osdConfig_t)()).item_pos.as_ptr().offset(0
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
               as *const uint16_t as *const libc::c_void,
           (::core::mem::size_of::<uint16_t>() as
                libc::c_ulong).wrapping_mul(OSD_ITEM_COUNT as libc::c_int as
                                                libc::c_ulong));
    return 0 as libc::c_int as libc::c_long;
}
unsafe extern "C" fn menuOsdActiveElemsOnExit(mut self_0: *const OSD_Entry)
 -> libc::c_long {
    memcpy(&mut *(*(osdConfigMutable as
                        unsafe extern "C" fn()
                            ->
                                *mut osdConfig_t)()).item_pos.as_mut_ptr().offset(0
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      isize)
               as *mut uint16_t as *mut libc::c_void,
           &mut *osdConfig_item_pos.as_mut_ptr().offset(0 as libc::c_int as
                                                            isize) as
               *mut uint16_t as *const libc::c_void,
           (::core::mem::size_of::<uint16_t>() as
                libc::c_ulong).wrapping_mul(OSD_ITEM_COUNT as libc::c_int as
                                                libc::c_ulong));
    return 0 as libc::c_int as libc::c_long;
}
// Initialized in run_static_initializers
#[no_mangle]
pub static mut menuOsdActiveElemsEntries: [OSD_Entry; 40] =
    [OSD_Entry{text: 0 as *const libc::c_char,
               type_0: OME_Label,
               func: None,
               data: 0 as *mut libc::c_void,
               flags: 0,}; 40];
#[no_mangle]
pub static mut menuOsdActiveElems: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter:
                             Some(menuOsdActiveElemsOnEnter as
                                      unsafe extern "C" fn() -> libc::c_long),
                         onExit:
                             Some(menuOsdActiveElemsOnExit as
                                      unsafe extern "C" fn(_:
                                                               *const OSD_Entry)
                                          -> libc::c_long),
                         entries:
                             menuOsdActiveElemsEntries.as_ptr() as *mut _,};
            init
        }
    };
static mut osdConfig_rssi_alarm: uint8_t = 0;
static mut osdConfig_cap_alarm: uint16_t = 0;
static mut osdConfig_alt_alarm: uint16_t = 0;
unsafe extern "C" fn menuAlarmsOnEnter() -> libc::c_long {
    osdConfig_rssi_alarm = (*osdConfig()).rssi_alarm;
    osdConfig_cap_alarm = (*osdConfig()).cap_alarm;
    osdConfig_alt_alarm = (*osdConfig()).alt_alarm;
    return 0 as libc::c_int as libc::c_long;
}
unsafe extern "C" fn menuAlarmsOnExit(mut self_0: *const OSD_Entry)
 -> libc::c_long {
    (*osdConfigMutable()).rssi_alarm = osdConfig_rssi_alarm;
    (*osdConfigMutable()).cap_alarm = osdConfig_cap_alarm;
    (*osdConfigMutable()).alt_alarm = osdConfig_alt_alarm;
    return 0 as libc::c_int as libc::c_long;
}
#[no_mangle]
pub static mut menuAlarms: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter:
                             Some(menuAlarmsOnEnter as
                                      unsafe extern "C" fn() -> libc::c_long),
                         onExit:
                             Some(menuAlarmsOnExit as
                                      unsafe extern "C" fn(_:
                                                               *const OSD_Entry)
                                          -> libc::c_long),
                         entries: menuAlarmsEntries.as_ptr() as *mut _,};
            init
        }
    };
#[no_mangle]
pub static mut timerSource: [osd_timer_source_e; 2] = [OSD_TIMER_SRC_ON; 2];
#[no_mangle]
pub static mut timerPrecision: [osd_timer_precision_e; 2] =
    [OSD_TIMER_PREC_SECOND; 2];
#[no_mangle]
pub static mut timerAlarm: [uint8_t; 2] = [0; 2];
unsafe extern "C" fn menuTimersOnEnter() -> libc::c_long {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < OSD_TIMER_COUNT as libc::c_int {
        let timer: uint16_t = (*osdConfig()).timers[i as usize];
        timerSource[i as usize] =
            (timer as libc::c_int & 0xf as libc::c_int) as osd_timer_source_e;
        timerPrecision[i as usize] =
            (timer as libc::c_int >> 4 as libc::c_int & 0xf as libc::c_int) as
                osd_timer_precision_e;
        timerAlarm[i as usize] =
            (timer as libc::c_int >> 8 as libc::c_int & 0xff as libc::c_int)
                as uint8_t;
        i += 1
    }
    return 0 as libc::c_int as libc::c_long;
}
unsafe extern "C" fn menuTimersOnExit(mut self_0: *const OSD_Entry)
 -> libc::c_long {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < OSD_TIMER_COUNT as libc::c_int {
        (*osdConfigMutable()).timers[i as usize] =
            (timerSource[i as usize] as libc::c_uint &
                 0xf as libc::c_int as libc::c_uint |
                 (timerPrecision[i as usize] as libc::c_uint &
                      0xf as libc::c_int as libc::c_uint) << 4 as libc::c_int
                 |
                 ((timerAlarm[i as usize] as libc::c_int &
                       0xff as libc::c_int) << 8 as libc::c_int) as
                     libc::c_uint) as uint16_t;
        i += 1
    }
    return 0 as libc::c_int as libc::c_long;
}
static mut osdTimerPrecisionNames: [*const libc::c_char; 2] =
    [b"SCND\x00" as *const u8 as *const libc::c_char,
     b"HDTH\x00" as *const u8 as *const libc::c_char];
#[no_mangle]
pub static mut menuTimers: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter:
                             Some(menuTimersOnEnter as
                                      unsafe extern "C" fn() -> libc::c_long),
                         onExit:
                             Some(menuTimersOnExit as
                                      unsafe extern "C" fn(_:
                                                               *const OSD_Entry)
                                          -> libc::c_long),
                         entries: menuTimersEntries.as_ptr() as *mut _,};
            init
        }
    };
/* USE_EXTENDED_CMS_MENUS */
unsafe extern "C" fn cmsx_menuOsdOnEnter() -> libc::c_long {
    return 0 as libc::c_int as libc::c_long;
}
unsafe extern "C" fn cmsx_menuOsdOnExit(mut self_0: *const OSD_Entry)
 -> libc::c_long {
    return 0 as libc::c_int as libc::c_long;
}
#[no_mangle]
pub static mut cmsx_menuOsdEntries: [OSD_Entry; 6] =
    unsafe {
        [{
             let mut init =
                 OSD_Entry{text:
                               b"---OSD---\x00" as *const u8 as
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
                               b"ACTIVE ELEM\x00" as *const u8 as
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
                               &menuOsdActiveElems as *const CMS_Menu as
                                   *mut CMS_Menu as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"TIMERS\x00" as *const u8 as
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
                               &menuTimers as *const CMS_Menu as *mut CMS_Menu
                                   as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"ALARMS\x00" as *const u8 as
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
                               &menuAlarms as *const CMS_Menu as *mut CMS_Menu
                                   as *mut libc::c_void,
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
pub static mut cmsx_menuOsd: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter:
                             Some(cmsx_menuOsdOnEnter as
                                      unsafe extern "C" fn() -> libc::c_long),
                         onExit:
                             Some(cmsx_menuOsdOnExit as
                                      unsafe extern "C" fn(_:
                                                               *const OSD_Entry)
                                          -> libc::c_long),
                         entries: cmsx_menuOsdEntries.as_ptr() as *mut _,};
            init
        }
    };
unsafe extern "C" fn run_static_initializers() {
    menuOsdActiveElemsEntries =
        [{
             let mut init =
                 OSD_Entry{text:
                               b"--- ACTIV ELEM ---\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Label,
                           func: None,
                           data: 0 as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"RSSI\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_RSSI_VALUE
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"BATTERY VOLTAGE\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_MAIN_BATT_VOLTAGE
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"BATTERY USAGE\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_MAIN_BATT_USAGE
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"AVG CELL VOLTAGE\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_AVG_CELL_VOLTAGE
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"CROSSHAIRS\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_CROSSHAIRS
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"HORIZON\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_ARTIFICIAL_HORIZON
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"HORIZON SIDEBARS\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_HORIZON_SIDEBARS
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"TIMER 1\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_ITEM_TIMER_1
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"TIMER 2\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_ITEM_TIMER_2
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"REMAINING TIME ESTIMATE\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_REMAINING_TIME_ESTIMATE
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"FLY MODE\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_FLYMODE
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"NAME\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_CRAFT_NAME
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"THROTTLE\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_THROTTLE_POS
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"VTX CHAN\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_VTX_CHANNEL
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"CURRENT (A)\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_CURRENT_DRAW
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"USED MAH\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_MAH_DRAWN
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"GPS SPEED\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_GPS_SPEED
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"GPS SATS\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_GPS_SATS
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"GPS LAT\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_GPS_LAT
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"GPS LON\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_GPS_LON
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"HOME DIR\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_HOME_DIR
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"HOME DIST\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_HOME_DIST
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"COMPASS BAR\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_COMPASS_BAR
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"ALTITUDE\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_ALTITUDE
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"POWER\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_POWER
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"ROLL PID\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_ROLL_PIDS
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"PITCH PID\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_PITCH_PIDS
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"YAW PID\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_YAW_PIDS
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"PROFILES\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_PIDRATE_PROFILE
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"DEBUG\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_DEBUG
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"WARNINGS\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_WARNINGS
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"DISARMED\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_DISARMED
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"PIT ANG\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_PITCH_ANGLE
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"ROL ANG\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_ROLL_ANGLE
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"HEADING\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_NUMERICAL_HEADING
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"VARIO\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_NUMERICAL_VARIO
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"G-FORCE\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_VISIBLE,
                           func: None,
                           data:
                               &mut *osdConfig_item_pos.as_mut_ptr().offset(OSD_G_FORCE
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                   as *mut uint16_t as *mut libc::c_void,
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
                           data: 0 as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text: 0 as *const libc::c_char,
                           type_0: OME_END,
                           func: None,
                           data: 0 as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         }]
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
// CMS
