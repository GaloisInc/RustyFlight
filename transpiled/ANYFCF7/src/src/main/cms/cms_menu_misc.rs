use ::libc;
extern "C" {
    #[no_mangle]
    static debugModeNames: [*const libc::c_char; 44];
    #[no_mangle]
    fn cmsMenuChange(pPort: *mut displayPort_t, ptr: *const libc::c_void)
     -> libc::c_long;
    #[no_mangle]
    static mut systemConfig_System: systemConfig_t;
    #[no_mangle]
    static mut motorConfig_System: motorConfig_t;
    #[no_mangle]
    static mut rcData: [int16_t; 18];
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
pub type debugType_e = libc::c_uint;
pub const DEBUG_COUNT: debugType_e = 44;
pub const DEBUG_ANTI_GRAVITY: debugType_e = 43;
pub const DEBUG_RC_SMOOTHING_RATE: debugType_e = 42;
pub const DEBUG_RX_SIGNAL_LOSS: debugType_e = 41;
pub const DEBUG_RC_SMOOTHING: debugType_e = 40;
pub const DEBUG_ACRO_TRAINER: debugType_e = 39;
pub const DEBUG_ITERM_RELAX: debugType_e = 38;
pub const DEBUG_RTH: debugType_e = 37;
pub const DEBUG_SMARTAUDIO: debugType_e = 36;
pub const DEBUG_USB: debugType_e = 35;
pub const DEBUG_CURRENT: debugType_e = 34;
pub const DEBUG_SDIO: debugType_e = 33;
pub const DEBUG_RUNAWAY_TAKEOFF: debugType_e = 32;
pub const DEBUG_CORE_TEMP: debugType_e = 31;
pub const DEBUG_LIDAR_TF: debugType_e = 30;
pub const DEBUG_RANGEFINDER_QUALITY: debugType_e = 29;
pub const DEBUG_RANGEFINDER: debugType_e = 28;
pub const DEBUG_FPORT: debugType_e = 27;
pub const DEBUG_SBUS: debugType_e = 26;
pub const DEBUG_MAX7456_SPICLOCK: debugType_e = 25;
pub const DEBUG_MAX7456_SIGNAL: debugType_e = 24;
pub const DEBUG_DUAL_GYRO_DIFF: debugType_e = 23;
pub const DEBUG_DUAL_GYRO_COMBINE: debugType_e = 22;
pub const DEBUG_DUAL_GYRO_RAW: debugType_e = 21;
pub const DEBUG_DUAL_GYRO: debugType_e = 20;
pub const DEBUG_GYRO_RAW: debugType_e = 19;
pub const DEBUG_RX_FRSKY_SPI: debugType_e = 18;
pub const DEBUG_FFT_FREQ: debugType_e = 17;
pub const DEBUG_FFT_TIME: debugType_e = 16;
pub const DEBUG_FFT: debugType_e = 15;
pub const DEBUG_ALTITUDE: debugType_e = 14;
pub const DEBUG_ESC_SENSOR_TMP: debugType_e = 13;
pub const DEBUG_ESC_SENSOR_RPM: debugType_e = 12;
pub const DEBUG_STACK: debugType_e = 11;
pub const DEBUG_SCHEDULER: debugType_e = 10;
pub const DEBUG_ESC_SENSOR: debugType_e = 9;
pub const DEBUG_ANGLERATE: debugType_e = 8;
pub const DEBUG_RC_INTERPOLATION: debugType_e = 7;
pub const DEBUG_GYRO_SCALED: debugType_e = 6;
pub const DEBUG_PIDLOOP: debugType_e = 5;
pub const DEBUG_ACCELEROMETER: debugType_e = 4;
pub const DEBUG_GYRO_FILTERED: debugType_e = 3;
pub const DEBUG_BATTERY: debugType_e = 2;
pub const DEBUG_CYCLETIME: debugType_e = 1;
pub const DEBUG_NONE: debugType_e = 0;
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
// CMS state
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
pub type ioTag_t = uint8_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct systemConfig_s {
    pub pidProfileIndex: uint8_t,
    pub activeRateProfile: uint8_t,
    pub debug_mode: uint8_t,
    pub task_statistics: uint8_t,
    pub rateProfile6PosSwitch: uint8_t,
    pub cpu_overclock: uint8_t,
    pub powerOnArmingGraceTime: uint8_t,
    pub boardIdentifier: [libc::c_char; 6],
}
pub type systemConfig_t = systemConfig_s;
pub type rc_alias = libc::c_uint;
pub const AUX8: rc_alias = 11;
pub const AUX7: rc_alias = 10;
pub const AUX6: rc_alias = 9;
pub const AUX5: rc_alias = 8;
pub const AUX4: rc_alias = 7;
pub const AUX3: rc_alias = 6;
pub const AUX2: rc_alias = 5;
pub const AUX1: rc_alias = 4;
pub const THROTTLE: rc_alias = 3;
pub const YAW: rc_alias = 2;
pub const PITCH: rc_alias = 1;
pub const ROLL: rc_alias = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct motorDevConfig_s {
    pub motorPwmRate: uint16_t,
    pub motorPwmProtocol: uint8_t,
    pub motorPwmInversion: uint8_t,
    pub useUnsyncedPwm: uint8_t,
    pub useBurstDshot: uint8_t,
    pub ioTags: [ioTag_t; 8],
}
pub type motorDevConfig_t = motorDevConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct motorConfig_s {
    pub dev: motorDevConfig_t,
    pub digitalIdleOffsetValue: uint16_t,
    pub minthrottle: uint16_t,
    pub maxthrottle: uint16_t,
    pub mincommand: uint16_t,
    pub motorPoleCount: uint8_t,
}
pub type motorConfig_t = motorConfig_s;
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
#[inline]
unsafe extern "C" fn systemConfigMutable() -> *mut systemConfig_t {
    return &mut systemConfig_System;
}
#[inline]
unsafe extern "C" fn systemConfig() -> *const systemConfig_t {
    return &mut systemConfig_System;
}
#[inline]
unsafe extern "C" fn motorConfigMutable() -> *mut motorConfig_t {
    return &mut motorConfig_System;
}
#[inline]
unsafe extern "C" fn motorConfig() -> *const motorConfig_t {
    return &mut motorConfig_System;
}
// in seconds
// Idle value for DShot protocol, full motor output = 10000
// Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
// This is the maximum value for the ESCs at full power this value can be increased up to 2000
// This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs
// Magnetic poles in the motors for calculating actual RPM from eRPM provided by ESC telemetry
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
// Misc
//
unsafe extern "C" fn cmsx_menuRcConfirmBack(mut self_0: *const OSD_Entry)
 -> libc::c_long {
    if !self_0.is_null() &&
           (*self_0).type_0 as libc::c_uint ==
               OME_Back as libc::c_int as libc::c_uint {
        return 0 as libc::c_int as libc::c_long
    } else { return -(1 as libc::c_int) as libc::c_long };
}
#[no_mangle]
pub static mut cmsx_menuRcPreview: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter: None,
                         onExit:
                             Some(cmsx_menuRcConfirmBack as
                                      unsafe extern "C" fn(_:
                                                               *const OSD_Entry)
                                          -> libc::c_long),
                         entries: cmsx_menuRcEntries.as_ptr() as *mut _,};
            init
        }
    };
static mut motorConfig_minthrottle: uint16_t = 0;
static mut motorConfig_digitalIdleOffsetValue: uint8_t = 0;
static mut systemConfig_debug_mode: debugType_e = DEBUG_NONE;
unsafe extern "C" fn cmsx_menuMiscOnEnter() -> libc::c_long {
    motorConfig_minthrottle = (*motorConfig()).minthrottle;
    motorConfig_digitalIdleOffsetValue =
        ((*motorConfig()).digitalIdleOffsetValue as libc::c_int /
             10 as libc::c_int) as uint8_t;
    systemConfig_debug_mode = (*systemConfig()).debug_mode as debugType_e;
    return 0 as libc::c_int as libc::c_long;
}
unsafe extern "C" fn cmsx_menuMiscOnExit(mut self_0: *const OSD_Entry)
 -> libc::c_long {
    (*motorConfigMutable()).minthrottle = motorConfig_minthrottle;
    (*motorConfigMutable()).digitalIdleOffsetValue =
        (10 as libc::c_int *
             motorConfig_digitalIdleOffsetValue as libc::c_int) as uint16_t;
    (*systemConfigMutable()).debug_mode = systemConfig_debug_mode as uint8_t;
    return 0 as libc::c_int as libc::c_long;
}
#[no_mangle]
pub static mut cmsx_menuMisc: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter:
                             Some(cmsx_menuMiscOnEnter as
                                      unsafe extern "C" fn() -> libc::c_long),
                         onExit:
                             Some(cmsx_menuMiscOnExit as
                                      unsafe extern "C" fn(_:
                                                               *const OSD_Entry)
                                          -> libc::c_long),
                         entries: menuMiscEntries.as_ptr() as *mut _,};
            init
        }
    };
// CMS
