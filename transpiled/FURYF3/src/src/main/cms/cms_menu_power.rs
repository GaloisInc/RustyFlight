use ::libc;
extern "C" {
    #[no_mangle]
    static currentMeterSourceNames: [*const libc::c_char; 5];
    #[no_mangle]
    static mut currentSensorADCConfig_System: currentSensorADCConfig_t;
    #[no_mangle]
    static mut currentSensorVirtualConfig_System:
           currentSensorVirtualConfig_t;
    #[no_mangle]
    static voltageMeterSourceNames: [*const libc::c_char; 3];
    #[no_mangle]
    static mut batteryConfig_System: batteryConfig_t;
    #[no_mangle]
    static mut voltageSensorADCConfig_SystemArray:
           [voltageSensorADCConfig_t; 1];
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
pub struct OSD_INT16_t {
    pub val: *mut int16_t,
    pub min: int16_t,
    pub max: int16_t,
    pub step: int16_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct OSD_TAB_t {
    pub val: *mut uint8_t,
    pub max: uint8_t,
    pub names: *const *const libc::c_char,
}
pub const CURRENT_METER_COUNT: currentMeterSource_e = 5;
pub type currentMeterSource_e = libc::c_uint;
pub const CURRENT_METER_MSP: currentMeterSource_e = 4;
pub const CURRENT_METER_ESC: currentMeterSource_e = 3;
pub const CURRENT_METER_VIRTUAL: currentMeterSource_e = 2;
pub const CURRENT_METER_ADC: currentMeterSource_e = 1;
pub const CURRENT_METER_NONE: currentMeterSource_e = 0;
pub const VOLTAGE_METER_COUNT: voltageMeterSource_e = 3;
pub type voltageMeterSource_e = libc::c_uint;
pub const VOLTAGE_METER_ESC: voltageMeterSource_e = 2;
pub const VOLTAGE_METER_ADC: voltageMeterSource_e = 1;
pub const VOLTAGE_METER_NONE: voltageMeterSource_e = 0;
pub type currentSensorVirtualConfig_t = currentSensorVirtualConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct currentSensorVirtualConfig_s {
    pub scale: int16_t,
    pub offset: uint16_t,
}
pub type currentSensorADCConfig_t = currentSensorADCConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct currentSensorADCConfig_s {
    pub scale: int16_t,
    pub offset: int16_t,
}
pub type voltageSensorADCConfig_t = voltageSensorADCConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct voltageSensorADCConfig_s {
    pub vbatscale: uint8_t,
    pub vbatresdivval: uint8_t,
    pub vbatresdivmultiplier: uint8_t,
}
pub type batteryConfig_t = batteryConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct batteryConfig_s {
    pub vbatmaxcellvoltage: uint8_t,
    pub vbatmincellvoltage: uint8_t,
    pub vbatwarningcellvoltage: uint8_t,
    pub vbatnotpresentcellvoltage: uint8_t,
    pub lvcPercentage: uint8_t,
    pub voltageMeterSource: voltageMeterSource_e,
    pub currentMeterSource: currentMeterSource_e,
    pub batteryCapacity: uint16_t,
    pub useVBatAlerts: bool,
    pub useConsumptionAlerts: bool,
    pub consumptionWarningPercentage: uint8_t,
    pub vbathysteresis: uint8_t,
    pub vbatfullcellvoltage: uint8_t,
}
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
#[inline]
unsafe extern "C" fn currentSensorADCConfig()
 -> *const currentSensorADCConfig_t {
    return &mut currentSensorADCConfig_System;
}
#[inline]
unsafe extern "C" fn currentSensorADCConfigMutable()
 -> *mut currentSensorADCConfig_t {
    return &mut currentSensorADCConfig_System;
}
#[inline]
unsafe extern "C" fn currentSensorVirtualConfig()
 -> *const currentSensorVirtualConfig_t {
    return &mut currentSensorVirtualConfig_System;
}
#[inline]
unsafe extern "C" fn currentSensorVirtualConfigMutable()
 -> *mut currentSensorVirtualConfig_t {
    return &mut currentSensorVirtualConfig_System;
}
#[inline]
unsafe extern "C" fn batteryConfig() -> *const batteryConfig_t {
    return &mut batteryConfig_System;
}
#[inline]
unsafe extern "C" fn batteryConfigMutable() -> *mut batteryConfig_t {
    return &mut batteryConfig_System;
}
// scale the current sensor output voltage to milliamps. Value in 1/10th mV/A
// offset of the current sensor in millivolt steps
// scale the current sensor output voltage to milliamps. Value in mV/10A
// offset of the current sensor in mA
// VBAT - some boards have external, 12V, 9V and 5V meters.
// see also voltageMeterADCtoIDMap
// adjust this to match battery voltage to reported value
// resistor divider R2 (default NAZE 10(K))
// multiplier for scale (e.g. 2.5:1 ratio with multiplier of 4 can use '100' instead of '25' in ratio) to get better precision
#[inline]
unsafe extern "C" fn voltageSensorADCConfigMutable(mut _index: libc::c_int)
 -> *mut voltageSensorADCConfig_t {
    return &mut *voltageSensorADCConfig_SystemArray.as_mut_ptr().offset(_index
                                                                            as
                                                                            isize)
               as *mut voltageSensorADCConfig_t;
}
#[inline]
unsafe extern "C" fn voltageSensorADCConfig(mut _index: libc::c_int)
 -> *const voltageSensorADCConfig_t {
    return &mut *voltageSensorADCConfig_SystemArray.as_mut_ptr().offset(_index
                                                                            as
                                                                            isize)
               as *mut voltageSensorADCConfig_t;
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
pub static mut batteryConfig_voltageMeterSource: voltageMeterSource_e =
    VOLTAGE_METER_NONE;
#[no_mangle]
pub static mut batteryConfig_currentMeterSource: currentMeterSource_e =
    CURRENT_METER_NONE;
#[no_mangle]
pub static mut batteryConfig_vbatmaxcellvoltage: uint8_t = 0;
#[no_mangle]
pub static mut voltageSensorADCConfig_vbatscale: uint8_t = 0;
#[no_mangle]
pub static mut currentSensorADCConfig_scale: int16_t = 0;
#[no_mangle]
pub static mut currentSensorADCConfig_offset: int16_t = 0;
#[no_mangle]
pub static mut currentSensorVirtualConfig_scale: int16_t = 0;
#[no_mangle]
pub static mut currentSensorVirtualConfig_offset: int16_t = 0;
unsafe extern "C" fn cmsx_Power_onEnter() -> libc::c_long {
    batteryConfig_voltageMeterSource = (*batteryConfig()).voltageMeterSource;
    batteryConfig_currentMeterSource = (*batteryConfig()).currentMeterSource;
    batteryConfig_vbatmaxcellvoltage = (*batteryConfig()).vbatmaxcellvoltage;
    voltageSensorADCConfig_vbatscale =
        (*voltageSensorADCConfig(0 as libc::c_int)).vbatscale;
    currentSensorADCConfig_scale = (*currentSensorADCConfig()).scale;
    currentSensorADCConfig_offset = (*currentSensorADCConfig()).offset;
    currentSensorVirtualConfig_scale = (*currentSensorVirtualConfig()).scale;
    currentSensorVirtualConfig_offset =
        (*currentSensorVirtualConfig()).offset as int16_t;
    return 0 as libc::c_int as libc::c_long;
}
unsafe extern "C" fn cmsx_Power_onExit(mut self_0: *const OSD_Entry)
 -> libc::c_long {
    (*batteryConfigMutable()).voltageMeterSource =
        batteryConfig_voltageMeterSource;
    (*batteryConfigMutable()).currentMeterSource =
        batteryConfig_currentMeterSource;
    (*batteryConfigMutable()).vbatmaxcellvoltage =
        batteryConfig_vbatmaxcellvoltage;
    (*voltageSensorADCConfigMutable(0 as libc::c_int)).vbatscale =
        voltageSensorADCConfig_vbatscale;
    (*currentSensorADCConfigMutable()).scale = currentSensorADCConfig_scale;
    (*currentSensorADCConfigMutable()).offset = currentSensorADCConfig_offset;
    (*currentSensorVirtualConfigMutable()).scale =
        currentSensorVirtualConfig_scale;
    (*currentSensorVirtualConfigMutable()).offset =
        currentSensorVirtualConfig_offset as uint16_t;
    return 0 as libc::c_int as libc::c_long;
}
#[no_mangle]
pub static mut cmsx_menuPower: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter:
                             Some(cmsx_Power_onEnter as
                                      unsafe extern "C" fn() -> libc::c_long),
                         onExit:
                             Some(cmsx_Power_onExit as
                                      unsafe extern "C" fn(_:
                                                               *const OSD_Entry)
                                          -> libc::c_long),
                         entries: cmsx_menuPowerEntries.as_ptr() as *mut _,};
            init
        }
    };
