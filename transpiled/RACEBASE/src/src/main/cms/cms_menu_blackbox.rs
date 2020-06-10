use ::libc;
extern "C" {
    #[no_mangle]
    static mut blackboxConfig_System: blackboxConfig_t;
    #[no_mangle]
    fn blackboxValidateConfig();
    #[no_mangle]
    fn blackboxMayEditConfig() -> bool;
    #[no_mangle]
    fn displayClearScreen(instance: *mut displayPort_t);
    #[no_mangle]
    fn displayWrite(instance: *mut displayPort_t, x: uint8_t, y: uint8_t,
                    s: *const libc::c_char) -> libc::c_int;
    #[no_mangle]
    fn displayResync(instance: *mut displayPort_t);
    // Disabling this, in favour of tfp_format to be used in cli.c
//int tfp_printf(const char *fmt, ...);
    #[no_mangle]
    fn tfp_sprintf(s: *mut libc::c_char, fmt: *const libc::c_char, _: ...)
     -> libc::c_int;
    #[no_mangle]
    fn delay(ms: timeMs_t);
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
    // Automatically trigger a flush when this much data is in the buffer
    #[no_mangle]
    fn flashfsEraseCompletely();
    #[no_mangle]
    fn flashfsGetOffset() -> uint32_t;
    #[no_mangle]
    fn flashfsGetGeometry() -> *const flashGeometry_s;
    #[no_mangle]
    fn flashfsIsSupported() -> bool;
    #[no_mangle]
    fn flashfsIsReady() -> bool;
    #[no_mangle]
    fn beeper(mode: beeperMode_e);
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type timeMs_t = uint32_t;
pub type BlackboxDevice = libc::c_uint;
pub const BLACKBOX_DEVICE_SERIAL: BlackboxDevice = 3;
pub const BLACKBOX_DEVICE_FLASH: BlackboxDevice = 1;
pub const BLACKBOX_DEVICE_NONE: BlackboxDevice = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct blackboxConfig_s {
    pub p_ratio: uint16_t,
    pub device: uint8_t,
    pub record_acc: uint8_t,
    pub mode: uint8_t,
}
pub type blackboxConfig_t = blackboxConfig_s;
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
// I-frame interval / P-frame interval
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
pub type flashGeometry_t = flashGeometry_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct flashGeometry_s {
    pub sectors: uint16_t,
    pub pageSize: uint16_t,
    pub sectorSize: uint32_t,
    pub totalSize: uint32_t,
    pub pagesPerSector: uint16_t,
    pub flashType: flashType_e,
}
pub type flashType_e = libc::c_uint;
pub const FLASH_TYPE_NAND: flashType_e = 1;
pub const FLASH_TYPE_NOR: flashType_e = 0;
pub type beeperMode_e = libc::c_uint;
pub const BEEPER_ALL: beeperMode_e = 24;
pub const BEEPER_RC_SMOOTHING_INIT_FAIL: beeperMode_e = 23;
pub const BEEPER_CAM_CONNECTION_CLOSE: beeperMode_e = 22;
pub const BEEPER_CAM_CONNECTION_OPEN: beeperMode_e = 21;
pub const BEEPER_CRASH_FLIP_MODE: beeperMode_e = 20;
pub const BEEPER_BLACKBOX_ERASE: beeperMode_e = 19;
pub const BEEPER_USB: beeperMode_e = 18;
pub const BEEPER_SYSTEM_INIT: beeperMode_e = 17;
pub const BEEPER_ARMED: beeperMode_e = 16;
pub const BEEPER_DISARM_REPEAT: beeperMode_e = 15;
pub const BEEPER_MULTI_BEEPS: beeperMode_e = 14;
pub const BEEPER_READY_BEEP: beeperMode_e = 13;
pub const BEEPER_ACC_CALIBRATION_FAIL: beeperMode_e = 12;
pub const BEEPER_ACC_CALIBRATION: beeperMode_e = 11;
pub const BEEPER_RX_SET: beeperMode_e = 10;
pub const BEEPER_GPS_STATUS: beeperMode_e = 9;
pub const BEEPER_BAT_LOW: beeperMode_e = 8;
pub const BEEPER_BAT_CRIT_LOW: beeperMode_e = 7;
pub const BEEPER_ARMING_GPS_FIX: beeperMode_e = 6;
pub const BEEPER_ARMING: beeperMode_e = 5;
pub const BEEPER_DISARMING: beeperMode_e = 4;
pub const BEEPER_RX_LOST_LANDING: beeperMode_e = 3;
pub const BEEPER_RX_LOST: beeperMode_e = 2;
pub const BEEPER_GYRO_CALIBRATED: beeperMode_e = 1;
pub const BEEPER_SILENCE: beeperMode_e = 0;
#[inline]
unsafe extern "C" fn blackboxConfig() -> *const blackboxConfig_t {
    return &mut blackboxConfig_System;
}
#[inline]
unsafe extern "C" fn blackboxConfigMutable() -> *mut blackboxConfig_t {
    return &mut blackboxConfig_System;
}
// Count of the number of erasable blocks on the device
// In bytes
// This is just pagesPerSector * pageSize
// This is just sectorSize * sectors
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
// CMS things for blackbox and flashfs.
//
static mut cmsx_BlackboxDeviceNames: [*const libc::c_char; 4] =
    [b"NONE\x00" as *const u8 as *const libc::c_char,
     b"FLASH \x00" as *const u8 as *const libc::c_char,
     b"SDCARD\x00" as *const u8 as *const libc::c_char,
     b"SERIAL\x00" as *const u8 as *const libc::c_char];
static mut blackboxConfig_p_ratio: uint16_t = 0;
static mut cmsx_BlackboxDevice: uint8_t = 0;
static mut cmsx_BlackboxDeviceTable: OSD_TAB_t =
    unsafe {
        {
            let mut init =
                OSD_TAB_t{val:
                              &cmsx_BlackboxDevice as *const uint8_t as
                                  *mut uint8_t,
                          max: 2 as libc::c_int as uint8_t,
                          names: cmsx_BlackboxDeviceNames.as_ptr(),};
            init
        }
    };
static mut cmsx_BlackboxStatus: [libc::c_char; 8] = [0; 8];
static mut cmsx_BlackboxDeviceStorageUsed: [libc::c_char; 8] = [0; 8];
static mut cmsx_BlackboxDeviceStorageFree: [libc::c_char; 8] = [0; 8];
unsafe extern "C" fn cmsx_Blackbox_GetDeviceStatus() {
    let mut unit: *mut libc::c_char =
        b"B\x00" as *const u8 as *const libc::c_char as *mut libc::c_char;
    let mut storageDeviceIsWorking: bool = 0 as libc::c_int != 0;
    let mut storageUsed: uint32_t = 0 as libc::c_int as uint32_t;
    let mut storageFree: uint32_t = 0 as libc::c_int as uint32_t;
    match (*blackboxConfig()).device as libc::c_int {
        1 => {
            unit =
                b"KB\x00" as *const u8 as *const libc::c_char as
                    *mut libc::c_char;
            storageDeviceIsWorking = flashfsIsSupported();
            if storageDeviceIsWorking {
                tfp_sprintf(cmsx_BlackboxStatus.as_mut_ptr(),
                            b"READY\x00" as *const u8 as *const libc::c_char);
                let mut geometry: *const flashGeometry_t =
                    flashfsGetGeometry();
                storageUsed =
                    flashfsGetOffset().wrapping_div(1024 as libc::c_int as
                                                        libc::c_uint);
                storageFree =
                    (*geometry).totalSize.wrapping_div(1024 as libc::c_int as
                                                           libc::c_uint).wrapping_sub(storageUsed)
            } else {
                tfp_sprintf(cmsx_BlackboxStatus.as_mut_ptr(),
                            b"FAULT\x00" as *const u8 as *const libc::c_char);
            }
        }
        _ => {
            tfp_sprintf(cmsx_BlackboxStatus.as_mut_ptr(),
                        b"---\x00" as *const u8 as *const libc::c_char);
        }
    }
    /* Storage counters */
    tfp_sprintf(cmsx_BlackboxDeviceStorageUsed.as_mut_ptr(),
                b"%ld%s\x00" as *const u8 as *const libc::c_char, storageUsed,
                unit); // Was max7456RefreshAll(); Why at this timing?
    tfp_sprintf(cmsx_BlackboxDeviceStorageFree.as_mut_ptr(),
                b"%ld%s\x00" as *const u8 as *const libc::c_char, storageFree,
                unit); // Was max7456RefreshAll(); wedges during heavy SPI?
}
unsafe extern "C" fn cmsx_EraseFlash(mut pDisplay: *mut displayPort_t,
                                     mut ptr: *const libc::c_void)
 -> libc::c_long {
    if !flashfsIsSupported() { return 0 as libc::c_int as libc::c_long }
    displayClearScreen(pDisplay);
    displayWrite(pDisplay, 5 as libc::c_int as uint8_t,
                 3 as libc::c_int as uint8_t,
                 b"ERASING FLASH...\x00" as *const u8 as *const libc::c_char);
    displayResync(pDisplay);
    flashfsEraseCompletely();
    while !flashfsIsReady() { delay(100 as libc::c_int as timeMs_t); }
    beeper(BEEPER_BLACKBOX_ERASE);
    displayClearScreen(pDisplay);
    displayResync(pDisplay);
    // Update storage device status to show new used space amount
    cmsx_Blackbox_GetDeviceStatus();
    return 0 as libc::c_int as libc::c_long;
}
// USE_FLASHFS
unsafe extern "C" fn cmsx_Blackbox_onEnter() -> libc::c_long {
    cmsx_Blackbox_GetDeviceStatus();
    cmsx_BlackboxDevice = (*blackboxConfig()).device;
    blackboxConfig_p_ratio = (*blackboxConfig()).p_ratio;
    return 0 as libc::c_int as libc::c_long;
}
unsafe extern "C" fn cmsx_Blackbox_onExit(mut self_0: *const OSD_Entry)
 -> libc::c_long {
    if blackboxMayEditConfig() {
        (*blackboxConfigMutable()).device = cmsx_BlackboxDevice;
        blackboxValidateConfig();
    }
    (*blackboxConfigMutable()).p_ratio = blackboxConfig_p_ratio;
    return 0 as libc::c_int as libc::c_long;
}
#[no_mangle]
pub static mut cmsx_menuBlackbox: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter:
                             Some(cmsx_Blackbox_onEnter as
                                      unsafe extern "C" fn() -> libc::c_long),
                         onExit:
                             Some(cmsx_Blackbox_onExit as
                                      unsafe extern "C" fn(_:
                                                               *const OSD_Entry)
                                          -> libc::c_long),
                         entries:
                             cmsx_menuBlackboxEntries.as_ptr() as *mut _,};
            init
        }
    };
