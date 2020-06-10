use ::libc;
extern "C" {
    #[no_mangle]
    fn strcpy(_: *mut libc::c_char, _: *const libc::c_char)
     -> *mut libc::c_char;
    #[no_mangle]
    fn strncpy(_: *mut libc::c_char, _: *const libc::c_char, _: libc::c_ulong)
     -> *mut libc::c_char;
    #[no_mangle]
    fn strncat(_: *mut libc::c_char, _: *const libc::c_char, _: libc::c_ulong)
     -> *mut libc::c_char;
    #[no_mangle]
    fn strlen(_: *const libc::c_char) -> libc::c_ulong;
    // Note: displayPortProfile_t used as a parameter group for CMS over CRSF (io/displayport_crsf)
    #[no_mangle]
    fn displayGrab(instance: *mut displayPort_t);
    #[no_mangle]
    fn displayRelease(instance: *mut displayPort_t);
    #[no_mangle]
    fn displayClearScreen(instance: *mut displayPort_t);
    #[no_mangle]
    fn displayWrite(instance: *mut displayPort_t, x: uint8_t, y: uint8_t,
                    s: *const libc::c_char) -> libc::c_int;
    #[no_mangle]
    fn displayHeartbeat(instance: *mut displayPort_t);
    #[no_mangle]
    fn displayResync(instance: *mut displayPort_t);
    #[no_mangle]
    fn displayTxBytesFree(instance: *const displayPort_t) -> uint16_t;
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
    static mut menuMain: CMS_Menu;
    #[no_mangle]
    fn itoa(i: libc::c_int, a: *mut libc::c_char, r: libc::c_int)
     -> *mut libc::c_char;
    // bootloader/IAP
    #[no_mangle]
    fn systemReset();
    #[no_mangle]
    fn delay(ms: timeMs_t);
    #[no_mangle]
    fn millis() -> timeMs_t;
    #[no_mangle]
    fn saveConfigAndNotify();
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    fn setArmingDisabled(flag: armingDisableFlags_e);
    #[no_mangle]
    fn unsetArmingDisabled(flag: armingDisableFlags_e);
    #[no_mangle]
    fn stopPwmAllMotors();
    #[no_mangle]
    fn stopMotors();
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
    static mut rcdeviceInMenu: bool;
    #[no_mangle]
    static mut rcData: [int16_t; 18];
    #[no_mangle]
    fn getBatteryCellCount() -> uint8_t;
    #[no_mangle]
    static mut usbDevConfig_System: usbDev_t;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
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
pub type timeMs_t = uint32_t;
pub type timeUs_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct OSD_Entry {
    pub text: *const libc::c_char,
    pub type_0: OSD_MenuElement,
    pub func: CMSEntryFuncPtr,
    pub data: *mut libc::c_void,
    pub flags: uint8_t,
}
pub type CMSEntryFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut displayPort_t, _: *const libc::c_void)
               -> libc::c_long>;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct OSD_FLOAT_t {
    pub val: *mut uint8_t,
    pub min: uint8_t,
    pub max: uint8_t,
    pub step: uint8_t,
    pub multipler: uint16_t,
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
pub struct OSD_INT8_t {
    pub val: *mut int8_t,
    pub min: int8_t,
    pub max: int8_t,
    pub step: int8_t,
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
// CMS state
// This is a function used in the func member if the type is OME_Submenu.
pub type CMSMenuOptFuncPtr
    =
    Option<unsafe extern "C" fn() -> *mut libc::c_char>;
pub type cmsCtx_t = cmsCtx_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct cmsCtx_s {
    pub menu: *const CMS_Menu,
    pub page: uint8_t,
    pub cursorRow: int8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct CMS_Menu {
    pub onEnter: CMSMenuFuncPtr,
    pub onExit: CMSMenuOnExitPtr,
    pub entries: *mut OSD_Entry,
}
// menu for this context
// page in the menu
// cursorRow in the page
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
// (Temporary) Flag for OME_Submenu, indicating func should be called to get a string to display.
pub type CMSMenuFuncPtr = Option<unsafe extern "C" fn() -> libc::c_long>;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct OSD_INT16_t {
    pub val: *mut int16_t,
    pub min: int16_t,
    pub max: int16_t,
    pub step: int16_t,
}
pub type armingDisableFlags_e = libc::c_uint;
pub const ARMING_DISABLED_ARM_SWITCH: armingDisableFlags_e = 524288;
pub const ARMING_DISABLED_GPS: armingDisableFlags_e = 262144;
pub const ARMING_DISABLED_PARALYZE: armingDisableFlags_e = 131072;
pub const ARMING_DISABLED_MSP: armingDisableFlags_e = 65536;
pub const ARMING_DISABLED_BST: armingDisableFlags_e = 32768;
pub const ARMING_DISABLED_OSD_MENU: armingDisableFlags_e = 16384;
pub const ARMING_DISABLED_CMS_MENU: armingDisableFlags_e = 8192;
pub const ARMING_DISABLED_CLI: armingDisableFlags_e = 4096;
pub const ARMING_DISABLED_CALIBRATING: armingDisableFlags_e = 2048;
pub const ARMING_DISABLED_LOAD: armingDisableFlags_e = 1024;
pub const ARMING_DISABLED_NOPREARM: armingDisableFlags_e = 512;
pub const ARMING_DISABLED_BOOT_GRACE_TIME: armingDisableFlags_e = 256;
pub const ARMING_DISABLED_ANGLE: armingDisableFlags_e = 128;
pub const ARMING_DISABLED_THROTTLE: armingDisableFlags_e = 64;
pub const ARMING_DISABLED_RUNAWAY_TAKEOFF: armingDisableFlags_e = 32;
pub const ARMING_DISABLED_BOXFAILSAFE: armingDisableFlags_e = 16;
pub const ARMING_DISABLED_BAD_RX_RECOVERY: armingDisableFlags_e = 8;
pub const ARMING_DISABLED_RX_FAILSAFE: armingDisableFlags_e = 4;
pub const ARMING_DISABLED_FAILSAFE: armingDisableFlags_e = 2;
pub const ARMING_DISABLED_NO_GYRO: armingDisableFlags_e = 1;
pub const YAW: rc_alias = 2;
pub const ROLL: rc_alias = 0;
pub const PITCH: rc_alias = 1;
pub const ARMED: C2RustUnnamed = 1;
pub const THROTTLE: rc_alias = 3;
pub const COMPOSITE: USB_DEV = 1;
pub type usbDev_t = usbDev_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct usbDev_s {
    pub type_0: uint8_t,
    pub mscButtonPin: ioTag_t,
    pub mscButtonUsePullup: uint8_t,
}
pub type ioTag_t = uint8_t;
pub type rc_alias = libc::c_uint;
pub const AUX8: rc_alias = 11;
pub const AUX7: rc_alias = 10;
pub const AUX6: rc_alias = 9;
pub const AUX5: rc_alias = 8;
pub const AUX4: rc_alias = 7;
pub const AUX3: rc_alias = 6;
pub const AUX2: rc_alias = 5;
pub const AUX1: rc_alias = 4;
pub type C2RustUnnamed = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed = 2;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rcdeviceSwitchState_s {
    pub isActivated: bool,
}
pub type rcdeviceSwitchState_t = rcdeviceSwitchState_s;
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
pub type USB_DEV = libc::c_uint;
pub const DEFAULT: USB_DEV = 0;
#[no_mangle]
pub static mut switchStates: [rcdeviceSwitchState_t; 3] =
    [rcdeviceSwitchState_t{isActivated: false,}; 3];
#[inline]
unsafe extern "C" fn usbDevConfig() -> *const usbDev_t {
    return &mut usbDevConfig_System; // -1 Okay
}
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
static mut cmsDisplayPorts: [*mut displayPort_t; 4] =
    [0 as *const displayPort_t as *mut displayPort_t; 4];
static mut cmsDeviceCount: libc::c_int = 0;
static mut cmsCurrentDevice: libc::c_int = -(1 as libc::c_int);
#[no_mangle]
pub unsafe extern "C" fn cmsDisplayPortRegister(mut pDisplay:
                                                    *mut displayPort_t)
 -> bool {
    if cmsDeviceCount == 4 as libc::c_int { return 0 as libc::c_int != 0 }
    let fresh0 = cmsDeviceCount;
    cmsDeviceCount = cmsDeviceCount + 1;
    cmsDisplayPorts[fresh0 as usize] = pDisplay;
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn cmsDisplayPortSelectCurrent() -> *mut displayPort_t {
    if cmsDeviceCount == 0 as libc::c_int { return 0 as *mut displayPort_t }
    if cmsCurrentDevice < 0 as libc::c_int {
        cmsCurrentDevice = 0 as libc::c_int
    }
    return cmsDisplayPorts[cmsCurrentDevice as usize];
}
unsafe extern "C" fn cmsDisplayPortSelectNext() -> *mut displayPort_t {
    if cmsDeviceCount == 0 as libc::c_int { return 0 as *mut displayPort_t }
    cmsCurrentDevice = (cmsCurrentDevice + 1 as libc::c_int) % cmsDeviceCount;
    return cmsDisplayPorts[cmsCurrentDevice as usize];
}
#[no_mangle]
pub unsafe extern "C" fn cmsDisplayPortSelect(mut instance:
                                                  *mut displayPort_t)
 -> bool {
    if cmsDeviceCount == 0 as libc::c_int { return 0 as libc::c_int != 0 }
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < cmsDeviceCount {
        if cmsDisplayPortSelectNext() == instance {
            return 1 as libc::c_int != 0
        }
        i += 1
    }
    return 0 as libc::c_int != 0;
}
// Less is a small screen
static mut smallScreen: bool = false;
static mut leftMenuColumn: uint8_t = 0;
static mut rightMenuColumn: uint8_t = 0;
static mut maxMenuItems: uint8_t = 0;
static mut linesPerMenuItem: uint8_t = 0;
#[no_mangle]
pub static mut cmsInMenu: bool = 0 as libc::c_int != 0;
static mut menuStack: [cmsCtx_t; 10] =
    [cmsCtx_t{menu: 0 as *const CMS_Menu, page: 0, cursorRow: 0,}; 10];
static mut menuStackIdx: uint8_t = 0 as libc::c_int as uint8_t;
static mut pageCount: int8_t = 0;
// Number of pages in the current menu
static mut pageTop: *mut OSD_Entry = 0 as *const OSD_Entry as *mut OSD_Entry;
// First entry for the current page
static mut pageMaxRow: uint8_t = 0;
// Max row in the current page
static mut currentCtx: cmsCtx_t =
    cmsCtx_t{menu: 0 as *const CMS_Menu, page: 0, cursorRow: 0,};
// For external menu content creators
unsafe extern "C" fn cmsUpdateMaxRow(mut instance: *mut displayPort_t) {
    pageMaxRow = 0 as libc::c_int as uint8_t;
    let mut ptr: *const OSD_Entry = pageTop;
    while (*ptr).type_0 as libc::c_uint !=
              OME_END as libc::c_int as libc::c_uint {
        pageMaxRow = pageMaxRow.wrapping_add(1);
        ptr = ptr.offset(1)
    }
    if pageMaxRow as libc::c_int > maxMenuItems as libc::c_int {
        pageMaxRow = maxMenuItems
    }
    pageMaxRow = pageMaxRow.wrapping_sub(1);
}
unsafe extern "C" fn cmsCursorAbsolute(mut instance: *mut displayPort_t)
 -> uint8_t {
    return (currentCtx.cursorRow as libc::c_int +
                currentCtx.page as libc::c_int * maxMenuItems as libc::c_int)
               as uint8_t;
}
unsafe extern "C" fn cmsPageSelect(mut instance: *mut displayPort_t,
                                   mut newpage: int8_t) {
    currentCtx.page =
        ((newpage as libc::c_int + pageCount as libc::c_int) %
             pageCount as libc::c_int) as uint8_t;
    pageTop =
        &mut *(*currentCtx.menu).entries.offset((currentCtx.page as
                                                     libc::c_int *
                                                     maxMenuItems as
                                                         libc::c_int) as
                                                    isize) as *mut OSD_Entry;
    cmsUpdateMaxRow(instance);
    displayClearScreen(instance);
}
unsafe extern "C" fn cmsPageNext(mut instance: *mut displayPort_t) {
    cmsPageSelect(instance,
                  (currentCtx.page as libc::c_int + 1 as libc::c_int) as
                      int8_t);
}
unsafe extern "C" fn cmsPagePrev(mut instance: *mut displayPort_t) {
    cmsPageSelect(instance,
                  (currentCtx.page as libc::c_int - 1 as libc::c_int) as
                      int8_t);
}
unsafe extern "C" fn cmsFormatFloat(mut value: int32_t,
                                    mut floatString: *mut libc::c_char) {
    let mut k: uint8_t = 0;
    // np. 3450
    itoa(100000 as libc::c_int + value, floatString,
         10 as libc::c_int); // Create string from abs of integer value
    // 103450
    *floatString.offset(0 as libc::c_int as isize) =
        *floatString.offset(1 as libc::c_int as isize);
    *floatString.offset(1 as libc::c_int as isize) =
        *floatString.offset(2 as libc::c_int as isize);
    *floatString.offset(2 as libc::c_int as isize) =
        '.' as i32 as libc::c_char;
    // 03.450
    // usuwam koncowe zera i kropke
    // Keep the first decimal place
    k = 5 as libc::c_int as uint8_t;
    while k as libc::c_int > 3 as libc::c_int {
        if !(*floatString.offset(k as isize) as libc::c_int == '0' as i32 ||
                 *floatString.offset(k as isize) as libc::c_int == '.' as i32)
           {
            break ;
        }
        *floatString.offset(k as isize) = 0 as libc::c_int as libc::c_char;
        k = k.wrapping_sub(1)
    }
    // oraz zero wiodonce
    if *floatString.offset(0 as libc::c_int as isize) as libc::c_int ==
           '0' as i32 {
        *floatString.offset(0 as libc::c_int as isize) =
            ' ' as i32 as libc::c_char
    };
}
// CMS on OSD legacy was to use LEFT aligned values, not the RIGHT way ;-)
// Pad buffer to the left, i.e. align right
unsafe extern "C" fn cmsPadLeftToSize(mut buf: *mut libc::c_char,
                                      mut size: libc::c_int) {
    let mut i: libc::c_int = 0;
    let mut j: libc::c_int = 0;
    let mut len: libc::c_int = strlen(buf) as libc::c_int;
    i = size - 1 as libc::c_int;
    j = size - len;
    while i - j >= 0 as libc::c_int {
        *buf.offset(i as isize) = *buf.offset((i - j) as isize);
        i -= 1
    }
    while i >= 0 as libc::c_int {
        *buf.offset(i as isize) = ' ' as i32 as libc::c_char;
        i -= 1
    }
    *buf.offset(size as isize) = 0 as libc::c_int as libc::c_char;
}
unsafe extern "C" fn cmsPadToSize(mut buf: *mut libc::c_char,
                                  mut size: libc::c_int) {
    // Make absolutely sure the string terminated.
    *buf.offset(size as isize) =
        0 as libc::c_int as libc::c_char; // Make room for null terminator.
    cmsPadLeftToSize(buf, size);
}
unsafe extern "C" fn cmsDrawMenuItemValue(mut pDisplay: *mut displayPort_t,
                                          mut buff: *mut libc::c_char,
                                          mut row: uint8_t,
                                          mut maxSize: uint8_t)
 -> libc::c_int {
    let mut colpos: libc::c_int = 0;
    let mut cnt: libc::c_int = 0;
    cmsPadToSize(buff, maxSize as libc::c_int);
    colpos = rightMenuColumn as libc::c_int - maxSize as libc::c_int;
    cnt = displayWrite(pDisplay, colpos as uint8_t, row, buff);
    return cnt;
}
unsafe extern "C" fn cmsDrawMenuEntry(mut pDisplay: *mut displayPort_t,
                                      mut p: *mut OSD_Entry, mut row: uint8_t)
 -> libc::c_int {
    let mut buff: [libc::c_char; 13] = [0; 13];
    let mut cnt: libc::c_int = 0 as libc::c_int;
    if smallScreen { row = row.wrapping_add(1) }
    match (*p).type_0 as libc::c_uint {
        10 => {
            if (*p).flags as libc::c_int & 0x1 as libc::c_int != 0 &&
                   !(*p).data.is_null() {
                strncpy(buff.as_mut_ptr(), (*p).data as *const libc::c_char,
                        12 as libc::c_int as libc::c_ulong);
                cnt =
                    cmsDrawMenuItemValue(pDisplay, buff.as_mut_ptr(), row,
                                         12 as libc::c_int as uint8_t);
                (*p).flags =
                    ((*p).flags as libc::c_int & !(0x1 as libc::c_int)) as
                        uint8_t
            }
        }
        3 | 4 => {
            if (*p).flags as libc::c_int & 0x1 as libc::c_int != 0 {
                buff[0 as libc::c_int as usize] =
                    0 as libc::c_int as libc::c_char;
                if (*p).type_0 as libc::c_uint ==
                       OME_Submenu as libc::c_int as libc::c_uint &&
                       (*p).func.is_some() &&
                       (*p).flags as libc::c_int & 0x8 as libc::c_int != 0 {
                    // Special case of sub menu entry with optional value display.
                    let mut str: *mut libc::c_char =
                        ::core::mem::transmute::<CMSEntryFuncPtr,
                                                 CMSMenuOptFuncPtr>((*p).func).expect("non-null function pointer")();
                    strncpy(buff.as_mut_ptr(), str,
                            12 as libc::c_int as libc::c_ulong);
                }
                strncat(buff.as_mut_ptr(),
                        b">\x00" as *const u8 as *const libc::c_char,
                        12 as libc::c_int as libc::c_ulong);
                row =
                    if smallScreen as libc::c_int != 0 {
                        (row as libc::c_int) - 1 as libc::c_int
                    } else { row as libc::c_int } as uint8_t;
                cnt =
                    cmsDrawMenuItemValue(pDisplay, buff.as_mut_ptr(), row,
                                         strlen(buff.as_mut_ptr()) as
                                             uint8_t);
                (*p).flags =
                    ((*p).flags as libc::c_int & !(0x1 as libc::c_int)) as
                        uint8_t
            }
        }
        5 => {
            if (*p).flags as libc::c_int & 0x1 as libc::c_int != 0 &&
                   !(*p).data.is_null() {
                if *((*p).data as *mut uint8_t) != 0 {
                    strcpy(buff.as_mut_ptr(),
                           b"YES\x00" as *const u8 as *const libc::c_char);
                } else {
                    strcpy(buff.as_mut_ptr(),
                           b"NO \x00" as *const u8 as *const libc::c_char);
                }
                cnt =
                    cmsDrawMenuItemValue(pDisplay, buff.as_mut_ptr(), row,
                                         3 as libc::c_int as uint8_t);
                (*p).flags =
                    ((*p).flags as libc::c_int & !(0x1 as libc::c_int)) as
                        uint8_t
            }
        }
        13 => {
            if (*p).flags as libc::c_int & 0x1 as libc::c_int != 0 {
                let mut ptr: *mut OSD_TAB_t = (*p).data as *mut OSD_TAB_t;
                let mut str_0: *mut libc::c_char =
                    *(*ptr).names.offset(*(*ptr).val as isize) as
                        *mut libc::c_char;
                strncpy(buff.as_mut_ptr(), str_0,
                        12 as libc::c_int as libc::c_ulong);
                cnt =
                    cmsDrawMenuItemValue(pDisplay, buff.as_mut_ptr(), row,
                                         12 as libc::c_int as uint8_t);
                (*p).flags =
                    ((*p).flags as libc::c_int & !(0x1 as libc::c_int)) as
                        uint8_t
            }
        }
        12 => {
            if (*p).flags as libc::c_int & 0x1 as libc::c_int != 0 &&
                   !(*p).data.is_null() {
                let mut val: *mut uint16_t = (*p).data as *mut uint16_t;
                if *val as libc::c_int & 0x800 as libc::c_int != 0 {
                    strcpy(buff.as_mut_ptr(),
                           b"YES\x00" as *const u8 as *const libc::c_char);
                } else {
                    strcpy(buff.as_mut_ptr(),
                           b"NO \x00" as *const u8 as *const libc::c_char);
                }
                cnt =
                    cmsDrawMenuItemValue(pDisplay, buff.as_mut_ptr(), row,
                                         3 as libc::c_int as uint8_t);
                (*p).flags =
                    ((*p).flags as libc::c_int & !(0x1 as libc::c_int)) as
                        uint8_t
            }
        }
        7 => {
            if (*p).flags as libc::c_int & 0x1 as libc::c_int != 0 &&
                   !(*p).data.is_null() {
                let mut ptr_0: *mut OSD_UINT8_t =
                    (*p).data as *mut OSD_UINT8_t;
                itoa(*(*ptr_0).val as libc::c_int, buff.as_mut_ptr(),
                     10 as libc::c_int);
                cnt =
                    cmsDrawMenuItemValue(pDisplay, buff.as_mut_ptr(), row,
                                         5 as libc::c_int as uint8_t);
                (*p).flags =
                    ((*p).flags as libc::c_int & !(0x1 as libc::c_int)) as
                        uint8_t
            }
        }
        6 => {
            if (*p).flags as libc::c_int & 0x1 as libc::c_int != 0 &&
                   !(*p).data.is_null() {
                let mut ptr_1: *mut OSD_INT8_t = (*p).data as *mut OSD_INT8_t;
                itoa(*(*ptr_1).val as libc::c_int, buff.as_mut_ptr(),
                     10 as libc::c_int);
                cnt =
                    cmsDrawMenuItemValue(pDisplay, buff.as_mut_ptr(), row,
                                         5 as libc::c_int as uint8_t);
                (*p).flags =
                    ((*p).flags as libc::c_int & !(0x1 as libc::c_int)) as
                        uint8_t
            }
        }
        8 => {
            if (*p).flags as libc::c_int & 0x1 as libc::c_int != 0 &&
                   !(*p).data.is_null() {
                let mut ptr_2: *mut OSD_UINT16_t =
                    (*p).data as *mut OSD_UINT16_t;
                itoa(*(*ptr_2).val as libc::c_int, buff.as_mut_ptr(),
                     10 as libc::c_int);
                cnt =
                    cmsDrawMenuItemValue(pDisplay, buff.as_mut_ptr(), row,
                                         5 as libc::c_int as uint8_t);
                (*p).flags =
                    ((*p).flags as libc::c_int & !(0x1 as libc::c_int)) as
                        uint8_t
            }
        }
        9 => {
            if (*p).flags as libc::c_int & 0x1 as libc::c_int != 0 &&
                   !(*p).data.is_null() {
                let mut ptr_3: *mut OSD_UINT16_t =
                    (*p).data as *mut OSD_UINT16_t;
                itoa(*(*ptr_3).val as libc::c_int, buff.as_mut_ptr(),
                     10 as libc::c_int);
                cnt =
                    cmsDrawMenuItemValue(pDisplay, buff.as_mut_ptr(), row,
                                         5 as libc::c_int as uint8_t);
                (*p).flags =
                    ((*p).flags as libc::c_int & !(0x1 as libc::c_int)) as
                        uint8_t
            }
        }
        11 => {
            if (*p).flags as libc::c_int & 0x1 as libc::c_int != 0 &&
                   !(*p).data.is_null() {
                let mut ptr_4: *mut OSD_FLOAT_t =
                    (*p).data as *mut OSD_FLOAT_t;
                cmsFormatFloat(*(*ptr_4).val as libc::c_int *
                                   (*ptr_4).multipler as libc::c_int,
                               buff.as_mut_ptr());
                cnt =
                    cmsDrawMenuItemValue(pDisplay, buff.as_mut_ptr(), row,
                                         5 as libc::c_int as uint8_t);
                (*p).flags =
                    ((*p).flags as libc::c_int & !(0x1 as libc::c_int)) as
                        uint8_t
            }
        }
        0 => {
            if (*p).flags as libc::c_int & 0x1 as libc::c_int != 0 &&
                   !(*p).data.is_null() {
                // A label with optional string, immediately following text
                cnt =
                    displayWrite(pDisplay,
                                 (leftMenuColumn as libc::c_int +
                                      1 as libc::c_int +
                                      strlen((*p).text) as uint8_t as
                                          libc::c_int) as uint8_t, row,
                                 (*p).data as *const libc::c_char);
                (*p).flags =
                    ((*p).flags as libc::c_int & !(0x1 as libc::c_int)) as
                        uint8_t
            }
        }
        2 | 14 | 1 | 15 | _ => { }
    }
    return cnt;
}
unsafe extern "C" fn cmsDrawMenu(mut pDisplay: *mut displayPort_t,
                                 mut currentTimeUs: uint32_t) {
    if pageTop.is_null() { return }
    let mut i: uint8_t = 0;
    let mut p: *mut OSD_Entry = 0 as *mut OSD_Entry;
    let mut top: uint8_t =
        if smallScreen as libc::c_int != 0 {
            1 as libc::c_int
        } else {
            ((*pDisplay).rows as libc::c_int - pageMaxRow as libc::c_int) /
                2 as libc::c_int
        } as uint8_t;
    // Polled (dynamic) value display denominator.
    let mut drawPolled: bool = 0 as libc::c_int != 0;
    static mut lastPolledUs: uint32_t = 0 as libc::c_int as uint32_t;
    if currentTimeUs >
           lastPolledUs.wrapping_add(100000 as libc::c_int as libc::c_uint) {
        drawPolled = 1 as libc::c_int != 0;
        lastPolledUs = currentTimeUs
    }
    let mut room: uint32_t = displayTxBytesFree(pDisplay) as uint32_t;
    if (*pDisplay).cleared {
        p = pageTop;
        i = 0 as libc::c_int as uint8_t;
        while (*p).type_0 as libc::c_uint !=
                  OME_END as libc::c_int as libc::c_uint {
            (*p).flags =
                ((*p).flags as libc::c_int | 0x2 as libc::c_int) as uint8_t;
            (*p).flags =
                ((*p).flags as libc::c_int | 0x1 as libc::c_int) as uint8_t;
            p = p.offset(1);
            i = i.wrapping_add(1)
        }
        (*pDisplay).cleared = 0 as libc::c_int != 0
    } else if drawPolled {
        p = pageTop;
        while p <= pageTop.offset(pageMaxRow as libc::c_int as isize) {
            if (*p).flags as libc::c_int & 0x4 as libc::c_int != 0 {
                (*p).flags =
                    ((*p).flags as libc::c_int | 0x1 as libc::c_int) as
                        uint8_t
            }
            p = p.offset(1)
        }
    }
    // Cursor manipulation
    while (*pageTop.offset(currentCtx.cursorRow as libc::c_int as
                               isize)).type_0 as libc::c_uint ==
              OME_Label as libc::c_int as libc::c_uint {
        // skip label
        currentCtx.cursorRow += 1
    }
    if (*pDisplay).cursorRow as libc::c_int >= 0 as libc::c_int &&
           currentCtx.cursorRow as libc::c_int !=
               (*pDisplay).cursorRow as libc::c_int {
        room =
            (room as
                 libc::c_uint).wrapping_sub(displayWrite(pDisplay,
                                                         leftMenuColumn,
                                                         (top as libc::c_int +
                                                              (*pDisplay).cursorRow
                                                                  as
                                                                  libc::c_int
                                                                  *
                                                                  linesPerMenuItem
                                                                      as
                                                                      libc::c_int)
                                                             as uint8_t,
                                                         b" \x00" as *const u8
                                                             as
                                                             *const libc::c_char)
                                                as libc::c_uint) as uint32_t
                as uint32_t
    }
    if room < 30 as libc::c_int as libc::c_uint { return }
    if (*pDisplay).cursorRow as libc::c_int !=
           currentCtx.cursorRow as libc::c_int {
        room =
            (room as
                 libc::c_uint).wrapping_sub(displayWrite(pDisplay,
                                                         leftMenuColumn,
                                                         (top as libc::c_int +
                                                              currentCtx.cursorRow
                                                                  as
                                                                  libc::c_int
                                                                  *
                                                                  linesPerMenuItem
                                                                      as
                                                                      libc::c_int)
                                                             as uint8_t,
                                                         b">\x00" as *const u8
                                                             as
                                                             *const libc::c_char)
                                                as libc::c_uint) as uint32_t
                as uint32_t;
        (*pDisplay).cursorRow = currentCtx.cursorRow
    }
    if room < 30 as libc::c_int as libc::c_uint { return }
    // Print text labels
    i = 0 as libc::c_int as uint8_t;
    p = pageTop;
    while (i as libc::c_int) < maxMenuItems as libc::c_int &&
              (*p).type_0 as libc::c_uint !=
                  OME_END as libc::c_int as libc::c_uint {
        if (*p).flags as libc::c_int & 0x2 as libc::c_int != 0 {
            let mut coloff: uint8_t = leftMenuColumn;
            coloff =
                (coloff as libc::c_int +
                     if (*p).type_0 as libc::c_uint ==
                            OME_Label as libc::c_int as libc::c_uint {
                         0 as libc::c_int
                     } else { 1 as libc::c_int }) as uint8_t;
            room =
                (room as
                     libc::c_uint).wrapping_sub(displayWrite(pDisplay, coloff,
                                                             (top as
                                                                  libc::c_int
                                                                  +
                                                                  i as
                                                                      libc::c_int
                                                                      *
                                                                      linesPerMenuItem
                                                                          as
                                                                          libc::c_int)
                                                                 as uint8_t,
                                                             (*p).text) as
                                                    libc::c_uint) as uint32_t
                    as uint32_t;
            (*p).flags =
                ((*p).flags as libc::c_int & !(0x2 as libc::c_int)) as
                    uint8_t;
            if room < 30 as libc::c_int as libc::c_uint { return }
        }
        // Print values
        // XXX Polled values at latter positions in the list may not be
    // XXX printed if not enough room in the middle of the list.
        if (*p).flags as libc::c_int & 0x1 as libc::c_int != 0 {
            room =
                (room as
                     libc::c_uint).wrapping_sub(cmsDrawMenuEntry(pDisplay, p,
                                                                 (top as
                                                                      libc::c_int
                                                                      +
                                                                      i as
                                                                          libc::c_int
                                                                          *
                                                                          linesPerMenuItem
                                                                              as
                                                                              libc::c_int)
                                                                     as
                                                                     uint8_t)
                                                    as libc::c_uint) as
                    uint32_t as uint32_t;
            if room < 30 as libc::c_int as libc::c_uint { return }
        }
        i = i.wrapping_add(1);
        p = p.offset(1)
    };
}
unsafe extern "C" fn cmsMenuCountPage(mut pDisplay: *mut displayPort_t) {
    let mut p: *const OSD_Entry = 0 as *const OSD_Entry;
    p = (*currentCtx.menu).entries;
    while (*p).type_0 as libc::c_uint !=
              OME_END as libc::c_int as libc::c_uint {
        p = p.offset(1)
    }
    pageCount =
        ((p.wrapping_offset_from((*currentCtx.menu).entries) as libc::c_long -
              1 as libc::c_int as libc::c_long) / maxMenuItems as libc::c_long
             + 1 as libc::c_int as libc::c_long) as int8_t;
}
// Forward; will be resolved after merging
#[no_mangle]
pub unsafe extern "C" fn cmsMenuChange(mut pDisplay: *mut displayPort_t,
                                       mut ptr: *const libc::c_void)
 -> libc::c_long {
    let mut pMenu: *const CMS_Menu = ptr as *const CMS_Menu;
    if pMenu.is_null() { return 0 as libc::c_int as libc::c_long }
    if pMenu != currentCtx.menu {
        // Stack the current menu and move to a new menu.
        let fresh1 = menuStackIdx;
        menuStackIdx = menuStackIdx.wrapping_add(1);
        menuStack[fresh1 as usize] = currentCtx;
        currentCtx.menu = pMenu;
        currentCtx.cursorRow = 0 as libc::c_int as int8_t;
        if (*pMenu).onEnter.is_some() &&
               (*pMenu).onEnter.expect("non-null function pointer")() ==
                   -(1 as libc::c_int) as libc::c_long {
            return cmsMenuBack(pDisplay)
        }
        cmsMenuCountPage(pDisplay);
        cmsPageSelect(pDisplay, 0 as libc::c_int as int8_t);
    } else {
        // The (pMenu == curretMenu) case occurs when reopening for display cycling
        // currentCtx.cursorRow has been saved as absolute; convert it back to page + relative
        let mut cursorAbs: int8_t = currentCtx.cursorRow;
        currentCtx.cursorRow =
            (cursorAbs as libc::c_int % maxMenuItems as libc::c_int) as
                int8_t;
        cmsMenuCountPage(pDisplay);
        cmsPageSelect(pDisplay,
                      (cursorAbs as libc::c_int / maxMenuItems as libc::c_int)
                          as int8_t);
    }
    return 0 as libc::c_int as libc::c_long;
}
unsafe extern "C" fn cmsMenuBack(mut pDisplay: *mut displayPort_t)
 -> libc::c_long {
    // Let onExit function decide whether to allow exit or not.
    if (*currentCtx.menu).onExit.is_some() &&
           (*currentCtx.menu).onExit.expect("non-null function pointer")(pageTop.offset(currentCtx.cursorRow
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            isize))
               < 0 as libc::c_int as libc::c_long {
        return -(1 as libc::c_int) as libc::c_long
    }
    if menuStackIdx == 0 { return 0 as libc::c_int as libc::c_long }
    menuStackIdx = menuStackIdx.wrapping_sub(1);
    currentCtx = menuStack[menuStackIdx as usize];
    cmsMenuCountPage(pDisplay);
    cmsPageSelect(pDisplay, currentCtx.page as int8_t);
    return 0 as libc::c_int as libc::c_long;
}
#[no_mangle]
pub unsafe extern "C" fn cmsMenuOpen() {
    if !cmsInMenu {
        // New open
        pCurrentDisplay = cmsDisplayPortSelectCurrent();
        if pCurrentDisplay.is_null() { return }
        cmsInMenu = 1 as libc::c_int != 0;
        currentCtx =
            {
                let mut init =
                    cmsCtx_s{menu: &mut menuMain,
                             page: 0 as libc::c_int as uint8_t,
                             cursorRow: 0 as libc::c_int as int8_t,};
                init
            };
        setArmingDisabled(ARMING_DISABLED_CMS_MENU);
    } else {
        // Switch display
        let mut pNextDisplay: *mut displayPort_t = cmsDisplayPortSelectNext();
        if pNextDisplay != pCurrentDisplay {
            // DisplayPort has been changed.
            // Convert cursorRow to absolute value
            currentCtx.cursorRow =
                cmsCursorAbsolute(pCurrentDisplay) as
                    int8_t; // grab the display for use by the CMS
            displayRelease(pCurrentDisplay); // Forced exit
            pCurrentDisplay = pNextDisplay
        } else { return }
    } // Was max7456RefreshAll(); why at this timing?
    displayGrab(pCurrentDisplay);
    if ((*pCurrentDisplay).cols as libc::c_int) < 18 as libc::c_int {
        smallScreen = 1 as libc::c_int != 0;
        linesPerMenuItem = 2 as libc::c_int as uint8_t;
        leftMenuColumn = 0 as libc::c_int as uint8_t;
        rightMenuColumn = (*pCurrentDisplay).cols;
        maxMenuItems =
            ((*pCurrentDisplay).rows as libc::c_int /
                 linesPerMenuItem as libc::c_int) as uint8_t
    } else {
        smallScreen = 0 as libc::c_int != 0;
        linesPerMenuItem = 1 as libc::c_int as uint8_t;
        leftMenuColumn = 2 as libc::c_int as uint8_t;
        rightMenuColumn =
            ((*pCurrentDisplay).cols as libc::c_int - 2 as libc::c_int) as
                uint8_t;
        maxMenuItems =
            ((*pCurrentDisplay).rows as libc::c_int - 2 as libc::c_int) as
                uint8_t
    }
    cmsMenuChange(pCurrentDisplay, currentCtx.menu as *const libc::c_void);
}
unsafe extern "C" fn cmsTraverseGlobalExit(mut pMenu: *const CMS_Menu) {
    let mut p: *const OSD_Entry = (*pMenu).entries;
    while (*p).type_0 as libc::c_uint !=
              OME_END as libc::c_int as libc::c_uint {
        if (*p).type_0 as libc::c_uint ==
               OME_Submenu as libc::c_int as libc::c_uint {
            cmsTraverseGlobalExit((*p).data as *const CMS_Menu);
        }
        p = p.offset(1)
    };
}
#[no_mangle]
pub unsafe extern "C" fn cmsMenuExit(mut pDisplay: *mut displayPort_t,
                                     mut ptr: *const libc::c_void)
 -> libc::c_long {
    let mut exitType: libc::c_int = ptr as libc::c_int;
    match exitType {
        1 | 2 => {
            cmsTraverseGlobalExit(&mut menuMain);
            if (*currentCtx.menu).onExit.is_some() {
                (*currentCtx.menu).onExit.expect("non-null function pointer")(0
                                                                                  as
                                                                                  *mut libc::c_void
                                                                                  as
                                                                                  *mut OSD_Entry);
            }
            saveConfigAndNotify();
        }
        0 | _ => { }
    }
    cmsInMenu = 0 as libc::c_int != 0;
    displayRelease(pDisplay);
    currentCtx.menu = 0 as *const CMS_Menu;
    if exitType == 2 as libc::c_int {
        displayClearScreen(pDisplay);
        displayWrite(pDisplay, 5 as libc::c_int as uint8_t,
                     3 as libc::c_int as uint8_t,
                     b"REBOOTING...\x00" as *const u8 as *const libc::c_char);
        displayResync(pDisplay);
        stopMotors();
        stopPwmAllMotors();
        delay(200 as libc::c_int as timeMs_t);
        systemReset();
    }
    unsetArmingDisabled(ARMING_DISABLED_CMS_MENU);
    return 0 as libc::c_int as libc::c_long;
}
// msec
unsafe extern "C" fn cmsHandleKey(mut pDisplay: *mut displayPort_t,
                                  mut key: uint8_t) -> uint16_t {
    let mut res: uint16_t = 250 as libc::c_int as uint16_t;
    let mut p: *mut OSD_Entry = 0 as *mut OSD_Entry;
    if currentCtx.menu.is_null() { return res }
    if key as libc::c_int == 6 as libc::c_int {
        cmsMenuOpen();
        return 500 as libc::c_int as uint16_t
    }
    if key as libc::c_int == 5 as libc::c_int {
        cmsMenuBack(pDisplay);
        return 500 as libc::c_int as uint16_t
    }
    if key as libc::c_int == 2 as libc::c_int {
        if (currentCtx.cursorRow as libc::c_int) < pageMaxRow as libc::c_int {
            currentCtx.cursorRow += 1
        } else {
            cmsPageNext(pDisplay);
            currentCtx.cursorRow = 0 as libc::c_int as int8_t
            // Goto top in any case
        }
    }
    if key as libc::c_int == 1 as libc::c_int {
        currentCtx.cursorRow -= 1;
        // Skip non-title labels
        if (*pageTop.offset(currentCtx.cursorRow as libc::c_int as
                                isize)).type_0 as libc::c_uint ==
               OME_Label as libc::c_int as libc::c_uint &&
               currentCtx.cursorRow as libc::c_int > 0 as libc::c_int {
            currentCtx.cursorRow -= 1
        }
        if currentCtx.cursorRow as libc::c_int == -(1 as libc::c_int) ||
               (*pageTop.offset(currentCtx.cursorRow as libc::c_int as
                                    isize)).type_0 as libc::c_uint ==
                   OME_Label as libc::c_int as libc::c_uint {
            // Goto previous page
            cmsPagePrev(pDisplay);
            currentCtx.cursorRow = pageMaxRow as int8_t
        }
    }
    if key as libc::c_int == 2 as libc::c_int ||
           key as libc::c_int == 1 as libc::c_int {
        return res
    }
    p = pageTop.offset(currentCtx.cursorRow as libc::c_int as isize);
    let mut retval: libc::c_long = 0;
    match (*p).type_0 as libc::c_uint {
        3 => {
            if key as libc::c_int == 4 as libc::c_int {
                cmsMenuChange(pDisplay, (*p).data);
                res = 500 as libc::c_int as uint16_t
            }
        }
        4 => {
            retval = 0;
            if (*p).func.is_some() && key as libc::c_int == 4 as libc::c_int {
                retval =
                    (*p).func.expect("non-null function pointer")(pDisplay,
                                                                  (*p).data);
                if retval == -(1 as libc::c_int) as libc::c_long {
                    cmsMenuBack(pDisplay);
                }
                res = 500 as libc::c_int as uint16_t
            }
        }
        2 => {
            if (*p).func.is_some() && key as libc::c_int == 4 as libc::c_int {
                (*p).func.expect("non-null function pointer")(pDisplay,
                                                              (*p).data);
                res = 500 as libc::c_int as uint16_t
            }
        }
        1 => { cmsMenuBack(pDisplay); res = 500 as libc::c_int as uint16_t }
        5 => {
            if !(*p).data.is_null() {
                let mut val: *mut uint8_t = (*p).data as *mut uint8_t;
                if key as libc::c_int == 4 as libc::c_int {
                    *val = 1 as libc::c_int as uint8_t
                } else { *val = 0 as libc::c_int as uint8_t }
                (*p).flags =
                    ((*p).flags as libc::c_int | 0x1 as libc::c_int) as
                        uint8_t
            }
        }
        12 => {
            if !(*p).data.is_null() {
                let mut val_0: *mut uint16_t = (*p).data as *mut uint16_t;
                if key as libc::c_int == 4 as libc::c_int {
                    *val_0 =
                        (*val_0 as libc::c_int | 0x800 as libc::c_int) as
                            uint16_t
                } else {
                    *val_0 =
                        (*val_0 as libc::c_int % !(0x800 as libc::c_int)) as
                            uint16_t
                }
                (*p).flags =
                    ((*p).flags as libc::c_int | 0x1 as libc::c_int) as
                        uint8_t
            }
        }
        7 | 11 => {
            if !(*p).data.is_null() {
                let mut ptr: *mut OSD_UINT8_t = (*p).data as *mut OSD_UINT8_t;
                if key as libc::c_int == 4 as libc::c_int {
                    if (*(*ptr).val as libc::c_int) <
                           (*ptr).max as libc::c_int {
                        *(*ptr).val =
                            (*(*ptr).val as libc::c_int +
                                 (*ptr).step as libc::c_int) as uint8_t
                    }
                } else if *(*ptr).val as libc::c_int >
                              (*ptr).min as libc::c_int {
                    *(*ptr).val =
                        (*(*ptr).val as libc::c_int -
                             (*ptr).step as libc::c_int) as uint8_t
                }
                (*p).flags =
                    ((*p).flags as libc::c_int | 0x1 as libc::c_int) as
                        uint8_t;
                if (*p).func.is_some() {
                    (*p).func.expect("non-null function pointer")(pDisplay,
                                                                  p as
                                                                      *const libc::c_void);
                }
            }
        }
        13 => {
            if (*p).type_0 as libc::c_uint ==
                   OME_TAB as libc::c_int as libc::c_uint {
                let mut ptr_0: *mut OSD_TAB_t = (*p).data as *mut OSD_TAB_t;
                if key as libc::c_int == 4 as libc::c_int {
                    if (*(*ptr_0).val as libc::c_int) <
                           (*ptr_0).max as libc::c_int {
                        *(*ptr_0).val =
                            (*(*ptr_0).val as libc::c_int + 1 as libc::c_int)
                                as uint8_t
                    }
                } else if *(*ptr_0).val as libc::c_int > 0 as libc::c_int {
                    *(*ptr_0).val =
                        (*(*ptr_0).val as libc::c_int - 1 as libc::c_int) as
                            uint8_t
                }
                if (*p).func.is_some() {
                    (*p).func.expect("non-null function pointer")(pDisplay,
                                                                  (*p).data);
                }
                (*p).flags =
                    ((*p).flags as libc::c_int | 0x1 as libc::c_int) as
                        uint8_t
            }
        }
        6 => {
            if !(*p).data.is_null() {
                let mut ptr_1: *mut OSD_INT8_t = (*p).data as *mut OSD_INT8_t;
                if key as libc::c_int == 4 as libc::c_int {
                    if (*(*ptr_1).val as libc::c_int) <
                           (*ptr_1).max as libc::c_int {
                        *(*ptr_1).val =
                            (*(*ptr_1).val as libc::c_int +
                                 (*ptr_1).step as libc::c_int) as int8_t
                    }
                } else if *(*ptr_1).val as libc::c_int >
                              (*ptr_1).min as libc::c_int {
                    *(*ptr_1).val =
                        (*(*ptr_1).val as libc::c_int -
                             (*ptr_1).step as libc::c_int) as int8_t
                }
                (*p).flags =
                    ((*p).flags as libc::c_int | 0x1 as libc::c_int) as
                        uint8_t;
                if (*p).func.is_some() {
                    (*p).func.expect("non-null function pointer")(pDisplay,
                                                                  p as
                                                                      *const libc::c_void);
                }
            }
        }
        8 => {
            if !(*p).data.is_null() {
                let mut ptr_2: *mut OSD_UINT16_t =
                    (*p).data as *mut OSD_UINT16_t;
                if key as libc::c_int == 4 as libc::c_int {
                    if (*(*ptr_2).val as libc::c_int) <
                           (*ptr_2).max as libc::c_int {
                        *(*ptr_2).val =
                            (*(*ptr_2).val as libc::c_int +
                                 (*ptr_2).step as libc::c_int) as uint16_t
                    }
                } else if *(*ptr_2).val as libc::c_int >
                              (*ptr_2).min as libc::c_int {
                    *(*ptr_2).val =
                        (*(*ptr_2).val as libc::c_int -
                             (*ptr_2).step as libc::c_int) as uint16_t
                }
                (*p).flags =
                    ((*p).flags as libc::c_int | 0x1 as libc::c_int) as
                        uint8_t;
                if (*p).func.is_some() {
                    (*p).func.expect("non-null function pointer")(pDisplay,
                                                                  p as
                                                                      *const libc::c_void);
                }
            }
        }
        9 => {
            if !(*p).data.is_null() {
                let mut ptr_3: *mut OSD_INT16_t =
                    (*p).data as *mut OSD_INT16_t;
                if key as libc::c_int == 4 as libc::c_int {
                    if (*(*ptr_3).val as libc::c_int) <
                           (*ptr_3).max as libc::c_int {
                        *(*ptr_3).val =
                            (*(*ptr_3).val as libc::c_int +
                                 (*ptr_3).step as libc::c_int) as int16_t
                    }
                } else if *(*ptr_3).val as libc::c_int >
                              (*ptr_3).min as libc::c_int {
                    *(*ptr_3).val =
                        (*(*ptr_3).val as libc::c_int -
                             (*ptr_3).step as libc::c_int) as int16_t
                }
                (*p).flags =
                    ((*p).flags as libc::c_int | 0x1 as libc::c_int) as
                        uint8_t;
                if (*p).func.is_some() {
                    (*p).func.expect("non-null function pointer")(pDisplay,
                                                                  p as
                                                                      *const libc::c_void);
                }
            }
        }
        10 | 0 | 14 | 15 | _ => { }
    }
    return res;
}
#[no_mangle]
pub unsafe extern "C" fn cmsHandleKeyWithRepeat(mut pDisplay:
                                                    *mut displayPort_t,
                                                mut key: uint8_t,
                                                mut repeatCount: libc::c_int)
 -> uint16_t {
    let mut ret: uint16_t = 0 as libc::c_int as uint16_t;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < repeatCount { ret = cmsHandleKey(pDisplay, key); i += 1 }
    return ret;
}
#[no_mangle]
pub unsafe extern "C" fn cmsUpdate(mut currentTimeUs: uint32_t) {
    if rcdeviceInMenu { return }
    if getBatteryCellCount() as libc::c_int == 0 as libc::c_int &&
           (*usbDevConfig()).type_0 as libc::c_int == COMPOSITE as libc::c_int
       {
        return
    }
    static mut rcDelayMs: int16_t = 250 as libc::c_int as int16_t;
    static mut holdCount: libc::c_int = 1 as libc::c_int;
    static mut repeatCount: libc::c_int = 1 as libc::c_int;
    static mut repeatBase: libc::c_int = 0 as libc::c_int;
    static mut lastCalledMs: uint32_t = 0 as libc::c_int as uint32_t;
    static mut lastCmsHeartBeatMs: uint32_t = 0 as libc::c_int as uint32_t;
    let currentTimeMs: uint32_t =
        currentTimeUs.wrapping_div(1000 as libc::c_int as libc::c_uint);
    if !cmsInMenu {
        // Detect menu invocation
        if rcData[THROTTLE as libc::c_int as usize] as libc::c_int >
               1250 as libc::c_int &&
               (rcData[THROTTLE as libc::c_int as usize] as libc::c_int) <
                   1750 as libc::c_int &&
               (rcData[YAW as libc::c_int as usize] as libc::c_int) <
                   1250 as libc::c_int &&
               rcData[PITCH as libc::c_int as usize] as libc::c_int >
                   1750 as libc::c_int &&
               armingFlags as libc::c_int & ARMED as libc::c_int == 0 {
            cmsMenuOpen();
            rcDelayMs = 500 as libc::c_int as int16_t
            // Tends to overshoot if BUTTON_TIME
        }
    } else {
        //
        // Scan 'key' first
        //
        let mut key: uint8_t = 0 as libc::c_int as uint8_t;
        if rcData[THROTTLE as libc::c_int as usize] as libc::c_int >
               1250 as libc::c_int &&
               (rcData[THROTTLE as libc::c_int as usize] as libc::c_int) <
                   1750 as libc::c_int &&
               (rcData[YAW as libc::c_int as usize] as libc::c_int) <
                   1250 as libc::c_int &&
               rcData[PITCH as libc::c_int as usize] as libc::c_int >
                   1750 as libc::c_int &&
               armingFlags as libc::c_int & ARMED as libc::c_int == 0 {
            key = 6 as libc::c_int as uint8_t
        } else if rcData[PITCH as libc::c_int as usize] as libc::c_int >
                      1750 as libc::c_int {
            key = 1 as libc::c_int as uint8_t
        } else if (rcData[PITCH as libc::c_int as usize] as libc::c_int) <
                      1250 as libc::c_int {
            key = 2 as libc::c_int as uint8_t
        } else if (rcData[ROLL as libc::c_int as usize] as libc::c_int) <
                      1250 as libc::c_int {
            key = 3 as libc::c_int as uint8_t
        } else if rcData[ROLL as libc::c_int as usize] as libc::c_int >
                      1750 as libc::c_int {
            key = 4 as libc::c_int as uint8_t
        } else if rcData[YAW as libc::c_int as usize] as libc::c_int >
                      1750 as libc::c_int ||
                      (rcData[YAW as libc::c_int as usize] as libc::c_int) <
                          1250 as libc::c_int {
            key = 5 as libc::c_int as uint8_t
        }
        if key as libc::c_int == 0 as libc::c_int {
            // No 'key' pressed, reset repeat control
            holdCount = 1 as libc::c_int;
            repeatCount = 1 as libc::c_int;
            repeatBase = 0 as libc::c_int
        } else {
            // The 'key' is being pressed; keep counting
            holdCount += 1
        }
        if rcDelayMs as libc::c_int > 0 as libc::c_int {
            rcDelayMs =
                (rcDelayMs as
                     libc::c_uint).wrapping_sub(currentTimeMs.wrapping_sub(lastCalledMs))
                    as int16_t as int16_t
        } else if key != 0 {
            rcDelayMs =
                cmsHandleKeyWithRepeat(pCurrentDisplay, key, repeatCount) as
                    int16_t;
            // Key repeat effect is implemented in two phases.
            // First phldase is to decrease rcDelayMs reciprocal to hold time.
            // When rcDelayMs reached a certain limit (scheduling interval),
            // repeat rate will not raise anymore, so we call key handler
            // multiple times (repeatCount).
            //
            // XXX Caveat: Most constants are adjusted pragmatically.
            // XXX Rewrite this someday, so it uses actual hold time instead
            // of holdCount, which depends on the scheduling interval.
            if (key as libc::c_int == 3 as libc::c_int ||
                    key as libc::c_int == 4 as libc::c_int) &&
                   holdCount > 20 as libc::c_int {
                // Decrease rcDelayMs reciprocally
                rcDelayMs =
                    (rcDelayMs as libc::c_int /
                         (holdCount - 20 as libc::c_int)) as int16_t;
                // When we reach the scheduling limit,
                if rcDelayMs as libc::c_int <= 50 as libc::c_int {
                    // start calling handler multiple times.
                    if repeatBase == 0 as libc::c_int {
                        repeatBase = holdCount
                    }
                    repeatCount =
                        repeatCount +
                            (holdCount - repeatBase) / 5 as libc::c_int;
                    if repeatCount > 5 as libc::c_int {
                        repeatCount = 5 as libc::c_int
                    }
                }
            }
        }
        cmsDrawMenu(pCurrentDisplay, currentTimeUs);
        if currentTimeMs >
               lastCmsHeartBeatMs.wrapping_add(500 as libc::c_int as
                                                   libc::c_uint) {
            // Heart beat for external CMS display device @ 500msec
            // (Timeout @ 1000msec)
            displayHeartbeat(pCurrentDisplay);
            lastCmsHeartBeatMs = currentTimeMs
        }
    }
    // Some key (command), notably flash erase, takes too long to use the
    // currentTimeMs to be used as lastCalledMs (freezes CMS for a minute or so
    // if used).
    lastCalledMs = millis();
}
#[no_mangle]
pub unsafe extern "C" fn cmsHandler(mut currentTimeUs: timeUs_t) {
    if cmsDeviceCount < 0 as libc::c_int { return }
    static mut lastCalledUs: timeUs_t = 0 as libc::c_int as timeUs_t;
    if currentTimeUs >=
           lastCalledUs.wrapping_add(50000 as libc::c_int as libc::c_uint) {
        lastCalledUs = currentTimeUs;
        cmsUpdate(currentTimeUs);
    };
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
// Device management
// For main.c and scheduler
#[no_mangle]
pub unsafe extern "C" fn cmsInit() {
    cmsDeviceCount = 0 as libc::c_int;
    cmsCurrentDevice = -(1 as libc::c_int);
}
// CMS
