use core;
use libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn cmsMenuChange(pPort: *mut displayPort_t, ptr: *const libc::c_void)
     -> libc::c_long;
    #[no_mangle]
    static mut currentPidProfile: *mut pidProfile_s;
    #[no_mangle]
    fn getCurrentPidProfileIndex() -> uint8_t;
    #[no_mangle]
    fn changePidProfile(pidProfileIndex_0: uint8_t);
    #[no_mangle]
    fn getCurrentControlRateProfileIndex() -> uint8_t;
    #[no_mangle]
    fn changeControlRateProfile(profileIndex: uint8_t);
    #[no_mangle]
    static mut controlRateProfiles_SystemArray: [controlRateConfig_t; 6];
    #[no_mangle]
    static mut pidProfiles_SystemArray: [pidProfile_t; 3];
    #[no_mangle]
    fn pidInitConfig(pidProfile: *const pidProfile_t);
    #[no_mangle]
    static mut gyroConfig_System: gyroConfig_t;
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct CMS_Menu {
    pub onEnter: CMSMenuFuncPtr,
    pub onExit: CMSMenuOnExitPtr,
    pub entries: *mut OSD_Entry,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct OSD_UINT8_t {
    pub val: *mut uint8_t,
    pub min: uint8_t,
    pub max: uint8_t,
    pub step: uint8_t,
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
pub struct OSD_FLOAT_t {
    pub val: *mut uint8_t,
    pub min: uint8_t,
    pub max: uint8_t,
    pub step: uint8_t,
    pub multipler: uint16_t,
}
pub type gyroConfig_t = gyroConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct gyroConfig_s {
    pub gyro_align: uint8_t,
    pub gyroMovementCalibrationThreshold: uint8_t,
    pub gyro_sync_denom: uint8_t,
    pub gyro_hardware_lpf: uint8_t,
    pub gyro_32khz_hardware_lpf: uint8_t,
    pub gyro_high_fsr: uint8_t,
    pub gyro_use_32khz: uint8_t,
    pub gyro_to_use: uint8_t,
    pub gyro_lowpass_hz: uint16_t,
    pub gyro_lowpass2_hz: uint16_t,
    pub gyro_soft_notch_hz_1: uint16_t,
    pub gyro_soft_notch_cutoff_1: uint16_t,
    pub gyro_soft_notch_hz_2: uint16_t,
    pub gyro_soft_notch_cutoff_2: uint16_t,
    pub gyro_offset_yaw: int16_t,
    pub checkOverflow: uint8_t,
    pub gyro_lowpass_type: uint8_t,
    pub gyro_lowpass2_type: uint8_t,
    pub yaw_spin_recovery: uint8_t,
    pub yaw_spin_threshold: int16_t,
    pub gyroCalibrationDuration: uint16_t,
    pub dyn_notch_quality: uint8_t,
    pub dyn_notch_width_percent: uint8_t,
}
pub type controlRateConfig_t = controlRateConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct controlRateConfig_s {
    pub thrMid8: uint8_t,
    pub thrExpo8: uint8_t,
    pub rates_type: uint8_t,
    pub rcRates: [uint8_t; 3],
    pub rcExpo: [uint8_t; 3],
    pub rates: [uint8_t; 3],
    pub dynThrPID: uint8_t,
    pub tpa_breakpoint: uint16_t,
    pub throttle_limit_type: uint8_t,
    pub throttle_limit_percent: uint8_t,
}
pub const FD_YAW: C2RustUnnamed_0 = 2;
pub const FD_PITCH: C2RustUnnamed_0 = 1;
pub const FD_ROLL: C2RustUnnamed_0 = 0;
pub type pidProfile_t = pidProfile_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct pidProfile_s {
    pub yaw_lowpass_hz: uint16_t,
    pub dterm_lowpass_hz: uint16_t,
    pub dterm_notch_hz: uint16_t,
    pub dterm_notch_cutoff: uint16_t,
    pub pid: [pidf_t; 5],
    pub dterm_filter_type: uint8_t,
    pub itermWindupPointPercent: uint8_t,
    pub pidSumLimit: uint16_t,
    pub pidSumLimitYaw: uint16_t,
    pub pidAtMinThrottle: uint8_t,
    pub levelAngleLimit: uint8_t,
    pub horizon_tilt_effect: uint8_t,
    pub horizon_tilt_expert_mode: uint8_t,
    pub antiGravityMode: uint8_t,
    pub itermThrottleThreshold: uint16_t,
    pub itermAcceleratorGain: uint16_t,
    pub yawRateAccelLimit: uint16_t,
    pub rateAccelLimit: uint16_t,
    pub crash_dthreshold: uint16_t,
    pub crash_gthreshold: uint16_t,
    pub crash_setpoint_threshold: uint16_t,
    pub crash_time: uint16_t,
    pub crash_delay: uint16_t,
    pub crash_recovery_angle: uint8_t,
    pub crash_recovery_rate: uint8_t,
    pub vbatPidCompensation: uint8_t,
    pub feedForwardTransition: uint8_t,
    pub crash_limit_yaw: uint16_t,
    pub itermLimit: uint16_t,
    pub dterm_lowpass2_hz: uint16_t,
    pub crash_recovery: uint8_t,
    pub throttle_boost: uint8_t,
    pub throttle_boost_cutoff: uint8_t,
    pub iterm_rotation: uint8_t,
    pub smart_feedforward: uint8_t,
    pub iterm_relax_type: uint8_t,
    pub iterm_relax_cutoff: uint8_t,
    pub iterm_relax: uint8_t,
    pub acro_trainer_angle_limit: uint8_t,
    pub acro_trainer_debug_axis: uint8_t,
    pub acro_trainer_gain: uint8_t,
    pub acro_trainer_lookahead_ms: uint16_t,
    pub abs_control_gain: uint8_t,
    pub abs_control_limit: uint8_t,
    pub abs_control_error_limit: uint8_t,
}
pub type pidf_t = pidf_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct pidf_s {
    pub P: uint8_t,
    pub I: uint8_t,
    pub D: uint8_t,
    pub F: uint16_t,
}
pub const PID_LEVEL: C2RustUnnamed = 3;
pub const PID_YAW: C2RustUnnamed = 2;
pub const PID_PITCH: C2RustUnnamed = 1;
pub const PID_ROLL: C2RustUnnamed = 0;
pub type C2RustUnnamed = libc::c_uint;
pub const PID_ITEM_COUNT: C2RustUnnamed = 5;
pub const PID_MAG: C2RustUnnamed = 4;
// Breakpoint where TPA is activated
// Sets the throttle limiting type - off, scale or clip
// Sets the maximum pilot commanded throttle limit
// Limit to the accumulated error
// See http://en.wikipedia.org/wiki/Flight_dynamics
pub type C2RustUnnamed_0 = libc::c_uint;
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
#[inline]
unsafe extern "C" fn controlRateProfiles(mut _index: libc::c_int)
 -> *const controlRateConfig_t {
    return &mut *controlRateProfiles_SystemArray.as_mut_ptr().offset(_index as
                                                                         isize)
               as *mut controlRateConfig_t;
}
#[inline]
unsafe extern "C" fn controlRateProfilesMutable(mut _index: libc::c_int)
 -> *mut controlRateConfig_t {
    return &mut *controlRateProfiles_SystemArray.as_mut_ptr().offset(_index as
                                                                         isize)
               as *mut controlRateConfig_t;
}
#[inline]
unsafe extern "C" fn pidProfiles(mut _index: libc::c_int)
 -> *const pidProfile_t {
    return &mut *pidProfiles_SystemArray.as_mut_ptr().offset(_index as isize)
               as *mut pidProfile_t;
}
#[inline]
unsafe extern "C" fn pidProfilesMutable(mut _index: libc::c_int)
 -> *mut pidProfile_t {
    return &mut *pidProfiles_SystemArray.as_mut_ptr().offset(_index as isize)
               as *mut pidProfile_t;
}
#[inline]
unsafe extern "C" fn gyroConfig() -> *const gyroConfig_t {
    return &mut gyroConfig_System;
}
#[inline]
unsafe extern "C" fn gyroConfigMutable() -> *mut gyroConfig_t {
    return &mut gyroConfig_System;
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
// Menu contents for PID, RATES, RC preview, misc
// Should be part of the relevant .c file.
//
// PID
//
static mut tmpPidProfileIndex: uint8_t = 0;
static mut pidProfileIndex: uint8_t = 0;
static mut pidProfileIndexString: [libc::c_char; 3] = [32, 112, 0];
static mut tempPid: [[uint8_t; 3]; 3] = [[0; 3]; 3];
static mut tempPidF: [uint16_t; 3] = [0; 3];
static mut tmpRateProfileIndex: uint8_t = 0;
static mut rateProfileIndex: uint8_t = 0;
static mut rateProfileIndexString: [libc::c_char; 5] = [32, 112, 45, 114, 0];
static mut rateProfile: controlRateConfig_t =
    controlRateConfig_t{thrMid8: 0,
                        thrExpo8: 0,
                        rates_type: 0,
                        rcRates: [0; 3],
                        rcExpo: [0; 3],
                        rates: [0; 3],
                        dynThrPID: 0,
                        tpa_breakpoint: 0,
                        throttle_limit_type: 0,
                        throttle_limit_percent: 0,};
unsafe extern "C" fn cmsx_menuImu_onEnter() -> libc::c_long {
    pidProfileIndex = getCurrentPidProfileIndex();
    tmpPidProfileIndex = (pidProfileIndex as libc::c_int + 1i32) as uint8_t;
    rateProfileIndex = getCurrentControlRateProfileIndex();
    tmpRateProfileIndex = (rateProfileIndex as libc::c_int + 1i32) as uint8_t;
    return 0i32 as libc::c_long;
}
unsafe extern "C" fn cmsx_menuImu_onExit(mut self_0: *const OSD_Entry)
 -> libc::c_long {
    changePidProfile(pidProfileIndex);
    changeControlRateProfile(rateProfileIndex);
    return 0i32 as libc::c_long;
}
unsafe extern "C" fn cmsx_profileIndexOnChange(mut displayPort:
                                                   *mut displayPort_t,
                                               mut ptr: *const libc::c_void)
 -> libc::c_long {
    pidProfileIndex = (tmpPidProfileIndex as libc::c_int - 1i32) as uint8_t;
    changePidProfile(pidProfileIndex);
    return 0i32 as libc::c_long;
}
unsafe extern "C" fn cmsx_rateProfileIndexOnChange(mut displayPort:
                                                       *mut displayPort_t,
                                                   mut ptr:
                                                       *const libc::c_void)
 -> libc::c_long {
    rateProfileIndex = (tmpRateProfileIndex as libc::c_int - 1i32) as uint8_t;
    changeControlRateProfile(rateProfileIndex);
    return 0i32 as libc::c_long;
}
unsafe extern "C" fn cmsx_PidRead() -> libc::c_long {
    let mut pidProfile: *const pidProfile_t =
        pidProfiles(pidProfileIndex as libc::c_int);
    let mut i: uint8_t = 0i32 as uint8_t;
    while (i as libc::c_int) < 3i32 {
        tempPid[i as usize][0] = (*pidProfile).pid[i as usize].P;
        tempPid[i as usize][1] = (*pidProfile).pid[i as usize].I;
        tempPid[i as usize][2] = (*pidProfile).pid[i as usize].D;
        tempPidF[i as usize] = (*pidProfile).pid[i as usize].F;
        i = i.wrapping_add(1)
    }
    return 0i32 as libc::c_long;
}
unsafe extern "C" fn cmsx_PidOnEnter() -> libc::c_long {
    pidProfileIndexString[1] =
        ('0' as i32 + tmpPidProfileIndex as libc::c_int) as libc::c_char;
    cmsx_PidRead();
    return 0i32 as libc::c_long;
}
unsafe extern "C" fn cmsx_PidWriteback(mut self_0: *const OSD_Entry)
 -> libc::c_long {
    let mut pidProfile: *mut pidProfile_t = currentPidProfile;
    let mut i: uint8_t = 0i32 as uint8_t;
    while (i as libc::c_int) < 3i32 {
        (*pidProfile).pid[i as usize].P = tempPid[i as usize][0];
        (*pidProfile).pid[i as usize].I = tempPid[i as usize][1];
        (*pidProfile).pid[i as usize].D = tempPid[i as usize][2];
        (*pidProfile).pid[i as usize].F = tempPidF[i as usize];
        i = i.wrapping_add(1)
    }
    pidInitConfig(currentPidProfile);
    return 0i32 as libc::c_long;
}
/*
static OSD_Entry cmsx_menuPidEntries[] =
{
    { "-- PID --", OME_Label, NULL, pidProfileIndexString, 0},

    { "ROLL  P", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][0],  0, 200, 1 }, 0 },
    { "ROLL  I", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][1],  0, 200, 1 }, 0 },
    { "ROLL  D", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][2],  0, 200, 1 }, 0 },
    { "ROLL  F", OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidF[PID_ROLL],  0, 2000, 1 }, 0 },

    { "PITCH P", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][0], 0, 200, 1 }, 0 },
    { "PITCH I", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][1], 0, 200, 1 }, 0 },
    { "PITCH D", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][2], 0, 200, 1 }, 0 },
    { "PITCH F", OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidF[PID_PITCH], 0, 2000, 1 }, 0 },

    { "YAW   P", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][0],   0, 200, 1 }, 0 },
    { "YAW   I", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][1],   0, 200, 1 }, 0 },
    { "YAW   D", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][2],   0, 200, 1 }, 0 },
    { "YAW   F", OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidF[PID_YAW],   0, 2000, 1 }, 0 },

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};
*/
// Initialized in run_static_initializers
static mut rollp_data: OSD_UINT8_t =
    OSD_UINT8_t{val: 0 as *const uint8_t as *mut uint8_t,
                min: 0,
                max: 0,
                step: 0,};
// Initialized in run_static_initializers
static mut rolli_data: OSD_UINT8_t =
    OSD_UINT8_t{val: 0 as *const uint8_t as *mut uint8_t,
                min: 0,
                max: 0,
                step: 0,};
// Initialized in run_static_initializers
static mut rolld_data: OSD_UINT8_t =
    OSD_UINT8_t{val: 0 as *const uint8_t as *mut uint8_t,
                min: 0,
                max: 0,
                step: 0,};
// Initialized in run_static_initializers
static mut rollf_data: OSD_UINT16_t =
    OSD_UINT16_t{val: 0 as *const uint16_t as *mut uint16_t,
                 min: 0,
                 max: 0,
                 step: 0,};
// Initialized in run_static_initializers
static mut pitchp_data: OSD_UINT8_t =
    OSD_UINT8_t{val: 0 as *const uint8_t as *mut uint8_t,
                min: 0,
                max: 0,
                step: 0,};
// Initialized in run_static_initializers
static mut pitchi_data: OSD_UINT8_t =
    OSD_UINT8_t{val: 0 as *const uint8_t as *mut uint8_t,
                min: 0,
                max: 0,
                step: 0,};
// Initialized in run_static_initializers
static mut pitchd_data: OSD_UINT8_t =
    OSD_UINT8_t{val: 0 as *const uint8_t as *mut uint8_t,
                min: 0,
                max: 0,
                step: 0,};
// Initialized in run_static_initializers
static mut pitchf_data: OSD_UINT16_t =
    OSD_UINT16_t{val: 0 as *const uint16_t as *mut uint16_t,
                 min: 0,
                 max: 0,
                 step: 0,};
// Initialized in run_static_initializers
static mut yawp_data: OSD_UINT8_t =
    OSD_UINT8_t{val: 0 as *const uint8_t as *mut uint8_t,
                min: 0,
                max: 0,
                step: 0,};
// Initialized in run_static_initializers
static mut yawi_data: OSD_UINT8_t =
    OSD_UINT8_t{val: 0 as *const uint8_t as *mut uint8_t,
                min: 0,
                max: 0,
                step: 0,};
// Initialized in run_static_initializers
static mut yawd_data: OSD_UINT8_t =
    OSD_UINT8_t{val: 0 as *const uint8_t as *mut uint8_t,
                min: 0,
                max: 0,
                step: 0,};
// Initialized in run_static_initializers
static mut yawf_data: OSD_UINT16_t =
    OSD_UINT16_t{val: 0 as *const uint16_t as *mut uint16_t,
                 min: 0,
                 max: 0,
                 step: 0,};
static mut cmsx_menuPidEntries: [OSD_Entry; 15] =
    unsafe {
        [{
             let mut init =
                 OSD_Entry{text:
                               b"-- PID --\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Label,
                           func: None,
                           data:
                               pidProfileIndexString.as_ptr() as *mut _ as
                                   *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"ROLL  P\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT8,
                           func: None,
                           data:
                               &rollp_data as *const OSD_UINT8_t as
                                   *mut OSD_UINT8_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"ROLL  I\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT8,
                           func: None,
                           data:
                               &rolli_data as *const OSD_UINT8_t as
                                   *mut OSD_UINT8_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"ROLL  D\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT8,
                           func: None,
                           data:
                               &rolld_data as *const OSD_UINT8_t as
                                   *mut OSD_UINT8_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"ROLL  F\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT16,
                           func: None,
                           data:
                               &rollf_data as *const OSD_UINT16_t as
                                   *mut OSD_UINT16_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"PITCH P\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT8,
                           func: None,
                           data:
                               &pitchp_data as *const OSD_UINT8_t as
                                   *mut OSD_UINT8_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"PITCH I\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT8,
                           func: None,
                           data:
                               &pitchi_data as *const OSD_UINT8_t as
                                   *mut OSD_UINT8_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"PITCH D\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT8,
                           func: None,
                           data:
                               &pitchd_data as *const OSD_UINT8_t as
                                   *mut OSD_UINT8_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"PITCH F\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT16,
                           func: None,
                           data:
                               &pitchf_data as *const OSD_UINT16_t as
                                   *mut OSD_UINT16_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"YAW   P\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT8,
                           func: None,
                           data:
                               &yawp_data as *const OSD_UINT8_t as
                                   *mut OSD_UINT8_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"YAW   I\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT8,
                           func: None,
                           data:
                               &yawi_data as *const OSD_UINT8_t as
                                   *mut OSD_UINT8_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"YAW   D\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT8,
                           func: None,
                           data:
                               &yawd_data as *const OSD_UINT8_t as
                                   *mut OSD_UINT8_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"YAW   F\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT16,
                           func: None,
                           data:
                               &yawf_data as *const OSD_UINT16_t as
                                   *mut OSD_UINT16_t as *mut libc::c_void,
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
static mut cmsx_menuPid: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter:
                             Some(cmsx_PidOnEnter as
                                      unsafe extern "C" fn() -> libc::c_long),
                         onExit:
                             Some(cmsx_PidWriteback as
                                      unsafe extern "C" fn(_:
                                                               *const OSD_Entry)
                                          -> libc::c_long),
                         entries: cmsx_menuPidEntries.as_ptr() as *mut _,};
            init
        }
    };
//
// Rate & Expo
//
unsafe extern "C" fn cmsx_RateProfileRead() -> libc::c_long {
    memcpy(&mut rateProfile as *mut controlRateConfig_t as *mut libc::c_void,
           controlRateProfiles(rateProfileIndex as libc::c_int) as
               *const libc::c_void,
           ::core::mem::size_of::<controlRateConfig_t>() as libc::c_ulong);
    return 0i32 as libc::c_long;
}
unsafe extern "C" fn cmsx_RateProfileWriteback(mut self_0: *const OSD_Entry)
 -> libc::c_long {
    memcpy(controlRateProfilesMutable(rateProfileIndex as libc::c_int) as
               *mut libc::c_void,
           &mut rateProfile as *mut controlRateConfig_t as
               *const libc::c_void,
           ::core::mem::size_of::<controlRateConfig_t>() as libc::c_ulong);
    return 0i32 as libc::c_long;
}
unsafe extern "C" fn cmsx_RateProfileOnEnter() -> libc::c_long {
    rateProfileIndexString[1] =
        ('0' as i32 + tmpPidProfileIndex as libc::c_int) as libc::c_char;
    rateProfileIndexString[3] =
        ('0' as i32 + tmpRateProfileIndex as libc::c_int) as libc::c_char;
    cmsx_RateProfileRead();
    return 0i32 as libc::c_long;
}
// Initialized in run_static_initializers
static mut rc_r_rate_data: OSD_FLOAT_t =
    OSD_FLOAT_t{val: 0 as *const uint8_t as *mut uint8_t,
                min: 0,
                max: 0,
                step: 0,
                multipler: 0,};
// Initialized in run_static_initializers
static mut rc_p_rate_data: OSD_FLOAT_t =
    OSD_FLOAT_t{val: 0 as *const uint8_t as *mut uint8_t,
                min: 0,
                max: 0,
                step: 0,
                multipler: 0,};
// Initialized in run_static_initializers
static mut rc_y_rate_data: OSD_FLOAT_t =
    OSD_FLOAT_t{val: 0 as *const uint8_t as *mut uint8_t,
                min: 0,
                max: 0,
                step: 0,
                multipler: 0,};
// Initialized in run_static_initializers
static mut roll_super_data: OSD_FLOAT_t =
    OSD_FLOAT_t{val: 0 as *const uint8_t as *mut uint8_t,
                min: 0,
                max: 0,
                step: 0,
                multipler: 0,};
// Initialized in run_static_initializers
static mut pitch_super_data: OSD_FLOAT_t =
    OSD_FLOAT_t{val: 0 as *const uint8_t as *mut uint8_t,
                min: 0,
                max: 0,
                step: 0,
                multipler: 0,};
// Initialized in run_static_initializers
static mut yaw_super_data: OSD_FLOAT_t =
    OSD_FLOAT_t{val: 0 as *const uint8_t as *mut uint8_t,
                min: 0,
                max: 0,
                step: 0,
                multipler: 0,};
// Initialized in run_static_initializers
static mut rc_r_expo_data: OSD_FLOAT_t =
    OSD_FLOAT_t{val: 0 as *const uint8_t as *mut uint8_t,
                min: 0,
                max: 0,
                step: 0,
                multipler: 0,};
// Initialized in run_static_initializers
static mut rc_p_expo_data: OSD_FLOAT_t =
    OSD_FLOAT_t{val: 0 as *const uint8_t as *mut uint8_t,
                min: 0,
                max: 0,
                step: 0,
                multipler: 0,};
// Initialized in run_static_initializers
static mut rc_y_expo_data: OSD_FLOAT_t =
    OSD_FLOAT_t{val: 0 as *const uint8_t as *mut uint8_t,
                min: 0,
                max: 0,
                step: 0,
                multipler: 0,};
// Initialized in run_static_initializers
static mut thr_mid_data: OSD_UINT8_t =
    OSD_UINT8_t{val: 0 as *const uint8_t as *mut uint8_t,
                min: 0,
                max: 0,
                step: 0,};
// Initialized in run_static_initializers
static mut thr_expo_data: OSD_UINT8_t =
    OSD_UINT8_t{val: 0 as *const uint8_t as *mut uint8_t,
                min: 0,
                max: 0,
                step: 0,};
// Initialized in run_static_initializers
static mut thrpid_att_data: OSD_FLOAT_t =
    OSD_FLOAT_t{val: 0 as *const uint8_t as *mut uint8_t,
                min: 0,
                max: 0,
                step: 0,
                multipler: 0,};
// Initialized in run_static_initializers
static mut tpa_brkpt_data: OSD_UINT16_t =
    OSD_UINT16_t{val: 0 as *const uint16_t as *mut uint16_t,
                 min: 0,
                 max: 0,
                 step: 0,};
static mut cmsx_menuRateProfileEntries: [OSD_Entry; 16] =
    unsafe {
        [{
             let mut init =
                 OSD_Entry{text:
                               b"-- RATE --\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Label,
                           func: None,
                           data:
                               rateProfileIndexString.as_ptr() as *mut _ as
                                   *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"RC R RATE\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_FLOAT,
                           func: None,
                           data:
                               &rc_r_rate_data as *const OSD_FLOAT_t as
                                   *mut OSD_FLOAT_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"RC P RATE\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_FLOAT,
                           func: None,
                           data:
                               &rc_p_rate_data as *const OSD_FLOAT_t as
                                   *mut OSD_FLOAT_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"RC Y RATE\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_FLOAT,
                           func: None,
                           data:
                               &rc_y_rate_data as *const OSD_FLOAT_t as
                                   *mut OSD_FLOAT_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"ROLL SUPER\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_FLOAT,
                           func: None,
                           data:
                               &roll_super_data as *const OSD_FLOAT_t as
                                   *mut OSD_FLOAT_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"PITCH SUPER\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_FLOAT,
                           func: None,
                           data:
                               &pitch_super_data as *const OSD_FLOAT_t as
                                   *mut OSD_FLOAT_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"YAW SUPER\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_FLOAT,
                           func: None,
                           data:
                               &yaw_super_data as *const OSD_FLOAT_t as
                                   *mut OSD_FLOAT_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"RC R EXPO\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_FLOAT,
                           func: None,
                           data:
                               &rc_r_expo_data as *const OSD_FLOAT_t as
                                   *mut OSD_FLOAT_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"RC P EXPO\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_FLOAT,
                           func: None,
                           data:
                               &rc_p_expo_data as *const OSD_FLOAT_t as
                                   *mut OSD_FLOAT_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"RC Y EXPO\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_FLOAT,
                           func: None,
                           data:
                               &rc_y_expo_data as *const OSD_FLOAT_t as
                                   *mut OSD_FLOAT_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"THR MID\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT8,
                           func: None,
                           data:
                               &thr_mid_data as *const OSD_UINT8_t as
                                   *mut OSD_UINT8_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"THR EXPO\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT8,
                           func: None,
                           data:
                               &thr_expo_data as *const OSD_UINT8_t as
                                   *mut OSD_UINT8_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"THRPID ATT\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_FLOAT,
                           func: None,
                           data:
                               &thrpid_att_data as *const OSD_FLOAT_t as
                                   *mut OSD_FLOAT_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"TPA BRKPT\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT16,
                           func: None,
                           data:
                               &tpa_brkpt_data as *const OSD_UINT16_t as
                                   *mut OSD_UINT16_t as *mut libc::c_void,
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
static mut cmsx_menuRateProfile: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter:
                             Some(cmsx_RateProfileOnEnter as
                                      unsafe extern "C" fn() -> libc::c_long),
                         onExit:
                             Some(cmsx_RateProfileWriteback as
                                      unsafe extern "C" fn(_:
                                                               *const OSD_Entry)
                                          -> libc::c_long),
                         entries:
                             cmsx_menuRateProfileEntries.as_ptr() as *mut _,};
            init
        }
    };
static mut cmsx_feedForwardTransition: uint8_t = 0;
static mut cmsx_angleStrength: uint8_t = 0;
static mut cmsx_horizonStrength: uint8_t = 0;
static mut cmsx_horizonTransition: uint8_t = 0;
static mut cmsx_throttleBoost: uint8_t = 0;
static mut cmsx_itermAcceleratorGain: uint16_t = 0;
static mut cmsx_itermThrottleThreshold: uint16_t = 0;
unsafe extern "C" fn cmsx_profileOtherOnEnter() -> libc::c_long {
    pidProfileIndexString[1] =
        ('0' as i32 + tmpPidProfileIndex as libc::c_int) as libc::c_char;
    let mut pidProfile: *const pidProfile_t =
        pidProfiles(pidProfileIndex as libc::c_int);
    cmsx_feedForwardTransition = (*pidProfile).feedForwardTransition;
    cmsx_angleStrength =
        (*pidProfile).pid[PID_LEVEL as libc::c_int as usize].P;
    cmsx_horizonStrength =
        (*pidProfile).pid[PID_LEVEL as libc::c_int as usize].I;
    cmsx_horizonTransition =
        (*pidProfile).pid[PID_LEVEL as libc::c_int as usize].D;
    cmsx_itermAcceleratorGain = (*pidProfile).itermAcceleratorGain;
    cmsx_itermThrottleThreshold = (*pidProfile).itermThrottleThreshold;
    cmsx_throttleBoost = (*pidProfile).throttle_boost;
    return 0i32 as libc::c_long;
}
unsafe extern "C" fn cmsx_profileOtherOnExit(mut self_0: *const OSD_Entry)
 -> libc::c_long {
    let mut pidProfile: *mut pidProfile_t =
        pidProfilesMutable(pidProfileIndex as libc::c_int);
    (*pidProfile).feedForwardTransition = cmsx_feedForwardTransition;
    pidInitConfig(currentPidProfile);
    (*pidProfile).pid[PID_LEVEL as libc::c_int as usize].P =
        cmsx_angleStrength;
    (*pidProfile).pid[PID_LEVEL as libc::c_int as usize].I =
        cmsx_horizonStrength;
    (*pidProfile).pid[PID_LEVEL as libc::c_int as usize].D =
        cmsx_horizonTransition;
    (*pidProfile).itermAcceleratorGain = cmsx_itermAcceleratorGain;
    (*pidProfile).itermThrottleThreshold = cmsx_itermThrottleThreshold;
    (*pidProfile).throttle_boost = cmsx_throttleBoost;
    return 0i32 as libc::c_long;
}
static mut ff_trans_data: OSD_FLOAT_t =
    unsafe {
        {
            let mut init =
                OSD_FLOAT_t{val:
                                &cmsx_feedForwardTransition as *const uint8_t
                                    as *mut uint8_t,
                            min: 0i32 as uint8_t,
                            max: 100i32 as uint8_t,
                            step: 1i32 as uint8_t,
                            multipler: 10i32 as uint16_t,};
            init
        }
    };
static mut angle_str_data: OSD_UINT8_t =
    unsafe {
        {
            let mut init =
                OSD_UINT8_t{val:
                                &cmsx_angleStrength as *const uint8_t as
                                    *mut uint8_t,
                            min: 0i32 as uint8_t,
                            max: 200i32 as uint8_t,
                            step: 1i32 as uint8_t,};
            init
        }
    };
static mut horzn_str_data: OSD_UINT8_t =
    unsafe {
        {
            let mut init =
                OSD_UINT8_t{val:
                                &cmsx_horizonStrength as *const uint8_t as
                                    *mut uint8_t,
                            min: 0i32 as uint8_t,
                            max: 200i32 as uint8_t,
                            step: 1i32 as uint8_t,};
            init
        }
    };
static mut horzn_trs_data: OSD_UINT8_t =
    unsafe {
        {
            let mut init =
                OSD_UINT8_t{val:
                                &cmsx_horizonTransition as *const uint8_t as
                                    *mut uint8_t,
                            min: 0i32 as uint8_t,
                            max: 200i32 as uint8_t,
                            step: 1i32 as uint8_t,};
            init
        }
    };
static mut ag_gain_data: OSD_UINT16_t =
    unsafe {
        {
            let mut init =
                OSD_UINT16_t{val:
                                 &cmsx_itermAcceleratorGain as *const uint16_t
                                     as *mut uint16_t,
                             min: 1000i32 as uint16_t,
                             max: 30000i32 as uint16_t,
                             step: 10i32 as uint16_t,};
            init
        }
    };
static mut ag_thr_data: OSD_UINT16_t =
    unsafe {
        {
            let mut init =
                OSD_UINT16_t{val:
                                 &cmsx_itermThrottleThreshold as
                                     *const uint16_t as *mut uint16_t,
                             min: 20i32 as uint16_t,
                             max: 1000i32 as uint16_t,
                             step: 1i32 as uint16_t,};
            init
        }
    };
static mut thr_boost_data: OSD_UINT8_t =
    unsafe {
        {
            let mut init =
                OSD_UINT8_t{val:
                                &cmsx_throttleBoost as *const uint8_t as
                                    *mut uint8_t,
                            min: 0i32 as uint8_t,
                            max: 100i32 as uint8_t,
                            step: 1i32 as uint8_t,};
            init
        }
    };
static mut cmsx_menuProfileOtherEntries: [OSD_Entry; 10] =
    unsafe {
        [{
             let mut init =
                 OSD_Entry{text:
                               b"-- OTHER PP --\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Label,
                           func: None,
                           data:
                               pidProfileIndexString.as_ptr() as *mut _ as
                                   *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"FF TRANS\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_FLOAT,
                           func: None,
                           data:
                               &ff_trans_data as *const OSD_FLOAT_t as
                                   *mut OSD_FLOAT_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"ANGLE STR\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT8,
                           func: None,
                           data:
                               &angle_str_data as *const OSD_UINT8_t as
                                   *mut OSD_UINT8_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"HORZN STR\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT8,
                           func: None,
                           data:
                               &horzn_str_data as *const OSD_UINT8_t as
                                   *mut OSD_UINT8_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"HORZN TRS\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT8,
                           func: None,
                           data:
                               &horzn_trs_data as *const OSD_UINT8_t as
                                   *mut OSD_UINT8_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"AG GAIN\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT16,
                           func: None,
                           data:
                               &ag_gain_data as *const OSD_UINT16_t as
                                   *mut OSD_UINT16_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"AG THR\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT16,
                           func: None,
                           data:
                               &ag_thr_data as *const OSD_UINT16_t as
                                   *mut OSD_UINT16_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"THR BOOST\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT8,
                           func: None,
                           data:
                               &thr_boost_data as *const OSD_UINT8_t as
                                   *mut OSD_UINT8_t as *mut libc::c_void,
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
static mut cmsx_menuProfileOther: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter:
                             Some(cmsx_profileOtherOnEnter as
                                      unsafe extern "C" fn() -> libc::c_long),
                         onExit:
                             Some(cmsx_profileOtherOnExit as
                                      unsafe extern "C" fn(_:
                                                               *const OSD_Entry)
                                          -> libc::c_long),
                         entries:
                             cmsx_menuProfileOtherEntries.as_ptr() as
                                 *mut _,};
            init
        }
    };
static mut gyroConfig_gyro_lowpass_hz: uint16_t = 0;
static mut gyroConfig_gyro_lowpass2_hz: uint16_t = 0;
static mut gyroConfig_gyro_soft_notch_hz_1: uint16_t = 0;
static mut gyroConfig_gyro_soft_notch_cutoff_1: uint16_t = 0;
static mut gyroConfig_gyro_soft_notch_hz_2: uint16_t = 0;
static mut gyroConfig_gyro_soft_notch_cutoff_2: uint16_t = 0;
unsafe extern "C" fn cmsx_menuGyro_onEnter() -> libc::c_long {
    gyroConfig_gyro_lowpass_hz = (*gyroConfig()).gyro_lowpass_hz;
    gyroConfig_gyro_lowpass2_hz = (*gyroConfig()).gyro_lowpass2_hz;
    gyroConfig_gyro_soft_notch_hz_1 = (*gyroConfig()).gyro_soft_notch_hz_1;
    gyroConfig_gyro_soft_notch_cutoff_1 =
        (*gyroConfig()).gyro_soft_notch_cutoff_1;
    gyroConfig_gyro_soft_notch_hz_2 = (*gyroConfig()).gyro_soft_notch_hz_2;
    gyroConfig_gyro_soft_notch_cutoff_2 =
        (*gyroConfig()).gyro_soft_notch_cutoff_2;
    return 0i32 as libc::c_long;
}
unsafe extern "C" fn cmsx_menuGyro_onExit(mut self_0: *const OSD_Entry)
 -> libc::c_long {
    (*gyroConfigMutable()).gyro_lowpass_hz = gyroConfig_gyro_lowpass_hz;
    (*gyroConfigMutable()).gyro_lowpass2_hz = gyroConfig_gyro_lowpass2_hz;
    (*gyroConfigMutable()).gyro_soft_notch_hz_1 =
        gyroConfig_gyro_soft_notch_hz_1;
    (*gyroConfigMutable()).gyro_soft_notch_cutoff_1 =
        gyroConfig_gyro_soft_notch_cutoff_1;
    (*gyroConfigMutable()).gyro_soft_notch_hz_2 =
        gyroConfig_gyro_soft_notch_hz_2;
    (*gyroConfigMutable()).gyro_soft_notch_cutoff_2 =
        gyroConfig_gyro_soft_notch_cutoff_2;
    return 0i32 as libc::c_long;
}
static mut gyro_lpf_data: OSD_UINT16_t =
    unsafe {
        {
            let mut init =
                OSD_UINT16_t{val:
                                 &gyroConfig_gyro_lowpass_hz as
                                     *const uint16_t as *mut uint16_t,
                             min: 0i32 as uint16_t,
                             max: 16000i32 as uint16_t,
                             step: 1i32 as uint16_t,};
            init
        }
    };
static mut gyro_lpf2_data: OSD_UINT16_t =
    unsafe {
        {
            let mut init =
                OSD_UINT16_t{val:
                                 &gyroConfig_gyro_lowpass2_hz as
                                     *const uint16_t as *mut uint16_t,
                             min: 0i32 as uint16_t,
                             max: 16000i32 as uint16_t,
                             step: 1i32 as uint16_t,};
            init
        }
    };
static mut gyro_nf1_data: OSD_UINT16_t =
    unsafe {
        {
            let mut init =
                OSD_UINT16_t{val:
                                 &gyroConfig_gyro_soft_notch_hz_1 as
                                     *const uint16_t as *mut uint16_t,
                             min: 0i32 as uint16_t,
                             max: 500i32 as uint16_t,
                             step: 1i32 as uint16_t,};
            init
        }
    };
static mut gyro_nf1c_data: OSD_UINT16_t =
    unsafe {
        {
            let mut init =
                OSD_UINT16_t{val:
                                 &gyroConfig_gyro_soft_notch_cutoff_1 as
                                     *const uint16_t as *mut uint16_t,
                             min: 0i32 as uint16_t,
                             max: 500i32 as uint16_t,
                             step: 1i32 as uint16_t,};
            init
        }
    };
static mut gyro_nf2_data: OSD_UINT16_t =
    unsafe {
        {
            let mut init =
                OSD_UINT16_t{val:
                                 &gyroConfig_gyro_soft_notch_hz_2 as
                                     *const uint16_t as *mut uint16_t,
                             min: 0i32 as uint16_t,
                             max: 500i32 as uint16_t,
                             step: 1i32 as uint16_t,};
            init
        }
    };
static mut gyro_nf2c_data: OSD_UINT16_t =
    unsafe {
        {
            let mut init =
                OSD_UINT16_t{val:
                                 &gyroConfig_gyro_soft_notch_cutoff_2 as
                                     *const uint16_t as *mut uint16_t,
                             min: 0i32 as uint16_t,
                             max: 500i32 as uint16_t,
                             step: 1i32 as uint16_t,};
            init
        }
    };
static mut cmsx_menuFilterGlobalEntries: [OSD_Entry; 9] =
    unsafe {
        [{
             let mut init =
                 OSD_Entry{text:
                               b"-- FILTER GLB  --\x00" as *const u8 as
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
                               b"GYRO LPF\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT16,
                           func: None,
                           data:
                               &gyro_lpf_data as *const OSD_UINT16_t as
                                   *mut OSD_UINT16_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"GYRO LPF2\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT16,
                           func: None,
                           data:
                               &gyro_lpf2_data as *const OSD_UINT16_t as
                                   *mut OSD_UINT16_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"GYRO NF1\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT16,
                           func: None,
                           data:
                               &gyro_nf1_data as *const OSD_UINT16_t as
                                   *mut OSD_UINT16_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"GYRO NF1C\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT16,
                           func: None,
                           data:
                               &gyro_nf1c_data as *const OSD_UINT16_t as
                                   *mut OSD_UINT16_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"GYRO NF2\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT16,
                           func: None,
                           data:
                               &gyro_nf2_data as *const OSD_UINT16_t as
                                   *mut OSD_UINT16_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"GYRO NF2C\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT16,
                           func: None,
                           data:
                               &gyro_nf2c_data as *const OSD_UINT16_t as
                                   *mut OSD_UINT16_t as *mut libc::c_void,
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
static mut cmsx_menuFilterGlobal: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter:
                             Some(cmsx_menuGyro_onEnter as
                                      unsafe extern "C" fn() -> libc::c_long),
                         onExit:
                             Some(cmsx_menuGyro_onExit as
                                      unsafe extern "C" fn(_:
                                                               *const OSD_Entry)
                                          -> libc::c_long),
                         entries:
                             cmsx_menuFilterGlobalEntries.as_ptr() as
                                 *mut _,};
            init
        }
    };
static mut cmsx_dterm_lowpass_hz: uint16_t = 0;
static mut cmsx_dterm_lowpass2_hz: uint16_t = 0;
static mut cmsx_dterm_notch_hz: uint16_t = 0;
static mut cmsx_dterm_notch_cutoff: uint16_t = 0;
static mut cmsx_yaw_lowpass_hz: uint16_t = 0;
unsafe extern "C" fn cmsx_FilterPerProfileRead() -> libc::c_long {
    let mut pidProfile: *const pidProfile_t =
        pidProfiles(pidProfileIndex as libc::c_int);
    cmsx_dterm_lowpass_hz = (*pidProfile).dterm_lowpass_hz;
    cmsx_dterm_lowpass2_hz = (*pidProfile).dterm_lowpass2_hz;
    cmsx_dterm_notch_hz = (*pidProfile).dterm_notch_hz;
    cmsx_dterm_notch_cutoff = (*pidProfile).dterm_notch_cutoff;
    cmsx_yaw_lowpass_hz = (*pidProfile).yaw_lowpass_hz;
    return 0i32 as libc::c_long;
}
unsafe extern "C" fn cmsx_FilterPerProfileWriteback(mut self_0:
                                                        *const OSD_Entry)
 -> libc::c_long {
    let mut pidProfile: *mut pidProfile_t = currentPidProfile;
    (*pidProfile).dterm_lowpass_hz = cmsx_dterm_lowpass_hz;
    (*pidProfile).dterm_lowpass2_hz = cmsx_dterm_lowpass2_hz;
    (*pidProfile).dterm_notch_hz = cmsx_dterm_notch_hz;
    (*pidProfile).dterm_notch_cutoff = cmsx_dterm_notch_cutoff;
    (*pidProfile).yaw_lowpass_hz = cmsx_yaw_lowpass_hz;
    return 0i32 as libc::c_long;
}
static mut dterm_lpf_data: OSD_UINT16_t =
    unsafe {
        {
            let mut init =
                OSD_UINT16_t{val:
                                 &cmsx_dterm_lowpass_hz as *const uint16_t as
                                     *mut uint16_t,
                             min: 0i32 as uint16_t,
                             max: 500i32 as uint16_t,
                             step: 1i32 as uint16_t,};
            init
        }
    };
static mut dterm_lpf2_data: OSD_UINT16_t =
    unsafe {
        {
            let mut init =
                OSD_UINT16_t{val:
                                 &cmsx_dterm_lowpass2_hz as *const uint16_t as
                                     *mut uint16_t,
                             min: 0i32 as uint16_t,
                             max: 500i32 as uint16_t,
                             step: 1i32 as uint16_t,};
            init
        }
    };
static mut dterm_nf_data: OSD_UINT16_t =
    unsafe {
        {
            let mut init =
                OSD_UINT16_t{val:
                                 &cmsx_dterm_notch_hz as *const uint16_t as
                                     *mut uint16_t,
                             min: 0i32 as uint16_t,
                             max: 500i32 as uint16_t,
                             step: 1i32 as uint16_t,};
            init
        }
    };
static mut dterm_nfco_data: OSD_UINT16_t =
    unsafe {
        {
            let mut init =
                OSD_UINT16_t{val:
                                 &cmsx_dterm_notch_cutoff as *const uint16_t
                                     as *mut uint16_t,
                             min: 0i32 as uint16_t,
                             max: 500i32 as uint16_t,
                             step: 1i32 as uint16_t,};
            init
        }
    };
static mut yaw_lpf_data: OSD_UINT16_t =
    unsafe {
        {
            let mut init =
                OSD_UINT16_t{val:
                                 &cmsx_yaw_lowpass_hz as *const uint16_t as
                                     *mut uint16_t,
                             min: 0i32 as uint16_t,
                             max: 500i32 as uint16_t,
                             step: 1i32 as uint16_t,};
            init
        }
    };
static mut cmsx_menuFilterPerProfileEntries: [OSD_Entry; 8] =
    unsafe {
        [{
             let mut init =
                 OSD_Entry{text:
                               b"-- FILTER PP  --\x00" as *const u8 as
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
                               b"DTERM LPF\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT16,
                           func: None,
                           data:
                               &dterm_lpf_data as *const OSD_UINT16_t as
                                   *mut OSD_UINT16_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"DTERM LPF2\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT16,
                           func: None,
                           data:
                               &dterm_lpf2_data as *const OSD_UINT16_t as
                                   *mut OSD_UINT16_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"DTERM NF\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT16,
                           func: None,
                           data:
                               &dterm_nf_data as *const OSD_UINT16_t as
                                   *mut OSD_UINT16_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"DTERM NFCO\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT16,
                           func: None,
                           data:
                               &dterm_nfco_data as *const OSD_UINT16_t as
                                   *mut OSD_UINT16_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"YAW LPF\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT16,
                           func: None,
                           data:
                               &yaw_lpf_data as *const OSD_UINT16_t as
                                   *mut OSD_UINT16_t as *mut libc::c_void,
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
static mut cmsx_menuFilterPerProfile: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter:
                             Some(cmsx_FilterPerProfileRead as
                                      unsafe extern "C" fn() -> libc::c_long),
                         onExit:
                             Some(cmsx_FilterPerProfileWriteback as
                                      unsafe extern "C" fn(_:
                                                               *const OSD_Entry)
                                          -> libc::c_long),
                         entries:
                             cmsx_menuFilterPerProfileEntries.as_ptr() as
                                 *mut _,};
            init
        }
    };
static mut pid_prof_data: OSD_UINT8_t =
    unsafe {
        {
            let mut init =
                OSD_UINT8_t{val:
                                &tmpPidProfileIndex as *const uint8_t as
                                    *mut uint8_t,
                            min: 1i32 as uint8_t,
                            max: 3i32 as uint8_t,
                            step: 1i32 as uint8_t,};
            init
        }
    };
static mut rate_prof_data: OSD_UINT8_t =
    unsafe {
        {
            let mut init =
                OSD_UINT8_t{val:
                                &tmpRateProfileIndex as *const uint8_t as
                                    *mut uint8_t,
                            min: 1i32 as uint8_t,
                            max: 6i32 as uint8_t,
                            step: 1i32 as uint8_t,};
            init
        }
    };
static mut cmsx_menuImuEntries: [OSD_Entry; 10] =
    unsafe {
        [{
             let mut init =
                 OSD_Entry{text:
                               b"-- PROFILE --\x00" as *const u8 as
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
                               b"PID PROF\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT8,
                           func:
                               Some(cmsx_profileIndexOnChange as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data:
                               &pid_prof_data as *const OSD_UINT8_t as
                                   *mut OSD_UINT8_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"PID\x00" as *const u8 as *const libc::c_char,
                           type_0: OME_Submenu,
                           func:
                               Some(cmsMenuChange as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data:
                               &cmsx_menuPid as *const CMS_Menu as
                                   *mut CMS_Menu as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"MISC PP\x00" as *const u8 as
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
                               &cmsx_menuProfileOther as *const CMS_Menu as
                                   *mut CMS_Menu as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"FILT PP\x00" as *const u8 as
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
                               &cmsx_menuFilterPerProfile as *const CMS_Menu
                                   as *mut CMS_Menu as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"RATE PROF\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_UINT8,
                           func:
                               Some(cmsx_rateProfileIndexOnChange as
                                        unsafe extern "C" fn(_:
                                                                 *mut displayPort_t,
                                                             _:
                                                                 *const libc::c_void)
                                            -> libc::c_long),
                           data:
                               &rate_prof_data as *const OSD_UINT8_t as
                                   *mut OSD_UINT8_t as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"RATE\x00" as *const u8 as
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
                               &cmsx_menuRateProfile as *const CMS_Menu as
                                   *mut CMS_Menu as *mut libc::c_void,
                           flags: 0i32 as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"FILT GLB\x00" as *const u8 as
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
                               &cmsx_menuFilterGlobal as *const CMS_Menu as
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
#[no_mangle]
pub static mut cmsx_menuImu: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter:
                             Some(cmsx_menuImu_onEnter as
                                      unsafe extern "C" fn() -> libc::c_long),
                         onExit:
                             Some(cmsx_menuImu_onExit as
                                      unsafe extern "C" fn(_:
                                                               *const OSD_Entry)
                                          -> libc::c_long),
                         entries: cmsx_menuImuEntries.as_ptr() as *mut _,};
            init
        }
    };
unsafe extern "C" fn run_static_initializers() {
    rollp_data =
        {
            let mut init =
                OSD_UINT8_t{val:
                                &mut *(*tempPid.as_mut_ptr().offset(PID_ROLL
                                                                        as
                                                                        libc::c_int
                                                                        as
                                                                        isize)).as_mut_ptr().offset(0)
                                    as *mut uint8_t,
                            min: 0i32 as uint8_t,
                            max: 200i32 as uint8_t,
                            step: 1i32 as uint8_t,};
            init
        };
    rolli_data =
        {
            let mut init =
                OSD_UINT8_t{val:
                                &mut *(*tempPid.as_mut_ptr().offset(PID_ROLL
                                                                        as
                                                                        libc::c_int
                                                                        as
                                                                        isize)).as_mut_ptr().offset(1)
                                    as *mut uint8_t,
                            min: 0i32 as uint8_t,
                            max: 200i32 as uint8_t,
                            step: 1i32 as uint8_t,};
            init
        };
    rolld_data =
        {
            let mut init =
                OSD_UINT8_t{val:
                                &mut *(*tempPid.as_mut_ptr().offset(PID_ROLL
                                                                        as
                                                                        libc::c_int
                                                                        as
                                                                        isize)).as_mut_ptr().offset(2)
                                    as *mut uint8_t,
                            min: 0i32 as uint8_t,
                            max: 200i32 as uint8_t,
                            step: 1i32 as uint8_t,};
            init
        };
    rollf_data =
        {
            let mut init =
                OSD_UINT16_t{val:
                                 &mut *tempPidF.as_mut_ptr().offset(PID_ROLL
                                                                        as
                                                                        libc::c_int
                                                                        as
                                                                        isize)
                                     as *mut uint16_t,
                             min: 0i32 as uint16_t,
                             max: 2000i32 as uint16_t,
                             step: 1i32 as uint16_t,};
            init
        };
    pitchp_data =
        {
            let mut init =
                OSD_UINT8_t{val:
                                &mut *(*tempPid.as_mut_ptr().offset(PID_PITCH
                                                                        as
                                                                        libc::c_int
                                                                        as
                                                                        isize)).as_mut_ptr().offset(0)
                                    as *mut uint8_t,
                            min: 0i32 as uint8_t,
                            max: 200i32 as uint8_t,
                            step: 1i32 as uint8_t,};
            init
        };
    pitchi_data =
        {
            let mut init =
                OSD_UINT8_t{val:
                                &mut *(*tempPid.as_mut_ptr().offset(PID_PITCH
                                                                        as
                                                                        libc::c_int
                                                                        as
                                                                        isize)).as_mut_ptr().offset(1)
                                    as *mut uint8_t,
                            min: 0i32 as uint8_t,
                            max: 200i32 as uint8_t,
                            step: 1i32 as uint8_t,};
            init
        };
    pitchd_data =
        {
            let mut init =
                OSD_UINT8_t{val:
                                &mut *(*tempPid.as_mut_ptr().offset(PID_PITCH
                                                                        as
                                                                        libc::c_int
                                                                        as
                                                                        isize)).as_mut_ptr().offset(2)
                                    as *mut uint8_t,
                            min: 0i32 as uint8_t,
                            max: 200i32 as uint8_t,
                            step: 1i32 as uint8_t,};
            init
        };
    pitchf_data =
        {
            let mut init =
                OSD_UINT16_t{val:
                                 &mut *tempPidF.as_mut_ptr().offset(PID_PITCH
                                                                        as
                                                                        libc::c_int
                                                                        as
                                                                        isize)
                                     as *mut uint16_t,
                             min: 0i32 as uint16_t,
                             max: 2000i32 as uint16_t,
                             step: 1i32 as uint16_t,};
            init
        };
    yawp_data =
        {
            let mut init =
                OSD_UINT8_t{val:
                                &mut *(*tempPid.as_mut_ptr().offset(PID_YAW as
                                                                        libc::c_int
                                                                        as
                                                                        isize)).as_mut_ptr().offset(0)
                                    as *mut uint8_t,
                            min: 0i32 as uint8_t,
                            max: 200i32 as uint8_t,
                            step: 1i32 as uint8_t,};
            init
        };
    yawi_data =
        {
            let mut init =
                OSD_UINT8_t{val:
                                &mut *(*tempPid.as_mut_ptr().offset(PID_YAW as
                                                                        libc::c_int
                                                                        as
                                                                        isize)).as_mut_ptr().offset(1)
                                    as *mut uint8_t,
                            min: 0i32 as uint8_t,
                            max: 200i32 as uint8_t,
                            step: 1i32 as uint8_t,};
            init
        };
    yawd_data =
        {
            let mut init =
                OSD_UINT8_t{val:
                                &mut *(*tempPid.as_mut_ptr().offset(PID_YAW as
                                                                        libc::c_int
                                                                        as
                                                                        isize)).as_mut_ptr().offset(2)
                                    as *mut uint8_t,
                            min: 0i32 as uint8_t,
                            max: 200i32 as uint8_t,
                            step: 1i32 as uint8_t,};
            init
        };
    yawf_data =
        {
            let mut init =
                OSD_UINT16_t{val:
                                 &mut *tempPidF.as_mut_ptr().offset(PID_YAW as
                                                                        libc::c_int
                                                                        as
                                                                        isize)
                                     as *mut uint16_t,
                             min: 0i32 as uint16_t,
                             max: 2000i32 as uint16_t,
                             step: 1i32 as uint16_t,};
            init
        };
    rc_r_rate_data =
        {
            let mut init =
                OSD_FLOAT_t{val:
                                &mut *rateProfile.rcRates.as_mut_ptr().offset(FD_ROLL
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  isize)
                                    as *mut uint8_t,
                            min: 0i32 as uint8_t,
                            max: 255i32 as uint8_t,
                            step: 1i32 as uint8_t,
                            multipler: 10i32 as uint16_t,};
            init
        };
    rc_p_rate_data =
        {
            let mut init =
                OSD_FLOAT_t{val:
                                &mut *rateProfile.rcRates.as_mut_ptr().offset(FD_PITCH
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  isize)
                                    as *mut uint8_t,
                            min: 0i32 as uint8_t,
                            max: 255i32 as uint8_t,
                            step: 1i32 as uint8_t,
                            multipler: 10i32 as uint16_t,};
            init
        };
    rc_y_rate_data =
        {
            let mut init =
                OSD_FLOAT_t{val:
                                &mut *rateProfile.rcRates.as_mut_ptr().offset(FD_YAW
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  isize)
                                    as *mut uint8_t,
                            min: 0i32 as uint8_t,
                            max: 255i32 as uint8_t,
                            step: 1i32 as uint8_t,
                            multipler: 10i32 as uint16_t,};
            init
        };
    roll_super_data =
        {
            let mut init =
                OSD_FLOAT_t{val:
                                &mut *rateProfile.rates.as_mut_ptr().offset(FD_ROLL
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                    as *mut uint8_t,
                            min: 0i32 as uint8_t,
                            max: 100i32 as uint8_t,
                            step: 1i32 as uint8_t,
                            multipler: 10i32 as uint16_t,};
            init
        };
    pitch_super_data =
        {
            let mut init =
                OSD_FLOAT_t{val:
                                &mut *rateProfile.rates.as_mut_ptr().offset(FD_PITCH
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                    as *mut uint8_t,
                            min: 0i32 as uint8_t,
                            max: 100i32 as uint8_t,
                            step: 1i32 as uint8_t,
                            multipler: 10i32 as uint16_t,};
            init
        };
    yaw_super_data =
        {
            let mut init =
                OSD_FLOAT_t{val:
                                &mut *rateProfile.rates.as_mut_ptr().offset(FD_YAW
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                    as *mut uint8_t,
                            min: 0i32 as uint8_t,
                            max: 100i32 as uint8_t,
                            step: 1i32 as uint8_t,
                            multipler: 10i32 as uint16_t,};
            init
        };
    rc_r_expo_data =
        {
            let mut init =
                OSD_FLOAT_t{val:
                                &mut *rateProfile.rcExpo.as_mut_ptr().offset(FD_ROLL
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 isize)
                                    as *mut uint8_t,
                            min: 0i32 as uint8_t,
                            max: 100i32 as uint8_t,
                            step: 1i32 as uint8_t,
                            multipler: 10i32 as uint16_t,};
            init
        };
    rc_p_expo_data =
        {
            let mut init =
                OSD_FLOAT_t{val:
                                &mut *rateProfile.rcExpo.as_mut_ptr().offset(FD_PITCH
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 isize)
                                    as *mut uint8_t,
                            min: 0i32 as uint8_t,
                            max: 100i32 as uint8_t,
                            step: 1i32 as uint8_t,
                            multipler: 10i32 as uint16_t,};
            init
        };
    rc_y_expo_data =
        {
            let mut init =
                OSD_FLOAT_t{val:
                                &mut *rateProfile.rcExpo.as_mut_ptr().offset(FD_YAW
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 isize)
                                    as *mut uint8_t,
                            min: 0i32 as uint8_t,
                            max: 100i32 as uint8_t,
                            step: 1i32 as uint8_t,
                            multipler: 10i32 as uint16_t,};
            init
        };
    thr_mid_data =
        {
            let mut init =
                OSD_UINT8_t{val: &mut rateProfile.thrMid8,
                            min: 0i32 as uint8_t,
                            max: 100i32 as uint8_t,
                            step: 1i32 as uint8_t,};
            init
        };
    thr_expo_data =
        {
            let mut init =
                OSD_UINT8_t{val: &mut rateProfile.thrExpo8,
                            min: 0i32 as uint8_t,
                            max: 100i32 as uint8_t,
                            step: 1i32 as uint8_t,};
            init
        };
    thrpid_att_data =
        {
            let mut init =
                OSD_FLOAT_t{val: &mut rateProfile.dynThrPID,
                            min: 0i32 as uint8_t,
                            max: 100i32 as uint8_t,
                            step: 1i32 as uint8_t,
                            multipler: 10i32 as uint16_t,};
            init
        };
    tpa_brkpt_data =
        {
            let mut init =
                OSD_UINT16_t{val: &mut rateProfile.tpa_breakpoint,
                             min: 1000i32 as uint16_t,
                             max: 2000i32 as uint16_t,
                             step: 10i32 as uint16_t,};
            init
        }
}
#[used]
#[cfg_attr ( target_os = "linux", link_section = ".init_array" )]
#[cfg_attr ( target_os = "windows", link_section = ".CRT$XIB" )]
#[cfg_attr ( target_os = "macos", link_section = "__DATA,__mod_init_func" )]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
// CMS
