use ::libc;
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
    fn copyControlRateProfile(dstControlRateProfileIndex: uint8_t,
                              srcControlRateProfileIndex: uint8_t);
    #[no_mangle]
    static mut pidProfiles_SystemArray: [pidProfile_t; 3];
    #[no_mangle]
    fn pidInitConfig(pidProfile: *const pidProfile_t);
    #[no_mangle]
    fn pidCopyProfile(dstPidProfileIndex: uint8_t,
                      srcPidProfileIndex: uint8_t);
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
pub struct OSD_FLOAT_t {
    pub val: *mut uint8_t,
    pub min: uint8_t,
    pub max: uint8_t,
    pub step: uint8_t,
    pub multipler: uint16_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct OSD_TAB_t {
    pub val: *mut uint8_t,
    pub max: uint8_t,
    pub names: *const *const libc::c_char,
}
pub type gyroConfig_t = gyroConfig_s;
#[derive(Copy, Clone)]
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
#[derive(Copy, Clone)]
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
#[derive(Copy, Clone)]
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
#[derive(Copy, Clone)]
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
pub type C2RustUnnamed_0 = libc::c_uint;
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
#[inline]
unsafe extern "C" fn controlRateProfilesMutable(mut _index: libc::c_int)
 -> *mut controlRateConfig_t {
    return &mut *controlRateProfiles_SystemArray.as_mut_ptr().offset(_index as
                                                                         isize)
               as *mut controlRateConfig_t;
}
#[inline]
unsafe extern "C" fn controlRateProfiles(mut _index: libc::c_int)
 -> *const controlRateConfig_t {
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
// Breakpoint where TPA is activated
// Sets the throttle limiting type - off, scale or clip
// Sets the maximum pilot commanded throttle limit
// Additional yaw filter when yaw axis too noisy
// Delta Filter in hz
// Biquad dterm notch hz
// Biquad dterm notch low cutoff
// Filter selection for dterm
// Experimental ITerm windup threshold, percent motor saturation
// Disable/Enable pids on zero throttle. Normally even without airmode P and D would be active.
// Max angle in degrees in level mode
// inclination factor for Horizon mode
// OFF or ON
// Betaflight PID controller parameters
// type of anti gravity method
// max allowed throttle delta before iterm accelerated in ms
// Iterm Accelerator Gain when itermThrottlethreshold is hit
// yaw accel limiter for deg/sec/ms
// accel limiter roll/pitch deg/sec/ms
// dterm crash value
// gyro crash value
// setpoint must be below this value to detect crash, so flips and rolls are not interpreted as crashes
// ms
// ms
// degrees
// degree/second
// Scale PIDsum to battery voltage
// Feed forward weight transition
// limits yaw errorRate, so crashes don't cause huge throttle increase
// Extra PT1 Filter on D in hz
// off, on, on and beeps when it is in crash recovery mode
// how much should throttle be boosted during transient changes 0-100, 100 adds 10x hpf filtered throttle
// Which cutoff frequency to use for throttle boost. higher cutoffs keep the boost on for shorter. Specified in hz.
// rotates iterm to translate world errors to local coordinate system
// takes only the larger of P and the D weight feed forward term if they have the same sign.
// Specifies type of relax algorithm
// This cutoff frequency specifies a low pass filter which predicts average response of the quad to setpoint
// Enable iterm suppression during stick input
// Acro trainer roll/pitch angle limit in degrees
// The axis for which record debugging values are captured 0=roll, 1=pitch
// The strength of the limiting. Raising may reduce overshoot but also lead to oscillation around the angle limit
// The lookahead window in milliseconds used to reduce overshoot
// How strongly should the absolute accumulated error be corrected for
// Limit to the correction
// Limit to the accumulated error
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
    tmpPidProfileIndex =
        (pidProfileIndex as libc::c_int + 1 as libc::c_int) as uint8_t;
    rateProfileIndex = getCurrentControlRateProfileIndex();
    tmpRateProfileIndex =
        (rateProfileIndex as libc::c_int + 1 as libc::c_int) as uint8_t;
    return 0 as libc::c_int as libc::c_long;
}
unsafe extern "C" fn cmsx_menuImu_onExit(mut self_0: *const OSD_Entry)
 -> libc::c_long {
    changePidProfile(pidProfileIndex);
    changeControlRateProfile(rateProfileIndex);
    return 0 as libc::c_int as libc::c_long;
}
unsafe extern "C" fn cmsx_profileIndexOnChange(mut displayPort:
                                                   *mut displayPort_t,
                                               mut ptr: *const libc::c_void)
 -> libc::c_long {
    pidProfileIndex =
        (tmpPidProfileIndex as libc::c_int - 1 as libc::c_int) as uint8_t;
    changePidProfile(pidProfileIndex);
    return 0 as libc::c_int as libc::c_long;
}
unsafe extern "C" fn cmsx_rateProfileIndexOnChange(mut displayPort:
                                                       *mut displayPort_t,
                                                   mut ptr:
                                                       *const libc::c_void)
 -> libc::c_long {
    rateProfileIndex =
        (tmpRateProfileIndex as libc::c_int - 1 as libc::c_int) as uint8_t;
    changeControlRateProfile(rateProfileIndex);
    return 0 as libc::c_int as libc::c_long;
}
unsafe extern "C" fn cmsx_PidRead() -> libc::c_long {
    let mut pidProfile: *const pidProfile_t =
        pidProfiles(pidProfileIndex as libc::c_int);
    let mut i: uint8_t = 0 as libc::c_int as uint8_t;
    while (i as libc::c_int) < 3 as libc::c_int {
        tempPid[i as usize][0 as libc::c_int as usize] =
            (*pidProfile).pid[i as usize].P;
        tempPid[i as usize][1 as libc::c_int as usize] =
            (*pidProfile).pid[i as usize].I;
        tempPid[i as usize][2 as libc::c_int as usize] =
            (*pidProfile).pid[i as usize].D;
        tempPidF[i as usize] = (*pidProfile).pid[i as usize].F;
        i = i.wrapping_add(1)
    }
    return 0 as libc::c_int as libc::c_long;
}
unsafe extern "C" fn cmsx_PidOnEnter() -> libc::c_long {
    pidProfileIndexString[1 as libc::c_int as usize] =
        ('0' as i32 + tmpPidProfileIndex as libc::c_int) as libc::c_char;
    cmsx_PidRead();
    return 0 as libc::c_int as libc::c_long;
}
unsafe extern "C" fn cmsx_PidWriteback(mut self_0: *const OSD_Entry)
 -> libc::c_long {
    let mut pidProfile: *mut pidProfile_t = currentPidProfile;
    let mut i: uint8_t = 0 as libc::c_int as uint8_t;
    while (i as libc::c_int) < 3 as libc::c_int {
        (*pidProfile).pid[i as usize].P =
            tempPid[i as usize][0 as libc::c_int as usize];
        (*pidProfile).pid[i as usize].I =
            tempPid[i as usize][1 as libc::c_int as usize];
        (*pidProfile).pid[i as usize].D =
            tempPid[i as usize][2 as libc::c_int as usize];
        (*pidProfile).pid[i as usize].F = tempPidF[i as usize];
        i = i.wrapping_add(1)
    }
    pidInitConfig(currentPidProfile);
    return 0 as libc::c_int as libc::c_long;
}
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
    return 0 as libc::c_int as libc::c_long;
}
unsafe extern "C" fn cmsx_RateProfileWriteback(mut self_0: *const OSD_Entry)
 -> libc::c_long {
    memcpy(controlRateProfilesMutable(rateProfileIndex as libc::c_int) as
               *mut libc::c_void,
           &mut rateProfile as *mut controlRateConfig_t as
               *const libc::c_void,
           ::core::mem::size_of::<controlRateConfig_t>() as libc::c_ulong);
    return 0 as libc::c_int as libc::c_long;
}
unsafe extern "C" fn cmsx_RateProfileOnEnter() -> libc::c_long {
    rateProfileIndexString[1 as libc::c_int as usize] =
        ('0' as i32 + tmpPidProfileIndex as libc::c_int) as libc::c_char;
    rateProfileIndexString[3 as libc::c_int as usize] =
        ('0' as i32 + tmpRateProfileIndex as libc::c_int) as libc::c_char;
    cmsx_RateProfileRead();
    return 0 as libc::c_int as libc::c_long;
}
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
    pidProfileIndexString[1 as libc::c_int as usize] =
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
    return 0 as libc::c_int as libc::c_long;
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
    return 0 as libc::c_int as libc::c_long;
}
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
    return 0 as libc::c_int as libc::c_long;
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
    return 0 as libc::c_int as libc::c_long;
}
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
    return 0 as libc::c_int as libc::c_long;
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
    return 0 as libc::c_int as libc::c_long;
}
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
static mut cmsx_dstPidProfile: uint8_t = 0;
static mut cmsx_dstControlRateProfile: uint8_t = 0;
static mut cmsx_ProfileNames: [*const libc::c_char; 4] =
    [b"-\x00" as *const u8 as *const libc::c_char,
     b"1\x00" as *const u8 as *const libc::c_char,
     b"2\x00" as *const u8 as *const libc::c_char,
     b"3\x00" as *const u8 as *const libc::c_char];
static mut cmsx_PidProfileTable: OSD_TAB_t =
    unsafe {
        {
            let mut init =
                OSD_TAB_t{val:
                              &cmsx_dstPidProfile as *const uint8_t as
                                  *mut uint8_t,
                          max: 3 as libc::c_int as uint8_t,
                          names: cmsx_ProfileNames.as_ptr(),};
            init
        }
    };
static mut cmsx_ControlRateProfileTable: OSD_TAB_t =
    unsafe {
        {
            let mut init =
                OSD_TAB_t{val:
                              &cmsx_dstControlRateProfile as *const uint8_t as
                                  *mut uint8_t,
                          max: 3 as libc::c_int as uint8_t,
                          names: cmsx_ProfileNames.as_ptr(),};
            init
        }
    };
unsafe extern "C" fn cmsx_menuCopyProfile_onEnter() -> libc::c_long {
    cmsx_dstPidProfile = 0 as libc::c_int as uint8_t;
    cmsx_dstControlRateProfile = 0 as libc::c_int as uint8_t;
    return 0 as libc::c_int as libc::c_long;
}
unsafe extern "C" fn cmsx_CopyPidProfile(mut pDisplay: *mut displayPort_t,
                                         mut ptr: *const libc::c_void)
 -> libc::c_long {
    if cmsx_dstPidProfile as libc::c_int > 0 as libc::c_int {
        pidCopyProfile((cmsx_dstPidProfile as libc::c_int - 1 as libc::c_int)
                           as uint8_t, getCurrentPidProfileIndex());
    }
    return 0 as libc::c_int as libc::c_long;
}
unsafe extern "C" fn cmsx_CopyControlRateProfile(mut pDisplay:
                                                     *mut displayPort_t,
                                                 mut ptr: *const libc::c_void)
 -> libc::c_long {
    if cmsx_dstControlRateProfile as libc::c_int > 0 as libc::c_int {
        copyControlRateProfile((cmsx_dstControlRateProfile as libc::c_int -
                                    1 as libc::c_int) as uint8_t,
                               getCurrentControlRateProfileIndex());
    }
    return 0 as libc::c_int as libc::c_long;
}
static mut cmsx_menuCopyProfileEntries: [OSD_Entry; 7] =
    unsafe {
        [{
             let mut init =
                 OSD_Entry{text:
                               b"-- COPY PROFILE --\x00" as *const u8 as
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
                               b"CPY PID PROF TO\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_TAB,
                           func: None,
                           data:
                               &cmsx_PidProfileTable as *const OSD_TAB_t as
                                   *mut OSD_TAB_t as *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"COPY PP\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Funcall,
                           func:
                               Some(cmsx_CopyPidProfile as
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
                               b"CPY RATE PROF TO\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_TAB,
                           func: None,
                           data:
                               &cmsx_ControlRateProfileTable as
                                   *const OSD_TAB_t as *mut OSD_TAB_t as
                                   *mut libc::c_void,
                           flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 OSD_Entry{text:
                               b"COPY RP\x00" as *const u8 as
                                   *const libc::c_char,
                           type_0: OME_Funcall,
                           func:
                               Some(cmsx_CopyControlRateProfile as
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
#[no_mangle]
pub static mut cmsx_menuCopyProfile: CMS_Menu =
    unsafe {
        {
            let mut init =
                CMS_Menu{onEnter:
                             Some(cmsx_menuCopyProfile_onEnter as
                                      unsafe extern "C" fn() -> libc::c_long),
                         onExit: None,
                         entries:
                             cmsx_menuCopyProfileEntries.as_ptr() as *mut _,};
            init
        }
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
// CMS
