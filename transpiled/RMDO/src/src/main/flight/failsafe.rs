use ::libc;
extern "C" {
    #[no_mangle]
    static mut rxConfig_System: rxConfig_t;
    #[no_mangle]
    fn millis() -> timeMs_t;
    #[no_mangle]
    fn disarm();
    #[no_mangle]
    fn calculateThrottleStatus() -> throttleStatus_e;
    #[no_mangle]
    fn isUsingSticksForArming() -> bool;
    #[no_mangle]
    fn IS_RC_MODE_ACTIVE(boxId: boxId_e) -> bool;
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    fn setArmingDisabled(flag: armingDisableFlags_e);
    #[no_mangle]
    fn unsetArmingDisabled(flag: armingDisableFlags_e);
    #[no_mangle]
    fn enableFlightMode(mask: flightModeFlags_e) -> uint16_t;
    #[no_mangle]
    fn disableFlightMode(mask: flightModeFlags_e) -> uint16_t;
    #[no_mangle]
    fn beeper(mode: beeperMode_e);
    #[no_mangle]
    static mut rcData: [int16_t; 18];
    #[no_mangle]
    fn crashRecoveryModeActive() -> bool;
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
pub type pgn_t = uint16_t;
pub type C2RustUnnamed = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed = 4095;
// function that resets a single parameter group instance
pub type pgResetFunc
    =
    unsafe extern "C" fn(_: *mut libc::c_void, _: libc::c_int) -> ();
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pgRegistry_s {
    pub pgn: pgn_t,
    pub size: uint16_t,
    pub address: *mut uint8_t,
    pub copy: *mut uint8_t,
    pub ptr: *mut *mut uint8_t,
    pub reset: C2RustUnnamed_0,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_0 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<pgResetFunc>,
}
pub type pgRegistry_t = pgRegistry_s;
/* base */
/* size */
// The parameter group number, the top 4 bits are reserved for version
// Size of the group in RAM, the top 4 bits are reserved for flags
// Address of the group in RAM.
// Address of the copy in RAM.
// The pointer to update after loading the record into ram.
// Pointer to init template
// Pointer to pgResetFunc
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
// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
pub type ioTag_t = uint8_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rxConfig_s {
    pub rcmap: [uint8_t; 8],
    pub serialrx_provider: uint8_t,
    pub serialrx_inverted: uint8_t,
    pub halfDuplex: uint8_t,
    pub spektrum_bind_pin_override_ioTag: ioTag_t,
    pub spektrum_bind_plug_ioTag: ioTag_t,
    pub spektrum_sat_bind: uint8_t,
    pub spektrum_sat_bind_autoreset: uint8_t,
    pub rssi_channel: uint8_t,
    pub rssi_scale: uint8_t,
    pub rssi_invert: uint8_t,
    pub midrc: uint16_t,
    pub mincheck: uint16_t,
    pub maxcheck: uint16_t,
    pub rcInterpolation: uint8_t,
    pub rcInterpolationChannels: uint8_t,
    pub rcInterpolationInterval: uint8_t,
    pub fpvCamAngleDegrees: uint8_t,
    pub airModeActivateThreshold: uint8_t,
    pub rx_min_usec: uint16_t,
    pub rx_max_usec: uint16_t,
    pub max_aux_channel: uint8_t,
    pub rssi_src_frame_errors: uint8_t,
    pub rssi_offset: int8_t,
    pub rc_smoothing_type: uint8_t,
    pub rc_smoothing_input_cutoff: uint8_t,
    pub rc_smoothing_derivative_cutoff: uint8_t,
    pub rc_smoothing_debug_axis: uint8_t,
    pub rc_smoothing_input_type: uint8_t,
    pub rc_smoothing_derivative_type: uint8_t,
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
pub type rxConfig_t = rxConfig_s;
pub type timeMs_t = uint32_t;
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
pub type throttleStatus_e = libc::c_uint;
pub const THROTTLE_HIGH: throttleStatus_e = 1;
pub const THROTTLE_LOW: throttleStatus_e = 0;
pub type boxId_e = libc::c_uint;
pub const CHECKBOX_ITEM_COUNT: boxId_e = 41;
pub const BOXACROTRAINER: boxId_e = 40;
pub const BOXPIDAUDIO: boxId_e = 39;
pub const BOXUSER4: boxId_e = 38;
pub const BOXUSER3: boxId_e = 37;
pub const BOXUSER2: boxId_e = 36;
pub const BOXUSER1: boxId_e = 35;
pub const BOXPARALYZE: boxId_e = 34;
pub const BOXVTXPITMODE: boxId_e = 33;
pub const BOXBEEPGPSCOUNT: boxId_e = 32;
pub const BOXPREARM: boxId_e = 31;
pub const BOXFLIPOVERAFTERCRASH: boxId_e = 30;
pub const BOXCAMERA3: boxId_e = 29;
pub const BOXCAMERA2: boxId_e = 28;
pub const BOXCAMERA1: boxId_e = 27;
pub const BOXBLACKBOXERASE: boxId_e = 26;
pub const BOXFPVANGLEMIX: boxId_e = 25;
pub const BOX3D: boxId_e = 24;
pub const BOXAIRMODE: boxId_e = 23;
pub const BOXBLACKBOX: boxId_e = 22;
pub const BOXSERVO3: boxId_e = 21;
pub const BOXSERVO2: boxId_e = 20;
pub const BOXSERVO1: boxId_e = 19;
pub const BOXTELEMETRY: boxId_e = 18;
pub const BOXOSD: boxId_e = 17;
pub const BOXCALIB: boxId_e = 16;
pub const BOXLEDLOW: boxId_e = 15;
pub const BOXBEEPERON: boxId_e = 14;
pub const BOXCAMSTAB: boxId_e = 13;
pub const BOXHEADADJ: boxId_e = 12;
pub const BOXANTIGRAVITY: boxId_e = 11;
pub const BOXID_FLIGHTMODE_LAST: boxId_e = 10;
pub const BOXGPSRESCUE: boxId_e = 10;
pub const BOXFAILSAFE: boxId_e = 9;
pub const BOXPASSTHRU: boxId_e = 8;
pub const BOXHEADFREE: boxId_e = 7;
pub const BOXGPSHOLD: boxId_e = 6;
pub const BOXGPSHOME: boxId_e = 5;
pub const BOXBARO: boxId_e = 4;
pub const BOXMAG: boxId_e = 3;
pub const BOXHORIZON: boxId_e = 2;
pub const BOXANGLE: boxId_e = 1;
pub const BOXARM: boxId_e = 0;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed_1 = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed_1 = 2;
pub const ARMED: C2RustUnnamed_1 = 1;
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
pub type flightModeFlags_e = libc::c_uint;
pub const GPS_RESCUE_MODE: flightModeFlags_e = 2048;
pub const FAILSAFE_MODE: flightModeFlags_e = 1024;
pub const PASSTHRU_MODE: flightModeFlags_e = 256;
pub const HEADFREE_MODE: flightModeFlags_e = 64;
pub const GPS_HOLD_MODE: flightModeFlags_e = 32;
pub const GPS_HOME_MODE: flightModeFlags_e = 16;
pub const BARO_MODE: flightModeFlags_e = 8;
pub const MAG_MODE: flightModeFlags_e = 4;
pub const HORIZON_MODE: flightModeFlags_e = 2;
pub const ANGLE_MODE: flightModeFlags_e = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct failsafeConfig_s {
    pub failsafe_throttle: uint16_t,
    pub failsafe_throttle_low_delay: uint16_t,
    pub failsafe_delay: uint8_t,
    pub failsafe_off_delay: uint8_t,
    pub failsafe_switch_mode: uint8_t,
    pub failsafe_procedure: uint8_t,
}
// mapping of radio channels to internal RPYTA+ order
// type of UART-based receiver (0 = spek 10, 1 = spek 11, 2 = sbus). Must be enabled by FEATURE_RX_SERIAL first.
// invert the serial RX protocol compared to it's default setting
// allow rx to operate in half duplex mode on F4, ignored for F1 and F3.
// number of bind pulses for Spektrum satellite receivers
// whenever we will reset (exit) binding mode after hard reboot
// Some radios have not a neutral point centered on 1500. can be changed here
// minimum rc end
// maximum rc end
// Camera angle to be scaled into rc commands
// Throttle setpoint percent where airmode gets activated
// true to use frame drop flags in the rx protocol
// offset applied to the RSSI value before it is returned
// Determines the smoothing algorithm to use: INTERPOLATION or FILTER
// Filter cutoff frequency for the input filter (0 = auto)
// Filter cutoff frequency for the setpoint weight derivative filter (0 = auto)
// Axis to log as debug values when debug_mode = RC_SMOOTHING
// Input filter type (0 = PT1, 1 = BIQUAD)
// Derivative filter type (0 = OFF, 1 = PT1, 2 = BIQUAD)
// millis
pub type failsafeConfig_t = failsafeConfig_s;
pub type failsafePhase_e = libc::c_uint;
pub const FAILSAFE_GPS_RESCUE: failsafePhase_e = 6;
pub const FAILSAFE_RX_LOSS_RECOVERED: failsafePhase_e = 5;
pub const FAILSAFE_RX_LOSS_MONITORING: failsafePhase_e = 4;
pub const FAILSAFE_LANDED: failsafePhase_e = 3;
pub const FAILSAFE_LANDING: failsafePhase_e = 2;
pub const FAILSAFE_RX_LOSS_DETECTED: failsafePhase_e = 1;
pub const FAILSAFE_IDLE: failsafePhase_e = 0;
pub type failsafeRxLinkState_e = libc::c_uint;
pub const FAILSAFE_RXLINK_UP: failsafeRxLinkState_e = 1;
pub const FAILSAFE_RXLINK_DOWN: failsafeRxLinkState_e = 0;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const FAILSAFE_PROCEDURE_GPS_RESCUE: C2RustUnnamed_2 = 2;
pub const FAILSAFE_PROCEDURE_DROP_IT: C2RustUnnamed_2 = 1;
pub const FAILSAFE_PROCEDURE_AUTO_LANDING: C2RustUnnamed_2 = 0;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const FAILSAFE_SWITCH_MODE_STAGE2: C2RustUnnamed_3 = 2;
pub const FAILSAFE_SWITCH_MODE_KILL: C2RustUnnamed_3 = 1;
pub const FAILSAFE_SWITCH_MODE_STAGE1: C2RustUnnamed_3 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct failsafeState_s {
    pub events: int16_t,
    pub monitoring: bool,
    pub active: bool,
    pub rxDataFailurePeriod: uint32_t,
    pub validRxDataReceivedAt: uint32_t,
    pub validRxDataFailedAt: uint32_t,
    pub throttleLowPeriod: uint32_t,
    pub landingShouldBeFinishedAt: uint32_t,
    pub receivingRxDataPeriod: uint32_t,
    pub receivingRxDataPeriodPreset: uint32_t,
    pub phase: failsafePhase_e,
    pub rxLinkState: failsafeRxLinkState_e,
}
pub type failsafeState_t = failsafeState_s;
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
unsafe extern "C" fn rxConfig() -> *const rxConfig_t {
    return &mut rxConfig_System;
}
#[inline]
unsafe extern "C" fn failsafeConfig() -> *const failsafeConfig_t {
    return &mut failsafeConfig_System;
}
// Throttle level used for landing - specify value between 1000..2000 (pwm pulse width for slightly below hover). center throttle = 1500.
// Time throttle stick must have been below 'min_check' to "JustDisarm" instead of "full failsafe procedure".
// Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example (10)
// Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example (200)
// failsafe switch action is 0: stage1 (identical to rc link loss), 1: disarms instantly, 2: stage2
// selected full failsafe procedure is 0: auto-landing, 1: Drop it
// throttle stick must have been below 'min_check' for this period
// period for the required period of valid rxData
// preset for the required period of valid rxData
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
/*
 * Usage:
 *
 * failsafeInit() and failsafeReset() must be called before the other methods are used.
 *
 * failsafeInit() and failsafeReset() can be called in any order.
 * failsafeInit() should only be called once.
 *
 * enable() should be called after system initialisation.
 */
static mut failsafeState: failsafeState_t =
    failsafeState_t{events: 0,
                    monitoring: false,
                    active: false,
                    rxDataFailurePeriod: 0,
                    validRxDataReceivedAt: 0,
                    validRxDataFailedAt: 0,
                    throttleLowPeriod: 0,
                    landingShouldBeFinishedAt: 0,
                    receivingRxDataPeriod: 0,
                    receivingRxDataPeriodPreset: 0,
                    phase: FAILSAFE_IDLE,
                    rxLinkState: FAILSAFE_RXLINK_DOWN,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut failsafeConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (1 as libc::c_int |
                                      (2 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<failsafeConfig_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &failsafeConfig_System as
                                     *const failsafeConfig_t as
                                     *mut failsafeConfig_t as *mut uint8_t,
                             copy:
                                 &failsafeConfig_Copy as
                                     *const failsafeConfig_t as
                                     *mut failsafeConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{ptr:
                                                     &pgResetTemplate_failsafeConfig
                                                         as
                                                         *const failsafeConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut failsafeConfig_Copy: failsafeConfig_t =
    failsafeConfig_t{failsafe_throttle: 0,
                     failsafe_throttle_low_delay: 0,
                     failsafe_delay: 0,
                     failsafe_off_delay: 0,
                     failsafe_switch_mode: 0,
                     failsafe_procedure: 0,};
#[no_mangle]
pub static mut failsafeConfig_System: failsafeConfig_t =
    failsafeConfig_t{failsafe_throttle: 0,
                     failsafe_throttle_low_delay: 0,
                     failsafe_delay: 0,
                     failsafe_off_delay: 0,
                     failsafe_switch_mode: 0,
                     failsafe_procedure: 0,};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_failsafeConfig: failsafeConfig_t =
    {
        let mut init =
            failsafeConfig_s{failsafe_throttle:
                                 1000 as libc::c_int as uint16_t,
                             failsafe_throttle_low_delay:
                                 100 as libc::c_int as uint16_t,
                             failsafe_delay: 4 as libc::c_int as uint8_t,
                             failsafe_off_delay: 10 as libc::c_int as uint8_t,
                             failsafe_switch_mode:
                                 0 as libc::c_int as uint8_t,
                             failsafe_procedure:
                                 FAILSAFE_PROCEDURE_DROP_IT as libc::c_int as
                                     uint8_t,};
        init
    };
// default full failsafe procedure is 0: auto-landing
/*
 * Should called when the failsafe config needs to be changed - e.g. a different profile has been selected.
 */
#[no_mangle]
pub unsafe extern "C" fn failsafeReset() {
    failsafeState.rxDataFailurePeriod =
        (200 as libc::c_int +
             (*failsafeConfig()).failsafe_delay as libc::c_int *
                 100 as libc::c_int) as uint32_t;
    failsafeState.validRxDataReceivedAt = 0 as libc::c_int as uint32_t;
    failsafeState.validRxDataFailedAt = 0 as libc::c_int as uint32_t;
    failsafeState.throttleLowPeriod = 0 as libc::c_int as uint32_t;
    failsafeState.landingShouldBeFinishedAt = 0 as libc::c_int as uint32_t;
    failsafeState.receivingRxDataPeriod = 0 as libc::c_int as uint32_t;
    failsafeState.receivingRxDataPeriodPreset = 0 as libc::c_int as uint32_t;
    failsafeState.phase = FAILSAFE_IDLE;
    failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;
}
#[no_mangle]
pub unsafe extern "C" fn failsafeInit() {
    failsafeState.events = 0 as libc::c_int as int16_t;
    failsafeState.monitoring = 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn failsafePhase() -> failsafePhase_e {
    return failsafeState.phase;
}
#[no_mangle]
pub unsafe extern "C" fn failsafeIsMonitoring() -> bool {
    return failsafeState.monitoring;
}
#[no_mangle]
pub unsafe extern "C" fn failsafeIsActive() -> bool {
    return failsafeState.active;
}
#[no_mangle]
pub unsafe extern "C" fn failsafeStartMonitoring() {
    failsafeState.monitoring = 1 as libc::c_int != 0;
}
unsafe extern "C" fn failsafeShouldHaveCausedLandingByNow() -> bool {
    return millis() > failsafeState.landingShouldBeFinishedAt;
}
unsafe extern "C" fn failsafeActivate() {
    failsafeState.active = 1 as libc::c_int != 0;
    failsafeState.phase = FAILSAFE_LANDING;
    enableFlightMode(FAILSAFE_MODE);
    failsafeState.landingShouldBeFinishedAt =
        millis().wrapping_add(((*failsafeConfig()).failsafe_off_delay as
                                   libc::c_int * 100 as libc::c_int) as
                                  libc::c_uint);
    failsafeState.events += 1;
}
unsafe extern "C" fn failsafeApplyControlInput() {
    if (*failsafeConfig()).failsafe_procedure as libc::c_int ==
           FAILSAFE_PROCEDURE_GPS_RESCUE as libc::c_int {
        enableFlightMode(GPS_RESCUE_MODE);
        return
    }
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 3 as libc::c_int {
        rcData[i as usize] = (*rxConfig()).midrc as int16_t;
        i += 1
    }
    rcData[THROTTLE as libc::c_int as usize] =
        (*failsafeConfig()).failsafe_throttle as int16_t;
}
#[no_mangle]
pub unsafe extern "C" fn failsafeIsReceivingRxData() -> bool {
    return failsafeState.rxLinkState as libc::c_uint ==
               FAILSAFE_RXLINK_UP as libc::c_int as libc::c_uint;
}
#[no_mangle]
pub unsafe extern "C" fn failsafeOnRxSuspend(mut usSuspendPeriod: uint32_t) {
    failsafeState.validRxDataReceivedAt =
        (failsafeState.validRxDataReceivedAt as
             libc::c_uint).wrapping_add(usSuspendPeriod.wrapping_div(1000 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint))
            as uint32_t as uint32_t;
    // / 1000 to convert micros to millis
}
#[no_mangle]
pub unsafe extern "C" fn failsafeOnRxResume() {
    failsafeState.validRxDataReceivedAt =
        millis(); // prevent RX link down trigger, restart rx link up
    failsafeState.rxLinkState = FAILSAFE_RXLINK_UP;
    // do so while rx link is up
}
#[no_mangle]
pub unsafe extern "C" fn failsafeOnValidDataReceived() {
    failsafeState.validRxDataReceivedAt =
        millis(); // To prevent arming with no RX link
    if failsafeState.validRxDataReceivedAt.wrapping_sub(failsafeState.validRxDataFailedAt)
           > 200 as libc::c_int as libc::c_uint {
        failsafeState.rxLinkState = FAILSAFE_RXLINK_UP;
        unsetArmingDisabled(ARMING_DISABLED_RX_FAILSAFE);
    };
}
#[no_mangle]
pub unsafe extern "C" fn failsafeOnValidDataFailed() {
    setArmingDisabled(ARMING_DISABLED_RX_FAILSAFE);
    failsafeState.validRxDataFailedAt = millis();
    if failsafeState.validRxDataFailedAt.wrapping_sub(failsafeState.validRxDataReceivedAt)
           > failsafeState.rxDataFailurePeriod {
        failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN
    };
}
#[no_mangle]
pub unsafe extern "C" fn failsafeUpdateState() {
    if !failsafeIsMonitoring() { return }
    let mut receivingRxData: bool = failsafeIsReceivingRxData();
    let mut armed: bool =
        armingFlags as libc::c_int & ARMED as libc::c_int != 0;
    let mut failsafeSwitchIsOn: bool = IS_RC_MODE_ACTIVE(BOXFAILSAFE);
    let mut beeperMode: beeperMode_e = BEEPER_SILENCE;
    if failsafeSwitchIsOn as libc::c_int != 0 &&
           (*failsafeConfig()).failsafe_switch_mode as libc::c_int ==
               FAILSAFE_SWITCH_MODE_STAGE2 as libc::c_int {
        receivingRxData = 0 as libc::c_int != 0
        // force Stage2
    }
    // Beep RX lost only if we are not seeing data and we have been armed earlier
    if !receivingRxData &&
           armingFlags as libc::c_int & WAS_EVER_ARMED as libc::c_int != 0 {
        beeperMode = BEEPER_RX_LOST
    }
    let mut reprocessState: bool = false;
    loop  {
        reprocessState = 0 as libc::c_int != 0;
        match failsafeState.phase as libc::c_uint {
            0 => {
                if armed {
                    // Track throttle command below minimum time
                    if THROTTLE_HIGH as libc::c_int as libc::c_uint ==
                           calculateThrottleStatus() as libc::c_uint {
                        failsafeState.throttleLowPeriod =
                            millis().wrapping_add(((*failsafeConfig()).failsafe_throttle_low_delay
                                                       as libc::c_int *
                                                       100 as libc::c_int) as
                                                      libc::c_uint)
                    }
                    // Kill switch logic (must be independent of receivingRxData to skip PERIOD_RXDATA_FAILURE delay before disarming)
                    if failsafeSwitchIsOn as libc::c_int != 0 &&
                           (*failsafeConfig()).failsafe_switch_mode as
                               libc::c_int ==
                               FAILSAFE_SWITCH_MODE_KILL as libc::c_int {
                        // KillswitchEvent: failsafe switch is configured as KILL switch and is switched ON
                        failsafeActivate(); // skip auto-landing procedure
                        failsafeState.phase =
                            FAILSAFE_LANDED; // require 1 seconds of valid rxData
                        failsafeState.receivingRxDataPeriodPreset =
                            (1 as libc::c_int * 1000 as libc::c_int) as
                                uint32_t;
                        reprocessState = 1 as libc::c_int != 0
                    } else if !receivingRxData {
                        if millis() > failsafeState.throttleLowPeriod &&
                               (*failsafeConfig()).failsafe_procedure as
                                   libc::c_int !=
                                   FAILSAFE_PROCEDURE_GPS_RESCUE as
                                       libc::c_int {
                            // JustDisarm: throttle was LOW for at least 'failsafe_throttle_low_delay' seconds
                            failsafeActivate();
                            // require 3 seconds of valid rxData
                            failsafeState.phase =
                                FAILSAFE_LANDED; // skip auto-landing procedure
                            failsafeState.receivingRxDataPeriodPreset =
                                (3 as libc::c_int * 1000 as libc::c_int) as
                                    uint32_t
                        } else {
                            failsafeState.phase = FAILSAFE_RX_LOSS_DETECTED
                        }
                        reprocessState = 1 as libc::c_int != 0
                    }
                } else {
                    // When NOT armed, show rxLinkState of failsafe switch in GUI (failsafe mode)
                    if failsafeSwitchIsOn {
                        enableFlightMode(FAILSAFE_MODE);
                    } else { disableFlightMode(FAILSAFE_MODE); }
                    // Throttle low period expired (= low long enough for JustDisarm)
                    failsafeState.throttleLowPeriod =
                        0 as libc::c_int as uint32_t
                }
            }
            1 => {
                if receivingRxData {
                    failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED
                } else {
                    match (*failsafeConfig()).failsafe_procedure as
                              libc::c_int {
                        0 => {
                            // Stabilize, and set Throttle to specified level
                            failsafeActivate();
                        }
                        1 => {
                            // Drop the craft
                            failsafeActivate(); // skip auto-landing procedure
                            failsafeState.phase =
                                FAILSAFE_LANDED; // require 3 seconds of valid rxData
                            failsafeState.receivingRxDataPeriodPreset =
                                (3 as libc::c_int * 1000 as libc::c_int) as
                                    uint32_t
                        }
                        2 => {
                            failsafeActivate(); // require 30 seconds of valid rxData
                            failsafeState.phase =
                                FAILSAFE_GPS_RESCUE; // require 30 seconds of valid rxData
                            failsafeState.receivingRxDataPeriodPreset =
                                (3 as libc::c_int * 1000 as libc::c_int) as
                                    uint32_t
                        }
                        _ => { }
                    }
                } // To prevent accidently rearming by an intermittent rx link
                reprocessState = 1 as libc::c_int != 0
            }
            2 => {
                if receivingRxData {
                    failsafeState.phase =
                        FAILSAFE_RX_LOSS_RECOVERED; // set required period of valid rxData
                    reprocessState = 1 as libc::c_int != 0
                }
                if armed {
                    failsafeApplyControlInput();
                    beeperMode = BEEPER_RX_LOST_LANDING
                }
                if failsafeShouldHaveCausedLandingByNow() as libc::c_int != 0
                       || crashRecoveryModeActive() as libc::c_int != 0 ||
                       !armed {
                    failsafeState.receivingRxDataPeriodPreset =
                        (30 as libc::c_int * 1000 as libc::c_int) as uint32_t;
                    failsafeState.phase = FAILSAFE_LANDED;
                    reprocessState = 1 as libc::c_int != 0
                }
            }
            6 => {
                if receivingRxData {
                    failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                    reprocessState = 1 as libc::c_int != 0
                }
                if armed {
                    failsafeApplyControlInput();
                    beeperMode = BEEPER_RX_LOST_LANDING
                } else {
                    failsafeState.receivingRxDataPeriodPreset =
                        (30 as libc::c_int * 1000 as libc::c_int) as uint32_t;
                    failsafeState.phase = FAILSAFE_LANDED;
                    reprocessState = 1 as libc::c_int != 0
                }
            }
            3 => {
                setArmingDisabled(ARMING_DISABLED_FAILSAFE);
                disarm();
                failsafeState.receivingRxDataPeriod =
                    millis().wrapping_add(failsafeState.receivingRxDataPeriodPreset);
                failsafeState.phase = FAILSAFE_RX_LOSS_MONITORING;
                reprocessState = 1 as libc::c_int != 0
            }
            4 => {
                // Monitoring the rx link to allow rearming when it has become good for > `receivingRxDataPeriodPreset` time.
                if receivingRxData {
                    if millis() > failsafeState.receivingRxDataPeriod {
                        // rx link is good now, when arming via ARM switch, it must be OFF first
                        if !(!isUsingSticksForArming() &&
                                 IS_RC_MODE_ACTIVE(BOXARM) as libc::c_int !=
                                     0) {
                            unsetArmingDisabled(ARMING_DISABLED_FAILSAFE);
                            failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                            reprocessState = 1 as libc::c_int != 0
                        }
                    }
                } else {
                    failsafeState.receivingRxDataPeriod =
                        millis().wrapping_add(failsafeState.receivingRxDataPeriodPreset)
                }
            }
            5 => {
                // Entering IDLE with the requirement that throttle first must be at min_check for failsafe_throttle_low_delay period.
                // This is to prevent that JustDisarm is activated on the next iteration.
                // Because that would have the effect of shutting down failsafe handling on intermittent connections.
                failsafeState.throttleLowPeriod =
                    millis().wrapping_add(((*failsafeConfig()).failsafe_throttle_low_delay
                                               as libc::c_int *
                                               100 as libc::c_int) as
                                              libc::c_uint);
                failsafeState.phase = FAILSAFE_IDLE;
                failsafeState.active = 0 as libc::c_int != 0;
                disableFlightMode(FAILSAFE_MODE);
                reprocessState = 1 as libc::c_int != 0
            }
            _ => { }
        }
        if !reprocessState { break ; }
    }
    if beeperMode as libc::c_uint !=
           BEEPER_SILENCE as libc::c_int as libc::c_uint {
        beeper(beeperMode);
    };
}
