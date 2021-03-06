use core;
use libc;
extern "C" {
    #[no_mangle]
    fn pwmWriteDshotCommand(index: uint8_t, motorCount: uint8_t,
                            command: uint8_t, blocking: bool);
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
    fn systemBeep(on: bool);
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    fn getMotorCount() -> uint8_t;
    #[no_mangle]
    fn areMotorsRunning() -> bool;
    #[no_mangle]
    fn getLastDisarmTimeUs() -> timeUs_t;
    #[no_mangle]
    fn isTryingToArm() -> bool;
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
    fn warningLedEnable();
    #[no_mangle]
    fn warningLedDisable();
    #[no_mangle]
    fn warningLedRefresh();
    #[no_mangle]
    fn IS_RC_MODE_ACTIVE(boxId: boxId_e) -> bool;
    #[no_mangle]
    static mut beeperConfig_System: beeperConfig_t;
    #[no_mangle]
    fn getBatteryState() -> batteryState_e;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
// microsecond time
pub type timeUs_t = uint32_t;
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct beeperConfig_s {
    pub beeper_off_flags: uint32_t,
    pub dshotBeaconTone: uint8_t,
    pub dshotBeaconOffFlags: uint32_t,
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
pub type beeperConfig_t = beeperConfig_s;
pub type batteryState_e = libc::c_uint;
pub const BATTERY_INIT: batteryState_e = 4;
pub const BATTERY_NOT_PRESENT: batteryState_e = 3;
pub const BATTERY_CRITICAL: batteryState_e = 2;
pub const BATTERY_WARNING: batteryState_e = 1;
pub const BATTERY_OK: batteryState_e = 0;
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
pub type beeperTableEntry_t = beeperTableEntry_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct beeperTableEntry_s {
    pub mode: uint8_t,
    pub priority: uint8_t,
    pub sequence: *const uint8_t,
    pub name: *const libc::c_char,
}
#[inline]
unsafe extern "C" fn beeperConfigMutable() -> *mut beeperConfig_t {
    return &mut beeperConfig_System;
}
#[inline]
unsafe extern "C" fn beeperConfig() -> *const beeperConfig_t {
    return &mut beeperConfig_System;
}
static mut lastDshotBeaconCommandTimeUs: timeUs_t = 0;
// 0 = Highest
/* Beeper Sound Sequences: (Square wave generation)
 * Sequence must end with 0xFF or 0xFE. 0xFE repeats the sequence from
 * start when 0xFF stops the sound when it's completed.
 *
 * "Sound" Sequences are made so that 1st, 3rd, 5th.. are the delays how
 * long the beeper is on and 2nd, 4th, 6th.. are the delays how long beeper
 * is off. Delays are in milliseconds/10 (i.e., 5 => 50ms).
 */
// short fast beep
static mut beep_shortBeep: [uint8_t; 3] =
    [10i32 as uint8_t, 10i32 as uint8_t, 0xffi32 as uint8_t];
// arming beep
static mut beep_armingBeep: [uint8_t; 5] =
    [30i32 as uint8_t, 5i32 as uint8_t, 5i32 as uint8_t, 5i32 as uint8_t,
     0xffi32 as uint8_t];
// armed beep (first pause, then short beep)
static mut beep_armedBeep: [uint8_t; 5] =
    [0i32 as uint8_t, 245i32 as uint8_t, 10i32 as uint8_t, 5i32 as uint8_t,
     0xffi32 as uint8_t];
// disarming beeps
static mut beep_disarmBeep: [uint8_t; 5] =
    [15i32 as uint8_t, 5i32 as uint8_t, 15i32 as uint8_t, 5i32 as uint8_t,
     0xffi32 as uint8_t];
// beeps while stick held in disarm position (after pause)
static mut beep_disarmRepeatBeep: [uint8_t; 4] =
    [0i32 as uint8_t, 100i32 as uint8_t, 10i32 as uint8_t,
     0xffi32 as uint8_t];
// Long beep and pause after that
static mut beep_lowBatteryBeep: [uint8_t; 3] =
    [25i32 as uint8_t, 50i32 as uint8_t, 0xffi32 as uint8_t];
// critical battery beep
static mut beep_critBatteryBeep: [uint8_t; 3] =
    [50i32 as uint8_t, 2i32 as uint8_t, 0xffi32 as uint8_t];
// transmitter-signal-lost tone
static mut beep_txLostBeep: [uint8_t; 3] =
    [50i32 as uint8_t, 50i32 as uint8_t, 0xffi32 as uint8_t];
// SOS morse code:
static mut beep_sos: [uint8_t; 19] =
    [10i32 as uint8_t, 10i32 as uint8_t, 10i32 as uint8_t, 10i32 as uint8_t,
     10i32 as uint8_t, 40i32 as uint8_t, 40i32 as uint8_t, 10i32 as uint8_t,
     40i32 as uint8_t, 10i32 as uint8_t, 40i32 as uint8_t, 40i32 as uint8_t,
     10i32 as uint8_t, 10i32 as uint8_t, 10i32 as uint8_t, 10i32 as uint8_t,
     10i32 as uint8_t, 70i32 as uint8_t, 0xffi32 as uint8_t];
// Arming when GPS is fixed
static mut beep_armedGpsFix: [uint8_t; 9] =
    [5i32 as uint8_t, 5i32 as uint8_t, 15i32 as uint8_t, 5i32 as uint8_t,
     5i32 as uint8_t, 5i32 as uint8_t, 15i32 as uint8_t, 30i32 as uint8_t,
     0xffi32 as uint8_t];
// Ready beeps. When gps has fix and copter is ready to fly.
static mut beep_readyBeep: [uint8_t; 15] =
    [4i32 as uint8_t, 5i32 as uint8_t, 4i32 as uint8_t, 5i32 as uint8_t,
     8i32 as uint8_t, 5i32 as uint8_t, 15i32 as uint8_t, 5i32 as uint8_t,
     8i32 as uint8_t, 5i32 as uint8_t, 4i32 as uint8_t, 5i32 as uint8_t,
     4i32 as uint8_t, 5i32 as uint8_t, 0xffi32 as uint8_t];
// 2 fast short beeps
static mut beep_2shortBeeps: [uint8_t; 5] =
    [5i32 as uint8_t, 5i32 as uint8_t, 5i32 as uint8_t, 5i32 as uint8_t,
     0xffi32 as uint8_t];
// 2 longer beeps
static mut beep_2longerBeeps: [uint8_t; 5] =
    [20i32 as uint8_t, 15i32 as uint8_t, 35i32 as uint8_t, 5i32 as uint8_t,
     0xffi32 as uint8_t];
// 3 beeps
static mut beep_gyroCalibrated: [uint8_t; 7] =
    [20i32 as uint8_t, 10i32 as uint8_t, 20i32 as uint8_t, 10i32 as uint8_t,
     20i32 as uint8_t, 10i32 as uint8_t, 0xffi32 as uint8_t];
// Cam connection opened
static mut beep_camOpenBeep: [uint8_t; 6] =
    [5i32 as uint8_t, 15i32 as uint8_t, 10i32 as uint8_t, 15i32 as uint8_t,
     20i32 as uint8_t, 0xffi32 as uint8_t];
// Cam connection close
static mut beep_camCloseBeep: [uint8_t; 4] =
    [10i32 as uint8_t, 8i32 as uint8_t, 5i32 as uint8_t, 0xffi32 as uint8_t];
// RC Smoothing filter not initialized - 3 short + 1 long
static mut beep_rcSmoothingInitFail: [uint8_t; 9] =
    [10i32 as uint8_t, 10i32 as uint8_t, 10i32 as uint8_t, 10i32 as uint8_t,
     10i32 as uint8_t, 10i32 as uint8_t, 50i32 as uint8_t, 25i32 as uint8_t,
     0xffi32 as uint8_t];
// array used for variable # of beeps (reporting GPS sat count, etc)
static mut beep_multiBeeps: [uint8_t; 65] = [0; 65];
// Beeper off = 0 Beeper on = 1
static mut beeperIsOn: uint8_t = 0i32 as uint8_t;
// Place in current sequence
static mut beeperPos: uint16_t = 0i32 as uint16_t;
// Time when beeper routine must act next time
static mut beeperNextToggleTime: uint32_t = 0i32 as uint32_t;
// Time of last arming beep in microseconds (for blackbox)
static mut armingBeepTimeMicros: uint32_t = 0i32 as uint32_t;
static mut beeperTable: [beeperTableEntry_t; 24] =
    unsafe {
        [{
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_GYRO_CALIBRATED as libc::c_int
                                            as uint8_t,
                                    priority: 0i32 as uint8_t,
                                    sequence: beep_gyroCalibrated.as_ptr(),
                                    name:
                                        b"GYRO_CALIBRATED\x00" as *const u8 as
                                            *const libc::c_char,};
             init
         },
         {
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_RX_LOST as libc::c_int as
                                            uint8_t,
                                    priority: 1i32 as uint8_t,
                                    sequence: beep_txLostBeep.as_ptr(),
                                    name:
                                        b"RX_LOST\x00" as *const u8 as
                                            *const libc::c_char,};
             init
         },
         {
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_RX_LOST_LANDING as libc::c_int
                                            as uint8_t,
                                    priority: 2i32 as uint8_t,
                                    sequence: beep_sos.as_ptr(),
                                    name:
                                        b"RX_LOST_LANDING\x00" as *const u8 as
                                            *const libc::c_char,};
             init
         },
         {
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_DISARMING as libc::c_int as
                                            uint8_t,
                                    priority: 3i32 as uint8_t,
                                    sequence: beep_disarmBeep.as_ptr(),
                                    name:
                                        b"DISARMING\x00" as *const u8 as
                                            *const libc::c_char,};
             init
         },
         {
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_ARMING as libc::c_int as
                                            uint8_t,
                                    priority: 4i32 as uint8_t,
                                    sequence: beep_armingBeep.as_ptr(),
                                    name:
                                        b"ARMING\x00" as *const u8 as
                                            *const libc::c_char,};
             init
         },
         {
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_ARMING_GPS_FIX as libc::c_int
                                            as uint8_t,
                                    priority: 5i32 as uint8_t,
                                    sequence: beep_armedGpsFix.as_ptr(),
                                    name:
                                        b"ARMING_GPS_FIX\x00" as *const u8 as
                                            *const libc::c_char,};
             init
         },
         {
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_BAT_CRIT_LOW as libc::c_int as
                                            uint8_t,
                                    priority: 6i32 as uint8_t,
                                    sequence: beep_critBatteryBeep.as_ptr(),
                                    name:
                                        b"BAT_CRIT_LOW\x00" as *const u8 as
                                            *const libc::c_char,};
             init
         },
         {
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_BAT_LOW as libc::c_int as
                                            uint8_t,
                                    priority: 7i32 as uint8_t,
                                    sequence: beep_lowBatteryBeep.as_ptr(),
                                    name:
                                        b"BAT_LOW\x00" as *const u8 as
                                            *const libc::c_char,};
             init
         },
         {
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_GPS_STATUS as libc::c_int as
                                            uint8_t,
                                    priority: 8i32 as uint8_t,
                                    sequence:
                                        beep_multiBeeps.as_ptr() as *mut _,
                                    name:
                                        b"GPS_STATUS\x00" as *const u8 as
                                            *const libc::c_char,};
             init
         },
         {
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_RX_SET as libc::c_int as
                                            uint8_t,
                                    priority: 9i32 as uint8_t,
                                    sequence: beep_shortBeep.as_ptr(),
                                    name:
                                        b"RX_SET\x00" as *const u8 as
                                            *const libc::c_char,};
             init
         },
         {
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_ACC_CALIBRATION as libc::c_int
                                            as uint8_t,
                                    priority: 10i32 as uint8_t,
                                    sequence: beep_2shortBeeps.as_ptr(),
                                    name:
                                        b"ACC_CALIBRATION\x00" as *const u8 as
                                            *const libc::c_char,};
             init
         },
         {
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_ACC_CALIBRATION_FAIL as
                                            libc::c_int as uint8_t,
                                    priority: 11i32 as uint8_t,
                                    sequence: beep_2longerBeeps.as_ptr(),
                                    name:
                                        b"ACC_CALIBRATION_FAIL\x00" as
                                            *const u8 as
                                            *const libc::c_char,};
             init
         },
         {
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_READY_BEEP as libc::c_int as
                                            uint8_t,
                                    priority: 12i32 as uint8_t,
                                    sequence: beep_readyBeep.as_ptr(),
                                    name:
                                        b"READY_BEEP\x00" as *const u8 as
                                            *const libc::c_char,};
             init
         },
         {
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_MULTI_BEEPS as libc::c_int as
                                            uint8_t,
                                    priority: 13i32 as uint8_t,
                                    sequence:
                                        beep_multiBeeps.as_ptr() as *mut _,
                                    name:
                                        b"MULTI_BEEPS\x00" as *const u8 as
                                            *const libc::c_char,};
             init
         },
         {
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_DISARM_REPEAT as libc::c_int as
                                            uint8_t,
                                    priority: 14i32 as uint8_t,
                                    sequence: beep_disarmRepeatBeep.as_ptr(),
                                    name:
                                        b"DISARM_REPEAT\x00" as *const u8 as
                                            *const libc::c_char,};
             init
         },
         {
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_ARMED as libc::c_int as
                                            uint8_t,
                                    priority: 15i32 as uint8_t,
                                    sequence: beep_armedBeep.as_ptr(),
                                    name:
                                        b"ARMED\x00" as *const u8 as
                                            *const libc::c_char,};
             init
         },
         {
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_SYSTEM_INIT as libc::c_int as
                                            uint8_t,
                                    priority: 16i32 as uint8_t,
                                    sequence: 0 as *const uint8_t,
                                    name:
                                        b"SYSTEM_INIT\x00" as *const u8 as
                                            *const libc::c_char,};
             init
         },
         {
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_USB as libc::c_int as uint8_t,
                                    priority: 17i32 as uint8_t,
                                    sequence: 0 as *const uint8_t,
                                    name:
                                        b"ON_USB\x00" as *const u8 as
                                            *const libc::c_char,};
             init
         },
         {
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_BLACKBOX_ERASE as libc::c_int
                                            as uint8_t,
                                    priority: 18i32 as uint8_t,
                                    sequence: beep_2shortBeeps.as_ptr(),
                                    name:
                                        b"BLACKBOX_ERASE\x00" as *const u8 as
                                            *const libc::c_char,};
             init
         },
         {
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_CRASH_FLIP_MODE as libc::c_int
                                            as uint8_t,
                                    priority: 19i32 as uint8_t,
                                    sequence: beep_2longerBeeps.as_ptr(),
                                    name:
                                        b"CRASH FLIP\x00" as *const u8 as
                                            *const libc::c_char,};
             init
         },
         {
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_CAM_CONNECTION_OPEN as
                                            libc::c_int as uint8_t,
                                    priority: 20i32 as uint8_t,
                                    sequence: beep_camOpenBeep.as_ptr(),
                                    name:
                                        b"CAM_CONNECTION_OPEN\x00" as
                                            *const u8 as
                                            *const libc::c_char,};
             init
         },
         {
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_CAM_CONNECTION_CLOSE as
                                            libc::c_int as uint8_t,
                                    priority: 21i32 as uint8_t,
                                    sequence: beep_camCloseBeep.as_ptr(),
                                    name:
                                        b"CAM_CONNECTION_CLOSED\x00" as
                                            *const u8 as
                                            *const libc::c_char,};
             init
         },
         {
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_RC_SMOOTHING_INIT_FAIL as
                                            libc::c_int as uint8_t,
                                    priority: 22i32 as uint8_t,
                                    sequence:
                                        beep_rcSmoothingInitFail.as_ptr(),
                                    name:
                                        b"RC_SMOOTHING_INIT_FAIL\x00" as
                                            *const u8 as
                                            *const libc::c_char,};
             init
         },
         {
             let mut init =
                 beeperTableEntry_s{mode:
                                        BEEPER_ALL as libc::c_int as uint8_t,
                                    priority: 23i32 as uint8_t,
                                    sequence: 0 as *const uint8_t,
                                    name:
                                        b"ALL\x00" as *const u8 as
                                            *const libc::c_char,};
             init
         }]
    };
static mut currentBeeperEntry: *const beeperTableEntry_t =
    0 as *const beeperTableEntry_t;
/*
 * Called to activate/deactivate beeper, using the given "BEEPER_..." value.
 * This function returns immediately (does not block).
 */
#[no_mangle]
pub unsafe extern "C" fn beeper(mut mode: beeperMode_e) {
    if mode as libc::c_uint == BEEPER_SILENCE as libc::c_int as libc::c_uint
           ||
           (*beeperConfigMutable()).beeper_off_flags &
               (1i32 << BEEPER_USB as libc::c_int - 1i32) as libc::c_uint != 0
               &&
               getBatteryState() as libc::c_uint ==
                   BATTERY_NOT_PRESENT as libc::c_int as libc::c_uint {
        beeperSilence();
        return
    }
    let mut selectedCandidate: *const beeperTableEntry_t =
        0 as *const beeperTableEntry_t;
    let mut i: uint32_t = 0i32 as uint32_t;
    while (i as libc::c_ulong) <
              (::core::mem::size_of::<[beeperTableEntry_t; 24]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<beeperTableEntry_t>()
                                                   as libc::c_ulong) {
        let mut candidate: *const beeperTableEntry_t =
            &*beeperTable.as_ptr().offset(i as isize) as
                *const beeperTableEntry_t;
        if (*candidate).mode as libc::c_uint != mode as libc::c_uint {
            i = i.wrapping_add(1)
        } else if currentBeeperEntry.is_null() {
            selectedCandidate = candidate;
            break ;
        } else {
            if ((*candidate).priority as libc::c_int) <
                   (*currentBeeperEntry).priority as libc::c_int {
                selectedCandidate = candidate
            }
            break ;
        }
    }
    if selectedCandidate.is_null() { return }
    currentBeeperEntry = selectedCandidate;
    beeperPos = 0i32 as uint16_t;
    beeperNextToggleTime = 0i32 as uint32_t;
}
#[no_mangle]
pub unsafe extern "C" fn beeperSilence() {
    systemBeep(0i32 != 0);
    warningLedDisable();
    warningLedRefresh();
    beeperIsOn = 0i32 as uint8_t;
    beeperNextToggleTime = 0i32 as uint32_t;
    beeperPos = 0i32 as uint16_t;
    currentBeeperEntry = 0 as *const beeperTableEntry_t;
}
/*
 * Emits the given number of 20ms beeps (with 200ms spacing).
 * This function returns immediately (does not block).
 */
#[no_mangle]
pub unsafe extern "C" fn beeperConfirmationBeeps(mut beepCount: uint8_t) {
    let mut i: uint32_t = 0i32 as uint32_t;
    let mut cLimit: uint32_t = (beepCount as libc::c_int * 2i32) as uint32_t;
    if cLimit > 64i32 as libc::c_uint { cLimit = 64i32 as uint32_t }
    loop  {
        let fresh0 = i;
        i = i.wrapping_add(1);
        beep_multiBeeps[fresh0 as usize] = 2i32 as uint8_t;
        let fresh1 = i;
        i = i.wrapping_add(1);
        beep_multiBeeps[fresh1 as usize] = 20i32 as uint8_t;
        if !(i < cLimit) { break ; }
    }
    beep_multiBeeps[i as usize] = 0xffi32 as uint8_t;
    beeper(BEEPER_MULTI_BEEPS);
}
#[no_mangle]
pub unsafe extern "C" fn beeperWarningBeeps(mut beepCount: uint8_t) {
    let mut longBeepCount: uint8_t =
        (beepCount as libc::c_int / 5i32) as uint8_t;
    let mut shortBeepCount: uint8_t =
        (beepCount as libc::c_int % 5i32) as uint8_t;
    let mut i: libc::c_uint = 0i32 as libc::c_uint;
    let mut count: libc::c_uint = 0i32 as libc::c_uint;
    while i < (64i32 - 1i32) as libc::c_uint && count < 5i32 as libc::c_uint {
        let fresh2 = i;
        i = i.wrapping_add(1);
        beep_multiBeeps[fresh2 as usize] = (50i32 / 10i32) as uint8_t;
        count = count.wrapping_add(1);
        if count < 5i32 as libc::c_uint {
            let fresh3 = i;
            i = i.wrapping_add(1);
            beep_multiBeeps[fresh3 as usize] = (50i32 / 10i32) as uint8_t
        } else {
            let fresh4 = i;
            i = i.wrapping_add(1);
            beep_multiBeeps[fresh4 as usize] = (500i32 / 10i32) as uint8_t
        }
    }
    while i < (64i32 - 1i32) as libc::c_uint &&
              longBeepCount as libc::c_int > 0i32 {
        let fresh5 = i;
        i = i.wrapping_add(1);
        beep_multiBeeps[fresh5 as usize] = (250i32 / 10i32) as uint8_t;
        longBeepCount = longBeepCount.wrapping_sub(1);
        if longBeepCount as libc::c_int > 0i32 {
            let fresh6 = i;
            i = i.wrapping_add(1);
            beep_multiBeeps[fresh6 as usize] = (250i32 / 10i32) as uint8_t
        } else {
            let fresh7 = i;
            i = i.wrapping_add(1);
            beep_multiBeeps[fresh7 as usize] = (500i32 / 10i32) as uint8_t
        }
    }
    while i < (64i32 - 1i32) as libc::c_uint &&
              shortBeepCount as libc::c_int > 0i32 {
        let fresh8 = i;
        i = i.wrapping_add(1);
        beep_multiBeeps[fresh8 as usize] = (50i32 / 10i32) as uint8_t;
        shortBeepCount = shortBeepCount.wrapping_sub(1);
        if shortBeepCount as libc::c_int > 0i32 {
            let fresh9 = i;
            i = i.wrapping_add(1);
            beep_multiBeeps[fresh9 as usize] = (250i32 / 10i32) as uint8_t
        }
    }
    beep_multiBeeps[i as usize] = 0xffi32 as uint8_t;
    beeper(BEEPER_MULTI_BEEPS);
}
/*
 * Beeper handler function to be called periodically in loop. Updates beeper
 * state via time schedule.
 */
#[no_mangle]
pub unsafe extern "C" fn beeperUpdate(mut currentTimeUs: timeUs_t) {
    // If beeper option from AUX switch has been selected
    if IS_RC_MODE_ACTIVE(BOXBEEPERON) { beeper(BEEPER_RX_SET); }
    // Beeper routine doesn't need to update if there aren't any sounds ongoing
    if currentBeeperEntry.is_null() { return }
    if beeperNextToggleTime > currentTimeUs { return }
    if beeperIsOn == 0 {
        beeperIsOn = 1i32 as uint8_t;
        if !areMotorsRunning() &&
               ((*currentBeeperEntry).mode as libc::c_int ==
                    BEEPER_RX_SET as libc::c_int &&
                    (*beeperConfig()).dshotBeaconOffFlags &
                        (1i32 << BEEPER_RX_SET as libc::c_int - 1i32) as
                            libc::c_uint == 0 ||
                    (*currentBeeperEntry).mode as libc::c_int ==
                        BEEPER_RX_LOST as libc::c_int &&
                        (*beeperConfig()).dshotBeaconOffFlags &
                            (1i32 << BEEPER_RX_LOST as libc::c_int - 1i32) as
                                libc::c_uint == 0) {
            if currentTimeUs.wrapping_sub(getLastDisarmTimeUs()) >
                   1200000i32 as libc::c_uint && !isTryingToArm() {
                lastDshotBeaconCommandTimeUs = currentTimeUs;
                pwmWriteDshotCommand(255i32 as uint8_t, getMotorCount(),
                                     (*beeperConfig()).dshotBeaconTone,
                                     0i32 != 0);
            }
        }
        if *(*currentBeeperEntry).sequence.offset(beeperPos as isize) as
               libc::c_int != 0i32 {
            if (*beeperConfigMutable()).beeper_off_flags &
                   (1i32 << (*currentBeeperEntry).mode as libc::c_int - 1i32)
                       as libc::c_uint == 0 {
                systemBeep(1i32 != 0);
            }
            warningLedEnable();
            warningLedRefresh();
            // if this was arming beep then mark time (for blackbox)
            if beeperPos as libc::c_int == 0i32 &&
                   ((*currentBeeperEntry).mode as libc::c_int ==
                        BEEPER_ARMING as libc::c_int ||
                        (*currentBeeperEntry).mode as libc::c_int ==
                            BEEPER_ARMING_GPS_FIX as libc::c_int) {
                armingBeepTimeMicros = micros()
            }
        }
    } else {
        beeperIsOn = 0i32 as uint8_t;
        if *(*currentBeeperEntry).sequence.offset(beeperPos as isize) as
               libc::c_int != 0i32 {
            systemBeep(0i32 != 0);
            warningLedDisable();
            warningLedRefresh();
        }
    }
    beeperProcessCommand(currentTimeUs);
}
/*
 * Calculates array position when next to change beeper state is due.
 */
unsafe extern "C" fn beeperProcessCommand(mut currentTimeUs: timeUs_t) {
    if *(*currentBeeperEntry).sequence.offset(beeperPos as isize) as
           libc::c_int == 0xfei32 {
        beeperPos = 0i32 as uint16_t
    } else if *(*currentBeeperEntry).sequence.offset(beeperPos as isize) as
                  libc::c_int == 0xffi32 {
        beeperSilence();
    } else {
        // Otherwise advance the sequence and calculate next toggle time
        beeperNextToggleTime =
            currentTimeUs.wrapping_add((1000i32 * 10i32 *
                                            *(*currentBeeperEntry).sequence.offset(beeperPos
                                                                                       as
                                                                                       isize)
                                                as libc::c_int) as
                                           libc::c_uint);
        beeperPos = beeperPos.wrapping_add(1)
    };
}
/*
 * Returns the time that the last arming beep occurred (in system-uptime
 * microseconds).  This is fetched and logged by blackbox.
 */
#[no_mangle]
pub unsafe extern "C" fn getArmingBeepTimeMicros() -> uint32_t {
    return armingBeepTimeMicros;
}
/*
 * Returns the 'beeperMode_e' value for the given beeper-table index,
 * or BEEPER_SILENCE if none.
 */
#[no_mangle]
pub unsafe extern "C" fn beeperModeForTableIndex(mut idx: libc::c_int)
 -> beeperMode_e {
    return if idx >= 0i32 &&
                  idx <
                      (::core::mem::size_of::<[beeperTableEntry_t; 24]>() as
                           libc::c_ulong).wrapping_div(::core::mem::size_of::<beeperTableEntry_t>()
                                                           as libc::c_ulong)
                          as libc::c_int {
               beeperTable[idx as usize].mode as libc::c_int
           } else { BEEPER_SILENCE as libc::c_int } as beeperMode_e;
}
/*
 * Returns the binary mask for the 'beeperMode_e' value corresponding to a given
 * beeper-table index, or 0 if the beeperMode is BEEPER_SILENCE.
 */
#[no_mangle]
pub unsafe extern "C" fn beeperModeMaskForTableIndex(mut idx: libc::c_int)
 -> uint32_t {
    let mut beeperMode: beeperMode_e = beeperModeForTableIndex(idx);
    if beeperMode as libc::c_uint ==
           BEEPER_SILENCE as libc::c_int as libc::c_uint {
        return 0i32 as uint32_t
    }
    return (1i32 <<
                (beeperMode as
                     libc::c_uint).wrapping_sub(1i32 as libc::c_uint)) as
               uint32_t;
}
/*
 * Returns the name for the given beeper-table index, or NULL if none.
 */
#[no_mangle]
pub unsafe extern "C" fn beeperNameForTableIndex(mut idx: libc::c_int)
 -> *const libc::c_char {
    return if idx >= 0i32 &&
                  idx <
                      (::core::mem::size_of::<[beeperTableEntry_t; 24]>() as
                           libc::c_ulong).wrapping_div(::core::mem::size_of::<beeperTableEntry_t>()
                                                           as libc::c_ulong)
                          as libc::c_int {
               beeperTable[idx as usize].name
           } else { 0 as *const libc::c_char };
}
/*
 * Returns the number of entries in the beeper-sounds table.
 */
#[no_mangle]
pub unsafe extern "C" fn beeperTableEntryCount() -> libc::c_int {
    return (::core::mem::size_of::<[beeperTableEntry_t; 24]>() as
                libc::c_ulong).wrapping_div(::core::mem::size_of::<beeperTableEntry_t>()
                                                as libc::c_ulong) as
               libc::c_int;
}
/*
 * Returns true if the beeper is on, false otherwise
 */
#[no_mangle]
pub unsafe extern "C" fn isBeeperOn() -> bool { return beeperIsOn != 0; }
#[no_mangle]
pub unsafe extern "C" fn getLastDshotBeaconCommandTimeUs() -> timeUs_t {
    return lastDshotBeaconCommandTimeUs;
}
