use core;
use libc;
extern "C" {
    #[no_mangle]
    fn scaleRange(x: libc::c_int, srcFrom: libc::c_int, srcTo: libc::c_int,
                  destFrom: libc::c_int, destTo: libc::c_int) -> libc::c_int;
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
    fn audioSetupIO();
    #[no_mangle]
    fn audioGenerateWhiteNoise();
    #[no_mangle]
    fn audioPlayTone(tone: uint8_t);
    // TONE_MIN to TONE_MAX
    #[no_mangle]
    fn audioSilence();
    #[no_mangle]
    fn IS_RC_MODE_ACTIVE(boxId: boxId_e) -> bool;
    #[no_mangle]
    static mut pidData: [pidAxisData_t; 3];
}
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
pub type C2RustUnnamed = libc::c_uint;
pub const FD_YAW: C2RustUnnamed = 2;
pub const FD_PITCH: C2RustUnnamed = 1;
pub const FD_ROLL: C2RustUnnamed = 0;
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
pub struct pidAxisData_s {
    pub P: libc::c_float,
    pub I: libc::c_float,
    pub D: libc::c_float,
    pub F: libc::c_float,
    pub Sum: libc::c_float,
}
pub type pidAxisData_t = pidAxisData_s;
pub const PID_AUDIO_PIDSUM_XY: pidAudioModes_e = 3;
pub const PID_AUDIO_PIDSUM_Y: pidAudioModes_e = 2;
pub const PID_AUDIO_PIDSUM_X: pidAudioModes_e = 1;
pub type pidAudioModes_e = libc::c_uint;
pub const PID_AUDIO_OFF: pidAudioModes_e = 0;
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
// SITL (software in the loop) simulator
// use simulatior's attitude directly
// disable this if wants to test AHRS algorithm
//#define SIMULATOR_ACC_SYNC
//#define SIMULATOR_GYRO_SYNC
//#define SIMULATOR_IMU_SYNC
//#define SIMULATOR_GYROPID_SYNC
// file name to save config
//#define USE_SOFTSERIAL1
//#define USE_SOFTSERIAL2
// I think SITL don't need this
// suppress 'no pins defined' warning
// belows are internal stuff
#[no_mangle]
pub static mut SystemCoreClock: uint32_t = 0;
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
static mut pidAudioEnabled: bool = 0i32 != 0;
static mut pidAudioMode: pidAudioModes_e = PID_AUDIO_PIDSUM_XY;
#[no_mangle]
pub unsafe extern "C" fn pidAudioInit() { audioSetupIO(); }
#[no_mangle]
pub unsafe extern "C" fn pidAudioStart() { audioGenerateWhiteNoise(); }
#[no_mangle]
pub unsafe extern "C" fn pidAudioStop() { audioSilence(); }
#[no_mangle]
pub unsafe extern "C" fn pidAudioGetMode() -> pidAudioModes_e {
    return pidAudioMode;
}
#[no_mangle]
pub unsafe extern "C" fn pidAudioSetMode(mut mode: pidAudioModes_e) {
    pidAudioMode = mode;
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
pub unsafe extern "C" fn pidAudioUpdate() {
    let mut newState: bool = IS_RC_MODE_ACTIVE(BOXPIDAUDIO);
    if pidAudioEnabled as libc::c_int != newState as libc::c_int {
        if newState { pidAudioStart(); } else { pidAudioStop(); }
        pidAudioEnabled = newState
    }
    if !pidAudioEnabled { return }
    let mut tone: uint8_t = (64i32 / 2i32 + 0i32) as uint8_t;
    match pidAudioMode as libc::c_uint {
        1 => {
            let pidSumX: uint32_t =
                ({
                     let mut _a: libc::c_float =
                         ({
                              let mut _x: libc::c_float =
                                  pidData[FD_ROLL as libc::c_int as
                                              usize].Sum;
                              if _x > 0i32 as libc::c_float {
                                  _x
                              } else { -_x }
                          });
                     let mut _b: libc::c_int = 500i32;
                     if _a < _b as libc::c_float {
                         _a
                     } else { _b as libc::c_float }
                 }) as uint32_t;
            tone =
                scaleRange(pidSumX as libc::c_int, 0i32, 500i32, 64i32 - 1i32,
                           0i32) as uint8_t
        }
        2 => {
            let pidSumY: uint32_t =
                ({
                     let mut _a: libc::c_float =
                         ({
                              let mut _x: libc::c_float =
                                  pidData[FD_PITCH as libc::c_int as
                                              usize].Sum;
                              if _x > 0i32 as libc::c_float {
                                  _x
                              } else { -_x }
                          });
                     let mut _b: libc::c_int = 500i32;
                     if _a < _b as libc::c_float {
                         _a
                     } else { _b as libc::c_float }
                 }) as uint32_t;
            tone =
                scaleRange(pidSumY as libc::c_int, 0i32, 500i32, 64i32 - 1i32,
                           0i32) as uint8_t
        }
        3 => {
            let pidSumXY: uint32_t =
                ({
                     let mut _a: libc::c_float =
                         (({
                               let mut _x: libc::c_float =
                                   pidData[FD_ROLL as libc::c_int as
                                               usize].Sum;
                               (if _x > 0i32 as libc::c_float {
                                    _x
                                } else { -_x })
                           }) +
                              ({
                                   let mut _x: libc::c_float =
                                       pidData[FD_PITCH as libc::c_int as
                                                   usize].Sum;
                                   (if _x > 0i32 as libc::c_float {
                                        _x
                                    } else { -_x })
                               })) / 2i32 as libc::c_float;
                     let mut _b: libc::c_int = 500i32;
                     if _a < _b as libc::c_float {
                         _a
                     } else { _b as libc::c_float }
                 }) as uint32_t;
            tone =
                scaleRange(pidSumXY as libc::c_int, 0i32, 500i32,
                           64i32 - 1i32, 0i32) as uint8_t
        }
        _ => { }
    }
    audioPlayTone(tone);
}
