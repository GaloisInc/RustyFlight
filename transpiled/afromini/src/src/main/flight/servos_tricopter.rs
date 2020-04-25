use core;
use libc;
extern "C" {
    #[no_mangle]
    static mut servoConfig_System: servoConfig_t;
    #[no_mangle]
    fn servoMixer();
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct servoDevConfig_s {
    pub servoCenterPulse: uint16_t,
    pub servoPwmRate: uint16_t,
    pub ioTags: [ioTag_t; 8],
}
pub type servoDevConfig_t = servoDevConfig_s;
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
// These must be consecutive, see 'reversedSources'
pub type C2RustUnnamed = libc::c_uint;
pub const INPUT_SOURCE_COUNT: C2RustUnnamed = 14;
pub const INPUT_GIMBAL_ROLL: C2RustUnnamed = 13;
pub const INPUT_GIMBAL_PITCH: C2RustUnnamed = 12;
pub const INPUT_RC_AUX4: C2RustUnnamed = 11;
pub const INPUT_RC_AUX3: C2RustUnnamed = 10;
pub const INPUT_RC_AUX2: C2RustUnnamed = 9;
pub const INPUT_RC_AUX1: C2RustUnnamed = 8;
pub const INPUT_RC_THROTTLE: C2RustUnnamed = 7;
pub const INPUT_RC_YAW: C2RustUnnamed = 6;
pub const INPUT_RC_PITCH: C2RustUnnamed = 5;
pub const INPUT_RC_ROLL: C2RustUnnamed = 4;
pub const INPUT_STABILIZED_THROTTLE: C2RustUnnamed = 3;
pub const INPUT_STABILIZED_YAW: C2RustUnnamed = 2;
pub const INPUT_STABILIZED_PITCH: C2RustUnnamed = 1;
pub const INPUT_STABILIZED_ROLL: C2RustUnnamed = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct servoConfig_s {
    pub dev: servoDevConfig_t,
    pub servo_lowpass_freq: uint16_t,
    pub tri_unarmed_servo: uint8_t,
    pub channelForwardingStartChannel: uint8_t,
}
pub type servoConfig_t = servoConfig_s;
#[no_mangle]
pub static mut inputSource_e: C2RustUnnamed = INPUT_STABILIZED_ROLL;
#[inline]
unsafe extern "C" fn servoConfig() -> *const servoConfig_t {
    return &mut servoConfig_System;
}
// lowpass servo filter frequency selection; 1/1000ths of loop freq
// send tail servo correction pulses even when unarmed
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
pub unsafe extern "C" fn servosTricopterIsEnabledServoUnarmed() -> bool {
    return (*servoConfig()).tri_unarmed_servo != 0;
}
#[no_mangle]
pub unsafe extern "C" fn servosTricopterMixer() { servoMixer(); }
// tricopter specific
#[no_mangle]
pub unsafe extern "C" fn servosTricopterInit() { }
// USE_SERVOS
