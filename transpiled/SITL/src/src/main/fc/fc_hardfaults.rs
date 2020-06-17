use core;
use libc;
extern "C" {
    #[no_mangle]
    fn delay(ms: timeMs_t);
    #[no_mangle]
    static mut systemState: uint8_t;
    #[no_mangle]
    fn stopMotors();
}
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
// millisecond time
pub type timeMs_t = uint32_t;
pub type C2RustUnnamed = libc::c_uint;
pub const SYSTEM_STATE_READY: C2RustUnnamed = 128;
pub const SYSTEM_STATE_TRANSPONDER_ENABLED: C2RustUnnamed = 8;
pub const SYSTEM_STATE_MOTORS_READY: C2RustUnnamed = 4;
pub const SYSTEM_STATE_SENSORS_READY: C2RustUnnamed = 2;
pub const SYSTEM_STATE_CONFIG_LOADED: C2RustUnnamed = 1;
pub const SYSTEM_STATE_INITIALISING: C2RustUnnamed = 0;
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
#[no_mangle]
pub unsafe extern "C" fn HardFault_Handler() {
    // fall out of the sky
    let mut requiredStateForMotors: uint8_t =
        (SYSTEM_STATE_CONFIG_LOADED as libc::c_int |
             SYSTEM_STATE_MOTORS_READY as libc::c_int) as uint8_t;
    if systemState as libc::c_int & requiredStateForMotors as libc::c_int ==
           requiredStateForMotors as libc::c_int {
        stopMotors();
    }
    loop  { delay(50i32 as timeMs_t); };
}
