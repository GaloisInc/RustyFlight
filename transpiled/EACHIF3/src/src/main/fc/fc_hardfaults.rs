use ::libc;
extern "C" {
    #[no_mangle]
    fn ledSet(led: libc::c_int, state: bool);
    #[no_mangle]
    fn ledToggle(led: libc::c_int);
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
#[no_mangle]
pub unsafe extern "C" fn HardFault_Handler() {
    ledSet(2 as libc::c_int, 1 as libc::c_int != 0);
    // fall out of the sky
    let mut requiredStateForMotors: uint8_t =
        (SYSTEM_STATE_CONFIG_LOADED as libc::c_int |
             SYSTEM_STATE_MOTORS_READY as libc::c_int) as uint8_t;
    if systemState as libc::c_int & requiredStateForMotors as libc::c_int ==
           requiredStateForMotors as libc::c_int {
        stopMotors();
    }
    ledSet(1 as libc::c_int, 0 as libc::c_int != 0);
    ledSet(0 as libc::c_int, 0 as libc::c_int != 0);
    loop  {
        delay(50 as libc::c_int as timeMs_t);
        ledToggle(2 as libc::c_int);
    };
}
