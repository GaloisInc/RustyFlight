use core;
use libc;
extern "C" {
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    fn millis() -> timeMs_t;
}
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int32_t = __int32_t;
pub type uint32_t = __uint32_t;
// millisecond time
pub type timeMs_t = uint32_t;
// microsecond time
pub type timeUs_t = uint32_t;
pub type warningLedState_e = libc::c_uint;
pub const WARNING_LED_FLASH: warningLedState_e = 2;
pub const WARNING_LED_ON: warningLedState_e = 1;
pub const WARNING_LED_OFF: warningLedState_e = 0;
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
static mut warningLedTimer: uint32_t = 0i32 as uint32_t;
static mut warningLedState: warningLedState_e = WARNING_LED_OFF;
#[no_mangle]
pub unsafe extern "C" fn warningLedResetTimer() {
    let mut now: uint32_t = millis();
    warningLedTimer = now.wrapping_add(500000i32 as libc::c_uint);
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
pub unsafe extern "C" fn warningLedEnable() {
    warningLedState = WARNING_LED_ON;
}
#[no_mangle]
pub unsafe extern "C" fn warningLedDisable() {
    warningLedState = WARNING_LED_OFF;
}
#[no_mangle]
pub unsafe extern "C" fn warningLedFlash() {
    warningLedState = WARNING_LED_FLASH;
}
#[no_mangle]
pub unsafe extern "C" fn warningLedRefresh() {
    match warningLedState as libc::c_uint { 0 | 1 | 2 | _ => { } }
    let mut now: uint32_t = micros();
    warningLedTimer = now.wrapping_add(500000i32 as libc::c_uint);
}
#[no_mangle]
pub unsafe extern "C" fn warningLedUpdate() {
    let mut now: uint32_t = micros();
    if (now.wrapping_sub(warningLedTimer) as int32_t) < 0i32 { return }
    warningLedRefresh();
}
