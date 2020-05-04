use ::libc;
extern "C" {
    #[no_mangle]
    fn micros() -> timeUs_t;
}
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int32_t = __int32_t;
pub type uint32_t = __uint32_t;
pub type timeUs_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct dispatchEntry_s {
    pub dispatch: Option<dispatchFunc>,
    pub delayedUntil: uint32_t,
    pub next: *mut dispatchEntry_s,
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
pub type dispatchFunc = unsafe extern "C" fn(_: *mut dispatchEntry_s) -> ();
pub type dispatchEntry_t = dispatchEntry_s;
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
// expand to t if bit is 1, f when bit is 0. Other bit values are not supported
// Expand all argumens and call macro with them. When expansion of some argument contains ',', it will be passed as multiple arguments
// #define TAKE3(_1,_2,_3) CONCAT3(_1,_2,_3)
// #define MULTI2 A,B
// PP_CALL(TAKE3, MULTI2, C) expands to ABC
/*
http://resnet.uoregon.edu/~gurney_j/jmpc/bitwise.html
*/
/*
 * https://groups.google.com/forum/?hl=en#!msg/comp.lang.c/attFnqwhvGk/sGBKXvIkY3AJ
 * Return (v ? floor(log2(v)) : 0) when 0 <= v < 1<<[8, 16, 32, 64].
 * Inefficient algorithm, intended for compile-time constants.
 */
// non ISO variant from linux kernel; checks ptr type, but triggers 'ISO C forbids braced-groups within expressions [-Wpedantic]'
//  __extension__ is here to disable this warning
#[inline]
unsafe extern "C" fn cmp32(mut a: uint32_t, mut b: uint32_t) -> int32_t {
    return a.wrapping_sub(b) as int32_t;
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
static mut head: *mut dispatchEntry_t =
    0 as *const dispatchEntry_t as *mut dispatchEntry_t;
static mut dispatchEnabled: bool = 0 as libc::c_int != 0;
#[no_mangle]
pub unsafe extern "C" fn dispatchIsEnabled() -> bool {
    return dispatchEnabled;
}
#[no_mangle]
pub unsafe extern "C" fn dispatchEnable() {
    dispatchEnabled = 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn dispatchProcess(mut currentTime: uint32_t) {
    let mut p: *mut *mut dispatchEntry_t = &mut head;
    while !(*p).is_null() {
        if cmp32(currentTime, (**p).delayedUntil) < 0 as libc::c_int {
            break ;
        }
        // unlink entry first, so handler can replan self
        let mut current: *mut dispatchEntry_t = *p;
        *p = (**p).next;
        Some((*current).dispatch.expect("non-null function pointer")).expect("non-null function pointer")(current);
    };
}
#[no_mangle]
pub unsafe extern "C" fn dispatchAdd(mut entry: *mut dispatchEntry_t,
                                     mut delayUs: libc::c_int) {
    let mut delayedUntil: uint32_t =
        micros().wrapping_add(delayUs as libc::c_uint);
    (*entry).delayedUntil = delayedUntil;
    let mut p: *mut *mut dispatchEntry_t = &mut head;
    while !(*p).is_null() &&
              cmp32((**p).delayedUntil, delayedUntil) < 0 as libc::c_int {
        p = &mut (**p).next
    }
    (*entry).next = *p;
    *p = entry;
}
