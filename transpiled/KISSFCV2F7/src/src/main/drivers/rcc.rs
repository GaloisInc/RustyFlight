use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
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
pub type rccPeriphTag_t = uint8_t;
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
pub unsafe extern "C" fn RCC_ClockCmd(mut periphTag: rccPeriphTag_t,
                                      mut NewState: FunctionalState) {
    let mut tag: libc::c_int = periphTag as libc::c_int >> 5 as libc::c_int;
    let mut mask: uint32_t =
        ((1 as libc::c_int) <<
             (periphTag as libc::c_int & 0x1f as libc::c_int)) as uint32_t;
}
#[no_mangle]
pub unsafe extern "C" fn RCC_ResetCmd(mut periphTag: rccPeriphTag_t,
                                      mut NewState: FunctionalState) {
    let mut tag: libc::c_int = periphTag as libc::c_int >> 5 as libc::c_int;
    let mut mask: uint32_t =
        ((1 as libc::c_int) <<
             (periphTag as libc::c_int & 0x1f as libc::c_int)) as uint32_t;
}
