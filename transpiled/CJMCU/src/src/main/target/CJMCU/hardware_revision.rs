use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_TypeDef {
    pub CRL: uint32_t,
    pub CRH: uint32_t,
    pub IDR: uint32_t,
    pub ODR: uint32_t,
    pub BSRR: uint32_t,
    pub BRR: uint32_t,
    pub LCKR: uint32_t,
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
// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
pub type ioTag_t = uint8_t;
pub type cjmcuHardwareRevision_t = libc::c_uint;
pub const REV_2: cjmcuHardwareRevision_t = 2;
pub const REV_1: cjmcuHardwareRevision_t = 1;
pub const UNKNOWN: cjmcuHardwareRevision_t = 0;
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
pub static mut hardwareRevision: uint8_t = UNKNOWN as libc::c_int as uint8_t;
#[no_mangle]
pub unsafe extern "C" fn detectHardwareRevision() {
    if (*((0x40000000 as libc::c_int as
               uint32_t).wrapping_add(0x10000 as libc::c_int as
                                          libc::c_uint).wrapping_add(0x1000 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint)
              as *mut GPIO_TypeDef)).IDR &
           0x8000 as libc::c_int as uint16_t as libc::c_uint != 0 {
        hardwareRevision = REV_2 as libc::c_int as uint8_t
    } else { hardwareRevision = REV_1 as libc::c_int as uint8_t };
}
#[no_mangle]
pub unsafe extern "C" fn updateHardwareRevision() { }
#[no_mangle]
pub unsafe extern "C" fn selectMPUIntExtiConfigByHardwareRevision()
 -> ioTag_t {
    return 0 as libc::c_int as ioTag_t;
}
