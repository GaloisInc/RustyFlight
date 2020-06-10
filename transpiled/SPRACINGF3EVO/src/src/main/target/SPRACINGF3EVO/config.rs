use ::libc;
extern "C" {
    #[no_mangle]
    static mut sdcardConfig_System: sdcardConfig_t;
}
pub type __uint8_t = libc::c_uchar;
pub type uint8_t = __uint8_t;
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
pub struct sdcardConfig_s {
    pub useDma: uint8_t,
    pub enabled: uint8_t,
    pub device: uint8_t,
    pub cardDetectTag: ioTag_t,
    pub chipSelectTag: ioTag_t,
    pub cardDetectInverted: uint8_t,
    pub dmaIdentifier: uint8_t,
    pub dmaChannel: uint8_t,
}
pub type sdcardConfig_t = sdcardConfig_s;
#[inline]
unsafe extern "C" fn sdcardConfigMutable() -> *mut sdcardConfig_t {
    return &mut sdcardConfig_System;
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
pub unsafe extern "C" fn targetConfiguration() {
    // Temporary workaround: Disable SDCard DMA by default since it causes errors on this target
    (*sdcardConfigMutable()).useDma = 0 as libc::c_int as uint8_t;
}
