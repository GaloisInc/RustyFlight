use ::libc;
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
pub static mut targetName: *const libc::c_char =
    b"OMNINXT7\x00" as *const u8 as *const libc::c_char;
#[no_mangle]
pub static mut shortGitRevision: *const libc::c_char =
    b"cabb13d30\x00" as *const u8 as *const libc::c_char;
#[no_mangle]
pub static mut buildDate: *const libc::c_char =
    b"Jun 10 2020\x00" as *const u8 as *const libc::c_char;
#[no_mangle]
pub static mut buildTime: *const libc::c_char =
    b"10:12:49\x00" as *const u8 as *const libc::c_char;
