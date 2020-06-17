#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![feature(main)]
#![no_std]
use sitl::*;
extern "C" {
    #[no_mangle]
    fn delayMicroseconds_real(us: uint32_t);
    #[no_mangle]
    fn init();
    #[no_mangle]
    fn processLoopback();
    #[no_mangle]
    fn scheduler();
}
pub type __uint32_t = libc::c_uint;
pub type uint32_t = __uint32_t;
#[no_mangle]
pub static mut SystemCoreClock: uint32_t = 0;
unsafe fn main_0() -> libc::c_int { init(); run(); return 0i32; }
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
pub unsafe extern "C" fn run() {
    loop  {
        scheduler();
        processLoopback();
        delayMicroseconds_real(50i32 as uint32_t);
        // max rate 20kHz
    };
}
#[main]
pub fn main() { unsafe { ::std::process::exit(main_0() as i32) } }
