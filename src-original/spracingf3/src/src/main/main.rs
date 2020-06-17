#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![feature(main)]
#![feature(asm)]
#![no_std]
use SPRACINGF3::*;
extern "C" {
    #[no_mangle]
    fn init();
    #[no_mangle]
    fn processLoopback();
    #[no_mangle]
    fn scheduler();
}
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
pub unsafe extern "C" fn run() { loop  { scheduler(); processLoopback(); }; }
#[main]
pub fn main() { unsafe { ::std::process::exit(main_0() as i32) } }


/**
 * Manually added functions
 */
// /**
//   \brief   Set Base Priority with condition
//   \details Assigns the given value to the Base Priority register only if BASEPRI masking is disabled,
//            or the new value increases the BASEPRI priority level.
//   \param [in]    basePri  Base Priority value to set
//  */
// __STATIC_FORCEINLINE void __set_BASEPRI_MAX(uint32_t basePri)
// {
//   __ASM volatile ("MSR basepri_max, %0" : : "r" (basePri) : "memory");
// }
#[inline]
fn __set_BASEPRI_MAX(_val: libc::c_int) {
    // TODO
    return 1i32 as libc::uint32_t;
}

// /**
//   \brief   Get Base Priority
//   \details Returns the current value of the Base Priority register.
//   \return               Base Priority register value
//  */
// __STATIC_FORCEINLINE uint32_t __get_BASEPRI(void)
// {
//   uint32_t result;

//   __ASM volatile ("MRS %0, basepri" : "=r" (result) );
//   return(result);
// }
#[inline]
fn __get_BASEPRI() -> libc::uint32_t {
    // TODO
    return 1i32 as libc::uint32_t;
}
