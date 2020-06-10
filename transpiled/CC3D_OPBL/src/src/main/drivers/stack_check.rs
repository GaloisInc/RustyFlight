use ::libc;
extern "C" {
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
    static mut _estack: libc::c_char;
    // end of stack, declared in .LD file
    #[no_mangle]
    static mut _Min_Stack_Size: libc::c_char;
}
pub type __uint32_t = libc::c_uint;
pub type uint32_t = __uint32_t;
pub type intptr_t = libc::c_long;
// declared in .LD file
/*
 * The ARM processor uses a full descending stack. This means the stack pointer holds the address
 * of the last stacked item in memory. When the processor pushes a new item onto the stack,
 * it decrements the stack pointer and then writes the item to the new memory location.
 *
 *
 * RAM layout is generally as below, although some targets vary
 *
 * F1 Boards
 * RAM is origin 0x20000000 length 20K that is:
 * 0x20000000 to 0x20005000
 *
 * F3 Boards
 * RAM is origin 0x20000000 length 40K that is:
 * 0x20000000 to 0x2000a000
 *
 * F4 Boards
 * RAM is origin 0x20000000 length 128K that is:
 * 0x20000000 to 0x20020000
 *
 */
#[no_mangle]
pub unsafe extern "C" fn stackTotalSize() -> uint32_t {
    return &mut _Min_Stack_Size as *mut libc::c_char as intptr_t as uint32_t;
}
#[no_mangle]
pub unsafe extern "C" fn stackHighMem() -> uint32_t {
    return &mut _estack as *mut libc::c_char as intptr_t as uint32_t;
}
