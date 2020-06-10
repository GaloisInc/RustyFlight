use ::libc;
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
// FIXME remove this for targets that don't need a CLI.  Perhaps use a no-op macro when USE_CLI is not enabled
// signal that we're in cli mode
#[no_mangle]
pub static mut cliMode: uint8_t = 0 as libc::c_int as uint8_t;
// USE_CLI
