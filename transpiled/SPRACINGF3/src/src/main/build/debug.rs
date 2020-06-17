use core;
use libc;
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint32_t = libc::c_uint;
pub type int16_t = __int16_t;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
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
pub static mut debug: [int16_t; 4] = [0; 4];
#[no_mangle]
pub static mut debugMode: uint8_t = 0;
#[no_mangle]
pub static mut sectionTimes: [[uint32_t; 4]; 2] = [[0; 4]; 2];
#[no_mangle]
pub static mut debugModeNames: [*const libc::c_char; 44] =
    [b"NONE\x00" as *const u8 as *const libc::c_char,
     b"CYCLETIME\x00" as *const u8 as *const libc::c_char,
     b"BATTERY\x00" as *const u8 as *const libc::c_char,
     b"GYRO_FILTERED\x00" as *const u8 as *const libc::c_char,
     b"ACCELEROMETER\x00" as *const u8 as *const libc::c_char,
     b"PIDLOOP\x00" as *const u8 as *const libc::c_char,
     b"GYRO_SCALED\x00" as *const u8 as *const libc::c_char,
     b"RC_INTERPOLATION\x00" as *const u8 as *const libc::c_char,
     b"ANGLERATE\x00" as *const u8 as *const libc::c_char,
     b"ESC_SENSOR\x00" as *const u8 as *const libc::c_char,
     b"SCHEDULER\x00" as *const u8 as *const libc::c_char,
     b"STACK\x00" as *const u8 as *const libc::c_char,
     b"ESC_SENSOR_RPM\x00" as *const u8 as *const libc::c_char,
     b"ESC_SENSOR_TMP\x00" as *const u8 as *const libc::c_char,
     b"ALTITUDE\x00" as *const u8 as *const libc::c_char,
     b"FFT\x00" as *const u8 as *const libc::c_char,
     b"FFT_TIME\x00" as *const u8 as *const libc::c_char,
     b"FFT_FREQ\x00" as *const u8 as *const libc::c_char,
     b"RX_FRSKY_SPI\x00" as *const u8 as *const libc::c_char,
     b"GYRO_RAW\x00" as *const u8 as *const libc::c_char,
     b"DUAL_GYRO\x00" as *const u8 as *const libc::c_char,
     b"DUAL_GYRO_RAW\x00" as *const u8 as *const libc::c_char,
     b"DUAL_GYRO_COMBINE\x00" as *const u8 as *const libc::c_char,
     b"DUAL_GYRO_DIFF\x00" as *const u8 as *const libc::c_char,
     b"MAX7456_SIGNAL\x00" as *const u8 as *const libc::c_char,
     b"MAX7456_SPICLOCK\x00" as *const u8 as *const libc::c_char,
     b"SBUS\x00" as *const u8 as *const libc::c_char,
     b"FPORT\x00" as *const u8 as *const libc::c_char,
     b"RANGEFINDER\x00" as *const u8 as *const libc::c_char,
     b"RANGEFINDER_QUALITY\x00" as *const u8 as *const libc::c_char,
     b"LIDAR_TF\x00" as *const u8 as *const libc::c_char,
     b"CORE_TEMP\x00" as *const u8 as *const libc::c_char,
     b"RUNAWAY_TAKEOFF\x00" as *const u8 as *const libc::c_char,
     b"SDIO\x00" as *const u8 as *const libc::c_char,
     b"CURRENT_SENSOR\x00" as *const u8 as *const libc::c_char,
     b"USB\x00" as *const u8 as *const libc::c_char,
     b"SMARTAUDIO\x00" as *const u8 as *const libc::c_char,
     b"RTH\x00" as *const u8 as *const libc::c_char,
     b"ITERM_RELAX\x00" as *const u8 as *const libc::c_char,
     b"ACRO_TRAINER\x00" as *const u8 as *const libc::c_char,
     b"RC_SMOOTHING\x00" as *const u8 as *const libc::c_char,
     b"RX_SIGNAL_LOSS\x00" as *const u8 as *const libc::c_char,
     b"RC_SMOOTHING_RATE\x00" as *const u8 as *const libc::c_char,
     b"ANTI_GRAVITY\x00" as *const u8 as *const libc::c_char];
