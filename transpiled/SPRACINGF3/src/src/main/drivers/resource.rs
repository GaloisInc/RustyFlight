use core;
use libc;
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
pub static mut ownerNames: [*const libc::c_char; 55] =
    [b"FREE\x00" as *const u8 as *const libc::c_char,
     b"PWM\x00" as *const u8 as *const libc::c_char,
     b"PPM\x00" as *const u8 as *const libc::c_char,
     b"MOTOR\x00" as *const u8 as *const libc::c_char,
     b"SERVO\x00" as *const u8 as *const libc::c_char,
     b"LED\x00" as *const u8 as *const libc::c_char,
     b"ADC\x00" as *const u8 as *const libc::c_char,
     b"ADC_BATT\x00" as *const u8 as *const libc::c_char,
     b"ADC_CURR\x00" as *const u8 as *const libc::c_char,
     b"ADC_EXT\x00" as *const u8 as *const libc::c_char,
     b"ADC_RSSI\x00" as *const u8 as *const libc::c_char,
     b"SERIAL_TX\x00" as *const u8 as *const libc::c_char,
     b"SERIAL_RX\x00" as *const u8 as *const libc::c_char,
     b"DEBUG\x00" as *const u8 as *const libc::c_char,
     b"TIMER\x00" as *const u8 as *const libc::c_char,
     b"SONAR_TRIGGER\x00" as *const u8 as *const libc::c_char,
     b"SONAR_ECHO\x00" as *const u8 as *const libc::c_char,
     b"SYSTEM\x00" as *const u8 as *const libc::c_char,
     b"SPI_SCK\x00" as *const u8 as *const libc::c_char,
     b"SPI_MISO\x00" as *const u8 as *const libc::c_char,
     b"SPI_MOSI\x00" as *const u8 as *const libc::c_char,
     b"I2C_SCL\x00" as *const u8 as *const libc::c_char,
     b"I2C_SDA\x00" as *const u8 as *const libc::c_char,
     b"SDCARD\x00" as *const u8 as *const libc::c_char,
     b"SDCARD_CS\x00" as *const u8 as *const libc::c_char,
     b"SDCARD_DETECT\x00" as *const u8 as *const libc::c_char,
     b"FLASH_CS\x00" as *const u8 as *const libc::c_char,
     b"BARO_CS\x00" as *const u8 as *const libc::c_char,
     b"MPU_CS\x00" as *const u8 as *const libc::c_char,
     b"OSD_CS\x00" as *const u8 as *const libc::c_char,
     b"RX_SPI_CS\x00" as *const u8 as *const libc::c_char,
     b"SPI_CS\x00" as *const u8 as *const libc::c_char,
     b"MPU_EXTI\x00" as *const u8 as *const libc::c_char,
     b"BARO_EXTI\x00" as *const u8 as *const libc::c_char,
     b"COMPASS_EXTI\x00" as *const u8 as *const libc::c_char,
     b"USB\x00" as *const u8 as *const libc::c_char,
     b"USB_DETECT\x00" as *const u8 as *const libc::c_char,
     b"BEEPER\x00" as *const u8 as *const libc::c_char,
     b"OSD\x00" as *const u8 as *const libc::c_char,
     b"RX_BIND\x00" as *const u8 as *const libc::c_char,
     b"INVERTER\x00" as *const u8 as *const libc::c_char,
     b"LED_STRIP\x00" as *const u8 as *const libc::c_char,
     b"TRANSPONDER\x00" as *const u8 as *const libc::c_char,
     b"VTX\x00" as *const u8 as *const libc::c_char,
     b"COMPASS_CS\x00" as *const u8 as *const libc::c_char,
     b"RX_BIND_PLUG\x00" as *const u8 as *const libc::c_char,
     b"ESCSERIAL\x00" as *const u8 as *const libc::c_char,
     b"CAMERA_CONTROL\x00" as *const u8 as *const libc::c_char,
     b"TIMUP\x00" as *const u8 as *const libc::c_char,
     b"RANGEFINDER\x00" as *const u8 as *const libc::c_char,
     b"RX_SPI\x00" as *const u8 as *const libc::c_char,
     b"PINIO\x00" as *const u8 as *const libc::c_char,
     b"USB_MSC_PIN\x00" as *const u8 as *const libc::c_char,
     b"SPI_PREINIT_IPU\x00" as *const u8 as *const libc::c_char,
     b"SPI_PREINIT_OPU\x00" as *const u8 as *const libc::c_char];
