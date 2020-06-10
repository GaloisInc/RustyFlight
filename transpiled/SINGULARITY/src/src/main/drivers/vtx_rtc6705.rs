use ::libc;
extern "C" {
    #[no_mangle]
    fn spiSetDivisor(instance: *mut SPI_TypeDef, divisor: uint16_t);
    #[no_mangle]
    fn spiTransferByte(instance: *mut SPI_TypeDef, data: uint8_t) -> uint8_t;
    // TODO
    // declare available IO pins. Available pins are specified per target
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn IOLo(io: IO_t);
    #[no_mangle]
    fn IOHi(io: IO_t);
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
    fn delayMicroseconds(us: timeUs_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type __uint64_t = libc::c_ulong;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
/* *
  * @brief Serial Peripheral Interface
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SPI_TypeDef {
    pub CR1: uint16_t,
    pub RESERVED0: uint16_t,
    pub CR2: uint16_t,
    pub RESERVED1: uint16_t,
    pub SR: uint16_t,
    pub RESERVED2: uint16_t,
    pub DR: uint16_t,
    pub RESERVED3: uint16_t,
    pub CRCPR: uint16_t,
    pub RESERVED4: uint16_t,
    pub RXCRCR: uint16_t,
    pub RESERVED5: uint16_t,
    pub TXCRCR: uint16_t,
    pub RESERVED6: uint16_t,
    pub I2SCFGR: uint16_t,
    pub RESERVED7: uint16_t,
    pub I2SPR: uint16_t,
    pub RESERVED8: uint16_t,
}
/* *
  ******************************************************************************
  * @file    stm32f30x_gpio.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the GPIO 
  *          firmware library. 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F30x_StdPeriph_Driver
  * @{
  */
/* * @addtogroup GPIO
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup Configuration_Mode_enumeration 
  * @{
  */
pub type C2RustUnnamed = libc::c_uint;
/* !< GPIO Analog In/Out Mode      */
/* !< GPIO Alternate function Mode */
pub const GPIO_Mode_AN: C2RustUnnamed = 3;
/* !< GPIO Output Mode */
pub const GPIO_Mode_AF: C2RustUnnamed = 2;
/* !< GPIO Input Mode */
pub const GPIO_Mode_OUT: C2RustUnnamed = 1;
pub const GPIO_Mode_IN: C2RustUnnamed = 0;
/* *
  * @}
  */
/* * @defgroup Output_type_enumeration
  * @{
  */
pub type C2RustUnnamed_0 = libc::c_uint;
pub const GPIO_OType_OD: C2RustUnnamed_0 = 1;
pub const GPIO_OType_PP: C2RustUnnamed_0 = 0;
/* *
  * @}
  */
/* * @defgroup Configuration_Pull-Up_Pull-Down_enumeration 
  * @{
  */
pub type C2RustUnnamed_1 = libc::c_uint;
pub const GPIO_PuPd_DOWN: C2RustUnnamed_1 = 2;
pub const GPIO_PuPd_UP: C2RustUnnamed_1 = 1;
pub const GPIO_PuPd_NOPULL: C2RustUnnamed_1 = 0;
pub type ioTag_t = uint8_t;
pub type IO_t = *mut libc::c_void;
// both ioTag_t and IO_t are guarantied to be zero if pinid is NONE (no pin)
// this simplifies initialization (globals are zeroed on start) and allows
//  omitting unused fields in structure initializers.
// it is also possible to use IO_t and ioTag_t as boolean value
//   TODO - this may conflict with requirement to generate warning/error on IO_t - ioTag_t assignment
//   IO_t being pointer is only possibility I know of ..
// pin config handling
// pin config is packed into ioConfig_t to decrease memory requirements
// IOCFG_x macros are defined for common combinations for all CPUs; this
//  helps masking CPU differences
pub type ioConfig_t = uint8_t;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const SPI_CLOCK_ULTRAFAST: C2RustUnnamed_2 = 2;
pub const SPI_CLOCK_FAST: C2RustUnnamed_2 = 2;
pub const SPI_CLOCK_STANDARD: C2RustUnnamed_2 = 4;
pub const SPI_CLOCK_SLOW: C2RustUnnamed_2 = 128;
pub const SPI_CLOCK_INITIALIZATON: C2RustUnnamed_2 = 256;
pub type resourceOwner_e = libc::c_uint;
pub const OWNER_TOTAL_COUNT: resourceOwner_e = 55;
pub const OWNER_SPI_PREINIT_OPU: resourceOwner_e = 54;
pub const OWNER_SPI_PREINIT_IPU: resourceOwner_e = 53;
pub const OWNER_USB_MSC_PIN: resourceOwner_e = 52;
pub const OWNER_PINIO: resourceOwner_e = 51;
pub const OWNER_RX_SPI: resourceOwner_e = 50;
pub const OWNER_RANGEFINDER: resourceOwner_e = 49;
pub const OWNER_TIMUP: resourceOwner_e = 48;
pub const OWNER_CAMERA_CONTROL: resourceOwner_e = 47;
pub const OWNER_ESCSERIAL: resourceOwner_e = 46;
pub const OWNER_RX_BIND_PLUG: resourceOwner_e = 45;
pub const OWNER_COMPASS_CS: resourceOwner_e = 44;
pub const OWNER_VTX: resourceOwner_e = 43;
pub const OWNER_TRANSPONDER: resourceOwner_e = 42;
pub const OWNER_LED_STRIP: resourceOwner_e = 41;
pub const OWNER_INVERTER: resourceOwner_e = 40;
pub const OWNER_RX_BIND: resourceOwner_e = 39;
pub const OWNER_OSD: resourceOwner_e = 38;
pub const OWNER_BEEPER: resourceOwner_e = 37;
pub const OWNER_USB_DETECT: resourceOwner_e = 36;
pub const OWNER_USB: resourceOwner_e = 35;
pub const OWNER_COMPASS_EXTI: resourceOwner_e = 34;
pub const OWNER_BARO_EXTI: resourceOwner_e = 33;
pub const OWNER_MPU_EXTI: resourceOwner_e = 32;
pub const OWNER_SPI_CS: resourceOwner_e = 31;
pub const OWNER_RX_SPI_CS: resourceOwner_e = 30;
pub const OWNER_OSD_CS: resourceOwner_e = 29;
pub const OWNER_MPU_CS: resourceOwner_e = 28;
pub const OWNER_BARO_CS: resourceOwner_e = 27;
pub const OWNER_FLASH_CS: resourceOwner_e = 26;
pub const OWNER_SDCARD_DETECT: resourceOwner_e = 25;
pub const OWNER_SDCARD_CS: resourceOwner_e = 24;
pub const OWNER_SDCARD: resourceOwner_e = 23;
pub const OWNER_I2C_SDA: resourceOwner_e = 22;
pub const OWNER_I2C_SCL: resourceOwner_e = 21;
pub const OWNER_SPI_MOSI: resourceOwner_e = 20;
pub const OWNER_SPI_MISO: resourceOwner_e = 19;
pub const OWNER_SPI_SCK: resourceOwner_e = 18;
pub const OWNER_SYSTEM: resourceOwner_e = 17;
pub const OWNER_SONAR_ECHO: resourceOwner_e = 16;
pub const OWNER_SONAR_TRIGGER: resourceOwner_e = 15;
pub const OWNER_TIMER: resourceOwner_e = 14;
pub const OWNER_PINDEBUG: resourceOwner_e = 13;
pub const OWNER_SERIAL_RX: resourceOwner_e = 12;
pub const OWNER_SERIAL_TX: resourceOwner_e = 11;
pub const OWNER_ADC_RSSI: resourceOwner_e = 10;
pub const OWNER_ADC_EXT: resourceOwner_e = 9;
pub const OWNER_ADC_CURR: resourceOwner_e = 8;
pub const OWNER_ADC_BATT: resourceOwner_e = 7;
pub const OWNER_ADC: resourceOwner_e = 6;
pub const OWNER_LED: resourceOwner_e = 5;
pub const OWNER_SERVO: resourceOwner_e = 4;
pub const OWNER_MOTOR: resourceOwner_e = 3;
pub const OWNER_PPMINPUT: resourceOwner_e = 2;
pub const OWNER_PWMINPUT: resourceOwner_e = 1;
pub const OWNER_FREE: resourceOwner_e = 0;
pub type timeUs_t = uint32_t;
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
//Division value (to fit into a uint32_t) (Hz to MHz)
static mut vtxCSPin: IO_t = 0 as *const libc::c_void as IO_t;
/* *
 * Reverse a uint32_t (LSB to MSB)
 * This is easier for when generating the frequency to then
 * reverse the bits afterwards
 */
unsafe extern "C" fn reverse32(mut in_0: uint32_t) -> uint32_t {
    let mut out: uint32_t = 0 as libc::c_int as uint32_t;
    let mut i: uint8_t = 0 as libc::c_int as uint8_t;
    while (i as libc::c_int) < 32 as libc::c_int {
        out |=
            (in_0 >> i as libc::c_int & 1 as libc::c_int as libc::c_uint) <<
                31 as libc::c_int - i as libc::c_int;
        i = i.wrapping_add(1)
    }
    return out;
}
// milliseconds
/* *
 * Start chip if available
 */
#[no_mangle]
pub unsafe extern "C" fn rtc6705IOInit() {
    vtxCSPin =
        IOGetByTag(((0 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int
                        | 4 as libc::c_int) as ioTag_t);
    IOInit(vtxCSPin, OWNER_VTX, 0 as libc::c_int as uint8_t);
    IOHi(vtxCSPin);
    // GPIO bit is enabled so here so the output is not pulled low when the GPIO is set in output mode.
    // Note: It's critical to ensure that incorrect signals are not sent to the VTX.
    IOConfigGPIO(vtxCSPin,
                 (GPIO_Mode_OUT as libc::c_int |
                      (0 as libc::c_int) << 2 as libc::c_int |
                      (GPIO_OType_PP as libc::c_int) << 4 as libc::c_int |
                      (GPIO_PuPd_NOPULL as libc::c_int) << 5 as libc::c_int)
                     as ioConfig_t);
}
/* *
 * Transfer a 25bit packet to RTC6705
 * This will just send it as a 32bit packet LSB meaning
 * extra 0's get truncated on RTC6705 end
 */
unsafe extern "C" fn rtc6705Transfer(mut command: uint32_t) {
    command = reverse32(command);
    IOLo(vtxCSPin);
    spiTransferByte((0x40000000 as libc::c_int as
                         uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                    libc::c_uint).wrapping_add(0x3000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   libc::c_uint)
                        as *mut SPI_TypeDef,
                    (command >> 24 as libc::c_int &
                         0xff as libc::c_int as libc::c_uint) as uint8_t);
    spiTransferByte((0x40000000 as libc::c_int as
                         uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                    libc::c_uint).wrapping_add(0x3000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   libc::c_uint)
                        as *mut SPI_TypeDef,
                    (command >> 16 as libc::c_int &
                         0xff as libc::c_int as libc::c_uint) as uint8_t);
    spiTransferByte((0x40000000 as libc::c_int as
                         uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                    libc::c_uint).wrapping_add(0x3000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   libc::c_uint)
                        as *mut SPI_TypeDef,
                    (command >> 8 as libc::c_int &
                         0xff as libc::c_int as libc::c_uint) as uint8_t);
    spiTransferByte((0x40000000 as libc::c_int as
                         uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                    libc::c_uint).wrapping_add(0x3000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   libc::c_uint)
                        as *mut SPI_TypeDef,
                    (command >> 0 as libc::c_int &
                         0xff as libc::c_int as libc::c_uint) as uint8_t);
    delayMicroseconds(2 as libc::c_int as timeUs_t);
    IOHi(vtxCSPin);
    delayMicroseconds(2 as libc::c_int as timeUs_t);
}
/* *
 * Set a frequency in Mhz
 * Formula derived from datasheet
 */
#[no_mangle]
pub unsafe extern "C" fn rtc6705SetFrequency(mut frequency: uint16_t) {
    frequency =
        constrain(frequency as libc::c_int, 5600 as libc::c_int,
                  5950 as libc::c_int) as
            uint16_t; //Casts required to make sure correct math (large numbers)
    let val_a: uint32_t =
        (frequency as
             uint64_t).wrapping_mul(1000000 as libc::c_int as
                                        uint64_t).wrapping_mul(400 as
                                                                   libc::c_int
                                                                   as
                                                                   uint64_t).wrapping_div(1000000
                                                                                              as
                                                                                              libc::c_int
                                                                                              as
                                                                                              uint64_t).wrapping_rem(1024
                                                                                                                         as
                                                                                                                         libc::c_int
                                                                                                                         as
                                                                                                                         libc::c_ulong).wrapping_div(16
                                                                                                                                                         as
                                                                                                                                                         libc::c_int
                                                                                                                                                         as
                                                                                                                                                         libc::c_ulong)
            as
            uint32_t; //Casts required to make sure correct math (large numbers)
    let val_n: uint32_t =
        (frequency as
             uint64_t).wrapping_mul(1000000 as libc::c_int as
                                        uint64_t).wrapping_mul(400 as
                                                                   libc::c_int
                                                                   as
                                                                   uint64_t).wrapping_div(1000000
                                                                                              as
                                                                                              libc::c_int
                                                                                              as
                                                                                              uint64_t).wrapping_div(1024
                                                                                                                         as
                                                                                                                         libc::c_int
                                                                                                                         as
                                                                                                                         libc::c_ulong)
            as uint32_t; // write
    let mut val_hex: uint32_t = 0x11 as libc::c_int as uint32_t; // address
    val_hex |= val_a << 5 as libc::c_int; // 4 address bits and 1 rw bit.
    val_hex |= val_n << 12 as libc::c_int;
    spiSetDivisor((0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x3000
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut SPI_TypeDef,
                  SPI_CLOCK_SLOW as libc::c_int as uint16_t);
    rtc6705Transfer(0x3210 as libc::c_int as uint32_t);
    delayMicroseconds(10 as libc::c_int as timeUs_t);
    rtc6705Transfer(val_hex);
}
#[no_mangle]
pub unsafe extern "C" fn rtc6705SetRFPower(mut rf_power: uint8_t) {
    rf_power =
        constrain(rf_power as libc::c_int, 1 as libc::c_int,
                  3 as libc::c_int - 1 as libc::c_int) as uint8_t;
    spiSetDivisor((0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x3000
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut SPI_TypeDef,
                  SPI_CLOCK_SLOW as libc::c_int as uint16_t);
    let mut val_hex: uint32_t =
        ((1 as libc::c_int) << 4 as libc::c_int) as uint32_t;
    val_hex |= 0x7 as libc::c_int as libc::c_uint;
    let data: uint32_t =
        if rf_power as libc::c_int > 1 as libc::c_int {
            0x4fbd as libc::c_int
        } else {
            (0x4fbd as libc::c_int | 0x40 as libc::c_int) &
                !(0x180 as libc::c_int | 0xe00 as libc::c_int)
        } as uint32_t;
    val_hex |= data << 5 as libc::c_int;
    rtc6705Transfer(val_hex);
}
#[no_mangle]
pub unsafe extern "C" fn rtc6705Disable() { }
#[no_mangle]
pub unsafe extern "C" fn rtc6705Enable() { }
