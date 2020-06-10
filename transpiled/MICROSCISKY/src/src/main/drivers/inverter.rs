use ::libc;
extern "C" {
    #[no_mangle]
    fn IOWrite(io: IO_t, value: bool);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* * 
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USART_TypeDef {
    pub SR: uint16_t,
    pub RESERVED0: uint16_t,
    pub DR: uint16_t,
    pub RESERVED1: uint16_t,
    pub BRR: uint16_t,
    pub RESERVED2: uint16_t,
    pub CR1: uint16_t,
    pub RESERVED3: uint16_t,
    pub CR2: uint16_t,
    pub RESERVED4: uint16_t,
    pub CR3: uint16_t,
    pub RESERVED5: uint16_t,
    pub GTPR: uint16_t,
    pub RESERVED6: uint16_t,
}
pub type C2RustUnnamed = libc::c_uint;
pub const GPIO_Speed_50MHz: C2RustUnnamed = 3;
pub const GPIO_Speed_2MHz: C2RustUnnamed = 2;
pub const GPIO_Speed_10MHz: C2RustUnnamed = 1;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const GPIO_Mode_AF_PP: C2RustUnnamed_0 = 24;
pub const GPIO_Mode_AF_OD: C2RustUnnamed_0 = 28;
pub const GPIO_Mode_Out_PP: C2RustUnnamed_0 = 16;
pub const GPIO_Mode_Out_OD: C2RustUnnamed_0 = 20;
pub const GPIO_Mode_IPU: C2RustUnnamed_0 = 72;
pub const GPIO_Mode_IPD: C2RustUnnamed_0 = 40;
pub const GPIO_Mode_IN_FLOATING: C2RustUnnamed_0 = 4;
pub const GPIO_Mode_AIN: C2RustUnnamed_0 = 0;
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
// packet tag to specify IO pin
pub type IO_t = *mut libc::c_void;
// type specifying IO pin. Currently ioRec_t pointer, but this may change
// NONE initializer for ioTag_t variables
// NONE initializer for IO_t variable
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPinConfig_s {
    pub ioTagTx: [ioTag_t; 10],
    pub ioTagRx: [ioTag_t; 10],
    pub ioTagInverter: [ioTag_t; 10],
}
pub type serialPinConfig_t = serialPinConfig_s;
pub type C2RustUnnamed_1 = libc::c_int;
pub const SERIAL_PORT_SOFTSERIAL2: C2RustUnnamed_1 = 31;
pub const SERIAL_PORT_SOFTSERIAL1: C2RustUnnamed_1 = 30;
pub const SERIAL_PORT_USB_VCP: C2RustUnnamed_1 = 20;
pub const SERIAL_PORT_USART8: C2RustUnnamed_1 = 7;
pub const SERIAL_PORT_USART7: C2RustUnnamed_1 = 6;
pub const SERIAL_PORT_USART6: C2RustUnnamed_1 = 5;
pub const SERIAL_PORT_UART5: C2RustUnnamed_1 = 4;
pub const SERIAL_PORT_UART4: C2RustUnnamed_1 = 3;
pub const SERIAL_PORT_USART3: C2RustUnnamed_1 = 2;
pub const SERIAL_PORT_USART2: C2RustUnnamed_1 = 1;
pub const SERIAL_PORT_USART1: C2RustUnnamed_1 = 0;
pub const SERIAL_PORT_NONE: C2RustUnnamed_1 = -1;
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
static mut pSerialPinConfig: *const serialPinConfig_t =
    0 as *const serialPinConfig_t;
unsafe extern "C" fn inverterSet(mut identifier: libc::c_int, mut on: bool) {
    let mut pin: IO_t =
        IOGetByTag((*pSerialPinConfig).ioTagInverter[if identifier <=
                                                            SERIAL_PORT_USART8
                                                                as libc::c_int
                                                        {
                                                         identifier
                                                     } else {
                                                         (10 as libc::c_int) +
                                                             (identifier -
                                                                  SERIAL_PORT_SOFTSERIAL1
                                                                      as
                                                                      libc::c_int)
                                                     } as usize]);
    if !pin.is_null() { IOWrite(pin, on); };
}
unsafe extern "C" fn initInverter(mut identifier: libc::c_int) {
    let mut uartIndex: libc::c_int =
        if identifier <= SERIAL_PORT_USART8 as libc::c_int {
            identifier
        } else {
            (10 as libc::c_int) +
                (identifier - SERIAL_PORT_SOFTSERIAL1 as libc::c_int)
        };
    let mut pin: IO_t =
        IOGetByTag((*pSerialPinConfig).ioTagInverter[uartIndex as usize]);
    if !pin.is_null() {
        IOInit(pin, OWNER_INVERTER,
               (uartIndex + 1 as libc::c_int) as uint8_t);
        IOConfigGPIO(pin,
                     (GPIO_Mode_Out_PP as libc::c_int |
                          GPIO_Speed_2MHz as libc::c_int) as ioConfig_t);
        inverterSet(identifier, 0 as libc::c_int != 0);
    };
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
pub unsafe extern "C" fn initInverters(mut serialPinConfigToUse:
                                           *const serialPinConfig_t) {
    pSerialPinConfig = serialPinConfigToUse;
    initInverter(SERIAL_PORT_USART1 as libc::c_int);
    initInverter(SERIAL_PORT_USART2 as libc::c_int);
}
#[no_mangle]
pub unsafe extern "C" fn enableInverter(mut USARTx: *mut USART_TypeDef,
                                        mut on: bool) {
    let mut identifier: libc::c_int = SERIAL_PORT_NONE as libc::c_int;
    if USARTx ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x10000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x3800
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut USART_TypeDef {
        identifier = SERIAL_PORT_USART1 as libc::c_int
    }
    if USARTx ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x4400 as libc::c_int as libc::c_uint)
               as *mut USART_TypeDef {
        identifier = SERIAL_PORT_USART2 as libc::c_int
    }
    if identifier != SERIAL_PORT_NONE as libc::c_int {
        inverterSet(identifier, on);
    };
}
// USE_INVERTER
