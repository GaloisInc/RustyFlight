use ::libc;
extern "C" {
    #[no_mangle]
    fn spiDeviceByInstance(instance: *mut SPI_TypeDef) -> SPIDevice;
    #[no_mangle]
    fn EXTIHandlerInit(cb: *mut extiCallbackRec_t,
                       fn_0: Option<extiHandlerCallback>);
    #[no_mangle]
    fn EXTIConfig(io: IO_t, cb: *mut extiCallbackRec_t,
                  irqPriority: libc::c_int, trigger: EXTITrigger_TypeDef);
    #[no_mangle]
    fn EXTIEnable(io: IO_t, enable: bool);
    // declare available IO pins. Available pins are specified per target
    #[no_mangle]
    fn IORead(io: IO_t) -> bool;
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn rxSpiWriteByte(data: uint8_t) -> uint8_t;
    #[no_mangle]
    fn rxSpiWriteCommand(command: uint8_t, data: uint8_t) -> uint8_t;
    #[no_mangle]
    fn rxSpiWriteCommandMulti(command: uint8_t, data: *const uint8_t,
                              length: uint8_t) -> uint8_t;
    #[no_mangle]
    fn rxSpiReadCommand(command: uint8_t, commandData: uint8_t) -> uint8_t;
    #[no_mangle]
    fn rxSpiReadCommandMulti(command: uint8_t, commandData: uint8_t,
                             retData: *mut uint8_t, length: uint8_t)
     -> uint8_t;
    #[no_mangle]
    fn micros() -> timeUs_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
  * @brief  EXTI Trigger enumeration  
  */
pub type EXTITrigger_TypeDef = libc::c_uint;
pub const EXTI_Trigger_Rising_Falling: EXTITrigger_TypeDef = 16;
pub const EXTI_Trigger_Falling: EXTITrigger_TypeDef = 12;
pub const EXTI_Trigger_Rising: EXTITrigger_TypeDef = 8;
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
/* * @defgroup Configuration_Pull-Up_Pull-Down_enumeration 
  * @{
  */
pub type C2RustUnnamed_0 = libc::c_uint;
pub const GPIO_PuPd_DOWN: C2RustUnnamed_0 = 2;
pub const GPIO_PuPd_UP: C2RustUnnamed_0 = 1;
pub const GPIO_PuPd_NOPULL: C2RustUnnamed_0 = 0;
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
pub type SPIDevice = libc::c_int;
pub const SPIDEV_4: SPIDevice = 3;
pub const SPIDEV_3: SPIDevice = 2;
pub const SPIDEV_2: SPIDevice = 1;
pub const SPIDEV_1: SPIDevice = 0;
pub const SPIINVALID: SPIDevice = -1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct extiCallbackRec_s {
    pub fn_0: Option<extiHandlerCallback>,
}
pub type extiHandlerCallback
    =
    unsafe extern "C" fn(_: *mut extiCallbackRec_t) -> ();
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
pub type extiCallbackRec_t = extiCallbackRec_s;
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
pub type A7105State_t = libc::c_uint;
pub const A7105_RST_RDPTR: A7105State_t = 240;
pub const A7105_RST_WRPTR: A7105State_t = 224;
pub const A7105_TX: A7105State_t = 208;
pub const A7105_RX: A7105State_t = 192;
pub const A7105_PLL: A7105State_t = 176;
pub const A7105_STANDBY: A7105State_t = 160;
pub const A7105_IDLE: A7105State_t = 144;
pub const A7105_SLEEP: A7105State_t = 128;
pub type A7105Reg_t = libc::c_uint;
pub const A7105_32_FILTER_TEST: A7105Reg_t = 50;
pub const A7105_31_RSCALE: A7105Reg_t = 49;
pub const A7105_30_IFAT: A7105Reg_t = 48;
pub const A7105_2F_VCO_TEST_II: A7105Reg_t = 47;
pub const A7105_2E_VCO_TEST_I: A7105Reg_t = 46;
pub const A7105_2D_PLL_TEST: A7105Reg_t = 45;
pub const A7105_2C_XTAL_TEST: A7105Reg_t = 44;
pub const A7105_2B_CPC: A7105Reg_t = 43;
pub const A7105_2A_RX_DEM_TEST_II: A7105Reg_t = 42;
pub const A7105_29_RX_DEM_TEST_I: A7105Reg_t = 41;
pub const A7105_28_TX_TEST: A7105Reg_t = 40;
pub const A7105_27_BATTERY_DET: A7105Reg_t = 39;
pub const A7105_26_VCO_SBCAL_II: A7105Reg_t = 38;
pub const A7105_25_VCO_SBCAL_I: A7105Reg_t = 37;
pub const A7105_24_VCO_CURCAL: A7105Reg_t = 36;
pub const A7105_23_IF_CALIB_II: A7105Reg_t = 35;
pub const A7105_22_IF_CALIB_I: A7105Reg_t = 34;
pub const A7105_21_CODE_III: A7105Reg_t = 33;
pub const A7105_20_CODE_II: A7105Reg_t = 32;
pub const A7105_1F_CODE_I: A7105Reg_t = 31;
pub const A7105_1E_ADC: A7105Reg_t = 30;
pub const A7105_1D_RSSI_THOLD: A7105Reg_t = 29;
pub const A7105_1C_RX_GAIN_IV: A7105Reg_t = 28;
pub const A7105_1B_RX_GAIN_III: A7105Reg_t = 27;
pub const A7105_1A_RX_GAIN_II: A7105Reg_t = 26;
pub const A7105_19_RX_GAIN_I: A7105Reg_t = 25;
pub const A7105_18_RX: A7105Reg_t = 24;
pub const A7105_17_DELAY_II: A7105Reg_t = 23;
pub const A7105_16_DELAY_I: A7105Reg_t = 22;
pub const A7105_15_TX_II: A7105Reg_t = 21;
pub const A7105_14_TX_I: A7105Reg_t = 20;
pub const A7105_13_PLL_V: A7105Reg_t = 19;
pub const A7105_12_PLL_IV: A7105Reg_t = 18;
pub const A7105_11_PLL_III: A7105Reg_t = 17;
pub const A7105_10_PLL_II: A7105Reg_t = 16;
pub const A7105_0F_CHANNEL: A7105Reg_t = 15;
pub const A7105_0F_PLL_I: A7105Reg_t = 15;
pub const A7105_0E_DATA_RATE: A7105Reg_t = 14;
pub const A7105_0D_CLOCK: A7105Reg_t = 13;
pub const A7105_0C_GPIO2_PIN_II: A7105Reg_t = 12;
pub const A7105_0B_GPIO1_PIN_I: A7105Reg_t = 11;
pub const A7105_0A_CK0_PIN: A7105Reg_t = 10;
pub const A7105_09_RC_OSC_III: A7105Reg_t = 9;
pub const A7105_08_RC_OSC_II: A7105Reg_t = 8;
pub const A7105_07_RC_OSC_I: A7105Reg_t = 7;
pub const A7105_06_ID_DATA: A7105Reg_t = 6;
pub const A7105_05_FIFO_DATA: A7105Reg_t = 5;
pub const A7105_04_FIFOII: A7105Reg_t = 4;
pub const A7105_03_FIFOI: A7105Reg_t = 3;
pub const A7105_02_CALC: A7105Reg_t = 2;
pub const A7105_01_MODE_CONTROL: A7105Reg_t = 1;
pub const A7105_00_MODE: A7105Reg_t = 0;
// microsecond time
pub type timeUs_t = uint32_t;
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
static mut rxIntIO: IO_t = 0 as *const libc::c_void as IO_t;
static mut a7105extiCallbackRec: extiCallbackRec_t =
    extiCallbackRec_t{fn_0: None,};
static mut timeEvent: uint32_t = 0 as libc::c_int as uint32_t;
static mut occurEvent: bool = 0 as libc::c_int != 0;
#[no_mangle]
pub unsafe extern "C" fn a7105extiHandler(mut cb: *mut extiCallbackRec_t) {
    if IORead(rxIntIO) as libc::c_int != 0 as libc::c_int {
        ::core::ptr::write_volatile(&mut timeEvent as *mut uint32_t,
                                    micros());
        ::core::ptr::write_volatile(&mut occurEvent as *mut bool,
                                    1 as libc::c_int != 0)
    };
}
/* Register: A7105_00_MODE */
// [0]: FEC pass. [1]: FEC error. (FECF is read only, it is updated internally while receiving every packet.)
// [0]: CRC pass. [1]: CRC error. (CRCF is read only, it is updated internally while receiving every packet.)
// [0]: RF chip is disabled. [1]: RF chip is enabled.
// [0]: Crystal oscillator is disabled. [1]: Crystal oscillator is enabled.
// [0]: PLL is disabled. [1]: PLL is enabled.
// [0]: RX state. [1]: TX state. Serviceable if TRER=1 (TRX is enable).
// [0]: TRX is disabled. [1]: TRX is enabled.
#[no_mangle]
pub unsafe extern "C" fn A7105Init(mut id: uint32_t) {
    spiDeviceByInstance((0x40000000 as libc::c_int as
                             uint32_t).wrapping_add(0x3800 as libc::c_int as
                                                        libc::c_uint) as
                            *mut SPI_TypeDef); /* config receiver IRQ pin */
    rxIntIO =
        IOGetByTag(((1 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int
                        | 12 as libc::c_int) as
                       ioTag_t); /* reset read pointer */
    IOInit(rxIntIO, OWNER_RX_SPI_CS,
           0 as libc::c_int as uint8_t); /* reset write pointer */
    IOConfigGPIO(rxIntIO,
                 (GPIO_Mode_IN as libc::c_int |
                      (0 as libc::c_int) << 2 as libc::c_int |
                      (0 as libc::c_int) << 4 as libc::c_int |
                      (GPIO_PuPd_DOWN as libc::c_int) << 5 as libc::c_int) as
                     ioConfig_t);
    EXTIHandlerInit(&mut a7105extiCallbackRec,
                    Some(a7105extiHandler as
                             unsafe extern "C" fn(_: *mut extiCallbackRec_t)
                                 -> ()));
    EXTIConfig(rxIntIO, &mut a7105extiCallbackRec,
               ((0xf as libc::c_int) <<
                    (4 as libc::c_int as
                         libc::c_uint).wrapping_sub((7 as libc::c_int as
                                                         libc::c_uint).wrapping_sub(0x500
                                                                                        as
                                                                                        libc::c_int
                                                                                        as
                                                                                        uint32_t
                                                                                        >>
                                                                                        8
                                                                                            as
                                                                                            libc::c_int))
                    |
                    0xf as libc::c_int &
                        0xf as libc::c_int >>
                            (7 as libc::c_int as
                                 libc::c_uint).wrapping_sub(0x500 as
                                                                libc::c_int as
                                                                uint32_t >>
                                                                8 as
                                                                    libc::c_int))
                   << 4 as libc::c_int & 0xf0 as libc::c_int,
               EXTI_Trigger_Rising);
    EXTIEnable(rxIntIO, 0 as libc::c_int != 0);
    A7105SoftReset();
    A7105WriteID(id);
}
#[no_mangle]
pub unsafe extern "C" fn A7105Config(mut regsTable: *const uint8_t,
                                     mut size: uint8_t) {
    if !regsTable.is_null() {
        let mut timeout: uint32_t = 1000 as libc::c_int as uint32_t;
        let mut i: uint8_t = 0 as libc::c_int as uint8_t;
        while (i as libc::c_int) < size as libc::c_int {
            if *regsTable.offset(i as isize) as libc::c_int !=
                   0xff as libc::c_int {
                A7105WriteReg(i as A7105Reg_t, *regsTable.offset(i as isize));
            }
            i = i.wrapping_add(1)
        }
        A7105Strobe(A7105_STANDBY);
        A7105WriteReg(A7105_02_CALC, 0x1 as libc::c_int as uint8_t);
        while A7105ReadReg(A7105_02_CALC) as libc::c_int != 0 as libc::c_int
                  &&
                  {
                      let fresh0 = timeout;
                      timeout = timeout.wrapping_sub(1);
                      (fresh0) != 0
                  } {
        }
        A7105ReadReg(A7105_22_IF_CALIB_I);
        A7105WriteReg(A7105_24_VCO_CURCAL, 0x13 as libc::c_int as uint8_t);
        A7105WriteReg(A7105_25_VCO_SBCAL_I, 0x9 as libc::c_int as uint8_t);
        A7105Strobe(A7105_STANDBY);
    };
}
#[no_mangle]
pub unsafe extern "C" fn A7105RxTxFinished(mut timeStamp: *mut uint32_t)
 -> bool {
    let mut result: bool = 0 as libc::c_int != 0;
    if occurEvent {
        if !timeStamp.is_null() { *timeStamp = timeEvent }
        ::core::ptr::write_volatile(&mut occurEvent as *mut bool,
                                    0 as libc::c_int != 0);
        result = 1 as libc::c_int != 0
    }
    return result;
}
#[no_mangle]
pub unsafe extern "C" fn A7105SoftReset() {
    rxSpiWriteCommand(A7105_00_MODE as libc::c_int as uint8_t,
                      0 as libc::c_int as uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn A7105ReadReg(mut reg: A7105Reg_t) -> uint8_t {
    return rxSpiReadCommand((reg as uint8_t as libc::c_int |
                                 0x40 as libc::c_int) as uint8_t,
                            0xff as libc::c_int as uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn A7105WriteReg(mut reg: A7105Reg_t,
                                       mut data: uint8_t) {
    rxSpiWriteCommand(reg as uint8_t, data);
}
#[no_mangle]
pub unsafe extern "C" fn A7105Strobe(mut state: A7105State_t) {
    if A7105_TX as libc::c_int as libc::c_uint == state as libc::c_uint ||
           A7105_RX as libc::c_int as libc::c_uint == state as libc::c_uint {
        EXTIEnable(rxIntIO, 1 as libc::c_int != 0);
    } else { EXTIEnable(rxIntIO, 0 as libc::c_int != 0); }
    rxSpiWriteByte(state as uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn A7105WriteID(mut id: uint32_t) {
    let mut data: [uint8_t; 4] = [0; 4];
    data[0 as libc::c_int as usize] =
        (id >> 24 as libc::c_int & 0xff as libc::c_int as libc::c_uint) as
            uint8_t;
    data[1 as libc::c_int as usize] =
        (id >> 16 as libc::c_int & 0xff as libc::c_int as libc::c_uint) as
            uint8_t;
    data[2 as libc::c_int as usize] =
        (id >> 8 as libc::c_int & 0xff as libc::c_int as libc::c_uint) as
            uint8_t;
    data[3 as libc::c_int as usize] =
        (id >> 0 as libc::c_int & 0xff as libc::c_int as libc::c_uint) as
            uint8_t;
    rxSpiWriteCommandMulti(A7105_06_ID_DATA as libc::c_int as uint8_t,
                           &mut *data.as_mut_ptr().offset(0 as libc::c_int as
                                                              isize),
                           ::core::mem::size_of::<[uint8_t; 4]>() as
                               libc::c_ulong as uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn A7105ReadID() -> uint32_t {
    let mut id: uint32_t = 0;
    let mut data: [uint8_t; 4] = [0; 4];
    rxSpiReadCommandMulti((A7105_06_ID_DATA as libc::c_int as uint8_t as
                               libc::c_int | 0x40 as libc::c_int) as uint8_t,
                          0xff as libc::c_int as uint8_t,
                          &mut *data.as_mut_ptr().offset(0 as libc::c_int as
                                                             isize),
                          ::core::mem::size_of::<[uint8_t; 4]>() as
                              libc::c_ulong as uint8_t);
    id =
        ((data[0 as libc::c_int as usize] as libc::c_int) << 24 as libc::c_int
             |
             (data[1 as libc::c_int as usize] as libc::c_int) <<
                 16 as libc::c_int |
             (data[2 as libc::c_int as usize] as libc::c_int) <<
                 8 as libc::c_int |
             (data[3 as libc::c_int as usize] as libc::c_int) <<
                 0 as libc::c_int) as uint32_t;
    return id;
}
#[no_mangle]
pub unsafe extern "C" fn A7105ReadFIFO(mut data: *mut uint8_t,
                                       mut num: uint8_t) {
    if !data.is_null() {
        if num as libc::c_int > 64 as libc::c_int {
            num = 64 as libc::c_int as uint8_t
        }
        A7105Strobe(A7105_RST_RDPTR);
        rxSpiReadCommandMulti((A7105_05_FIFO_DATA as libc::c_int as uint8_t as
                                   libc::c_int | 0x40 as libc::c_int) as
                                  uint8_t, 0xff as libc::c_int as uint8_t,
                              data, num);
    };
}
#[no_mangle]
pub unsafe extern "C" fn A7105WriteFIFO(mut data: *mut uint8_t,
                                        mut num: uint8_t) {
    if !data.is_null() {
        if num as libc::c_int > 64 as libc::c_int {
            num = 64 as libc::c_int as uint8_t
        }
        A7105Strobe(A7105_RST_WRPTR);
        rxSpiWriteCommandMulti(A7105_05_FIFO_DATA as libc::c_int as uint8_t,
                               data, num);
    };
}
/* USE_RX_FLYSKY */
