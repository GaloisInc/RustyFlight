use ::libc;
extern "C" {
    #[no_mangle]
    fn spiSetDivisor(instance: *mut SPI_TypeDef, divisor: uint16_t);
    #[no_mangle]
    fn spiBusWriteRegister(bus: *const busDevice_t, reg: uint8_t,
                           data: uint8_t) -> bool;
    #[no_mangle]
    fn spiBusReadRegisterBuffer(bus: *const busDevice_t, reg: uint8_t,
                                data: *mut uint8_t, length: uint8_t) -> bool;
    #[no_mangle]
    fn IOHi(io: IO_t);
    // unimplemented
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn millis() -> timeMs_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_Stream_TypeDef {
    pub CR: uint32_t,
    pub NDTR: uint32_t,
    pub PAR: uint32_t,
    pub M0AR: uint32_t,
    pub M1AR: uint32_t,
    pub FCR: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SPI_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub SR: uint32_t,
    pub DR: uint32_t,
    pub CRCPR: uint32_t,
    pub RXCRCR: uint32_t,
    pub TXCRCR: uint32_t,
    pub I2SCFGR: uint32_t,
    pub I2SPR: uint32_t,
}
/* * 
  * @brief  HAL Lock structures definition  
  */
pub type HAL_LockTypeDef = libc::c_uint;
pub const HAL_LOCKED: HAL_LockTypeDef = 1;
pub const HAL_UNLOCKED: HAL_LockTypeDef = 0;
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_dma.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of DMA HAL module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F7xx_HAL_Driver
  * @{
  */
/* * @addtogroup DMA
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup DMA_Exported_Types DMA Exported Types
  * @brief    DMA Exported Types 
  * @{
  */
/* * 
  * @brief  DMA Configuration Structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_InitTypeDef {
    pub Channel: uint32_t,
    pub Direction: uint32_t,
    pub PeriphInc: uint32_t,
    pub MemInc: uint32_t,
    pub PeriphDataAlignment: uint32_t,
    pub MemDataAlignment: uint32_t,
    pub Mode: uint32_t,
    pub Priority: uint32_t,
    pub FIFOMode: uint32_t,
    pub FIFOThreshold: uint32_t,
    pub MemBurst: uint32_t,
    pub PeriphBurst: uint32_t,
}
/* * 
  * @brief  HAL DMA State structures definition
  */
pub type HAL_DMA_StateTypeDef = libc::c_uint;
/* !< DMA Abort state                     */
/* !< DMA error state                     */
pub const HAL_DMA_STATE_ABORT: HAL_DMA_StateTypeDef = 5;
/* !< DMA timeout state                   */
pub const HAL_DMA_STATE_ERROR: HAL_DMA_StateTypeDef = 4;
/* !< DMA process is ongoing              */
pub const HAL_DMA_STATE_TIMEOUT: HAL_DMA_StateTypeDef = 3;
/* !< DMA initialized and ready for use   */
pub const HAL_DMA_STATE_BUSY: HAL_DMA_StateTypeDef = 2;
/* !< DMA not yet initialized or disabled */
pub const HAL_DMA_STATE_READY: HAL_DMA_StateTypeDef = 1;
pub const HAL_DMA_STATE_RESET: HAL_DMA_StateTypeDef = 0;
/* * 
  * @brief  DMA handle Structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct __DMA_HandleTypeDef {
    pub Instance: *mut DMA_Stream_TypeDef,
    pub Init: DMA_InitTypeDef,
    pub Lock: HAL_LockTypeDef,
    pub State: HAL_DMA_StateTypeDef,
    pub Parent: *mut libc::c_void,
    pub XferCpltCallback: Option<unsafe extern "C" fn(_:
                                                          *mut __DMA_HandleTypeDef)
                                     -> ()>,
    pub XferHalfCpltCallback: Option<unsafe extern "C" fn(_:
                                                              *mut __DMA_HandleTypeDef)
                                         -> ()>,
    pub XferM1CpltCallback: Option<unsafe extern "C" fn(_:
                                                            *mut __DMA_HandleTypeDef)
                                       -> ()>,
    pub XferM1HalfCpltCallback: Option<unsafe extern "C" fn(_:
                                                                *mut __DMA_HandleTypeDef)
                                           -> ()>,
    pub XferErrorCallback: Option<unsafe extern "C" fn(_:
                                                           *mut __DMA_HandleTypeDef)
                                      -> ()>,
    pub XferAbortCallback: Option<unsafe extern "C" fn(_:
                                                           *mut __DMA_HandleTypeDef)
                                      -> ()>,
    pub ErrorCode: uint32_t,
    pub StreamBaseAddress: uint32_t,
    pub StreamIndex: uint32_t,
}
pub type DMA_HandleTypeDef = __DMA_HandleTypeDef;
/* !< DMA Stream Index                       */
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_spi.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of SPI HAL module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F7xx_HAL_Driver
  * @{
  */
/* * @addtogroup SPI
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup SPI_Exported_Types SPI Exported Types
  * @{
  */
/* *
  * @brief  SPI Configuration Structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SPI_InitTypeDef {
    pub Mode: uint32_t,
    pub Direction: uint32_t,
    pub DataSize: uint32_t,
    pub CLKPolarity: uint32_t,
    pub CLKPhase: uint32_t,
    pub NSS: uint32_t,
    pub BaudRatePrescaler: uint32_t,
    pub FirstBit: uint32_t,
    pub TIMode: uint32_t,
    pub CRCCalculation: uint32_t,
    pub CRCPolynomial: uint32_t,
    pub CRCLength: uint32_t,
    pub NSSPMode: uint32_t,
}
/* *
  * @brief  HAL SPI State structure definition
  */
pub type HAL_SPI_StateTypeDef = libc::c_uint;
/* !< SPI abort is ongoing                               */
/* !< SPI error state                                    */
pub const HAL_SPI_STATE_ABORT: HAL_SPI_StateTypeDef = 7;
/* !< Data Transmission and Reception process is ongoing */
pub const HAL_SPI_STATE_ERROR: HAL_SPI_StateTypeDef = 6;
/* !< Data Reception process is ongoing                  */
pub const HAL_SPI_STATE_BUSY_TX_RX: HAL_SPI_StateTypeDef = 5;
/* !< Data Transmission process is ongoing               */
pub const HAL_SPI_STATE_BUSY_RX: HAL_SPI_StateTypeDef = 4;
/* !< an internal process is ongoing                     */
pub const HAL_SPI_STATE_BUSY_TX: HAL_SPI_StateTypeDef = 3;
/* !< Peripheral Initialized and ready for use           */
pub const HAL_SPI_STATE_BUSY: HAL_SPI_StateTypeDef = 2;
/* !< Peripheral not Initialized                         */
pub const HAL_SPI_STATE_READY: HAL_SPI_StateTypeDef = 1;
pub const HAL_SPI_STATE_RESET: HAL_SPI_StateTypeDef = 0;
/* *
  * @brief  SPI handle Structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct __SPI_HandleTypeDef {
    pub Instance: *mut SPI_TypeDef,
    pub Init: SPI_InitTypeDef,
    pub pTxBuffPtr: *mut uint8_t,
    pub TxXferSize: uint16_t,
    pub TxXferCount: uint16_t,
    pub pRxBuffPtr: *mut uint8_t,
    pub RxXferSize: uint16_t,
    pub RxXferCount: uint16_t,
    pub CRCSize: uint32_t,
    pub RxISR: Option<unsafe extern "C" fn(_: *mut __SPI_HandleTypeDef)
                          -> ()>,
    pub TxISR: Option<unsafe extern "C" fn(_: *mut __SPI_HandleTypeDef)
                          -> ()>,
    pub hdmatx: *mut DMA_HandleTypeDef,
    pub hdmarx: *mut DMA_HandleTypeDef,
    pub Lock: HAL_LockTypeDef,
    pub State: HAL_SPI_StateTypeDef,
    pub ErrorCode: uint32_t,
}
pub type SPI_HandleTypeDef = __SPI_HandleTypeDef;
/* !< SPI Error code                           */
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
pub type I2CDevice = libc::c_int;
pub const I2CDEV_4: I2CDevice = 3;
pub const I2CDEV_3: I2CDevice = 2;
pub const I2CDEV_2: I2CDevice = 1;
pub const I2CDEV_1: I2CDevice = 0;
pub const I2CINVALID: I2CDevice = -1;
pub type busType_e = libc::c_uint;
pub const BUSTYPE_MPU_SLAVE: busType_e = 3;
pub const BUSTYPE_SPI: busType_e = 2;
pub const BUSTYPE_I2C: busType_e = 1;
pub const BUSTYPE_NONE: busType_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct busDevice_s {
    pub bustype: busType_e,
    pub busdev_u: C2RustUnnamed,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed {
    pub spi: deviceSpi_s,
    pub i2c: deviceI2C_s,
    pub mpuSlave: deviceMpuSlave_s,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceMpuSlave_s {
    pub master: *const busDevice_s,
    pub address: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceI2C_s {
    pub device: I2CDevice,
    pub address: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceSpi_s {
    pub instance: *mut SPI_TypeDef,
    pub handle: *mut SPI_HandleTypeDef,
    pub csnPin: IO_t,
}
pub type busDevice_t = busDevice_s;
// cached here for efficiency
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
// baro start operation
// baro calculation (filled params are pressure and temperature)
#[derive(Copy, Clone)]
#[repr(C)]
pub struct baroDev_s {
    pub busdev: busDevice_t,
    pub ut_delay: uint16_t,
    pub up_delay: uint16_t,
    pub start_ut: baroOpFuncPtr,
    pub get_ut: baroOpFuncPtr,
    pub start_up: baroOpFuncPtr,
    pub get_up: baroOpFuncPtr,
    pub calculate: baroCalculateFuncPtr,
}
pub type baroCalculateFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut int32_t, _: *mut int32_t) -> ()>;
pub type baroOpFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut baroDev_s) -> ()>;
pub type baroDev_t = baroDev_s;
// millisecond time
pub type timeMs_t = uint32_t;
pub const SPI_CLOCK_STANDARD: C2RustUnnamed_0 = 16;
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
pub type C2RustUnnamed_0 = libc::c_uint;
pub const SPI_CLOCK_ULTRAFAST: C2RustUnnamed_0 = 2;
pub const SPI_CLOCK_FAST: C2RustUnnamed_0 = 8;
pub const SPI_CLOCK_SLOW: C2RustUnnamed_0 = 256;
pub const SPI_CLOCK_INITIALIZATON: C2RustUnnamed_0 = 256;
//======================================Moving average number=============================================//
//Raw register values
static mut rawP: uint32_t = 0 as libc::c_int as uint32_t;
static mut rawT: uint16_t = 0 as libc::c_int as uint16_t;
#[no_mangle]
pub unsafe extern "C" fn lpsWriteCommand(mut busdev: *mut busDevice_t,
                                         mut cmd: uint8_t, mut byte: uint8_t)
 -> bool {
    return spiBusWriteRegister(busdev, cmd, byte);
}
#[no_mangle]
pub unsafe extern "C" fn lpsReadCommand(mut busdev: *mut busDevice_t,
                                        mut cmd: uint8_t,
                                        mut data: *mut uint8_t,
                                        mut len: uint8_t) -> bool {
    return spiBusReadRegisterBuffer(busdev,
                                    (cmd as libc::c_int | 0x80 as libc::c_int
                                         | 0x40 as libc::c_int) as uint8_t,
                                    data, len);
}
#[no_mangle]
pub unsafe extern "C" fn lpsWriteVerify(mut busdev: *mut busDevice_t,
                                        mut cmd: uint8_t, mut byte: uint8_t)
 -> bool {
    let mut temp: uint8_t = 0xff as libc::c_int as uint8_t;
    spiBusWriteRegister(busdev, cmd, byte);
    spiBusReadRegisterBuffer(busdev, cmd, &mut temp,
                             1 as libc::c_int as uint8_t);
    if byte as libc::c_int == temp as libc::c_int {
        return 1 as libc::c_int != 0
    }
    return 0 as libc::c_int != 0;
}
unsafe extern "C" fn lpsOn(mut busdev: *mut busDevice_t,
                           mut CTRL1_val: uint8_t) {
    lpsWriteCommand(busdev, 0x20 as libc::c_int as uint8_t,
                    (CTRL1_val as libc::c_int | 0x80 as libc::c_int) as
                        uint8_t);
    //Instead of delay let's ready status reg
}
unsafe extern "C" fn lpsOff(mut busdev: *mut busDevice_t) {
    lpsWriteCommand(busdev, 0x20 as libc::c_int as uint8_t,
                    (0 as libc::c_int |
                         (0x1 as libc::c_int) << 2 as libc::c_int) as
                        uint8_t);
}
unsafe extern "C" fn lps_nothing(mut baro: *mut baroDev_t) { }
unsafe extern "C" fn lps_read(mut baro: *mut baroDev_t) {
    let mut status: uint8_t = 0 as libc::c_int as uint8_t;
    lpsReadCommand(&mut (*baro).busdev, 0x27 as libc::c_int as uint8_t,
                   &mut status, 1 as libc::c_int as uint8_t);
    if status as libc::c_int & 0x3 as libc::c_int != 0 {
        let mut temp: [uint8_t; 5] = [0; 5];
        lpsReadCommand(&mut (*baro).busdev, 0x28 as libc::c_int as uint8_t,
                       temp.as_mut_ptr(), 5 as libc::c_int as uint8_t);
        /* Build the raw data */
        rawP =
            (temp[0 as libc::c_int as usize] as libc::c_int |
                 (temp[1 as libc::c_int as usize] as libc::c_int) <<
                     8 as libc::c_int |
                 (temp[2 as libc::c_int as usize] as libc::c_int) <<
                     16 as libc::c_int) as libc::c_uint |
                (if temp[2 as libc::c_int as usize] as libc::c_int &
                        0x80 as libc::c_int != 0 {
                     0xff000000 as libc::c_uint
                 } else { 0 as libc::c_int as libc::c_uint });
        rawT =
            ((temp[4 as libc::c_int as usize] as libc::c_int) <<
                 8 as libc::c_int |
                 temp[3 as libc::c_int as usize] as libc::c_int) as uint16_t
    } else {
        rawP = 0 as libc::c_int as uint32_t;
        rawT = 0 as libc::c_int as uint16_t
    };
}
unsafe extern "C" fn lps_calculate(mut pressure: *mut int32_t,
                                   mut temperature: *mut int32_t) {
    *pressure = rawP as int32_t * 100 as libc::c_int / 4096 as libc::c_int;
    *temperature =
        rawT as int32_t * 10 as libc::c_int / 48 as libc::c_int +
            4250 as libc::c_int;
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
pub unsafe extern "C" fn lpsDetect(mut baro: *mut baroDev_t) -> bool {
    //Detect
    let mut busdev: *mut busDevice_t = &mut (*baro).busdev; // Disable
    IOInit((*busdev).busdev_u.spi.csnPin, OWNER_BARO_CS,
           0 as libc::c_int as
               uint8_t); // Baro can work only on up to 10Mhz SPI bus
    IOConfigGPIO((*busdev).busdev_u.spi.csnPin,
                 (0x1 as libc::c_uint |
                      (0 as libc::c_uint) << 2 as libc::c_int |
                      (0 as libc::c_uint) << 5 as libc::c_int) as ioConfig_t);
    IOHi((*busdev).busdev_u.spi.csnPin);
    spiSetDivisor((*busdev).busdev_u.spi.instance,
                  SPI_CLOCK_STANDARD as libc::c_int as uint16_t);
    let mut temp: uint8_t = 0 as libc::c_int as uint8_t;
    lpsReadCommand(&mut (*baro).busdev, 0xf as libc::c_int as uint8_t,
                   &mut temp, 1 as libc::c_int as uint8_t);
    if temp as libc::c_int != 0xbd as libc::c_int &&
           temp as libc::c_int != 0xb1 as libc::c_int &&
           temp as libc::c_int != 0xb1 as libc::c_int &&
           temp as libc::c_int != 0xb1 as libc::c_int {
        return 0 as libc::c_int != 0
    }
    //Init, if writeVerify is false fallback to false on detect
    let mut ret: bool = 0 as libc::c_int != 0;
    lpsOff(busdev);
    ret =
        lpsWriteVerify(busdev, 0x21 as libc::c_int as uint8_t,
                       ((0 as libc::c_int) << 1 as libc::c_int) as uint8_t);
    if ret as libc::c_int != 1 as libc::c_int { return 0 as libc::c_int != 0 }
    ret =
        lpsWriteVerify(busdev, 0x10 as libc::c_int as uint8_t,
                       (0xc as libc::c_int | 0x3 as libc::c_int) as uint8_t);
    if ret as libc::c_int != 1 as libc::c_int { return 0 as libc::c_int != 0 }
    ret =
        lpsWriteVerify(busdev, 0x23 as libc::c_int as uint8_t,
                       0x1 as libc::c_int as uint8_t);
    if ret as libc::c_int != 1 as libc::c_int { return 0 as libc::c_int != 0 }
    lpsOn(busdev,
          ((0x4 as libc::c_int) << 4 as libc::c_int |
               (0x1 as libc::c_int) << 1 as libc::c_int |
               (0x1 as libc::c_int) << 2 as libc::c_int |
               (0x1 as libc::c_int) << 3 as libc::c_int) as uint8_t);
    lpsReadCommand(busdev, 0x20 as libc::c_int as uint8_t, &mut temp,
                   1 as libc::c_int as uint8_t);
    (*baro).ut_delay = 1 as libc::c_int as uint16_t;
    (*baro).up_delay =
        (1000000 as libc::c_int / 24 as libc::c_int) as uint16_t;
    (*baro).start_ut =
        Some(lps_nothing as unsafe extern "C" fn(_: *mut baroDev_t) -> ());
    (*baro).get_ut =
        Some(lps_nothing as unsafe extern "C" fn(_: *mut baroDev_t) -> ());
    (*baro).start_up =
        Some(lps_nothing as unsafe extern "C" fn(_: *mut baroDev_t) -> ());
    (*baro).get_up =
        Some(lps_read as unsafe extern "C" fn(_: *mut baroDev_t) -> ());
    (*baro).calculate =
        Some(lps_calculate as
                 unsafe extern "C" fn(_: *mut int32_t, _: *mut int32_t)
                     -> ());
    let mut timeout: uint32_t = millis();
    loop  {
        lps_read(baro);
        if millis().wrapping_sub(timeout) > 500 as libc::c_int as libc::c_uint
           {
            return 0 as libc::c_int != 0
        }
        if !(rawT as libc::c_int == 0 as libc::c_int &&
                 rawP == 0 as libc::c_int as libc::c_uint) {
            break ;
        }
    }
    rawT = 0 as libc::c_int as uint16_t;
    rawP = 0 as libc::c_int as uint32_t;
    return 1 as libc::c_int != 0;
}
