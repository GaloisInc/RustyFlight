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
    static mut debug: [int16_t; 4];
    #[no_mangle]
    fn mpuGyroInit(gyro: *mut gyroDev_s);
    #[no_mangle]
    fn mpuGyroRead(gyro: *mut gyroDev_s) -> bool;
    #[no_mangle]
    fn mpuGyroReadSPI(gyro: *mut gyroDev_s) -> bool;
    #[no_mangle]
    fn mpuGyroDLPF(gyro: *mut gyroDev_s) -> uint8_t;
    #[no_mangle]
    fn mpuGyroFCHOICE(gyro: *mut gyroDev_s) -> uint8_t;
    #[no_mangle]
    fn mpuAccRead(acc: *mut accDev_s) -> bool;
    #[no_mangle]
    fn spiSetDivisor(instance: *mut SPI_TypeDef, divisor: uint16_t);
    #[no_mangle]
    fn spiTransferByte(instance: *mut SPI_TypeDef, data: uint8_t) -> uint8_t;
    #[no_mangle]
    fn spiTransfer(instance: *mut SPI_TypeDef, txData: *const uint8_t,
                   rxData: *mut uint8_t, len: libc::c_int) -> bool;
    #[no_mangle]
    fn spiGetErrorCounter(instance: *mut SPI_TypeDef) -> uint16_t;
    #[no_mangle]
    fn spiResetErrorCounter(instance: *mut SPI_TypeDef);
    #[no_mangle]
    fn spiBusReadRegister(bus: *const busDevice_t, reg: uint8_t) -> uint8_t;
    #[no_mangle]
    fn IOHi(io: IO_t);
    #[no_mangle]
    fn IOLo(io: IO_t);
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
    #[no_mangle]
    fn delay(ms: timeMs_t);
    #[no_mangle]
    fn failureMode(mode: failureMode_e);
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* *
  * @brief DMA Controller
  */
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
/* *
  * @brief Serial Peripheral Interface
  */
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct extiCallbackRec_s {
    pub fn_0: Option<extiHandlerCallback>,
}
pub type extiHandlerCallback
    =
    unsafe extern "C" fn(_: *mut extiCallbackRec_t) -> ();
pub type extiCallbackRec_t = extiCallbackRec_s;
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
pub type I2CDevice = libc::c_int;
pub const I2CDEV_4: I2CDevice = 3;
pub const I2CDEV_3: I2CDevice = 2;
pub const I2CDEV_2: I2CDevice = 1;
pub const I2CDEV_1: I2CDevice = 0;
pub const I2CINVALID: I2CDevice = -1;
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
// Slave I2C on SPI master
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
pub type sensor_align_e = libc::c_uint;
pub const CW270_DEG_FLIP: sensor_align_e = 8;
pub const CW180_DEG_FLIP: sensor_align_e = 7;
pub const CW90_DEG_FLIP: sensor_align_e = 6;
pub const CW0_DEG_FLIP: sensor_align_e = 5;
pub const CW270_DEG: sensor_align_e = 4;
pub const CW180_DEG: sensor_align_e = 3;
pub const CW90_DEG: sensor_align_e = 2;
// driver-provided alignment
pub const CW0_DEG: sensor_align_e = 1;
pub const ALIGN_DEFAULT: sensor_align_e = 0;
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
// initialize function
// read 3 axis data function
// read temperature if available
// scalefactor
// gyro data after calibration and alignment
#[derive(Copy, Clone)]
#[repr(C)]
pub struct accDev_s {
    pub initFn: sensorAccInitFuncPtr,
    pub readFn: sensorAccReadFuncPtr,
    pub bus: busDevice_t,
    pub acc_1G: uint16_t,
    pub ADCRaw: [int16_t; 3],
    pub mpuDetectionResult: mpuDetectionResult_t,
    pub accAlign: sensor_align_e,
    pub dataReady: bool,
    pub acc_high_fsr: bool,
    pub revisionCode: libc::c_char,
    pub filler: [uint8_t; 2],
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
//#define DEBUG_MPU_DATA_READY_INTERRUPT
// MPU6050
// MPU3050, 6000 and 6050
// RA = Register Address
//[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
//[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
//[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
//[7:0] X_FINE_GAIN
//[7:0] Y_FINE_GAIN
//[7:0] Z_FINE_GAIN
//[15:0] XA_OFFS
//[15:0] YA_OFFS
//[15:0] ZA_OFFS
// Product ID Register
//[15:0] XG_OFFS_USR
//[15:0] YG_OFFS_USR
//[15:0] ZG_OFFS_USR
// RF = Register Flag
pub type mpuDetectionResult_t = mpuDetectionResult_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mpuDetectionResult_s {
    pub sensor: mpuSensor_e,
    pub resolution: mpu6050Resolution_e,
}
pub type mpu6050Resolution_e = libc::c_uint;
pub const MPU_FULL_RESOLUTION: mpu6050Resolution_e = 1;
pub const MPU_HALF_RESOLUTION: mpu6050Resolution_e = 0;
pub type mpuSensor_e = libc::c_uint;
pub const BMI_160_SPI: mpuSensor_e = 12;
pub const ICM_20689_SPI: mpuSensor_e = 11;
pub const ICM_20649_SPI: mpuSensor_e = 10;
pub const ICM_20608_SPI: mpuSensor_e = 9;
pub const ICM_20602_SPI: mpuSensor_e = 8;
pub const ICM_20601_SPI: mpuSensor_e = 7;
pub const MPU_9250_SPI: mpuSensor_e = 6;
pub const MPU_65xx_SPI: mpuSensor_e = 5;
pub const MPU_65xx_I2C: mpuSensor_e = 4;
pub const MPU_60x0_SPI: mpuSensor_e = 3;
pub const MPU_60x0: mpuSensor_e = 2;
pub const MPU_3050: mpuSensor_e = 1;
pub const MPU_NONE: mpuSensor_e = 0;
pub type sensorAccReadFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut accDev_s) -> bool>;
pub type sensorAccInitFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut accDev_s) -> ()>;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gyroDev_s {
    pub initFn: sensorGyroInitFuncPtr,
    pub readFn: sensorGyroReadFuncPtr,
    pub temperatureFn: sensorGyroReadDataFuncPtr,
    pub exti: extiCallbackRec_t,
    pub bus: busDevice_t,
    pub scale: libc::c_float,
    pub gyroZero: [libc::c_float; 3],
    pub gyroADC: [libc::c_float; 3],
    pub gyroADCf: [libc::c_float; 3],
    pub gyroADCRawPrevious: [int32_t; 3],
    pub gyroADCRaw: [int16_t; 3],
    pub temperature: int16_t,
    pub mpuConfiguration: mpuConfiguration_t,
    pub mpuDetectionResult: mpuDetectionResult_t,
    pub gyroAlign: sensor_align_e,
    pub gyroRateKHz: gyroRateKHz_e,
    pub dataReady: bool,
    pub gyro_high_fsr: bool,
    pub hardware_lpf: uint8_t,
    pub hardware_32khz_lpf: uint8_t,
    pub mpuDividerDrops: uint8_t,
    pub mpuIntExtiTag: ioTag_t,
    pub gyroHasOverflowProtection: uint8_t,
    pub gyroHardware: gyroSensor_e,
}
pub type gyroSensor_e = libc::c_uint;
pub const GYRO_FAKE: gyroSensor_e = 15;
pub const GYRO_BMI160: gyroSensor_e = 14;
pub const GYRO_ICM20689: gyroSensor_e = 13;
pub const GYRO_ICM20649: gyroSensor_e = 12;
pub const GYRO_ICM20608G: gyroSensor_e = 11;
pub const GYRO_ICM20602: gyroSensor_e = 10;
pub const GYRO_ICM20601: gyroSensor_e = 9;
pub const GYRO_MPU9250: gyroSensor_e = 8;
pub const GYRO_MPU6500: gyroSensor_e = 7;
pub const GYRO_MPU6000: gyroSensor_e = 6;
pub const GYRO_L3GD20: gyroSensor_e = 5;
pub const GYRO_MPU3050: gyroSensor_e = 4;
pub const GYRO_L3G4200D: gyroSensor_e = 3;
pub const GYRO_MPU6050: gyroSensor_e = 2;
pub const GYRO_DEFAULT: gyroSensor_e = 1;
pub const GYRO_NONE: gyroSensor_e = 0;
pub type gyroRateKHz_e = libc::c_uint;
pub const GYRO_RATE_32_kHz: gyroRateKHz_e = 5;
pub const GYRO_RATE_9_kHz: gyroRateKHz_e = 4;
pub const GYRO_RATE_8_kHz: gyroRateKHz_e = 3;
pub const GYRO_RATE_3200_Hz: gyroRateKHz_e = 2;
pub const GYRO_RATE_1100_Hz: gyroRateKHz_e = 1;
pub const GYRO_RATE_1_kHz: gyroRateKHz_e = 0;
pub type mpuConfiguration_t = mpuConfiguration_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mpuConfiguration_s {
    pub resetFn: mpuResetFnPtr,
}
pub type mpuResetFnPtr = Option<unsafe extern "C" fn() -> ()>;
pub type sensorGyroReadDataFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut gyroDev_s, _: *mut int16_t) -> bool>;
pub type sensorGyroReadFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut gyroDev_s) -> bool>;
pub type sensorGyroInitFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut gyroDev_s) -> ()>;
pub type gyro_fsr_e = libc::c_uint;
pub const NUM_GYRO_FSR: gyro_fsr_e = 4;
pub const INV_FSR_2000DPS: gyro_fsr_e = 3;
pub const INV_FSR_1000DPS: gyro_fsr_e = 2;
pub const INV_FSR_500DPS: gyro_fsr_e = 1;
pub const INV_FSR_250DPS: gyro_fsr_e = 0;
pub type clock_sel_e = libc::c_uint;
pub const NUM_CLK: clock_sel_e = 2;
pub const INV_CLK_PLL: clock_sel_e = 1;
pub const INV_CLK_INTERNAL: clock_sel_e = 0;
pub type accel_fsr_e = libc::c_uint;
pub const NUM_ACCEL_FSR: accel_fsr_e = 4;
pub const INV_FSR_16G: accel_fsr_e = 3;
pub const INV_FSR_8G: accel_fsr_e = 2;
pub const INV_FSR_4G: accel_fsr_e = 1;
pub const INV_FSR_2G: accel_fsr_e = 0;
pub type timeMs_t = uint32_t;
pub type timeUs_t = uint32_t;
pub type gyroDev_t = gyroDev_s;
pub type accDev_t = accDev_s;
pub const SPI_CLOCK_FAST: C2RustUnnamed_0 = 8;
pub const SPI_CLOCK_INITIALIZATON: C2RustUnnamed_0 = 256;
pub type failureMode_e = libc::c_uint;
pub const FAILURE_GYRO_INIT_FAILED: failureMode_e = 6;
pub const FAILURE_FLASH_WRITE_FAILED: failureMode_e = 5;
pub const FAILURE_INVALID_EEPROM_CONTENTS: failureMode_e = 4;
pub const FAILURE_ACC_INCOMPATIBLE: failureMode_e = 3;
pub const FAILURE_ACC_INIT: failureMode_e = 2;
pub const FAILURE_MISSING_ACC: failureMode_e = 1;
pub const FAILURE_DEVELOPER: failureMode_e = 0;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const SPI_CLOCK_ULTRAFAST: C2RustUnnamed_0 = 2;
pub const SPI_CLOCK_STANDARD: C2RustUnnamed_0 = 16;
pub const SPI_CLOCK_SLOW: C2RustUnnamed_0 = 256;
static mut mpuSpi9250InitDone: bool = 0 as libc::c_int != 0;
#[no_mangle]
pub unsafe extern "C" fn mpu9250SpiWriteRegister(mut bus: *const busDevice_t,
                                                 mut reg: uint8_t,
                                                 mut data: uint8_t) -> bool {
    IOLo((*bus).busdev_u.spi.csnPin); // read transaction
    delayMicroseconds(1 as libc::c_int as timeUs_t);
    spiTransferByte((*bus).busdev_u.spi.instance, reg);
    spiTransferByte((*bus).busdev_u.spi.instance, data);
    IOHi((*bus).busdev_u.spi.csnPin);
    delayMicroseconds(1 as libc::c_int as timeUs_t);
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn mpu9250SpiSlowReadRegisterBuffer(mut bus:
                                                          *const busDevice_t,
                                                      mut reg: uint8_t,
                                                      mut data: *mut uint8_t,
                                                      mut length: uint8_t)
 -> bool {
    IOLo((*bus).busdev_u.spi.csnPin);
    delayMicroseconds(1 as libc::c_int as timeUs_t);
    spiTransferByte((*bus).busdev_u.spi.instance,
                    (reg as libc::c_int | 0x80 as libc::c_int) as uint8_t);
    spiTransfer((*bus).busdev_u.spi.instance, 0 as *const uint8_t, data,
                length as libc::c_int);
    IOHi((*bus).busdev_u.spi.csnPin);
    delayMicroseconds(1 as libc::c_int as timeUs_t);
    return 1 as libc::c_int != 0;
}
// RF = Register Flag
#[no_mangle]
pub unsafe extern "C" fn mpu9250SpiResetGyro() { }
#[no_mangle]
pub unsafe extern "C" fn mpu9250SpiGyroInit(mut gyro: *mut gyroDev_t) {
    mpuGyroInit(gyro); //high speed now that we don't need to write to the slow registers
    mpu9250AccAndGyroInit(gyro);
    spiResetErrorCounter((*gyro).bus.busdev_u.spi.instance);
    spiSetDivisor((*gyro).bus.busdev_u.spi.instance,
                  SPI_CLOCK_FAST as libc::c_int as uint16_t);
    mpuGyroRead(gyro);
    if (*gyro).gyroADCRaw[1 as libc::c_int as usize] as int8_t as libc::c_int
           == -(1 as libc::c_int) &&
           (*gyro).gyroADCRaw[0 as libc::c_int as usize] as int8_t as
               libc::c_int == -(1 as libc::c_int) ||
           spiGetErrorCounter((*gyro).bus.busdev_u.spi.instance) as
               libc::c_int != 0 as libc::c_int {
        spiResetErrorCounter((*gyro).bus.busdev_u.spi.instance);
        failureMode(FAILURE_GYRO_INIT_FAILED);
    };
}
#[no_mangle]
pub unsafe extern "C" fn mpu9250SpiAccInit(mut acc: *mut accDev_t) {
    (*acc).acc_1G = (512 as libc::c_int * 4 as libc::c_int) as uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn mpu9250SpiWriteRegisterVerify(mut bus:
                                                           *const busDevice_t,
                                                       mut reg: uint8_t,
                                                       mut data: uint8_t)
 -> bool {
    mpu9250SpiWriteRegister(bus, reg, data);
    delayMicroseconds(100 as libc::c_int as timeUs_t);
    let mut attemptsRemaining: uint8_t = 20 as libc::c_int as uint8_t;
    loop  {
        let mut in_0: uint8_t = 0;
        mpu9250SpiSlowReadRegisterBuffer(bus, reg, &mut in_0,
                                         1 as libc::c_int as uint8_t);
        if in_0 as libc::c_int == data as libc::c_int {
            return 1 as libc::c_int != 0
        } else {
            debug[3 as libc::c_int as usize] += 1;
            mpu9250SpiWriteRegister(bus, reg, data);
            delayMicroseconds(100 as libc::c_int as timeUs_t);
        }
        let fresh0 = attemptsRemaining;
        attemptsRemaining = attemptsRemaining.wrapping_sub(1);
        if !(fresh0 != 0) { break ; }
    }
    return 0 as libc::c_int != 0;
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
/*
 * Authors:
 * Dominic Clifton - Cleanflight implementation
 * John Ihlein - Initial FF32 code
 * Kalyn Doerr (RS2K) - Robust rewrite
*/
unsafe extern "C" fn mpu9250AccAndGyroInit(mut gyro: *mut gyroDev_t) {
    if mpuSpi9250InitDone { return } //low speed for writing to slow registers
    spiSetDivisor((*gyro).bus.busdev_u.spi.instance,
                  SPI_CLOCK_INITIALIZATON as libc::c_int as
                      uint16_t); // INT_ANYRD_2CLEAR, BYPASS_EN
    mpu9250SpiWriteRegister(&mut (*gyro).bus, 0x6b as libc::c_int as uint8_t,
                            0x80 as libc::c_int as uint8_t);
    delay(50 as libc::c_int as timeMs_t);
    mpu9250SpiWriteRegisterVerify(&mut (*gyro).bus,
                                  0x6b as libc::c_int as uint8_t,
                                  INV_CLK_PLL as libc::c_int as uint8_t);
    mpu9250SpiWriteRegisterVerify(&mut (*gyro).bus,
                                  0x1b as libc::c_int as uint8_t,
                                  ((INV_FSR_2000DPS as libc::c_int) <<
                                       3 as libc::c_int |
                                       mpuGyroFCHOICE(gyro) as libc::c_int) as
                                      uint8_t);
    mpu9250SpiWriteRegisterVerify(&mut (*gyro).bus,
                                  0x1a as libc::c_int as uint8_t,
                                  mpuGyroDLPF(gyro));
    mpu9250SpiWriteRegisterVerify(&mut (*gyro).bus,
                                  0x19 as libc::c_int as uint8_t,
                                  (*gyro).mpuDividerDrops);
    mpu9250SpiWriteRegisterVerify(&mut (*gyro).bus,
                                  0x1c as libc::c_int as uint8_t,
                                  ((INV_FSR_16G as libc::c_int) <<
                                       3 as libc::c_int) as uint8_t);
    mpu9250SpiWriteRegisterVerify(&mut (*gyro).bus,
                                  0x37 as libc::c_int as uint8_t,
                                  ((0 as libc::c_int) << 7 as libc::c_int |
                                       (0 as libc::c_int) << 6 as libc::c_int
                                       |
                                       (0 as libc::c_int) << 5 as libc::c_int
                                       |
                                       (1 as libc::c_int) << 4 as libc::c_int
                                       |
                                       (0 as libc::c_int) << 3 as libc::c_int
                                       |
                                       (0 as libc::c_int) << 2 as libc::c_int
                                       |
                                       (1 as libc::c_int) << 1 as libc::c_int
                                       |
                                       (0 as libc::c_int) << 0 as libc::c_int)
                                      as uint8_t);
    spiSetDivisor((*gyro).bus.busdev_u.spi.instance,
                  SPI_CLOCK_FAST as libc::c_int as uint16_t);
    mpuSpi9250InitDone = 1 as libc::c_int != 0;
    //init done
}
#[no_mangle]
pub unsafe extern "C" fn mpu9250SpiDetect(mut bus: *const busDevice_t)
 -> uint8_t {
    spiSetDivisor((*bus).busdev_u.spi.instance,
                  SPI_CLOCK_INITIALIZATON as libc::c_int as
                      uint16_t); //low speed
    mpu9250SpiWriteRegister(bus, 0x6b as libc::c_int as uint8_t,
                            0x80 as libc::c_int as uint8_t);
    let mut attemptsRemaining: uint8_t = 20 as libc::c_int as uint8_t;
    loop  {
        delay(150 as libc::c_int as timeMs_t);
        let in_0: uint8_t =
            spiBusReadRegister(bus, 0x75 as libc::c_int as uint8_t);
        if in_0 as libc::c_int == 0x71 as libc::c_int ||
               in_0 as libc::c_int == 0x73 as libc::c_int {
            break ;
        }
        if attemptsRemaining == 0 {
            return MPU_NONE as libc::c_int as uint8_t
        }
        let fresh1 = attemptsRemaining;
        attemptsRemaining = attemptsRemaining.wrapping_sub(1);
        if !(fresh1 != 0) { break ; }
    }
    spiSetDivisor((*bus).busdev_u.spi.instance,
                  SPI_CLOCK_FAST as libc::c_int as uint16_t);
    return MPU_9250_SPI as libc::c_int as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn mpu9250SpiAccDetect(mut acc: *mut accDev_t) -> bool {
    if (*acc).mpuDetectionResult.sensor as libc::c_uint !=
           MPU_9250_SPI as libc::c_int as libc::c_uint {
        return 0 as libc::c_int != 0
    }
    (*acc).initFn =
        Some(mpu9250SpiAccInit as
                 unsafe extern "C" fn(_: *mut accDev_t) -> ());
    (*acc).readFn =
        Some(mpuAccRead as unsafe extern "C" fn(_: *mut accDev_s) -> bool);
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn mpu9250SpiGyroDetect(mut gyro: *mut gyroDev_t)
 -> bool {
    if (*gyro).mpuDetectionResult.sensor as libc::c_uint !=
           MPU_9250_SPI as libc::c_int as libc::c_uint {
        return 0 as libc::c_int != 0
    }
    (*gyro).initFn =
        Some(mpu9250SpiGyroInit as
                 unsafe extern "C" fn(_: *mut gyroDev_t) -> ());
    (*gyro).readFn =
        Some(mpuGyroReadSPI as
                 unsafe extern "C" fn(_: *mut gyroDev_s) -> bool);
    // 16.4 dps/lsb scalefactor
    (*gyro).scale = 1.0f32 / 16.4f32;
    return 1 as libc::c_int != 0;
}
