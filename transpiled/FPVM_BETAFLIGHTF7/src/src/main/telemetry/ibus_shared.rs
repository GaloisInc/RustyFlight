use ::libc;
extern "C" {
    #[no_mangle]
    fn serialWrite(instance: *mut serialPort_t, ch: uint8_t);
    #[no_mangle]
    static mut telemetryConfig_System: telemetryConfig_t;
    #[no_mangle]
    fn feature(mask: uint32_t) -> bool;
    #[no_mangle]
    fn getAmperage() -> int32_t;
    #[no_mangle]
    fn getBatteryAverageCellVoltage() -> uint16_t;
    #[no_mangle]
    static mut batteryConfig_System: batteryConfig_t;
    #[no_mangle]
    fn getMAhDrawn() -> int32_t;
    #[no_mangle]
    fn calculateBatteryPercentageRemaining() -> uint8_t;
    #[no_mangle]
    fn getBatteryVoltage() -> uint16_t;
    #[no_mangle]
    fn getBatteryCellCount() -> uint8_t;
    // (Super) rates are constrained to [0, 100] for Betaflight rates, so values higher than 100 won't make a difference. Range extended for RaceFlight rates.
    #[no_mangle]
    static mut rcCommand: [libc::c_float; 4];
    #[no_mangle]
    fn calculateThrottleStatus() -> throttleStatus_e;
    #[no_mangle]
    fn gyroGetTemperature() -> int16_t;
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    static mut flightModeFlags: uint16_t;
    #[no_mangle]
    static mut stateFlags: uint8_t;
    #[no_mangle]
    fn sensors(mask: uint32_t) -> bool;
    #[no_mangle]
    static mut acc: acc_t;
    #[no_mangle]
    static mut baro: baro_t;
    #[no_mangle]
    static mut attitude: attitudeEulerAngles_t;
    #[no_mangle]
    fn getEstimatedVario() -> int16_t;
    #[no_mangle]
    static mut GPS_distanceToHome: uint16_t;
    #[no_mangle]
    static mut gpsSol: gpsSolutionData_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int16_t = __int16_t;
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
pub type portMode_e = libc::c_uint;
pub const MODE_RXTX: portMode_e = 3;
pub const MODE_TX: portMode_e = 2;
pub const MODE_RX: portMode_e = 1;
pub type portOptions_e = libc::c_uint;
pub const SERIAL_BIDIR_NOPULL: portOptions_e = 32;
pub const SERIAL_BIDIR_PP: portOptions_e = 16;
pub const SERIAL_BIDIR_OD: portOptions_e = 0;
pub const SERIAL_BIDIR: portOptions_e = 8;
pub const SERIAL_UNIDIR: portOptions_e = 0;
pub const SERIAL_PARITY_EVEN: portOptions_e = 4;
pub const SERIAL_PARITY_NO: portOptions_e = 0;
pub const SERIAL_STOPBITS_2: portOptions_e = 2;
pub const SERIAL_STOPBITS_1: portOptions_e = 0;
pub const SERIAL_INVERTED: portOptions_e = 1;
pub const SERIAL_NOT_INVERTED: portOptions_e = 0;
// Define known line control states which may be passed up by underlying serial driver callback
pub type serialReceiveCallbackPtr
    =
    Option<unsafe extern "C" fn(_: uint16_t, _: *mut libc::c_void) -> ()>;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPort_s {
    pub vTable: *const serialPortVTable,
    pub mode: portMode_e,
    pub options: portOptions_e,
    pub baudRate: uint32_t,
    pub rxBufferSize: uint32_t,
    pub txBufferSize: uint32_t,
    pub rxBuffer: *mut uint8_t,
    pub txBuffer: *mut uint8_t,
    pub rxBufferHead: uint32_t,
    pub rxBufferTail: uint32_t,
    pub txBufferHead: uint32_t,
    pub txBufferTail: uint32_t,
    pub rxCallback: serialReceiveCallbackPtr,
    pub rxCallbackData: *mut libc::c_void,
    pub identifier: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPortVTable {
    pub serialWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                 _: uint8_t) -> ()>,
    pub serialTotalRxWaiting: Option<unsafe extern "C" fn(_:
                                                              *const serialPort_t)
                                         -> uint32_t>,
    pub serialTotalTxFree: Option<unsafe extern "C" fn(_: *const serialPort_t)
                                      -> uint32_t>,
    pub serialRead: Option<unsafe extern "C" fn(_: *mut serialPort_t)
                               -> uint8_t>,
    pub serialSetBaudRate: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                       _: uint32_t) -> ()>,
    pub isSerialTransmitBufferEmpty: Option<unsafe extern "C" fn(_:
                                                                     *const serialPort_t)
                                                -> bool>,
    pub setMode: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                             _: portMode_e) -> ()>,
    pub setCtrlLineStateCb: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                        _:
                                                            Option<unsafe extern "C" fn(_:
                                                                                            *mut libc::c_void,
                                                                                        _:
                                                                                            uint16_t)
                                                                       -> ()>,
                                                        _: *mut libc::c_void)
                                       -> ()>,
    pub setBaudRateCb: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                   _:
                                                       Option<unsafe extern "C" fn(_:
                                                                                       *mut serialPort_t,
                                                                                   _:
                                                                                       uint32_t)
                                                                  -> ()>,
                                                   _: *mut serialPort_t)
                                  -> ()>,
    pub writeBuf: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                              _: *const libc::c_void,
                                              _: libc::c_int) -> ()>,
    pub beginWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t) -> ()>,
    pub endWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t) -> ()>,
}
// used by serial drivers to return frames to app
pub type serialPort_t = serialPort_s;
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
 * The ibus_shared module implements the ibus telemetry packet handling
 * which is shared between the ibus serial rx and the ibus telemetry.
 *
 * There is only one 'user' active at any time, serial rx will open the
 * serial port if both functions are enabled at the same time
 */
pub type C2RustUnnamed = libc::c_uint;
//defined(TELEMETRY_IBUS_EXTENDED)
pub const IBUS_SENSOR_TYPE_UNKNOWN: C2RustUnnamed = 255;
pub const IBUS_SENSOR_TYPE_ACC_FULL: C2RustUnnamed = 239;
pub const IBUS_SENSOR_TYPE_VOLT_FULL: C2RustUnnamed = 240;
// Altitude 2 bytes signed in m
pub const IBUS_SENSOR_TYPE_GPS_FULL: C2RustUnnamed = 253;
//4bytes signed MaxAlt m*100
pub const IBUS_SENSOR_TYPE_ALT_FLYSKY: C2RustUnnamed = 249;
//4bytes signed!!! Alt m*100
pub const IBUS_SENSOR_TYPE_ALT_MAX: C2RustUnnamed = 132;
//4bytes signed!!! GPS alt m*100
pub const IBUS_SENSOR_TYPE_ALT: C2RustUnnamed = 131;
//4bytes signed WGS84 in degrees * 1E7
pub const IBUS_SENSOR_TYPE_GPS_ALT: C2RustUnnamed = 130;
//4bytes signed WGS84 in degrees * 1E7
pub const IBUS_SENSOR_TYPE_GPS_LON: C2RustUnnamed = 129;
// Speed 2bytes km/h
pub const IBUS_SENSOR_TYPE_GPS_LAT: C2RustUnnamed = 128;
// Odometer2
pub const IBUS_SENSOR_TYPE_SPE: C2RustUnnamed = 126;
// Odometer1
pub const IBUS_SENSOR_TYPE_ODO2: C2RustUnnamed = 125;
// Pressure
pub const IBUS_SENSOR_TYPE_ODO1: C2RustUnnamed = 124;
//2 bytes
pub const IBUS_SENSOR_TYPE_PRES: C2RustUnnamed = 65;
//2 bytes
pub const IBUS_SENSOR_TYPE_FLIGHT_MODE: C2RustUnnamed = 22;
//2 bytes dist from home m unsigned
pub const IBUS_SENSOR_TYPE_ARMED: C2RustUnnamed = 21;
//2 bytes m/s *100 different unit than build-in sensor
pub const IBUS_SENSOR_TYPE_GPS_DIST: C2RustUnnamed = 20;
//2 bytes m/s *100
pub const IBUS_SENSOR_TYPE_GROUND_SPEED: C2RustUnnamed = 19;
//2 bytes deg *100 signed
pub const IBUS_SENSOR_TYPE_VERTICAL_SPEED: C2RustUnnamed = 18;
//2 bytes deg *100 signed
pub const IBUS_SENSOR_TYPE_YAW: C2RustUnnamed = 17;
//2 bytes deg *100 signed
pub const IBUS_SENSOR_TYPE_PITCH: C2RustUnnamed = 16;
//2 bytes m/s *100 signed
pub const IBUS_SENSOR_TYPE_ROLL: C2RustUnnamed = 15;
//2 bytes m/s *100 signed
pub const IBUS_SENSOR_TYPE_ACC_Z: C2RustUnnamed = 14;
//2 bytes m/s *100 signed
pub const IBUS_SENSOR_TYPE_ACC_Y: C2RustUnnamed = 13;
//2 bytes
pub const IBUS_SENSOR_TYPE_ACC_X: C2RustUnnamed = 12;
//2 bytes  Course over ground(NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. unknown max uint
pub const IBUS_SENSOR_TYPE_GPS_STATUS: C2RustUnnamed = 11;
//2 bytes m/s *100
pub const IBUS_SENSOR_TYPE_COG: C2RustUnnamed = 10;
//Heading  0..360 deg, 0=north 2bytes
pub const IBUS_SENSOR_TYPE_CLIMB_RATE: C2RustUnnamed = 9;
// throttle value / battery capacity
pub const IBUS_SENSOR_TYPE_CMP_HEAD: C2RustUnnamed = 8;
// remaining battery percentage / mah drawn otherwise or fuel level no unit!
pub const IBUS_SENSOR_TYPE_RPM: C2RustUnnamed = 7;
// battery current A * 100
pub const IBUS_SENSOR_TYPE_FUEL: C2RustUnnamed = 6;
// Avg Cell voltage
pub const IBUS_SENSOR_TYPE_BAT_CURR: C2RustUnnamed = 5;
pub const IBUS_SENSOR_TYPE_CELL: C2RustUnnamed = 4;
pub const IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE: C2RustUnnamed = 3;
pub const IBUS_SENSOR_TYPE_RPM_FLYSKY: C2RustUnnamed = 2;
pub const IBUS_SENSOR_TYPE_TEMPERATURE: C2RustUnnamed = 1;
pub const IBUS_SENSOR_TYPE_NONE: C2RustUnnamed = 0;
pub type ibusAddress_t = uint8_t;
pub type telemetryConfig_t = telemetryConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct telemetryConfig_s {
    pub gpsNoFixLatitude: int16_t,
    pub gpsNoFixLongitude: int16_t,
    pub telemetry_inverted: uint8_t,
    pub halfDuplex: uint8_t,
    pub frsky_coordinate_format: frskyGpsCoordFormat_e,
    pub frsky_unit: frskyUnit_e,
    pub frsky_vfas_precision: uint8_t,
    pub hottAlarmSoundInterval: uint8_t,
    pub pidValuesAsTelemetry: uint8_t,
    pub report_cell_voltage: uint8_t,
    pub flysky_sensors: [uint8_t; 15],
    pub smartport_use_extra_sensors: uint8_t,
}
pub type frskyUnit_e = libc::c_uint;
pub const FRSKY_UNIT_IMPERIALS: frskyUnit_e = 1;
pub const FRSKY_UNIT_METRICS: frskyUnit_e = 0;
pub type frskyGpsCoordFormat_e = libc::c_uint;
pub const FRSKY_FORMAT_NMEA: frskyGpsCoordFormat_e = 1;
pub const FRSKY_FORMAT_DMS: frskyGpsCoordFormat_e = 0;
pub type ibusTelemetry_s = ibusTelemetry;
#[derive(Copy, Clone)]
#[repr(C)]
pub union ibusTelemetry {
    pub uint16: uint16_t,
    pub uint32: uint32_t,
    pub int16: int16_t,
    pub int32: int32_t,
    pub byte: [uint8_t; 4],
}
pub type baro_t = baro_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct baro_s {
    pub dev: baroDev_t,
    pub BaroAlt: int32_t,
    pub baroTemperature: int32_t,
    pub baroPressure: int32_t,
}
// Use temperature for telemetry
// Use pressure for telemetry
// baro calculation (filled params are pressure and temperature)
pub type baroDev_t = baroDev_s;
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
// baro start operation
pub type baroCalculateFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut int32_t, _: *mut int32_t) -> ()>;
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
// XXX
pub type baroOpFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut baroDev_s) -> ()>;
pub type busDevice_t = busDevice_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct busDevice_s {
    pub bustype: busType_e,
    pub busdev_u: C2RustUnnamed_0,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_0 {
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
pub type I2CDevice = libc::c_int;
pub const I2CDEV_4: I2CDevice = 3;
pub const I2CDEV_3: I2CDevice = 2;
pub const I2CDEV_2: I2CDevice = 1;
pub const I2CDEV_1: I2CDevice = 0;
pub const I2CINVALID: I2CDevice = -1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceSpi_s {
    pub instance: *mut SPI_TypeDef,
    pub handle: *mut SPI_HandleTypeDef,
    pub csnPin: IO_t,
}
pub type busType_e = libc::c_uint;
pub const BUSTYPE_MPU_SLAVE: busType_e = 3;
pub const BUSTYPE_SPI: busType_e = 2;
pub const BUSTYPE_I2C: busType_e = 1;
pub const BUSTYPE_NONE: busType_e = 0;
pub const SENSOR_BARO: C2RustUnnamed_6 = 4;
pub const SENSOR_SONAR: C2RustUnnamed_6 = 16;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed_1 {
    pub roll: int16_t,
    pub pitch: int16_t,
    pub yaw: int16_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union attitudeEulerAngles_t {
    pub raw: [int16_t; 3],
    pub values: C2RustUnnamed_1,
}
pub const ARMED: C2RustUnnamed_3 = 1;
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
pub type accDev_t = accDev_s;
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
pub type sensor_align_e = libc::c_uint;
pub const CW270_DEG_FLIP: sensor_align_e = 8;
pub const CW180_DEG_FLIP: sensor_align_e = 7;
pub const CW90_DEG_FLIP: sensor_align_e = 6;
pub const CW0_DEG_FLIP: sensor_align_e = 5;
pub const CW270_DEG: sensor_align_e = 4;
pub const CW180_DEG: sensor_align_e = 3;
pub const CW90_DEG: sensor_align_e = 2;
pub const CW0_DEG: sensor_align_e = 1;
pub const ALIGN_DEFAULT: sensor_align_e = 0;
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
pub type acc_t = acc_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct acc_s {
    pub dev: accDev_t,
    pub accSamplingInterval: uint32_t,
    pub accADC: [libc::c_float; 3],
    pub isAccelUpdatedAtLeastOnce: bool,
}
pub const FAILSAFE_MODE: C2RustUnnamed_4 = 1024;
pub const GPS_HOLD_MODE: C2RustUnnamed_4 = 32;
pub const HORIZON_MODE: C2RustUnnamed_4 = 2;
pub const GPS_HOME_MODE: C2RustUnnamed_4 = 16;
pub const BARO_MODE: C2RustUnnamed_4 = 8;
pub const MAG_MODE: C2RustUnnamed_4 = 4;
pub const HEADFREE_MODE: C2RustUnnamed_4 = 64;
pub const PASSTHRU_MODE: C2RustUnnamed_4 = 256;
pub const ANGLE_MODE: C2RustUnnamed_4 = 1;
pub type batteryConfig_t = batteryConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct batteryConfig_s {
    pub vbatmaxcellvoltage: uint8_t,
    pub vbatmincellvoltage: uint8_t,
    pub vbatwarningcellvoltage: uint8_t,
    pub vbatnotpresentcellvoltage: uint8_t,
    pub lvcPercentage: uint8_t,
    pub voltageMeterSource: voltageMeterSource_e,
    pub currentMeterSource: currentMeterSource_e,
    pub batteryCapacity: uint16_t,
    pub useVBatAlerts: bool,
    pub useConsumptionAlerts: bool,
    pub consumptionWarningPercentage: uint8_t,
    pub vbathysteresis: uint8_t,
    pub vbatfullcellvoltage: uint8_t,
}
pub type currentMeterSource_e = libc::c_uint;
pub const CURRENT_METER_COUNT: currentMeterSource_e = 5;
pub const CURRENT_METER_MSP: currentMeterSource_e = 4;
pub const CURRENT_METER_ESC: currentMeterSource_e = 3;
pub const CURRENT_METER_VIRTUAL: currentMeterSource_e = 2;
pub const CURRENT_METER_ADC: currentMeterSource_e = 1;
pub const CURRENT_METER_NONE: currentMeterSource_e = 0;
pub type voltageMeterSource_e = libc::c_uint;
pub const VOLTAGE_METER_COUNT: voltageMeterSource_e = 3;
pub const VOLTAGE_METER_ESC: voltageMeterSource_e = 2;
pub const VOLTAGE_METER_ADC: voltageMeterSource_e = 1;
pub const VOLTAGE_METER_NONE: voltageMeterSource_e = 0;
pub const FEATURE_MOTOR_STOP: C2RustUnnamed_2 = 16;
pub const THROTTLE_LOW: throttleStatus_e = 0;
pub type throttleStatus_e = libc::c_uint;
pub const THROTTLE_HIGH: throttleStatus_e = 1;
pub const THROTTLE: rc_alias = 3;
pub type gpsSolutionData_t = gpsSolutionData_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsSolutionData_s {
    pub llh: gpsLocation_t,
    pub groundSpeed: uint16_t,
    pub groundCourse: uint16_t,
    pub hdop: uint16_t,
    pub numSat: uint8_t,
}
// initialize function
// read 3 axis data function
// a revision code for the sensor, if known
// speed in 0.1m/s
// degrees * 10
// generic HDOP value (*100)
/* LLH Location in NEU axis system */
pub type gpsLocation_t = gpsLocation_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsLocation_s {
    pub lat: int32_t,
    pub lon: int32_t,
    pub alt: int32_t,
}
pub const GPS_FIX: C2RustUnnamed_5 = 2;
pub const SENSOR_GPS: C2RustUnnamed_6 = 32;
pub const IBUS_COMMAND_MEASUREMENT: ibusCommand_e = 160;
pub type ibusCommand_e = libc::c_uint;
pub const IBUS_COMMAND_SENSOR_TYPE: ibusCommand_e = 144;
pub const IBUS_COMMAND_DISCOVER_SENSOR: ibusCommand_e = 128;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const FEATURE_DYNAMIC_FILTER: C2RustUnnamed_2 = 536870912;
pub const FEATURE_ANTI_GRAVITY: C2RustUnnamed_2 = 268435456;
pub const FEATURE_ESC_SENSOR: C2RustUnnamed_2 = 134217728;
pub const FEATURE_SOFTSPI: C2RustUnnamed_2 = 67108864;
pub const FEATURE_RX_SPI: C2RustUnnamed_2 = 33554432;
pub const FEATURE_AIRMODE: C2RustUnnamed_2 = 4194304;
pub const FEATURE_TRANSPONDER: C2RustUnnamed_2 = 2097152;
pub const FEATURE_CHANNEL_FORWARDING: C2RustUnnamed_2 = 1048576;
pub const FEATURE_OSD: C2RustUnnamed_2 = 262144;
pub const FEATURE_DASHBOARD: C2RustUnnamed_2 = 131072;
pub const FEATURE_LED_STRIP: C2RustUnnamed_2 = 65536;
pub const FEATURE_RSSI_ADC: C2RustUnnamed_2 = 32768;
pub const FEATURE_RX_MSP: C2RustUnnamed_2 = 16384;
pub const FEATURE_RX_PARALLEL_PWM: C2RustUnnamed_2 = 8192;
pub const FEATURE_3D: C2RustUnnamed_2 = 4096;
pub const FEATURE_TELEMETRY: C2RustUnnamed_2 = 1024;
pub const FEATURE_RANGEFINDER: C2RustUnnamed_2 = 512;
pub const FEATURE_GPS: C2RustUnnamed_2 = 128;
pub const FEATURE_SOFTSERIAL: C2RustUnnamed_2 = 64;
pub const FEATURE_SERVO_TILT: C2RustUnnamed_2 = 32;
pub const FEATURE_RX_SERIAL: C2RustUnnamed_2 = 8;
pub const FEATURE_INFLIGHT_ACC_CAL: C2RustUnnamed_2 = 4;
pub const FEATURE_RX_PPM: C2RustUnnamed_2 = 1;
pub type rc_alias = libc::c_uint;
pub const AUX8: rc_alias = 11;
pub const AUX7: rc_alias = 10;
pub const AUX6: rc_alias = 9;
pub const AUX5: rc_alias = 8;
pub const AUX4: rc_alias = 7;
pub const AUX3: rc_alias = 6;
pub const AUX2: rc_alias = 5;
pub const AUX1: rc_alias = 4;
pub const YAW: rc_alias = 2;
pub const PITCH: rc_alias = 1;
pub const ROLL: rc_alias = 0;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed_3 = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed_3 = 2;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const GPS_RESCUE_MODE: C2RustUnnamed_4 = 2048;
pub type C2RustUnnamed_5 = libc::c_uint;
pub const FIXED_WING: C2RustUnnamed_5 = 16;
pub const SMALL_ANGLE: C2RustUnnamed_5 = 8;
pub const CALIBRATE_MAG: C2RustUnnamed_5 = 4;
pub const GPS_FIX_HOME: C2RustUnnamed_5 = 1;
pub type C2RustUnnamed_6 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_6 = 64;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_6 = 16;
pub const SENSOR_MAG: C2RustUnnamed_6 = 8;
pub const SENSOR_ACC: C2RustUnnamed_6 = 2;
pub const SENSOR_GYRO: C2RustUnnamed_6 = 1;
#[inline]
unsafe extern "C" fn telemetryConfig() -> *const telemetryConfig_t {
    return &mut telemetryConfig_System;
}
#[inline]
unsafe extern "C" fn batteryConfig() -> *const batteryConfig_t {
    return &mut batteryConfig_System;
}
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
#[no_mangle]
pub static mut GPS_IDS: [uint8_t; 10] =
    [IBUS_SENSOR_TYPE_GPS_STATUS as libc::c_int as uint8_t,
     IBUS_SENSOR_TYPE_SPE as libc::c_int as uint8_t,
     IBUS_SENSOR_TYPE_GPS_LAT as libc::c_int as uint8_t,
     IBUS_SENSOR_TYPE_GPS_LON as libc::c_int as uint8_t,
     IBUS_SENSOR_TYPE_GPS_ALT as libc::c_int as uint8_t,
     IBUS_SENSOR_TYPE_GROUND_SPEED as libc::c_int as uint8_t,
     IBUS_SENSOR_TYPE_ODO1 as libc::c_int as uint8_t,
     IBUS_SENSOR_TYPE_ODO2 as libc::c_int as uint8_t,
     IBUS_SENSOR_TYPE_GPS_DIST as libc::c_int as uint8_t,
     IBUS_SENSOR_TYPE_COG as libc::c_int as uint8_t];
#[no_mangle]
pub static mut FULL_GPS_IDS: [uint8_t; 4] =
    [IBUS_SENSOR_TYPE_GPS_STATUS as libc::c_int as uint8_t,
     IBUS_SENSOR_TYPE_GPS_LAT as libc::c_int as uint8_t,
     IBUS_SENSOR_TYPE_GPS_LON as libc::c_int as uint8_t,
     IBUS_SENSOR_TYPE_GPS_ALT as libc::c_int as uint8_t];
#[no_mangle]
pub static mut FULL_VOLT_IDS: [uint8_t; 5] =
    [IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE as libc::c_int as uint8_t,
     IBUS_SENSOR_TYPE_CELL as libc::c_int as uint8_t,
     IBUS_SENSOR_TYPE_BAT_CURR as libc::c_int as uint8_t,
     IBUS_SENSOR_TYPE_FUEL as libc::c_int as uint8_t,
     IBUS_SENSOR_TYPE_RPM as libc::c_int as uint8_t];
#[no_mangle]
pub static mut FULL_ACC_IDS: [uint8_t; 6] =
    [IBUS_SENSOR_TYPE_ACC_X as libc::c_int as uint8_t,
     IBUS_SENSOR_TYPE_ACC_Y as libc::c_int as uint8_t,
     IBUS_SENSOR_TYPE_ACC_Z as libc::c_int as uint8_t,
     IBUS_SENSOR_TYPE_ROLL as libc::c_int as uint8_t,
     IBUS_SENSOR_TYPE_PITCH as libc::c_int as uint8_t,
     IBUS_SENSOR_TYPE_YAW as libc::c_int as uint8_t];
// latitude * 1e+7
// longitude * 1e+7
// altitude in 0.01m
//defined(USE_TELEMETRY_IBUS_EXTENDED)
static mut ibusSerialPort: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
static mut ibusBaseAddress: ibusAddress_t = 0 as libc::c_int as ibusAddress_t;
static mut sendBuffer: [uint8_t; 33] = [0; 33];
unsafe extern "C" fn getSensorID(mut address: ibusAddress_t) -> uint8_t {
    //all checks are done in theAddressIsWithinOurRange
    let mut index: uint32_t =
        (address as libc::c_int - ibusBaseAddress as libc::c_int) as
            uint32_t; // / BLADE_NUMBER_DIVIDER;
    return (*telemetryConfig()).flysky_sensors[index as usize];
}
unsafe extern "C" fn getSensorLength(mut sensorID: uint8_t) -> uint8_t {
    if sensorID as libc::c_int == IBUS_SENSOR_TYPE_PRES as libc::c_int ||
           sensorID as libc::c_int >= IBUS_SENSOR_TYPE_GPS_LAT as libc::c_int
               &&
               sensorID as libc::c_int <=
                   IBUS_SENSOR_TYPE_ALT_MAX as libc::c_int {
        return 4 as libc::c_int as uint8_t
    }
    if sensorID as libc::c_int == IBUS_SENSOR_TYPE_GPS_FULL as libc::c_int {
        return 14 as libc::c_int as uint8_t
    }
    if sensorID as libc::c_int == IBUS_SENSOR_TYPE_VOLT_FULL as libc::c_int {
        return 10 as libc::c_int as uint8_t
    }
    if sensorID as libc::c_int == IBUS_SENSOR_TYPE_VOLT_FULL as libc::c_int {
        return 12 as libc::c_int as uint8_t
    }
    return 2 as libc::c_int as uint8_t;
}
unsafe extern "C" fn transmitIbusPacket() -> uint8_t {
    let mut frameLength: libc::c_uint =
        sendBuffer[0 as libc::c_int as usize] as libc::c_uint;
    if frameLength == 0 as libc::c_int as libc::c_uint {
        return 0 as libc::c_int as uint8_t
    }
    let mut payloadLength: libc::c_uint =
        frameLength.wrapping_sub(2 as libc::c_int as libc::c_uint);
    let mut checksum: uint16_t = calculateChecksum(sendBuffer.as_mut_ptr());
    let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while i < payloadLength {
        serialWrite(ibusSerialPort, sendBuffer[i as usize]);
        i = i.wrapping_add(1)
    }
    serialWrite(ibusSerialPort,
                (checksum as libc::c_int & 0xff as libc::c_int) as uint8_t);
    serialWrite(ibusSerialPort,
                (checksum as libc::c_int >> 8 as libc::c_int) as uint8_t);
    return frameLength as uint8_t;
}
unsafe extern "C" fn setIbusDiscoverSensorReply(mut address: ibusAddress_t) {
    sendBuffer[0 as libc::c_int as usize] = 4 as libc::c_int as uint8_t;
    sendBuffer[1 as libc::c_int as usize] =
        (IBUS_COMMAND_DISCOVER_SENSOR as libc::c_int | address as libc::c_int)
            as uint8_t;
}
unsafe extern "C" fn setIbusSensorType(mut address: ibusAddress_t) {
    let mut sensorID: uint8_t = getSensorID(address);
    let mut sensorLength: uint8_t = getSensorLength(sensorID);
    sendBuffer[0 as libc::c_int as usize] =
        (4 as libc::c_int + 2 as libc::c_int) as uint8_t;
    sendBuffer[1 as libc::c_int as usize] =
        (IBUS_COMMAND_SENSOR_TYPE as libc::c_int | address as libc::c_int) as
            uint8_t;
    sendBuffer[2 as libc::c_int as usize] = sensorID;
    sendBuffer[3 as libc::c_int as usize] = sensorLength;
}
unsafe extern "C" fn getVoltage() -> uint16_t {
    let mut voltage: uint16_t =
        (getBatteryVoltage() as libc::c_int * 10 as libc::c_int) as uint16_t;
    if (*telemetryConfig()).report_cell_voltage != 0 {
        voltage =
            (voltage as libc::c_int / getBatteryCellCount() as libc::c_int) as
                uint16_t
    }
    return voltage;
}
unsafe extern "C" fn getTemperature() -> uint16_t {
    let mut temperature: uint16_t =
        (gyroGetTemperature() as libc::c_int * 10 as libc::c_int) as uint16_t;
    if sensors(SENSOR_BARO as libc::c_int as uint32_t) {
        temperature =
            ((baro.baroTemperature + 50 as libc::c_int) / 10 as libc::c_int)
                as uint16_t
    }
    return (temperature as libc::c_int + 400 as libc::c_int) as uint16_t;
}
unsafe extern "C" fn getFuel() -> uint16_t {
    let mut fuel: uint16_t = 0 as libc::c_int as uint16_t;
    if (*batteryConfig()).batteryCapacity as libc::c_int > 0 as libc::c_int {
        fuel = calculateBatteryPercentageRemaining() as uint16_t
    } else {
        fuel =
            constrain(getMAhDrawn(), 0 as libc::c_int, 0xffff as libc::c_int)
                as uint16_t
    }
    return fuel;
}
unsafe extern "C" fn getRPM() -> uint16_t {
    let mut rpm: uint16_t = 0 as libc::c_int as uint16_t;
    if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
        let throttleStatus: throttleStatus_e = calculateThrottleStatus();
        rpm = rcCommand[THROTTLE as libc::c_int as usize] as uint16_t;
        if throttleStatus as libc::c_uint ==
               THROTTLE_LOW as libc::c_int as libc::c_uint &&
               feature(FEATURE_MOTOR_STOP as libc::c_int as uint32_t) as
                   libc::c_int != 0 {
            rpm = 0 as libc::c_int as uint16_t
        }
    } else {
        rpm = (*batteryConfig()).batteryCapacity
        //  / BLADE_NUMBER_DIVIDER
    } //Acro
    return rpm;
}
unsafe extern "C" fn getMode() -> uint16_t {
    let mut flightMode: uint16_t = 1 as libc::c_int as uint16_t;
    if flightModeFlags as libc::c_int & ANGLE_MODE as libc::c_int != 0 {
        flightMode = 0 as libc::c_int as uint16_t
        //Stab
    }
    if flightModeFlags as libc::c_int & BARO_MODE as libc::c_int != 0 {
        flightMode = 2 as libc::c_int as uint16_t
        //AltHold
    }
    if flightModeFlags as libc::c_int & PASSTHRU_MODE as libc::c_int != 0 {
        flightMode = 3 as libc::c_int as uint16_t
        //Auto
    }
    if flightModeFlags as libc::c_int & HEADFREE_MODE as libc::c_int != 0 ||
           flightModeFlags as libc::c_int & MAG_MODE as libc::c_int != 0 {
        flightMode = 4 as libc::c_int as uint16_t
        //Guided! (there in no HEAD, MAG so use Guided)
    }
    if flightModeFlags as libc::c_int & GPS_HOLD_MODE as libc::c_int != 0 &&
           flightModeFlags as libc::c_int & BARO_MODE as libc::c_int != 0 {
        flightMode = 5 as libc::c_int as uint16_t
        //Loiter
    }
    if flightModeFlags as libc::c_int & GPS_HOME_MODE as libc::c_int != 0 {
        flightMode = 6 as libc::c_int as uint16_t
        //RTL
    }
    if flightModeFlags as libc::c_int & HORIZON_MODE as libc::c_int != 0 {
        flightMode = 7 as libc::c_int as uint16_t
        //Circle! (there in no horizon so use Circle)
    }
    if flightModeFlags as libc::c_int & GPS_HOLD_MODE as libc::c_int != 0 {
        flightMode = 8 as libc::c_int as uint16_t
        //PosHold
    }
    if flightModeFlags as libc::c_int & FAILSAFE_MODE as libc::c_int != 0 {
        flightMode = 9 as libc::c_int as uint16_t
        //Land
    }
    return flightMode;
}
unsafe extern "C" fn getACC(mut index: uint8_t) -> int16_t {
    return (acc.accADC[index as usize] /
                acc.dev.acc_1G as libc::c_int as libc::c_float *
                1000 as libc::c_int as libc::c_float) as int16_t;
}
unsafe extern "C" fn setCombinedFrame(mut bufferPtr: *mut uint8_t,
                                      mut structure: *const uint8_t,
                                      mut itemCount: uint8_t) {
    let mut offset: uint8_t = 0 as libc::c_int as uint8_t;
    let mut size: uint8_t = 0 as libc::c_int as uint8_t;
    let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while i < itemCount as libc::c_uint {
        size = getSensorLength(*structure.offset(i as isize));
        setValue(bufferPtr.offset(offset as libc::c_int as isize),
                 *structure.offset(i as isize), size);
        offset = (offset as libc::c_int + size as libc::c_int) as uint8_t;
        i = i.wrapping_add(1)
    };
}
unsafe extern "C" fn setGPS(mut sensorType: uint8_t,
                            mut value: *mut ibusTelemetry_s) -> bool {
    let mut result: bool = 0 as libc::c_int != 0;
    let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while (i as libc::c_ulong) <
              ::core::mem::size_of::<[uint8_t; 10]>() as libc::c_ulong {
        if sensorType as libc::c_int == GPS_IDS[i as usize] as libc::c_int {
            result = 1 as libc::c_int != 0;
            break ;
        } else { i = i.wrapping_add(1) }
    }
    if !result { return result }
    let mut gpsFixType: uint16_t = 0 as libc::c_int as uint16_t;
    let mut sats: uint16_t = 0 as libc::c_int as uint16_t;
    if sensors(SENSOR_GPS as libc::c_int as uint32_t) {
        gpsFixType =
            if stateFlags as libc::c_int & GPS_FIX as libc::c_int == 0 {
                1 as libc::c_int
            } else if (gpsSol.numSat as libc::c_int) < 5 as libc::c_int {
                2 as libc::c_int
            } else { 3 as libc::c_int } as uint16_t;
        sats = gpsSol.numSat as uint16_t;
        if stateFlags as libc::c_int & GPS_FIX as libc::c_int != 0 ||
               sensorType as libc::c_int ==
                   IBUS_SENSOR_TYPE_GPS_STATUS as libc::c_int {
            result = 1 as libc::c_int != 0;
            match sensorType as libc::c_int {
                126 => {
                    (*value).uint16 =
                        (gpsSol.groundSpeed as libc::c_int * 36 as libc::c_int
                             / 100 as libc::c_int) as uint16_t
                }
                128 => { (*value).int32 = gpsSol.llh.lat }
                129 => { (*value).int32 = gpsSol.llh.lon }
                130 => { (*value).int32 = gpsSol.llh.alt }
                19 => { (*value).uint16 = gpsSol.groundSpeed }
                124 | 125 | 20 => { (*value).uint16 = GPS_distanceToHome }
                10 => {
                    (*value).uint16 =
                        (gpsSol.groundCourse as libc::c_int *
                             100 as libc::c_int) as uint16_t
                }
                11 => {
                    (*value).byte[0 as libc::c_int as usize] =
                        gpsFixType as uint8_t;
                    (*value).byte[1 as libc::c_int as usize] = sats as uint8_t
                }
                _ => { }
            }
        }
    }
    return result;
}
//defined(USE_GPS)
unsafe extern "C" fn setValue(mut bufferPtr: *mut uint8_t,
                              mut sensorType: uint8_t, mut length: uint8_t) {
    let mut value: ibusTelemetry_s = ibusTelemetry{uint16: 0,};
    let mut structure: *const uint8_t = 0 as *const uint8_t;
    let mut itemCount: uint8_t = 0;
    if sensorType as libc::c_int == IBUS_SENSOR_TYPE_GPS_FULL as libc::c_int {
        structure = FULL_GPS_IDS.as_ptr();
        itemCount =
            ::core::mem::size_of::<[uint8_t; 4]>() as libc::c_ulong as uint8_t
    }
    if sensorType as libc::c_int == IBUS_SENSOR_TYPE_VOLT_FULL as libc::c_int
       {
        structure = FULL_VOLT_IDS.as_ptr();
        itemCount =
            ::core::mem::size_of::<[uint8_t; 5]>() as libc::c_ulong as uint8_t
    }
    if sensorType as libc::c_int == IBUS_SENSOR_TYPE_ACC_FULL as libc::c_int {
        structure = FULL_ACC_IDS.as_ptr();
        itemCount =
            ::core::mem::size_of::<[uint8_t; 6]>() as libc::c_ulong as uint8_t
    }
    if !structure.is_null() {
        setCombinedFrame(bufferPtr, structure,
                         ::core::mem::size_of::<uint8_t>() as libc::c_ulong as
                             uint8_t);
        return
    }
    //defined(USE_TELEMETRY_IBUS_EXTENDED)
    if setGPS(sensorType, &mut value) { return }
    //defined(USE_TELEMETRY_IBUS_EXTENDED)
    let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while i < length as libc::c_uint {
        value.byte[i as usize] = 0 as libc::c_int as uint8_t;
        *bufferPtr.offset(i as isize) = value.byte[i as usize];
        i = i.wrapping_add(1)
    }
    match sensorType as libc::c_int {
        3 => {
            value.uint16 = getVoltage()
            //defined(TELEMETRY_IBUS_EXTENDED)
        }
        1 => { value.uint16 = getTemperature() }
        2 => {
            value.int16 =
                rcCommand[THROTTLE as libc::c_int as usize] as int16_t
        }
        6 => { value.uint16 = getFuel() }
        7 => { value.uint16 = getRPM() }
        22 => { value.uint16 = getMode() }
        4 => {
            value.uint16 =
                (getBatteryAverageCellVoltage() as libc::c_int *
                     10 as libc::c_int) as uint16_t
        }
        5 => { value.uint16 = getAmperage() as uint16_t }
        12 | 13 | 14 => {
            value.int16 =
                getACC((sensorType as libc::c_int -
                            IBUS_SENSOR_TYPE_ACC_X as libc::c_int) as uint8_t)
        }
        15 | 16 | 17 => {
            value.int16 =
                (attitude.raw[(sensorType as libc::c_int -
                                   IBUS_SENSOR_TYPE_ROLL as libc::c_int) as
                                  usize] as libc::c_int * 10 as libc::c_int)
                    as int16_t
        }
        21 => {
            value.uint16 =
                if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
                    1 as libc::c_int
                } else { 0 as libc::c_int } as uint16_t
        }
        8 => {
            value.uint16 =
                (attitude.values.yaw as libc::c_int / 10 as libc::c_int) as
                    uint16_t
        }
        18 | 9 => {
            if sensors(SENSOR_SONAR as libc::c_int as uint32_t) as libc::c_int
                   != 0 ||
                   sensors(SENSOR_BARO as libc::c_int as uint32_t) as
                       libc::c_int != 0 {
                value.int16 = getEstimatedVario()
            }
        }
        131 | 132 => { value.int32 = baro.BaroAlt }
        65 => {
            value.uint32 =
                baro.baroPressure as libc::c_uint |
                    (getTemperature() as uint32_t) << 19 as libc::c_int
        }
        _ => { }
    }
    let mut i_0: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while i_0 < length as libc::c_uint {
        *bufferPtr.offset(i_0 as isize) = value.byte[i_0 as usize];
        i_0 = i_0.wrapping_add(1)
    };
}
unsafe extern "C" fn setIbusMeasurement(mut address: ibusAddress_t) {
    let mut sensorID: uint8_t = getSensorID(address);
    let mut sensorLength: uint8_t = getSensorLength(sensorID);
    sendBuffer[0 as libc::c_int as usize] =
        (4 as libc::c_int + sensorLength as libc::c_int) as uint8_t;
    sendBuffer[1 as libc::c_int as usize] =
        (IBUS_COMMAND_MEASUREMENT as libc::c_int | address as libc::c_int) as
            uint8_t;
    setValue(sendBuffer.as_mut_ptr().offset(2 as libc::c_int as isize),
             sensorID, sensorLength);
}
unsafe extern "C" fn isCommand(mut expected: ibusCommand_e,
                               mut ibusPacket: *const uint8_t) -> bool {
    return (*ibusPacket.offset(1 as libc::c_int as isize) as libc::c_int &
                0xf0 as libc::c_int) as libc::c_uint ==
               expected as libc::c_uint;
}
unsafe extern "C" fn getAddress(mut ibusPacket: *const uint8_t)
 -> ibusAddress_t {
    return (*ibusPacket.offset(1 as libc::c_int as isize) as libc::c_int &
                0xf as libc::c_int) as ibusAddress_t;
}
unsafe extern "C" fn autodetectFirstReceivedAddressAsBaseAddress(mut returnAddress:
                                                                     ibusAddress_t) {
    if 0 as libc::c_int == ibusBaseAddress as libc::c_int &&
           0 as libc::c_int != returnAddress as libc::c_int {
        ibusBaseAddress = returnAddress
    };
}
unsafe extern "C" fn theAddressIsWithinOurRange(mut returnAddress:
                                                    ibusAddress_t) -> bool {
    return returnAddress as libc::c_int >= ibusBaseAddress as libc::c_int &&
               ((returnAddress as libc::c_int -
                     ibusBaseAddress as libc::c_int) as ibusAddress_t as
                    libc::c_ulong) <
                   (::core::mem::size_of::<[uint8_t; 15]>() as
                        libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>()
                                                        as libc::c_ulong) &&
               (*telemetryConfig()).flysky_sensors[(returnAddress as
                                                        libc::c_int -
                                                        ibusBaseAddress as
                                                            libc::c_int) as
                                                       usize] as libc::c_int
                   != IBUS_SENSOR_TYPE_NONE as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn respondToIbusRequest(ibusPacket: *const uint8_t)
 -> uint8_t {
    let mut returnAddress: ibusAddress_t = getAddress(ibusPacket);
    autodetectFirstReceivedAddressAsBaseAddress(returnAddress);
    //set buffer to invalid
    sendBuffer[0 as libc::c_int as usize] = 0 as libc::c_int as uint8_t;
    if theAddressIsWithinOurRange(returnAddress) {
        if isCommand(IBUS_COMMAND_DISCOVER_SENSOR, ibusPacket) {
            setIbusDiscoverSensorReply(returnAddress);
        } else if isCommand(IBUS_COMMAND_SENSOR_TYPE, ibusPacket) {
            setIbusSensorType(returnAddress);
        } else if isCommand(IBUS_COMMAND_MEASUREMENT, ibusPacket) {
            setIbusMeasurement(returnAddress);
        }
    }
    //transmit if content was set
    return transmitIbusPacket();
}
#[no_mangle]
pub unsafe extern "C" fn initSharedIbusTelemetry(mut port:
                                                     *mut serialPort_t) {
    ibusSerialPort = port;
    ibusBaseAddress = 0 as libc::c_int as ibusAddress_t;
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
 * FlySky iBus telemetry implementation by CraigJPerry.
 * Unit tests and some additions by Unitware
 *
 * Many thanks to Dave Borthwick's iBus telemetry dongle converter for
 * PIC 12F1572 (also distributed under GPLv3) which was referenced to
 * clarify the protocol.
 */
// #include <string.h>
//#include "common/utils.h"
//defined(USE_TELEMETRY) && defined(USE_TELEMETRY_IBUS)
unsafe extern "C" fn calculateChecksum(mut ibusPacket: *const uint8_t)
 -> uint16_t {
    let mut checksum: uint16_t = 0xffff as libc::c_int as uint16_t;
    let mut dataSize: uint8_t =
        (*ibusPacket.offset(0 as libc::c_int as isize) as libc::c_int -
             2 as libc::c_int) as uint8_t;
    let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while i < dataSize as libc::c_uint {
        checksum =
            (checksum as libc::c_int -
                 *ibusPacket.offset(i as isize) as libc::c_int) as uint16_t;
        i = i.wrapping_add(1)
    }
    return checksum;
}
//defined(TELEMETRY) && defined(TELEMETRY_IBUS)
#[no_mangle]
pub unsafe extern "C" fn isChecksumOkIa6b(mut ibusPacket: *const uint8_t,
                                          length: uint8_t) -> bool {
    let mut calculatedChecksum: uint16_t = calculateChecksum(ibusPacket);
    // Note that there's a byte order swap to little endian here
    return calculatedChecksum as libc::c_int >> 8 as libc::c_int ==
               *ibusPacket.offset((length as libc::c_int - 1 as libc::c_int)
                                      as isize) as libc::c_int &&
               calculatedChecksum as libc::c_int & 0xff as libc::c_int ==
                   *ibusPacket.offset((length as libc::c_int -
                                           2 as libc::c_int) as isize) as
                       libc::c_int;
}
