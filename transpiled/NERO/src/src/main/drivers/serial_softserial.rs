use ::libc;
extern "C" {
    #[no_mangle]
    fn TIM_CCxChannelCmd(TIMx: *mut TIM_TypeDef, Channel: uint32_t,
                         ChannelState: uint32_t);
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
    // preprocessor is used to convert pinid to requested C data value
// compile-time error is generated if requested pin is not available (not set in TARGET_IO_PORTx)
// ioTag_t and IO_t is supported, but ioTag_t is preferred
    // expand pinid to to ioTag_t
    //speed is packed inside modebits 5 and 2,
    // declare available IO pins. Available pins are specified per target
    // unimplemented
    #[no_mangle]
    fn IOConfigGPIOAF(io: IO_t, cfg: ioConfig_t, af: uint8_t);
    #[no_mangle]
    fn IOHi(io: IO_t);
    #[no_mangle]
    fn IOLo(io: IO_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn timerConfigure(timHw: *const timerHardware_t, period: uint16_t,
                      hz: uint32_t);
    #[no_mangle]
    fn timerChConfigIC(timHw: *const timerHardware_t, polarityRising: bool,
                       inputFilterSamples: libc::c_uint);
    #[no_mangle]
    fn timerChCCHandlerInit(self_0: *mut timerCCHandlerRec_t,
                            fn_0: Option<timerCCHandlerCallback>);
    #[no_mangle]
    fn timerChOvrHandlerInit(self_0: *mut timerOvrHandlerRec_t,
                             fn_0: Option<timerOvrHandlerCallback>);
    #[no_mangle]
    fn timerChConfigCallbacks(channel: *const timerHardware_t,
                              edgeCallback: *mut timerCCHandlerRec_t,
                              overflowCallback: *mut timerOvrHandlerRec_t);
    #[no_mangle]
    fn timerClock(tim: *mut TIM_TypeDef) -> uint32_t;
    #[no_mangle]
    fn timerFindTimerHandle(tim: *mut TIM_TypeDef) -> *mut TIM_HandleTypeDef;
    #[no_mangle]
    fn timerGetByTag(ioTag: ioTag_t) -> *const timerHardware_t;
    #[no_mangle]
    static mut serialPinConfig_System: serialPinConfig_t;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
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
  * @brief TIM
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub SMCR: uint32_t,
    pub DIER: uint32_t,
    pub SR: uint32_t,
    pub EGR: uint32_t,
    pub CCMR1: uint32_t,
    pub CCMR2: uint32_t,
    pub CCER: uint32_t,
    pub CNT: uint32_t,
    pub PSC: uint32_t,
    pub ARR: uint32_t,
    pub RCR: uint32_t,
    pub CCR1: uint32_t,
    pub CCR2: uint32_t,
    pub CCR3: uint32_t,
    pub CCR4: uint32_t,
    pub BDTR: uint32_t,
    pub DCR: uint32_t,
    pub DMAR: uint32_t,
    pub OR: uint32_t,
    pub CCMR3: uint32_t,
    pub CCR5: uint32_t,
    pub CCR6: uint32_t,
}
pub type C2RustUnnamed = libc::c_uint;
pub const ENABLE: C2RustUnnamed = 1;
pub const DISABLE: C2RustUnnamed = 0;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_Base_InitTypeDef {
    pub Prescaler: uint32_t,
    pub CounterMode: uint32_t,
    pub Period: uint32_t,
    pub ClockDivision: uint32_t,
    pub RepetitionCounter: uint32_t,
    pub AutoReloadPreload: uint32_t,
}
pub type HAL_TIM_StateTypeDef = libc::c_uint;
pub const HAL_TIM_STATE_ERROR: HAL_TIM_StateTypeDef = 4;
pub const HAL_TIM_STATE_TIMEOUT: HAL_TIM_StateTypeDef = 3;
pub const HAL_TIM_STATE_BUSY: HAL_TIM_StateTypeDef = 2;
pub const HAL_TIM_STATE_READY: HAL_TIM_StateTypeDef = 1;
pub const HAL_TIM_STATE_RESET: HAL_TIM_StateTypeDef = 0;
pub type HAL_TIM_ActiveChannel = libc::c_uint;
pub const HAL_TIM_ACTIVE_CHANNEL_CLEARED: HAL_TIM_ActiveChannel = 0;
pub const HAL_TIM_ACTIVE_CHANNEL_4: HAL_TIM_ActiveChannel = 8;
pub const HAL_TIM_ACTIVE_CHANNEL_3: HAL_TIM_ActiveChannel = 4;
pub const HAL_TIM_ACTIVE_CHANNEL_2: HAL_TIM_ActiveChannel = 2;
pub const HAL_TIM_ACTIVE_CHANNEL_1: HAL_TIM_ActiveChannel = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_HandleTypeDef {
    pub Instance: *mut TIM_TypeDef,
    pub Init: TIM_Base_InitTypeDef,
    pub Channel: HAL_TIM_ActiveChannel,
    pub hdma: [*mut DMA_HandleTypeDef; 7],
    pub Lock: HAL_LockTypeDef,
    pub State: HAL_TIM_StateTypeDef,
}
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
/* !< DMA Stream Index                       */
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
pub type captureCompare_t = uint16_t;
pub type timerUsageFlag_e = libc::c_uint;
pub const TIM_USE_BEEPER: timerUsageFlag_e = 64;
pub const TIM_USE_TRANSPONDER: timerUsageFlag_e = 32;
pub const TIM_USE_LED: timerUsageFlag_e = 16;
pub const TIM_USE_SERVO: timerUsageFlag_e = 8;
pub const TIM_USE_MOTOR: timerUsageFlag_e = 4;
pub const TIM_USE_PWM: timerUsageFlag_e = 2;
pub const TIM_USE_PPM: timerUsageFlag_e = 1;
pub const TIM_USE_NONE: timerUsageFlag_e = 0;
pub const TIM_USE_ANY: timerUsageFlag_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerCCHandlerRec_s {
    pub fn_0: Option<timerCCHandlerCallback>,
}
pub type timerCCHandlerCallback
    =
    unsafe extern "C" fn(_: *mut timerCCHandlerRec_s, _: uint16_t) -> ();
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerOvrHandlerRec_s {
    pub fn_0: Option<timerOvrHandlerCallback>,
    pub next: *mut timerOvrHandlerRec_s,
}
pub type timerOvrHandlerCallback
    =
    unsafe extern "C" fn(_: *mut timerOvrHandlerRec_s, _: uint16_t) -> ();
pub type timerCCHandlerRec_t = timerCCHandlerRec_s;
pub type timerOvrHandlerRec_t = timerOvrHandlerRec_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerHardware_s {
    pub tim: *mut TIM_TypeDef,
    pub tag: ioTag_t,
    pub channel: uint8_t,
    pub usageFlags: timerUsageFlag_e,
    pub output: uint8_t,
    pub alternateFunction: uint8_t,
    pub dmaRef: *mut DMA_Stream_TypeDef,
    pub dmaChannel: uint32_t,
    pub dmaIrqHandler: uint8_t,
    pub dmaTimUPRef: *mut DMA_Stream_TypeDef,
    pub dmaTimUPChannel: uint32_t,
    pub dmaTimUPIrqHandler: uint8_t,
}
pub type timerHardware_t = timerHardware_s;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const TIMER_OUTPUT_N_CHANNEL: C2RustUnnamed_0 = 2;
pub const TIMER_OUTPUT_INVERTED: C2RustUnnamed_0 = 1;
pub const TIMER_OUTPUT_NONE: C2RustUnnamed_0 = 0;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPinConfig_s {
    pub ioTagTx: [ioTag_t; 12],
    pub ioTagRx: [ioTag_t; 12],
    pub ioTagInverter: [ioTag_t; 12],
}
pub type serialPinConfig_t = serialPinConfig_s;
pub type softSerialPortIndex_e = libc::c_uint;
pub const SOFTSERIAL2: softSerialPortIndex_e = 1;
pub const SOFTSERIAL1: softSerialPortIndex_e = 0;
pub type softSerial_t = softSerial_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct softSerial_s {
    pub port: serialPort_t,
    pub rxIO: IO_t,
    pub txIO: IO_t,
    pub timerHardware: *const timerHardware_t,
    pub timerHandle: *const TIM_HandleTypeDef,
    pub exTimerHardware: *const timerHardware_t,
    pub rxBuffer: [uint8_t; 256],
    pub txBuffer: [uint8_t; 256],
    pub isSearchingForStartBit: uint8_t,
    pub rxBitIndex: uint8_t,
    pub rxLastLeadingEdgeAtBitIndex: uint8_t,
    pub rxEdge: uint8_t,
    pub rxActive: uint8_t,
    pub isTransmittingData: uint8_t,
    pub bitsLeftToTransmit: int8_t,
    pub internalTxBuffer: uint16_t,
    pub internalRxBuffer: uint16_t,
    pub transmissionErrors: uint16_t,
    pub receiveErrors: uint16_t,
    pub softSerialPortIndex: uint8_t,
    pub timerMode: timerMode_e,
    pub overCb: timerOvrHandlerRec_t,
    pub edgeCb: timerCCHandlerRec_t,
}
pub type timerMode_e = libc::c_uint;
pub const TIMER_MODE_DUAL: timerMode_e = 1;
pub const TIMER_MODE_SINGLE: timerMode_e = 0;
pub const TRAILING: C2RustUnnamed_1 = 0;
pub const LEADING: C2RustUnnamed_1 = 1;
pub type C2RustUnnamed_1 = libc::c_uint;
#[inline]
unsafe extern "C" fn serialPinConfig() -> *const serialPinConfig_t {
    return &mut serialPinConfig_System;
}
// includes start and stop bits
// includes start and stop bits
// Forward
static mut softSerialPorts: [softSerial_t; 2] =
    [softSerial_t{port:
                      serialPort_t{vTable: 0 as *const serialPortVTable,
                                   mode: 0 as portMode_e,
                                   options: SERIAL_NOT_INVERTED,
                                   baudRate: 0,
                                   rxBufferSize: 0,
                                   txBufferSize: 0,
                                   rxBuffer:
                                       0 as *const uint8_t as *mut uint8_t,
                                   txBuffer:
                                       0 as *const uint8_t as *mut uint8_t,
                                   rxBufferHead: 0,
                                   rxBufferTail: 0,
                                   txBufferHead: 0,
                                   txBufferTail: 0,
                                   rxCallback: None,
                                   rxCallbackData:
                                       0 as *const libc::c_void as
                                           *mut libc::c_void,
                                   identifier: 0,},
                  rxIO: 0 as *const libc::c_void as *mut libc::c_void,
                  txIO: 0 as *const libc::c_void as *mut libc::c_void,
                  timerHardware: 0 as *const timerHardware_t,
                  timerHandle: 0 as *const TIM_HandleTypeDef,
                  exTimerHardware: 0 as *const timerHardware_t,
                  rxBuffer: [0; 256],
                  txBuffer: [0; 256],
                  isSearchingForStartBit: 0,
                  rxBitIndex: 0,
                  rxLastLeadingEdgeAtBitIndex: 0,
                  rxEdge: 0,
                  rxActive: 0,
                  isTransmittingData: 0,
                  bitsLeftToTransmit: 0,
                  internalTxBuffer: 0,
                  internalRxBuffer: 0,
                  transmissionErrors: 0,
                  receiveErrors: 0,
                  softSerialPortIndex: 0,
                  timerMode: TIMER_MODE_SINGLE,
                  overCb:
                      timerOvrHandlerRec_t{fn_0: None,
                                           next:
                                               0 as
                                                   *const timerOvrHandlerRec_s
                                                   as
                                                   *mut timerOvrHandlerRec_s,},
                  edgeCb: timerCCHandlerRec_t{fn_0: None,},}; 2];
unsafe extern "C" fn setTxSignal(mut softSerial: *mut softSerial_t,
                                 mut state: uint8_t) {
    if (*softSerial).port.options as libc::c_uint &
           SERIAL_INVERTED as libc::c_int as libc::c_uint != 0 {
        state = (state == 0) as libc::c_int as uint8_t
    }
    if state != 0 {
        IOHi((*softSerial).txIO);
    } else { IOLo((*softSerial).txIO); };
}
unsafe extern "C" fn serialEnableCC(mut softSerial: *mut softSerial_t) {
    TIM_CCxChannelCmd((*(*softSerial).timerHardware).tim,
                      (*(*softSerial).timerHardware).channel as uint32_t,
                      0x1 as libc::c_uint);
}
unsafe extern "C" fn serialInputPortActivate(mut softSerial:
                                                 *mut softSerial_t) {
    if (*softSerial).port.options as libc::c_uint &
           SERIAL_INVERTED as libc::c_int as libc::c_uint != 0 {
        let pinConfig: uint8_t =
            if (*softSerial).port.options as libc::c_uint &
                   SERIAL_BIDIR_NOPULL as libc::c_int as libc::c_uint != 0 {
                (0x2 as libc::c_uint |
                     (0 as libc::c_uint) << 2 as libc::c_int) |
                    (0 as libc::c_uint) << 5 as libc::c_int
            } else {
                (0x2 as libc::c_uint |
                     (0 as libc::c_uint) << 2 as libc::c_int) |
                    (0x2 as libc::c_uint) << 5 as libc::c_int
            } as uint8_t;
        IOConfigGPIOAF((*softSerial).rxIO, pinConfig,
                       (*(*softSerial).timerHardware).alternateFunction);
    } else {
        let pinConfig_0: uint8_t =
            if (*softSerial).port.options as libc::c_uint &
                   SERIAL_BIDIR_NOPULL as libc::c_int as libc::c_uint != 0 {
                (0x2 as libc::c_uint |
                     (0 as libc::c_uint) << 2 as libc::c_int) |
                    (0 as libc::c_uint) << 5 as libc::c_int
            } else {
                (0x2 as libc::c_uint |
                     (0 as libc::c_uint) << 2 as libc::c_int) |
                    (0x1 as libc::c_uint) << 5 as libc::c_int
            } as uint8_t;
        IOConfigGPIOAF((*softSerial).rxIO, pinConfig_0,
                       (*(*softSerial).timerHardware).alternateFunction);
    }
    (*softSerial).rxActive = 1 as libc::c_int as uint8_t;
    (*softSerial).isSearchingForStartBit = 1 as libc::c_int as uint8_t;
    (*softSerial).rxBitIndex = 0 as libc::c_int as uint8_t;
    // Enable input capture
    serialEnableCC(softSerial);
}
unsafe extern "C" fn serialInputPortDeActivate(mut softSerial:
                                                   *mut softSerial_t) {
    // Disable input capture
    TIM_CCxChannelCmd((*(*softSerial).timerHardware).tim,
                      (*(*softSerial).timerHardware).channel as uint32_t,
                      0 as libc::c_uint);
    IOConfigGPIO((*softSerial).rxIO,
                 (0 as libc::c_uint | (0 as libc::c_uint) << 2 as libc::c_int
                      | (0 as libc::c_uint) << 5 as libc::c_int) as
                     ioConfig_t);
    (*softSerial).rxActive = 0 as libc::c_int as uint8_t;
}
unsafe extern "C" fn serialOutputPortActivate(mut softSerial:
                                                  *mut softSerial_t) {
    if !(*softSerial).exTimerHardware.is_null() {
        IOConfigGPIOAF((*softSerial).txIO,
                       (0x1 as libc::c_uint |
                            (0 as libc::c_uint) << 2 as libc::c_int |
                            (0 as libc::c_uint) << 5 as libc::c_int) as
                           ioConfig_t,
                       (*(*softSerial).exTimerHardware).alternateFunction);
    } else {
        IOConfigGPIO((*softSerial).txIO,
                     (0x1 as libc::c_uint |
                          (0 as libc::c_uint) << 2 as libc::c_int |
                          (0 as libc::c_uint) << 5 as libc::c_int) as
                         ioConfig_t);
    };
}
unsafe extern "C" fn serialOutputPortDeActivate(mut softSerial:
                                                    *mut softSerial_t) {
    if !(*softSerial).exTimerHardware.is_null() {
        IOConfigGPIOAF((*softSerial).txIO,
                       (0 as libc::c_uint |
                            (0 as libc::c_uint) << 2 as libc::c_int |
                            (0 as libc::c_uint) << 5 as libc::c_int) as
                           ioConfig_t,
                       (*(*softSerial).exTimerHardware).alternateFunction);
    } else {
        IOConfigGPIO((*softSerial).txIO,
                     (0 as libc::c_uint |
                          (0 as libc::c_uint) << 2 as libc::c_int |
                          (0 as libc::c_uint) << 5 as libc::c_int) as
                         ioConfig_t);
    };
}
unsafe extern "C" fn isTimerPeriodTooLarge(mut timerPeriod: uint32_t)
 -> bool {
    return timerPeriod > 0xffff as libc::c_int as libc::c_uint;
}
unsafe extern "C" fn serialTimerConfigureTimebase(mut timerHardwarePtr:
                                                      *const timerHardware_t,
                                                  mut baud: uint32_t) {
    let mut baseClock: uint32_t = timerClock((*timerHardwarePtr).tim);
    let mut clock: uint32_t = baseClock;
    let mut timerPeriod: uint32_t = 0;
    loop  {
        timerPeriod = clock.wrapping_div(baud);
        if isTimerPeriodTooLarge(timerPeriod) {
            if clock > 1 as libc::c_int as libc::c_uint {
                clock = clock.wrapping_div(2 as libc::c_int as libc::c_uint)
                // this is wrong - mhz stays the same ... This will double baudrate until ok (but minimum baudrate is < 1200)
            }
        }
        if !isTimerPeriodTooLarge(timerPeriod) { break ; }
    }
    timerConfigure(timerHardwarePtr, timerPeriod as uint16_t, baseClock);
}
unsafe extern "C" fn resetBuffers(mut softSerial: *mut softSerial_t) {
    (*softSerial).port.rxBufferSize = 256 as libc::c_int as uint32_t;
    (*softSerial).port.rxBuffer = (*softSerial).rxBuffer.as_mut_ptr();
    (*softSerial).port.rxBufferTail = 0 as libc::c_int as uint32_t;
    (*softSerial).port.rxBufferHead = 0 as libc::c_int as uint32_t;
    (*softSerial).port.txBuffer = (*softSerial).txBuffer.as_mut_ptr();
    (*softSerial).port.txBufferSize = 256 as libc::c_int as uint32_t;
    (*softSerial).port.txBufferTail = 0 as libc::c_int as uint32_t;
    (*softSerial).port.txBufferHead = 0 as libc::c_int as uint32_t;
}
#[no_mangle]
pub unsafe extern "C" fn openSoftSerial(mut portIndex: softSerialPortIndex_e,
                                        mut rxCallback:
                                            serialReceiveCallbackPtr,
                                        mut rxCallbackData: *mut libc::c_void,
                                        mut baud: uint32_t,
                                        mut mode: portMode_e,
                                        mut options: portOptions_e)
 -> *mut serialPort_t {
    let mut softSerial: *mut softSerial_t =
        &mut *softSerialPorts.as_mut_ptr().offset(portIndex as isize) as
            *mut softSerial_t;
    let mut pinCfgIndex: libc::c_int =
        (portIndex as
             libc::c_uint).wrapping_add(10 as libc::c_int as libc::c_uint) as
            libc::c_int;
    let mut tagRx: ioTag_t =
        (*serialPinConfig()).ioTagRx[pinCfgIndex as usize];
    let mut tagTx: ioTag_t =
        (*serialPinConfig()).ioTagTx[pinCfgIndex as usize];
    let mut timerRx: *const timerHardware_t = timerGetByTag(tagRx);
    let mut timerTx: *const timerHardware_t = timerGetByTag(tagTx);
    let mut rxIO: IO_t = IOGetByTag(tagRx);
    let mut txIO: IO_t = IOGetByTag(tagTx);
    if options as libc::c_uint & SERIAL_BIDIR as libc::c_int as libc::c_uint
           != 0 {
        // If RX and TX pins are both assigned, we CAN use either with a timer.
        // However, for consistency with hardware UARTs, we only use TX pin,
        // and this pin must have a timer, and it should not be N-Channel.
        if timerTx.is_null() ||
               (*timerTx).output as libc::c_int &
                   TIMER_OUTPUT_N_CHANNEL as libc::c_int != 0 {
            return 0 as *mut serialPort_t
        }
        (*softSerial).timerHardware = timerTx;
        (*softSerial).txIO = txIO;
        (*softSerial).rxIO = txIO;
        IOInit(txIO, OWNER_SERIAL_TX,
               (portIndex as
                    libc::c_uint).wrapping_add(10 as libc::c_int as
                                                   libc::c_uint).wrapping_add(1
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                   as uint8_t);
    } else {
        if mode as libc::c_uint & MODE_RX as libc::c_int as libc::c_uint != 0
           {
            // Need a pin & a timer on RX. Channel should not be N-Channel.
            if timerRx.is_null() ||
                   (*timerRx).output as libc::c_int &
                       TIMER_OUTPUT_N_CHANNEL as libc::c_int != 0 {
                return 0 as *mut serialPort_t
            }
            (*softSerial).rxIO = rxIO;
            (*softSerial).timerHardware = timerRx;
            IOInit(rxIO, OWNER_SERIAL_RX,
                   (portIndex as
                        libc::c_uint).wrapping_add(10 as libc::c_int as
                                                       libc::c_uint).wrapping_add(1
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                       as uint8_t);
        }
        if mode as libc::c_uint & MODE_TX as libc::c_int as libc::c_uint != 0
           {
            // Need a pin on TX
            if tagTx == 0 { return 0 as *mut serialPort_t }
            (*softSerial).txIO = txIO;
            if mode as libc::c_uint & MODE_RX as libc::c_int as libc::c_uint
                   == 0 {
                // TX Simplex, must have a timer
                if timerTx.is_null() { return 0 as *mut serialPort_t }
                (*softSerial).timerHardware = timerTx
            } else {
                // Duplex
                (*softSerial).exTimerHardware = timerTx
            }
            IOInit(txIO, OWNER_SERIAL_TX,
                   (portIndex as
                        libc::c_uint).wrapping_add(10 as libc::c_int as
                                                       libc::c_uint).wrapping_add(1
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                       as uint8_t);
        }
    }
    (*softSerial).port.vTable = &softSerialVTable;
    (*softSerial).port.baudRate = baud;
    (*softSerial).port.mode = mode;
    (*softSerial).port.options = options;
    (*softSerial).port.rxCallback = rxCallback;
    (*softSerial).port.rxCallbackData = rxCallbackData;
    resetBuffers(softSerial);
    (*softSerial).softSerialPortIndex = portIndex as uint8_t;
    (*softSerial).transmissionErrors = 0 as libc::c_int as uint16_t;
    (*softSerial).receiveErrors = 0 as libc::c_int as uint16_t;
    (*softSerial).rxActive = 0 as libc::c_int as uint8_t;
    (*softSerial).isTransmittingData = 0 as libc::c_int as uint8_t;
    // Configure master timer (on RX); time base and input capture
    serialTimerConfigureTimebase((*softSerial).timerHardware, baud);
    timerChConfigIC((*softSerial).timerHardware,
                    if options as libc::c_uint &
                           SERIAL_INVERTED as libc::c_int as libc::c_uint != 0
                       {
                        1 as libc::c_int
                    } else { 0 as libc::c_int } != 0,
                    0 as libc::c_int as libc::c_uint);
    // Initialize callbacks
    timerChCCHandlerInit(&mut (*softSerial).edgeCb,
                         Some(onSerialRxPinChange as
                                  unsafe extern "C" fn(_:
                                                           *mut timerCCHandlerRec_t,
                                                       _: captureCompare_t)
                                      -> ()));
    timerChOvrHandlerInit(&mut (*softSerial).overCb,
                          Some(onSerialTimerOverflow as
                                   unsafe extern "C" fn(_:
                                                            *mut timerOvrHandlerRec_t,
                                                        _: captureCompare_t)
                                       -> ()));
    // Configure bit clock interrupt & handler.
    // If we have an extra timer (on TX), it is initialized and configured
    // for overflow interrupt.
    // Receiver input capture is configured when input is activated.
    if mode as libc::c_uint & MODE_TX as libc::c_int as libc::c_uint != 0 &&
           !(*softSerial).exTimerHardware.is_null() &&
           (*(*softSerial).exTimerHardware).tim !=
               (*(*softSerial).timerHardware).tim {
        (*softSerial).timerMode = TIMER_MODE_DUAL;
        serialTimerConfigureTimebase((*softSerial).exTimerHardware, baud);
        timerChConfigCallbacks((*softSerial).exTimerHardware,
                               0 as *mut timerCCHandlerRec_t,
                               &mut (*softSerial).overCb);
        timerChConfigCallbacks((*softSerial).timerHardware,
                               &mut (*softSerial).edgeCb,
                               0 as *mut timerOvrHandlerRec_t);
    } else {
        (*softSerial).timerMode = TIMER_MODE_SINGLE;
        timerChConfigCallbacks((*softSerial).timerHardware,
                               &mut (*softSerial).edgeCb,
                               &mut (*softSerial).overCb);
    }
    (*softSerial).timerHandle =
        timerFindTimerHandle((*(*softSerial).timerHardware).tim);
    if options as libc::c_uint & SERIAL_BIDIR as libc::c_int as libc::c_uint
           == 0 {
        serialOutputPortActivate(softSerial);
        setTxSignal(softSerial, ENABLE as libc::c_int as uint8_t);
    }
    serialInputPortActivate(softSerial);
    return &mut (*softSerial).port;
}
/*
 * Serial Engine
 */
#[no_mangle]
pub unsafe extern "C" fn processTxState(mut softSerial: *mut softSerial_t) {
    let mut mask: uint8_t = 0;
    if (*softSerial).isTransmittingData == 0 {
        if isSoftSerialTransmitBufferEmpty(softSerial as *mut serialPort_t) {
            // Transmit buffer empty.
            // Start listening if not already in if half-duplex
            if (*softSerial).rxActive == 0 &&
                   (*softSerial).port.options as libc::c_uint &
                       SERIAL_BIDIR as libc::c_int as libc::c_uint != 0 {
                serialOutputPortDeActivate(softSerial);
                serialInputPortActivate(softSerial);
            }
            return
        }
        // data to send
        let fresh0 = (*softSerial).port.txBufferTail;
        (*softSerial).port.txBufferTail =
            (*softSerial).port.txBufferTail.wrapping_add(1);
        let mut byteToSend: uint8_t =
            *(*softSerial).port.txBuffer.offset(fresh0 as isize);
        if (*softSerial).port.txBufferTail >= (*softSerial).port.txBufferSize
           {
            (*softSerial).port.txBufferTail = 0 as libc::c_int as uint32_t
        }
        // build internal buffer, MSB = Stop Bit (1) + data bits (MSB to LSB) + start bit(0) LSB
        (*softSerial).internalTxBuffer =
            ((1 as libc::c_int) << 10 as libc::c_int - 1 as libc::c_int |
                 (byteToSend as libc::c_int) << 1 as libc::c_int) as uint16_t;
        (*softSerial).bitsLeftToTransmit = 10 as libc::c_int as int8_t;
        (*softSerial).isTransmittingData = 1 as libc::c_int as uint8_t;
        if (*softSerial).rxActive as libc::c_int != 0 &&
               (*softSerial).port.options as libc::c_uint &
                   SERIAL_BIDIR as libc::c_int as libc::c_uint != 0 {
            // Half-duplex: Deactivate receiver, activate transmitter
            serialInputPortDeActivate(softSerial);
            serialOutputPortActivate(softSerial);
            // Start sending on next bit timing, as port manipulation takes time,
            // and continuing here may cause bit period to decrease causing sampling errors
            // at the receiver under high rates.
            // Note that there will be (little less than) 1-bit delay; take it as "turn around time".
            // XXX We may be able to reload counter and continue. (Future work.)
            return
        }
    }
    if (*softSerial).bitsLeftToTransmit != 0 {
        mask =
            ((*softSerial).internalTxBuffer as libc::c_int & 1 as libc::c_int)
                as uint8_t;
        (*softSerial).internalTxBuffer =
            ((*softSerial).internalTxBuffer as libc::c_int >>
                 1 as libc::c_int) as uint16_t;
        setTxSignal(softSerial, mask);
        (*softSerial).bitsLeftToTransmit -= 1;
        return
    }
    (*softSerial).isTransmittingData = 0 as libc::c_int as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn applyChangedBits(mut softSerial: *mut softSerial_t) {
    if (*softSerial).rxEdge as libc::c_int == TRAILING as libc::c_int {
        let mut bitToSet: uint8_t = 0;
        bitToSet = (*softSerial).rxLastLeadingEdgeAtBitIndex;
        while (bitToSet as libc::c_int) <
                  (*softSerial).rxBitIndex as libc::c_int {
            (*softSerial).internalRxBuffer =
                ((*softSerial).internalRxBuffer as libc::c_int |
                     (1 as libc::c_int) << bitToSet as libc::c_int) as
                    uint16_t;
            bitToSet = bitToSet.wrapping_add(1)
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn prepareForNextRxByte(mut softSerial:
                                                  *mut softSerial_t) {
    // prepare for next byte
    (*softSerial).rxBitIndex = 0 as libc::c_int as uint8_t;
    (*softSerial).isSearchingForStartBit = 1 as libc::c_int as uint8_t;
    if (*softSerial).rxEdge as libc::c_int == LEADING as libc::c_int {
        (*softSerial).rxEdge = TRAILING as libc::c_int as uint8_t;
        timerChConfigIC((*softSerial).timerHardware,
                        if (*softSerial).port.options as libc::c_uint &
                               SERIAL_INVERTED as libc::c_int as libc::c_uint
                               != 0 {
                            1 as libc::c_int
                        } else { 0 as libc::c_int } != 0,
                        0 as libc::c_int as libc::c_uint);
        serialEnableCC(softSerial);
    };
}
#[no_mangle]
pub unsafe extern "C" fn extractAndStoreRxByte(mut softSerial:
                                                   *mut softSerial_t) {
    if (*softSerial).port.mode as libc::c_uint &
           MODE_RX as libc::c_int as libc::c_uint ==
           0 as libc::c_int as libc::c_uint {
        return
    }
    let mut haveStartBit: uint8_t =
        ((*softSerial).internalRxBuffer as libc::c_int &
             (1 as libc::c_int) << 10 as libc::c_int - 1 as libc::c_int ==
             0 as libc::c_int) as libc::c_int as uint8_t;
    let mut haveStopBit: uint8_t =
        ((*softSerial).internalRxBuffer as libc::c_int &
             (1 as libc::c_int) << 0 as libc::c_int == 1 as libc::c_int) as
            libc::c_int as uint8_t;
    if haveStartBit == 0 || haveStopBit == 0 {
        (*softSerial).receiveErrors =
            (*softSerial).receiveErrors.wrapping_add(1);
        return
    }
    let mut rxByte: uint8_t =
        ((*softSerial).internalRxBuffer as libc::c_int >> 1 as libc::c_int &
             0xff as libc::c_int) as uint8_t;
    if (*softSerial).port.rxCallback.is_some() {
        (*softSerial).port.rxCallback.expect("non-null function pointer")(rxByte
                                                                              as
                                                                              uint16_t,
                                                                          (*softSerial).port.rxCallbackData);
    } else {
        ::core::ptr::write_volatile((*softSerial).port.rxBuffer.offset((*softSerial).port.rxBufferHead
                                                                           as
                                                                           isize),
                                    rxByte);
        (*softSerial).port.rxBufferHead =
            (*softSerial).port.rxBufferHead.wrapping_add(1 as libc::c_int as
                                                             libc::c_uint).wrapping_rem((*softSerial).port.rxBufferSize)
    };
}
#[no_mangle]
pub unsafe extern "C" fn processRxState(mut softSerial: *mut softSerial_t) {
    if (*softSerial).isSearchingForStartBit != 0 { return }
    (*softSerial).rxBitIndex = (*softSerial).rxBitIndex.wrapping_add(1);
    if (*softSerial).rxBitIndex as libc::c_int ==
           10 as libc::c_int - 1 as libc::c_int {
        applyChangedBits(softSerial);
        return
    }
    if (*softSerial).rxBitIndex as libc::c_int == 10 as libc::c_int {
        if (*softSerial).rxEdge as libc::c_int == TRAILING as libc::c_int {
            (*softSerial).internalRxBuffer =
                ((*softSerial).internalRxBuffer as libc::c_int |
                     (1 as libc::c_int) << 0 as libc::c_int) as uint16_t
        }
        extractAndStoreRxByte(softSerial);
        prepareForNextRxByte(softSerial);
    };
}
#[no_mangle]
pub unsafe extern "C" fn onSerialTimerOverflow(mut cbRec:
                                                   *mut timerOvrHandlerRec_t,
                                               mut capture:
                                                   captureCompare_t) {
    let mut self_0: *mut softSerial_t =
        ({
             let mut __mptr: *const timerOvrHandlerRec_t = cbRec;
             (__mptr as
                  *mut libc::c_char).offset(-(664 as libc::c_ulong as isize))
                 as *mut softSerial_t
         });
    if (*self_0).port.mode as libc::c_uint &
           MODE_TX as libc::c_int as libc::c_uint != 0 {
        processTxState(self_0);
    }
    if (*self_0).port.mode as libc::c_uint &
           MODE_RX as libc::c_int as libc::c_uint != 0 {
        processRxState(self_0);
    };
}
#[no_mangle]
pub unsafe extern "C" fn onSerialRxPinChange(mut cbRec:
                                                 *mut timerCCHandlerRec_t,
                                             mut capture: captureCompare_t) {
    let mut self_0: *mut softSerial_t =
        ({
             let mut __mptr: *const timerCCHandlerRec_t = cbRec;
             (__mptr as
                  *mut libc::c_char).offset(-(680 as libc::c_ulong as isize))
                 as *mut softSerial_t
         });
    let mut inverted: bool =
        (*self_0).port.options as libc::c_uint &
            SERIAL_INVERTED as libc::c_int as libc::c_uint != 0;
    if (*self_0).port.mode as libc::c_uint &
           MODE_RX as libc::c_int as libc::c_uint ==
           0 as libc::c_int as libc::c_uint {
        return
    }
    if (*self_0).isSearchingForStartBit != 0 {
        // Synchronize the bit timing so that it will interrupt at the center
        // of the bit period.
        ::core::ptr::write_volatile(&mut (*(*(*self_0).timerHandle).Instance).CNT
                                        as *mut uint32_t,
                                    (*(*(*self_0).timerHandle).Instance).ARR.wrapping_div(2
                                                                                              as
                                                                                              libc::c_int
                                                                                              as
                                                                                              libc::c_uint));
        // For a mono-timer full duplex configuration, this may clobber the
        // transmission because the next callback to the onSerialTimerOverflow
        // will happen too early causing transmission errors.
        // For a dual-timer configuration, there is no problem.
        if (*self_0).timerMode as libc::c_uint !=
               TIMER_MODE_DUAL as libc::c_int as libc::c_uint &&
               (*self_0).isTransmittingData as libc::c_int != 0 {
            (*self_0).transmissionErrors =
                (*self_0).transmissionErrors.wrapping_add(1)
        }
        timerChConfigIC((*self_0).timerHardware,
                        if inverted as libc::c_int != 0 {
                            0 as libc::c_int
                        } else { 1 as libc::c_int } != 0,
                        0 as libc::c_int as libc::c_uint);
        serialEnableCC(self_0);
        (*self_0).rxEdge = LEADING as libc::c_int as uint8_t;
        (*self_0).rxBitIndex = 0 as libc::c_int as uint8_t;
        (*self_0).rxLastLeadingEdgeAtBitIndex = 0 as libc::c_int as uint8_t;
        (*self_0).internalRxBuffer = 0 as libc::c_int as uint16_t;
        (*self_0).isSearchingForStartBit = 0 as libc::c_int as uint8_t;
        return
    }
    if (*self_0).rxEdge as libc::c_int == LEADING as libc::c_int {
        (*self_0).rxLastLeadingEdgeAtBitIndex = (*self_0).rxBitIndex
    }
    applyChangedBits(self_0);
    if (*self_0).rxEdge as libc::c_int == TRAILING as libc::c_int {
        (*self_0).rxEdge = LEADING as libc::c_int as uint8_t;
        timerChConfigIC((*self_0).timerHardware,
                        if inverted as libc::c_int != 0 {
                            0 as libc::c_int
                        } else { 1 as libc::c_int } != 0,
                        0 as libc::c_int as libc::c_uint);
    } else {
        (*self_0).rxEdge = TRAILING as libc::c_int as uint8_t;
        timerChConfigIC((*self_0).timerHardware,
                        if inverted as libc::c_int != 0 {
                            1 as libc::c_int
                        } else { 0 as libc::c_int } != 0,
                        0 as libc::c_int as libc::c_uint);
    }
    serialEnableCC(self_0);
}
/*
 * Standard serial driver API
 */
#[no_mangle]
pub unsafe extern "C" fn softSerialRxBytesWaiting(mut instance:
                                                      *const serialPort_t)
 -> uint32_t {
    if (*instance).mode as libc::c_uint &
           MODE_RX as libc::c_int as libc::c_uint ==
           0 as libc::c_int as libc::c_uint {
        return 0 as libc::c_int as uint32_t
    }
    let mut s: *mut softSerial_t = instance as *mut softSerial_t;
    return (*s).port.rxBufferHead.wrapping_sub((*s).port.rxBufferTail) &
               (*s).port.rxBufferSize.wrapping_sub(1 as libc::c_int as
                                                       libc::c_uint);
}
#[no_mangle]
pub unsafe extern "C" fn softSerialTxBytesFree(mut instance:
                                                   *const serialPort_t)
 -> uint32_t {
    if (*instance).mode as libc::c_uint &
           MODE_TX as libc::c_int as libc::c_uint ==
           0 as libc::c_int as libc::c_uint {
        return 0 as libc::c_int as uint32_t
    }
    let mut s: *mut softSerial_t = instance as *mut softSerial_t;
    let mut bytesUsed: uint8_t =
        ((*s).port.txBufferHead.wrapping_sub((*s).port.txBufferTail) &
             (*s).port.txBufferSize.wrapping_sub(1 as libc::c_int as
                                                     libc::c_uint)) as
            uint8_t;
    return (*s).port.txBufferSize.wrapping_sub(1 as libc::c_int as
                                                   libc::c_uint).wrapping_sub(bytesUsed
                                                                                  as
                                                                                  libc::c_uint);
}
#[no_mangle]
pub unsafe extern "C" fn softSerialReadByte(mut instance: *mut serialPort_t)
 -> uint8_t {
    let mut ch: uint8_t = 0;
    if (*instance).mode as libc::c_uint &
           MODE_RX as libc::c_int as libc::c_uint ==
           0 as libc::c_int as libc::c_uint {
        return 0 as libc::c_int as uint8_t
    }
    if softSerialRxBytesWaiting(instance) == 0 as libc::c_int as libc::c_uint
       {
        return 0 as libc::c_int as uint8_t
    }
    ch = *(*instance).rxBuffer.offset((*instance).rxBufferTail as isize);
    (*instance).rxBufferTail =
        (*instance).rxBufferTail.wrapping_add(1 as libc::c_int as
                                                  libc::c_uint).wrapping_rem((*instance).rxBufferSize);
    return ch;
}
// serialPort API
#[no_mangle]
pub unsafe extern "C" fn softSerialWriteByte(mut s: *mut serialPort_t,
                                             mut ch: uint8_t) {
    if (*s).mode as libc::c_uint & MODE_TX as libc::c_int as libc::c_uint ==
           0 as libc::c_int as libc::c_uint {
        return
    }
    ::core::ptr::write_volatile((*s).txBuffer.offset((*s).txBufferHead as
                                                         isize), ch);
    (*s).txBufferHead =
        (*s).txBufferHead.wrapping_add(1 as libc::c_int as
                                           libc::c_uint).wrapping_rem((*s).txBufferSize);
}
#[no_mangle]
pub unsafe extern "C" fn softSerialSetBaudRate(mut s: *mut serialPort_t,
                                               mut baudRate: uint32_t) {
    let mut softSerial: *mut softSerial_t = s as *mut softSerial_t;
    (*softSerial).port.baudRate = baudRate;
    serialTimerConfigureTimebase((*softSerial).timerHardware, baudRate);
}
#[no_mangle]
pub unsafe extern "C" fn softSerialSetMode(mut instance: *mut serialPort_t,
                                           mut mode: portMode_e) {
    (*instance).mode = mode;
}
#[no_mangle]
pub unsafe extern "C" fn isSoftSerialTransmitBufferEmpty(mut instance:
                                                             *const serialPort_t)
 -> bool {
    return (*instance).txBufferHead == (*instance).txBufferTail;
}
static mut softSerialVTable: serialPortVTable =
    unsafe {
        {
            let mut init =
                serialPortVTable{serialWrite:
                                     Some(softSerialWriteByte as
                                              unsafe extern "C" fn(_:
                                                                       *mut serialPort_t,
                                                                   _: uint8_t)
                                                  -> ()),
                                 serialTotalRxWaiting:
                                     Some(softSerialRxBytesWaiting as
                                              unsafe extern "C" fn(_:
                                                                       *const serialPort_t)
                                                  -> uint32_t),
                                 serialTotalTxFree:
                                     Some(softSerialTxBytesFree as
                                              unsafe extern "C" fn(_:
                                                                       *const serialPort_t)
                                                  -> uint32_t),
                                 serialRead:
                                     Some(softSerialReadByte as
                                              unsafe extern "C" fn(_:
                                                                       *mut serialPort_t)
                                                  -> uint8_t),
                                 serialSetBaudRate:
                                     Some(softSerialSetBaudRate as
                                              unsafe extern "C" fn(_:
                                                                       *mut serialPort_t,
                                                                   _:
                                                                       uint32_t)
                                                  -> ()),
                                 isSerialTransmitBufferEmpty:
                                     Some(isSoftSerialTransmitBufferEmpty as
                                              unsafe extern "C" fn(_:
                                                                       *const serialPort_t)
                                                  -> bool),
                                 setMode:
                                     Some(softSerialSetMode as
                                              unsafe extern "C" fn(_:
                                                                       *mut serialPort_t,
                                                                   _:
                                                                       portMode_e)
                                                  -> ()),
                                 setCtrlLineStateCb: None,
                                 setBaudRateCb: None,
                                 writeBuf: None,
                                 beginWrite: None,
                                 endWrite: None,};
            init
        }
    };
