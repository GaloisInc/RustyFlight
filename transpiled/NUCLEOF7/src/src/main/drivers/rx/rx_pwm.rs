use ::libc;
extern "C" {
    #[no_mangle]
    fn HAL_TIM_IC_Start_IT(htim: *mut TIM_HandleTypeDef, Channel: uint32_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_TIM_IC_ConfigChannel(htim: *mut TIM_HandleTypeDef,
                                sConfig: *mut TIM_IC_InitTypeDef,
                                Channel: uint32_t) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIOAF(io: IO_t, cfg: ioConfig_t, af: uint8_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn timerConfigure(timHw: *const timerHardware_t, period: uint16_t,
                      hz: uint32_t);
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
    fn timerGetByTag(ioTag: ioTag_t) -> *const timerHardware_t;
    #[no_mangle]
    fn timerFindTimerHandle(tim: *mut TIM_TypeDef) -> *mut TIM_HandleTypeDef;
    #[no_mangle]
    fn pwmGetMotors() -> *mut pwmOutputPort_t;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
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
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_def.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   This file contains HAL common defines, enumeration, macros and 
  *          structures definitions. 
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
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief  HAL Status structures definition  
  */
pub type HAL_StatusTypeDef = libc::c_uint;
pub const HAL_TIMEOUT: HAL_StatusTypeDef = 3;
pub const HAL_BUSY: HAL_StatusTypeDef = 2;
pub const HAL_ERROR: HAL_StatusTypeDef = 1;
pub const HAL_OK: HAL_StatusTypeDef = 0;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_IC_InitTypeDef {
    pub ICPolarity: uint32_t,
    pub ICSelection: uint32_t,
    pub ICPrescaler: uint32_t,
    pub ICFilter: uint32_t,
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
// TIM_Channel_1..4
pub type captureCompare_t = uint16_t;
// 16 bit on both 103 and 303, just register access must be 32bit sometimes (use timCCR_t)
pub type timCCR_t = uint32_t;
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
// use different types from capture and overflow - multiple overflow handlers are implemented as linked list
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerChannel_t {
    pub ccr: *mut timCCR_t,
    pub tim: *mut TIM_TypeDef,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pwmOutputPort_t {
    pub channel: timerChannel_t,
    pub pulseScale: libc::c_float,
    pub pulseOffset: libc::c_float,
    pub forceOverflow: bool,
    pub enabled: bool,
    pub io: IO_t,
}
pub type inputFilteringMode_e = libc::c_uint;
pub const INPUT_FILTERING_ENABLED: inputFilteringMode_e = 1;
pub const INPUT_FILTERING_DISABLED: inputFilteringMode_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ppmConfig_s {
    pub ioTag: ioTag_t,
}
pub type ppmConfig_t = ppmConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pwmInputPort_t {
    pub mode: pwmInputMode_e,
    pub channel: uint8_t,
    pub state: uint8_t,
    pub missedEvents: uint8_t,
    pub rise: captureCompare_t,
    pub fall: captureCompare_t,
    pub capture: captureCompare_t,
    pub timerHardware: *const timerHardware_t,
    pub edgeCb: timerCCHandlerRec_t,
    pub overflowCb: timerOvrHandlerRec_t,
}
pub type pwmInputMode_e = libc::c_uint;
pub const INPUT_MODE_PWM: pwmInputMode_e = 1;
pub const INPUT_MODE_PPM: pwmInputMode_e = 0;
pub type ppmDevice_t = ppmDevice_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ppmDevice_s {
    pub currentCapture: uint32_t,
    pub currentTime: uint32_t,
    pub deltaTime: uint32_t,
    pub captures: [uint32_t; 12],
    pub largeCounter: uint32_t,
    pub pulseIndex: uint8_t,
    pub numChannels: int8_t,
    pub numChannelsPrevFrame: int8_t,
    pub stableFramesSeenCount: uint8_t,
    pub tracking: bool,
    pub overflowed: bool,
}
pub type eventSource_e = libc::c_uint;
pub const SOURCE_EDGE: eventSource_e = 1;
pub const SOURCE_OVERFLOW: eventSource_e = 0;
pub type ppmISREvent_t = ppmISREvent_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ppmISREvent_s {
    pub capture: uint32_t,
    pub source: eventSource_e,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pwmConfig_s {
    pub ioTags: [ioTag_t; 8],
    pub inputFilteringMode: inputFilteringMode_e,
}
pub type pwmConfig_t = pwmConfig_s;
static mut inputFilteringMode: inputFilteringMode_e =
    INPUT_FILTERING_DISABLED;
static mut pwmInputPorts: [pwmInputPort_t; 8] =
    [pwmInputPort_t{mode: INPUT_MODE_PPM,
                    channel: 0,
                    state: 0,
                    missedEvents: 0,
                    rise: 0,
                    fall: 0,
                    capture: 0,
                    timerHardware: 0 as *const timerHardware_t,
                    edgeCb: timerCCHandlerRec_t{fn_0: None,},
                    overflowCb:
                        timerOvrHandlerRec_t{fn_0: None,
                                             next:
                                                 0 as
                                                     *const timerOvrHandlerRec_s
                                                     as
                                                     *mut timerOvrHandlerRec_s,},};
        8];
static mut captures: [uint16_t; 12] = [0; 12];
static mut ppmFrameCount: uint8_t = 0 as libc::c_int as uint8_t;
static mut lastPPMFrameCount: uint8_t = 0 as libc::c_int as uint8_t;
static mut ppmCountDivisor: uint8_t = 1 as libc::c_int as uint8_t;
static mut ppmDev: ppmDevice_t =
    ppmDevice_t{currentCapture: 0,
                currentTime: 0,
                deltaTime: 0,
                captures: [0; 12],
                largeCounter: 0,
                pulseIndex: 0,
                numChannels: 0,
                numChannelsPrevFrame: 0,
                stableFramesSeenCount: 0,
                tracking: false,
                overflowed: false,};
#[no_mangle]
pub unsafe extern "C" fn isPPMDataBeingReceived() -> bool {
    return ppmFrameCount as libc::c_int != lastPPMFrameCount as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn resetPPMDataReceivedState() {
    lastPPMFrameCount = ppmFrameCount;
}
static mut ppmEvents: [ppmISREvent_t; 20] =
    [ppmISREvent_t{capture: 0, source: SOURCE_OVERFLOW,}; 20];
static mut ppmEventIndex: uint8_t = 0 as libc::c_int as uint8_t;
#[no_mangle]
pub unsafe extern "C" fn ppmISREvent(mut source: eventSource_e,
                                     mut capture: uint32_t) {
    ppmEventIndex =
        ((ppmEventIndex as libc::c_int + 1 as libc::c_int) as
             libc::c_ulong).wrapping_rem((::core::mem::size_of::<[ppmISREvent_t; 20]>()
                                              as
                                              libc::c_ulong).wrapping_div(::core::mem::size_of::<ppmISREvent_t>()
                                                                              as
                                                                              libc::c_ulong))
            as uint8_t;
    ppmEvents[ppmEventIndex as usize].source = source;
    ppmEvents[ppmEventIndex as usize].capture = capture;
}
unsafe extern "C" fn ppmResetDevice() {
    ppmDev.pulseIndex = 0 as libc::c_int as uint8_t;
    ppmDev.currentCapture = 0 as libc::c_int as uint32_t;
    ppmDev.currentTime = 0 as libc::c_int as uint32_t;
    ppmDev.deltaTime = 0 as libc::c_int as uint32_t;
    ppmDev.largeCounter = 0 as libc::c_int as uint32_t;
    ppmDev.numChannels = -(1 as libc::c_int) as int8_t;
    ppmDev.numChannelsPrevFrame = -(1 as libc::c_int) as int8_t;
    ppmDev.stableFramesSeenCount = 0 as libc::c_int as uint8_t;
    ppmDev.tracking = 0 as libc::c_int != 0;
    ppmDev.overflowed = 0 as libc::c_int != 0;
}
unsafe extern "C" fn ppmOverflowCallback(mut cbRec: *mut timerOvrHandlerRec_t,
                                         mut capture: captureCompare_t) {
    ppmISREvent(SOURCE_OVERFLOW, capture as uint32_t);
    ppmDev.largeCounter =
        (ppmDev.largeCounter as
             libc::c_uint).wrapping_add((capture as libc::c_int +
                                             1 as libc::c_int) as
                                            libc::c_uint) as uint32_t as
            uint32_t;
    if capture as libc::c_int == 0x10000 as libc::c_int - 1 as libc::c_int {
        ppmDev.overflowed = 1 as libc::c_int != 0
    };
}
unsafe extern "C" fn ppmEdgeCallback(mut cbRec: *mut timerCCHandlerRec_t,
                                     mut capture: captureCompare_t) {
    ppmISREvent(SOURCE_EDGE, capture as uint32_t);
    let mut i: int32_t = 0;
    let mut previousTime: uint32_t = ppmDev.currentTime;
    let mut previousCapture: uint32_t = ppmDev.currentCapture;
    // TIMUP
    //uint32_t previousTime;
    /* Grab the new count */
    let mut currentTime: uint32_t = capture as uint32_t;
    /* Convert to 32-bit timer result */
    currentTime =
        (currentTime as libc::c_uint).wrapping_add(ppmDev.largeCounter) as
            uint32_t as uint32_t;
    if (capture as libc::c_uint) < previousCapture {
        if ppmDev.overflowed {
            currentTime =
                (currentTime as
                     libc::c_uint).wrapping_add(0x10000 as libc::c_int as
                                                    libc::c_uint) as uint32_t
                    as uint32_t
        }
    }
    // Divide value if Oneshot, Multishot or brushed motors are active and the timer is shared
    currentTime = currentTime.wrapping_div(ppmCountDivisor as libc::c_uint);
    /* Capture computation */
    if currentTime > previousTime {
        ppmDev.deltaTime =
            currentTime.wrapping_sub(previousTime.wrapping_add((if ppmDev.overflowed
                                                                       as
                                                                       libc::c_int
                                                                       != 0 {
                                                                    (0x10000
                                                                         as
                                                                         libc::c_int)
                                                                        /
                                                                        ppmCountDivisor
                                                                            as
                                                                            libc::c_int
                                                                } else {
                                                                    0 as
                                                                        libc::c_int
                                                                }) as
                                                                   libc::c_uint))
    } else {
        ppmDev.deltaTime =
            ((0x10000 as libc::c_int / ppmCountDivisor as libc::c_int) as
                 libc::c_uint).wrapping_add(currentTime).wrapping_sub(previousTime)
    }
    ppmDev.overflowed = 0 as libc::c_int != 0;
    /* Store the current measurement */
    ppmDev.currentTime = currentTime;
    ppmDev.currentCapture = capture as uint32_t;
    /* Sync pulse detection */
    if ppmDev.deltaTime > 2700 as libc::c_int as libc::c_uint {
        if ppmDev.pulseIndex as libc::c_int ==
               ppmDev.numChannelsPrevFrame as libc::c_int &&
               ppmDev.pulseIndex as libc::c_int >= 4 as libc::c_int &&
               ppmDev.pulseIndex as libc::c_int <= 12 as libc::c_int {
            /* If we see n simultaneous frames of the same
               number of channels we save it as our frame size */
            if (ppmDev.stableFramesSeenCount as libc::c_int) <
                   25 as libc::c_int {
                ppmDev.stableFramesSeenCount =
                    ppmDev.stableFramesSeenCount.wrapping_add(1)
            } else { ppmDev.numChannels = ppmDev.pulseIndex as int8_t }
        } else { ppmDev.stableFramesSeenCount = 0 as libc::c_int as uint8_t }
        /* We rely on the supervisor to set captureValue to invalid
           if no valid frame is found otherwise we ride over it */
        if ppmDev.pulseIndex as libc::c_int ==
               ppmDev.numChannels as libc::c_int &&
               ppmDev.tracking as libc::c_int != 0 {
            /* Check if the last frame was well formed */
            /* The last frame was well formed */
            i = 0 as libc::c_int;
            while i < ppmDev.numChannels as libc::c_int {
                captures[i as usize] =
                    ppmDev.captures[i as usize] as uint16_t;
                i += 1
            }
            i = ppmDev.numChannels as int32_t;
            while i < 12 as libc::c_int {
                captures[i as usize] = 0 as libc::c_int as uint16_t;
                i += 1
            }
            ppmFrameCount = ppmFrameCount.wrapping_add(1)
        }
        ppmDev.tracking = 1 as libc::c_int != 0;
        ppmDev.numChannelsPrevFrame = ppmDev.pulseIndex as int8_t;
        ppmDev.pulseIndex = 0 as libc::c_int as uint8_t
    } else if ppmDev.tracking {
        /* Valid pulse duration 0.75 to 2.5 ms*/
        if ppmDev.deltaTime > 750 as libc::c_int as libc::c_uint &&
               ppmDev.deltaTime < 2250 as libc::c_int as libc::c_uint &&
               (ppmDev.pulseIndex as libc::c_int) < 12 as libc::c_int {
            ppmDev.captures[ppmDev.pulseIndex as usize] = ppmDev.deltaTime;
            ppmDev.pulseIndex = ppmDev.pulseIndex.wrapping_add(1)
        } else {
            /* Not a valid pulse duration */
            ppmDev.tracking = 0 as libc::c_int != 0;
            i = 0 as libc::c_int;
            while i < 12 as libc::c_int {
                ppmDev.captures[i as usize] = 0 as libc::c_int as uint32_t;
                i += 1
            }
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn isPWMDataBeingReceived() -> bool {
    let mut channel: libc::c_int = 0;
    channel = 0 as libc::c_int;
    while channel < 12 as libc::c_int {
        if captures[channel as usize] as libc::c_int != 0 as libc::c_int {
            return 1 as libc::c_int != 0
        }
        channel += 1
    }
    return 0 as libc::c_int != 0;
}
unsafe extern "C" fn pwmOverflowCallback(mut cbRec: *mut timerOvrHandlerRec_t,
                                         mut capture: captureCompare_t) {
    let mut pwmInputPort: *mut pwmInputPort_t =
        ({
             let mut __mptr: *const timerOvrHandlerRec_t = cbRec;
             (__mptr as
                  *mut libc::c_char).offset(-(32 as libc::c_ulong as isize))
                 as *mut pwmInputPort_t
         });
    (*pwmInputPort).missedEvents =
        (*pwmInputPort).missedEvents.wrapping_add(1);
    if (*pwmInputPort).missedEvents as libc::c_int > 10 as libc::c_int {
        captures[(*pwmInputPort).channel as usize] =
            0 as libc::c_int as uint16_t;
        (*pwmInputPort).missedEvents = 0 as libc::c_int as uint8_t
    };
}
unsafe extern "C" fn pwmEdgeCallback(mut cbRec: *mut timerCCHandlerRec_t,
                                     mut capture: captureCompare_t) {
    let mut pwmInputPort: *mut pwmInputPort_t =
        ({
             let mut __mptr: *const timerCCHandlerRec_t = cbRec;
             (__mptr as
                  *mut libc::c_char).offset(-(24 as libc::c_ulong as isize))
                 as *mut pwmInputPort_t
         });
    let mut timerHardwarePtr: *const timerHardware_t =
        (*pwmInputPort).timerHardware;
    if (*pwmInputPort).state as libc::c_int == 0 as libc::c_int {
        (*pwmInputPort).rise = capture;
        (*pwmInputPort).state = 1 as libc::c_int as uint8_t;
        pwmICConfig((*timerHardwarePtr).tim, (*timerHardwarePtr).channel,
                    ((0x1 as libc::c_uint) << 1 as libc::c_uint) as uint16_t);
    } else {
        (*pwmInputPort).fall = capture;
        // compute and store capture
        (*pwmInputPort).capture =
            ((*pwmInputPort).fall as libc::c_int -
                 (*pwmInputPort).rise as libc::c_int) as captureCompare_t;
        captures[(*pwmInputPort).channel as usize] = (*pwmInputPort).capture;
        // switch state
        (*pwmInputPort).state = 0 as libc::c_int as uint8_t;
        pwmICConfig((*timerHardwarePtr).tim, (*timerHardwarePtr).channel,
                    0 as libc::c_uint as uint16_t);
        (*pwmInputPort).missedEvents = 0 as libc::c_int as uint8_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn pwmICConfig(mut tim: *mut TIM_TypeDef,
                                     mut channel: uint8_t,
                                     mut polarity: uint16_t) {
    let mut Handle: *mut TIM_HandleTypeDef = timerFindTimerHandle(tim);
    if Handle.is_null() { return }
    let mut TIM_ICInitStructure: TIM_IC_InitTypeDef =
        TIM_IC_InitTypeDef{ICPolarity: 0,
                           ICSelection: 0,
                           ICPrescaler: 0,
                           ICFilter: 0,};
    TIM_ICInitStructure.ICPolarity = polarity as uint32_t;
    TIM_ICInitStructure.ICSelection =
        (0x1 as libc::c_uint) << 0 as libc::c_uint;
    TIM_ICInitStructure.ICPrescaler = 0 as libc::c_uint;
    if inputFilteringMode as libc::c_uint ==
           INPUT_FILTERING_ENABLED as libc::c_int as libc::c_uint {
        TIM_ICInitStructure.ICFilter = 0x3 as libc::c_int as uint32_t
    } else { TIM_ICInitStructure.ICFilter = 0 as libc::c_int as uint32_t }
    HAL_TIM_IC_ConfigChannel(Handle, &mut TIM_ICInitStructure,
                             channel as uint32_t);
    HAL_TIM_IC_Start_IT(Handle, channel as uint32_t);
}
#[no_mangle]
pub unsafe extern "C" fn pwmRxInit(mut pwmConfig: *const pwmConfig_t) {
    inputFilteringMode = (*pwmConfig).inputFilteringMode;
    let mut channel: libc::c_int = 0 as libc::c_int;
    while channel < 8 as libc::c_int {
        let mut port: *mut pwmInputPort_t =
            &mut *pwmInputPorts.as_mut_ptr().offset(channel as isize) as
                *mut pwmInputPort_t;
        let mut timer: *const timerHardware_t =
            timerGetByTag((*pwmConfig).ioTags[channel as usize]);
        if !timer.is_null() {
            (*port).state = 0 as libc::c_int as uint8_t;
            (*port).missedEvents = 0 as libc::c_int as uint8_t;
            (*port).channel = channel as uint8_t;
            (*port).mode = INPUT_MODE_PWM;
            (*port).timerHardware = timer;
            let mut io: IO_t =
                IOGetByTag((*pwmConfig).ioTags[channel as usize]);
            IOInit(io, OWNER_PWMINPUT,
                   (channel + 1 as libc::c_int) as uint8_t);
            IOConfigGPIOAF(io,
                           (0x2 as libc::c_uint |
                                (0 as libc::c_uint) << 2 as libc::c_int |
                                (0 as libc::c_uint) << 5 as libc::c_int) as
                               ioConfig_t, (*timer).alternateFunction);
            timerConfigure(timer, 0x10000 as libc::c_int as uint16_t,
                           (1 as libc::c_int * 1000000 as libc::c_int) as
                               uint32_t);
            timerChCCHandlerInit(&mut (*port).edgeCb,
                                 Some(pwmEdgeCallback as
                                          unsafe extern "C" fn(_:
                                                                   *mut timerCCHandlerRec_t,
                                                               _:
                                                                   captureCompare_t)
                                              -> ()));
            timerChOvrHandlerInit(&mut (*port).overflowCb,
                                  Some(pwmOverflowCallback as
                                           unsafe extern "C" fn(_:
                                                                    *mut timerOvrHandlerRec_t,
                                                                _:
                                                                    captureCompare_t)
                                               -> ()));
            timerChConfigCallbacks(timer, &mut (*port).edgeCb,
                                   &mut (*port).overflowCb);
            pwmICConfig((*timer).tim, (*timer).channel,
                        0 as libc::c_uint as uint16_t);
        }
        /* TODO: maybe fail here if not enough channels? */
        channel += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn ppmAvoidPWMTimerClash(mut pwmTimer:
                                                   *mut TIM_TypeDef) {
    let mut motors: *mut pwmOutputPort_t = pwmGetMotors();
    let mut motorIndex: libc::c_int = 0 as libc::c_int;
    while motorIndex < 8 as libc::c_int {
        if !(*motors.offset(motorIndex as isize)).enabled ||
               (*motors.offset(motorIndex as isize)).channel.tim != pwmTimer {
            motorIndex += 1
        } else {
            ppmCountDivisor =
                timerClock(pwmTimer).wrapping_div((*pwmTimer).PSC.wrapping_add(1
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   libc::c_uint))
                    as uint8_t;
            return
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn ppmRxInit(mut ppmConfig: *const ppmConfig_t) {
    ppmResetDevice();
    let mut port: *mut pwmInputPort_t =
        &mut *pwmInputPorts.as_mut_ptr().offset(0 as libc::c_int as isize) as
            *mut pwmInputPort_t;
    let mut timer: *const timerHardware_t = timerGetByTag((*ppmConfig).ioTag);
    if timer.is_null() {
        /* TODO: fail here? */
        return
    }
    ppmAvoidPWMTimerClash((*timer).tim);
    (*port).mode = INPUT_MODE_PPM;
    (*port).timerHardware = timer;
    let mut io: IO_t = IOGetByTag((*ppmConfig).ioTag);
    IOInit(io, OWNER_PPMINPUT, 0 as libc::c_int as uint8_t);
    IOConfigGPIOAF(io,
                   (0x2 as libc::c_uint |
                        (0 as libc::c_uint) << 2 as libc::c_int |
                        (0 as libc::c_uint) << 5 as libc::c_int) as
                       ioConfig_t, (*timer).alternateFunction);
    timerConfigure(timer, 0x10000 as libc::c_int as uint16_t,
                   (1 as libc::c_int * 1000000 as libc::c_int) as uint32_t);
    timerChCCHandlerInit(&mut (*port).edgeCb,
                         Some(ppmEdgeCallback as
                                  unsafe extern "C" fn(_:
                                                           *mut timerCCHandlerRec_t,
                                                       _: captureCompare_t)
                                      -> ()));
    timerChOvrHandlerInit(&mut (*port).overflowCb,
                          Some(ppmOverflowCallback as
                                   unsafe extern "C" fn(_:
                                                            *mut timerOvrHandlerRec_t,
                                                        _: captureCompare_t)
                                       -> ()));
    timerChConfigCallbacks(timer, &mut (*port).edgeCb,
                           &mut (*port).overflowCb);
    pwmICConfig((*timer).tim, (*timer).channel,
                0 as libc::c_uint as uint16_t);
}
#[no_mangle]
pub unsafe extern "C" fn ppmRead(mut channel: uint8_t) -> uint16_t {
    return captures[channel as usize];
}
#[no_mangle]
pub unsafe extern "C" fn pwmRead(mut channel: uint8_t) -> uint16_t {
    return captures[channel as usize];
}
