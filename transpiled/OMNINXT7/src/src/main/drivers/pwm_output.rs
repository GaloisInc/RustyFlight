use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn ceilf(_: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn lrintf(_: libc::c_float) -> libc::c_long;
    /* *
  * @}
  */
    /* * @addtogroup TIMEx_Exported_Functions_Group3
  * @{
  */
/*  Timer Complementary PWM functions  ****************************************/
/* Blocking mode: Polling */
    #[no_mangle]
    fn HAL_TIMEx_PWMN_Start(htim: *mut TIM_HandleTypeDef, Channel: uint32_t)
     -> HAL_StatusTypeDef;
    /* Blocking mode: Polling */
    #[no_mangle]
    fn HAL_TIM_Base_Start(htim: *mut TIM_HandleTypeDef) -> HAL_StatusTypeDef;
    /* Blocking mode: Polling */
    #[no_mangle]
    fn HAL_TIM_PWM_Start(htim: *mut TIM_HandleTypeDef, Channel: uint32_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_TIM_PWM_ConfigChannel(htim: *mut TIM_HandleTypeDef,
                                 sConfig: *mut TIM_OC_InitTypeDef,
                                 Channel: uint32_t) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn delayMicroseconds(us: timeUs_t);
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIOAF(io: IO_t, cfg: ioConfig_t, af: uint8_t);
    #[no_mangle]
    fn timerChCCR(timHw: *const timerHardware_t) -> *mut timCCR_t;
    #[no_mangle]
    fn timerForceOverflow(tim: *mut TIM_TypeDef);
    #[no_mangle]
    fn timerClock(tim: *mut TIM_TypeDef) -> uint32_t;
    #[no_mangle]
    fn configTimeBase(tim: *mut TIM_TypeDef, period: uint16_t, hz: uint32_t);
    #[no_mangle]
    fn timerGetByTag(ioTag: ioTag_t) -> *const timerHardware_t;
    #[no_mangle]
    fn timerFindTimerHandle(tim: *mut TIM_TypeDef) -> *mut TIM_HandleTypeDef;
    #[no_mangle]
    fn getMotorDmaOutput(index: uint8_t) -> *mut motorDmaOutput_t;
    #[no_mangle]
    fn pwmDshotMotorHardwareConfig(timerHardware: *const timerHardware_t,
                                   motorIndex: uint8_t,
                                   pwmProtocolType: motorPwmProtocolTypes_e,
                                   output: uint8_t);
    #[no_mangle]
    fn pwmCompleteDshotMotorUpdate(motorCount: uint8_t);
    #[no_mangle]
    fn pwmWriteDshotInt(index: uint8_t, value: uint16_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
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
/* !< DMA Stream Index                       */
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_tim.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of TIM HAL module.
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
/* * @addtogroup TIM
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup TIM_Exported_Types TIM Exported Types
  * @{
  */
/* * 
  * @brief  TIM Time base Configuration Structure definition  
  */
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
/* * 
  * @brief  TIM Output Compare Configuration Structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_OC_InitTypeDef {
    pub OCMode: uint32_t,
    pub Pulse: uint32_t,
    pub OCPolarity: uint32_t,
    pub OCNPolarity: uint32_t,
    pub OCFastMode: uint32_t,
    pub OCIdleState: uint32_t,
    pub OCNIdleState: uint32_t,
}
/* * 
  * @brief  HAL State structures definition  
  */
pub type HAL_TIM_StateTypeDef = libc::c_uint;
/* !< Reception process is ongoing                */
/* !< Timeout state                               */
pub const HAL_TIM_STATE_ERROR: HAL_TIM_StateTypeDef = 4;
/* !< An internal process is ongoing              */
pub const HAL_TIM_STATE_TIMEOUT: HAL_TIM_StateTypeDef = 3;
/* !< Peripheral Initialized and ready for use    */
pub const HAL_TIM_STATE_BUSY: HAL_TIM_StateTypeDef = 2;
/* !< Peripheral not yet initialized or disabled  */
pub const HAL_TIM_STATE_READY: HAL_TIM_StateTypeDef = 1;
pub const HAL_TIM_STATE_RESET: HAL_TIM_StateTypeDef = 0;
/* * 
  * @brief  HAL Active channel structures definition  
  */
pub type HAL_TIM_ActiveChannel = libc::c_uint;
/* !< All active channels cleared */
/* !< The active channel is 4     */
pub const HAL_TIM_ACTIVE_CHANNEL_CLEARED: HAL_TIM_ActiveChannel = 0;
/* !< The active channel is 3     */
pub const HAL_TIM_ACTIVE_CHANNEL_4: HAL_TIM_ActiveChannel = 8;
/* !< The active channel is 2     */
pub const HAL_TIM_ACTIVE_CHANNEL_3: HAL_TIM_ActiveChannel = 4;
/* !< The active channel is 1     */
pub const HAL_TIM_ACTIVE_CHANNEL_2: HAL_TIM_ActiveChannel = 2;
pub const HAL_TIM_ACTIVE_CHANNEL_1: HAL_TIM_ActiveChannel = 1;
/* * 
  * @brief  TIM Time Base Handle Structure definition  
  */
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
pub type timeDelta_t = int32_t;
pub type timeUs_t = uint32_t;
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
pub type ioTag_t = uint8_t;
pub type IO_t = *mut libc::c_void;
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
// packet tag to specify IO pin
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
pub type C2RustUnnamed = libc::c_uint;
pub const TIMER_OUTPUT_N_CHANNEL: C2RustUnnamed = 2;
pub const TIMER_OUTPUT_INVERTED: C2RustUnnamed = 1;
pub const TIMER_OUTPUT_NONE: C2RustUnnamed = 0;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const DSHOT_CMD_MAX: C2RustUnnamed_0 = 47;
pub const DSHOT_CMD_SILENT_MODE_ON_OFF: C2RustUnnamed_0 = 31;
pub const DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF: C2RustUnnamed_0 = 30;
pub const DSHOT_CMD_LED3_OFF: C2RustUnnamed_0 = 29;
pub const DSHOT_CMD_LED2_OFF: C2RustUnnamed_0 = 28;
pub const DSHOT_CMD_LED1_OFF: C2RustUnnamed_0 = 27;
pub const DSHOT_CMD_LED0_OFF: C2RustUnnamed_0 = 26;
pub const DSHOT_CMD_LED3_ON: C2RustUnnamed_0 = 25;
pub const DSHOT_CMD_LED2_ON: C2RustUnnamed_0 = 24;
pub const DSHOT_CMD_LED1_ON: C2RustUnnamed_0 = 23;
pub const DSHOT_CMD_LED0_ON: C2RustUnnamed_0 = 22;
pub const DSHOT_CMD_SPIN_DIRECTION_REVERSED: C2RustUnnamed_0 = 21;
pub const DSHOT_CMD_SPIN_DIRECTION_NORMAL: C2RustUnnamed_0 = 20;
pub const DSHOT_CMD_SAVE_SETTINGS: C2RustUnnamed_0 = 12;
pub const DSHOT_CMD_SETTINGS_REQUEST: C2RustUnnamed_0 = 11;
pub const DSHOT_CMD_3D_MODE_ON: C2RustUnnamed_0 = 10;
pub const DSHOT_CMD_3D_MODE_OFF: C2RustUnnamed_0 = 9;
pub const DSHOT_CMD_SPIN_DIRECTION_2: C2RustUnnamed_0 = 8;
pub const DSHOT_CMD_SPIN_DIRECTION_1: C2RustUnnamed_0 = 7;
pub const DSHOT_CMD_ESC_INFO: C2RustUnnamed_0 = 6;
pub const DSHOT_CMD_BEACON5: C2RustUnnamed_0 = 5;
pub const DSHOT_CMD_BEACON4: C2RustUnnamed_0 = 4;
pub const DSHOT_CMD_BEACON3: C2RustUnnamed_0 = 3;
pub const DSHOT_CMD_BEACON2: C2RustUnnamed_0 = 2;
pub const DSHOT_CMD_BEACON1: C2RustUnnamed_0 = 1;
pub const DSHOT_CMD_MOTOR_STOP: C2RustUnnamed_0 = 0;
pub type motorPwmProtocolTypes_e = libc::c_uint;
pub const PWM_TYPE_MAX: motorPwmProtocolTypes_e = 10;
pub const PWM_TYPE_PROSHOT1000: motorPwmProtocolTypes_e = 9;
pub const PWM_TYPE_DSHOT1200: motorPwmProtocolTypes_e = 8;
pub const PWM_TYPE_DSHOT600: motorPwmProtocolTypes_e = 7;
pub const PWM_TYPE_DSHOT300: motorPwmProtocolTypes_e = 6;
pub const PWM_TYPE_DSHOT150: motorPwmProtocolTypes_e = 5;
pub const PWM_TYPE_BRUSHED: motorPwmProtocolTypes_e = 4;
pub const PWM_TYPE_MULTISHOT: motorPwmProtocolTypes_e = 3;
pub const PWM_TYPE_ONESHOT42: motorPwmProtocolTypes_e = 2;
pub const PWM_TYPE_ONESHOT125: motorPwmProtocolTypes_e = 1;
pub const PWM_TYPE_STANDARD: motorPwmProtocolTypes_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct motorDmaTimer_t {
    pub timer: *mut TIM_TypeDef,
    pub dmaBurstRef: *mut DMA_Stream_TypeDef,
    pub dmaBurstLength: uint16_t,
    pub dmaBurstBuffer: [uint32_t; 72],
    pub timerDmaSources: uint16_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct motorDmaOutput_t {
    pub ioTag: ioTag_t,
    pub timerHardware: *const timerHardware_t,
    pub value: uint16_t,
    pub timerDmaSource: uint16_t,
    pub configured: bool,
    pub timer: *mut motorDmaTimer_t,
    pub requestTelemetry: bool,
    pub dmaBuffer: [uint32_t; 18],
}
pub type pwmWriteFn
    =
    unsafe extern "C" fn(_: uint8_t, _: libc::c_float) -> ();
pub type pwmCompleteWriteFn = unsafe extern "C" fn(_: uint8_t) -> ();
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct motorDevConfig_s {
    pub motorPwmRate: uint16_t,
    pub motorPwmProtocol: uint8_t,
    pub motorPwmInversion: uint8_t,
    pub useUnsyncedPwm: uint8_t,
    pub useBurstDshot: uint8_t,
    pub ioTags: [ioTag_t; 8],
}
pub type motorDevConfig_t = motorDevConfig_s;
pub type loadDmaBufferFn
    =
    unsafe extern "C" fn(_: *mut uint32_t, _: libc::c_int, _: uint16_t)
        -> uint8_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct servoDevConfig_s {
    pub servoCenterPulse: uint16_t,
    pub servoPwmRate: uint16_t,
    pub ioTags: [ioTag_t; 8],
}
pub type servoDevConfig_t = servoDevConfig_s;
pub type dshotCommandControl_t = dshotCommandControl_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct dshotCommandControl_s {
    pub nextCommandAtUs: timeUs_t,
    pub delayAfterCommandUs: timeUs_t,
    pub waitingForIdle: bool,
    pub repeats: uint8_t,
    pub command: [uint8_t; 8],
}
#[inline]
unsafe extern "C" fn cmpTimeUs(mut a: timeUs_t, mut b: timeUs_t)
 -> timeDelta_t {
    return a.wrapping_sub(b) as timeDelta_t;
}
// TIMUP
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
#[link_section = ".fastram_bss"]
static mut pwmWrite: Option<pwmWriteFn> = None;
#[link_section = ".fastram_bss"]
static mut motors: [pwmOutputPort_t; 8] =
    [pwmOutputPort_t{channel:
                         timerChannel_t{ccr:
                                            0 as *const timCCR_t as
                                                *mut timCCR_t,
                                        tim:
                                            0 as *const TIM_TypeDef as
                                                *mut TIM_TypeDef,},
                     pulseScale: 0.,
                     pulseOffset: 0.,
                     forceOverflow: false,
                     enabled: false,
                     io: 0 as *const libc::c_void as *mut libc::c_void,}; 8];
#[link_section = ".fastram_bss"]
static mut pwmCompleteWrite: Option<pwmCompleteWriteFn> = None;
#[no_mangle]
#[link_section = ".fastram_bss"]
pub static mut loadDmaBuffer: Option<loadDmaBufferFn> = None;
static mut dshotCommandControl: dshotCommandControl_t =
    dshotCommandControl_t{nextCommandAtUs: 0,
                          delayAfterCommandUs: 0,
                          waitingForIdle: false,
                          repeats: 0,
                          command: [0; 8],};
static mut servos: [pwmOutputPort_t; 8] =
    [pwmOutputPort_t{channel:
                         timerChannel_t{ccr:
                                            0 as *const timCCR_t as
                                                *mut timCCR_t,
                                        tim:
                                            0 as *const TIM_TypeDef as
                                                *mut TIM_TypeDef,},
                     pulseScale: 0.,
                     pulseOffset: 0.,
                     forceOverflow: false,
                     enabled: false,
                     io: 0 as *const libc::c_void as *mut libc::c_void,}; 8];
static mut beeperPwm: pwmOutputPort_t =
    pwmOutputPort_t{channel:
                        timerChannel_t{ccr:
                                           0 as *const timCCR_t as
                                               *mut timCCR_t,
                                       tim:
                                           0 as *const TIM_TypeDef as
                                               *mut TIM_TypeDef,},
                    pulseScale: 0.,
                    pulseOffset: 0.,
                    forceOverflow: false,
                    enabled: false,
                    io: 0 as *const libc::c_void as *mut libc::c_void,};
static mut freqBeep: uint16_t = 0 as libc::c_int as uint16_t;
static mut pwmMotorsEnabled: bool = 0 as libc::c_int != 0;
static mut isDshot: bool = 0 as libc::c_int != 0;
#[no_mangle]
#[link_section = ".fastram_bss"]
pub static mut useBurstDshot: bool = 0 as libc::c_int != 0;
unsafe extern "C" fn pwmOCConfig(mut tim: *mut TIM_TypeDef,
                                 mut channel: uint8_t, mut value: uint16_t,
                                 mut output: uint8_t) {
    let mut Handle: *mut TIM_HandleTypeDef = timerFindTimerHandle(tim);
    if Handle.is_null() { return }
    let mut TIM_OCInitStructure: TIM_OC_InitTypeDef =
        TIM_OC_InitTypeDef{OCMode: 0,
                           Pulse: 0,
                           OCPolarity: 0,
                           OCNPolarity: 0,
                           OCFastMode: 0,
                           OCIdleState: 0,
                           OCNIdleState: 0,};
    TIM_OCInitStructure.OCMode =
        (0x4 as libc::c_uint) << 4 as libc::c_uint |
            (0x2 as libc::c_uint) << 4 as libc::c_uint;
    TIM_OCInitStructure.OCIdleState =
        (0x1 as libc::c_uint) << 8 as libc::c_uint;
    TIM_OCInitStructure.OCPolarity =
        if output as libc::c_int & TIMER_OUTPUT_INVERTED as libc::c_int != 0 {
            ((0x1 as libc::c_uint)) << 1 as libc::c_uint
        } else { 0 as libc::c_uint };
    TIM_OCInitStructure.OCNIdleState =
        (0x1 as libc::c_uint) << 9 as libc::c_uint;
    TIM_OCInitStructure.OCNPolarity =
        if output as libc::c_int & TIMER_OUTPUT_INVERTED as libc::c_int != 0 {
            ((0x1 as libc::c_uint)) << 3 as libc::c_uint
        } else { 0 as libc::c_uint };
    TIM_OCInitStructure.Pulse = value as uint32_t;
    TIM_OCInitStructure.OCFastMode = 0 as libc::c_uint;
    HAL_TIM_PWM_ConfigChannel(Handle, &mut TIM_OCInitStructure,
                              channel as uint32_t);
}
#[no_mangle]
pub unsafe extern "C" fn pwmOutConfig(mut channel: *mut timerChannel_t,
                                      mut timerHardware:
                                          *const timerHardware_t,
                                      mut hz: uint32_t, mut period: uint16_t,
                                      mut value: uint16_t,
                                      mut inversion: uint8_t) {
    let mut Handle: *mut TIM_HandleTypeDef =
        timerFindTimerHandle((*timerHardware).tim);
    if Handle.is_null() { return }
    configTimeBase((*timerHardware).tim, period, hz);
    pwmOCConfig((*timerHardware).tim, (*timerHardware).channel, value,
                if inversion as libc::c_int != 0 {
                    ((*timerHardware).output as libc::c_int) ^
                        TIMER_OUTPUT_INVERTED as libc::c_int
                } else { (*timerHardware).output as libc::c_int } as uint8_t);
    if (*timerHardware).output as libc::c_int &
           TIMER_OUTPUT_N_CHANNEL as libc::c_int != 0 {
        HAL_TIMEx_PWMN_Start(Handle, (*timerHardware).channel as uint32_t);
    } else {
        HAL_TIM_PWM_Start(Handle, (*timerHardware).channel as uint32_t);
    }
    HAL_TIM_Base_Start(Handle);
    (*channel).ccr = timerChCCR(timerHardware);
    (*channel).tim = (*timerHardware).tim;
    ::core::ptr::write_volatile((*channel).ccr, 0 as libc::c_int as timCCR_t);
}
unsafe extern "C" fn pwmWriteUnused(mut index: uint8_t,
                                    mut value: libc::c_float) {
}
unsafe extern "C" fn pwmWriteStandard(mut index: uint8_t,
                                      mut value: libc::c_float) {
    /* TODO: move value to be a number between 0-1 (i.e. percent throttle from mixer) */
    ::core::ptr::write_volatile(motors[index as usize].channel.ccr,
                                lrintf(value *
                                           motors[index as usize].pulseScale +
                                           motors[index as usize].pulseOffset)
                                    as timCCR_t); // MSB first
}
unsafe extern "C" fn pwmWriteDshot(mut index: uint8_t,
                                   mut value: libc::c_float) {
    pwmWriteDshotInt(index,
                     lrintf(value) as
                         uint16_t); // Most significant nibble first
}
unsafe extern "C" fn loadDmaBufferDshot(mut dmaBuffer: *mut uint32_t,
                                        mut stride: libc::c_int,
                                        mut packet: uint16_t) -> uint8_t {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 16 as libc::c_int {
        *dmaBuffer.offset((i * stride) as isize) =
            if packet as libc::c_int & 0x8000 as libc::c_int != 0 {
                14 as libc::c_int
            } else { 7 as libc::c_int } as uint32_t;
        packet = ((packet as libc::c_int) << 1 as libc::c_int) as uint16_t;
        i += 1
    }
    return 18 as libc::c_int as uint8_t;
}
unsafe extern "C" fn loadDmaBufferProshot(mut dmaBuffer: *mut uint32_t,
                                          mut stride: libc::c_int,
                                          mut packet: uint16_t) -> uint8_t {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 4 as libc::c_int {
        *dmaBuffer.offset((i * stride) as isize) =
            (24 as libc::c_int +
                 ((packet as libc::c_int & 0xf000 as libc::c_int) >>
                      12 as libc::c_int) * 3 as libc::c_int) as uint32_t;
        packet = ((packet as libc::c_int) << 4 as libc::c_int) as uint16_t;
        i += 1
        // Shift 4 bits
    }
    return 6 as libc::c_int as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn pwmWriteMotor(mut index: uint8_t,
                                       mut value: libc::c_float) {
    pwmWrite.expect("non-null function pointer")(index, value);
}
#[no_mangle]
pub unsafe extern "C" fn pwmShutdownPulsesForAllMotors(mut motorCount:
                                                           uint8_t) {
    let mut index: libc::c_int = 0 as libc::c_int;
    while index < motorCount as libc::c_int {
        // Set the compare register to 0, which stops the output pulsing if the timer overflows
        if !motors[index as usize].channel.ccr.is_null() {
            ::core::ptr::write_volatile(motors[index as usize].channel.ccr,
                                        0 as libc::c_int as timCCR_t)
        }
        index += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn pwmDisableMotors() {
    pwmShutdownPulsesForAllMotors(8 as libc::c_int as uint8_t);
    pwmMotorsEnabled = 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn pwmEnableMotors() {
    /* check motors can be enabled */
    pwmMotorsEnabled =
        pwmWrite !=
            Some(pwmWriteUnused as
                     unsafe extern "C" fn(_: uint8_t, _: libc::c_float)
                         -> ());
}
#[no_mangle]
pub unsafe extern "C" fn pwmAreMotorsEnabled() -> bool {
    return pwmMotorsEnabled;
}
unsafe extern "C" fn pwmCompleteWriteUnused(mut motorCount: uint8_t) { }
unsafe extern "C" fn pwmCompleteOneshotMotorUpdate(mut motorCount: uint8_t) {
    let mut index: libc::c_int = 0 as libc::c_int;
    while index < motorCount as libc::c_int {
        if motors[index as usize].forceOverflow {
            timerForceOverflow(motors[index as usize].channel.tim);
        }
        // Set the compare register to 0, which stops the output pulsing if the timer overflows before the main loop completes again.
        // This compare register will be set to the output value on the next main loop.
        ::core::ptr::write_volatile(motors[index as usize].channel.ccr,
                                    0 as libc::c_int as timCCR_t);
        index += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn pwmCompleteMotorUpdate(mut motorCount: uint8_t) {
    pwmCompleteWrite.expect("non-null function pointer")(motorCount);
}
#[no_mangle]
pub unsafe extern "C" fn motorDevInit(mut motorConfig:
                                          *const motorDevConfig_t,
                                      mut idlePulse: uint16_t,
                                      mut motorCount: uint8_t) {
    memset(motors.as_mut_ptr() as *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<[pwmOutputPort_t; 8]>() as libc::c_ulong);
    let mut useUnsyncedPwm: bool = (*motorConfig).useUnsyncedPwm != 0;
    let mut sMin: libc::c_float = 0 as libc::c_int as libc::c_float;
    let mut sLen: libc::c_float = 0 as libc::c_int as libc::c_float;
    match (*motorConfig).motorPwmProtocol as libc::c_int {
        2 => { sMin = 42e-6f32; sLen = 42e-6f32 }
        3 => { sMin = 5e-6f32; sLen = 20e-6f32 }
        4 => {
            sMin = 0 as libc::c_int as libc::c_float;
            useUnsyncedPwm = 1 as libc::c_int != 0;
            idlePulse = 0 as libc::c_int as uint16_t
        }
        0 => {
            sMin = 1e-3f32;
            sLen = 1e-3f32;
            useUnsyncedPwm = 1 as libc::c_int != 0;
            idlePulse = 0 as libc::c_int as uint16_t
        }
        9 => {
            pwmWrite =
                Some(pwmWriteDshot as
                         unsafe extern "C" fn(_: uint8_t, _: libc::c_float)
                             -> ());
            loadDmaBuffer =
                Some(loadDmaBufferProshot as
                         unsafe extern "C" fn(_: *mut uint32_t,
                                              _: libc::c_int, _: uint16_t)
                             -> uint8_t);
            pwmCompleteWrite =
                Some(pwmCompleteDshotMotorUpdate as
                         unsafe extern "C" fn(_: uint8_t) -> ());
            isDshot = 1 as libc::c_int != 0
        }
        8 | 7 | 6 | 5 => {
            pwmWrite =
                Some(pwmWriteDshot as
                         unsafe extern "C" fn(_: uint8_t, _: libc::c_float)
                             -> ());
            loadDmaBuffer =
                Some(loadDmaBufferDshot as
                         unsafe extern "C" fn(_: *mut uint32_t,
                                              _: libc::c_int, _: uint16_t)
                             -> uint8_t);
            pwmCompleteWrite =
                Some(pwmCompleteDshotMotorUpdate as
                         unsafe extern "C" fn(_: uint8_t) -> ());
            isDshot = 1 as libc::c_int != 0;
            if (*motorConfig).useBurstDshot != 0 {
                useBurstDshot = 1 as libc::c_int != 0
            }
        }
        1 | _ => { sMin = 125e-6f32; sLen = 125e-6f32 }
    }
    if !isDshot {
        pwmWrite =
            Some(pwmWriteStandard as
                     unsafe extern "C" fn(_: uint8_t, _: libc::c_float)
                         -> ());
        pwmCompleteWrite =
            if useUnsyncedPwm as libc::c_int != 0 {
                Some(pwmCompleteWriteUnused as
                         unsafe extern "C" fn(_: uint8_t) -> ())
            } else {
                Some(pwmCompleteOneshotMotorUpdate as
                         unsafe extern "C" fn(_: uint8_t) -> ())
            }
    }
    let mut motorIndex: libc::c_int = 0 as libc::c_int;
    while motorIndex < 8 as libc::c_int &&
              motorIndex < motorCount as libc::c_int {
        let tag: ioTag_t = (*motorConfig).ioTags[motorIndex as usize];
        let mut timerHardware: *const timerHardware_t = timerGetByTag(tag);
        if timerHardware.is_null() {
            /* not enough motors initialised for the mixer or a break in the motors */
            pwmWrite =
                Some(pwmWriteUnused as
                         unsafe extern "C" fn(_: uint8_t, _: libc::c_float)
                             -> ());
            pwmCompleteWrite =
                Some(pwmCompleteWriteUnused as
                         unsafe extern "C" fn(_: uint8_t) -> ());
            /* TODO: block arming and add reason system cannot arm */
            return
        }
        motors[motorIndex as usize].io = IOGetByTag(tag);
        IOInit(motors[motorIndex as usize].io, OWNER_MOTOR,
               (motorIndex + 1 as libc::c_int) as uint8_t);
        if isDshot {
            pwmDshotMotorHardwareConfig(timerHardware, motorIndex as uint8_t,
                                        (*motorConfig).motorPwmProtocol as
                                            motorPwmProtocolTypes_e,
                                        if (*motorConfig).motorPwmInversion as
                                               libc::c_int != 0 {
                                            ((*timerHardware).output as
                                                 libc::c_int) ^
                                                TIMER_OUTPUT_INVERTED as
                                                    libc::c_int
                                        } else {
                                            (*timerHardware).output as
                                                libc::c_int
                                        } as uint8_t);
            motors[motorIndex as usize].enabled = 1 as libc::c_int != 0
        } else {
            IOConfigGPIOAF(motors[motorIndex as usize].io,
                           (0x2 as libc::c_uint |
                                (0 as libc::c_uint) << 2 as libc::c_int |
                                (0 as libc::c_uint) << 5 as libc::c_int) as
                               ioConfig_t,
                           (*timerHardware).alternateFunction);
            /* standard PWM outputs */
        // margin of safety is 4 periods when unsynced
            let pwmRateHz: libc::c_uint =
                if useUnsyncedPwm as libc::c_int != 0 {
                    (*motorConfig).motorPwmRate as libc::c_int as
                        libc::c_float
                } else {
                    ceilf(1 as libc::c_int as libc::c_float /
                              ((sMin + sLen) *
                                   4 as libc::c_int as libc::c_float))
                } as libc::c_uint;
            let clock: uint32_t = timerClock((*timerHardware).tim);
            /* used to find the desired timer frequency for max resolution */
            let prescaler: libc::c_uint =
                clock.wrapping_div(pwmRateHz).wrapping_add(0xffff as
                                                               libc::c_int as
                                                               libc::c_uint).wrapping_div(0x10000
                                                                                              as
                                                                                              libc::c_int
                                                                                              as
                                                                                              libc::c_uint); /* rounding up */
            let hz: uint32_t = clock.wrapping_div(prescaler);
            let period: libc::c_uint =
                if useUnsyncedPwm as libc::c_int != 0 {
                    hz.wrapping_div(pwmRateHz)
                } else { 0xffff as libc::c_int as libc::c_uint };
            /*
            if brushed then it is the entire length of the period.
            TODO: this can be moved back to periodMin and periodLen
            once mixer outputs a 0..1 float value.
        */
            motors[motorIndex as usize].pulseScale =
                (if (*motorConfig).motorPwmProtocol as libc::c_int ==
                        PWM_TYPE_BRUSHED as libc::c_int {
                     period as libc::c_float
                 } else { (sLen) * hz as libc::c_float }) / 1000.0f32;
            motors[motorIndex as usize].pulseOffset =
                sMin * hz as libc::c_float -
                    motors[motorIndex as usize].pulseScale *
                        1000 as libc::c_int as libc::c_float;
            pwmOutConfig(&mut (*motors.as_mut_ptr().offset(motorIndex as
                                                               isize)).channel,
                         timerHardware, hz, period as uint16_t, idlePulse,
                         (*motorConfig).motorPwmInversion);
            let mut timerAlreadyUsed: bool = 0 as libc::c_int != 0;
            let mut i: libc::c_int = 0 as libc::c_int;
            while i < motorIndex {
                if motors[i as usize].channel.tim ==
                       motors[motorIndex as usize].channel.tim {
                    timerAlreadyUsed = 1 as libc::c_int != 0;
                    break ;
                } else { i += 1 }
            }
            motors[motorIndex as usize].forceOverflow = !timerAlreadyUsed;
            motors[motorIndex as usize].enabled = 1 as libc::c_int != 0
        }
        motorIndex += 1
    }
    pwmMotorsEnabled = 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn pwmGetMotors() -> *mut pwmOutputPort_t {
    return motors.as_mut_ptr();
}
#[no_mangle]
pub unsafe extern "C" fn isMotorProtocolDshot() -> bool { return isDshot; }
#[no_mangle]
pub unsafe extern "C" fn getDshotHz(mut pwmProtocolType:
                                        motorPwmProtocolTypes_e) -> uint32_t {
    match pwmProtocolType as libc::c_uint {
        9 => {
            return (24 as libc::c_int * 1000000 as libc::c_int) as uint32_t
        }
        8 => {
            return (24 as libc::c_int * 1000000 as libc::c_int) as uint32_t
        }
        7 => {
            return (12 as libc::c_int * 1000000 as libc::c_int) as uint32_t
        }
        6 => {
            return (6 as libc::c_int * 1000000 as libc::c_int) as uint32_t
        }
        5 | _ => {
            return (3 as libc::c_int * 1000000 as libc::c_int) as uint32_t
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn allMotorsAreIdle(mut motorCount: uint8_t) -> bool {
    let mut allMotorsIdle: bool = 1 as libc::c_int != 0;
    let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while i < motorCount as libc::c_uint {
        let mut motor: *const motorDmaOutput_t =
            getMotorDmaOutput(i as uint8_t);
        if (*motor).value != 0 { allMotorsIdle = 0 as libc::c_int != 0 }
        i = i.wrapping_add(1)
    }
    return allMotorsIdle;
}
#[no_mangle]
pub unsafe extern "C" fn pwmDshotCommandIsQueued() -> bool {
    return dshotCommandControl.nextCommandAtUs != 0;
}
#[no_mangle]
pub unsafe extern "C" fn pwmDshotCommandIsProcessing() -> bool {
    return dshotCommandControl.nextCommandAtUs != 0 &&
               !dshotCommandControl.waitingForIdle &&
               dshotCommandControl.repeats as libc::c_int > 0 as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn pwmWriteDshotCommand(mut index: uint8_t,
                                              mut motorCount: uint8_t,
                                              mut command: uint8_t,
                                              mut blocking: bool) {
    let mut timeNowUs: timeUs_t = micros();
    if !isMotorProtocolDshot() || command as libc::c_int > 47 as libc::c_int
           || pwmDshotCommandIsQueued() as libc::c_int != 0 {
        return
    }
    let mut repeats: uint8_t = 1 as libc::c_int as uint8_t;
    let mut delayAfterCommandUs: timeUs_t = 1000 as libc::c_int as timeUs_t;
    match command as libc::c_int {
        7 | 8 | 9 | 10 | 12 | 20 | 21 => {
            repeats = 10 as libc::c_int as uint8_t
        }
        1 | 2 | 3 | 4 | 5 => {
            delayAfterCommandUs = 100000 as libc::c_int as timeUs_t
        }
        _ => { }
    }
    if blocking {
        delayMicroseconds((10000 as libc::c_int - 1000 as libc::c_int) as
                              timeUs_t);
        while repeats != 0 {
            delayMicroseconds(1000 as libc::c_int as timeUs_t);
            let mut i: uint8_t = 0 as libc::c_int as uint8_t;
            while (i as libc::c_int) < motorCount as libc::c_int {
                if i as libc::c_int == index as libc::c_int ||
                       index as libc::c_int == 255 as libc::c_int {
                    let motor: *mut motorDmaOutput_t = getMotorDmaOutput(i);
                    ::core::ptr::write_volatile(&mut (*motor).requestTelemetry
                                                    as *mut bool,
                                                1 as libc::c_int != 0);
                    pwmWriteDshotInt(i, command as uint16_t);
                }
                i = i.wrapping_add(1)
            }
            pwmCompleteDshotMotorUpdate(0 as libc::c_int as uint8_t);
            repeats = repeats.wrapping_sub(1)
        }
        delayMicroseconds(delayAfterCommandUs);
    } else {
        dshotCommandControl.repeats = repeats;
        dshotCommandControl.nextCommandAtUs =
            timeNowUs.wrapping_add(10000 as libc::c_int as libc::c_uint);
        dshotCommandControl.delayAfterCommandUs = delayAfterCommandUs;
        let mut i_0: libc::c_uint = 0 as libc::c_int as libc::c_uint;
        while i_0 < motorCount as libc::c_uint {
            if index as libc::c_uint == i_0 ||
                   index as libc::c_int == 255 as libc::c_int {
                dshotCommandControl.command[i_0 as usize] = command
            } else { dshotCommandControl.command[i_0 as usize] = command }
            i_0 = i_0.wrapping_add(1)
        }
        dshotCommandControl.waitingForIdle = !allMotorsAreIdle(motorCount)
    };
}
#[no_mangle]
pub unsafe extern "C" fn pwmGetDshotCommand(mut index: uint8_t) -> uint8_t {
    return dshotCommandControl.command[index as usize];
}
#[no_mangle]
#[inline(never)]
pub unsafe extern "C" fn pwmDshotCommandOutputIsEnabled(mut motorCount:
                                                            uint8_t) -> bool {
    let mut timeNowUs: timeUs_t = micros();
    if dshotCommandControl.waitingForIdle {
        if allMotorsAreIdle(motorCount) {
            dshotCommandControl.nextCommandAtUs =
                timeNowUs.wrapping_add(10000 as libc::c_int as libc::c_uint);
            dshotCommandControl.waitingForIdle = 0 as libc::c_int != 0
        }
        // Send normal motor output while waiting for motors to go idle
        return 1 as libc::c_int != 0
    }
    if cmpTimeUs(timeNowUs, dshotCommandControl.nextCommandAtUs) <
           0 as libc::c_int {
        //Skip motor update because it isn't time yet for a new command
        return 0 as libc::c_int != 0
    }
    //Timed motor update happening with dshot command
    if dshotCommandControl.repeats as libc::c_int > 0 as libc::c_int {
        dshotCommandControl.repeats =
            dshotCommandControl.repeats.wrapping_sub(1); // reset telemetry request to make sure it's triggered only once in a row
        if dshotCommandControl.repeats as libc::c_int > 0 as libc::c_int {
            dshotCommandControl.nextCommandAtUs =
                timeNowUs.wrapping_add(1000 as libc::c_int as libc::c_uint)
        } else {
            dshotCommandControl.nextCommandAtUs =
                timeNowUs.wrapping_add(dshotCommandControl.delayAfterCommandUs)
        }
    } else {
        dshotCommandControl.nextCommandAtUs = 0 as libc::c_int as timeUs_t
    }
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn prepareDshotPacket(motor: *mut motorDmaOutput_t)
 -> uint16_t {
    let mut packet: uint16_t =
        (((*motor).value as libc::c_int) << 1 as libc::c_int |
             (if (*motor).requestTelemetry as libc::c_int != 0 {
                  1 as libc::c_int
              } else { 0 as libc::c_int })) as uint16_t;
    ::core::ptr::write_volatile(&mut (*motor).requestTelemetry as *mut bool,
                                0 as libc::c_int != 0);
    // compute checksum
    let mut csum: libc::c_int = 0 as libc::c_int; // xor data by nibbles
    let mut csum_data: libc::c_int = packet as libc::c_int;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 3 as libc::c_int {
        csum ^= csum_data;
        csum_data >>= 4 as libc::c_int;
        i += 1
    }
    csum &= 0xf as libc::c_int;
    // append checksum
    packet = ((packet as libc::c_int) << 4 as libc::c_int | csum) as uint16_t;
    return packet;
}
#[no_mangle]
pub unsafe extern "C" fn pwmWriteServo(mut index: uint8_t,
                                       mut value: libc::c_float) {
    if (index as libc::c_int) < 8 as libc::c_int &&
           !servos[index as usize].channel.ccr.is_null() {
        ::core::ptr::write_volatile(servos[index as usize].channel.ccr,
                                    lrintf(value) as timCCR_t)
    };
}
#[no_mangle]
pub unsafe extern "C" fn servoDevInit(mut servoConfig:
                                          *const servoDevConfig_t) {
    let mut servoIndex: uint8_t = 0 as libc::c_int as uint8_t;
    while (servoIndex as libc::c_int) < 8 as libc::c_int {
        let tag: ioTag_t = (*servoConfig).ioTags[servoIndex as usize];
        if tag == 0 { break ; }
        servos[servoIndex as usize].io = IOGetByTag(tag);
        IOInit(servos[servoIndex as usize].io, OWNER_SERVO,
               (servoIndex as libc::c_int + 1 as libc::c_int) as uint8_t);
        let mut timer: *const timerHardware_t = timerGetByTag(tag);
        IOConfigGPIOAF(servos[servoIndex as usize].io,
                       (0x2 as libc::c_uint |
                            (0 as libc::c_uint) << 2 as libc::c_int |
                            (0 as libc::c_uint) << 5 as libc::c_int) as
                           ioConfig_t, (*timer).alternateFunction);
        if timer.is_null() { break ; }
        pwmOutConfig(&mut (*servos.as_mut_ptr().offset(servoIndex as
                                                           isize)).channel,
                     timer,
                     (1 as libc::c_int * 1000000 as libc::c_int) as uint32_t,
                     (1 as libc::c_int * 1000000 as libc::c_int /
                          (*servoConfig).servoPwmRate as libc::c_int) as
                         uint16_t, (*servoConfig).servoCenterPulse,
                     0 as libc::c_int as uint8_t);
        servos[servoIndex as usize].enabled = 1 as libc::c_int != 0;
        servoIndex = servoIndex.wrapping_add(1)
    };
}
#[no_mangle]
pub unsafe extern "C" fn pwmWriteBeeper(mut onoffBeep: bool) {
    if beeperPwm.io.is_null() { return }
    if onoffBeep as libc::c_int == 1 as libc::c_int {
        ::core::ptr::write_volatile(beeperPwm.channel.ccr,
                                    (1 as libc::c_int * 1000000 as libc::c_int
                                         / freqBeep as libc::c_int /
                                         2 as libc::c_int) as timCCR_t);
        beeperPwm.enabled = 1 as libc::c_int != 0
    } else {
        ::core::ptr::write_volatile(beeperPwm.channel.ccr,
                                    0 as libc::c_int as timCCR_t);
        beeperPwm.enabled = 0 as libc::c_int != 0
    };
}
#[no_mangle]
pub unsafe extern "C" fn pwmToggleBeeper() {
    pwmWriteBeeper(!beeperPwm.enabled);
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
  DshotSettingRequest (KISS24). Spin direction, 3d and save Settings reqire 10 requests.. and the TLM Byte must always be high if 1-47 are used to send settings

  3D Mode:
  0 = stop
  48   (low) - 1047 (high) -> negative direction
  1048 (low) - 2047 (high) -> positive direction
 */
// V2 includes settings
// Currently not implemented
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// KISS audio Stream mode on/Off
// KISS silent Mode on/Off
// 1uS
// 4uS
/* resolution + frame reset (2us) */
/* resolution + frame reset (2us) */
// function pointer used to write motors
// function pointer used after motors are written
//CAVEAT: This is used in the `motorConfig_t` parameter group, so the parameter group constraints apply
// The update rate of motor outputs (50-498Hz)
// Pwm Protocol
// Active-High vs Active-Low. Useful for brushed FCs converted for brushless operation
// PWM values, in milliseconds, common range is 1000-2000 (1ms to 2ms)
// This is the value for servos when they should be in the middle. e.g. 1500.
// The update rate of servo outputs (50-498Hz)
// function pointer used to encode a digital motor value into the DMA buffer representation
#[no_mangle]
pub unsafe extern "C" fn beeperPwmInit(tag: ioTag_t,
                                       mut frequency: uint16_t) {
    let mut timer: *const timerHardware_t = timerGetByTag(tag);
    let mut beeperIO: IO_t = IOGetByTag(tag);
    if !beeperIO.is_null() && !timer.is_null() {
        beeperPwm.io = beeperIO;
        IOInit(beeperPwm.io, OWNER_BEEPER,
               (0 as libc::c_int + 1 as libc::c_int) as uint8_t);
        IOConfigGPIOAF(beeperPwm.io,
                       (0x2 as libc::c_uint |
                            (0 as libc::c_uint) << 2 as libc::c_int |
                            (0 as libc::c_uint) << 5 as libc::c_int) as
                           ioConfig_t, (*timer).alternateFunction);
        freqBeep = frequency;
        pwmOutConfig(&mut beeperPwm.channel, timer,
                     (1 as libc::c_int * 1000000 as libc::c_int) as uint32_t,
                     (1 as libc::c_int * 1000000 as libc::c_int /
                          freqBeep as libc::c_int) as uint16_t,
                     (1 as libc::c_int * 1000000 as libc::c_int /
                          freqBeep as libc::c_int / 2 as libc::c_int) as
                         uint16_t, 0 as libc::c_int as uint8_t);
        ::core::ptr::write_volatile(beeperPwm.channel.ccr,
                                    0 as libc::c_int as timCCR_t);
        beeperPwm.enabled = 0 as libc::c_int != 0
    };
}
