use core;
use libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    /* *
  ******************************************************************************
  * @file    system_stm32f30x.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    28-March-2014
  * @brief   CMSIS Cortex-M4 Device System Source File for STM32F30x devices.
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
    /* * @addtogroup CMSIS
  * @{
  */
    /* * @addtogroup stm32f30x_system
  * @{
  */
    /* *
  * @brief Define to prevent recursive inclusion
  */
    /* Exported types ------------------------------------------------------------*/
    #[no_mangle]
    static mut SystemCoreClock: uint32_t;
    /* Exported constants --------------------------------------------------------*/
    /* * @defgroup TIM_Exported_constants 
  * @{
  */
    /* LIST1: TIM1, TIM2, TIM3, TIM4, TIM8, TIM15, TIM16 and TIM17 */
    /* LIST2: TIM1, TIM2, TIM3, TIM4, TIM8 and TIM15 */
    /* LIST3: TIM1, TIM2, TIM3, TIM4 and TIM8 */
    /* LIST4: TIM1 and TIM8 */
    /* LIST5: TIM1, TIM2, TIM3, TIM4, TIM5, TIM6, TIM7 and TIM8 */
    /* LIST6: TIM1, TIM8, TIM15, TIM16 and TIM17 */
    /* LIST5: TIM1, TIM2, TIM3, TIM4, TIM5, TIM6, TIM7 and TIM8 */
    /* LIST8: TIM16 (option register) */
    /* * @defgroup TIM_Output_Compare_and_PWM_modes 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_One_Pulse_Mode 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Channel 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Clock_Division_CKD 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Counter_Mode 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Output_Compare_Polarity 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Output_Compare_N_Polarity 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Output_Compare_State 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Output_Compare_N_State
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Capture_Compare_State
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Capture_Compare_N_State
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Break_Input_enable_disable 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Break1_Input_enable_disable 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Break2_Input_enable_disable 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Break_Polarity 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Break1_Polarity 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Break2_Polarity 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Break1_Filter 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Break2_Filter 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_AOE_Bit_Set_Reset 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Lock_level
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_OSSI_Off_State_Selection_for_Idle_mode_state 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_OSSR_Off_State_Selection_for_Run_mode_state
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Output_Compare_Idle_State 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Output_Compare_N_Idle_State 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Input_Capture_Polarity 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Input_Capture_Selection 
  * @{
  */
    /* !< TIM Input 1, 2, 3 or 4 is selected to be 
                                                                   connected to IC1, IC2, IC3 or IC4, respectively */
    /* !< TIM Input 1, 2, 3 or 4 is selected to be
                                                                   connected to IC2, IC1, IC4 or IC3, respectively. */
    /* !< TIM Input 1, 2, 3 or 4 is selected to be connected to TRC. */
    /* *
  * @}
  */
    /* * @defgroup TIM_Input_Capture_Prescaler 
  * @{
  */
    /* !< Capture performed each time an edge is detected on the capture input. */
    /* !< Capture performed once every 2 events. */
    /* !< Capture performed once every 4 events. */
    /* !< Capture performed once every 8 events. */
    /* *
  * @}
  */
    /* * @defgroup TIM_interrupt_sources 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_DMA_Base_address 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_DMA_Burst_Length 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_DMA_sources 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_External_Trigger_Prescaler 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Internal_Trigger_Selection 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_TIx_External_Clock_Source 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_External_Trigger_Polarity 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Prescaler_Reload_Mode 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Forced_Action 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Encoder_Mode 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Event_Source 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Update_Source 
  * @{
  */
    /* !< Source of update is the counter overflow/underflow
                                                                   or the setting of UG bit, or an update generation
                                                                   through the slave mode controller. */
    /* !< Source of update is counter overflow/underflow. */
    /* *
  * @}
  */
    /* * @defgroup TIM_Output_Compare_Preload_State 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Output_Compare_Fast_State 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Output_Compare_Clear_State 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Trigger_Output_Source 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Slave_Mode 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Master_Slave_Mode 
  * @{
  */
    /* *
  * @}
  */ 
/* * @defgroup TIM_Remap 
  * @{
  */
    /* *
  * @}
  */ 
/* * @defgroup TIM_Flags 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_OCReferenceClear 
  * @{
  */
    /* * @defgroup TIM_Input_Capture_Filer_Value 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_External_Trigger_Filter 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup TIM_Legacy 
  * @{
  */
    /* *
  * @}
  */
    /* *
  * @}
  */
    /* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
    /* TimeBase management ********************************************************/
    #[no_mangle]
    fn TIM_DeInit(TIMx: *mut TIM_TypeDef);
    #[no_mangle]
    fn TIM_SetCounter(TIMx: *mut TIM_TypeDef, Counter: uint32_t);
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
    fn ledSet(led: libc::c_int, state: bool);
    #[no_mangle]
    static timerHardware: [timerHardware_t; 0];
    #[no_mangle]
    fn timerConfigure(timHw: *const timerHardware_t, period: uint16_t,
                      hz: uint32_t);
    // This interface should be replaced.
    #[no_mangle]
    fn timerChCCHandlerInit(self_0: *mut timerCCHandlerRec_t,
                            fn_0:
                                Option<unsafe extern "C" fn(_:
                                                                *mut timerCCHandlerRec_s,
                                                            _: uint16_t)
                                           -> ()>);
    #[no_mangle]
    fn timerChConfigCallbacks(channel: *const timerHardware_t,
                              edgeCallback: *mut timerCCHandlerRec_t,
                              overflowCallback: *mut timerOvrHandlerRec_t);
    #[no_mangle]
    fn timerChITConfig(timHw: *const timerHardware_t,
                       newState: FunctionalState);
    #[no_mangle]
    fn timerChClearCCFlag(timHw: *const timerHardware_t);
    #[no_mangle]
    fn timerGetByTag(ioTag: ioTag_t) -> *const timerHardware_t;
    #[no_mangle]
    fn timerChConfigIC(timHw: *const timerHardware_t, polarityRising: bool,
                       inputFilterSamples: libc::c_uint);
    #[no_mangle]
    fn pwmGetMotors() -> *mut pwmOutputPort_t;
    #[no_mangle]
    fn pwmDisableMotors();
    #[no_mangle]
    fn serialWrite(instance: *mut serialPort_t, ch: uint8_t);
    #[no_mangle]
    fn serialRxBytesWaiting(instance: *const serialPort_t) -> uint32_t;
    #[no_mangle]
    fn serialRead(instance: *mut serialPort_t) -> uint8_t;
    #[no_mangle]
    fn delay(ms: timeMs_t);
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
/* * 
  * @brief DMA Controller
  */
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct DMA_Channel_TypeDef {
    pub CCR: uint32_t,
    pub CNDTR: uint32_t,
    pub CPAR: uint32_t,
    pub CMAR: uint32_t,
    /* !< DMA channel x memory address register                                          */
}
/* *
  * @brief TIM
  */
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct TIM_TypeDef {
    pub CR1: uint16_t,
    pub RESERVED0: uint16_t,
    pub CR2: uint32_t,
    pub SMCR: uint32_t,
    pub DIER: uint32_t,
    pub SR: uint32_t,
    pub EGR: uint32_t,
    pub CCMR1: uint32_t,
    pub CCMR2: uint32_t,
    pub CCER: uint32_t,
    pub CNT: uint32_t,
    pub PSC: uint16_t,
    pub RESERVED9: uint16_t,
    pub ARR: uint32_t,
    pub RCR: uint16_t,
    pub RESERVED10: uint16_t,
    pub CCR1: uint32_t,
    pub CCR2: uint32_t,
    pub CCR3: uint32_t,
    pub CCR4: uint32_t,
    pub BDTR: uint32_t,
    pub DCR: uint16_t,
    pub RESERVED12: uint16_t,
    pub DMAR: uint16_t,
    pub RESERVED13: uint16_t,
    pub OR: uint16_t,
    pub CCMR3: uint32_t,
    pub CCR5: uint32_t,
    pub CCR6: uint32_t,
    /* !< TIM capture/compare register 4,      Address offset: 0x5C */
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
pub type pgn_t = uint16_t;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed_2 = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed_2 = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed_2 = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed_2 = 4095;
// function that resets a single parameter group instance
pub type pgResetFunc
    =
    unsafe extern "C" fn(_: *mut libc::c_void, _: libc::c_int) -> ();
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct pgRegistry_s {
    pub pgn: pgn_t,
    pub size: uint16_t,
    pub address: *mut uint8_t,
    pub copy: *mut uint8_t,
    pub ptr: *mut *mut uint8_t,
    pub reset: C2RustUnnamed_3,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union C2RustUnnamed_3 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<unsafe extern "C" fn(_: *mut libc::c_void,
                                          _: libc::c_int) -> ()>,
}
pub type pgRegistry_t = pgRegistry_s;
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
/* base */
/* size */
// The parameter group number, the top 4 bits are reserved for version
// Size of the group in RAM, the top 4 bits are reserved for flags
// Address of the group in RAM.
// Address of the copy in RAM.
// The pointer to update after loading the record into ram.
// Pointer to init template
// Pointer to pgResetFunc
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
// use different types from capture and overflow - multiple overflow handlers are implemented as linked list
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct timerCCHandlerRec_s {
    pub fn_0: Option<unsafe extern "C" fn(_: *mut timerCCHandlerRec_s,
                                          _: uint16_t) -> ()>,
}
pub type timerCCHandlerCallback
    =
    unsafe extern "C" fn(_: *mut timerCCHandlerRec_s, _: uint16_t) -> ();
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct timerOvrHandlerRec_s {
    pub fn_0: Option<unsafe extern "C" fn(_: *mut timerOvrHandlerRec_s,
                                          _: uint16_t) -> ()>,
    pub next: *mut timerOvrHandlerRec_s,
}
pub type timerOvrHandlerCallback
    =
    unsafe extern "C" fn(_: *mut timerOvrHandlerRec_s, _: uint16_t) -> ();
pub type timerCCHandlerRec_t = timerCCHandlerRec_s;
pub type timerOvrHandlerRec_t = timerOvrHandlerRec_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct timerHardware_s {
    pub tim: *mut TIM_TypeDef,
    pub tag: ioTag_t,
    pub channel: uint8_t,
    pub usageFlags: timerUsageFlag_e,
    pub output: uint8_t,
    pub alternateFunction: uint8_t,
    pub dmaRef: *mut DMA_Channel_TypeDef,
    pub dmaIrqHandler: uint8_t,
    pub dmaTimUPRef: *mut DMA_Channel_TypeDef,
    pub dmaTimUPIrqHandler: uint8_t,
}
pub type timerHardware_t = timerHardware_s;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const TIMER_OUTPUT_N_CHANNEL: C2RustUnnamed_4 = 2;
pub const TIMER_OUTPUT_INVERTED: C2RustUnnamed_4 = 1;
pub const TIMER_OUTPUT_NONE: C2RustUnnamed_4 = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct timerChannel_t {
    pub ccr: *mut timCCR_t,
    pub tim: *mut TIM_TypeDef,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct pwmOutputPort_t {
    pub channel: timerChannel_t,
    pub pulseScale: libc::c_float,
    pub pulseOffset: libc::c_float,
    pub forceOverflow: bool,
    pub enabled: bool,
    pub io: IO_t,
}
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
// TIMUP
// Define known line control states which may be passed up by underlying serial driver callback
pub type serialReceiveCallbackPtr
    =
    Option<unsafe extern "C" fn(_: uint16_t, _: *mut libc::c_void) -> ()>;
#[derive ( Copy, Clone )]
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
#[derive ( Copy, Clone )]
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
pub type escSerialPortIndex_e = libc::c_uint;
pub const ESCSERIAL2: escSerialPortIndex_e = 1;
pub const ESCSERIAL1: escSerialPortIndex_e = 0;
pub type C2RustUnnamed_5 = libc::c_uint;
pub const PROTOCOL_COUNT: C2RustUnnamed_5 = 5;
pub const PROTOCOL_CASTLE: C2RustUnnamed_5 = 4;
pub const PROTOCOL_KISSALL: C2RustUnnamed_5 = 3;
pub const PROTOCOL_KISS: C2RustUnnamed_5 = 2;
pub const PROTOCOL_BLHELI: C2RustUnnamed_5 = 1;
pub const PROTOCOL_SIMONK: C2RustUnnamed_5 = 0;
// millisecond time
pub type timeMs_t = uint32_t;
pub type escSerial_t = escSerial_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct escSerial_s {
    pub port: serialPort_t,
    pub rxIO: IO_t,
    pub txIO: IO_t,
    pub rxTimerHardware: *const timerHardware_t,
    pub rxBuffer: [uint8_t; 1024],
    pub txTimerHardware: *const timerHardware_t,
    pub txBuffer: [uint8_t; 1024],
    pub isSearchingForStartBit: uint8_t,
    pub rxBitIndex: uint8_t,
    pub rxLastLeadingEdgeAtBitIndex: uint8_t,
    pub rxEdge: uint8_t,
    pub isTransmittingData: uint8_t,
    pub isReceivingData: uint8_t,
    pub bitsLeftToTransmit: int8_t,
    pub internalTxBuffer: uint16_t,
    pub internalRxBuffer: uint16_t,
    pub receiveTimeout: uint16_t,
    pub transmissionErrors: uint16_t,
    pub receiveErrors: uint16_t,
    pub escSerialPortIndex: uint8_t,
    pub mode: uint8_t,
    pub outputCount: uint8_t,
    pub timerCb: timerCCHandlerRec_t,
    pub edgeCb: timerCCHandlerRec_t,
}
pub type mspState_e = libc::c_uint;
pub const COMMAND_RECEIVED: mspState_e = 6;
pub const HEADER_CMD: mspState_e = 5;
pub const HEADER_SIZE: mspState_e = 4;
pub const HEADER_ARROW: mspState_e = 3;
pub const HEADER_M: mspState_e = 2;
pub const HEADER_START: mspState_e = 1;
pub const IDLE: mspState_e = 0;
pub type mspPort_t = mspPort_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct mspPort_s {
    pub offset: uint8_t,
    pub dataSize: uint8_t,
    pub checksum: uint8_t,
    pub indRX: uint8_t,
    pub inBuf: [uint8_t; 10],
    pub c_state: mspState_e,
    pub cmdMSP: uint8_t,
}
pub const TRAILING: C2RustUnnamed_7 = 0;
pub const LEADING: C2RustUnnamed_7 = 1;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct escOutputs_t {
    pub io: IO_t,
    pub inverted: uint8_t,
}
pub type escSerialConfig_t = escSerialConfig_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct escSerialConfig_s {
    pub ioTag: ioTag_t,
}
pub const BAUDRATE_NORMAL: C2RustUnnamed_6 = 19200;
pub const BAUDRATE_CASTLE: C2RustUnnamed_6 = 18880;
pub const BAUDRATE_KISS: C2RustUnnamed_6 = 38400;
pub type C2RustUnnamed_6 = libc::c_uint;
pub type C2RustUnnamed_7 = libc::c_uint;
#[inline]
unsafe extern "C" fn escSerialConfig() -> *const escSerialConfig_t {
    return &mut escSerialConfig_System;
}
static mut escPort: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
static mut passPort: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
#[no_mangle]
pub static mut escOutputs: [escOutputs_t; 8] =
    [escOutputs_t{io: 0 as *const libc::c_void as *mut libc::c_void,
                  inverted: 0,}; 8];
#[no_mangle]
pub static mut escSerialPorts: [escSerial_t; 1] =
    [escSerial_t{port:
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
                 rxTimerHardware: 0 as *const timerHardware_t,
                 rxBuffer: [0; 1024],
                 txTimerHardware: 0 as *const timerHardware_t,
                 txBuffer: [0; 1024],
                 isSearchingForStartBit: 0,
                 rxBitIndex: 0,
                 rxLastLeadingEdgeAtBitIndex: 0,
                 rxEdge: 0,
                 isTransmittingData: 0,
                 isReceivingData: 0,
                 bitsLeftToTransmit: 0,
                 internalTxBuffer: 0,
                 internalRxBuffer: 0,
                 receiveTimeout: 0,
                 transmissionErrors: 0,
                 receiveErrors: 0,
                 escSerialPortIndex: 0,
                 mode: 0,
                 outputCount: 0,
                 timerCb: timerCCHandlerRec_t{fn_0: None,},
                 edgeCb: timerCCHandlerRec_t{fn_0: None,},}; 1];
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut escSerialConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn: (521i32 | 0i32 << 12i32) as pgn_t,
                             size:
                                 (::core::mem::size_of::<escSerialConfig_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &escSerialConfig_System as
                                     *const escSerialConfig_t as
                                     *mut escSerialConfig_t as *mut uint8_t,
                             copy:
                                 &escSerialConfig_Copy as
                                     *const escSerialConfig_t as
                                     *mut escSerialConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_3{ptr:
                                                     &pgResetTemplate_escSerialConfig
                                                         as
                                                         *const escSerialConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut escSerialConfig_System: escSerialConfig_t =
    escSerialConfig_t{ioTag: 0,};
#[no_mangle]
pub static mut escSerialConfig_Copy: escSerialConfig_t =
    escSerialConfig_t{ioTag: 0,};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_escSerialConfig: escSerialConfig_t =
    {
        let mut init =
            escSerialConfig_s{ioTag:
                                  (0i32 + 1i32 << 4i32 | 0i32) as ioTag_t,};
        init
    };
// includes start and stop bits
// includes start and stop bits
// XXX No TIM_DeInit equivalent in HAL driver???
unsafe extern "C" fn setTxSignalEsc(mut escSerial: *mut escSerial_t,
                                    mut state: uint8_t) {
    if (*escSerial).mode as libc::c_int == PROTOCOL_KISSALL as libc::c_int {
        let mut i: uint8_t = 0i32 as uint8_t;
        while (i as libc::c_int) < (*escSerial).outputCount as libc::c_int {
            let mut state_temp: uint8_t = state;
            if escOutputs[i as usize].inverted != 0 {
                state_temp =
                    (state_temp as libc::c_int ^ ENABLE as libc::c_int) as
                        uint8_t
            }
            if state_temp != 0 {
                IOHi(escOutputs[i as usize].io);
            } else { IOLo(escOutputs[i as usize].io); }
            ::core::ptr::write_volatile(&mut i as *mut uint8_t,
                                        ::core::ptr::read_volatile::<uint8_t>(&i
                                                                                  as
                                                                                  *const uint8_t).wrapping_add(1))
        }
    } else {
        if (*(*escSerial).rxTimerHardware).output as libc::c_int &
               TIMER_OUTPUT_INVERTED as libc::c_int != 0 {
            state = (state as libc::c_int ^ ENABLE as libc::c_int) as uint8_t
        }
        if state != 0 {
            IOHi((*escSerial).txIO);
        } else { IOLo((*escSerial).txIO); }
    };
}
unsafe extern "C" fn escSerialGPIOConfig(mut timhw: *const timerHardware_t,
                                         mut cfg: ioConfig_t) {
    let mut tag: ioTag_t = (*timhw).tag;
    if tag == 0 { return }
    IOInit(IOGetByTag(tag), OWNER_MOTOR, 0i32 as uint8_t);
    IOConfigGPIO(IOGetByTag(tag), cfg);
}
unsafe extern "C" fn escSerialInputPortConfig(mut timerHardwarePtr:
                                                  *const timerHardware_t) {
    escSerialGPIOConfig(timerHardwarePtr,
                        (GPIO_Mode_AF as libc::c_int | 0i32 << 2i32 |
                             (GPIO_OType_PP as libc::c_int) << 4i32 |
                             (GPIO_PuPd_UP as libc::c_int) << 5i32) as
                            ioConfig_t);
    timerChClearCCFlag(timerHardwarePtr);
    timerChITConfig(timerHardwarePtr, ENABLE);
}
unsafe extern "C" fn isTimerPeriodTooLarge(mut timerPeriod: uint32_t)
 -> bool {
    return timerPeriod > 0xffffi32 as libc::c_uint;
}
unsafe extern "C" fn isEscSerialTransmitBufferEmpty(mut instance:
                                                        *const serialPort_t)
 -> bool {
    // start listening
    return (*instance).txBufferHead == (*instance).txBufferTail;
}
unsafe extern "C" fn escSerialOutputPortConfig(mut timerHardwarePtr:
                                                   *const timerHardware_t) {
    escSerialGPIOConfig(timerHardwarePtr,
                        (GPIO_Mode_OUT as libc::c_int | 0i32 << 2i32 |
                             (GPIO_OType_PP as libc::c_int) << 4i32 |
                             (GPIO_PuPd_NOPULL as libc::c_int) << 5i32) as
                            ioConfig_t);
    timerChITConfig(timerHardwarePtr, DISABLE);
}
unsafe extern "C" fn processTxStateBL(mut escSerial: *mut escSerial_t) {
    let mut mask: uint8_t = 0;
    if (*escSerial).isReceivingData != 0 { return }
    if (*escSerial).isTransmittingData == 0 {
        let mut byteToSend: libc::c_char = 0;
        if isEscSerialTransmitBufferEmpty(escSerial as *mut serialPort_t) {
            // canreceive
            return
        }
        // data to send
        let fresh0 = (*escSerial).port.txBufferTail;
        (*escSerial).port.txBufferTail =
            (*escSerial).port.txBufferTail.wrapping_add(1);
        byteToSend =
            *(*escSerial).port.txBuffer.offset(fresh0 as isize) as
                libc::c_char;
        if (*escSerial).port.txBufferTail >= (*escSerial).port.txBufferSize {
            (*escSerial).port.txBufferTail = 0i32 as uint32_t
        }
        // build internal buffer, MSB = Stop Bit (1) + data bits (MSB to LSB) + start bit(0) LSB
        (*escSerial).internalTxBuffer =
            (1i32 << 10i32 - 1i32 | (byteToSend as libc::c_int) << 1i32) as
                uint16_t;
        (*escSerial).bitsLeftToTransmit = 10i32 as int8_t;
        (*escSerial).isTransmittingData = 1i32 as uint8_t;
        //set output
        if (*escSerial).mode as libc::c_int == PROTOCOL_BLHELI as libc::c_int
               ||
               (*escSerial).mode as libc::c_int ==
                   PROTOCOL_CASTLE as libc::c_int {
            escSerialOutputPortConfig((*escSerial).rxTimerHardware);
        }
        return
    }
    if (*escSerial).bitsLeftToTransmit != 0 {
        mask =
            ((*escSerial).internalTxBuffer as libc::c_int & 1i32) as uint8_t;
        (*escSerial).internalTxBuffer =
            ((*escSerial).internalTxBuffer as libc::c_int >> 1i32) as
                uint16_t;
        setTxSignalEsc(escSerial, mask);
        (*escSerial).bitsLeftToTransmit -= 1;
        return
    }
    (*escSerial).isTransmittingData = 0i32 as uint8_t;
    if isEscSerialTransmitBufferEmpty(escSerial as *mut serialPort_t) {
        if (*escSerial).mode as libc::c_int == PROTOCOL_BLHELI as libc::c_int
               ||
               (*escSerial).mode as libc::c_int ==
                   PROTOCOL_CASTLE as libc::c_int {
            escSerialInputPortConfig((*escSerial).rxTimerHardware);
        }
    };
}
unsafe extern "C" fn extractAndStoreRxByteBL(mut escSerial:
                                                 *mut escSerial_t) {
    if (*escSerial).port.mode as libc::c_uint &
           MODE_RX as libc::c_int as libc::c_uint == 0i32 as libc::c_uint {
        return
    }
    let mut haveStartBit: uint8_t =
        ((*escSerial).internalRxBuffer as libc::c_int & 1i32 << 10i32 - 1i32
             == 0i32) as libc::c_int as uint8_t;
    let mut haveStopBit: uint8_t =
        ((*escSerial).internalRxBuffer as libc::c_int & 1i32 << 0i32 == 1i32)
            as libc::c_int as uint8_t;
    if haveStartBit == 0 || haveStopBit == 0 {
        (*escSerial).receiveErrors =
            (*escSerial).receiveErrors.wrapping_add(1);
        return
    }
    let mut rxByte: uint8_t =
        ((*escSerial).internalRxBuffer as libc::c_int >> 1i32 & 0xffi32) as
            uint8_t;
    if (*escSerial).port.rxCallback.is_some() {
        (*escSerial).port.rxCallback.expect("non-null function pointer")(rxByte
                                                                             as
                                                                             uint16_t,
                                                                         (*escSerial).port.rxCallbackData);
    } else {
        ::core::ptr::write_volatile((*escSerial).port.rxBuffer.offset((*escSerial).port.rxBufferHead
                                                                          as
                                                                          isize),
                                    rxByte);
        (*escSerial).port.rxBufferHead =
            (*escSerial).port.rxBufferHead.wrapping_add(1i32 as
                                                            libc::c_uint).wrapping_rem((*escSerial).port.rxBufferSize)
    };
}
unsafe extern "C" fn prepareForNextRxByteBL(mut escSerial: *mut escSerial_t) {
    // prepare for next byte
    (*escSerial).rxBitIndex = 0i32 as uint8_t;
    (*escSerial).isSearchingForStartBit = 1i32 as uint8_t;
    if (*escSerial).rxEdge as libc::c_int == LEADING as libc::c_int {
        (*escSerial).rxEdge = TRAILING as libc::c_int as uint8_t;
        timerChConfigIC((*escSerial).rxTimerHardware,
                        if (*escSerial).port.options as libc::c_uint &
                               SERIAL_INVERTED as libc::c_int as libc::c_uint
                               != 0 {
                            1i32
                        } else { 0i32 } != 0, 0i32 as libc::c_uint);
    };
}
unsafe extern "C" fn applyChangedBitsBL(mut escSerial: *mut escSerial_t) {
    if (*escSerial).rxEdge as libc::c_int == TRAILING as libc::c_int {
        let mut bitToSet: uint8_t = 0;
        bitToSet = (*escSerial).rxLastLeadingEdgeAtBitIndex;
        while (bitToSet as libc::c_int) <
                  (*escSerial).rxBitIndex as libc::c_int {
            (*escSerial).internalRxBuffer =
                ((*escSerial).internalRxBuffer as libc::c_int |
                     1i32 << bitToSet as libc::c_int) as uint16_t;
            bitToSet = bitToSet.wrapping_add(1)
        }
    };
}
unsafe extern "C" fn processRxStateBL(mut escSerial: *mut escSerial_t) {
    if (*escSerial).isSearchingForStartBit != 0 { return }
    (*escSerial).rxBitIndex = (*escSerial).rxBitIndex.wrapping_add(1);
    if (*escSerial).rxBitIndex as libc::c_int == 10i32 - 1i32 {
        applyChangedBitsBL(escSerial);
        return
    }
    if (*escSerial).rxBitIndex as libc::c_int == 10i32 {
        if (*escSerial).rxEdge as libc::c_int == TRAILING as libc::c_int {
            (*escSerial).internalRxBuffer =
                ((*escSerial).internalRxBuffer as libc::c_int | 1i32 << 0i32)
                    as uint16_t
        }
        extractAndStoreRxByteBL(escSerial);
        prepareForNextRxByteBL(escSerial);
    };
}
unsafe extern "C" fn onSerialTimerBL(mut cbRec: *mut timerCCHandlerRec_t,
                                     mut capture: captureCompare_t) {
    let mut escSerial: *mut escSerial_t =
        ({
             let mut __mptr: *const timerCCHandlerRec_t = cbRec;
             (__mptr as *mut libc::c_char).offset(-2192) as *mut escSerial_t
         });
    processTxStateBL(escSerial);
    processRxStateBL(escSerial);
}
unsafe extern "C" fn serialTimerTxConfigBL(mut timerHardwarePtr:
                                               *const timerHardware_t,
                                           mut reference: uint8_t,
                                           mut baud: uint32_t) {
    let mut clock: uint32_t =
        SystemCoreClock.wrapping_div(2i32 as libc::c_uint);
    let mut timerPeriod: uint32_t = 0;
    TIM_DeInit((*timerHardwarePtr).tim);
    loop  {
        timerPeriod = clock.wrapping_div(baud);
        if isTimerPeriodTooLarge(timerPeriod) {
            if clock > 1i32 as libc::c_uint {
                clock = clock.wrapping_div(2i32 as libc::c_uint)
                // this is wrong - mhz stays the same ... This will double baudrate until ok (but minimum baudrate is < 1200)
            }
        }
        if !isTimerPeriodTooLarge(timerPeriod) { break ; }
    }
    timerConfigure(timerHardwarePtr, timerPeriod as uint16_t, clock);
    timerChCCHandlerInit(&mut (*escSerialPorts.as_mut_ptr().offset(reference
                                                                       as
                                                                       isize)).timerCb,
                         Some(onSerialTimerBL as
                                  unsafe extern "C" fn(_:
                                                           *mut timerCCHandlerRec_t,
                                                       _: captureCompare_t)
                                      -> ()));
    timerChConfigCallbacks(timerHardwarePtr,
                           &mut (*escSerialPorts.as_mut_ptr().offset(reference
                                                                         as
                                                                         isize)).timerCb,
                           0 as *mut timerOvrHandlerRec_t);
}
unsafe extern "C" fn onSerialRxPinChangeBL(mut cbRec:
                                               *mut timerCCHandlerRec_t,
                                           mut capture: captureCompare_t) {
    let mut escSerial: *mut escSerial_t =
        ({
             let mut __mptr: *const timerCCHandlerRec_t = cbRec;
             (__mptr as *mut libc::c_char).offset(-2200) as *mut escSerial_t
         });
    let mut inverted: bool =
        (*escSerial).port.options as libc::c_uint &
            SERIAL_INVERTED as libc::c_int as libc::c_uint != 0;
    if (*escSerial).port.mode as libc::c_uint &
           MODE_RX as libc::c_int as libc::c_uint == 0i32 as libc::c_uint {
        return
    }
    if (*escSerial).isSearchingForStartBit != 0 {
        // Adjust the timing so it will interrupt on the middle.
        // This is clobbers transmission, but it is okay because we are
        // always half-duplex.
        TIM_SetCounter((*(*escSerial).txTimerHardware).tim,
                       (*(*(*escSerial).txTimerHardware).tim).ARR.wrapping_div(2i32
                                                                                   as
                                                                                   libc::c_uint));
        if (*escSerial).isTransmittingData != 0 {
            (*escSerial).transmissionErrors =
                (*escSerial).transmissionErrors.wrapping_add(1)
        }
        timerChConfigIC((*escSerial).rxTimerHardware,
                        if inverted as libc::c_int != 0 { 0i32 } else { 1i32 }
                            != 0, 0i32 as libc::c_uint);
        (*escSerial).rxEdge = LEADING as libc::c_int as uint8_t;
        (*escSerial).rxBitIndex = 0i32 as uint8_t;
        (*escSerial).rxLastLeadingEdgeAtBitIndex = 0i32 as uint8_t;
        (*escSerial).internalRxBuffer = 0i32 as uint16_t;
        (*escSerial).isSearchingForStartBit = 0i32 as uint8_t;
        return
    }
    if (*escSerial).rxEdge as libc::c_int == LEADING as libc::c_int {
        (*escSerial).rxLastLeadingEdgeAtBitIndex = (*escSerial).rxBitIndex
    }
    applyChangedBitsBL(escSerial);
    if (*escSerial).rxEdge as libc::c_int == TRAILING as libc::c_int {
        (*escSerial).rxEdge = LEADING as libc::c_int as uint8_t;
        timerChConfigIC((*escSerial).rxTimerHardware,
                        if inverted as libc::c_int != 0 { 0i32 } else { 1i32 }
                            != 0, 0i32 as libc::c_uint);
    } else {
        (*escSerial).rxEdge = TRAILING as libc::c_int as uint8_t;
        timerChConfigIC((*escSerial).rxTimerHardware,
                        if inverted as libc::c_int != 0 { 1i32 } else { 0i32 }
                            != 0, 0i32 as libc::c_uint);
    };
}
unsafe extern "C" fn serialTimerRxConfigBL(mut timerHardwarePtr:
                                               *const timerHardware_t,
                                           mut reference: uint8_t,
                                           mut options: portOptions_e) {
    // start bit is usually a FALLING signal
    TIM_DeInit((*timerHardwarePtr).tim);
    timerConfigure(timerHardwarePtr, 0xffffi32 as uint16_t,
                   SystemCoreClock.wrapping_div(2i32 as libc::c_uint));
    timerChConfigIC(timerHardwarePtr,
                    if options as libc::c_uint &
                           SERIAL_INVERTED as libc::c_int as libc::c_uint != 0
                       {
                        1i32
                    } else { 0i32 } != 0, 0i32 as libc::c_uint);
    timerChCCHandlerInit(&mut (*escSerialPorts.as_mut_ptr().offset(reference
                                                                       as
                                                                       isize)).edgeCb,
                         Some(onSerialRxPinChangeBL as
                                  unsafe extern "C" fn(_:
                                                           *mut timerCCHandlerRec_t,
                                                       _: captureCompare_t)
                                      -> ()));
    timerChConfigCallbacks(timerHardwarePtr,
                           &mut (*escSerialPorts.as_mut_ptr().offset(reference
                                                                         as
                                                                         isize)).edgeCb,
                           0 as *mut timerOvrHandlerRec_t);
}
unsafe extern "C" fn processTxStateEsc(mut escSerial: *mut escSerial_t) {
    let mut byteToSend: libc::c_char = 0;
    let mut current_block: u64;
    let mut mask: uint8_t = 0;
    static mut bitq: uint8_t = 0i32 as uint8_t;
    static mut transmitStart: uint8_t = 0i32 as uint8_t;
    if (*escSerial).isReceivingData != 0 { return }
    if transmitStart as libc::c_int == 0i32 {
        setTxSignalEsc(escSerial, 1i32 as uint8_t);
    }
    if (*escSerial).isTransmittingData == 0 {
        byteToSend = 0;
    } else if (*escSerial).bitsLeftToTransmit != 0 {
        mask =
            ((*escSerial).internalTxBuffer as libc::c_int & 1i32) as uint8_t;
        if mask != 0 {
            if bitq as libc::c_int == 0i32 || bitq as libc::c_int == 1i32 {
                setTxSignalEsc(escSerial, 1i32 as uint8_t);
            }
            if bitq as libc::c_int == 2i32 || bitq as libc::c_int == 3i32 {
                setTxSignalEsc(escSerial, 0i32 as uint8_t);
            }
        } else {
            if bitq as libc::c_int == 0i32 || bitq as libc::c_int == 2i32 {
                setTxSignalEsc(escSerial, 1i32 as uint8_t);
            }
            if bitq as libc::c_int == 1i32 || bitq as libc::c_int == 3i32 {
                setTxSignalEsc(escSerial, 0i32 as uint8_t);
            }
        }
        bitq = bitq.wrapping_add(1);
        if bitq as libc::c_int > 3i32 {
            (*escSerial).internalTxBuffer =
                ((*escSerial).internalTxBuffer as libc::c_int >> 1i32) as
                    uint16_t;
            (*escSerial).bitsLeftToTransmit -= 1;
            bitq = 0i32 as uint8_t;
            if (*escSerial).bitsLeftToTransmit as libc::c_int == 0i32 {
                current_block = 3711197927947612708;
            } else { current_block = 11793792312832361944; }
        } else { current_block = 11793792312832361944; }
        match current_block { 3711197927947612708 => { } _ => { return } }
    } else {
        if isEscSerialTransmitBufferEmpty(escSerial as *mut serialPort_t) {
            (*escSerial).isTransmittingData = 0i32 as uint8_t;
            escSerialInputPortConfig((*escSerial).rxTimerHardware);
        }
        return;
    }
    if isEscSerialTransmitBufferEmpty(escSerial as *mut serialPort_t) {
        // canreceive
        transmitStart = 0i32 as uint8_t;
        return
    }
    if (transmitStart as libc::c_int) < 3i32 {
        if transmitStart as libc::c_int == 0i32 {
            byteToSend = 0xffi32 as libc::c_char
        }
        if transmitStart as libc::c_int == 1i32 {
            byteToSend = 0xffi32 as libc::c_char
        }
        if transmitStart as libc::c_int == 2i32 {
            byteToSend = 0x7fi32 as libc::c_char
        }
        transmitStart = transmitStart.wrapping_add(1)
    } else {
        // data to send
        let fresh1 = (*escSerial).port.txBufferTail;
        (*escSerial).port.txBufferTail =
            (*escSerial).port.txBufferTail.wrapping_add(1);
        byteToSend =
            *(*escSerial).port.txBuffer.offset(fresh1 as isize) as
                libc::c_char;
        if (*escSerial).port.txBufferTail >= (*escSerial).port.txBufferSize {
            (*escSerial).port.txBufferTail = 0i32 as uint32_t
        }
    }
    // build internal buffer, data bits (MSB to LSB)
    (*escSerial).internalTxBuffer = byteToSend as uint16_t;
    (*escSerial).bitsLeftToTransmit = 8i32 as int8_t;
    (*escSerial).isTransmittingData = 1i32 as uint8_t;
    //set output
    escSerialOutputPortConfig((*escSerial).rxTimerHardware);
}
unsafe extern "C" fn onSerialTimerEsc(mut cbRec: *mut timerCCHandlerRec_t,
                                      mut capture: captureCompare_t) {
    let mut escSerial: *mut escSerial_t =
        ({
             let mut __mptr: *const timerCCHandlerRec_t = cbRec;
             (__mptr as *mut libc::c_char).offset(-2192) as *mut escSerial_t
         });
    if (*escSerial).isReceivingData != 0 {
        (*escSerial).receiveTimeout =
            (*escSerial).receiveTimeout.wrapping_add(1);
        if (*escSerial).receiveTimeout as libc::c_int > 8i32 {
            (*escSerial).isReceivingData = 0i32 as uint8_t;
            (*escSerial).receiveTimeout = 0i32 as uint16_t;
            timerChConfigIC((*escSerial).rxTimerHardware, 0i32 != 0,
                            0i32 as libc::c_uint);
        }
    }
    processTxStateEsc(escSerial);
}
unsafe extern "C" fn escSerialTimerTxConfig(mut timerHardwarePtr:
                                                *const timerHardware_t,
                                            mut reference: uint8_t) {
    let mut timerPeriod: uint32_t = 34i32 as uint32_t;
    TIM_DeInit((*timerHardwarePtr).tim);
    timerConfigure(timerHardwarePtr, timerPeriod as uint16_t,
                   (1i32 * 1000000i32) as uint32_t);
    timerChCCHandlerInit(&mut (*escSerialPorts.as_mut_ptr().offset(reference
                                                                       as
                                                                       isize)).timerCb,
                         Some(onSerialTimerEsc as
                                  unsafe extern "C" fn(_:
                                                           *mut timerCCHandlerRec_t,
                                                       _: captureCompare_t)
                                      -> ()));
    timerChConfigCallbacks(timerHardwarePtr,
                           &mut (*escSerialPorts.as_mut_ptr().offset(reference
                                                                         as
                                                                         isize)).timerCb,
                           0 as *mut timerOvrHandlerRec_t);
}
unsafe extern "C" fn extractAndStoreRxByteEsc(mut escSerial:
                                                  *mut escSerial_t) {
    if (*escSerial).port.mode as libc::c_uint &
           MODE_RX as libc::c_int as libc::c_uint == 0i32 as libc::c_uint {
        return
    }
    let mut rxByte: uint8_t =
        ((*escSerial).internalRxBuffer as libc::c_int & 0xffi32) as uint8_t;
    if (*escSerial).port.rxCallback.is_some() {
        (*escSerial).port.rxCallback.expect("non-null function pointer")(rxByte
                                                                             as
                                                                             uint16_t,
                                                                         (*escSerial).port.rxCallbackData);
    } else {
        ::core::ptr::write_volatile((*escSerial).port.rxBuffer.offset((*escSerial).port.rxBufferHead
                                                                          as
                                                                          isize),
                                    rxByte);
        (*escSerial).port.rxBufferHead =
            (*escSerial).port.rxBufferHead.wrapping_add(1i32 as
                                                            libc::c_uint).wrapping_rem((*escSerial).port.rxBufferSize)
    };
}
unsafe extern "C" fn onSerialRxPinChangeEsc(mut cbRec:
                                                *mut timerCCHandlerRec_t,
                                            mut capture: captureCompare_t) {
    static mut zerofirst: uint8_t = 0i32 as uint8_t;
    static mut bits: uint8_t = 0i32 as uint8_t;
    static mut bytes: uint16_t = 0i32 as uint16_t;
    let mut escSerial: *mut escSerial_t =
        ({
             let mut __mptr: *const timerCCHandlerRec_t = cbRec;
             (__mptr as *mut libc::c_char).offset(-2200) as *mut escSerial_t
         });
    //clear timer
    TIM_SetCounter((*(*escSerial).rxTimerHardware).tim, 0i32 as uint32_t);
    if capture as libc::c_int > 40i32 && (capture as libc::c_int) < 90i32 {
        zerofirst = zerofirst.wrapping_add(1);
        if zerofirst as libc::c_int > 1i32 {
            zerofirst = 0i32 as uint8_t;
            (*escSerial).internalRxBuffer =
                ((*escSerial).internalRxBuffer as libc::c_int >> 1i32) as
                    uint16_t;
            bits = bits.wrapping_add(1)
        }
    } else if capture as libc::c_int > 90i32 &&
                  (capture as libc::c_int) < 200i32 {
        zerofirst = 0i32 as uint8_t;
        (*escSerial).internalRxBuffer =
            ((*escSerial).internalRxBuffer as libc::c_int >> 1i32) as
                uint16_t;
        (*escSerial).internalRxBuffer =
            ((*escSerial).internalRxBuffer as libc::c_int | 0x80i32) as
                uint16_t;
        bits = bits.wrapping_add(1)
    } else if (*escSerial).isReceivingData == 0 {
        //start
            //lets reset
        (*escSerial).isReceivingData = 1i32 as uint8_t;
        zerofirst = 0i32 as uint8_t;
        bytes = 0i32 as uint16_t;
        bits = 1i32 as uint8_t;
        (*escSerial).internalRxBuffer = 0x80i32 as uint16_t;
        timerChConfigIC((*escSerial).rxTimerHardware, 1i32 != 0,
                        0i32 as libc::c_uint);
    }
    (*escSerial).receiveTimeout = 0i32 as uint16_t;
    if bits as libc::c_int == 8i32 {
        bits = 0i32 as uint8_t;
        bytes = bytes.wrapping_add(1);
        if bytes as libc::c_int > 3i32 {
            extractAndStoreRxByteEsc(escSerial);
        }
        (*escSerial).internalRxBuffer = 0i32 as uint16_t
    };
}
unsafe extern "C" fn escSerialTimerRxConfig(mut timerHardwarePtr:
                                                *const timerHardware_t,
                                            mut reference: uint8_t) {
    // start bit is usually a FALLING signal
    TIM_DeInit((*timerHardwarePtr).tim);
    timerConfigure(timerHardwarePtr, 0xffffi32 as uint16_t,
                   (1i32 * 1000000i32) as uint32_t);
    timerChConfigIC(timerHardwarePtr, 0i32 != 0, 0i32 as libc::c_uint);
    timerChCCHandlerInit(&mut (*escSerialPorts.as_mut_ptr().offset(reference
                                                                       as
                                                                       isize)).edgeCb,
                         Some(onSerialRxPinChangeEsc as
                                  unsafe extern "C" fn(_:
                                                           *mut timerCCHandlerRec_t,
                                                       _: captureCompare_t)
                                      -> ()));
    timerChConfigCallbacks(timerHardwarePtr,
                           &mut (*escSerialPorts.as_mut_ptr().offset(reference
                                                                         as
                                                                         isize)).edgeCb,
                           0 as *mut timerOvrHandlerRec_t);
}
unsafe extern "C" fn resetBuffers(mut escSerial: *mut escSerial_t) {
    (*escSerial).port.rxBufferSize = 1024i32 as uint32_t;
    (*escSerial).port.rxBuffer = (*escSerial).rxBuffer.as_mut_ptr();
    (*escSerial).port.rxBufferTail = 0i32 as uint32_t;
    (*escSerial).port.rxBufferHead = 0i32 as uint32_t;
    (*escSerial).port.txBuffer = (*escSerial).txBuffer.as_mut_ptr();
    (*escSerial).port.txBufferSize = 1024i32 as uint32_t;
    (*escSerial).port.txBufferTail = 0i32 as uint32_t;
    (*escSerial).port.txBufferHead = 0i32 as uint32_t;
}
unsafe extern "C" fn openEscSerial(mut portIndex: escSerialPortIndex_e,
                                   mut callback: serialReceiveCallbackPtr,
                                   mut output: uint16_t, mut baud: uint32_t,
                                   mut options: portOptions_e,
                                   mut mode: uint8_t) -> *mut serialPort_t {
    let mut escSerial: *mut escSerial_t =
        &mut *escSerialPorts.as_mut_ptr().offset(portIndex as isize) as
            *mut escSerial_t;
    if mode as libc::c_int != PROTOCOL_KISSALL as libc::c_int {
        (*escSerial).rxTimerHardware =
            &*timerHardware.as_ptr().offset(output as isize) as
                *const timerHardware_t;
        // N-Channels can't be used as RX.
        if (*(*escSerial).rxTimerHardware).output as libc::c_int &
               TIMER_OUTPUT_N_CHANNEL as libc::c_int != 0 {
            return 0 as *mut serialPort_t
        }
    } // rx is the pin used
    (*escSerial).mode = mode;
    (*escSerial).txTimerHardware = timerGetByTag((*escSerialConfig()).ioTag);
    (*escSerial).port.vTable = escSerialVTable.as_ptr();
    (*escSerial).port.baudRate = baud;
    (*escSerial).port.mode = MODE_RXTX;
    (*escSerial).port.options = options;
    (*escSerial).port.rxCallback = callback;
    resetBuffers(escSerial);
    (*escSerial).isTransmittingData = 0i32 as uint8_t;
    (*escSerial).isSearchingForStartBit = 1i32 as uint8_t;
    (*escSerial).rxBitIndex = 0i32 as uint8_t;
    (*escSerial).transmissionErrors = 0i32 as uint16_t;
    (*escSerial).receiveErrors = 0i32 as uint16_t;
    (*escSerial).receiveTimeout = 0i32 as uint16_t;
    (*escSerial).escSerialPortIndex = portIndex as uint8_t;
    if mode as libc::c_int != PROTOCOL_KISSALL as libc::c_int {
        (*escSerial).txIO = IOGetByTag((*(*escSerial).rxTimerHardware).tag);
        escSerialInputPortConfig((*escSerial).rxTimerHardware);
        setTxSignalEsc(escSerial, ENABLE as libc::c_int as uint8_t);
    }
    delay(50i32 as timeMs_t);
    if mode as libc::c_int == PROTOCOL_SIMONK as libc::c_int {
        escSerialTimerTxConfig((*escSerial).txTimerHardware,
                               portIndex as uint8_t);
        escSerialTimerRxConfig((*escSerial).rxTimerHardware,
                               portIndex as uint8_t);
    } else if mode as libc::c_int == PROTOCOL_BLHELI as libc::c_int {
        serialTimerTxConfigBL((*escSerial).txTimerHardware,
                              portIndex as uint8_t, baud);
        serialTimerRxConfigBL((*escSerial).rxTimerHardware,
                              portIndex as uint8_t, options);
    } else if mode as libc::c_int == PROTOCOL_KISS as libc::c_int {
        escSerialOutputPortConfig((*escSerial).rxTimerHardware);
        serialTimerTxConfigBL((*escSerial).txTimerHardware,
                              portIndex as uint8_t, baud);
    } else if mode as libc::c_int == PROTOCOL_KISSALL as libc::c_int {
        (*escSerial).outputCount = 0i32 as uint8_t;
        memset(&mut escOutputs as *mut [escOutputs_t; 8] as *mut libc::c_void,
               0i32,
               ::core::mem::size_of::<[escOutputs_t; 8]>() as libc::c_ulong);
        let mut pwmMotors: *mut pwmOutputPort_t = pwmGetMotors();
        let mut i: uint8_t = 0i32 as uint8_t;
        while (i as libc::c_int) < 8i32 {
            if (*pwmMotors.offset(i as isize)).enabled {
                if !(*pwmMotors.offset(i as isize)).io.is_null() {
                    let mut j: uint8_t = 0i32 as uint8_t;
                    while (j as libc::c_int) < 17i32 {
                        if (*pwmMotors.offset(i as isize)).io ==
                               IOGetByTag((*timerHardware.as_ptr().offset(j as
                                                                              isize)).tag)
                           {
                            escSerialOutputPortConfig(&*timerHardware.as_ptr().offset(j
                                                                                          as
                                                                                          isize));
                            if (*timerHardware.as_ptr().offset(j as
                                                                   isize)).output
                                   as libc::c_int &
                                   TIMER_OUTPUT_INVERTED as libc::c_int != 0 {
                                escOutputs[(*escSerial).outputCount as
                                               usize].inverted =
                                    1i32 as uint8_t
                            }
                            break ;
                        } else {
                            ::core::ptr::write_volatile(&mut j as
                                                            *mut uint8_t,
                                                        ::core::ptr::read_volatile::<uint8_t>(&j
                                                                                                  as
                                                                                                  *const uint8_t).wrapping_add(1))
                        }
                    }
                    escOutputs[(*escSerial).outputCount as usize].io =
                        (*pwmMotors.offset(i as isize)).io;
                    (*escSerial).outputCount =
                        (*escSerial).outputCount.wrapping_add(1)
                }
            }
            ::core::ptr::write_volatile(&mut i as *mut uint8_t,
                                        ::core::ptr::read_volatile::<uint8_t>(&i
                                                                                  as
                                                                                  *const uint8_t).wrapping_add(1))
        }
        setTxSignalEsc(escSerial, ENABLE as libc::c_int as uint8_t);
        serialTimerTxConfigBL((*escSerial).txTimerHardware,
                              portIndex as uint8_t, baud);
    } else if mode as libc::c_int == PROTOCOL_CASTLE as libc::c_int {
        escSerialOutputPortConfig((*escSerial).rxTimerHardware);
        serialTimerTxConfigBL((*escSerial).txTimerHardware,
                              portIndex as uint8_t, baud);
        serialTimerRxConfigBL((*escSerial).rxTimerHardware,
                              portIndex as uint8_t, options);
    }
    return &mut (*escSerial).port;
}
unsafe extern "C" fn escSerialInputPortDeConfig(mut timerHardwarePtr:
                                                    *const timerHardware_t) {
    timerChClearCCFlag(timerHardwarePtr);
    timerChITConfig(timerHardwarePtr, DISABLE);
    escSerialGPIOConfig(timerHardwarePtr,
                        (GPIO_Mode_IN as libc::c_int | 0i32 << 2i32 |
                             0i32 << 4i32 |
                             (GPIO_PuPd_UP as libc::c_int) << 5i32) as
                            ioConfig_t);
}
unsafe extern "C" fn closeEscSerial(mut portIndex: escSerialPortIndex_e,
                                    mut mode: uint8_t) {
    let mut escSerial: *mut escSerial_t =
        &mut *escSerialPorts.as_mut_ptr().offset(portIndex as isize) as
            *mut escSerial_t;
    if mode as libc::c_int != PROTOCOL_KISSALL as libc::c_int {
        escSerialInputPortDeConfig((*escSerial).rxTimerHardware);
        timerChConfigCallbacks((*escSerial).rxTimerHardware,
                               0 as *mut timerCCHandlerRec_t,
                               0 as *mut timerOvrHandlerRec_t);
        TIM_DeInit((*(*escSerial).rxTimerHardware).tim);
    }
    timerChConfigCallbacks((*escSerial).txTimerHardware,
                           0 as *mut timerCCHandlerRec_t,
                           0 as *mut timerOvrHandlerRec_t);
    TIM_DeInit((*(*escSerial).txTimerHardware).tim);
}
unsafe extern "C" fn escSerialTotalBytesWaiting(mut instance:
                                                    *const serialPort_t)
 -> uint32_t {
    if (*instance).mode as libc::c_uint &
           MODE_RX as libc::c_int as libc::c_uint == 0i32 as libc::c_uint {
        return 0i32 as uint32_t
    }
    let mut s: *mut escSerial_t = instance as *mut escSerial_t;
    return (*s).port.rxBufferHead.wrapping_sub((*s).port.rxBufferTail) &
               (*s).port.rxBufferSize.wrapping_sub(1i32 as libc::c_uint);
}
unsafe extern "C" fn escSerialReadByte(mut instance: *mut serialPort_t)
 -> uint8_t {
    let mut ch: uint8_t = 0;
    if (*instance).mode as libc::c_uint &
           MODE_RX as libc::c_int as libc::c_uint == 0i32 as libc::c_uint {
        return 0i32 as uint8_t
    }
    if escSerialTotalBytesWaiting(instance) == 0i32 as libc::c_uint {
        return 0i32 as uint8_t
    }
    ch = *(*instance).rxBuffer.offset((*instance).rxBufferTail as isize);
    (*instance).rxBufferTail =
        (*instance).rxBufferTail.wrapping_add(1i32 as
                                                  libc::c_uint).wrapping_rem((*instance).rxBufferSize);
    return ch;
}
unsafe extern "C" fn escSerialWriteByte(mut s: *mut serialPort_t,
                                        mut ch: uint8_t) {
    if (*s).mode as libc::c_uint & MODE_TX as libc::c_int as libc::c_uint ==
           0i32 as libc::c_uint {
        return
    }
    ::core::ptr::write_volatile((*s).txBuffer.offset((*s).txBufferHead as
                                                         isize), ch);
    (*s).txBufferHead =
        (*s).txBufferHead.wrapping_add(1i32 as
                                           libc::c_uint).wrapping_rem((*s).txBufferSize);
}
unsafe extern "C" fn escSerialSetBaudRate(mut s: *mut serialPort_t,
                                          mut baudRate: uint32_t) {
}
unsafe extern "C" fn escSerialSetMode(mut instance: *mut serialPort_t,
                                      mut mode: portMode_e) {
    (*instance).mode = mode;
}
unsafe extern "C" fn escSerialTxBytesFree(mut instance: *const serialPort_t)
 -> uint32_t {
    if (*instance).mode as libc::c_uint &
           MODE_TX as libc::c_int as libc::c_uint == 0i32 as libc::c_uint {
        return 0i32 as uint32_t
    }
    let mut s: *mut escSerial_t = instance as *mut escSerial_t;
    let mut bytesUsed: uint8_t =
        ((*s).port.txBufferHead.wrapping_sub((*s).port.txBufferTail) &
             (*s).port.txBufferSize.wrapping_sub(1i32 as libc::c_uint)) as
            uint8_t;
    return (*s).port.txBufferSize.wrapping_sub(1i32 as
                                                   libc::c_uint).wrapping_sub(bytesUsed
                                                                                  as
                                                                                  libc::c_uint);
}
#[no_mangle]
pub static mut escSerialVTable: [serialPortVTable; 1] =
    unsafe {
        [{
             let mut init =
                 serialPortVTable{serialWrite:
                                      Some(escSerialWriteByte as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t,
                                                                    _:
                                                                        uint8_t)
                                                   -> ()),
                                  serialTotalRxWaiting:
                                      Some(escSerialTotalBytesWaiting as
                                               unsafe extern "C" fn(_:
                                                                        *const serialPort_t)
                                                   -> uint32_t),
                                  serialTotalTxFree:
                                      Some(escSerialTxBytesFree as
                                               unsafe extern "C" fn(_:
                                                                        *const serialPort_t)
                                                   -> uint32_t),
                                  serialRead:
                                      Some(escSerialReadByte as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t)
                                                   -> uint8_t),
                                  serialSetBaudRate:
                                      Some(escSerialSetBaudRate as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t,
                                                                    _:
                                                                        uint32_t)
                                                   -> ()),
                                  isSerialTransmitBufferEmpty:
                                      Some(isEscSerialTransmitBufferEmpty as
                                               unsafe extern "C" fn(_:
                                                                        *const serialPort_t)
                                                   -> bool),
                                  setMode:
                                      Some(escSerialSetMode as
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
         }]
    };
static mut currentPort: mspPort_t =
    mspPort_t{offset: 0,
              dataSize: 0,
              checksum: 0,
              indRX: 0,
              inBuf: [0; 10],
              c_state: IDLE,
              cmdMSP: 0,};
unsafe extern "C" fn processExitCommand(mut c: uint8_t) -> bool {
    if currentPort.c_state as libc::c_uint ==
           IDLE as libc::c_int as libc::c_uint {
        if c as libc::c_int == '$' as i32 {
            currentPort.c_state = HEADER_START
        } else { return 0i32 != 0 }
    } else if currentPort.c_state as libc::c_uint ==
                  HEADER_START as libc::c_int as libc::c_uint {
        currentPort.c_state =
            if c as libc::c_int == 'M' as i32 {
                HEADER_M as libc::c_int
            } else { IDLE as libc::c_int } as mspState_e
    } else if currentPort.c_state as libc::c_uint ==
                  HEADER_M as libc::c_int as libc::c_uint {
        currentPort.c_state =
            if c as libc::c_int == '<' as i32 {
                HEADER_ARROW as libc::c_int
            } else { IDLE as libc::c_int } as mspState_e
    } else if currentPort.c_state as libc::c_uint ==
                  HEADER_ARROW as libc::c_int as libc::c_uint {
        if c as libc::c_int > 10i32 {
            currentPort.c_state = IDLE
        } else {
            currentPort.dataSize = c;
            currentPort.offset = 0i32 as uint8_t;
            currentPort.checksum = 0i32 as uint8_t;
            currentPort.indRX = 0i32 as uint8_t;
            currentPort.checksum =
                (currentPort.checksum as libc::c_int ^ c as libc::c_int) as
                    uint8_t;
            currentPort.c_state = HEADER_SIZE
        }
    } else if currentPort.c_state as libc::c_uint ==
                  HEADER_SIZE as libc::c_int as libc::c_uint {
        currentPort.cmdMSP = c;
        currentPort.checksum =
            (currentPort.checksum as libc::c_int ^ c as libc::c_int) as
                uint8_t;
        currentPort.c_state = HEADER_CMD
    } else if currentPort.c_state as libc::c_uint ==
                  HEADER_CMD as libc::c_int as libc::c_uint &&
                  (currentPort.offset as libc::c_int) <
                      currentPort.dataSize as libc::c_int {
        currentPort.checksum =
            (currentPort.checksum as libc::c_int ^ c as libc::c_int) as
                uint8_t;
        let fresh2 = currentPort.offset;
        currentPort.offset = currentPort.offset.wrapping_add(1);
        currentPort.inBuf[fresh2 as usize] = c
    } else if currentPort.c_state as libc::c_uint ==
                  HEADER_CMD as libc::c_int as libc::c_uint &&
                  currentPort.offset as libc::c_int >=
                      currentPort.dataSize as libc::c_int {
        if currentPort.checksum as libc::c_int == c as libc::c_int {
            currentPort.c_state = COMMAND_RECEIVED;
            if currentPort.cmdMSP as libc::c_int == 0xf4i32 &&
                   currentPort.dataSize as libc::c_int == 0i32 {
                currentPort.c_state = IDLE;
                return 1i32 != 0
            }
        } else { currentPort.c_state = IDLE }
    }
    return 0i32 != 0;
}
// serialPort API
#[no_mangle]
pub unsafe extern "C" fn escEnablePassthrough(mut escPassthroughPort:
                                                  *mut serialPort_t,
                                              mut output: uint16_t,
                                              mut mode: uint8_t) {
    let mut exitEsc: bool = 0i32 != 0;
    let mut motor_output: uint8_t = 0i32 as uint8_t;
    ledSet(0i32, 0i32 != 0);
    ledSet(1i32, 0i32 != 0);
    //StopPwmAllMotors();
    pwmDisableMotors();
    passPort = escPassthroughPort;
    let mut escBaudrate: uint32_t = 0;
    match mode as libc::c_int {
        2 => { escBaudrate = BAUDRATE_KISS as libc::c_int as uint32_t }
        4 => { escBaudrate = BAUDRATE_CASTLE as libc::c_int as uint32_t }
        _ => { escBaudrate = BAUDRATE_NORMAL as libc::c_int as uint32_t }
    }
    if mode as libc::c_int == PROTOCOL_KISS as libc::c_int &&
           output as libc::c_int == 255i32 {
        motor_output = 255i32 as uint8_t;
        mode = PROTOCOL_KISSALL as libc::c_int as uint8_t
    } else {
        let mut first_output: uint8_t = 0i32 as uint8_t;
        let mut i: libc::c_uint = 0i32 as libc::c_uint;
        while i < 17i32 as libc::c_uint {
            if (*timerHardware.as_ptr().offset(i as isize)).usageFlags as
                   libc::c_uint & TIM_USE_MOTOR as libc::c_int as libc::c_uint
                   != 0 {
                first_output = i as uint8_t;
                break ;
            } else { i = i.wrapping_add(1) }
        }
        //doesn't work with messy timertable
        motor_output =
            (first_output as libc::c_int + output as libc::c_int) as uint8_t;
        if motor_output as libc::c_int >= 17i32 { return }
    }
    escPort =
        openEscSerial(ESCSERIAL1, None, motor_output as uint16_t, escBaudrate,
                      SERIAL_NOT_INVERTED, mode);
    if escPort.is_null() { return }
    let mut ch: uint8_t = 0;
    loop  {
        if mode as libc::c_int != 2i32 {
            if serialRxBytesWaiting(escPort) != 0 {
                ledSet(0i32, 1i32 != 0);
                while serialRxBytesWaiting(escPort) != 0 {
                    ch = serialRead(escPort);
                    serialWrite(escPassthroughPort, ch);
                }
                ledSet(0i32, 0i32 != 0);
            }
        }
        if serialRxBytesWaiting(escPassthroughPort) != 0 {
            ledSet(1i32, 1i32 != 0);
            while serialRxBytesWaiting(escPassthroughPort) != 0 {
                ch = serialRead(escPassthroughPort);
                exitEsc = processExitCommand(ch);
                if exitEsc {
                    serialWrite(escPassthroughPort, 0x24i32 as uint8_t);
                    serialWrite(escPassthroughPort, 0x4di32 as uint8_t);
                    serialWrite(escPassthroughPort, 0x3ei32 as uint8_t);
                    serialWrite(escPassthroughPort, 0i32 as uint8_t);
                    serialWrite(escPassthroughPort, 0xf4i32 as uint8_t);
                    serialWrite(escPassthroughPort, 0xf4i32 as uint8_t);
                    closeEscSerial(ESCSERIAL1, mode);
                    return
                }
                if mode as libc::c_int == PROTOCOL_BLHELI as libc::c_int {
                    serialWrite(escPassthroughPort, ch);
                    // blheli loopback
                }
                serialWrite(escPort, ch);
            }
            ledSet(1i32, 0i32 != 0);
        }
        if mode as libc::c_int != PROTOCOL_CASTLE as libc::c_int {
            delay(5i32 as timeMs_t);
        }
    };
}
