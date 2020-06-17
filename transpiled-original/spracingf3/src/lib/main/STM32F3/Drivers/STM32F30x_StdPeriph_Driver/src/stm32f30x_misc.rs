use core;
use libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct NVIC_Type {
    pub ISER: [uint32_t; 8],
    pub RESERVED0: [uint32_t; 24],
    pub ICER: [uint32_t; 8],
    pub RSERVED1: [uint32_t; 24],
    pub ISPR: [uint32_t; 8],
    pub RESERVED2: [uint32_t; 24],
    pub ICPR: [uint32_t; 8],
    pub RESERVED3: [uint32_t; 24],
    pub IABR: [uint32_t; 8],
    pub RESERVED4: [uint32_t; 56],
    pub IP: [uint8_t; 240],
    pub RESERVED5: [uint32_t; 644],
    pub STIR: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct SCB_Type {
    pub CPUID: uint32_t,
    pub ICSR: uint32_t,
    pub VTOR: uint32_t,
    pub AIRCR: uint32_t,
    pub SCR: uint32_t,
    pub CCR: uint32_t,
    pub SHP: [uint8_t; 12],
    pub SHCSR: uint32_t,
    pub CFSR: uint32_t,
    pub HFSR: uint32_t,
    pub DFSR: uint32_t,
    pub MMFAR: uint32_t,
    pub BFAR: uint32_t,
    pub AFSR: uint32_t,
    pub PFR: [uint32_t; 2],
    pub DFR: uint32_t,
    pub ADR: uint32_t,
    pub MMFR: [uint32_t; 4],
    pub ISAR: [uint32_t; 5],
    pub RESERVED0: [uint32_t; 5],
    pub CPACR: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct SysTick_Type {
    pub CTRL: uint32_t,
    pub LOAD: uint32_t,
    pub VAL: uint32_t,
    pub CALIB: uint32_t,
}
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct NVIC_InitTypeDef {
    pub NVIC_IRQChannel: uint8_t,
    pub NVIC_IRQChannelPreemptionPriority: uint8_t,
    pub NVIC_IRQChannelSubPriority: uint8_t,
    pub NVIC_IRQChannelCmd: FunctionalState,
}
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @defgroup MISC_Private_Functions
  * @{
  */
/* *
  * @brief  Configures the priority grouping: pre-emption priority and subpriority.
  * @param  NVIC_PriorityGroup: specifies the priority grouping bits length. 
  *   This parameter can be one of the following values:
  *     @arg NVIC_PriorityGroup_0: 0 bits for pre-emption priority.
  *                                4 bits for subpriority.
  *     @arg NVIC_PriorityGroup_1: 1 bits for pre-emption priority.
  *                                3 bits for subpriority.
  *     @arg NVIC_PriorityGroup_2: 2 bits for pre-emption priority.
  *                                2 bits for subpriority.
  *     @arg NVIC_PriorityGroup_3: 3 bits for pre-emption priority.
  *                                1 bits for subpriority.
  *     @arg NVIC_PriorityGroup_4: 4 bits for pre-emption priority.
  *                                0 bits for subpriority.
  *     @note When NVIC_PriorityGroup_0 is selected, it will no be any nested 
  *           interrupt. This interrupts priority is managed only with subpriority.                                    
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn NVIC_PriorityGroupConfig(mut NVIC_PriorityGroup:
                                                      uint32_t) {
    /* Check the parameters */
    /* Set the PRIGROUP[10:8] bits according to NVIC_PriorityGroup value */
    ::core::ptr::write_volatile(&mut (*(0xe000e000u64.wrapping_add(0xd00u64)
                                            as *mut SCB_Type)).AIRCR as
                                    *mut uint32_t,
                                0x5fa0000i32 as uint32_t |
                                    NVIC_PriorityGroup);
}
/* *
  * @brief  Initializes the NVIC peripheral according to the specified
  *         parameters in the NVIC_InitStruct.
  * @note   To configure interrupts priority correctly, the NVIC_PriorityGroupConfig()
  *         function should be called before. 
  * @param  NVIC_InitStruct: pointer to a NVIC_InitTypeDef structure that contains
  *         the configuration information for the specified NVIC peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn NVIC_Init(mut NVIC_InitStruct:
                                       *mut NVIC_InitTypeDef) {
    let mut tmppriority: uint32_t = 0i32 as uint32_t;
    let mut tmppre: uint32_t = 0i32 as uint32_t;
    let mut tmpsub: uint32_t = 0xfi32 as uint32_t;
    /* Check the parameters */
    if (*NVIC_InitStruct).NVIC_IRQChannelCmd as libc::c_uint !=
           DISABLE as libc::c_int as libc::c_uint {
        /* Compute the Corresponding IRQ Priority --------------------------------*/
        tmppriority =
            (0x700i32 as
                 libc::c_uint).wrapping_sub((*(0xe000e000u64.wrapping_add(0xd00u64)
                                                   as *mut SCB_Type)).AIRCR &
                                                0x700i32 as uint32_t) >>
                0x8i32;
        tmppre = (0x4i32 as libc::c_uint).wrapping_sub(tmppriority);
        tmpsub = tmpsub >> tmppriority;
        tmppriority =
            ((*NVIC_InitStruct).NVIC_IRQChannelPreemptionPriority as uint32_t)
                << tmppre;
        tmppriority |=
            (*NVIC_InitStruct).NVIC_IRQChannelSubPriority as libc::c_uint &
                tmpsub;
        tmppriority = tmppriority << 0x4i32;
        ::core::ptr::write_volatile(&mut (*(0xe000e000u64.wrapping_add(0x100u64)
                                                as
                                                *mut NVIC_Type)).IP[(*NVIC_InitStruct).NVIC_IRQChannel
                                                                        as
                                                                        usize]
                                        as *mut uint8_t,
                                    tmppriority as uint8_t);
        /* Enable the Selected IRQ Channels --------------------------------------*/
        ::core::ptr::write_volatile(&mut (*(0xe000e000u64.wrapping_add(0x100u64)
                                                as
                                                *mut NVIC_Type)).ISER[((*NVIC_InitStruct).NVIC_IRQChannel
                                                                           as
                                                                           libc::c_int
                                                                           >>
                                                                           0x5i32)
                                                                          as
                                                                          usize]
                                        as *mut uint32_t,
                                    (0x1i32 as uint32_t) <<
                                        ((*NVIC_InitStruct).NVIC_IRQChannel as
                                             libc::c_int &
                                             0x1fi32 as uint8_t as
                                                 libc::c_int))
    } else {
        /* Disable the Selected IRQ Channels -------------------------------------*/
        ::core::ptr::write_volatile(&mut (*(0xe000e000u64.wrapping_add(0x100u64)
                                                as
                                                *mut NVIC_Type)).ICER[((*NVIC_InitStruct).NVIC_IRQChannel
                                                                           as
                                                                           libc::c_int
                                                                           >>
                                                                           0x5i32)
                                                                          as
                                                                          usize]
                                        as *mut uint32_t,
                                    (0x1i32 as uint32_t) <<
                                        ((*NVIC_InitStruct).NVIC_IRQChannel as
                                             libc::c_int &
                                             0x1fi32 as uint8_t as
                                                 libc::c_int))
    };
}
/* *
  * @brief  Sets the vector table location and Offset.
  * @param  NVIC_VectTab: specifies if the vector table is in RAM or FLASH memory.
  *   This parameter can be one of the following values:
  *     @arg NVIC_VectTab_RAM
  *     @arg NVIC_VectTab_FLASH
  * @param  Offset: Vector Table base offset field. This value must be a multiple of 0x200.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn NVIC_SetVectorTable(mut NVIC_VectTab: uint32_t,
                                             mut Offset: uint32_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*(0xe000e000u64.wrapping_add(0xd00u64)
                                            as *mut SCB_Type)).VTOR as
                                    *mut uint32_t,
                                NVIC_VectTab |
                                    Offset & 0x1fffff80i32 as uint32_t);
}
/* *
  * @brief  Selects the condition for the system to enter low power mode.
  * @param  LowPowerMode: Specifies the new mode for the system to enter low power mode.
  *   This parameter can be one of the following values:
  *     @arg NVIC_LP_SEVONPEND
  *     @arg NVIC_LP_SLEEPDEEP
  *     @arg NVIC_LP_SLEEPONEXIT
  * @param  NewState: new state of LP condition. This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn NVIC_SystemLPConfig(mut LowPowerMode: uint8_t,
                                             mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        let ref mut fresh0 =
            (*(0xe000e000u64.wrapping_add(0xd00u64) as *mut SCB_Type)).SCR;
        ::core::ptr::write_volatile(fresh0,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         LowPowerMode as libc::c_uint) as
                                        uint32_t as uint32_t)
    } else {
        let ref mut fresh1 =
            (*(0xe000e000u64.wrapping_add(0xd00u64) as *mut SCB_Type)).SCR;
        ::core::ptr::write_volatile(fresh1,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(LowPowerMode as uint32_t)) as
                                        uint32_t as uint32_t)
    };
}
/* *
  ******************************************************************************
  * @file    stm32f30x_misc.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the miscellaneous
  *          firmware library functions (add-on to CMSIS functions).
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
/* * @addtogroup MISC
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief  NVIC Init Structure definition  
  */
/* !< Specifies the IRQ channel to be enabled or disabled.
                                                   This parameter can be a value of @ref IRQn_Type (For 
                                                   the complete STM32 Devices IRQ Channels list, please
                                                    refer to stm32f30x.h file) */
/* !< Specifies the pre-emption priority for the IRQ channel
                                                   specified in NVIC_IRQChannel. This parameter can be a value
                                                   between 0 and 15.
                                                   A lower priority value indicates a higher priority */
/* !< Specifies the subpriority level for the IRQ channel specified
                                                   in NVIC_IRQChannel. This parameter can be a value 
                                                   between 0 and 15.
                                                   A lower priority value indicates a higher priority */
/* !< Specifies whether the IRQ channel defined in NVIC_IRQChannel
                                                   will be enabled or disabled. 
                                                   This parameter can be set either to ENABLE or DISABLE */
/* *  
  *
@verbatim   
 The table below gives the allowed values of the pre-emption priority and subpriority according
 to the Priority Grouping configuration performed by NVIC_PriorityGroupConfig function
  ============================================================================================================================
    NVIC_PriorityGroup   | NVIC_IRQChannelPreemptionPriority | NVIC_IRQChannelSubPriority  | Description
  ============================================================================================================================
   NVIC_PriorityGroup_0  |                0                  |            0-15             |   0 bits for pre-emption priority
                         |                                   |                             |   4 bits for subpriority
  ----------------------------------------------------------------------------------------------------------------------------
   NVIC_PriorityGroup_1  |                0-1                |            0-7              |   1 bits for pre-emption priority
                         |                                   |                             |   3 bits for subpriority
  ----------------------------------------------------------------------------------------------------------------------------    
   NVIC_PriorityGroup_2  |                0-3                |            0-3              |   2 bits for pre-emption priority
                         |                                   |                             |   2 bits for subpriority
  ----------------------------------------------------------------------------------------------------------------------------    
   NVIC_PriorityGroup_3  |                0-7                |            0-1              |   3 bits for pre-emption priority
                         |                                   |                             |   1 bits for subpriority
  ----------------------------------------------------------------------------------------------------------------------------    
   NVIC_PriorityGroup_4  |                0-15               |            0                |   4 bits for pre-emption priority
                         |                                   |                             |   0 bits for subpriority                       
  ============================================================================================================================
@endverbatim
*/
/* Exported constants --------------------------------------------------------*/
/* * @defgroup MISC_Exported_Constants
  * @{
  */
/* * @defgroup MISC_Vector_Table_Base 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup MISC_System_Low_Power 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup MISC_Preemption_Priority_Group 
  * @{
  */
/* !< 0 bits for pre-emption priority
                                                            4 bits for subpriority */
/* !< 1 bits for pre-emption priority
                                                            3 bits for subpriority */
/* !< 2 bits for pre-emption priority
                                                            2 bits for subpriority */
/* !< 3 bits for pre-emption priority
                                                            1 bits for subpriority */
/* !< 4 bits for pre-emption priority
                                                            0 bits for subpriority */
/* *
  * @}
  */
/* * @defgroup MISC_SysTick_clock_source 
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* *
  * @brief  Configures the SysTick clock source.
  * @param  SysTick_CLKSource: specifies the SysTick clock source.
  *   This parameter can be one of the following values:
  *     @arg SysTick_CLKSource_HCLK_Div8: AHB clock divided by 8 selected as SysTick clock source.
  *     @arg SysTick_CLKSource_HCLK: AHB clock selected as SysTick clock source.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SysTick_CLKSourceConfig(mut SysTick_CLKSource:
                                                     uint32_t) {
    /* Check the parameters */
    if SysTick_CLKSource == 0x4i32 as uint32_t {
        let ref mut fresh2 =
            (*(0xe000e000u64.wrapping_add(0x10u64) as
                   *mut SysTick_Type)).CTRL;
        ::core::ptr::write_volatile(fresh2,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | 0x4i32 as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        let ref mut fresh3 =
            (*(0xe000e000u64.wrapping_add(0x10u64) as
                   *mut SysTick_Type)).CTRL;
        ::core::ptr::write_volatile(fresh3,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & 0xfffffffbu32) as
                                        uint32_t as uint32_t)
    };
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* *
  * @}
  */
