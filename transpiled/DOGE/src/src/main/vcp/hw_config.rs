use ::libc;
extern "C" {
    /* !< System Clock Frequency (Core Clock) */
    /* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
    /* * @addtogroup STM32F30x_System_Exported_Functions
  * @{
  */
    #[no_mangle]
    fn SystemInit();
    /* Initialization and Configuration functions *********************************/
    #[no_mangle]
    fn EXTI_Init(EXTI_InitStruct: *mut EXTI_InitTypeDef);
    #[no_mangle]
    fn EXTI_ClearITPendingBit(EXTI_Line: uint32_t);
    /* Initialization and Configuration functions *********************************/
    #[no_mangle]
    fn GPIO_Init(GPIOx: *mut GPIO_TypeDef,
                 GPIO_InitStruct: *mut GPIO_InitTypeDef);
    /* GPIO Alternate functions configuration functions ***************************/
    #[no_mangle]
    fn GPIO_PinAFConfig(GPIOx: *mut GPIO_TypeDef, GPIO_PinSource: uint16_t,
                        GPIO_AF: uint8_t);
    #[no_mangle]
    fn RCC_USBCLKConfig(RCC_USBCLKSource: uint32_t);
    #[no_mangle]
    fn RCC_AHBPeriphClockCmd(RCC_AHBPeriph: uint32_t,
                             NewState: FunctionalState);
    #[no_mangle]
    fn RCC_APB2PeriphClockCmd(RCC_APB2Periph: uint32_t,
                              NewState: FunctionalState);
    #[no_mangle]
    fn RCC_APB1PeriphClockCmd(RCC_APB1Periph: uint32_t,
                              NewState: FunctionalState);
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
    #[no_mangle]
    fn NVIC_PriorityGroupConfig(NVIC_PriorityGroup: uint32_t);
    #[no_mangle]
    fn NVIC_Init(NVIC_InitStruct: *mut NVIC_InitTypeDef);
    #[no_mangle]
    fn SetEPRxStatus(_: uint8_t, _: uint16_t);
    #[no_mangle]
    fn SetEPTxValid(_: uint8_t);
    #[no_mangle]
    fn SetEPTxCount(_: uint8_t, _: uint16_t);
    #[no_mangle]
    fn SetEPRxCount(_: uint8_t, _: uint16_t);
    #[no_mangle]
    fn UserToPMABufferCopy(pbUsrBuf: *const uint8_t, wPMABufAddr: uint16_t,
                           wNBytes: uint16_t);
    #[no_mangle]
    static mut Device_Info: DEVICE_INFO;
    #[no_mangle]
    fn Virtual_Com_Port_GetBaudRate() -> uint32_t;
    #[no_mangle]
    static mut Virtual_Com_Port_StringSerial: [uint8_t; 26];
    /* External variables --------------------------------------------------------*/
    #[no_mangle]
    static mut bDeviceState: uint32_t;
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
    fn usbGenerateDisconnectPulse();
    // HJI
    #[no_mangle]
    static mut receiveLength: uint32_t;
}
pub type IRQn = libc::c_int;
pub const FPU_IRQn: IRQn = 81;
pub const USBWakeUp_RMP_IRQn: IRQn = 76;
pub const USB_LP_IRQn: IRQn = 75;
pub const USB_HP_IRQn: IRQn = 74;
pub const COMP7_IRQn: IRQn = 66;
pub const COMP4_5_6_IRQn: IRQn = 65;
pub const COMP1_2_3_IRQn: IRQn = 64;
pub const ADC4_IRQn: IRQn = 61;
pub const DMA2_Channel5_IRQn: IRQn = 60;
pub const DMA2_Channel4_IRQn: IRQn = 59;
pub const DMA2_Channel3_IRQn: IRQn = 58;
pub const DMA2_Channel2_IRQn: IRQn = 57;
pub const DMA2_Channel1_IRQn: IRQn = 56;
pub const TIM7_IRQn: IRQn = 55;
pub const TIM6_DAC_IRQn: IRQn = 54;
pub const UART5_IRQn: IRQn = 53;
pub const UART4_IRQn: IRQn = 52;
pub const SPI3_IRQn: IRQn = 51;
pub const ADC3_IRQn: IRQn = 47;
pub const TIM8_CC_IRQn: IRQn = 46;
pub const TIM8_TRG_COM_IRQn: IRQn = 45;
pub const TIM8_UP_IRQn: IRQn = 44;
pub const TIM8_BRK_IRQn: IRQn = 43;
pub const USBWakeUp_IRQn: IRQn = 42;
pub const RTC_Alarm_IRQn: IRQn = 41;
pub const EXTI15_10_IRQn: IRQn = 40;
pub const USART3_IRQn: IRQn = 39;
pub const USART2_IRQn: IRQn = 38;
pub const USART1_IRQn: IRQn = 37;
pub const SPI2_IRQn: IRQn = 36;
pub const SPI1_IRQn: IRQn = 35;
pub const I2C2_ER_IRQn: IRQn = 34;
pub const I2C2_EV_IRQn: IRQn = 33;
pub const I2C1_ER_IRQn: IRQn = 32;
pub const I2C1_EV_IRQn: IRQn = 31;
pub const TIM4_IRQn: IRQn = 30;
pub const TIM3_IRQn: IRQn = 29;
pub const TIM2_IRQn: IRQn = 28;
pub const TIM1_CC_IRQn: IRQn = 27;
pub const TIM1_TRG_COM_TIM17_IRQn: IRQn = 26;
pub const TIM1_UP_TIM16_IRQn: IRQn = 25;
pub const TIM1_BRK_TIM15_IRQn: IRQn = 24;
pub const EXTI9_5_IRQn: IRQn = 23;
pub const CAN1_SCE_IRQn: IRQn = 22;
pub const CAN1_RX1_IRQn: IRQn = 21;
pub const USB_LP_CAN1_RX0_IRQn: IRQn = 20;
pub const USB_HP_CAN1_TX_IRQn: IRQn = 19;
pub const ADC1_2_IRQn: IRQn = 18;
pub const DMA1_Channel7_IRQn: IRQn = 17;
pub const DMA1_Channel6_IRQn: IRQn = 16;
pub const DMA1_Channel5_IRQn: IRQn = 15;
pub const DMA1_Channel4_IRQn: IRQn = 14;
pub const DMA1_Channel3_IRQn: IRQn = 13;
pub const DMA1_Channel2_IRQn: IRQn = 12;
pub const DMA1_Channel1_IRQn: IRQn = 11;
pub const EXTI4_IRQn: IRQn = 10;
pub const EXTI3_IRQn: IRQn = 9;
pub const EXTI2_TS_IRQn: IRQn = 8;
pub const EXTI1_IRQn: IRQn = 7;
pub const EXTI0_IRQn: IRQn = 6;
pub const RCC_IRQn: IRQn = 5;
pub const FLASH_IRQn: IRQn = 4;
pub const RTC_WKUP_IRQn: IRQn = 3;
pub const TAMPER_STAMP_IRQn: IRQn = 2;
pub const PVD_IRQn: IRQn = 1;
pub const WWDG_IRQn: IRQn = 0;
pub const SysTick_IRQn: IRQn = -1;
pub const PendSV_IRQn: IRQn = -2;
pub const DebugMonitor_IRQn: IRQn = -4;
pub const SVCall_IRQn: IRQn = -5;
pub const UsageFault_IRQn: IRQn = -10;
pub const BusFault_IRQn: IRQn = -11;
pub const MemoryManagement_IRQn: IRQn = -12;
pub const NonMaskableInt_IRQn: IRQn = -14;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
pub type ErrorStatus = libc::c_uint;
pub const SUCCESS: ErrorStatus = 1;
pub const ERROR: ErrorStatus = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_TypeDef {
    pub MODER: uint32_t,
    pub OTYPER: uint16_t,
    pub RESERVED0: uint16_t,
    pub OSPEEDR: uint32_t,
    pub PUPDR: uint32_t,
    pub IDR: uint16_t,
    pub RESERVED1: uint16_t,
    pub ODR: uint16_t,
    pub RESERVED2: uint16_t,
    pub BSRR: uint32_t,
    pub LCKR: uint32_t,
    pub AFR: [uint32_t; 2],
    pub BRR: uint16_t,
    pub RESERVED3: uint16_t,
}
/* *
  ******************************************************************************
  * @file    stm32f30x_exti.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the EXTI 
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
/* * @addtogroup EXTI
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief  EXTI mode enumeration  
  */
pub type EXTIMode_TypeDef = libc::c_uint;
pub const EXTI_Mode_Event: EXTIMode_TypeDef = 4;
pub const EXTI_Mode_Interrupt: EXTIMode_TypeDef = 0;
/* * 
  * @brief  EXTI Trigger enumeration  
  */
pub type EXTITrigger_TypeDef = libc::c_uint;
pub const EXTI_Trigger_Rising_Falling: EXTITrigger_TypeDef = 16;
pub const EXTI_Trigger_Falling: EXTITrigger_TypeDef = 12;
pub const EXTI_Trigger_Rising: EXTITrigger_TypeDef = 8;
/* * 
  * @brief  EXTI Init Structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EXTI_InitTypeDef {
    pub EXTI_Line: uint32_t,
    pub EXTI_Mode: EXTIMode_TypeDef,
    pub EXTI_Trigger: EXTITrigger_TypeDef,
    pub EXTI_LineCmd: FunctionalState,
}
pub type GPIOMode_TypeDef = libc::c_uint;
pub const GPIO_Mode_AN: GPIOMode_TypeDef = 3;
pub const GPIO_Mode_AF: GPIOMode_TypeDef = 2;
pub const GPIO_Mode_OUT: GPIOMode_TypeDef = 1;
pub const GPIO_Mode_IN: GPIOMode_TypeDef = 0;
pub type GPIOOType_TypeDef = libc::c_uint;
pub const GPIO_OType_OD: GPIOOType_TypeDef = 1;
pub const GPIO_OType_PP: GPIOOType_TypeDef = 0;
pub type GPIOSpeed_TypeDef = libc::c_uint;
pub const GPIO_Speed_Level_3: GPIOSpeed_TypeDef = 3;
pub const GPIO_Speed_Level_2: GPIOSpeed_TypeDef = 2;
pub const GPIO_Speed_Level_1: GPIOSpeed_TypeDef = 1;
pub type GPIOPuPd_TypeDef = libc::c_uint;
pub const GPIO_PuPd_DOWN: GPIOPuPd_TypeDef = 2;
pub const GPIO_PuPd_UP: GPIOPuPd_TypeDef = 1;
pub const GPIO_PuPd_NOPULL: GPIOPuPd_TypeDef = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_InitTypeDef {
    pub GPIO_Pin: uint32_t,
    pub GPIO_Mode: GPIOMode_TypeDef,
    pub GPIO_Speed: GPIOSpeed_TypeDef,
    pub GPIO_OType: GPIOOType_TypeDef,
    pub GPIO_PuPd: GPIOPuPd_TypeDef,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct NVIC_InitTypeDef {
    pub NVIC_IRQChannel: uint8_t,
    pub NVIC_IRQChannelPreemptionPriority: uint8_t,
    pub NVIC_IRQChannelSubPriority: uint8_t,
    pub NVIC_IRQChannelCmd: FunctionalState,
}
pub const SUSPENDED: _DEVICE_STATE = 3;
pub const ATTACHED: _DEVICE_STATE = 1;
pub const CONFIGURED: _DEVICE_STATE = 5;
/* *
  ******************************************************************************
  * @file    usb_core.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Standard protocol processing functions prototypes
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
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
/* Exported types ------------------------------------------------------------*/
/* 0 */
/* 1 */
/* 2 */
/* 3 */
/* 4 */
/* 5 */
/* 7 */
/* 8 */
/* 9 */
/* 10 */
/* The state machine states of a control pipe */
/* All the request process routines return a value of this type
   If the return value is not SUCCESS or NOT_READY,
   the software will STALL the correspond endpoint */
/* Process successfully */
/* The process has not been finished, endpoint will be
                         NAK to further request */
/*-*-*-*-*-*-*-*-*-*-* Definitions for endpoint level -*-*-*-*-*-*-*-*-*-*-*-*/
/* When send data out of the device,
   CopyData() is used to get data buffer 'Length' bytes data
   if Length is 0,
    CopyData() returns the total length of the data
    if the request is not supported, returns 0
    (NEW Feature )
     if CopyData() returns -1, the calling routine should not proceed
     further and will resume the SETUP process by the class device
   if Length is not 0,
    CopyData() returns a pointer to indicate the data location
   Usb_wLength is the data remain to be sent,
   Usb_wOffset is the Offset of original data
  When receive data from the host,
   CopyData() is used to get user data buffer which is capable
   of Length bytes data to copy data from the endpoint buffer.
   if Length is 0,
    CopyData() returns the available data length,
   if Length is not 0,
    CopyData() returns user buffer address
   Usb_rLength is the data remain to be received,
   Usb_rPointer is the Offset of data buffer
  */
/*-*-*-*-*-*-*-*-*-*-*-* Definitions for device level -*-*-*-*-*-*-*-*-*-*-*-*/
/* Number of endpoints that are used */
/* Number of configuration available */
pub type DEVICE_INFO = _DEVICE_INFO;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _DEVICE_INFO {
    pub USBbmRequestType: uint8_t,
    pub USBbRequest: uint8_t,
    pub USBwValues: uint16_t_uint8_t,
    pub USBwIndexs: uint16_t_uint8_t,
    pub USBwLengths: uint16_t_uint8_t,
    pub ControlState: uint8_t,
    pub Current_Feature: uint8_t,
    pub Current_Configuration: uint8_t,
    pub Current_Interface: uint8_t,
    pub Current_AlternateSetting: uint8_t,
    pub Ctrl_Info: ENDPOINT_INFO,
}
pub type ENDPOINT_INFO = _ENDPOINT_INFO;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _ENDPOINT_INFO {
    pub Usb_wLength: uint16_t,
    pub Usb_wOffset: uint16_t,
    pub PacketSize: uint16_t,
    pub CopyData: Option<unsafe extern "C" fn(_: uint16_t) -> *mut uint8_t>,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union uint16_t_uint8_t {
    pub w: uint16_t,
    pub bw: BW,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct BW {
    pub bb1: uint8_t,
    pub bb0: uint8_t,
}
pub const UNCONNECTED: _DEVICE_STATE = 0;
pub type _DEVICE_STATE = libc::c_uint;
pub const ADDRESSED: _DEVICE_STATE = 4;
pub const POWERED: _DEVICE_STATE = 2;
/* bmRequestType */
/* bRequest */
/* wValue */
/* wIndex */
/* wLength */
/* of type CONTROL_STATE */
/* Selected configuration */
/* Selected interface of current configuration */
/* Selected Alternate Setting of current
                                     interface*/
/* *
 ******************************************************************************
 * @file    hw_config.c
 * @author  MCD Application Team
 * @version V4.0.0
 * @date    21-January-2013
 * @brief   Hardware Configuration & Setup
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
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
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#[no_mangle]
pub static mut HSEStartUpStatus: ErrorStatus = ERROR;
#[no_mangle]
pub static mut EXTI_InitStructure: EXTI_InitTypeDef =
    EXTI_InitTypeDef{EXTI_Line: 0,
                     EXTI_Mode: EXTI_Mode_Interrupt,
                     EXTI_Trigger: 0 as EXTITrigger_TypeDef,
                     EXTI_LineCmd: DISABLE,};
#[no_mangle]
pub static mut packetSent: uint32_t = 0;
// HJI
#[no_mangle]
pub static mut receiveBuffer: [uint8_t; 64] = [0; 64];
// HJI
#[no_mangle]
pub static mut sendLength: uint32_t = 0;
static mut ctrlLineStateCb:
       Option<unsafe extern "C" fn(_: *mut libc::c_void, _: uint16_t) -> ()> =
    None;
static mut ctrlLineStateCbContext: *mut libc::c_void =
    0 as *const libc::c_void as *mut libc::c_void;
static mut baudRateCb:
       Option<unsafe extern "C" fn(_: *mut libc::c_void, _: uint32_t) -> ()> =
    None;
static mut baudRateCbContext: *mut libc::c_void =
    0 as *const libc::c_void as *mut libc::c_void;
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* ******************************************************************************
 * Function Name  : Set_System
 * Description    : Configures Main system clocks & power
 * Input          : None.
 * Return         : None.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Set_System() {
    let mut GPIO_InitStructure: GPIO_InitTypeDef =
        GPIO_InitTypeDef{GPIO_Pin: 0,
                         GPIO_Mode: GPIO_Mode_IN,
                         GPIO_Speed: 0 as GPIOSpeed_TypeDef,
                         GPIO_OType: GPIO_OType_PP,
                         GPIO_PuPd: GPIO_PuPd_NOPULL,};
    /* STM32L1XX_MD && STM32L1XX_XD */
    /* USB_USE_EXTERNAL_PULLUP */
    /* !< At this stage the microcontroller clock setting is already configured,
     this is done through SystemInit() function which is called from startup
     file (startup_stm32f10x_xx.s) before to branch to application main.
     To reconfigure the default setting of SystemInit() function, refer to
     system_stm32f10x.c file
     */
    /* Enable the SYSCFG module clock */
    RCC_APB2PeriphClockCmd(0x1 as libc::c_int as uint32_t, ENABLE);
    /* STM32L1XX_XD */
    usbGenerateDisconnectPulse();
    /*Set PA11,12 as IN - USB_DM,DP*/
    RCC_AHBPeriphClockCmd(0x20000 as libc::c_int as uint32_t, ENABLE);
    GPIO_InitStructure.GPIO_Pin =
        (0x800 as libc::c_int as uint16_t as libc::c_int |
             0x1000 as libc::c_int as uint16_t as libc::c_int) as uint32_t;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x8000000 as libc::c_int as
                                              libc::c_uint).wrapping_add(0 as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint)
                  as *mut GPIO_TypeDef, &mut GPIO_InitStructure);
    /*SET PA11,12 for USB: USB_DM,DP*/
    GPIO_PinAFConfig((0x40000000 as libc::c_int as
                          uint32_t).wrapping_add(0x8000000 as libc::c_int as
                                                     libc::c_uint).wrapping_add(0
                                                                                    as
                                                                                    libc::c_int
                                                                                    as
                                                                                    libc::c_uint)
                         as *mut GPIO_TypeDef,
                     0xb as libc::c_int as uint8_t as uint16_t,
                     0xe as libc::c_int as uint8_t);
    GPIO_PinAFConfig((0x40000000 as libc::c_int as
                          uint32_t).wrapping_add(0x8000000 as libc::c_int as
                                                     libc::c_uint).wrapping_add(0
                                                                                    as
                                                                                    libc::c_int
                                                                                    as
                                                                                    libc::c_uint)
                         as *mut GPIO_TypeDef,
                     0xc as libc::c_int as uint8_t as uint16_t,
                     0xe as libc::c_int as uint8_t);
    /* STM32F37X  && STM32F303xC)*/
    // Initialise callbacks
    ctrlLineStateCb = None;
    baudRateCb = None;
    /* Configure the EXTI line 18 connected internally to the USB IP */
    EXTI_ClearITPendingBit(0x12 as libc::c_int as uint32_t);
    EXTI_InitStructure.EXTI_Line = 0x12 as libc::c_int as uint32_t;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&mut EXTI_InitStructure);
}
/* ******************************************************************************
 * Function Name  : Set_USBClock
 * Description    : Configures USB Clock input (48MHz)
 * Input          : None.
 * Return         : None.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Set_USBClock() {
    /* Select USBCLK source */
    RCC_USBCLKConfig(0 as libc::c_int as uint8_t as uint32_t);
    /* Enable the USB clock */
    RCC_APB1PeriphClockCmd(0x800000 as libc::c_int as uint32_t, ENABLE);
}
/* ******************************************************************************
 * Function Name  : Enter_LowPowerMode
 * Description    : Power-off system clocks and power while entering suspend mode
 * Input          : None.
 * Return         : None.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Enter_LowPowerMode() {
    /* Set the device state to suspend */
    ::core::ptr::write_volatile(&mut bDeviceState as *mut uint32_t,
                                SUSPENDED as libc::c_int as uint32_t);
}
/* ******************************************************************************
 * Function Name  : Leave_LowPowerMode
 * Description    : Restores system clocks and power while exiting suspend mode
 * Input          : None.
 * Return         : None.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Leave_LowPowerMode() {
    let mut pInfo: *mut DEVICE_INFO = &mut Device_Info;
    /* Set the device state to the correct state */
    if (*pInfo).Current_Configuration as libc::c_int != 0 as libc::c_int {
        /* Device configured */
        ::core::ptr::write_volatile(&mut bDeviceState as *mut uint32_t,
                                    CONFIGURED as libc::c_int as uint32_t)
    } else {
        ::core::ptr::write_volatile(&mut bDeviceState as *mut uint32_t,
                                    ATTACHED as libc::c_int as uint32_t)
    }
    /*Enable SystemCoreClock*/
    SystemInit();
}
/* ******************************************************************************
 * Function Name  : USB_Interrupts_Config
 * Description    : Configures the USB interrupts
 * Input          : None.
 * Return         : None.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn USB_Interrupts_Config() {
    let mut NVIC_InitStructure: NVIC_InitTypeDef =
        NVIC_InitTypeDef{NVIC_IRQChannel: 0,
                         NVIC_IRQChannelPreemptionPriority: 0,
                         NVIC_IRQChannelSubPriority: 0,
                         NVIC_IRQChannelCmd: DISABLE,};
    /* 2 bit for pre-emption priority, 2 bits for subpriority */
    NVIC_PriorityGroupConfig(0x500 as libc::c_int as
                                 uint32_t); // is this really neccesary?
    /* Enable the USB interrupt */
    NVIC_InitStructure.NVIC_IRQChannel =
        USB_LP_CAN1_RX0_IRQn as libc::c_int as uint8_t;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =
        ((((2 as libc::c_int) <<
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
               0 as libc::c_int &
                   0xf as libc::c_int >>
                       (7 as libc::c_int as
                            libc::c_uint).wrapping_sub(0x500 as libc::c_int as
                                                           uint32_t >>
                                                           8 as libc::c_int))
              << 4 as libc::c_int & 0xf0 as libc::c_int) >>
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
             >> 4 as libc::c_int) as uint8_t;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =
        ((((2 as libc::c_int) <<
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
               0 as libc::c_int &
                   0xf as libc::c_int >>
                       (7 as libc::c_int as
                            libc::c_uint).wrapping_sub(0x500 as libc::c_int as
                                                           uint32_t >>
                                                           8 as libc::c_int))
              << 4 as libc::c_int & 0xf0 as libc::c_int &
              0xf as libc::c_int >>
                  (7 as libc::c_int as
                       libc::c_uint).wrapping_sub(0x500 as libc::c_int as
                                                      uint32_t >>
                                                      8 as libc::c_int)) >>
             4 as libc::c_int) as uint8_t;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&mut NVIC_InitStructure);
    /* Enable the USB Wake-up interrupt */
    NVIC_InitStructure.NVIC_IRQChannel =
        USBWakeUp_IRQn as libc::c_int as uint8_t;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =
        ((((1 as libc::c_int) <<
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
               0 as libc::c_int &
                   0xf as libc::c_int >>
                       (7 as libc::c_int as
                            libc::c_uint).wrapping_sub(0x500 as libc::c_int as
                                                           uint32_t >>
                                                           8 as libc::c_int))
              << 4 as libc::c_int & 0xf0 as libc::c_int) >>
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
             >> 4 as libc::c_int) as uint8_t;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =
        ((((1 as libc::c_int) <<
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
               0 as libc::c_int &
                   0xf as libc::c_int >>
                       (7 as libc::c_int as
                            libc::c_uint).wrapping_sub(0x500 as libc::c_int as
                                                           uint32_t >>
                                                           8 as libc::c_int))
              << 4 as libc::c_int & 0xf0 as libc::c_int &
              0xf as libc::c_int >>
                  (7 as libc::c_int as
                       libc::c_uint).wrapping_sub(0x500 as libc::c_int as
                                                      uint32_t >>
                                                      8 as libc::c_int)) >>
             4 as libc::c_int) as uint8_t;
    NVIC_Init(&mut NVIC_InitStructure);
}
/* ******************************************************************************
 * Function Name  : USB_Cable_Config
 * Description    : Software Connection/Disconnection of USB Cable
 * Input          : None.
 * Return         : Status
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn USB_Cable_Config(mut NewState: FunctionalState) { }
/* ******************************************************************************
 * Function Name  : Get_SerialNum.
 * Description    : Create the serial number string descriptor.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Get_SerialNum() {
    let mut Device_Serial0: uint32_t = 0;
    let mut Device_Serial1: uint32_t = 0;
    let mut Device_Serial2: uint32_t = 0;
    Device_Serial0 = *(0x1ffff7ac as libc::c_int as *mut uint32_t);
    Device_Serial1 = *(0x1ffff7b0 as libc::c_int as *mut uint32_t);
    Device_Serial2 = *(0x1ffff7b4 as libc::c_int as *mut uint32_t);
    Device_Serial0 =
        (Device_Serial0 as libc::c_uint).wrapping_add(Device_Serial2) as
            uint32_t as uint32_t;
    if Device_Serial0 != 0 as libc::c_int as libc::c_uint {
        IntToUnicode(Device_Serial0,
                     &mut *Virtual_Com_Port_StringSerial.as_mut_ptr().offset(2
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 isize),
                     8 as libc::c_int as uint8_t);
        IntToUnicode(Device_Serial1,
                     &mut *Virtual_Com_Port_StringSerial.as_mut_ptr().offset(18
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 isize),
                     4 as libc::c_int as uint8_t);
    };
}
// HJI
/* ******************************************************************************
 * Function Name  : HexToChar.
 * Description    : Convert Hex 32Bits value into char.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
unsafe extern "C" fn IntToUnicode(mut value: uint32_t, mut pbuf: *mut uint8_t,
                                  mut len: uint8_t) {
    let mut idx: uint8_t = 0 as libc::c_int as uint8_t;
    idx = 0 as libc::c_int as uint8_t;
    while (idx as libc::c_int) < len as libc::c_int {
        if (value >> 28 as libc::c_int) < 0xa as libc::c_int as libc::c_uint {
            *pbuf.offset((2 as libc::c_int * idx as libc::c_int) as isize) =
                (value >>
                     28 as
                         libc::c_int).wrapping_add('0' as i32 as libc::c_uint)
                    as uint8_t
        } else {
            *pbuf.offset((2 as libc::c_int * idx as libc::c_int) as isize) =
                (value >>
                     28 as
                         libc::c_int).wrapping_add('A' as i32 as
                                                       libc::c_uint).wrapping_sub(10
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                    as uint8_t
        }
        value = value << 4 as libc::c_int;
        *pbuf.offset((2 as libc::c_int * idx as libc::c_int +
                          1 as libc::c_int) as isize) =
            0 as libc::c_int as uint8_t;
        idx = idx.wrapping_add(1)
    };
}
/* *
 ******************************************************************************
 * @file    hw_config.h
 * @author  MCD Application Team
 * @version V4.0.0
 * @date    21-January-2013
 * @brief   Hardware Configuration & Setup
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
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
//#include "platform_config.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* ******************************************************************************
 * Function Name  : Send DATA .
 * Description    : send the data received from the STM32 to the PC through USB
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn CDC_Send_DATA(mut ptrBuffer: *const uint8_t,
                                       mut sendLength_0: uint32_t)
 -> uint32_t {
    /* Last transmission hasn't finished, abort */
    if packetSent != 0 { return 0 as libc::c_int as uint32_t }
    // We can only put 64 bytes in the buffer
    if sendLength_0 > (64 as libc::c_int / 2 as libc::c_int) as libc::c_uint {
        sendLength_0 = (64 as libc::c_int / 2 as libc::c_int) as uint32_t
    }
    // Try to load some bytes if we can
    if sendLength_0 != 0 {
        UserToPMABufferCopy(ptrBuffer, 0xc0 as libc::c_int as uint16_t,
                            sendLength_0 as uint16_t);
        SetEPTxCount(1 as libc::c_int as uint8_t, sendLength_0 as uint16_t);
        ::core::ptr::write_volatile(&mut packetSent as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&packetSent
                                                                                as
                                                                                *const uint32_t)
                                         as
                                         libc::c_uint).wrapping_add(sendLength_0)
                                        as uint32_t as uint32_t);
        SetEPTxValid(1 as libc::c_int as uint8_t);
    }
    return sendLength_0;
}
#[no_mangle]
pub unsafe extern "C" fn CDC_Send_FreeBytes() -> uint32_t {
    /* this driver is blocking, so the buffer is unlimited */
    return 255 as libc::c_int as uint32_t;
}
/* ******************************************************************************
 * Function Name  : Receive DATA .
 * Description    : receive the data from the PC to STM32 and send it through USB
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn CDC_Receive_DATA(mut recvBuf: *mut uint8_t,
                                          mut len: uint32_t) -> uint32_t {
    static mut offset: uint8_t = 0 as libc::c_int as uint8_t;
    let mut i: uint8_t = 0;
    if len > receiveLength { len = receiveLength }
    i = 0 as libc::c_int as uint8_t;
    while (i as libc::c_uint) < len {
        *recvBuf.offset(i as isize) =
            receiveBuffer[(i as libc::c_int + offset as libc::c_int) as
                              usize];
        i = i.wrapping_add(1)
    }
    ::core::ptr::write_volatile(&mut receiveLength as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&receiveLength
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint).wrapping_sub(len) as
                                    uint32_t as uint32_t);
    offset = (offset as libc::c_uint).wrapping_add(len) as uint8_t as uint8_t;
    /* re-enable the rx endpoint which we had set to receive 0 bytes */
    if receiveLength == 0 as libc::c_int as libc::c_uint {
        SetEPRxCount(3 as libc::c_int as uint8_t,
                     64 as libc::c_int as uint16_t);
        SetEPRxStatus(3 as libc::c_int as uint8_t,
                      0x3000 as libc::c_int as uint16_t);
        offset = 0 as libc::c_int as uint8_t
    }
    return len;
}
// HJI
// HJI
#[no_mangle]
pub unsafe extern "C" fn CDC_Receive_BytesAvailable() -> uint32_t {
    return receiveLength;
}
/* ******************************************************************************
 * Function Name  : usbIsConfigured.
 * Description    : Determines if USB VCP is configured or not
 * Input          : None.
 * Output         : None.
 * Return         : True if configured.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn usbIsConfigured() -> uint8_t {
    return (bDeviceState == CONFIGURED as libc::c_int as libc::c_uint) as
               libc::c_int as uint8_t;
}
// HJI
/* ******************************************************************************
 * Function Name  : usbIsConnected.
 * Description    : Determines if USB VCP is connected ot not
 * Input          : None.
 * Output         : None.
 * Return         : True if connected.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn usbIsConnected() -> uint8_t {
    return (bDeviceState != UNCONNECTED as libc::c_int as libc::c_uint) as
               libc::c_int as uint8_t;
}
/* ******************************************************************************
 * Function Name  : CDC_BaudRate.
 * Description    : Get the current baud rate
 * Input          : None.
 * Output         : None.
 * Return         : Baud rate in bps
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn CDC_BaudRate() -> uint32_t {
    return Virtual_Com_Port_GetBaudRate();
}
/* ******************************************************************************
 * Function Name  : CDC_SetBaudRateCb
 * Description    : Set a callback to call when baud rate changes
 * Input          : callback function and context.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn CDC_SetBaudRateCb(mut cb:
                                               Option<unsafe extern "C" fn(_:
                                                                               *mut libc::c_void,
                                                                           _:
                                                                               uint32_t)
                                                          -> ()>,
                                           mut context: *mut libc::c_void) {
    baudRateCbContext = context;
    baudRateCb = cb;
}
// HJI
/* ******************************************************************************
 * Function Name  : CDC_SetCtrlLineStateCb
 * Description    : Set a callback to call when control line state changes
 * Input          : callback function and context.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn CDC_SetCtrlLineStateCb(mut cb:
                                                    Option<unsafe extern "C" fn(_:
                                                                                    *mut libc::c_void,
                                                                                _:
                                                                                    uint16_t)
                                                               -> ()>,
                                                mut context:
                                                    *mut libc::c_void) {
    ctrlLineStateCbContext = context;
    ctrlLineStateCb = cb;
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
