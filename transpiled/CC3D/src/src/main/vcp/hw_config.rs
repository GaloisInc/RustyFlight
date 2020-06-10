use ::libc;
extern "C" {
    /* !< System Clock Frequency (Core Clock) */
    /* *
  * @}
  */
    /* * @addtogroup STM32F10x_System_Exported_Constants
  * @{
  */
    /* *
  * @}
  */
    /* * @addtogroup STM32F10x_System_Exported_Macros
  * @{
  */
    /* *
  * @}
  */
    /* * @addtogroup STM32F10x_System_Exported_Functions
  * @{
  */
    #[no_mangle]
    fn SystemInit();
    #[no_mangle]
    fn EXTI_Init(EXTI_InitStruct: *mut EXTI_InitTypeDef);
    #[no_mangle]
    fn EXTI_ClearITPendingBit(EXTI_Line: uint32_t);
    #[no_mangle]
    fn GPIO_Init(GPIOx: *mut GPIO_TypeDef,
                 GPIO_InitStruct: *mut GPIO_InitTypeDef);
    #[no_mangle]
    fn RCC_USBCLKConfig(RCC_USBCLKSource: uint32_t);
    #[no_mangle]
    fn RCC_APB2PeriphClockCmd(RCC_APB2Periph: uint32_t,
                              NewState: FunctionalState);
    #[no_mangle]
    fn RCC_APB1PeriphClockCmd(RCC_APB1Periph: uint32_t,
                              NewState: FunctionalState);
    #[no_mangle]
    fn NVIC_Init(NVIC_InitStruct: *mut NVIC_InitTypeDef);
    #[no_mangle]
    fn NVIC_PriorityGroupConfig(NVIC_PriorityGroup: uint32_t);
    #[no_mangle]
    fn SetEPRxStatus(_: uint8_t, _: uint16_t);
    #[no_mangle]
    fn SetEPTxValid(_: uint8_t);
    #[no_mangle]
    fn SetEPTxCount(_: uint8_t, _: uint16_t);
    #[no_mangle]
    fn SetEPRxCount(_: uint8_t, _: uint16_t);
    /* *
  ******************************************************************************
  * @file    usb_mem.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Utility prototypes functions for memory/PMA transfers
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
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
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
pub const USBWakeUp_IRQn: IRQn = 42;
pub const RTCAlarm_IRQn: IRQn = 41;
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
pub const TIM1_TRG_COM_IRQn: IRQn = 26;
pub const TIM1_UP_IRQn: IRQn = 25;
pub const TIM1_BRK_IRQn: IRQn = 24;
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
pub const EXTI2_IRQn: IRQn = 8;
pub const EXTI1_IRQn: IRQn = 7;
pub const EXTI0_IRQn: IRQn = 6;
pub const RCC_IRQn: IRQn = 5;
pub const FLASH_IRQn: IRQn = 4;
pub const RTC_IRQn: IRQn = 3;
pub const TAMPER_IRQn: IRQn = 2;
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
/* * 
  * @brief General Purpose I/O
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_TypeDef {
    pub CRL: uint32_t,
    pub CRH: uint32_t,
    pub IDR: uint32_t,
    pub ODR: uint32_t,
    pub BSRR: uint32_t,
    pub BRR: uint32_t,
    pub LCKR: uint32_t,
}
/* *
  ******************************************************************************
  * @file    stm32f10x_exti.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the EXTI firmware
  *          library.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F10x_StdPeriph_Driver
  * @{
  */
/* * @addtogroup EXTI
  * @{
  */
/* * @defgroup EXTI_Exported_Types
  * @{
  */
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
/* *
  ******************************************************************************
  * @file    stm32f10x_gpio.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the GPIO 
  *          firmware library.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F10x_StdPeriph_Driver
  * @{
  */
/* * @addtogroup GPIO
  * @{
  */
/* * @defgroup GPIO_Exported_Types
  * @{
  */
/* * 
  * @brief  Output Maximum frequency selection  
  */
pub type GPIOSpeed_TypeDef = libc::c_uint;
pub const GPIO_Speed_50MHz: GPIOSpeed_TypeDef = 3;
pub const GPIO_Speed_2MHz: GPIOSpeed_TypeDef = 2;
pub const GPIO_Speed_10MHz: GPIOSpeed_TypeDef = 1;
/* * 
  * @brief  Configuration Mode enumeration  
  */
pub type GPIOMode_TypeDef = libc::c_uint;
pub const GPIO_Mode_AF_PP: GPIOMode_TypeDef = 24;
pub const GPIO_Mode_AF_OD: GPIOMode_TypeDef = 28;
pub const GPIO_Mode_Out_PP: GPIOMode_TypeDef = 16;
pub const GPIO_Mode_Out_OD: GPIOMode_TypeDef = 20;
pub const GPIO_Mode_IPU: GPIOMode_TypeDef = 72;
pub const GPIO_Mode_IPD: GPIOMode_TypeDef = 40;
pub const GPIO_Mode_IN_FLOATING: GPIOMode_TypeDef = 4;
pub const GPIO_Mode_AIN: GPIOMode_TypeDef = 0;
/* * 
  * @brief  GPIO Init structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_InitTypeDef {
    pub GPIO_Pin: uint16_t,
    pub GPIO_Speed: GPIOSpeed_TypeDef,
    pub GPIO_Mode: GPIOMode_TypeDef,
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
                         GPIO_Speed: 0 as GPIOSpeed_TypeDef,
                         GPIO_Mode: GPIO_Mode_AIN,};
    /* STM32L1XX_MD && STM32L1XX_XD */
    /* USB_USE_EXTERNAL_PULLUP */
    /* !< At this stage the microcontroller clock setting is already configured,
     this is done through SystemInit() function which is called from startup
     file (startup_stm32f10x_xx.s) before to branch to application main.
     To reconfigure the default setting of SystemInit() function, refer to
     system_stm32f10x.c file
     */
    /* STM32L1XX_XD */
    usbGenerateDisconnectPulse();
    /* STM32F37X  && STM32F303xC)*/
    RCC_APB2PeriphClockCmd(0x4 as libc::c_int as uint32_t, ENABLE);
    GPIO_InitStructure.GPIO_Pin =
        (0x800 as libc::c_int as uint16_t as libc::c_int |
             0x1000 as libc::c_int as uint16_t as libc::c_int) as uint16_t;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x10000 as libc::c_int as
                                              libc::c_uint).wrapping_add(0x800
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint)
                  as *mut GPIO_TypeDef, &mut GPIO_InitStructure);
    // Initialise callbacks
    ctrlLineStateCb = None;
    baudRateCb = None;
    /* Configure the EXTI line 18 connected internally to the USB IP */
    EXTI_ClearITPendingBit(0x40000 as libc::c_int as uint32_t);
    EXTI_InitStructure.EXTI_Line = 0x40000 as libc::c_int as uint32_t;
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
 * Function Name  : USB_Interrupts_Disable
 * Description    : Disables the USB interrupts
 * Input          : None.
 * Return         : None.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn USB_Interrupts_Disable() {
    let mut NVIC_InitStructure: NVIC_InitTypeDef =
        NVIC_InitTypeDef{NVIC_IRQChannel: 0,
                         NVIC_IRQChannelPreemptionPriority: 0,
                         NVIC_IRQChannelSubPriority: 0,
                         NVIC_IRQChannelCmd: DISABLE,};
    /* Disable the USB interrupt */
    NVIC_InitStructure.NVIC_IRQChannel =
        USB_LP_CAN1_RX0_IRQn as libc::c_int as uint8_t;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&mut NVIC_InitStructure);
    /* Disable the USB Wake-up interrupt */
    NVIC_InitStructure.NVIC_IRQChannel =
        USBWakeUp_IRQn as libc::c_int as uint8_t;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
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
    Device_Serial0 = *(0x1ffff7e8 as libc::c_int as *mut uint32_t);
    Device_Serial1 = *(0x1ffff7ec as libc::c_int as *mut uint32_t);
    Device_Serial2 = *(0x1ffff7f0 as libc::c_int as *mut uint32_t);
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
// HJI
// HJI
// HJI
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
