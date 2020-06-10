use ::libc;
extern "C" {
    /* *
  * @}
  */
    /* Exported constants --------------------------------------------------------*/
    /* * @defgroup GPIO_Exported_Constants GPIO Exported Constants
  * @{
  */
    /* * @defgroup GPIO_pins_define GPIO pins define
  * @{
  */
    /* Pin 0 selected    */
    /* Pin 1 selected    */
    /* Pin 2 selected    */
    /* Pin 3 selected    */
    /* Pin 4 selected    */
    /* Pin 5 selected    */
    /* Pin 6 selected    */
    /* Pin 7 selected    */
    /* Pin 8 selected    */
    /* Pin 9 selected    */
    /* Pin 10 selected   */
    /* Pin 11 selected   */
    /* Pin 12 selected   */
    /* Pin 13 selected   */
    /* Pin 14 selected   */
    /* Pin 15 selected   */
    /* All pins selected */
    /* PIN mask for assert test */
    /* *
  * @}
  */
    /* * @defgroup GPIO_mode_define GPIO mode define
  * @brief GPIO Configuration Mode 
  *        Elements values convention: 0xX0yz00YZ
  *           - X  : GPIO mode or EXTI Mode
  *           - y  : External IT or Event trigger detection 
  *           - z  : IO configuration on External IT or Event
  *           - Y  : Output type (Push Pull or Open Drain)
  *           - Z  : IO Direction mode (Input, Output, Alternate or Analog)
  * @{
  */
    /* !< Input Floating Mode                   */
    /* !< Output Push Pull Mode                 */
    /* !< Output Open Drain Mode                */
    /* !< Alternate Function Push Pull Mode     */
    /* !< Alternate Function Open Drain Mode    */
    /* !< Analog Mode  */
    /* !< External Interrupt Mode with Rising edge trigger detection          */
    /* !< External Interrupt Mode with Falling edge trigger detection         */
    /* !< External Interrupt Mode with Rising/Falling edge trigger detection  */
    /* !< External Event Mode with Rising edge trigger detection               */
    /* !< External Event Mode with Falling edge trigger detection              */
    /* !< External Event Mode with Rising/Falling edge trigger detection       */
    /* *
  * @}
  */
    /* * @defgroup GPIO_speed_define  GPIO speed define
  * @brief GPIO Output Maximum frequency
  * @{
  */
    /* !< Low speed     */
    /* !< Medium speed  */
    /* !< Fast speed    */
    /* !< High speed    */
    /* *
  * @}
  */
    /* * @defgroup GPIO_pull_define GPIO pull define
   * @brief GPIO Pull-Up or Pull-Down Activation
   * @{
   */
    /* !< No Pull-up or Pull-down activation  */
    /* !< Pull-up activation                  */
    /* !< Pull-down activation                */
    /* *
  * @}
  */
    /* *
  * @}
  */
    /* Exported macro ------------------------------------------------------------*/
/* * @defgroup GPIO_Exported_Macros GPIO Exported Macros
  * @{
  */
    /* *
  * @brief  Checks whether the specified EXTI line flag is set or not.
  * @param  __EXTI_LINE__: specifies the EXTI line flag to check.
  *         This parameter can be GPIO_PIN_x where x can be(0..15)
  * @retval The new state of __EXTI_LINE__ (SET or RESET).
  */
    /* *
  * @brief  Clears the EXTI's line pending flags.
  * @param  __EXTI_LINE__: specifies the EXTI lines flags to clear.
  *         This parameter can be any combination of GPIO_PIN_x where x can be (0..15)
  * @retval None
  */
    /* *
  * @brief  Checks whether the specified EXTI line is asserted or not.
  * @param  __EXTI_LINE__: specifies the EXTI line to check.
  *          This parameter can be GPIO_PIN_x where x can be(0..15)
  * @retval The new state of __EXTI_LINE__ (SET or RESET).
  */
    /* *
  * @brief  Clears the EXTI's line pending bits.
  * @param  __EXTI_LINE__: specifies the EXTI lines to clear.
  *          This parameter can be any combination of GPIO_PIN_x where x can be (0..15)
  * @retval None
  */
    /* *
  * @brief  Generates a Software interrupt on selected EXTI line.
  * @param  __EXTI_LINE__: specifies the EXTI line to check.
  *          This parameter can be GPIO_PIN_x where x can be(0..15)
  * @retval None
  */
    /* *
  * @}
  */
    /* Include GPIO HAL Extension module */
    /* Exported functions --------------------------------------------------------*/
/* * @addtogroup GPIO_Exported_Functions
  * @{
  */
    /* * @addtogroup GPIO_Exported_Functions_Group1
  * @{
  */
/* Initialization and de-initialization functions *****************************/
    #[no_mangle]
    fn HAL_GPIO_Init(GPIOx: *mut GPIO_TypeDef,
                     GPIO_Init: *mut GPIO_InitTypeDef);
    #[no_mangle]
    fn HAL_NVIC_SetPriority(IRQn: IRQn_Type, PreemptPriority: uint32_t,
                            SubPriority: uint32_t);
    #[no_mangle]
    fn HAL_NVIC_EnableIRQ(IRQn: IRQn_Type);
    /* Exported constants --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup PCDEx_Exported_Functions PCDEx Exported Functions
  * @{
  */
/* * @addtogroup PCDEx_Exported_Functions_Group1 Peripheral Control functions
  * @{
  */
    #[no_mangle]
    fn HAL_PCDEx_SetTxFiFo(hpcd_0: *mut PCD_HandleTypeDef, fifo: uint8_t,
                           size: uint16_t) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_PCDEx_SetRxFiFo(hpcd_0: *mut PCD_HandleTypeDef, size: uint16_t)
     -> HAL_StatusTypeDef;
    /* *
  * @}
  */
    /* Include PCD HAL Extension module */
    /* Exported constants --------------------------------------------------------*/
/* * @defgroup PCD_Exported_Constants PCD Exported Constants
  * @{
  */
    /* * @defgroup PCD_Speed PCD Speed
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup PCD_PHY_Module PCD PHY Module
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup PCD_Turnaround_Timeout Turnaround Timeout Value
  * @{
  */
    /* USBD_HS_TRDT_VALUE */
    /* USBD_HS_TRDT_VALUE */
    /* *
  * @}
  */
    /* *
  * @}
  */
    /* Exported macros -----------------------------------------------------------*/
/* * @defgroup PCD_Exported_Macros PCD Exported Macros
 *  @brief macros to handle interrupts and specific clock configurations
 * @{
 */
    /* !< External interrupt line 20 Connected to the USB HS EXTI Line */
    /* !< External interrupt line 18 Connected to the USB FS EXTI Line */
    /* *
  * @}
  */
    /* Exported functions --------------------------------------------------------*/
/* * @addtogroup PCD_Exported_Functions PCD Exported Functions
  * @{
  */
    /* Initialization/de-initialization functions  ********************************/
/* * @addtogroup PCD_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */
    #[no_mangle]
    fn HAL_PCD_Init(hpcd_0: *mut PCD_HandleTypeDef) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_PCD_DeInit(hpcd_0: *mut PCD_HandleTypeDef) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_PCD_Start(hpcd_0: *mut PCD_HandleTypeDef) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_PCD_Stop(hpcd_0: *mut PCD_HandleTypeDef) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_PCD_IRQHandler(hpcd_0: *mut PCD_HandleTypeDef);
    #[no_mangle]
    fn HAL_PCD_SetAddress(hpcd_0: *mut PCD_HandleTypeDef, address: uint8_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_PCD_EP_Open(hpcd_0: *mut PCD_HandleTypeDef, ep_addr: uint8_t,
                       ep_mps: uint16_t, ep_type: uint8_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_PCD_EP_Close(hpcd_0: *mut PCD_HandleTypeDef, ep_addr: uint8_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_PCD_EP_Receive(hpcd_0: *mut PCD_HandleTypeDef, ep_addr: uint8_t,
                          pBuf: *mut uint8_t, len: uint32_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_PCD_EP_Transmit(hpcd_0: *mut PCD_HandleTypeDef, ep_addr: uint8_t,
                           pBuf: *mut uint8_t, len: uint32_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_PCD_EP_GetRxCount(hpcd_0: *mut PCD_HandleTypeDef, ep_addr: uint8_t)
     -> uint16_t;
    #[no_mangle]
    fn HAL_PCD_EP_SetStall(hpcd_0: *mut PCD_HandleTypeDef, ep_addr: uint8_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_PCD_EP_ClrStall(hpcd_0: *mut PCD_HandleTypeDef, ep_addr: uint8_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_PCD_EP_Flush(hpcd_0: *mut PCD_HandleTypeDef, ep_addr: uint8_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_Delay(Delay: uint32_t);
    #[no_mangle]
    fn USBD_LL_DataOutStage(pdev: *mut USBD_HandleTypeDef, epnum: uint8_t,
                            pdata: *mut uint8_t) -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_DataInStage(pdev: *mut USBD_HandleTypeDef, epnum: uint8_t,
                           pdata: *mut uint8_t) -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_SetupStage(pdev: *mut USBD_HandleTypeDef, psetup: *mut uint8_t)
     -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_SOF(pdev: *mut USBD_HandleTypeDef) -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_SetSpeed(pdev: *mut USBD_HandleTypeDef,
                        speed: USBD_SpeedTypeDef) -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_Reset(pdev: *mut USBD_HandleTypeDef) -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_Suspend(pdev: *mut USBD_HandleTypeDef) -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_Resume(pdev: *mut USBD_HandleTypeDef) -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_IsoOUTIncomplete(pdev: *mut USBD_HandleTypeDef, epnum: uint8_t)
     -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_IsoINIncomplete(pdev: *mut USBD_HandleTypeDef, epnum: uint8_t)
     -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_DevConnected(pdev: *mut USBD_HandleTypeDef)
     -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_LL_DevDisconnected(pdev: *mut USBD_HandleTypeDef)
     -> USBD_StatusTypeDef;
    #[no_mangle]
    static mut usbDevConfig_System: usbDev_t;
    #[no_mangle]
    fn mscCheckBoot() -> bool;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type IRQn_Type = libc::c_int;
pub const SDMMC2_IRQn: IRQn_Type = 103;
pub const LPTIM1_IRQn: IRQn_Type = 93;
pub const QUADSPI_IRQn: IRQn_Type = 92;
pub const SAI2_IRQn: IRQn_Type = 91;
pub const SAI1_IRQn: IRQn_Type = 87;
pub const SPI5_IRQn: IRQn_Type = 85;
pub const SPI4_IRQn: IRQn_Type = 84;
pub const UART8_IRQn: IRQn_Type = 83;
pub const UART7_IRQn: IRQn_Type = 82;
pub const FPU_IRQn: IRQn_Type = 81;
pub const RNG_IRQn: IRQn_Type = 80;
pub const OTG_HS_IRQn: IRQn_Type = 77;
pub const OTG_HS_WKUP_IRQn: IRQn_Type = 76;
pub const OTG_HS_EP1_IN_IRQn: IRQn_Type = 75;
pub const OTG_HS_EP1_OUT_IRQn: IRQn_Type = 74;
pub const I2C3_ER_IRQn: IRQn_Type = 73;
pub const I2C3_EV_IRQn: IRQn_Type = 72;
pub const USART6_IRQn: IRQn_Type = 71;
pub const DMA2_Stream7_IRQn: IRQn_Type = 70;
pub const DMA2_Stream6_IRQn: IRQn_Type = 69;
pub const DMA2_Stream5_IRQn: IRQn_Type = 68;
pub const OTG_FS_IRQn: IRQn_Type = 67;
pub const ETH_WKUP_IRQn: IRQn_Type = 62;
pub const ETH_IRQn: IRQn_Type = 61;
pub const DMA2_Stream4_IRQn: IRQn_Type = 60;
pub const DMA2_Stream3_IRQn: IRQn_Type = 59;
pub const DMA2_Stream2_IRQn: IRQn_Type = 58;
pub const DMA2_Stream1_IRQn: IRQn_Type = 57;
pub const DMA2_Stream0_IRQn: IRQn_Type = 56;
pub const TIM7_IRQn: IRQn_Type = 55;
pub const TIM6_DAC_IRQn: IRQn_Type = 54;
pub const UART5_IRQn: IRQn_Type = 53;
pub const UART4_IRQn: IRQn_Type = 52;
pub const SPI3_IRQn: IRQn_Type = 51;
pub const TIM5_IRQn: IRQn_Type = 50;
pub const SDMMC1_IRQn: IRQn_Type = 49;
pub const FMC_IRQn: IRQn_Type = 48;
pub const DMA1_Stream7_IRQn: IRQn_Type = 47;
pub const TIM8_CC_IRQn: IRQn_Type = 46;
pub const TIM8_TRG_COM_TIM14_IRQn: IRQn_Type = 45;
pub const TIM8_UP_TIM13_IRQn: IRQn_Type = 44;
pub const TIM8_BRK_TIM12_IRQn: IRQn_Type = 43;
pub const OTG_FS_WKUP_IRQn: IRQn_Type = 42;
pub const RTC_Alarm_IRQn: IRQn_Type = 41;
pub const EXTI15_10_IRQn: IRQn_Type = 40;
pub const USART3_IRQn: IRQn_Type = 39;
pub const USART2_IRQn: IRQn_Type = 38;
pub const USART1_IRQn: IRQn_Type = 37;
pub const SPI2_IRQn: IRQn_Type = 36;
pub const SPI1_IRQn: IRQn_Type = 35;
pub const I2C2_ER_IRQn: IRQn_Type = 34;
pub const I2C2_EV_IRQn: IRQn_Type = 33;
pub const I2C1_ER_IRQn: IRQn_Type = 32;
pub const I2C1_EV_IRQn: IRQn_Type = 31;
pub const TIM4_IRQn: IRQn_Type = 30;
pub const TIM3_IRQn: IRQn_Type = 29;
pub const TIM2_IRQn: IRQn_Type = 28;
pub const TIM1_CC_IRQn: IRQn_Type = 27;
pub const TIM1_TRG_COM_TIM11_IRQn: IRQn_Type = 26;
pub const TIM1_UP_TIM10_IRQn: IRQn_Type = 25;
pub const TIM1_BRK_TIM9_IRQn: IRQn_Type = 24;
pub const EXTI9_5_IRQn: IRQn_Type = 23;
pub const CAN1_SCE_IRQn: IRQn_Type = 22;
pub const CAN1_RX1_IRQn: IRQn_Type = 21;
pub const CAN1_RX0_IRQn: IRQn_Type = 20;
pub const CAN1_TX_IRQn: IRQn_Type = 19;
pub const ADC_IRQn: IRQn_Type = 18;
pub const DMA1_Stream6_IRQn: IRQn_Type = 17;
pub const DMA1_Stream5_IRQn: IRQn_Type = 16;
pub const DMA1_Stream4_IRQn: IRQn_Type = 15;
pub const DMA1_Stream3_IRQn: IRQn_Type = 14;
pub const DMA1_Stream2_IRQn: IRQn_Type = 13;
pub const DMA1_Stream1_IRQn: IRQn_Type = 12;
pub const DMA1_Stream0_IRQn: IRQn_Type = 11;
pub const EXTI4_IRQn: IRQn_Type = 10;
pub const EXTI3_IRQn: IRQn_Type = 9;
pub const EXTI2_IRQn: IRQn_Type = 8;
pub const EXTI1_IRQn: IRQn_Type = 7;
pub const EXTI0_IRQn: IRQn_Type = 6;
pub const RCC_IRQn: IRQn_Type = 5;
pub const FLASH_IRQn: IRQn_Type = 4;
pub const RTC_WKUP_IRQn: IRQn_Type = 3;
pub const TAMP_STAMP_IRQn: IRQn_Type = 2;
pub const PVD_IRQn: IRQn_Type = 1;
pub const WWDG_IRQn: IRQn_Type = 0;
pub const SysTick_IRQn: IRQn_Type = -1;
pub const PendSV_IRQn: IRQn_Type = -2;
pub const DebugMonitor_IRQn: IRQn_Type = -4;
pub const SVCall_IRQn: IRQn_Type = -5;
pub const UsageFault_IRQn: IRQn_Type = -10;
pub const BusFault_IRQn: IRQn_Type = -11;
pub const MemoryManagement_IRQn: IRQn_Type = -12;
pub const NonMaskableInt_IRQn: IRQn_Type = -14;
/* *
  * @brief General Purpose I/O
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_TypeDef {
    pub MODER: uint32_t,
    pub OTYPER: uint32_t,
    pub OSPEEDR: uint32_t,
    pub PUPDR: uint32_t,
    pub IDR: uint32_t,
    pub ODR: uint32_t,
    pub BSRR: uint32_t,
    pub LCKR: uint32_t,
    pub AFR: [uint32_t; 2],
}
/* *
  * @brief Reset and Clock Control
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RCC_TypeDef {
    pub CR: uint32_t,
    pub PLLCFGR: uint32_t,
    pub CFGR: uint32_t,
    pub CIR: uint32_t,
    pub AHB1RSTR: uint32_t,
    pub AHB2RSTR: uint32_t,
    pub AHB3RSTR: uint32_t,
    pub RESERVED0: uint32_t,
    pub APB1RSTR: uint32_t,
    pub APB2RSTR: uint32_t,
    pub RESERVED1: [uint32_t; 2],
    pub AHB1ENR: uint32_t,
    pub AHB2ENR: uint32_t,
    pub AHB3ENR: uint32_t,
    pub RESERVED2: uint32_t,
    pub APB1ENR: uint32_t,
    pub APB2ENR: uint32_t,
    pub RESERVED3: [uint32_t; 2],
    pub AHB1LPENR: uint32_t,
    pub AHB2LPENR: uint32_t,
    pub AHB3LPENR: uint32_t,
    pub RESERVED4: uint32_t,
    pub APB1LPENR: uint32_t,
    pub APB2LPENR: uint32_t,
    pub RESERVED5: [uint32_t; 2],
    pub BDCR: uint32_t,
    pub CSR: uint32_t,
    pub RESERVED6: [uint32_t; 2],
    pub SSCGR: uint32_t,
    pub PLLI2SCFGR: uint32_t,
    pub PLLSAICFGR: uint32_t,
    pub DCKCFGR1: uint32_t,
    pub DCKCFGR2: uint32_t,
}
/* *
  * @}
  */
/* *
  * @brief USB_OTG_Core_Registers
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USB_OTG_GlobalTypeDef {
    pub GOTGCTL: uint32_t,
    pub GOTGINT: uint32_t,
    pub GAHBCFG: uint32_t,
    pub GUSBCFG: uint32_t,
    pub GRSTCTL: uint32_t,
    pub GINTSTS: uint32_t,
    pub GINTMSK: uint32_t,
    pub GRXSTSR: uint32_t,
    pub GRXSTSP: uint32_t,
    pub GRXFSIZ: uint32_t,
    pub DIEPTXF0_HNPTXFSIZ: uint32_t,
    pub HNPTXSTS: uint32_t,
    pub Reserved30: [uint32_t; 2],
    pub GCCFG: uint32_t,
    pub CID: uint32_t,
    pub Reserved5: [uint32_t; 3],
    pub GHWCFG3: uint32_t,
    pub Reserved6: uint32_t,
    pub GLPMCFG: uint32_t,
    pub GPWRDN: uint32_t,
    pub GDFIFOCFG: uint32_t,
    pub GADPCTL: uint32_t,
    pub Reserved43: [uint32_t; 39],
    pub HPTXFSIZ: uint32_t,
    pub DIEPTXF: [uint32_t; 15],
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
  * @file    stm32f7xx_hal_gpio.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of GPIO HAL module.
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
/* * @addtogroup GPIO
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup GPIO_Exported_Types GPIO Exported Types
  * @{
  */
/* * 
  * @brief GPIO Init structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_InitTypeDef {
    pub Pin: uint32_t,
    pub Mode: uint32_t,
    pub Pull: uint32_t,
    pub Speed: uint32_t,
    pub Alternate: uint32_t,
}
/* * 
  * @brief  PCD Initialization Structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USB_OTG_CfgTypeDef {
    pub dev_endpoints: uint32_t,
    pub Host_channels: uint32_t,
    pub speed: uint32_t,
    pub dma_enable: uint32_t,
    pub ep0_mps: uint32_t,
    pub phy_itface: uint32_t,
    pub Sof_enable: uint32_t,
    pub low_power_enable: uint32_t,
    pub lpm_enable: uint32_t,
    pub battery_charging_enable: uint32_t,
    pub vbus_sensing_enable: uint32_t,
    pub use_dedicated_ep1: uint32_t,
    pub use_external_vbus: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USB_OTG_EPTypeDef {
    pub num: uint8_t,
    pub is_in: uint8_t,
    pub is_stall: uint8_t,
    pub type_0: uint8_t,
    pub data_pid_start: uint8_t,
    pub even_odd_frame: uint8_t,
    pub tx_fifo_num: uint16_t,
    pub maxpacket: uint32_t,
    pub xfer_buff: *mut uint8_t,
    pub dma_addr: uint32_t,
    pub xfer_len: uint32_t,
    pub xfer_count: uint32_t,
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_pcd.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of PCD HAL module.
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
/* * @addtogroup PCD
  * @{
  */
/* Exported types ------------------------------------------------------------*/ 
/* * @defgroup PCD_Exported_Types PCD Exported Types
  * @{
  */
/* *
  * @brief  PCD State structure definition
  */
pub type PCD_StateTypeDef = libc::c_uint;
pub const HAL_PCD_STATE_TIMEOUT: PCD_StateTypeDef = 4;
pub const HAL_PCD_STATE_BUSY: PCD_StateTypeDef = 3;
pub const HAL_PCD_STATE_ERROR: PCD_StateTypeDef = 2;
pub const HAL_PCD_STATE_READY: PCD_StateTypeDef = 1;
pub const HAL_PCD_STATE_RESET: PCD_StateTypeDef = 0;
/* Device LPM suspend state */
pub type PCD_LPM_StateTypeDef = libc::c_uint;
/* off */
/* suspend */
pub const LPM_L3: PCD_LPM_StateTypeDef = 3;
/* LPM L1 sleep */
pub const LPM_L2: PCD_LPM_StateTypeDef = 2;
/* on */
pub const LPM_L1: PCD_LPM_StateTypeDef = 1;
pub const LPM_L0: PCD_LPM_StateTypeDef = 0;
pub type PCD_TypeDef = USB_OTG_GlobalTypeDef;
pub type PCD_InitTypeDef = USB_OTG_CfgTypeDef;
pub type PCD_EPTypeDef = USB_OTG_EPTypeDef;
/* * 
  * @brief  PCD Handle Structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PCD_HandleTypeDef {
    pub Instance: *mut PCD_TypeDef,
    pub Init: PCD_InitTypeDef,
    pub IN_ep: [PCD_EPTypeDef; 16],
    pub OUT_ep: [PCD_EPTypeDef; 16],
    pub Lock: HAL_LockTypeDef,
    pub State: PCD_StateTypeDef,
    pub Setup: [uint32_t; 12],
    pub LPM_State: PCD_LPM_StateTypeDef,
    pub BESL: uint32_t,
    pub lpm_active: uint32_t,
    pub battery_charging_active: uint32_t,
    pub pData: *mut libc::c_void,
}
pub type USBD_StatusTypeDef = libc::c_uint;
pub const USBD_FAIL: USBD_StatusTypeDef = 2;
pub const USBD_BUSY: USBD_StatusTypeDef = 1;
pub const USBD_OK: USBD_StatusTypeDef = 0;
/* USB Device descriptors structure */
/* USB Device handle structure */
/* USB Device handle structure */
pub type USBD_HandleTypeDef = _USBD_HandleTypeDef;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _USBD_HandleTypeDef {
    pub id: uint8_t,
    pub dev_config: uint32_t,
    pub dev_default_config: uint32_t,
    pub dev_config_status: uint32_t,
    pub dev_speed: USBD_SpeedTypeDef,
    pub ep_in: [USBD_EndpointTypeDef; 15],
    pub ep_out: [USBD_EndpointTypeDef; 15],
    pub ep0_state: uint32_t,
    pub ep0_data_len: uint32_t,
    pub dev_state: uint8_t,
    pub dev_old_state: uint8_t,
    pub dev_address: uint8_t,
    pub dev_connection_status: uint8_t,
    pub dev_test_mode: uint8_t,
    pub dev_remote_wakeup: uint32_t,
    pub request: USBD_SetupReqTypedef,
    pub pDesc: *mut USBD_DescriptorsTypeDef,
    pub pClass: *mut USBD_ClassTypeDef,
    pub pCDC_ClassData: *mut libc::c_void,
    pub pCDC_UserData: *mut libc::c_void,
    pub pHID_ClassData: *mut libc::c_void,
    pub pHID_UserData: *mut libc::c_void,
    pub pMSC_ClassData: *mut libc::c_void,
    pub pMSC_UserData: *mut libc::c_void,
    pub pData: *mut libc::c_void,
}
pub type USBD_ClassTypeDef = _Device_cb;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _Device_cb {
    pub Init: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef,
                                          _: uint8_t) -> uint8_t>,
    pub DeInit: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef,
                                            _: uint8_t) -> uint8_t>,
    pub Setup: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef,
                                           _: *mut USBD_SetupReqTypedef)
                          -> uint8_t>,
    pub EP0_TxSent: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef)
                               -> uint8_t>,
    pub EP0_RxReady: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef)
                                -> uint8_t>,
    pub DataIn: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef,
                                            _: uint8_t) -> uint8_t>,
    pub DataOut: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef,
                                             _: uint8_t) -> uint8_t>,
    pub SOF: Option<unsafe extern "C" fn(_: *mut _USBD_HandleTypeDef)
                        -> uint8_t>,
    pub IsoINIncomplete: Option<unsafe extern "C" fn(_:
                                                         *mut _USBD_HandleTypeDef,
                                                     _: uint8_t) -> uint8_t>,
    pub IsoOUTIncomplete: Option<unsafe extern "C" fn(_:
                                                          *mut _USBD_HandleTypeDef,
                                                      _: uint8_t) -> uint8_t>,
    pub GetHSConfigDescriptor: Option<unsafe extern "C" fn(_: *mut uint16_t)
                                          -> *mut uint8_t>,
    pub GetFSConfigDescriptor: Option<unsafe extern "C" fn(_: *mut uint16_t)
                                          -> *mut uint8_t>,
    pub GetOtherSpeedConfigDescriptor: Option<unsafe extern "C" fn(_:
                                                                       *mut uint16_t)
                                                  -> *mut uint8_t>,
    pub GetDeviceQualifierDescriptor: Option<unsafe extern "C" fn(_:
                                                                      *mut uint16_t)
                                                 -> *mut uint8_t>,
}
pub type USBD_SetupReqTypedef = usb_setup_req;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct usb_setup_req {
    pub bmRequest: uint8_t,
    pub bRequest: uint8_t,
    pub wValue: uint16_t,
    pub wIndex: uint16_t,
    pub wLength: uint16_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USBD_DescriptorsTypeDef {
    pub GetDeviceDescriptor: Option<unsafe extern "C" fn(_: USBD_SpeedTypeDef,
                                                         _: *mut uint16_t)
                                        -> *mut uint8_t>,
    pub GetLangIDStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                USBD_SpeedTypeDef,
                                                            _: *mut uint16_t)
                                           -> *mut uint8_t>,
    pub GetManufacturerStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                      USBD_SpeedTypeDef,
                                                                  _:
                                                                      *mut uint16_t)
                                                 -> *mut uint8_t>,
    pub GetProductStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                 USBD_SpeedTypeDef,
                                                             _: *mut uint16_t)
                                            -> *mut uint8_t>,
    pub GetSerialStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                USBD_SpeedTypeDef,
                                                            _: *mut uint16_t)
                                           -> *mut uint8_t>,
    pub GetConfigurationStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                       USBD_SpeedTypeDef,
                                                                   _:
                                                                       *mut uint16_t)
                                                  -> *mut uint8_t>,
    pub GetInterfaceStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                   USBD_SpeedTypeDef,
                                                               _:
                                                                   *mut uint16_t)
                                              -> *mut uint8_t>,
}
pub type USBD_SpeedTypeDef = libc::c_uint;
pub const USBD_SPEED_LOW: USBD_SpeedTypeDef = 2;
pub const USBD_SPEED_FULL: USBD_SpeedTypeDef = 1;
pub const USBD_SPEED_HIGH: USBD_SpeedTypeDef = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USBD_EndpointTypeDef {
    pub status: uint32_t,
    pub total_length: uint32_t,
    pub rem_length: uint32_t,
    pub maxpacket: uint32_t,
}
pub const COMPOSITE: USB_DEV = 1;
pub type usbDev_t = usbDev_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct usbDev_s {
    pub type_0: uint8_t,
    pub mscButtonPin: ioTag_t,
    pub mscButtonUsePullup: uint8_t,
}
pub type ioTag_t = uint8_t;
/* This is stupid, any nice solution to handle multiple interfaces
   * would be much apriciated. Or at least a flow how this should be rewritten instead.
   */
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
pub type USB_DEV = libc::c_uint;
pub const DEFAULT: USB_DEV = 0;
#[inline]
unsafe extern "C" fn usbDevConfig() -> *const usbDev_t {
    return &mut usbDevConfig_System;
}
/* *
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/Src/usbd_conf.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-April-2016
  * @brief   This file implements the USB Device library callbacks and MSP
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#[no_mangle]
pub static mut hpcd: PCD_HandleTypeDef =
    PCD_HandleTypeDef{Instance: 0 as *const PCD_TypeDef as *mut PCD_TypeDef,
                      Init:
                          PCD_InitTypeDef{dev_endpoints: 0,
                                          Host_channels: 0,
                                          speed: 0,
                                          dma_enable: 0,
                                          ep0_mps: 0,
                                          phy_itface: 0,
                                          Sof_enable: 0,
                                          low_power_enable: 0,
                                          lpm_enable: 0,
                                          battery_charging_enable: 0,
                                          vbus_sensing_enable: 0,
                                          use_dedicated_ep1: 0,
                                          use_external_vbus: 0,},
                      IN_ep:
                          [PCD_EPTypeDef{num: 0,
                                         is_in: 0,
                                         is_stall: 0,
                                         type_0: 0,
                                         data_pid_start: 0,
                                         even_odd_frame: 0,
                                         tx_fifo_num: 0,
                                         maxpacket: 0,
                                         xfer_buff:
                                             0 as *const uint8_t as
                                                 *mut uint8_t,
                                         dma_addr: 0,
                                         xfer_len: 0,
                                         xfer_count: 0,}; 16],
                      OUT_ep:
                          [PCD_EPTypeDef{num: 0,
                                         is_in: 0,
                                         is_stall: 0,
                                         type_0: 0,
                                         data_pid_start: 0,
                                         even_odd_frame: 0,
                                         tx_fifo_num: 0,
                                         maxpacket: 0,
                                         xfer_buff:
                                             0 as *const uint8_t as
                                                 *mut uint8_t,
                                         dma_addr: 0,
                                         xfer_len: 0,
                                         xfer_count: 0,}; 16],
                      Lock: HAL_UNLOCKED,
                      State: HAL_PCD_STATE_RESET,
                      Setup: [0; 12],
                      LPM_State: LPM_L0,
                      BESL: 0,
                      lpm_active: 0,
                      battery_charging_active: 0,
                      pData: 0 as *const libc::c_void as *mut libc::c_void,};
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* ******************************************************************************
                       PCD BSP Routines
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn OTG_FS_IRQHandler() {
    HAL_PCD_IRQHandler(&mut hpcd);
}
/* *
  * @brief  Initializes the PCD MSP.
  * @param  hpcd: PCD handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_MspInit(mut hpcd_0: *mut PCD_HandleTypeDef) {
    let mut GPIO_InitStruct: GPIO_InitTypeDef =
        GPIO_InitTypeDef{Pin: 0, Mode: 0, Pull: 0, Speed: 0, Alternate: 0,};
    if (*hpcd_0).Instance ==
           0x50000000 as libc::c_uint as *mut USB_OTG_GlobalTypeDef {
        /* Configure USB FS GPIOs */
        let mut tmpreg: uint32_t = 0;
        let ref mut fresh0 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).AHB1ENR;
        ::core::ptr::write_volatile(fresh0,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).AHB1ENR &
                                        (0x1 as libc::c_uint) <<
                                            0 as libc::c_uint);
        /* Configure DM DP Pins */
        GPIO_InitStruct.Pin =
            (0x800 as libc::c_uint as uint16_t as libc::c_int |
                 0x1000 as libc::c_uint as uint16_t as libc::c_int) as
                uint32_t;
        GPIO_InitStruct.Mode = 0x2 as libc::c_uint;
        GPIO_InitStruct.Pull = 0 as libc::c_uint;
        GPIO_InitStruct.Speed = 0x3 as libc::c_uint;
        GPIO_InitStruct.Alternate =
            0xa as libc::c_uint as uint8_t as uint32_t;
        HAL_GPIO_Init((0x40000000 as
                           libc::c_uint).wrapping_add(0x20000 as
                                                          libc::c_uint).wrapping_add(0
                                                                                         as
                                                                                         libc::c_uint)
                          as *mut GPIO_TypeDef, &mut GPIO_InitStruct);
        if (*hpcd_0).Init.vbus_sensing_enable ==
               1 as libc::c_int as libc::c_uint {
            /* Configure VBUS Pin */
            GPIO_InitStruct.Pin =
                0x200 as libc::c_uint as uint16_t as uint32_t;
            GPIO_InitStruct.Mode = 0 as libc::c_uint;
            GPIO_InitStruct.Pull = 0 as libc::c_uint;
            HAL_GPIO_Init((0x40000000 as
                               libc::c_uint).wrapping_add(0x20000 as
                                                              libc::c_uint).wrapping_add(0
                                                                                             as
                                                                                             libc::c_uint)
                              as *mut GPIO_TypeDef, &mut GPIO_InitStruct);
        }
        /* Enable USB FS Clock */
        let mut tmpreg_0: uint32_t = 0;
        let ref mut fresh1 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).AHB2ENR;
        ::core::ptr::write_volatile(fresh1,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             7 as libc::c_uint) as uint32_t as
                                        uint32_t);
        ::core::ptr::write_volatile(&mut tmpreg_0 as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).AHB2ENR &
                                        (0x1 as libc::c_uint) <<
                                            7 as libc::c_uint);
        let mut tmpreg_1: uint32_t = 0;
        let ref mut fresh2 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).APB2ENR;
        ::core::ptr::write_volatile(fresh2,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             14 as libc::c_uint) as uint32_t
                                        as uint32_t);
        ::core::ptr::write_volatile(&mut tmpreg_1 as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).APB2ENR &
                                        (0x1 as libc::c_uint) <<
                                            14 as libc::c_uint);
        /* Set USBFS Interrupt priority */
        HAL_NVIC_SetPriority(OTG_FS_IRQn, 6 as libc::c_int as uint32_t,
                             0 as libc::c_int as uint32_t);
        /* Enable USBFS Interrupt */
        HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
    } else if (*hpcd_0).Instance ==
                  0x40040000 as libc::c_uint as *mut USB_OTG_GlobalTypeDef {
        /* Configure USB FS GPIOs */
        let mut tmpreg_2: uint32_t = 0;
        let ref mut fresh3 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).AHB1ENR;
        ::core::ptr::write_volatile(fresh3,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        ::core::ptr::write_volatile(&mut tmpreg_2 as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).AHB1ENR &
                                        (0x1 as libc::c_uint) <<
                                            0 as libc::c_uint);
        let mut tmpreg_3: uint32_t = 0;
        let ref mut fresh4 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).AHB1ENR;
        ::core::ptr::write_volatile(fresh4,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             1 as libc::c_uint) as uint32_t as
                                        uint32_t);
        ::core::ptr::write_volatile(&mut tmpreg_3 as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).AHB1ENR &
                                        (0x1 as libc::c_uint) <<
                                            1 as libc::c_uint);
        let mut tmpreg_4: uint32_t = 0;
        let ref mut fresh5 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).AHB1ENR;
        ::core::ptr::write_volatile(fresh5,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             2 as libc::c_uint) as uint32_t as
                                        uint32_t);
        ::core::ptr::write_volatile(&mut tmpreg_4 as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).AHB1ENR &
                                        (0x1 as libc::c_uint) <<
                                            2 as libc::c_uint);
        let mut tmpreg_5: uint32_t = 0;
        let ref mut fresh6 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).AHB1ENR;
        ::core::ptr::write_volatile(fresh6,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             7 as libc::c_uint) as uint32_t as
                                        uint32_t);
        ::core::ptr::write_volatile(&mut tmpreg_5 as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).AHB1ENR &
                                        (0x1 as libc::c_uint) <<
                                            7 as libc::c_uint);
        let mut tmpreg_6: uint32_t = 0;
        let ref mut fresh7 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).AHB1ENR;
        ::core::ptr::write_volatile(fresh7,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             8 as libc::c_uint) as uint32_t as
                                        uint32_t);
        ::core::ptr::write_volatile(&mut tmpreg_6 as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).AHB1ENR &
                                        (0x1 as libc::c_uint) <<
                                            8 as libc::c_uint);
        /* CLK */
        GPIO_InitStruct.Pin = 0x20 as libc::c_uint as uint16_t as uint32_t;
        GPIO_InitStruct.Mode = 0x2 as libc::c_uint;
        GPIO_InitStruct.Pull = 0 as libc::c_uint;
        GPIO_InitStruct.Speed = 0x3 as libc::c_uint;
        GPIO_InitStruct.Alternate =
            0xa as libc::c_uint as uint8_t as uint32_t;
        HAL_GPIO_Init((0x40000000 as
                           libc::c_uint).wrapping_add(0x20000 as
                                                          libc::c_uint).wrapping_add(0
                                                                                         as
                                                                                         libc::c_uint)
                          as *mut GPIO_TypeDef, &mut GPIO_InitStruct);
        /* D0 */
        GPIO_InitStruct.Pin = 0x8 as libc::c_uint as uint16_t as uint32_t;
        GPIO_InitStruct.Mode = 0x2 as libc::c_uint;
        GPIO_InitStruct.Pull = 0 as libc::c_uint;
        GPIO_InitStruct.Speed = 0x3 as libc::c_uint;
        GPIO_InitStruct.Alternate =
            0xa as libc::c_uint as uint8_t as uint32_t;
        HAL_GPIO_Init((0x40000000 as
                           libc::c_uint).wrapping_add(0x20000 as
                                                          libc::c_uint).wrapping_add(0
                                                                                         as
                                                                                         libc::c_uint)
                          as *mut GPIO_TypeDef, &mut GPIO_InitStruct);
        /* D1 D2 D3 D4 D5 D6 D7 */
        GPIO_InitStruct.Pin =
            (0x1 as libc::c_uint as uint16_t as libc::c_int |
                 0x2 as libc::c_uint as uint16_t as libc::c_int |
                 0x20 as libc::c_uint as uint16_t as libc::c_int |
                 0x400 as libc::c_uint as uint16_t as libc::c_int |
                 0x800 as libc::c_uint as uint16_t as libc::c_int |
                 0x1000 as libc::c_uint as uint16_t as libc::c_int |
                 0x2000 as libc::c_uint as uint16_t as libc::c_int) as
                uint32_t;
        GPIO_InitStruct.Mode = 0x2 as libc::c_uint;
        GPIO_InitStruct.Pull = 0 as libc::c_uint;
        GPIO_InitStruct.Alternate =
            0xa as libc::c_uint as uint8_t as uint32_t;
        HAL_GPIO_Init((0x40000000 as
                           libc::c_uint).wrapping_add(0x20000 as
                                                          libc::c_uint).wrapping_add(0x400
                                                                                         as
                                                                                         libc::c_uint)
                          as *mut GPIO_TypeDef, &mut GPIO_InitStruct);
        /* STP */
        GPIO_InitStruct.Pin = 0x1 as libc::c_uint as uint16_t as uint32_t;
        GPIO_InitStruct.Mode = 0x2 as libc::c_uint;
        GPIO_InitStruct.Pull = 0 as libc::c_uint;
        GPIO_InitStruct.Alternate =
            0xa as libc::c_uint as uint8_t as uint32_t;
        HAL_GPIO_Init((0x40000000 as
                           libc::c_uint).wrapping_add(0x20000 as
                                                          libc::c_uint).wrapping_add(0x800
                                                                                         as
                                                                                         libc::c_uint)
                          as *mut GPIO_TypeDef, &mut GPIO_InitStruct);
        /* NXT */
        GPIO_InitStruct.Pin = 0x10 as libc::c_uint as uint16_t as uint32_t;
        GPIO_InitStruct.Mode = 0x2 as libc::c_uint;
        GPIO_InitStruct.Pull = 0 as libc::c_uint;
        GPIO_InitStruct.Alternate =
            0xa as libc::c_uint as uint8_t as uint32_t;
        HAL_GPIO_Init((0x40000000 as
                           libc::c_uint).wrapping_add(0x20000 as
                                                          libc::c_uint).wrapping_add(0x1c00
                                                                                         as
                                                                                         libc::c_uint)
                          as *mut GPIO_TypeDef, &mut GPIO_InitStruct);
        /* DIR */
        GPIO_InitStruct.Pin = 0x800 as libc::c_uint as uint16_t as uint32_t;
        GPIO_InitStruct.Mode = 0x2 as libc::c_uint;
        GPIO_InitStruct.Pull = 0 as libc::c_uint;
        GPIO_InitStruct.Alternate =
            0xa as libc::c_uint as uint8_t as uint32_t;
        HAL_GPIO_Init((0x40000000 as
                           libc::c_uint).wrapping_add(0x20000 as
                                                          libc::c_uint).wrapping_add(0x2000
                                                                                         as
                                                                                         libc::c_uint)
                          as *mut GPIO_TypeDef, &mut GPIO_InitStruct);
        let mut tmpreg_7: uint32_t = 0;
        let ref mut fresh8 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).AHB1ENR;
        ::core::ptr::write_volatile(fresh8,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             30 as libc::c_uint) as uint32_t
                                        as uint32_t);
        ::core::ptr::write_volatile(&mut tmpreg_7 as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).AHB1ENR &
                                        (0x1 as libc::c_uint) <<
                                            30 as libc::c_uint);
        /* Enable USB HS Clocks */
        let mut tmpreg_8: uint32_t = 0;
        let ref mut fresh9 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).AHB1ENR;
        ::core::ptr::write_volatile(fresh9,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             29 as libc::c_uint) as uint32_t
                                        as uint32_t);
        ::core::ptr::write_volatile(&mut tmpreg_8 as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3800
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut RCC_TypeDef)).AHB1ENR &
                                        (0x1 as libc::c_uint) <<
                                            29 as libc::c_uint);
        /* Set USBHS Interrupt to the lowest priority */
        HAL_NVIC_SetPriority(OTG_HS_IRQn, 6 as libc::c_int as uint32_t,
                             0 as libc::c_int as uint32_t);
        /* Enable USBHS Interrupt */
        HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
    };
}
/* *
  * @brief  De-Initializes the PCD MSP.
  * @param  hpcd: PCD handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_MspDeInit(mut hpcd_0:
                                               *mut PCD_HandleTypeDef) {
    if (*hpcd_0).Instance ==
           0x50000000 as libc::c_uint as *mut USB_OTG_GlobalTypeDef {
        /* Disable USB FS Clock */
        let ref mut fresh10 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).AHB2ENR;
        ::core::ptr::write_volatile(fresh10,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh10
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               7 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        let ref mut fresh11 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).APB2ENR;
        ::core::ptr::write_volatile(fresh11,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh11
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               14 as libc::c_uint)) as
                                        uint32_t as uint32_t)
    } else if (*hpcd_0).Instance ==
                  0x40040000 as libc::c_uint as *mut USB_OTG_GlobalTypeDef {
        /* Disable USB HS Clocks */
        let ref mut fresh12 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).AHB1ENR;
        ::core::ptr::write_volatile(fresh12,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh12
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               29 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        let ref mut fresh13 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).APB2ENR;
        ::core::ptr::write_volatile(fresh13,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh13
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               14 as libc::c_uint)) as
                                        uint32_t as uint32_t)
    };
}
/* ******************************************************************************
                       LL Driver Callbacks (PCD -> USB Device Library)
*******************************************************************************/
/* *
  * @brief  SetupStage callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_SetupStageCallback(mut hpcd_0:
                                                        *mut PCD_HandleTypeDef) {
    USBD_LL_SetupStage((*hpcd_0).pData as *mut USBD_HandleTypeDef,
                       (*hpcd_0).Setup.as_mut_ptr() as *mut uint8_t);
}
/* *
  * @brief  DataOut Stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_DataOutStageCallback(mut hpcd_0:
                                                          *mut PCD_HandleTypeDef,
                                                      mut epnum: uint8_t) {
    USBD_LL_DataOutStage((*hpcd_0).pData as *mut USBD_HandleTypeDef, epnum,
                         (*hpcd_0).OUT_ep[epnum as usize].xfer_buff);
}
/* *
  * @brief  DataIn Stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_DataInStageCallback(mut hpcd_0:
                                                         *mut PCD_HandleTypeDef,
                                                     mut epnum: uint8_t) {
    USBD_LL_DataInStage((*hpcd_0).pData as *mut USBD_HandleTypeDef, epnum,
                        (*hpcd_0).IN_ep[epnum as usize].xfer_buff);
}
/* *
  * @brief  SOF callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_SOFCallback(mut hpcd_0:
                                                 *mut PCD_HandleTypeDef) {
    USBD_LL_SOF((*hpcd_0).pData as *mut USBD_HandleTypeDef);
}
/* *
  * @brief  Reset callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_ResetCallback(mut hpcd_0:
                                                   *mut PCD_HandleTypeDef) {
    let mut speed: USBD_SpeedTypeDef = USBD_SPEED_FULL;
    /* Set USB Current Speed */
    match (*hpcd_0).Init.speed {
        0 => { speed = USBD_SPEED_HIGH }
        2 => { speed = USBD_SPEED_FULL }
        _ => { speed = USBD_SPEED_FULL }
    }
    /* Reset Device */
    USBD_LL_Reset((*hpcd_0).pData as *mut USBD_HandleTypeDef);
    USBD_LL_SetSpeed((*hpcd_0).pData as *mut USBD_HandleTypeDef, speed);
}
/* *
  * @brief  Suspend callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_SuspendCallback(mut hpcd_0:
                                                     *mut PCD_HandleTypeDef) {
    USBD_LL_Suspend((*hpcd_0).pData as *mut USBD_HandleTypeDef);
}
/* *
  * @brief  Resume callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_ResumeCallback(mut hpcd_0:
                                                    *mut PCD_HandleTypeDef) {
    USBD_LL_Resume((*hpcd_0).pData as *mut USBD_HandleTypeDef);
}
/* *
  * @brief  ISOOUTIncomplete callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_ISOOUTIncompleteCallback(mut hpcd_0:
                                                              *mut PCD_HandleTypeDef,
                                                          mut epnum:
                                                              uint8_t) {
    USBD_LL_IsoOUTIncomplete((*hpcd_0).pData as *mut USBD_HandleTypeDef,
                             epnum);
}
/* *
  * @brief  ISOINIncomplete callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_ISOINIncompleteCallback(mut hpcd_0:
                                                             *mut PCD_HandleTypeDef,
                                                         mut epnum: uint8_t) {
    USBD_LL_IsoINIncomplete((*hpcd_0).pData as *mut USBD_HandleTypeDef,
                            epnum);
}
/* *
  * @brief  ConnectCallback callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_ConnectCallback(mut hpcd_0:
                                                     *mut PCD_HandleTypeDef) {
    USBD_LL_DevConnected((*hpcd_0).pData as *mut USBD_HandleTypeDef);
}
/* *
  * @}
  */
/* I/O operation functions  ***************************************************/
/* Non-Blocking mode: Interrupt */
/* * @addtogroup PCD_Exported_Functions_Group2 Input and Output operation functions
  * @{
  */
/* *
  * @brief  Disconnect callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_DisconnectCallback(mut hpcd_0:
                                                        *mut PCD_HandleTypeDef) {
    USBD_LL_DevDisconnected((*hpcd_0).pData as *mut USBD_HandleTypeDef);
}
/* ******************************************************************************
                       LL Driver Interface (USB Device Library --> PCD)
*******************************************************************************/
/* *
  * @brief  Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_Init(mut pdev: *mut USBD_HandleTypeDef)
 -> USBD_StatusTypeDef {
    /* Set LL Driver parameters */
    hpcd.Instance = 0x50000000 as libc::c_uint as *mut USB_OTG_GlobalTypeDef;
    hpcd.Init.dev_endpoints = 4 as libc::c_int as uint32_t;
    hpcd.Init.use_dedicated_ep1 = 0 as libc::c_int as uint32_t;
    hpcd.Init.ep0_mps = 0x40 as libc::c_int as uint32_t;
    hpcd.Init.dma_enable = 0 as libc::c_int as uint32_t;
    hpcd.Init.low_power_enable = 0 as libc::c_int as uint32_t;
    hpcd.Init.phy_itface = 2 as libc::c_uint;
    hpcd.Init.Sof_enable = 0 as libc::c_int as uint32_t;
    hpcd.Init.speed = 2 as libc::c_uint;
    hpcd.Init.vbus_sensing_enable = 0 as libc::c_int as uint32_t;
    hpcd.Init.lpm_enable = 0 as libc::c_int as uint32_t;
    /* Link The driver to the stack */
    hpcd.pData = pdev as *mut libc::c_void;
    (*pdev).pData = &mut hpcd as *mut PCD_HandleTypeDef as *mut libc::c_void;
    /* Initialize LL Driver */
    HAL_PCD_Init(&mut hpcd);
    if (*usbDevConfig()).type_0 as libc::c_int == COMPOSITE as libc::c_int &&
           !mscCheckBoot() {
        HAL_PCDEx_SetRxFiFo(&mut hpcd, 0x80 as libc::c_int as uint16_t);
        HAL_PCDEx_SetTxFiFo(&mut hpcd, 0 as libc::c_int as uint8_t,
                            0x20 as libc::c_int as uint16_t);
        HAL_PCDEx_SetTxFiFo(&mut hpcd, 1 as libc::c_int as uint8_t,
                            0x40 as libc::c_int as uint16_t);
        HAL_PCDEx_SetTxFiFo(&mut hpcd, 2 as libc::c_int as uint8_t,
                            0x20 as libc::c_int as uint16_t);
        HAL_PCDEx_SetTxFiFo(&mut hpcd, 3 as libc::c_int as uint8_t,
                            0x40 as libc::c_int as uint16_t);
    } else {
        /* CDC_HID */
        HAL_PCDEx_SetRxFiFo(&mut hpcd, 0x80 as libc::c_int as uint16_t);
        HAL_PCDEx_SetTxFiFo(&mut hpcd, 0 as libc::c_int as uint8_t,
                            0x40 as libc::c_int as uint16_t);
        HAL_PCDEx_SetTxFiFo(&mut hpcd, 1 as libc::c_int as uint8_t,
                            0x80 as libc::c_int as uint16_t);
    }
    /* CDC_HID */
    return USBD_OK;
}
/* *
  * @brief  De-Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_DeInit(mut pdev: *mut USBD_HandleTypeDef)
 -> USBD_StatusTypeDef {
    HAL_PCD_DeInit((*pdev).pData as *mut PCD_HandleTypeDef);
    return USBD_OK;
}
/* *
  * @brief  Starts the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_Start(mut pdev: *mut USBD_HandleTypeDef)
 -> USBD_StatusTypeDef {
    HAL_PCD_Start((*pdev).pData as *mut PCD_HandleTypeDef);
    return USBD_OK;
}
/* *
  * @brief  Stops the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_Stop(mut pdev: *mut USBD_HandleTypeDef)
 -> USBD_StatusTypeDef {
    HAL_PCD_Stop((*pdev).pData as *mut PCD_HandleTypeDef);
    return USBD_OK;
}
/* *
  * @brief  Opens an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  ep_type: Endpoint Type
  * @param  ep_mps: Endpoint Max Packet Size
  * @retval USBD Status
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_OpenEP(mut pdev: *mut USBD_HandleTypeDef,
                                        mut ep_addr: uint8_t,
                                        mut ep_type: uint8_t,
                                        mut ep_mps: uint16_t)
 -> USBD_StatusTypeDef {
    HAL_PCD_EP_Open((*pdev).pData as *mut PCD_HandleTypeDef, ep_addr, ep_mps,
                    ep_type);
    return USBD_OK;
}
/* *
  * @brief  Closes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_CloseEP(mut pdev: *mut USBD_HandleTypeDef,
                                         mut ep_addr: uint8_t)
 -> USBD_StatusTypeDef {
    HAL_PCD_EP_Close((*pdev).pData as *mut PCD_HandleTypeDef, ep_addr);
    return USBD_OK;
}
/* *
  * @brief  Flushes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_FlushEP(mut pdev: *mut USBD_HandleTypeDef,
                                         mut ep_addr: uint8_t)
 -> USBD_StatusTypeDef {
    HAL_PCD_EP_Flush((*pdev).pData as *mut PCD_HandleTypeDef, ep_addr);
    return USBD_OK;
}
/* *
  * @brief  Sets a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_StallEP(mut pdev: *mut USBD_HandleTypeDef,
                                         mut ep_addr: uint8_t)
 -> USBD_StatusTypeDef {
    HAL_PCD_EP_SetStall((*pdev).pData as *mut PCD_HandleTypeDef, ep_addr);
    return USBD_OK;
}
/* *
  * @brief  Clears a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_ClearStallEP(mut pdev:
                                                  *mut USBD_HandleTypeDef,
                                              mut ep_addr: uint8_t)
 -> USBD_StatusTypeDef {
    HAL_PCD_EP_ClrStall((*pdev).pData as *mut PCD_HandleTypeDef, ep_addr);
    return USBD_OK;
}
/* *
  * @brief  Returns Stall condition.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Stall (1: Yes, 0: No)
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_IsStallEP(mut pdev: *mut USBD_HandleTypeDef,
                                           mut ep_addr: uint8_t) -> uint8_t {
    let mut hpcd_0: *mut PCD_HandleTypeDef =
        (*pdev).pData as *mut PCD_HandleTypeDef;
    if ep_addr as libc::c_int & 0x80 as libc::c_int == 0x80 as libc::c_int {
        return (*hpcd_0).IN_ep[(ep_addr as libc::c_int & 0x7f as libc::c_int)
                                   as usize].is_stall
    } else {
        return (*hpcd_0).OUT_ep[(ep_addr as libc::c_int & 0x7f as libc::c_int)
                                    as usize].is_stall
    };
}
/* *
  * @brief  Assigns a USB address to the device.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_SetUSBAddress(mut pdev:
                                                   *mut USBD_HandleTypeDef,
                                               mut dev_addr: uint8_t)
 -> USBD_StatusTypeDef {
    HAL_PCD_SetAddress((*pdev).pData as *mut PCD_HandleTypeDef, dev_addr);
    return USBD_OK;
}
/* *
  * @brief  Transmits data over an endpoint.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf: Pointer to data to be sent
  * @param  size: Data size
  * @retval USBD Status
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_Transmit(mut pdev: *mut USBD_HandleTypeDef,
                                          mut ep_addr: uint8_t,
                                          mut pbuf: *mut uint8_t,
                                          mut size: uint16_t)
 -> USBD_StatusTypeDef {
    HAL_PCD_EP_Transmit((*pdev).pData as *mut PCD_HandleTypeDef, ep_addr,
                        pbuf, size as uint32_t);
    return USBD_OK;
}
/* *
  * @brief  Prepares an endpoint for reception.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf: Pointer to data to be received
  * @param  size: Data size
  * @retval USBD Status
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_PrepareReceive(mut pdev:
                                                    *mut USBD_HandleTypeDef,
                                                mut ep_addr: uint8_t,
                                                mut pbuf: *mut uint8_t,
                                                mut size: uint16_t)
 -> USBD_StatusTypeDef {
    HAL_PCD_EP_Receive((*pdev).pData as *mut PCD_HandleTypeDef, ep_addr, pbuf,
                       size as uint32_t);
    return USBD_OK;
}
/* *
  * @brief  Returns the last transferred packet size.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Received Data Size
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_GetRxDataSize(mut pdev:
                                                   *mut USBD_HandleTypeDef,
                                               mut ep_addr: uint8_t)
 -> uint32_t {
    return HAL_PCD_EP_GetRxCount((*pdev).pData as *mut PCD_HandleTypeDef,
                                 ep_addr) as uint32_t;
}
/* *
  ******************************************************************************
  * @file    usbd_core.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   Header file for usbd_core.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
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
/* * @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */
/* * @defgroup USBD_CORE
  * @brief This file is the Header file for usbd_core.c file
  * @{
  */
/* * @defgroup USBD_CORE_Exported_Defines
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USBD_CORE_Exported_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USBD_CORE_Exported_Variables
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USBD_CORE_Exported_FunctionsPrototype
  * @{
  */
/* USBD Low Level Driver */
/* *
  * @brief  Delays routine for the USB Device Library.
  * @param  Delay: Delay in ms
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_LL_Delay(mut Delay: uint32_t) {
    HAL_Delay(Delay);
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
