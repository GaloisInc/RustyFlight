use ::libc;
extern "C" {
    /* *
  ******************************************************************************
  * @file    stm32f7xx_hal_pwr.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of PWR HAL module.
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
    /* * @addtogroup PWR
  * @{
  */
    /* Exported types ------------------------------------------------------------*/
    /* * @defgroup PWR_Exported_Types PWR Exported Types
  * @{
  */
    /* *
  * @brief  PWR PVD configuration structure definition
  */
    /* !< PVDLevel: Specifies the PVD detection level.
                            This parameter can be a value of @ref PWR_PVD_detection_level */
    /* !< Mode: Specifies the operating mode for the selected pins.
                           This parameter can be a value of @ref PWR_PVD_Mode */
    /* *
  * @}
  */
    /* Exported constants --------------------------------------------------------*/
/* * @defgroup PWR_Exported_Constants PWR Exported Constants
  * @{
  */
    /* * @defgroup PWR_PVD_detection_level PWR PVD detection level
  * @{
  */
    /* External input analog voltage 
                                                          (Compare internally to VREFINT) */
    /* *
  * @}
  */
    /* * @defgroup PWR_PVD_Mode PWR PVD Mode
  * @{
  */
    /* !< basic mode is used */
    /* !< External Interrupt Mode with Rising edge trigger detection */
    /* !< External Interrupt Mode with Falling edge trigger detection */
    /* !< External Interrupt Mode with Rising/Falling edge trigger detection */
    /* !< Event Mode with Rising edge trigger detection */
    /* !< Event Mode with Falling edge trigger detection */
    /* !< Event Mode with Rising/Falling edge trigger detection */
    /* *
  * @}
  */
    /* * @defgroup PWR_Regulator_state_in_STOP_mode PWR Regulator state in SLEEP/STOP mode
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup PWR_SLEEP_mode_entry PWR SLEEP mode entry
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup PWR_STOP_mode_entry PWR STOP mode entry
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup PWR_Regulator_Voltage_Scale PWR Regulator Voltage Scale
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup PWR_Flag PWR Flag
  * @{
  */
    /* *
  * @}
  */
    /* *
  * @}
  */
    /* Exported macro ------------------------------------------------------------*/
/* * @defgroup PWR_Exported_Macro PWR Exported Macro
  * @{
  */
    /* * @brief  macros configure the main internal regulator output voltage.
  * @param  __REGULATOR__: specifies the regulator output voltage to achieve
  *         a tradeoff between performance and power consumption when the device does
  *         not operate at the maximum frequency (refer to the datasheets for more details).
  *          This parameter can be one of the following values:
  *            @arg PWR_REGULATOR_VOLTAGE_SCALE1: Regulator voltage output Scale 1 mode
  *            @arg PWR_REGULATOR_VOLTAGE_SCALE2: Regulator voltage output Scale 2 mode
  *            @arg PWR_REGULATOR_VOLTAGE_SCALE3: Regulator voltage output Scale 3 mode
  * @retval None
  */
    /* Delay after an RCC peripheral clock enabling */
    /* * @brief  Check PWR flag is set or not.
  * @param  __FLAG__: specifies the flag to check.
  *           This parameter can be one of the following values:
  *            @arg PWR_FLAG_WU: Wake Up flag. This flag indicates that a wakeup event 
  *                  was received on the internal wakeup line in standby mode (RTC alarm (Alarm A or Alarm B),
  *                  RTC Tamper event, RTC TimeStamp event or RTC Wakeup)).
  *            @arg PWR_FLAG_SB: StandBy flag. This flag indicates that the system was
  *                  resumed from StandBy mode.    
  *            @arg PWR_FLAG_PVDO: PVD Output. This flag is valid only if PVD is enabled 
  *                  by the HAL_PWR_EnablePVD() function. The PVD is stopped by Standby mode 
  *                  For this reason, this bit is equal to 0 after Standby or reset
  *                  until the PVDE bit is set.
  *            @arg PWR_FLAG_BRR: Backup regulator ready flag. This bit is not reset 
  *                  when the device wakes up from Standby mode or by a system reset 
  *                  or power reset.  
  *            @arg PWR_FLAG_VOSRDY: This flag indicates that the Regulator voltage 
  *                 scaling output selection is ready.
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
    /* * @brief  Clear the PWR's pending flags.
  * @param  __FLAG__: specifies the flag to clear.
  *          This parameter can be one of the following values:
  *            @arg PWR_FLAG_SB: StandBy flag
  */
    /* *
  * @brief Enable the PVD Exti Line 16.
  * @retval None.
  */
    /* *
  * @brief Disable the PVD EXTI Line 16.
  * @retval None.
  */
    /* *
  * @brief Enable event on PVD Exti Line 16.
  * @retval None.
  */
    /* *
  * @brief Disable event on PVD Exti Line 16.
  * @retval None.
  */
    /* *
  * @brief Enable the PVD Extended Interrupt Rising Trigger.
  * @retval None.
  */
    /* *
  * @brief Disable the PVD Extended Interrupt Rising Trigger.
  * @retval None.
  */
    /* *
  * @brief Enable the PVD Extended Interrupt Falling Trigger.
  * @retval None.
  */
    /* *
  * @brief Disable the PVD Extended Interrupt Falling Trigger.
  * @retval None.
  */
    /* *
  * @brief  PVD EXTI line configuration: set rising & falling edge trigger.
  * @retval None.
  */
    /* *
  * @brief Disable the PVD Extended Interrupt Rising & Falling Trigger.
  * @retval None.
  */
    /* *
  * @brief checks whether the specified PVD Exti interrupt flag is set or not.
  * @retval EXTI PVD Line Status.
  */
    /* *
  * @brief Clear the PVD Exti flag.
  * @retval None.
  */
    /* *
  * @brief  Generates a Software interrupt on PVD EXTI line.
  * @retval None
  */
    /* *
  * @}
  */
    /* Include PWR HAL Extension module */
    /* Exported functions --------------------------------------------------------*/
/* * @addtogroup PWR_Exported_Functions PWR Exported Functions
  * @{
  */
    /* * @addtogroup PWR_Exported_Functions_Group1 Initialization and de-initialization functions 
  * @{
  */
/* Initialization and de-initialization functions *****************************/
    #[no_mangle]
    fn HAL_PWR_EnableBkUpAccess();
    /* *
  * @}
  */
    /* * @defgroup RCC_System_Clock_Type RCC System Clock Type
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_System_Clock_Source RCC System Clock Source
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_System_Clock_Source_Status System Clock Source Status
  * @{
  */
    /* !< HSI used as system clock */
    /* !< HSE used as system clock */
    /* !< PLL used as system clock */
    /* *
  * @}
  */
    /* * @defgroup RCC_AHB_Clock_Source RCC AHB Clock Source
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_APB1_APB2_Clock_Source RCC APB1/APB2 Clock Source
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_RTC_Clock_Source RCC RTC Clock Source
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_MCO_Index RCC MCO Index
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_MCO1_Clock_Source RCC MCO1 Clock Source
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_MCO2_Clock_Source RCC MCO2 Clock Source
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_MCOx_Clock_Prescaler RCC MCO1 Clock Prescaler
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_Interrupt RCC Interrupt 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_Flag RCC Flags
  *        Elements values convention: 0XXYYYYYb
  *           - YYYYY  : Flag position in the register
  *           - 0XX  : Register index
  *                 - 01: CR register
  *                 - 10: BDCR register
  *                 - 11: CSR register
  * @{
  */
/* Flags in the CR register */
    /* Flags in the BDCR register */
    /* Flags in the CSR register */
    /* *
  * @}
  */
    /* * @defgroup RCC_LSEDrive_Configuration RCC LSE Drive configurations
  * @{
  */
    /* *
  * @}
  */
    /* *
  * @}
  */
    /* Exported macro ------------------------------------------------------------*/
/* * @defgroup RCC_Exported_Macros RCC Exported Macros
  * @{
  */
    /* * @defgroup RCC_AHB1_Clock_Enable_Disable AHB1 Peripheral Clock Enable Disable
  * @brief  Enable or disable the AHB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.   
  * @{
  */
    /* Delay after an RCC peripheral clock enabling */
    /* Delay after an RCC peripheral clock enabling */
    /* *
  * @}
  */
    /* * @defgroup RCC_APB1_Clock_Enable_Disable APB1 Peripheral Clock Enable Disable
  * @brief  Enable or disable the Low Speed APB (APB1) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
    /* Delay after an RCC peripheral clock enabling */
    /* Delay after an RCC peripheral clock enabling */
    /* *
  * @}
  */
    /* * @defgroup RCC_APB2_Clock_Enable_Disable APB2 Peripheral Clock Enable Disable                                      
  * @brief  Enable or disable the High Speed APB (APB2) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  * @{
  */
    /* Delay after an RCC peripheral clock enabling */
    /* *
  * @}
  */
    /* * @defgroup RCC_AHB1_Peripheral_Clock_Enable_Disable_Status AHB1 Peripheral Clock Enable Disable Status
  * @brief  Get the enable or disable status of the AHB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_APB1_Clock_Enable_Disable_Status APB1 Peripheral Clock Enable Disable  Status
  * @brief  Get the enable or disable status of the APB1 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_APB2_Clock_Enable_Disable_Status APB2 Peripheral Clock Enable Disable Status
  * @brief  EGet the enable or disable status of the APB2 peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before
  *         using it.
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_Peripheral_Clock_Force_Release RCC Peripheral Clock Force Release
  * @brief  Force or release AHB peripheral reset.
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_APB1_Force_Release_Reset APB1 Force Release Reset 
  * @brief  Force or release APB1 peripheral reset.
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_APB2_Force_Release_Reset APB2 Force Release Reset 
  * @brief  Force or release APB2 peripheral reset.
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_Peripheral_Clock_Sleep_Enable_Disable RCC Peripheral Clock Sleep Enable Disable
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
    /* * @brief  Enable or disable the APB1 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  */
    /* * @brief  Enable or disable the APB2 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_AHB1_Clock_Sleep_Enable_Disable_Status AHB1 Peripheral Clock Sleep Enable Disable Status
  * @brief  Get the enable or disable status of the AHB1 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_APB1_Clock_Sleep_Enable_Disable_Status APB1 Peripheral Clock Sleep Enable Disable Status
  * @brief  Get the enable or disable status of the APB1 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_APB2_Clock_Sleep_Enable_Disable_Status APB2 Peripheral Clock Sleep Enable Disable Status
  * @brief  Get the enable or disable status of the APB2 peripheral clock during Low Power (Sleep) mode.
  * @note   Peripheral clock gating in SLEEP mode can be used to further reduce
  *         power consumption.
  * @note   After wakeup from SLEEP mode, the peripheral clock is enabled again.
  * @note   By default, all peripheral clocks are enabled during SLEEP mode.
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_HSI_Configuration HSI Configuration
  * @{   
  */
    /* * @brief  Macros to enable or disable the Internal High Speed oscillator (HSI).
  * @note   The HSI is stopped by hardware when entering STOP and STANDBY modes.
  *         It is used (enabled by hardware) as system clock source after startup
  *         from Reset, wakeup from STOP and STANDBY mode, or in case of failure
  *         of the HSE used directly or indirectly as system clock (if the Clock
  *         Security System CSS is enabled).             
  * @note   HSI can not be stopped if it is used as system clock source. In this case,
  *         you have to select another source of the system clock then stop the HSI.  
  * @note   After enabling the HSI, the application software should wait on HSIRDY
  *         flag to be set indicating that HSI clock is stable and can be used as
  *         system clock source.  
  * @note   When the HSI is stopped, HSIRDY flag goes low after 6 HSI oscillator
  *         clock cycles.  
  */
    /* * @brief  Macro to adjust the Internal High Speed oscillator (HSI) calibration value.
  * @note   The calibration is used to compensate for the variations in voltage
  *         and temperature that influence the frequency of the internal HSI RC.
  * @param  __HSICALIBRATIONVALUE__: specifies the calibration trimming value.
  *         (default is RCC_HSICALIBRATION_DEFAULT).
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_LSI_Configuration LSI Configuration
  * @{   
  */
    /* * @brief  Macros to enable or disable the Internal Low Speed oscillator (LSI).
  * @note   After enabling the LSI, the application software should wait on 
  *         LSIRDY flag to be set indicating that LSI clock is stable and can
  *         be used to clock the IWDG and/or the RTC.
  * @note   LSI can not be disabled if the IWDG is running.
  * @note   When the LSI is stopped, LSIRDY flag goes low after 6 LSI oscillator
  *         clock cycles. 
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_HSE_Configuration HSE Configuration
  * @{   
  */ 
/* *
  * @brief  Macro to configure the External High Speed oscillator (HSE).
  * @note   Transitions HSE Bypass to HSE On and HSE On to HSE Bypass are not
  *         supported by this macro. User should request a transition to HSE Off
  *         first and then HSE On or HSE Bypass.
  * @note   After enabling the HSE (RCC_HSE_ON or RCC_HSE_Bypass), the application
  *         software should wait on HSERDY flag to be set indicating that HSE clock
  *         is stable and can be used to clock the PLL and/or system clock.
  * @note   HSE state can not be changed if it is used directly or through the
  *         PLL as system clock. In this case, you have to select another source
  *         of the system clock then change the HSE state (ex. disable it).
  * @note   The HSE is stopped by hardware when entering STOP and STANDBY modes.
  * @note   This function reset the CSSON bit, so if the clock security system(CSS)
  *         was previously enabled you have to enable it again after calling this
  *         function.
  * @param  __STATE__: specifies the new state of the HSE.
  *         This parameter can be one of the following values:
  *            @arg RCC_HSE_OFF: turn OFF the HSE oscillator, HSERDY flag goes low after
  *                              6 HSE oscillator clock cycles.
  *            @arg RCC_HSE_ON: turn ON the HSE oscillator.
  *            @arg RCC_HSE_BYPASS: HSE oscillator bypassed with external clock.
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_LSE_Configuration LSE Configuration
  * @{   
  */
    /* *
  * @brief  Macro to configure the External Low Speed oscillator (LSE).
  * @note   Transitions LSE Bypass to LSE On and LSE On to LSE Bypass are not supported by this macro. 
  *         User should request a transition to LSE Off first and then LSE On or LSE Bypass.  
  * @note   As the LSE is in the Backup domain and write access is denied to
  *         this domain after reset, you have to enable write access using 
  *         HAL_PWR_EnableBkUpAccess() function before to configure the LSE
  *         (to be done once after reset).  
  * @note   After enabling the LSE (RCC_LSE_ON or RCC_LSE_BYPASS), the application
  *         software should wait on LSERDY flag to be set indicating that LSE clock
  *         is stable and can be used to clock the RTC.
  * @param  __STATE__: specifies the new state of the LSE.
  *         This parameter can be one of the following values:
  *            @arg RCC_LSE_OFF: turn OFF the LSE oscillator, LSERDY flag goes low after
  *                              6 LSE oscillator clock cycles.
  *            @arg RCC_LSE_ON: turn ON the LSE oscillator.
  *            @arg RCC_LSE_BYPASS: LSE oscillator bypassed with external clock.
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_Internal_RTC_Clock_Configuration RTC Clock Configuration
  * @{   
  */
    /* * @brief  Macros to enable or disable the RTC clock.
  * @note   These macros must be used only after the RTC clock source was selected.
  */
    /* * @brief  Macros to configure the RTC clock (RTCCLK).
  * @note   As the RTC clock configuration bits are in the Backup domain and write
  *         access is denied to this domain after reset, you have to enable write
  *         access using the Power Backup Access macro before to configure
  *         the RTC clock source (to be done once after reset).    
  * @note   Once the RTC clock is configured it can't be changed unless the  
  *         Backup domain is reset using __HAL_RCC_BackupReset_RELEASE() macro, or by
  *         a Power On Reset (POR).
  * @param  __RTCCLKSource__: specifies the RTC clock source.
  *         This parameter can be one of the following values:
  *            @arg RCC_RTCCLKSOURCE_LSE: LSE selected as RTC clock.
  *            @arg RCC_RTCCLKSOURCE_LSI: LSI selected as RTC clock.
  *            @arg RCC_RTCCLKSOURCE_HSE_DIVx: HSE clock divided by x selected
  *                                            as RTC clock, where x:[2,31]
  * @note   If the LSE or LSI is used as RTC clock source, the RTC continues to
  *         work in STOP and STANDBY modes, and can be used as wakeup source.
  *         However, when the HSE clock is used as RTC clock source, the RTC
  *         cannot be used in STOP and STANDBY modes.    
  * @note   The maximum input clock frequency for RTC is 1MHz (when using HSE as
  *         RTC clock source).
  */
    /* * @brief  Macros to force or release the Backup domain reset.
  * @note   This function resets the RTC peripheral (including the backup registers)
  *         and the RTC clock source selection in RCC_CSR register.
  * @note   The BKPSRAM is not affected by this reset.   
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_PLL_Configuration PLL Configuration
  * @{   
  */
    /* * @brief  Macros to enable or disable the main PLL.
  * @note   After enabling the main PLL, the application software should wait on 
  *         PLLRDY flag to be set indicating that PLL clock is stable and can
  *         be used as system clock source.
  * @note   The main PLL can not be disabled if it is used as system clock source
  * @note   The main PLL is disabled by hardware when entering STOP and STANDBY modes.
  */
    /* * @brief  Macro to configure the PLL clock source.
  * @note   This function must be used only when the main PLL is disabled.
  * @param  __PLLSOURCE__: specifies the PLL entry clock source.
  *         This parameter can be one of the following values:
  *            @arg RCC_PLLSOURCE_HSI: HSI oscillator clock selected as PLL clock entry
  *            @arg RCC_PLLSOURCE_HSE: HSE oscillator clock selected as PLL clock entry
  *      
  */
    /* * @brief  Macro to configure the PLL multiplication factor.
  * @note   This function must be used only when the main PLL is disabled.
  * @param  __PLLM__: specifies the division factor for PLL VCO input clock
  *         This parameter must be a number between Min_Data = 2 and Max_Data = 63.
  * @note   You have to set the PLLM parameter correctly to ensure that the VCO input
  *         frequency ranges from 1 to 2 MHz. It is recommended to select a frequency
  *         of 2 MHz to limit PLL jitter.
  *      
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_PLL_I2S_Configuration PLL I2S Configuration
  * @{   
  */
    /* * @brief  Macro to configure the I2S clock source (I2SCLK).
  * @note   This function must be called before enabling the I2S APB clock.
  * @param  __SOURCE__: specifies the I2S clock source.
  *         This parameter can be one of the following values:
  *            @arg RCC_I2SCLKSOURCE_PLLI2S: PLLI2S clock used as I2S clock source.
  *            @arg RCC_I2SCLKSOURCE_EXT: External clock mapped on the I2S_CKIN pin
  *                                       used as I2S clock source.
  */
    /* * @brief Macros to enable or disable the PLLI2S. 
  * @note  The PLLI2S is disabled by hardware when entering STOP and STANDBY modes.
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_Get_Clock_source Get Clock source
  * @{   
  */
/* *
  * @brief Macro to configure the system clock source.
  * @param __RCC_SYSCLKSOURCE__: specifies the system clock source.
  * This parameter can be one of the following values:
  *              - RCC_SYSCLKSOURCE_HSI: HSI oscillator is used as system clock source.
  *              - RCC_SYSCLKSOURCE_HSE: HSE oscillator is used as system clock source.
  *              - RCC_SYSCLKSOURCE_PLLCLK: PLL output is used as system clock source.
  */
    /* * @brief  Macro to get the clock source used as system clock.
  * @retval The clock source used as system clock. The returned value can be one
  *         of the following:
  *              - RCC_SYSCLKSOURCE_STATUS_HSI: HSI used as system clock.
  *              - RCC_SYSCLKSOURCE_STATUS_HSE: HSE used as system clock.
  *              - RCC_SYSCLKSOURCE_STATUS_PLLCLK: PLL used as system clock.
  */
    /* *
  * @brief  Macro to configures the External Low Speed oscillator (LSE) drive capability.
  * @note   As the LSE is in the Backup domain and write access is denied to
  *         this domain after reset, you have to enable write access using
  *         HAL_PWR_EnableBkUpAccess() function before to configure the LSE
  *         (to be done once after reset).
  * @param  __RCC_LSEDRIVE__: specifies the new state of the LSE drive capability.
  *          This parameter can be one of the following values:
  *            @arg RCC_LSEDRIVE_LOW: LSE oscillator low drive capability.
  *            @arg RCC_LSEDRIVE_MEDIUMLOW: LSE oscillator medium low drive capability.
  *            @arg RCC_LSEDRIVE_MEDIUMHIGH: LSE oscillator medium high drive capability.
  *            @arg RCC_LSEDRIVE_HIGH: LSE oscillator high drive capability.
  * @retval None
  */
    /* * @brief  Macro to get the oscillator used as PLL clock source.
  * @retval The oscillator used as PLL clock source. The returned value can be one
  *         of the following:
  *              - RCC_PLLSOURCE_HSI: HSI oscillator is used as PLL clock source.
  *              - RCC_PLLSOURCE_HSE: HSE oscillator is used as PLL clock source.
  */
    /* *
  * @}
  */
    /* * @defgroup RCCEx_MCOx_Clock_Config RCC Extended MCOx Clock Config
  * @{   
  */
    /* * @brief  Macro to configure the MCO1 clock.
  * @param  __MCOCLKSOURCE__ specifies the MCO clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_MCO1SOURCE_HSI: HSI clock selected as MCO1 source
  *            @arg RCC_MCO1SOURCE_LSE: LSE clock selected as MCO1 source
  *            @arg RCC_MCO1SOURCE_HSE: HSE clock selected as MCO1 source
  *            @arg RCC_MCO1SOURCE_PLLCLK: main PLL clock selected as MCO1 source
  * @param  __MCODIV__ specifies the MCO clock prescaler.
  *          This parameter can be one of the following values:
  *            @arg RCC_MCODIV_1: no division applied to MCOx clock
  *            @arg RCC_MCODIV_2: division by 2 applied to MCOx clock
  *            @arg RCC_MCODIV_3: division by 3 applied to MCOx clock
  *            @arg RCC_MCODIV_4: division by 4 applied to MCOx clock
  *            @arg RCC_MCODIV_5: division by 5 applied to MCOx clock
  */
    /* * @brief  Macro to configure the MCO2 clock.
  * @param  __MCOCLKSOURCE__ specifies the MCO clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_MCO2SOURCE_SYSCLK: System clock (SYSCLK) selected as MCO2 source
  *            @arg RCC_MCO2SOURCE_PLLI2SCLK: PLLI2S clock selected as MCO2 source 
  *            @arg RCC_MCO2SOURCE_HSE: HSE clock selected as MCO2 source
  *            @arg RCC_MCO2SOURCE_PLLCLK: main PLL clock selected as MCO2 source
  * @param  __MCODIV__ specifies the MCO clock prescaler.
  *          This parameter can be one of the following values:
  *            @arg RCC_MCODIV_1: no division applied to MCOx clock
  *            @arg RCC_MCODIV_2: division by 2 applied to MCOx clock
  *            @arg RCC_MCODIV_3: division by 3 applied to MCOx clock
  *            @arg RCC_MCODIV_4: division by 4 applied to MCOx clock
  *            @arg RCC_MCODIV_5: division by 5 applied to MCOx clock
  */
    /* *
  * @}
  */
    /* * @defgroup RCC_Flags_Interrupts_Management Flags Interrupts Management
  * @brief macros to manage the specified RCC Flags and interrupts.
  * @{
  */
    /* * @brief  Enable RCC interrupt (Perform Byte access to RCC_CIR[14:8] bits to enable
  *         the selected interrupts).
  * @param  __INTERRUPT__: specifies the RCC interrupt sources to be enabled.
  *         This parameter can be any combination of the following values:
  *            @arg RCC_IT_LSIRDY: LSI ready interrupt.
  *            @arg RCC_IT_LSERDY: LSE ready interrupt.
  *            @arg RCC_IT_HSIRDY: HSI ready interrupt.
  *            @arg RCC_IT_HSERDY: HSE ready interrupt.
  *            @arg RCC_IT_PLLRDY: Main PLL ready interrupt.
  *            @arg RCC_IT_PLLI2SRDY: PLLI2S ready interrupt.
  */
    /* * @brief Disable RCC interrupt (Perform Byte access to RCC_CIR[14:8] bits to disable 
  *        the selected interrupts).
  * @param  __INTERRUPT__: specifies the RCC interrupt sources to be disabled.
  *         This parameter can be any combination of the following values:
  *            @arg RCC_IT_LSIRDY: LSI ready interrupt.
  *            @arg RCC_IT_LSERDY: LSE ready interrupt.
  *            @arg RCC_IT_HSIRDY: HSI ready interrupt.
  *            @arg RCC_IT_HSERDY: HSE ready interrupt.
  *            @arg RCC_IT_PLLRDY: Main PLL ready interrupt.
  *            @arg RCC_IT_PLLI2SRDY: PLLI2S ready interrupt.
  */
    /* * @brief  Clear the RCC's interrupt pending bits (Perform Byte access to RCC_CIR[23:16]
  *         bits to clear the selected interrupt pending bits.
  * @param  __INTERRUPT__: specifies the interrupt pending bit to clear.
  *         This parameter can be any combination of the following values:
  *            @arg RCC_IT_LSIRDY: LSI ready interrupt.
  *            @arg RCC_IT_LSERDY: LSE ready interrupt.
  *            @arg RCC_IT_HSIRDY: HSI ready interrupt.
  *            @arg RCC_IT_HSERDY: HSE ready interrupt.
  *            @arg RCC_IT_PLLRDY: Main PLL ready interrupt.
  *            @arg RCC_IT_PLLI2SRDY: PLLI2S ready interrupt.  
  *            @arg RCC_IT_CSS: Clock Security System interrupt
  */
    /* * @brief  Check the RCC's interrupt has occurred or not.
  * @param  __INTERRUPT__: specifies the RCC interrupt source to check.
  *         This parameter can be one of the following values:
  *            @arg RCC_IT_LSIRDY: LSI ready interrupt.
  *            @arg RCC_IT_LSERDY: LSE ready interrupt.
  *            @arg RCC_IT_HSIRDY: HSI ready interrupt.
  *            @arg RCC_IT_HSERDY: HSE ready interrupt.
  *            @arg RCC_IT_PLLRDY: Main PLL ready interrupt.
  *            @arg RCC_IT_PLLI2SRDY: PLLI2S ready interrupt.
  *            @arg RCC_IT_CSS: Clock Security System interrupt
  * @retval The new state of __INTERRUPT__ (TRUE or FALSE).
  */
    /* * @brief Set RMVF bit to clear the reset flags: RCC_FLAG_PINRST, RCC_FLAG_PORRST, 
  *        RCC_FLAG_SFTRST, RCC_FLAG_IWDGRST, RCC_FLAG_WWDGRST and RCC_FLAG_LPWRRST.
  */
    /* * @brief  Check RCC flag is set or not.
  * @param  __FLAG__: specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg RCC_FLAG_HSIRDY: HSI oscillator clock ready.
  *            @arg RCC_FLAG_HSERDY: HSE oscillator clock ready.
  *            @arg RCC_FLAG_PLLRDY: Main PLL clock ready.
  *            @arg RCC_FLAG_PLLI2SRDY: PLLI2S clock ready.
  *            @arg RCC_FLAG_LSERDY: LSE oscillator clock ready.
  *            @arg RCC_FLAG_LSIRDY: LSI oscillator clock ready.
  *            @arg RCC_FLAG_BORRST: POR/PDR or BOR reset.
  *            @arg RCC_FLAG_PINRST: Pin reset.
  *            @arg RCC_FLAG_PORRST: POR/PDR reset.
  *            @arg RCC_FLAG_SFTRST: Software reset.
  *            @arg RCC_FLAG_IWDGRST: Independent Watchdog reset.
  *            @arg RCC_FLAG_WWDGRST: Window Watchdog reset.
  *            @arg RCC_FLAG_LPWRRST: Low Power reset.
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
    /* *
  * @}
  */
    /* *
  * @}
  */
    /* Include RCC HAL Extension module */
    /* Exported functions --------------------------------------------------------*/
 /* * @addtogroup RCC_Exported_Functions
  * @{
  */
    /* * @addtogroup RCC_Exported_Functions_Group1
  * @{
  */                             
/* Initialization and de-initialization functions  ******************************/
    #[no_mangle]
    fn HAL_RCC_OscConfig(RCC_OscInitStruct: *mut RCC_OscInitTypeDef)
     -> HAL_StatusTypeDef;
    /* *
  ******************************************************************************
  * @file    stm32f7xx_hal_pwr_ex.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of PWR HAL Extension module.
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
    /* * @addtogroup PWREx
  * @{
  */
    /* Exported types ------------------------------------------------------------*/ 
/* Exported constants --------------------------------------------------------*/
/* * @defgroup PWREx_Exported_Constants PWREx Exported Constants
  * @{
  */
/* * @defgroup PWREx_WakeUp_Pins PWREx Wake Up Pins
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup PWREx_Regulator_state_in_UnderDrive_mode PWREx Regulator state in UnderDrive mode
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup PWREx_Over_Under_Drive_Flag PWREx Over Under Drive Flag
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup PWREx_Wakeup_Pins_Flag PWREx Wake Up Pin Flags
  * @{
  */
    /* *
  * @}
  */
    /* *
  * @}
  */
    /* Exported macro ------------------------------------------------------------*/
/* * @defgroup PWREx_Exported_Macro PWREx Exported Macro
  *  @{
  */
/* * @brief Macros to enable or disable the Over drive mode.
  */
    /* * @brief Macros to enable or disable the Over drive switching.
  */
    /* * @brief Macros to enable or disable the Under drive mode.
  * @note  This mode is enabled only with STOP low power mode.
  *        In this mode, the 1.2V domain is preserved in reduced leakage mode. This 
  *        mode is only available when the main regulator or the low power regulator 
  *        is in low voltage mode.      
  * @note  If the Under-drive mode was enabled, it is automatically disabled after 
  *        exiting Stop mode. 
  *        When the voltage regulator operates in Under-drive mode, an additional  
  *        startup delay is induced when waking up from Stop mode.
  */
    /* * @brief  Check PWR flag is set or not.
  * @param  __FLAG__: specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg PWR_FLAG_ODRDY: This flag indicates that the Over-drive mode
  *                                 is ready 
  *            @arg PWR_FLAG_ODSWRDY: This flag indicates that the Over-drive mode
  *                                   switching is ready  
  *            @arg PWR_FLAG_UDRDY: This flag indicates that the Under-drive mode
  *                                 is enabled in Stop mode
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
    /* * @brief Clear the Under-Drive Ready flag.
  */
    /* * @brief  Check Wake Up flag is set or not.
  * @param  __WUFLAG__: specifies the Wake Up flag to check.
  *          This parameter can be one of the following values:
  *            @arg PWR_WAKEUP_PIN_FLAG1: Wakeup Pin Flag for PA0
  *            @arg PWR_WAKEUP_PIN_FLAG2: Wakeup Pin Flag for PA2
  *            @arg PWR_WAKEUP_PIN_FLAG3: Wakeup Pin Flag for PC1
  *            @arg PWR_WAKEUP_PIN_FLAG4: Wakeup Pin Flag for PC13
  *            @arg PWR_WAKEUP_PIN_FLAG5: Wakeup Pin Flag for PI8
  *            @arg PWR_WAKEUP_PIN_FLAG6: Wakeup Pin Flag for PI11          
  */
    /* * @brief  Clear the WakeUp pins flags.
  * @param  __WUFLAG__: specifies the Wake Up pin flag to clear.
  *          This parameter can be one of the following values:
  *            @arg PWR_WAKEUP_PIN_FLAG1: Wakeup Pin Flag for PA0
  *            @arg PWR_WAKEUP_PIN_FLAG2: Wakeup Pin Flag for PA2
  *            @arg PWR_WAKEUP_PIN_FLAG3: Wakeup Pin Flag for PC1
  *            @arg PWR_WAKEUP_PIN_FLAG4: Wakeup Pin Flag for PC13
  *            @arg PWR_WAKEUP_PIN_FLAG5: Wakeup Pin Flag for PI8
  *            @arg PWR_WAKEUP_PIN_FLAG6: Wakeup Pin Flag for PI11          
  */
    /* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup PWREx_Exported_Functions PWREx Exported Functions
  *  @{
  */
    /* * @addtogroup PWREx_Exported_Functions_Group1
  * @{
  */
    #[no_mangle]
    fn HAL_PWREx_EnableOverDrive() -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_RCC_ClockConfig(RCC_ClkInitStruct: *mut RCC_ClkInitTypeDef,
                           FLatency: uint32_t) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_RCCEx_PeriphCLKConfig(PeriphClkInit: *mut RCC_PeriphCLKInitTypeDef)
     -> HAL_StatusTypeDef;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SCB_Type {
    pub CPUID: uint32_t,
    pub ICSR: uint32_t,
    pub VTOR: uint32_t,
    pub AIRCR: uint32_t,
    pub SCR: uint32_t,
    pub CCR: uint32_t,
    pub SHPR: [uint8_t; 12],
    pub SHCSR: uint32_t,
    pub CFSR: uint32_t,
    pub HFSR: uint32_t,
    pub DFSR: uint32_t,
    pub MMFAR: uint32_t,
    pub BFAR: uint32_t,
    pub AFSR: uint32_t,
    pub ID_PFR: [uint32_t; 2],
    pub ID_DFR: uint32_t,
    pub ID_AFR: uint32_t,
    pub ID_MFR: [uint32_t; 4],
    pub ID_ISAR: [uint32_t; 5],
    pub RESERVED0: [uint32_t; 1],
    pub CLIDR: uint32_t,
    pub CTR: uint32_t,
    pub CCSIDR: uint32_t,
    pub CSSELR: uint32_t,
    pub CPACR: uint32_t,
    pub RESERVED3: [uint32_t; 93],
    pub STIR: uint32_t,
    pub RESERVED4: [uint32_t; 15],
    pub MVFR0: uint32_t,
    pub MVFR1: uint32_t,
    pub MVFR2: uint32_t,
    pub RESERVED5: [uint32_t; 1],
    pub ICIALLU: uint32_t,
    pub RESERVED6: [uint32_t; 1],
    pub ICIMVAU: uint32_t,
    pub DCIMVAC: uint32_t,
    pub DCISW: uint32_t,
    pub DCCMVAU: uint32_t,
    pub DCCMVAC: uint32_t,
    pub DCCSW: uint32_t,
    pub DCCIMVAC: uint32_t,
    pub DCCISW: uint32_t,
    pub RESERVED7: [uint32_t; 6],
    pub ITCMCR: uint32_t,
    pub DTCMCR: uint32_t,
    pub AHBPCR: uint32_t,
    pub CACR: uint32_t,
    pub AHBSCR: uint32_t,
    pub RESERVED8: [uint32_t; 1],
    pub ABFSR: uint32_t,
}
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
pub const HAL_OK: HAL_StatusTypeDef = 0;
pub type HAL_StatusTypeDef = libc::c_uint;
pub const HAL_TIMEOUT: HAL_StatusTypeDef = 3;
pub const HAL_BUSY: HAL_StatusTypeDef = 2;
pub const HAL_ERROR: HAL_StatusTypeDef = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RCC_PeriphCLKInitTypeDef {
    pub PeriphClockSelection: uint32_t,
    pub PLLI2S: RCC_PLLI2SInitTypeDef,
    pub PLLSAI: RCC_PLLSAIInitTypeDef,
    pub PLLI2SDivQ: uint32_t,
    pub PLLSAIDivQ: uint32_t,
    pub PLLSAIDivR: uint32_t,
    pub RTCClockSelection: uint32_t,
    pub I2sClockSelection: uint32_t,
    pub TIMPresSelection: uint32_t,
    pub Sai1ClockSelection: uint32_t,
    pub Sai2ClockSelection: uint32_t,
    pub Usart1ClockSelection: uint32_t,
    pub Usart2ClockSelection: uint32_t,
    pub Usart3ClockSelection: uint32_t,
    pub Uart4ClockSelection: uint32_t,
    pub Uart5ClockSelection: uint32_t,
    pub Usart6ClockSelection: uint32_t,
    pub Uart7ClockSelection: uint32_t,
    pub Uart8ClockSelection: uint32_t,
    pub I2c1ClockSelection: uint32_t,
    pub I2c2ClockSelection: uint32_t,
    pub I2c3ClockSelection: uint32_t,
    pub I2c4ClockSelection: uint32_t,
    pub Lptim1ClockSelection: uint32_t,
    pub CecClockSelection: uint32_t,
    pub Clk48ClockSelection: uint32_t,
    pub Sdmmc1ClockSelection: uint32_t,
    pub Sdmmc2ClockSelection: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RCC_PLLSAIInitTypeDef {
    pub PLLSAIN: uint32_t,
    pub PLLSAIQ: uint32_t,
    pub PLLSAIP: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RCC_PLLI2SInitTypeDef {
    pub PLLI2SN: uint32_t,
    pub PLLI2SR: uint32_t,
    pub PLLI2SQ: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RCC_ClkInitTypeDef {
    pub ClockType: uint32_t,
    pub SYSCLKSource: uint32_t,
    pub AHBCLKDivider: uint32_t,
    pub APB1CLKDivider: uint32_t,
    pub APB2CLKDivider: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RCC_OscInitTypeDef {
    pub OscillatorType: uint32_t,
    pub HSEState: uint32_t,
    pub LSEState: uint32_t,
    pub HSIState: uint32_t,
    pub HSICalibrationValue: uint32_t,
    pub LSIState: uint32_t,
    pub PLL: RCC_PLLInitTypeDef,
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_rcc_ex.h
  * @author  MCD Application Team                                                                                                     
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of RCC HAL Extension module.
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
/* * @addtogroup RCCEx
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup RCCEx_Exported_Types RCCEx Exported Types
  * @{
  */
/* * 
  * @brief  RCC PLL configuration structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RCC_PLLInitTypeDef {
    pub PLLState: uint32_t,
    pub PLLSource: uint32_t,
    pub PLLM: uint32_t,
    pub PLLN: uint32_t,
    pub PLLP: uint32_t,
    pub PLLQ: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PWR_TypeDef {
    pub CR1: uint32_t,
    pub CSR1: uint32_t,
    pub CR2: uint32_t,
    pub CSR2: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FLASH_TypeDef {
    pub ACR: uint32_t,
    pub KEYR: uint32_t,
    pub OPTKEYR: uint32_t,
    pub SR: uint32_t,
    pub CR: uint32_t,
    pub OPTCR: uint32_t,
    pub OPTCR1: uint32_t,
    pub OPTCR2: uint32_t,
}
pub type assert_failed_isr_vector_table_base_is_not_512_byte_aligned
    =
    *mut libc::c_char;
pub type pllConfig_t = pllConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pllConfig_s {
    pub n: uint16_t,
    pub p: uint16_t,
    pub q: uint16_t,
}
#[inline(always)]
unsafe extern "C" fn __disable_irq() {
    asm!("cpsid i" : : : "memory" : "volatile");
}
#[inline(always)]
unsafe extern "C" fn __ISB() { asm!("isb 0xF" : : : "memory" : "volatile"); }
#[inline(always)]
unsafe extern "C" fn __DSB() { asm!("dsb 0xF" : : : "memory" : "volatile"); }
#[inline]
unsafe extern "C" fn SCB_EnableDCache() {
    let mut ccsidr: uint32_t = 0;
    let mut sets: uint32_t = 0;
    let mut ways: uint32_t = 0;
    ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                             libc::c_ulong).wrapping_add(0xd00
                                                                             as
                                                                             libc::c_ulong)
                                            as *mut SCB_Type)).CSSELR as
                                    *mut uint32_t, 0 as libc::c_uint);
    __DSB();
    ccsidr =
        (*((0xe000e000 as libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong)
               as *mut SCB_Type)).CCSIDR;
    sets =
        ((ccsidr as libc::c_ulong &
              (0x7fff as libc::c_ulong) << 13 as libc::c_uint) >>
             13 as libc::c_uint) as uint32_t;
    loop  {
        ways =
            ((ccsidr as libc::c_ulong &
                  (0x3ff as libc::c_ulong) << 3 as libc::c_uint) >>
                 3 as libc::c_uint) as uint32_t;
        loop  {
            ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                                     libc::c_ulong).wrapping_add(0xd00
                                                                                     as
                                                                                     libc::c_ulong)
                                                    as *mut SCB_Type)).DCISW
                                            as *mut uint32_t,
                                        ((sets << 5 as libc::c_uint) as
                                             libc::c_ulong &
                                             (0x1ff as libc::c_ulong) <<
                                                 5 as libc::c_uint |
                                             (ways << 30 as libc::c_uint) as
                                                 libc::c_ulong &
                                                 (3 as libc::c_ulong) <<
                                                     30 as libc::c_uint) as
                                            uint32_t);
            let fresh0 = ways;
            ways = ways.wrapping_sub(1);
            if !(fresh0 != 0 as libc::c_uint) { break ; }
        }
        let fresh1 = sets;
        sets = sets.wrapping_sub(1);
        if !(fresh1 != 0 as libc::c_uint) { break ; }
    }
    __DSB();
    let ref mut fresh2 =
        (*((0xe000e000 as libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong)
               as *mut SCB_Type)).CCR;
    ::core::ptr::write_volatile(fresh2,
                                (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((1 as libc::c_ulong) <<
                                          16 as libc::c_uint) as uint32_t) as
                                    uint32_t as uint32_t);
    __DSB();
    __ISB();
}
#[inline]
unsafe extern "C" fn SCB_EnableICache() {
    __DSB();
    __ISB();
    ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                             libc::c_ulong).wrapping_add(0xd00
                                                                             as
                                                                             libc::c_ulong)
                                            as *mut SCB_Type)).ICIALLU as
                                    *mut uint32_t,
                                0 as libc::c_ulong as uint32_t);
    __DSB();
    __ISB();
    let ref mut fresh3 =
        (*((0xe000e000 as libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong)
               as *mut SCB_Type)).CCR;
    ::core::ptr::write_volatile(fresh3,
                                (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((1 as libc::c_ulong) <<
                                          17 as libc::c_uint) as uint32_t) as
                                    uint32_t as uint32_t);
    __DSB();
    __ISB();
}
#[inline]
unsafe extern "C" fn __NVIC_SystemReset() {
    __DSB();
    ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                             libc::c_ulong).wrapping_add(0xd00
                                                                             as
                                                                             libc::c_ulong)
                                            as *mut SCB_Type)).AIRCR as
                                    *mut uint32_t,
                                ((0x5fa as libc::c_ulong) <<
                                     16 as libc::c_uint |
                                     (*((0xe000e000 as
                                             libc::c_ulong).wrapping_add(0xd00
                                                                             as
                                                                             libc::c_ulong)
                                            as *mut SCB_Type)).AIRCR as
                                         libc::c_ulong &
                                         (7 as libc::c_ulong) <<
                                             8 as libc::c_uint |
                                     (1 as libc::c_ulong) <<
                                         2 as libc::c_uint) as uint32_t);
    __DSB();
    loop  { asm!("nop" : : : : "volatile") };
}
#[inline]
unsafe extern "C" fn LL_FLASH_EnablePrefetch() {
    let ref mut fresh4 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).ACR;
    ::core::ptr::write_volatile(fresh4,
                                (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         8 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @}
  */
/* * @addtogroup STM32F7xx_System_Private_TypesDefinitions
  * @{
  */
/* *
  * @}
  */
/* * @addtogroup STM32F7xx_System_Private_Defines
  * @{
  */
/* *
  * @}
  */
/* * @addtogroup STM32F7xx_System_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* * @addtogroup STM32F7xx_System_Private_Variables
  * @{
  */
/* This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetHCLKFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
  */
#[no_mangle]
pub static mut SystemCoreClock: uint32_t = 0;
#[no_mangle]
pub static mut pll_p: uint32_t = 0x2 as libc::c_uint;
#[no_mangle]
pub static mut pll_n: uint32_t = 432 as libc::c_int as uint32_t;
#[no_mangle]
pub static mut pll_q: uint32_t = 9 as libc::c_int as uint32_t;
#[no_mangle]
pub static mut AHBPrescTable: [uint8_t; 16] =
    [0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     1 as libc::c_int as uint8_t, 2 as libc::c_int as uint8_t,
     3 as libc::c_int as uint8_t, 4 as libc::c_int as uint8_t,
     6 as libc::c_int as uint8_t, 7 as libc::c_int as uint8_t,
     8 as libc::c_int as uint8_t, 9 as libc::c_int as uint8_t];
#[no_mangle]
pub static mut APBPrescTable: [uint8_t; 8] =
    [0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     1 as libc::c_int as uint8_t, 2 as libc::c_int as uint8_t,
     3 as libc::c_int as uint8_t, 4 as libc::c_int as uint8_t];
/* *
  * @}
  */
/* * @addtogroup STM32F7xx_System_Private_FunctionPrototypes
  * @{
  */
// / TODO: F7 check if this is the best configuration for the clocks.
  // current settings are just a copy from one of the example projects
#[no_mangle]
pub unsafe extern "C" fn SystemClock_Config() {
    let mut RCC_ClkInitStruct: RCC_ClkInitTypeDef =
        RCC_ClkInitTypeDef{ClockType: 0,
                           SYSCLKSource: 0,
                           AHBCLKDivider: 0,
                           APB1CLKDivider: 0,
                           APB2CLKDivider: 0,};
    let mut RCC_OscInitStruct: RCC_OscInitTypeDef =
        RCC_OscInitTypeDef{OscillatorType: 0,
                           HSEState: 0,
                           LSEState: 0,
                           HSIState: 0,
                           HSICalibrationValue: 0,
                           LSIState: 0,
                           PLL:
                               RCC_PLLInitTypeDef{PLLState: 0,
                                                  PLLSource: 0,
                                                  PLLM: 0,
                                                  PLLN: 0,
                                                  PLLP: 0,
                                                  PLLQ: 0,},};
    let mut PeriphClkInitStruct: RCC_PeriphCLKInitTypeDef =
        RCC_PeriphCLKInitTypeDef{PeriphClockSelection: 0,
                                 PLLI2S:
                                     RCC_PLLI2SInitTypeDef{PLLI2SN: 0,
                                                           PLLI2SR: 0,
                                                           PLLI2SQ: 0,},
                                 PLLSAI:
                                     RCC_PLLSAIInitTypeDef{PLLSAIN: 0,
                                                           PLLSAIQ: 0,
                                                           PLLSAIP: 0,},
                                 PLLI2SDivQ: 0,
                                 PLLSAIDivQ: 0,
                                 PLLSAIDivR: 0,
                                 RTCClockSelection: 0,
                                 I2sClockSelection: 0,
                                 TIMPresSelection: 0,
                                 Sai1ClockSelection: 0,
                                 Sai2ClockSelection: 0,
                                 Usart1ClockSelection: 0,
                                 Usart2ClockSelection: 0,
                                 Usart3ClockSelection: 0,
                                 Uart4ClockSelection: 0,
                                 Uart5ClockSelection: 0,
                                 Usart6ClockSelection: 0,
                                 Uart7ClockSelection: 0,
                                 Uart8ClockSelection: 0,
                                 I2c1ClockSelection: 0,
                                 I2c2ClockSelection: 0,
                                 I2c3ClockSelection: 0,
                                 I2c4ClockSelection: 0,
                                 Lptim1ClockSelection: 0,
                                 CecClockSelection: 0,
                                 Clk48ClockSelection: 0,
                                 Sdmmc1ClockSelection: 0,
                                 Sdmmc2ClockSelection: 0,};
    let mut ret: HAL_StatusTypeDef = HAL_OK;
    let mut tmpreg: uint32_t = 0;
    let ref mut fresh5 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh5,
                                (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         28 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        28 as libc::c_uint);
    let mut tmpreg_0: uint32_t = 0;
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x7000
                                                                            as
                                                                            libc::c_uint)
                                            as *mut PWR_TypeDef)).CR1 as
                                    *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x7000 as
                                                                       libc::c_uint)
                                       as *mut PWR_TypeDef)).CR1 &
                                    !((0x3 as libc::c_uint) <<
                                          14 as libc::c_uint) |
                                    (0x3 as libc::c_uint) <<
                                        14 as libc::c_uint);
    ::core::ptr::write_volatile(&mut tmpreg_0 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x7000 as
                                                                       libc::c_uint)
                                       as *mut PWR_TypeDef)).CR1 &
                                    (0x3 as libc::c_uint) <<
                                        14 as libc::c_uint);
    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = 0x1 as libc::c_uint;
    RCC_OscInitStruct.HSEState = (0x1 as libc::c_uint) << 16 as libc::c_uint;
    RCC_OscInitStruct.PLL.PLLState = 0x2 as libc::c_uint;
    RCC_OscInitStruct.PLL.PLLSource =
        (0x1 as libc::c_uint) << 22 as libc::c_uint;
    RCC_OscInitStruct.PLL.PLLM = 8 as libc::c_int as uint32_t;
    RCC_OscInitStruct.PLL.PLLN = pll_n;
    RCC_OscInitStruct.PLL.PLLP = pll_p;
    RCC_OscInitStruct.PLL.PLLQ = pll_q;
    ret = HAL_RCC_OscConfig(&mut RCC_OscInitStruct);
    if ret as libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        loop  { }
    }
    /* Activate the OverDrive to reach the 216 MHz Frequency */
    ret = HAL_PWREx_EnableOverDrive();
    if ret as libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        loop  { }
    }
    /* Select PLLSAI output as USB clock source */
    PeriphClkInitStruct.PeriphClockSelection = 0x200000 as libc::c_uint;
    PeriphClkInitStruct.Clk48ClockSelection =
        (0x1 as libc::c_uint) << 27 as libc::c_uint;
    PeriphClkInitStruct.PLLSAI.PLLSAIN = 384 as libc::c_int as uint32_t;
    PeriphClkInitStruct.PLLSAI.PLLSAIQ = 7 as libc::c_int as uint32_t;
    PeriphClkInitStruct.PLLSAI.PLLSAIP = 0x3 as libc::c_uint;
    if HAL_RCCEx_PeriphCLKConfig(&mut PeriphClkInitStruct) as libc::c_uint !=
           HAL_OK as libc::c_int as libc::c_uint {
        loop  { }
    }
    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
    RCC_ClkInitStruct.ClockType =
        0x1 as libc::c_uint | 0x2 as libc::c_uint | 0x4 as libc::c_uint |
            0x8 as libc::c_uint;
    RCC_ClkInitStruct.SYSCLKSource = 0x2 as libc::c_uint;
    RCC_ClkInitStruct.AHBCLKDivider = 0 as libc::c_uint;
    RCC_ClkInitStruct.APB1CLKDivider = 0x1400 as libc::c_uint;
    RCC_ClkInitStruct.APB2CLKDivider = 0x1000 as libc::c_uint;
    ret = HAL_RCC_ClockConfig(&mut RCC_ClkInitStruct, 0x7 as libc::c_uint);
    if ret as libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        loop  { }
    }
    PeriphClkInitStruct.PeriphClockSelection =
        0x40 as libc::c_uint | 0x80 as libc::c_uint | 0x100 as libc::c_uint |
            0x800 as libc::c_uint | 0x200 as libc::c_uint |
            0x400 as libc::c_uint | 0x1000 as libc::c_uint |
            0x2000 as libc::c_uint | 0x4000 as libc::c_uint |
            0x10000 as libc::c_uint | 0x8000 as libc::c_uint |
            0x20000 as libc::c_uint;
    PeriphClkInitStruct.Usart1ClockSelection = 0 as libc::c_uint;
    PeriphClkInitStruct.Usart2ClockSelection = 0 as libc::c_uint;
    PeriphClkInitStruct.Usart3ClockSelection = 0 as libc::c_uint;
    PeriphClkInitStruct.Uart4ClockSelection = 0 as libc::c_uint;
    PeriphClkInitStruct.Uart5ClockSelection = 0 as libc::c_uint;
    PeriphClkInitStruct.Usart6ClockSelection = 0 as libc::c_uint;
    PeriphClkInitStruct.Uart7ClockSelection = 0 as libc::c_uint;
    PeriphClkInitStruct.Uart8ClockSelection = 0 as libc::c_uint;
    PeriphClkInitStruct.I2c1ClockSelection = 0 as libc::c_uint;
    PeriphClkInitStruct.I2c2ClockSelection = 0 as libc::c_uint;
    PeriphClkInitStruct.I2c3ClockSelection = 0 as libc::c_uint;
    PeriphClkInitStruct.I2c4ClockSelection = 0 as libc::c_uint;
    ret = HAL_RCCEx_PeriphCLKConfig(&mut PeriphClkInitStruct);
    if ret as libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        loop  { }
    }
    // Activating the timerprescalers while the APBx prescalers are 1/2/4 will connect the TIMxCLK to HCLK which has been configured to 216MHz
    let ref mut fresh6 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).DCKCFGR1;
    ::core::ptr::write_volatile(fresh6,
                                (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           24 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    let ref mut fresh7 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).DCKCFGR1;
    ::core::ptr::write_volatile(fresh7,
                                (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         24 as libc::c_uint) as uint32_t as
                                    uint32_t);
    SystemCoreClockUpdate();
}
static mut overclockLevels: [pllConfig_t; 2] =
    [{
         let mut init =
             pllConfig_s{n: 432 as libc::c_int as uint16_t,
                         p: 0x2 as libc::c_uint as uint16_t,
                         q: 9 as libc::c_int as uint16_t,};
         init
     },
     {
         let mut init =
             pllConfig_s{n: 480 as libc::c_int as uint16_t,
                         p: 0x2 as libc::c_uint as uint16_t,
                         q: 10 as libc::c_int as uint16_t,};
         init
     }];
/* *
  ******************************************************************************
  * @file    system_stm32f7xx.h
  * @author  MCD Application Team
  * @version V1.6.1
  * @date    21-October-2015
  * @brief   CMSIS Cortex-M4 Device System Source File for STM32F4xx devices.
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
/* !< System Clock Frequency (Core Clock) */
#[no_mangle]
pub unsafe extern "C" fn SystemInitOC() {
    let mut tmpreg: uint32_t = 0;
    let ref mut fresh8 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1ENR;
    ::core::ptr::write_volatile(fresh8,
                                (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         28 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).APB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        28 as libc::c_uint);
    let mut tmpreg_0: uint32_t = 0;
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
                                         18 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg_0 as *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).AHB1ENR &
                                    (0x1 as libc::c_uint) <<
                                        18 as libc::c_uint);
    HAL_PWR_EnableBkUpAccess();
    if 0xbabeface as libc::c_uint ==
           *((0x40024000 as
                  libc::c_uint).wrapping_add(8 as libc::c_int as libc::c_uint)
                 as *mut uint32_t) {
        let overclockLevel: uint32_t =
            *((0x40024000 as
                   libc::c_uint).wrapping_add(12 as libc::c_int as
                                                  libc::c_uint) as
                  *mut uint32_t);
        /* PLL setting for overclocking */
        if (overclockLevel as libc::c_ulong) <
               (::core::mem::size_of::<[pllConfig_t; 2]>() as
                    libc::c_ulong).wrapping_div(::core::mem::size_of::<pllConfig_t>()
                                                    as libc::c_ulong) {
            let pll: *const pllConfig_t =
                overclockLevels.as_ptr().offset(overclockLevel as isize);
            pll_n = (*pll).n as uint32_t;
            pll_p = (*pll).p as uint32_t;
            pll_q = (*pll).q as uint32_t
        }
        ::core::ptr::write_volatile((0x40024000 as
                                         libc::c_uint).wrapping_add(8 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                        as *mut uint32_t,
                                    0 as libc::c_int as uint32_t)
    };
}
#[no_mangle]
pub unsafe extern "C" fn OverclockRebootIfNecessary(mut overclockLevel:
                                                        uint32_t) {
    if overclockLevel as libc::c_ulong >=
           (::core::mem::size_of::<[pllConfig_t; 2]>() as
                libc::c_ulong).wrapping_div(::core::mem::size_of::<pllConfig_t>()
                                                as libc::c_ulong) {
        return
    }
    let pll: *const pllConfig_t =
        overclockLevels.as_ptr().offset(overclockLevel as isize);
    // Reboot to adjust overclock frequency
    if SystemCoreClock !=
           (((*pll).n as libc::c_int / (*pll).p as libc::c_int) as
                uint32_t).wrapping_mul(1000000 as libc::c_int as libc::c_uint)
       {
        ::core::ptr::write_volatile((0x40024000 as
                                         libc::c_uint).wrapping_add(8 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                        as *mut uint32_t,
                                    0xbabeface as libc::c_uint);
        ::core::ptr::write_volatile((0x40024000 as
                                         libc::c_uint).wrapping_add(12 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                        as *mut uint32_t, overclockLevel);
        __disable_irq();
        __NVIC_SystemReset();
    };
}
/* *
  * @}
  */
/* * @addtogroup STM32F7xx_System_Private_Functions
  * @{
  */
/* *
  * @brief  Setup the microcontroller system
  *         Initialize the Embedded Flash Interface, the PLL and update the
  *         SystemFrequency variable.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SystemInit() {
    SystemInitOC();
    SystemCoreClock =
        pll_n.wrapping_div(pll_p).wrapping_mul(1000000 as libc::c_int as
                                                   libc::c_uint);
    /* FPU settings ------------------------------------------------------------*/
    /* Reset the RCC clock configuration to the default reset state ------------*/
    /* Set HSION bit */
    let ref mut fresh10 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh10,
                                (::core::ptr::read_volatile::<uint32_t>(fresh10
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x1 as libc::c_int as uint32_t) as
                                    uint32_t as uint32_t);
    /* Reset CFGR register */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).CFGR as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    /* Reset HSEON, CSSON and PLLON bits */
    let ref mut fresh11 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh11,
                                (::core::ptr::read_volatile::<uint32_t>(fresh11
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     0xfef6ffff as libc::c_uint) as uint32_t
                                    as uint32_t);
    /* Reset PLLCFGR register */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).PLLCFGR as
                                    *mut uint32_t,
                                0x24003010 as libc::c_int as uint32_t);
    /* Reset HSEBYP bit */
    let ref mut fresh12 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh12,
                                (::core::ptr::read_volatile::<uint32_t>(fresh12
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     0xfffbffff as libc::c_uint) as uint32_t
                                    as uint32_t);
    /* Disable all interrupts */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).CIR as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    /* Configure the Vector Table location add offset address ------------------*/
    extern "C" {
        #[no_mangle]
        static mut isr_vector_table_base: uint8_t;
    }
    let vtorOffset: uint32_t =
        &mut isr_vector_table_base as *mut uint8_t as uint32_t;
    ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                             libc::c_ulong).wrapping_add(0xd00
                                                                             as
                                                                             libc::c_ulong)
                                            as *mut SCB_Type)).VTOR as
                                    *mut uint32_t, vtorOffset);
    /* Enable I-Cache */
    SCB_EnableICache();
    /* Enable D-Cache */
    LL_FLASH_EnablePrefetch();
    /* Configure the system clock to specified frequency */
    SystemClock_Config();
    if SystemCoreClock !=
           pll_n.wrapping_div(pll_p).wrapping_mul(1000000 as libc::c_int as
                                                      libc::c_uint) {
        loop 
             // There is a mismatch between the configured clock and the expected clock in portable.h
             {
        }
    };
}
/* *
  ******************************************************************************
  * @file    system_stm32f7xx.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    30-December-2016
  * @brief   CMSIS Cortex-M7 Device System Source File for STM32F7xx devices.       
  ******************************************************************************  
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
/* * @addtogroup CMSIS
  * @{
  */
/* * @addtogroup stm32f7xx_system
  * @{
  */
/* *
  * @brief Define to prevent recursive inclusion
  */
/* * @addtogroup STM32F7xx_System_Includes
  * @{
  */
/* *
  * @}
  */
/* * @addtogroup STM32F7xx_System_Exported_Variables
  * @{
  */
  /* The SystemCoreClock variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency 
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
    */
/* !< System Clock Frequency (Core Clock) */
/* !< AHB prescalers table values */
/* !< APB prescalers table values */
/* *
  * @}
  */
/* * @addtogroup STM32F7xx_System_Exported_Constants
  * @{
  */
/* *
  * @}
  */
/* * @addtogroup STM32F7xx_System_Exported_Macros
  * @{
  */
/* *
  * @}
  */
/* * @addtogroup STM32F7xx_System_Exported_Functions
  * @{
  */
/* *
   * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.
  *
  * @note   - The system frequency computed by this function is not the real
  *           frequency in the chip. It is calculated based on the predefined
  *           constant and the selected clock source:
  *
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(*)
  *
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(**)
  *
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(**)
  *             or HSI_VALUE(*) multiplied/divided by the PLL factors.
  *
  *         (*) HSI_VALUE is a constant defined in stm32f7xx_hal_conf.h file (default value
  *             16 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.
  *
  *         (**) HSE_VALUE is a constant defined in stm32f7xx_hal_conf.h file (default value
  *              25 MHz), user has to ensure that HSE_VALUE is same as the real
  *              frequency of the crystal used. Otherwise, this function may
  *              have wrong result.
  *
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  *
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SystemCoreClockUpdate() {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    let mut pllvco: uint32_t = 0 as libc::c_int as uint32_t;
    let mut pllp: uint32_t = 2 as libc::c_int as uint32_t;
    let mut pllsource: uint32_t = 0 as libc::c_int as uint32_t;
    let mut pllm: uint32_t = 2 as libc::c_int as uint32_t;
    /* Get SYSCLK source -------------------------------------------------------*/
    tmp =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).CFGR &
            (0x3 as libc::c_uint) << 2 as libc::c_uint;
    match tmp {
        0 => {
            /* HSI used as system clock source */
            SystemCoreClock = 16000000 as libc::c_uint
        }
        4 => {
            /* HSE used as system clock source */
            SystemCoreClock = 8000000 as libc::c_int as uint32_t
        }
        8 => {
            /* PLL used as system clock source */
            /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
         SYSCLK = PLL_VCO / PLL_P
         */
            pllsource =
                ((*((0x40000000 as
                         libc::c_uint).wrapping_add(0x20000 as
                                                        libc::c_uint).wrapping_add(0x3800
                                                                                       as
                                                                                       libc::c_uint)
                        as *mut RCC_TypeDef)).PLLCFGR &
                     (0x1 as libc::c_uint) << 22 as libc::c_uint) >>
                    22 as libc::c_int;
            pllm =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x3800
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut RCC_TypeDef)).PLLCFGR &
                    (0x3f as libc::c_uint) << 0 as libc::c_uint;
            if pllsource != 0 as libc::c_int as libc::c_uint {
                /* HSE used as PLL clock source */
                pllvco =
                    (8000000 as libc::c_int as
                         libc::c_uint).wrapping_div(pllm).wrapping_mul(((*((0x40000000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x20000
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0x3800
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                                               as
                                                                               *mut RCC_TypeDef)).PLLCFGR
                                                                            &
                                                                            (0x1ff
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                6
                                                                                    as
                                                                                    libc::c_uint)
                                                                           >>
                                                                           6
                                                                               as
                                                                               libc::c_int)
            } else {
                /* HSI used as PLL clock source */
                pllvco =
                    (16000000 as
                         libc::c_uint).wrapping_div(pllm).wrapping_mul(((*((0x40000000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x20000
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_add(0x3800
                                                                                                                                              as
                                                                                                                                              libc::c_uint)
                                                                               as
                                                                               *mut RCC_TypeDef)).PLLCFGR
                                                                            &
                                                                            (0x1ff
                                                                                 as
                                                                                 libc::c_uint)
                                                                                <<
                                                                                6
                                                                                    as
                                                                                    libc::c_uint)
                                                                           >>
                                                                           6
                                                                               as
                                                                               libc::c_int)
            }
            pllp =
                (((*((0x40000000 as
                          libc::c_uint).wrapping_add(0x20000 as
                                                         libc::c_uint).wrapping_add(0x3800
                                                                                        as
                                                                                        libc::c_uint)
                         as *mut RCC_TypeDef)).PLLCFGR &
                      (0x3 as libc::c_uint) << 16 as libc::c_uint) >>
                     16 as
                         libc::c_int).wrapping_add(1 as libc::c_int as
                                                       libc::c_uint).wrapping_mul(2
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint);
            SystemCoreClock = pllvco.wrapping_div(pllp)
        }
        _ => { SystemCoreClock = 16000000 as libc::c_uint }
    }
    /* Compute HCLK frequency --------------------------------------------------*/
    /* Get HCLK prescaler */
    tmp =
        AHBPrescTable[(((*((0x40000000 as
                                libc::c_uint).wrapping_add(0x20000 as
                                                               libc::c_uint).wrapping_add(0x3800
                                                                                              as
                                                                                              libc::c_uint)
                               as *mut RCC_TypeDef)).CFGR &
                            (0xf as libc::c_uint) << 4 as libc::c_uint) >>
                           4 as libc::c_int) as usize] as uint32_t;
    /* HCLK frequency */
    SystemCoreClock >>= tmp;
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
