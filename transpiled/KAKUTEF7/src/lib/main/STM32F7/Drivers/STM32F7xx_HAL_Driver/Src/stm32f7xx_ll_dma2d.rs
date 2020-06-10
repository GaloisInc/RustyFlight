use ::libc;
pub type __uint32_t = libc::c_uint;
pub type uint32_t = __uint32_t;
/* *
  * @brief DMA2D Controller
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA2D_TypeDef {
    pub CR: uint32_t,
    pub ISR: uint32_t,
    pub IFCR: uint32_t,
    pub FGMAR: uint32_t,
    pub FGOR: uint32_t,
    pub BGMAR: uint32_t,
    pub BGOR: uint32_t,
    pub FGPFCCR: uint32_t,
    pub FGCOLR: uint32_t,
    pub BGPFCCR: uint32_t,
    pub BGCOLR: uint32_t,
    pub FGCMAR: uint32_t,
    pub BGCMAR: uint32_t,
    pub OPFCCR: uint32_t,
    pub OCOLR: uint32_t,
    pub OMAR: uint32_t,
    pub OOR: uint32_t,
    pub NLR: uint32_t,
    pub LWR: uint32_t,
    pub AMTCR: uint32_t,
    pub RESERVED: [uint32_t; 236],
    pub FGCLUT: [uint32_t; 256],
    pub BGCLUT: [uint32_t; 256],
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
pub type ErrorStatus = libc::c_uint;
pub const SUCCESS: ErrorStatus = 1;
pub const ERROR: ErrorStatus = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LL_DMA2D_InitTypeDef {
    pub Mode: uint32_t,
    pub ColorMode: uint32_t,
    pub OutputBlue: uint32_t,
    pub OutputGreen: uint32_t,
    pub OutputRed: uint32_t,
    pub OutputAlpha: uint32_t,
    pub OutputMemoryAddress: uint32_t,
    pub LineOffset: uint32_t,
    pub NbrOfLines: uint32_t,
    pub NbrOfPixelsPerLines: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LL_DMA2D_LayerCfgTypeDef {
    pub MemoryAddress: uint32_t,
    pub LineOffset: uint32_t,
    pub ColorMode: uint32_t,
    pub CLUTColorMode: uint32_t,
    pub CLUTSize: uint32_t,
    pub AlphaMode: uint32_t,
    pub Alpha: uint32_t,
    pub Blue: uint32_t,
    pub Green: uint32_t,
    pub Red: uint32_t,
    pub CLUTMemoryAddress: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LL_DMA2D_ColorTypeDef {
    pub ColorMode: uint32_t,
    pub OutputBlue: uint32_t,
    pub OutputGreen: uint32_t,
    pub OutputRed: uint32_t,
    pub OutputAlpha: uint32_t,
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_ll_dma2d.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of DMA2D LL module.
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
/* * @addtogroup STM32F7xx_LL_Driver
  * @{
  */
/* * @defgroup DMA2D_LL DMA2D
  * @{
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* * @defgroup DMA2D_LL_Private_Macros DMA2D Private Macros
  * @{
  */
/* *
  * @}
  */
/*USE_FULL_LL_DRIVER*/
/* Exported types ------------------------------------------------------------*/
/* * @defgroup DMA2D_LL_ES_Init_Struct DMA2D Exported Init structures
  * @{
  */
/* *
  * @brief LL DMA2D Init Structure Definition
  */
/* !< Specifies the DMA2D transfer mode.
                                      - This parameter can be one value of @ref DMA2D_LL_EC_MODE.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetMode().*/
/* !< Specifies the color format of the output image.
                                      - This parameter can be one value of @ref DMA2D_LL_EC_OUTPUT_COLOR_MODE.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputColorMode(). */
/* !< Specifies the Blue value of the output image.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if ARGB8888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if RGB888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F if RGB565 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F if ARGB1555 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x0F if ARGB4444 color mode is selected.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputColor() or configuration
                                      function @ref LL_DMA2D_ConfigOutputColor(). */
/* !< Specifies the Green value of the output image.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if ARGB8888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if RGB888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x3F if RGB565 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F if ARGB1555 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x0F if ARGB4444 color mode is selected.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputColor() or configuration
                                      function @ref LL_DMA2D_ConfigOutputColor(). */
/* !< Specifies the Red value of the output image.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if ARGB8888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if RGB888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F if RGB565 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F if ARGB1555 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x0F if ARGB4444 color mode is selected.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputColor() or configuration
                                      function @ref LL_DMA2D_ConfigOutputColor(). */
/* !< Specifies the Alpha channel of the output image.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if ARGB8888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x01 if ARGB1555 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x0F if ARGB4444 color mode is selected.
                                      - This parameter is not considered if RGB888 or RGB565 color mode is selected.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputColor() or configuration
                                      function @ref LL_DMA2D_ConfigOutputColor(). */
/* !< Specifies the memory address.
                                      - This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFFFFFF.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputMemAddr(). */
/* !< Specifies the output line offset value.
                                      - This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0x3FFF.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetLineOffset(). */
/* !< Specifies the number of lines of the area to be transferred.
                                      - This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetNbrOfLines(). */
/* !< Specifies the number of pixels per lines of the area to be transfered.
                                      - This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0x3FFF.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetNbrOfPixelsPerLines(). */
/* DMA2D_ALPHA_INV_RB_SWAP_SUPPORT */
/* *
  * @brief LL DMA2D Layer Configuration Structure Definition
  */
/* !< Specifies the foreground or background memory address.
                                      - This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFFFFFF.

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetMemAddr() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetMemAddr() for background layer. */
/* !< Specifies the foreground or background line offset value.
                                      - This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0x3FFF.

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetLineOffset() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetLineOffset() for background layer. */
/* !< Specifies the foreground or background color mode.
                                      - This parameter can be one value of @ref DMA2D_LL_EC_INPUT_COLOR_MODE.

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetColorMode() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetColorMode() for background layer. */
/* !< Specifies the foreground or background CLUT color mode.
                                       - This parameter can be one value of @ref DMA2D_LL_EC_CLUT_COLOR_MODE.

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetCLUTColorMode() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetCLUTColorMode() for background layer. */
/* !< Specifies the foreground or background CLUT size.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF.

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetCLUTSize() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetCLUTSize() for background layer. */
/* !< Specifies the foreground or background alpha mode.
                                       - This parameter can be one value of @ref DMA2D_LL_EC_ALPHA_MODE.

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetAlphaMode() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetAlphaMode() for background layer. */
/* !< Specifies the foreground or background Alpha value.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF.

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetAlpha() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetAlpha() for background layer. */
/* !< Specifies the foreground or background Blue color value.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF.

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetBlueColor() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetBlueColor() for background layer. */
/* !< Specifies the foreground or background Green color value.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF.

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetGreenColor() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetGreenColor() for background layer. */
/* !< Specifies the foreground or background Red color value.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF.

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetRedColor() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetRedColor() for background layer. */
/* !< Specifies the foreground or background CLUT memory address.
                                      - This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFFFFFF.

                                      This parameter can be modified afterwards using unitary functions
                                      - @ref LL_DMA2D_FGND_SetCLUTMemAddr() for foreground layer,
                                      - @ref LL_DMA2D_BGND_SetCLUTMemAddr() for background layer. */
/* DMA2D_ALPHA_INV_RB_SWAP_SUPPORT */
/* *
  * @brief LL DMA2D Output Color Structure Definition
  */
/* !< Specifies the color format of the output image.
                                      - This parameter can be one value of @ref DMA2D_LL_EC_OUTPUT_COLOR_MODE.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputColorMode(). */
/* !< Specifies the Blue value of the output image.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if ARGB8888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if RGB888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F if RGB565 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F if ARGB1555 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x0F if ARGB4444 color mode is selected.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputColor() or configuration
                                      function @ref LL_DMA2D_ConfigOutputColor(). */
/* !< Specifies the Green value of the output image.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if ARGB8888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if RGB888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x3F if RGB565 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F if ARGB1555 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x0F if ARGB4444 color mode is selected.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputColor() or configuration
                                      function @ref LL_DMA2D_ConfigOutputColor(). */
/* !< Specifies the Red value of the output image.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if ARGB8888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if RGB888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F if RGB565 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F if ARGB1555 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x0F if ARGB4444 color mode is selected.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputColor() or configuration
                                      function @ref LL_DMA2D_ConfigOutputColor(). */
/* !< Specifies the Alpha channel of the output image.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF if ARGB8888 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x01 if ARGB1555 color mode is selected.
                                      - This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x0F if ARGB4444 color mode is selected.
                                      - This parameter is not considered if RGB888 or RGB565 color mode is selected.

                                      This parameter can be modified afterwards using unitary function @ref LL_DMA2D_SetOutputColor() or configuration
                                      function @ref LL_DMA2D_ConfigOutputColor(). */
/* *
  * @}
  */
/* USE_FULL_LL_DRIVER */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup DMA2D_LL_Exported_Constants DMA2D Exported Constants
  * @{
  */
/* * @defgroup DMA2D_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with LL_DMA2D_ReadReg function
  * @{
  */
/* !< Configuration Error Interrupt Flag */
/* !< CLUT Transfer Complete Interrupt Flag */
/* !< CLUT Access Error Interrupt Flag */
/* !< Transfer Watermark Interrupt Flag */
/* !< Transfer Complete Interrupt Flag */
/* !< Transfer Error Interrupt Flag */
/* *
  * @}
  */
/* * @defgroup DMA2D_LL_EC_IT IT Defines
  * @brief    IT defines which can be used with LL_DMA2D_ReadReg and  LL_DMA2D_WriteReg functions
  * @{
  */
/* !< Configuration Error Interrupt */
/* !< CLUT Transfer Complete Interrupt */
/* !< CLUT Access Error Interrupt */
/* !< Transfer Watermark Interrupt */
/* !< Transfer Complete Interrupt */
/* !< Transfer Error Interrupt */
/* *
  * @}
  */
/* * @defgroup DMA2D_LL_EC_MODE Mode
  * @{
  */
/* !< DMA2D memory to memory transfer mode */
/* !< DMA2D memory to memory with pixel format conversion transfer mode */
/* !< DMA2D memory to memory with blending transfer mode */
/* !< DMA2D register to memory transfer mode */
/* *
  * @}
  */
/* * @defgroup DMA2D_LL_EC_OUTPUT_COLOR_MODE Output Color Mode
  * @{
  */
/* !< ARGB8888 */
/* !< RGB888   */
/* !< RGB565   */
/* !< ARGB1555 */
/* !< ARGB4444 */
/* *
  * @}
  */
/* * @defgroup DMA2D_LL_EC_INPUT_COLOR_MODE Input Color Mode
  * @{
  */
/* !< ARGB8888 */
/* !< RGB888   */
/* !< RGB565   */
/* !< ARGB1555 */
/* !< ARGB4444 */
/* !< L8       */
/* !< AL44     */
/* !< AL88     */
/* !< L4       */
/* !< A8       */
/* !< A4       */
/* *
  * @}
  */
/* * @defgroup DMA2D_LL_EC_ALPHA_MODE Alpha Mode
  * @{
  */
/* !< No modification of the alpha channel value */
/* !< Replace original alpha channel value by programmed alpha value */
/* !< Replace original alpha channel value by programmed alpha value
                                                                   with original alpha channel value                              */
/* *
  * @}
  */
/* DMA2D_ALPHA_INV_RB_SWAP_SUPPORT */
/* * @defgroup DMA2D_LL_EC_CLUT_COLOR_MODE CLUT Color Mode
  * @{
  */
/* !< ARGB8888 */
/* !< RGB888   */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* * @defgroup DMA2D_LL_Exported_Macros DMA2D Exported Macros
  * @{
  */
/* * @defgroup DMA2D_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */
/* *
  * @brief  Write a value in DMA2D register.
  * @param  __INSTANCE__ DMA2D Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
/* *
  * @brief  Read a value in DMA2D register.
  * @param  __INSTANCE__ DMA2D Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @defgroup DMA2D_LL_Exported_Functions DMA2D Exported Functions
  * @{
  */
/* * @defgroup DMA2D_LL_EF_Configuration Configuration Functions
  * @{
  */
/* *
  * @brief  Start a DMA2D transfer.
  * @rmtoll CR          START            LL_DMA2D_Start
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
/* *
  * @brief  Indicate if a DMA2D transfer is ongoing.
  * @rmtoll CR          START            LL_DMA2D_IsTransferOngoing
  * @param  DMA2Dx DMA2D Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Suspend DMA2D transfer.
  * @note   This API can be used to suspend automatic foreground or background CLUT loading.
  * @rmtoll CR          SUSP            LL_DMA2D_Suspend
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
/* *
  * @brief  Resume DMA2D transfer.
  * @note   This API can be used to resume automatic foreground or background CLUT loading.
  * @rmtoll CR          SUSP            LL_DMA2D_Resume
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
/* *
  * @brief  Indicate if DMA2D transfer is suspended.
  * @note   This API can be used to indicate whether or not automatic foreground or
  *         background CLUT loading is suspended.
  * @rmtoll CR          SUSP            LL_DMA2D_IsSuspended
  * @param  DMA2Dx DMA2D Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Abort DMA2D transfer.
  * @note   This API can be used to abort automatic foreground or background CLUT loading.
  * @rmtoll CR          ABORT            LL_DMA2D_Abort
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
/* *
  * @brief  Indicate if DMA2D transfer is aborted.
  * @note   This API can be used to indicate whether or not automatic foreground or
  *         background CLUT loading is aborted.
  * @rmtoll CR          ABORT            LL_DMA2D_IsAborted
  * @param  DMA2Dx DMA2D Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Set DMA2D mode.
  * @rmtoll CR          MODE          LL_DMA2D_SetMode
  * @param  DMA2Dx DMA2D Instance
  * @param  Mode This parameter can be one of the following values:
  *         @arg @ref LL_DMA2D_MODE_M2M
  *         @arg @ref LL_DMA2D_MODE_M2M_PFC
  *         @arg @ref LL_DMA2D_MODE_M2M_BLEND
  *         @arg @ref LL_DMA2D_MODE_R2M
  * @retval None
  */
/* *
  * @brief  Return DMA2D mode
  * @rmtoll CR          MODE         LL_DMA2D_GetMode
  * @param  DMA2Dx DMA2D Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA2D_MODE_M2M
  *         @arg @ref LL_DMA2D_MODE_M2M_PFC
  *         @arg @ref LL_DMA2D_MODE_M2M_BLEND
  *         @arg @ref LL_DMA2D_MODE_R2M
  */
/* *
  * @brief  Set DMA2D output color mode.
  * @rmtoll OPFCCR          CM          LL_DMA2D_SetOutputColorMode
  * @param  DMA2Dx DMA2D Instance
  * @param  ColorMode This parameter can be one of the following values:
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_ARGB8888
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_RGB888
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_RGB565
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_ARGB1555
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_ARGB4444
  * @retval None
  */
/* *
  * @brief  Return DMA2D output color mode.
  * @rmtoll OPFCCR          CM         LL_DMA2D_GetOutputColorMode
  * @param  DMA2Dx DMA2D Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_ARGB8888
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_RGB888
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_RGB565
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_ARGB1555
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_ARGB4444
  */
/* DMA2D_ALPHA_INV_RB_SWAP_SUPPORT */
/* *
  * @brief  Set DMA2D line offset, expressed on 14 bits ([13:0] bits).
  * @rmtoll OOR          LO          LL_DMA2D_SetLineOffset
  * @param  DMA2Dx DMA2D Instance
  * @param  LineOffset Value between Min_Data=0 and Max_Data=0x3FFF
  * @retval None
  */
/* *
  * @brief  Return DMA2D line offset, expressed on 14 bits ([13:0] bits).
  * @rmtoll OOR          LO         LL_DMA2D_GetLineOffset
  * @param  DMA2Dx DMA2D Instance
  * @retval Line offset value between Min_Data=0 and Max_Data=0x3FFF
  */
/* *
  * @brief  Set DMA2D number of pixels per lines, expressed on 14 bits ([13:0] bits).
  * @rmtoll NLR          PL          LL_DMA2D_SetNbrOfPixelsPerLines
  * @param  DMA2Dx DMA2D Instance
  * @param  NbrOfPixelsPerLines Value between Min_Data=0 and Max_Data=0x3FFF
  * @retval None
  */
/* *
  * @brief  Return DMA2D number of pixels per lines, expressed on 14 bits ([13:0] bits)
  * @rmtoll NLR          PL          LL_DMA2D_GetNbrOfPixelsPerLines
  * @param  DMA2Dx DMA2D Instance
  * @retval Number of pixels per lines value between Min_Data=0 and Max_Data=0x3FFF
  */
/* *
  * @brief  Set DMA2D number of lines, expressed on 16 bits ([15:0] bits).
  * @rmtoll NLR          NL          LL_DMA2D_SetNbrOfLines
  * @param  DMA2Dx DMA2D Instance
  * @param  NbrOfLines Value between Min_Data=0 and Max_Data=0xFFFF
  * @retval None
  */
/* *
  * @brief  Return DMA2D number of lines, expressed on 16 bits ([15:0] bits).
  * @rmtoll NLR          NL          LL_DMA2D_GetNbrOfLines
  * @param  DMA2Dx DMA2D Instance
  * @retval Number of lines value between Min_Data=0 and Max_Data=0xFFFF
  */
/* *
  * @brief  Set DMA2D output memory address, expressed on 32 bits ([31:0] bits).
  * @rmtoll OMAR          MA          LL_DMA2D_SetOutputMemAddr
  * @param  DMA2Dx DMA2D Instance
  * @param  OutputMemoryAddress Value between Min_Data=0 and Max_Data=0xFFFFFFFF
  * @retval None
  */
/* *
  * @brief  Get DMA2D output memory address, expressed on 32 bits ([31:0] bits).
  * @rmtoll OMAR          MA          LL_DMA2D_GetOutputMemAddr
  * @param  DMA2Dx DMA2D Instance
  * @retval Output memory address value between Min_Data=0 and Max_Data=0xFFFFFFFF
  */
/* *
  * @brief  Set DMA2D output color, expressed on 32 bits ([31:0] bits).
  * @note   Output color format depends on output color mode, ARGB8888, RGB888,
  *         RGB565, ARGB1555 or ARGB4444.
  * @note LL_DMA2D_ConfigOutputColor() API may be used instead if colors values formatting
  *       with respect to color mode is not done by the user code.
  * @rmtoll OCOLR        BLUE        LL_DMA2D_SetOutputColor\n
  *         OCOLR        GREEN       LL_DMA2D_SetOutputColor\n
  *         OCOLR        RED         LL_DMA2D_SetOutputColor\n
  *         OCOLR        ALPHA       LL_DMA2D_SetOutputColor
  * @param  DMA2Dx DMA2D Instance
  * @param  OutputColor Value between Min_Data=0 and Max_Data=0xFFFFFFFF
  * @retval None
  */
/* *
  * @brief  Get DMA2D output color, expressed on 32 bits ([31:0] bits).
  * @note   Alpha channel and red, green, blue color values must be retrieved from the returned
  *         value based on the output color mode (ARGB8888, RGB888,  RGB565, ARGB1555 or ARGB4444)
  *         as set by @ref LL_DMA2D_SetOutputColorMode.
  * @rmtoll OCOLR        BLUE        LL_DMA2D_GetOutputColor\n
  *         OCOLR        GREEN       LL_DMA2D_GetOutputColor\n
  *         OCOLR        RED         LL_DMA2D_GetOutputColor\n
  *         OCOLR        ALPHA       LL_DMA2D_GetOutputColor
  * @param  DMA2Dx DMA2D Instance
  * @retval Output color value between Min_Data=0 and Max_Data=0xFFFFFFFF
  */
/* *
  * @brief  Set DMA2D line watermark, expressed on 16 bits ([15:0] bits).
  * @rmtoll LWR          LW          LL_DMA2D_SetLineWatermark
  * @param  DMA2Dx DMA2D Instance
  * @param  LineWatermark Value between Min_Data=0 and Max_Data=0xFFFF
  * @retval None
  */
/* *
  * @brief  Return DMA2D line watermark, expressed on 16 bits ([15:0] bits).
  * @rmtoll LWR          LW          LL_DMA2D_GetLineWatermark
  * @param  DMA2Dx DMA2D Instance
  * @retval Line watermark value between Min_Data=0 and Max_Data=0xFFFF
  */
/* *
  * @brief  Set DMA2D dead time, expressed on 8 bits ([7:0] bits).
  * @rmtoll AMTCR          DT          LL_DMA2D_SetDeadTime
  * @param  DMA2Dx DMA2D Instance
  * @param  DeadTime Value between Min_Data=0 and Max_Data=0xFF
  * @retval None
  */
/* *
  * @brief  Return DMA2D dead time, expressed on 8 bits ([7:0] bits).
  * @rmtoll AMTCR          DT          LL_DMA2D_GetDeadTime
  * @param  DMA2Dx DMA2D Instance
  * @retval Dead time value between Min_Data=0 and Max_Data=0xFF
  */
/* *
  * @brief  Enable DMA2D dead time functionality.
  * @rmtoll AMTCR          EN            LL_DMA2D_EnableDeadTime
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
/* *
  * @brief  Disable DMA2D dead time functionality.
  * @rmtoll AMTCR          EN            LL_DMA2D_DisableDeadTime
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
/* *
  * @brief  Indicate if DMA2D dead time functionality is enabled.
  * @rmtoll AMTCR          EN            LL_DMA2D_IsEnabledDeadTime
  * @param  DMA2Dx DMA2D Instance
  * @retval State of bit (1 or 0).
  */
/* * @defgroup DMA2D_LL_EF_FGND_Configuration Foreground Configuration Functions
  * @{
  */
/* *
  * @brief  Set DMA2D foreground memory address, expressed on 32 bits ([31:0] bits).
  * @rmtoll FGMAR          MA          LL_DMA2D_FGND_SetMemAddr
  * @param  DMA2Dx DMA2D Instance
  * @param  MemoryAddress Value between Min_Data=0 and Max_Data=0xFFFFFFFF
  * @retval None
  */
/* *
  * @brief  Get DMA2D foreground memory address, expressed on 32 bits ([31:0] bits).
  * @rmtoll FGMAR          MA          LL_DMA2D_FGND_GetMemAddr
  * @param  DMA2Dx DMA2D Instance
  * @retval Foreground memory address value between Min_Data=0 and Max_Data=0xFFFFFFFF
  */
/* *
  * @brief  Enable DMA2D foreground CLUT loading.
  * @rmtoll FGPFCCR          START            LL_DMA2D_FGND_EnableCLUTLoad
  * @param  DMA2Dx DMA2D Instance
  * @retval None
  */
/* *
  * @brief  Indicate if DMA2D foreground CLUT loading is enabled.
  * @rmtoll FGPFCCR          START            LL_DMA2D_FGND_IsEnabledCLUTLoad
  * @param  DMA2Dx DMA2D Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Set DMA2D foreground color mode.
  * @rmtoll FGPFCCR          CM          LL_DMA2D_FGND_SetColorMode
  * @param  DMA2Dx DMA2D Instance
  * @param  ColorMode This parameter can be one of the following values:
  *         @arg @ref LL_DMA2D_INPUT_MODE_ARGB8888
  *         @arg @ref LL_DMA2D_INPUT_MODE_RGB888
  *         @arg @ref LL_DMA2D_INPUT_MODE_RGB565
  *         @arg @ref LL_DMA2D_INPUT_MODE_ARGB1555
  *         @arg @ref LL_DMA2D_INPUT_MODE_ARGB4444
  *         @arg @ref LL_DMA2D_INPUT_MODE_L8
  *         @arg @ref LL_DMA2D_INPUT_MODE_AL44
  *         @arg @ref LL_DMA2D_INPUT_MODE_AL88
  *         @arg @ref LL_DMA2D_INPUT_MODE_L4
  *         @arg @ref LL_DMA2D_INPUT_MODE_A8
  *         @arg @ref LL_DMA2D_INPUT_MODE_A4
  * @retval None
  */
/* *
  * @brief  Return DMA2D foreground color mode.
  * @rmtoll FGPFCCR          CM         LL_DMA2D_FGND_GetColorMode
  * @param  DMA2Dx DMA2D Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA2D_INPUT_MODE_ARGB8888
  *         @arg @ref LL_DMA2D_INPUT_MODE_RGB888
  *         @arg @ref LL_DMA2D_INPUT_MODE_RGB565
  *         @arg @ref LL_DMA2D_INPUT_MODE_ARGB1555
  *         @arg @ref LL_DMA2D_INPUT_MODE_ARGB4444
  *         @arg @ref LL_DMA2D_INPUT_MODE_L8
  *         @arg @ref LL_DMA2D_INPUT_MODE_AL44
  *         @arg @ref LL_DMA2D_INPUT_MODE_AL88
  *         @arg @ref LL_DMA2D_INPUT_MODE_L4
  *         @arg @ref LL_DMA2D_INPUT_MODE_A8
  *         @arg @ref LL_DMA2D_INPUT_MODE_A4
  */
/* *
  * @brief  Set DMA2D foreground alpha mode.
  * @rmtoll FGPFCCR          AM          LL_DMA2D_FGND_SetAlphaMode
  * @param  DMA2Dx DMA2D Instance
  * @param  AphaMode This parameter can be one of the following values:
  *         @arg @ref LL_DMA2D_ALPHA_MODE_NO_MODIF
  *         @arg @ref LL_DMA2D_ALPHA_MODE_REPLACE
  *         @arg @ref LL_DMA2D_ALPHA_MODE_COMBINE
  * @retval None
  */
/* *
  * @brief  Return DMA2D foreground alpha mode.
  * @rmtoll FGPFCCR          AM         LL_DMA2D_FGND_GetAlphaMode
  * @param  DMA2Dx DMA2D Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_DMA2D_ALPHA_MODE_NO_MODIF
  *         @arg @ref LL_DMA2D_ALPHA_MODE_REPLACE
  *         @arg @ref LL_DMA2D_ALPHA_MODE_COMBINE
  */
/* *
  * @brief  Set DMA2D foreground alpha value, expressed on 8 bits ([7:0] bits).
  * @rmtoll FGPFCCR          ALPHA          LL_DMA2D_FGND_SetAlpha
  * @param  DMA2Dx DMA2D Instance
  * @param  Alpha Value between Min_Data=0 and Max_Data=0xFF
  * @retval None
  */
/* *
  * @brief  Return DMA2D foreground alpha value, expressed on 8 bits ([7:0] bits).
  * @rmtoll FGPFCCR          ALPHA         LL_DMA2D_FGND_GetAlpha
  * @param  DMA2Dx DMA2D Instance
  * @retval Alpha value between Min_Data=0 and Max_Data=0xFF
  */
/* DMA2D_ALPHA_INV_RB_SWAP_SUPPORT */
/* *
  * @brief  Set DMA2D foreground line offset, expressed on 14 bits ([13:0] bits).
  * @rmtoll FGOR          LO          LL_DMA2D_FGND_SetLineOffset
  * @param  DMA2Dx DMA2D Instance
  * @param  LineOffset Value between Min_Data=0 and Max_Data=0x3FF
  * @retval None
  */
/* *
  * @brief  Return DMA2D foreground line offset, expressed on 14 bits ([13:0] bits).
  * @rmtoll FGOR          LO         LL_DMA2D_FGND_GetLineOffset
  * @param  DMA2Dx DMA2D Instance
  * @retval Foreground line offset value between Min_Data=0 and Max_Data=0x3FF
  */
/* *
  * @brief  Set DMA2D foreground color values, expressed on 24 bits ([23:0] bits).
  * @rmtoll FGCOLR          RED          LL_DMA2D_FGND_SetColor
  * @rmtoll FGCOLR          GREEN        LL_DMA2D_FGND_SetColor
  * @rmtoll FGCOLR          BLUE         LL_DMA2D_FGND_SetColor
  * @param  DMA2Dx DMA2D Instance
  * @param  Red   Value between Min_Data=0 and Max_Data=0xFF
  * @param  Green Value between Min_Data=0 and Max_Data=0xFF
  * @param  Blue  Value between Min_Data=0 and Max_Data=0xFF
  * @retval None
  */
/* *
  * @brief  Set DMA2D foreground red color value, expressed on 8 bits ([7:0] bits).
  * @rmtoll FGCOLR          RED          LL_DMA2D_FGND_SetRedColor
  * @param  DMA2Dx DMA2D Instance
  * @param  Red Value between Min_Data=0 and Max_Data=0xFF
  * @retval None
  */
/* *
  * @brief  Return DMA2D foreground red color value, expressed on 8 bits ([7:0] bits).
  * @rmtoll FGCOLR          RED         LL_DMA2D_FGND_GetRedColor
  * @param  DMA2Dx DMA2D Instance
  * @retval Red color value between Min_Data=0 and Max_Data=0xFF
  */
/* *
  * @brief  Set DMA2D foreground green color value, expressed on 8 bits ([7:0] bits).
  * @rmtoll FGCOLR          GREEN          LL_DMA2D_FGND_SetGreenColor
  * @param  DMA2Dx DMA2D Instance
  * @param  Green Value between Min_Data=0 and Max_Data=0xFF
  * @retval None
  */
/* *
  * @brief  Return DMA2D foreground green color value, expressed on 8 bits ([7:0] bits).
  * @rmtoll FGCOLR          GREEN         LL_DMA2D_FGND_GetGreenColor
  * @param  DMA2Dx DMA2D Instance
  * @retval Green color value between Min_Data=0 and Max_Data=0xFF
  */
/* *
  * @brief  Set DMA2D foreground blue color value, expressed on 8 bits ([7:0] bits).
  * @rmtoll FGCOLR          BLUE          LL_DMA2D_FGND_SetBlueColor
  * @param  DMA2Dx DMA2D Instance
  * @param  Blue Value between Min_Data=0 and Max_Data=0xFF
  * @retval None
  */
/* *
  * @brief  Return DMA2D foreground blue color value, expressed on 8 bits ([7:0] bits).
  * @rmtoll FGCOLR          BLUE         LL_DMA2D_FGND_GetBlueColor
  * @param  DMA2Dx DMA2D Instance
  * @retval Blue color value between Min_Data=0 and Max_Data=0xFF
  */
/* *
  * @brief  Set DMA2D foreground CLUT memory address, expressed on 32 bits ([31:0] bits).
  * @rmtoll FGCMAR          MA          LL_DMA2D_FGND_SetCLUTMemAddr
  * @param  DMA2Dx DMA2D Instance
  * @param  CLUTMemoryAddress Value between Min_Data=0 and Max_Data=0xFFFFFFFF
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_DMA2D_FGND_SetCLUTMemAddr(mut DMA2Dx:
                                                      *mut DMA2D_TypeDef,
                                                  mut CLUTMemoryAddress:
                                                      uint32_t) {
    ::core::ptr::write_volatile(&mut (*DMA2Dx).FGCMAR as *mut uint32_t,
                                CLUTMemoryAddress);
}
#[inline]
unsafe extern "C" fn LL_DMA2D_IsTransferOngoing(mut DMA2Dx:
                                                    *mut DMA2D_TypeDef)
 -> uint32_t {
    return ((*DMA2Dx).CR & (0x1 as libc::c_uint) << 0 as libc::c_uint ==
                (0x1 as libc::c_uint) << 0 as libc::c_uint) as libc::c_int as
               uint32_t;
}
#[inline]
unsafe extern "C" fn LL_DMA2D_SetMode(mut DMA2Dx: *mut DMA2D_TypeDef,
                                      mut Mode: uint32_t) {
    ::core::ptr::write_volatile(&mut (*DMA2Dx).CR as *mut uint32_t,
                                (*DMA2Dx).CR &
                                    !((0x3 as libc::c_uint) <<
                                          16 as libc::c_uint) | Mode);
}
#[inline]
unsafe extern "C" fn LL_DMA2D_SetLineOffset(mut DMA2Dx: *mut DMA2D_TypeDef,
                                            mut LineOffset: uint32_t) {
    ::core::ptr::write_volatile(&mut (*DMA2Dx).OOR as *mut uint32_t,
                                (*DMA2Dx).OOR &
                                    !((0x3fff as libc::c_uint) <<
                                          0 as libc::c_uint) | LineOffset);
}
#[inline]
unsafe extern "C" fn LL_DMA2D_SetOutputMemAddr(mut DMA2Dx: *mut DMA2D_TypeDef,
                                               mut OutputMemoryAddress:
                                                   uint32_t) {
    ::core::ptr::write_volatile(&mut (*DMA2Dx).OMAR as *mut uint32_t,
                                OutputMemoryAddress);
}
#[inline]
unsafe extern "C" fn LL_DMA2D_SetOutputColor(mut DMA2Dx: *mut DMA2D_TypeDef,
                                             mut OutputColor: uint32_t) {
    ::core::ptr::write_volatile(&mut (*DMA2Dx).OCOLR as *mut uint32_t,
                                (*DMA2Dx).OCOLR &
                                    !(0xff as libc::c_uint |
                                          0xff00 as libc::c_uint |
                                          0xff0000 as libc::c_uint |
                                          0xff000000 as libc::c_uint) |
                                    OutputColor);
}
#[inline]
unsafe extern "C" fn LL_DMA2D_FGND_SetMemAddr(mut DMA2Dx: *mut DMA2D_TypeDef,
                                              mut MemoryAddress: uint32_t) {
    ::core::ptr::write_volatile(&mut (*DMA2Dx).FGMAR as *mut uint32_t,
                                MemoryAddress);
}
#[inline]
unsafe extern "C" fn LL_DMA2D_FGND_IsEnabledCLUTLoad(mut DMA2Dx:
                                                         *mut DMA2D_TypeDef)
 -> uint32_t {
    return ((*DMA2Dx).FGPFCCR & (0x1 as libc::c_uint) << 5 as libc::c_uint ==
                (0x1 as libc::c_uint) << 5 as libc::c_uint) as libc::c_int as
               uint32_t;
}
#[inline]
unsafe extern "C" fn LL_DMA2D_FGND_SetLineOffset(mut DMA2Dx:
                                                     *mut DMA2D_TypeDef,
                                                 mut LineOffset: uint32_t) {
    ::core::ptr::write_volatile(&mut (*DMA2Dx).FGOR as *mut uint32_t,
                                (*DMA2Dx).FGOR &
                                    !((0x3fff as libc::c_uint) <<
                                          0 as libc::c_uint) | LineOffset);
}
#[inline]
unsafe extern "C" fn LL_DMA2D_FGND_SetColor(mut DMA2Dx: *mut DMA2D_TypeDef,
                                            mut Red: uint32_t,
                                            mut Green: uint32_t,
                                            mut Blue: uint32_t) {
    ::core::ptr::write_volatile(&mut (*DMA2Dx).FGCOLR as *mut uint32_t,
                                (*DMA2Dx).FGCOLR &
                                    !((0xff as libc::c_uint) <<
                                          16 as libc::c_uint |
                                          (0xff as libc::c_uint) <<
                                              8 as libc::c_uint |
                                          (0xff as libc::c_uint) <<
                                              0 as libc::c_uint) |
                                    (Red << 16 as libc::c_uint |
                                         Green << 8 as libc::c_uint | Blue));
}
#[inline]
unsafe extern "C" fn LL_DMA2D_BGND_IsEnabledCLUTLoad(mut DMA2Dx:
                                                         *mut DMA2D_TypeDef)
 -> uint32_t {
    return ((*DMA2Dx).BGPFCCR & (0x1 as libc::c_uint) << 5 as libc::c_uint ==
                (0x1 as libc::c_uint) << 5 as libc::c_uint) as libc::c_int as
               uint32_t;
}
#[inline]
unsafe extern "C" fn LL_DMA2D_BGND_SetLineOffset(mut DMA2Dx:
                                                     *mut DMA2D_TypeDef,
                                                 mut LineOffset: uint32_t) {
    ::core::ptr::write_volatile(&mut (*DMA2Dx).BGOR as *mut uint32_t,
                                (*DMA2Dx).BGOR &
                                    !((0x3fff as libc::c_uint) <<
                                          0 as libc::c_uint) | LineOffset);
}
#[inline]
unsafe extern "C" fn LL_DMA2D_BGND_SetColor(mut DMA2Dx: *mut DMA2D_TypeDef,
                                            mut Red: uint32_t,
                                            mut Green: uint32_t,
                                            mut Blue: uint32_t) {
    ::core::ptr::write_volatile(&mut (*DMA2Dx).BGCOLR as *mut uint32_t,
                                (*DMA2Dx).BGCOLR &
                                    !((0xff as libc::c_uint) <<
                                          16 as libc::c_uint |
                                          (0xff as libc::c_uint) <<
                                              8 as libc::c_uint |
                                          (0xff as libc::c_uint) <<
                                              0 as libc::c_uint) |
                                    (Red << 16 as libc::c_uint |
                                         Green << 8 as libc::c_uint | Blue));
}
#[inline]
unsafe extern "C" fn LL_DMA2D_BGND_SetCLUTMemAddr(mut DMA2Dx:
                                                      *mut DMA2D_TypeDef,
                                                  mut CLUTMemoryAddress:
                                                      uint32_t) {
    ::core::ptr::write_volatile(&mut (*DMA2Dx).BGCMAR as *mut uint32_t,
                                CLUTMemoryAddress);
}
#[inline]
unsafe extern "C" fn LL_DMA2D_BGND_SetMemAddr(mut DMA2Dx: *mut DMA2D_TypeDef,
                                              mut MemoryAddress: uint32_t) {
    ::core::ptr::write_volatile(&mut (*DMA2Dx).BGMAR as *mut uint32_t,
                                MemoryAddress);
}
/* *
  * @brief  Force AHB1 peripherals reset.
  * @rmtoll AHB1RSTR     GPIOARST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     GPIOBRST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     GPIOCRST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     GPIODRST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     GPIOERST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     GPIOFRST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     GPIOGRST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     GPIOHRST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     GPIOIRST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     GPIOJRST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     GPIOKRST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     CRCRST        LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     DMA1RST       LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     DMA2RST       LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     DMA2DRST      LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     ETHMACRST     LL_AHB1_GRP1_ForceReset\n
  *         AHB1RSTR     OTGHSRST      LL_AHB1_GRP1_ForceReset
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOA
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOB
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOD
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOE
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOF
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOG
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOH
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOI
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOJ (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOK (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CRC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_DMA1
  *         @arg @ref LL_AHB1_GRP1_PERIPH_DMA2
  *         @arg @ref LL_AHB1_GRP1_PERIPH_DMA2D (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETHMAC (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_OTGHS
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
#[inline]
unsafe extern "C" fn LL_AHB1_GRP1_ForceReset(mut Periphs: uint32_t) {
    let ref mut fresh0 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1RSTR;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | Periphs) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Release AHB1 peripherals reset.
  * @rmtoll AHB1RSTR     GPIOARST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     GPIOBRST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     GPIOCRST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     GPIODRST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     GPIOERST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     GPIOFRST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     GPIOGRST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     GPIOHRST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     GPIOIRST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     GPIOJRST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     GPIOKRST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     CRCRST        LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     DMA1RST       LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     DMA2RST       LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     DMA2DRST      LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     ETHMACRST     LL_AHB1_GRP1_ReleaseReset\n
  *         AHB1RSTR     OTGHSRST      LL_AHB1_GRP1_ReleaseReset
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ALL
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOA
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOB
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOD
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOE
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOF
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOG
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOH
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOI
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOJ (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_GPIOK (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_CRC
  *         @arg @ref LL_AHB1_GRP1_PERIPH_DMA1
  *         @arg @ref LL_AHB1_GRP1_PERIPH_DMA2
  *         @arg @ref LL_AHB1_GRP1_PERIPH_DMA2D (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_ETHMAC (*)
  *         @arg @ref LL_AHB1_GRP1_PERIPH_OTGHS
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
#[inline]
unsafe extern "C" fn LL_AHB1_GRP1_ReleaseReset(mut Periphs: uint32_t) {
    let ref mut fresh1 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).AHB1RSTR;
    ::core::ptr::write_volatile(fresh1,
                                (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !Periphs) as uint32_t
                                    as uint32_t);
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_ll_dma2d.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   DMA2D LL module driver.
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
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F7xx_LL_Driver
  * @{
  */
/* * @addtogroup DMA2D_LL
  * @{
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* * @addtogroup DMA2D_LL_Private_Constants DMA2D Private Constants
  * @{
  */
/* !< Maximum output color setting                   */
/* !< Maximum number of lines                        */
/* !< Maximum number of pixels per lines             */
/* !< Maximum output line offset expressed in pixels */
/* !< Maximum CLUT size                              */
/* *
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/* * @addtogroup DMA2D_LL_Private_Macros
  * @{
  */
/* DMA2D_ALPHA_INV_RB_SWAP_SUPPORT */
/* *
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup DMA2D_LL_Exported_Functions
  * @{
  */
/* * @addtogroup DMA2D_LL_EF_Init_Functions Initialization and De-initialization Functions
  * @{
  */
/* *
  * @brief  De-initialize DMA2D registers (registers restored to their default values).
  * @param  DMA2Dx DMA2D Instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: DMA2D registers are de-initialized
  *          - ERROR: DMA2D registers are not de-initialized
  */
#[no_mangle]
pub unsafe extern "C" fn LL_DMA2D_DeInit(mut DMA2Dx: *mut DMA2D_TypeDef)
 -> ErrorStatus {
    let mut status: ErrorStatus = SUCCESS;
    /* Check the parameters */
    if DMA2Dx ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0xb000
                                                                              as
                                                                              libc::c_uint)
               as *mut DMA2D_TypeDef {
        /* Force reset of DMA2D clock */
        LL_AHB1_GRP1_ForceReset((0x1 as libc::c_uint) << 23 as libc::c_uint);
        /* Release reset of DMA2D clock */
        LL_AHB1_GRP1_ReleaseReset((0x1 as libc::c_uint) <<
                                      23 as libc::c_uint);
    } else { status = ERROR }
    return status;
}
/* *
  * @brief  Check if the DMA2D Transfer Error interrupt source is enabled or disabled.
  * @rmtoll CR          TEIE        LL_DMA2D_IsEnabledIT_TE
  * @param  DMA2Dx DMA2D Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @}
  */
/* * @defgroup DMA2D_LL_EF_Init_Functions Initialization and De-initialization Functions
  * @{
  */
/* *
  * @brief  Initialize DMA2D registers according to the specified parameters in DMA2D_InitStruct.
  * @note   DMA2D transfers must be disabled to set initialization bits in configuration registers,
  *         otherwise ERROR result is returned.
  * @param  DMA2Dx DMA2D Instance
  * @param  DMA2D_InitStruct: pointer to a LL_DMA2D_InitTypeDef structure
  *         that contains the configuration information for the specified DMA2D peripheral.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: DMA2D registers are initialized according to DMA2D_InitStruct content
  *          - ERROR: Issue occurred during DMA2D registers initialization
  */
#[no_mangle]
pub unsafe extern "C" fn LL_DMA2D_Init(mut DMA2Dx: *mut DMA2D_TypeDef,
                                       mut DMA2D_InitStruct:
                                           *mut LL_DMA2D_InitTypeDef)
 -> ErrorStatus {
    let mut status: ErrorStatus = ERROR;
    let mut DMA2D_ColorStruct: LL_DMA2D_ColorTypeDef =
        LL_DMA2D_ColorTypeDef{ColorMode: 0,
                              OutputBlue: 0,
                              OutputGreen: 0,
                              OutputRed: 0,
                              OutputAlpha: 0,};
    let mut tmp: uint32_t = 0 as libc::c_uint;
    let mut tmp1: uint32_t = 0 as libc::c_uint;
    let mut tmp2: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    /* DMA2D_ALPHA_INV_RB_SWAP_SUPPORT */
    /* DMA2D transfers must be disabled to configure bits in initialization registers */
    tmp = LL_DMA2D_IsTransferOngoing(DMA2Dx);
    tmp1 = LL_DMA2D_FGND_IsEnabledCLUTLoad(DMA2Dx);
    tmp2 = LL_DMA2D_BGND_IsEnabledCLUTLoad(DMA2Dx);
    if tmp == 0 as libc::c_uint && tmp1 == 0 as libc::c_uint &&
           tmp2 == 0 as libc::c_uint {
        /* DMA2D CR register configuration -------------------------------------------*/
        LL_DMA2D_SetMode(DMA2Dx, (*DMA2D_InitStruct).Mode);
        /* DMA2D OPFCCR register configuration ---------------------------------------*/
        ::core::ptr::write_volatile(&mut (*DMA2Dx).OPFCCR as *mut uint32_t,
                                    (*DMA2Dx).OPFCCR &
                                        !((0x7 as libc::c_uint) <<
                                              0 as libc::c_uint) |
                                        (*DMA2D_InitStruct).ColorMode);
        /* DMA2D_ALPHA_INV_RB_SWAP_SUPPORT */
        /* DMA2D OOR register configuration ------------------------------------------*/
        LL_DMA2D_SetLineOffset(DMA2Dx, (*DMA2D_InitStruct).LineOffset);
        /* DMA2D NLR register configuration ------------------------------------------*/
        LL_DMA2D_ConfigSize(DMA2Dx, (*DMA2D_InitStruct).NbrOfLines,
                            (*DMA2D_InitStruct).NbrOfPixelsPerLines);
        /* DMA2D OMAR register configuration ------------------------------------------*/
        LL_DMA2D_SetOutputMemAddr(DMA2Dx,
                                  (*DMA2D_InitStruct).OutputMemoryAddress);
        /* DMA2D OCOLR register configuration ------------------------------------------*/
        DMA2D_ColorStruct.ColorMode = (*DMA2D_InitStruct).ColorMode;
        DMA2D_ColorStruct.OutputBlue = (*DMA2D_InitStruct).OutputBlue;
        DMA2D_ColorStruct.OutputGreen = (*DMA2D_InitStruct).OutputGreen;
        DMA2D_ColorStruct.OutputRed = (*DMA2D_InitStruct).OutputRed;
        DMA2D_ColorStruct.OutputAlpha = (*DMA2D_InitStruct).OutputAlpha;
        LL_DMA2D_ConfigOutputColor(DMA2Dx, &mut DMA2D_ColorStruct);
        status = SUCCESS
    }
    /* If DMA2D transfers are not disabled, return ERROR */
    return status;
}
/* *
  * @brief Set each @ref LL_DMA2D_InitTypeDef field to default value.
  * @param DMA2D_InitStruct: pointer to a @ref LL_DMA2D_InitTypeDef structure
  *                          whose fields will be set to default values.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn LL_DMA2D_StructInit(mut DMA2D_InitStruct:
                                                 *mut LL_DMA2D_InitTypeDef) {
    /* Set DMA2D_InitStruct fields to default values */
    (*DMA2D_InitStruct).Mode = 0 as libc::c_uint;
    (*DMA2D_InitStruct).ColorMode = 0 as libc::c_uint;
    (*DMA2D_InitStruct).LineOffset = 0 as libc::c_uint;
    (*DMA2D_InitStruct).OutputBlue = 0 as libc::c_uint;
    (*DMA2D_InitStruct).OutputGreen = 0 as libc::c_uint;
    (*DMA2D_InitStruct).OutputRed = 0 as libc::c_uint;
    (*DMA2D_InitStruct).OutputAlpha = 0 as libc::c_uint;
    (*DMA2D_InitStruct).OutputMemoryAddress = 0 as libc::c_uint;
    (*DMA2D_InitStruct).NbrOfLines = 0 as libc::c_uint;
    (*DMA2D_InitStruct).NbrOfPixelsPerLines = 0 as libc::c_uint;
    /* DMA2D_ALPHA_INV_RB_SWAP_SUPPORT */
}
/* *
  * @brief  Configure the foreground or background according to the specified parameters
  *         in the LL_DMA2D_LayerCfgTypeDef structure.
  * @param  DMA2Dx DMA2D Instance
  * @param  DMA2D_LayerCfg: pointer to a LL_DMA2D_LayerCfgTypeDef structure that contains
  *         the configuration information for the specified layer.
  * @param  LayerIdx: DMA2D Layer index.
  *                   This parameter can be one of the following values:
  *                   0(background) / 1(foreground)
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn LL_DMA2D_ConfigLayer(mut DMA2Dx: *mut DMA2D_TypeDef,
                                              mut DMA2D_LayerCfg:
                                                  *mut LL_DMA2D_LayerCfgTypeDef,
                                              mut LayerIdx: uint32_t) {
    /* Check the parameters */
    /* DMA2D_ALPHA_INV_RB_SWAP_SUPPORT */
    if LayerIdx == 0 as libc::c_uint {
        /* Configure the background memory address */
        LL_DMA2D_BGND_SetMemAddr(DMA2Dx, (*DMA2D_LayerCfg).MemoryAddress);
        /* Configure the background line offset */
        LL_DMA2D_BGND_SetLineOffset(DMA2Dx, (*DMA2D_LayerCfg).LineOffset);
        /* Configure the background Alpha value, Alpha mode, CLUT size, CLUT Color mode and Color mode */
        ::core::ptr::write_volatile(&mut (*DMA2Dx).BGPFCCR as *mut uint32_t,
                                    (*DMA2Dx).BGPFCCR &
                                        !((0xff as libc::c_uint) <<
                                              24 as libc::c_uint |
                                              (0x3 as libc::c_uint) <<
                                                  16 as libc::c_uint |
                                              (0xff as libc::c_uint) <<
                                                  8 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  4 as libc::c_uint |
                                              (0xf as libc::c_uint) <<
                                                  0 as libc::c_uint) |
                                        ((*DMA2D_LayerCfg).Alpha <<
                                             24 as libc::c_uint |
                                             (*DMA2D_LayerCfg).AlphaMode |
                                             (*DMA2D_LayerCfg).CLUTSize <<
                                                 8 as libc::c_uint |
                                             (*DMA2D_LayerCfg).CLUTColorMode |
                                             (*DMA2D_LayerCfg).ColorMode));
        /* DMA2D_ALPHA_INV_RB_SWAP_SUPPORT */
        /* Configure the background color */
        LL_DMA2D_BGND_SetColor(DMA2Dx, (*DMA2D_LayerCfg).Red,
                               (*DMA2D_LayerCfg).Green,
                               (*DMA2D_LayerCfg).Blue);
        /* Configure the background CLUT memory address */
        LL_DMA2D_BGND_SetCLUTMemAddr(DMA2Dx,
                                     (*DMA2D_LayerCfg).CLUTMemoryAddress);
    } else {
        /* Configure the foreground memory address */
        LL_DMA2D_FGND_SetMemAddr(DMA2Dx, (*DMA2D_LayerCfg).MemoryAddress);
        /* Configure the foreground line offset */
        LL_DMA2D_FGND_SetLineOffset(DMA2Dx, (*DMA2D_LayerCfg).LineOffset);
        /* Configure the foreground Alpha value, Alpha mode, CLUT size, CLUT Color mode and Color mode */
        ::core::ptr::write_volatile(&mut (*DMA2Dx).FGPFCCR as *mut uint32_t,
                                    (*DMA2Dx).FGPFCCR &
                                        !((0xff as libc::c_uint) <<
                                              24 as libc::c_uint |
                                              (0x3 as libc::c_uint) <<
                                                  16 as libc::c_uint |
                                              (0xff as libc::c_uint) <<
                                                  8 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  4 as libc::c_uint |
                                              (0xf as libc::c_uint) <<
                                                  0 as libc::c_uint) |
                                        ((*DMA2D_LayerCfg).Alpha <<
                                             24 as libc::c_uint |
                                             (*DMA2D_LayerCfg).AlphaMode |
                                             (*DMA2D_LayerCfg).CLUTSize <<
                                                 8 as libc::c_uint |
                                             (*DMA2D_LayerCfg).CLUTColorMode |
                                             (*DMA2D_LayerCfg).ColorMode));
        /* DMA2D_ALPHA_INV_RB_SWAP_SUPPORT */
        /* Configure the foreground color */
        LL_DMA2D_FGND_SetColor(DMA2Dx, (*DMA2D_LayerCfg).Red,
                               (*DMA2D_LayerCfg).Green,
                               (*DMA2D_LayerCfg).Blue);
        /* Configure the foreground CLUT memory address */
        LL_DMA2D_FGND_SetCLUTMemAddr(DMA2Dx,
                                     (*DMA2D_LayerCfg).CLUTMemoryAddress);
    };
}
/* *
  * @brief Set each @ref LL_DMA2D_LayerCfgTypeDef field to default value.
  * @param DMA2D_LayerCfg: pointer to a @ref LL_DMA2D_LayerCfgTypeDef structure
  *                        whose fields will be set to default values.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn LL_DMA2D_LayerCfgStructInit(mut DMA2D_LayerCfg:
                                                         *mut LL_DMA2D_LayerCfgTypeDef) {
    /* Set DMA2D_LayerCfg fields to default values */
    (*DMA2D_LayerCfg).MemoryAddress = 0 as libc::c_uint;
    (*DMA2D_LayerCfg).ColorMode = 0 as libc::c_uint;
    (*DMA2D_LayerCfg).LineOffset = 0 as libc::c_uint;
    (*DMA2D_LayerCfg).CLUTColorMode = 0 as libc::c_uint;
    (*DMA2D_LayerCfg).CLUTSize = 0 as libc::c_uint;
    (*DMA2D_LayerCfg).AlphaMode = 0 as libc::c_uint;
    (*DMA2D_LayerCfg).Alpha = 0 as libc::c_uint;
    (*DMA2D_LayerCfg).Blue = 0 as libc::c_uint;
    (*DMA2D_LayerCfg).Green = 0 as libc::c_uint;
    (*DMA2D_LayerCfg).Red = 0 as libc::c_uint;
    (*DMA2D_LayerCfg).CLUTMemoryAddress = 0 as libc::c_uint;
    /* DMA2D_ALPHA_INV_RB_SWAP_SUPPORT */
}
/* *
  * @brief  Initialize DMA2D output color register according to the specified parameters
  *         in DMA2D_ColorStruct.
  * @param  DMA2Dx DMA2D Instance
  * @param  DMA2D_ColorStruct: pointer to a LL_DMA2D_ColorTypeDef structure that contains
  *         the color configuration information for the specified DMA2D peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn LL_DMA2D_ConfigOutputColor(mut DMA2Dx:
                                                        *mut DMA2D_TypeDef,
                                                    mut DMA2D_ColorStruct:
                                                        *mut LL_DMA2D_ColorTypeDef) {
    let mut outgreen: uint32_t = 0 as libc::c_uint;
    let mut outred: uint32_t = 0 as libc::c_uint;
    let mut outalpha: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    /* DMA2D OCOLR register configuration ------------------------------------------*/
    if (*DMA2D_ColorStruct).ColorMode == 0 as libc::c_uint {
        outgreen = (*DMA2D_ColorStruct).OutputGreen << 8 as libc::c_uint;
        outred = (*DMA2D_ColorStruct).OutputRed << 16 as libc::c_uint;
        outalpha = (*DMA2D_ColorStruct).OutputAlpha << 24 as libc::c_uint
    } else if (*DMA2D_ColorStruct).ColorMode ==
                  (0x1 as libc::c_uint) << 0 as libc::c_uint {
        outgreen = (*DMA2D_ColorStruct).OutputGreen << 8 as libc::c_uint;
        outred = (*DMA2D_ColorStruct).OutputRed << 16 as libc::c_uint;
        outalpha = 0 as libc::c_uint
    } else if (*DMA2D_ColorStruct).ColorMode ==
                  (0x2 as libc::c_uint) << 0 as libc::c_uint {
        outgreen = (*DMA2D_ColorStruct).OutputGreen << 5 as libc::c_uint;
        outred = (*DMA2D_ColorStruct).OutputRed << 11 as libc::c_uint;
        outalpha = 0 as libc::c_uint
    } else if (*DMA2D_ColorStruct).ColorMode ==
                  (0x1 as libc::c_uint) << 0 as libc::c_uint |
                      (0x2 as libc::c_uint) << 0 as libc::c_uint {
        outgreen = (*DMA2D_ColorStruct).OutputGreen << 5 as libc::c_uint;
        outred = (*DMA2D_ColorStruct).OutputRed << 10 as libc::c_uint;
        outalpha = (*DMA2D_ColorStruct).OutputAlpha << 15 as libc::c_uint
    } else {
        /* ColorMode = LL_DMA2D_OUTPUT_MODE_ARGB4444 */
        outgreen = (*DMA2D_ColorStruct).OutputGreen << 4 as libc::c_uint;
        outred = (*DMA2D_ColorStruct).OutputRed << 8 as libc::c_uint;
        outalpha = (*DMA2D_ColorStruct).OutputAlpha << 12 as libc::c_uint
    }
    LL_DMA2D_SetOutputColor(DMA2Dx,
                            outgreen | outred |
                                (*DMA2D_ColorStruct).OutputBlue | outalpha);
}
/* *
  * @brief  Return DMA2D output Blue color.
  * @param  DMA2Dx DMA2D Instance.
  * @param  ColorMode This parameter can be one of the following values:
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_ARGB8888
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_RGB888
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_RGB565
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_ARGB1555
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_ARGB4444
  * @retval Output Blue color value between Min_Data=0 and Max_Data=0xFF
  */
#[no_mangle]
pub unsafe extern "C" fn LL_DMA2D_GetOutputBlueColor(mut DMA2Dx:
                                                         *mut DMA2D_TypeDef,
                                                     mut ColorMode: uint32_t)
 -> uint32_t {
    let mut color: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    /* DMA2D OCOLR register reading ------------------------------------------*/
    if ColorMode == 0 as libc::c_uint {
        color = (*DMA2Dx).OCOLR & 0xff as libc::c_uint
    } else if ColorMode == (0x1 as libc::c_uint) << 0 as libc::c_uint {
        color = (*DMA2Dx).OCOLR & 0xff as libc::c_uint
    } else if ColorMode == (0x2 as libc::c_uint) << 0 as libc::c_uint {
        color = (*DMA2Dx).OCOLR & 0x1f as libc::c_uint
    } else if ColorMode ==
                  (0x1 as libc::c_uint) << 0 as libc::c_uint |
                      (0x2 as libc::c_uint) << 0 as libc::c_uint {
        color = (*DMA2Dx).OCOLR & 0x1f as libc::c_uint
    } else {
        /* ColorMode = LL_DMA2D_OUTPUT_MODE_ARGB4444 */
        color = (*DMA2Dx).OCOLR & 0xf as libc::c_uint
    }
    return color;
}
/* *
  * @brief  Return DMA2D output Green color.
  * @param  DMA2Dx DMA2D Instance.
  * @param  ColorMode This parameter can be one of the following values:
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_ARGB8888
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_RGB888
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_RGB565
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_ARGB1555
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_ARGB4444
  * @retval Output Green color value between Min_Data=0 and Max_Data=0xFF
  */
#[no_mangle]
pub unsafe extern "C" fn LL_DMA2D_GetOutputGreenColor(mut DMA2Dx:
                                                          *mut DMA2D_TypeDef,
                                                      mut ColorMode: uint32_t)
 -> uint32_t {
    let mut color: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    /* DMA2D OCOLR register reading ------------------------------------------*/
    if ColorMode == 0 as libc::c_uint {
        color =
            ((*DMA2Dx).OCOLR & 0xff00 as libc::c_uint) >> 8 as libc::c_uint
    } else if ColorMode == (0x1 as libc::c_uint) << 0 as libc::c_uint {
        color =
            ((*DMA2Dx).OCOLR & 0xff00 as libc::c_uint) >> 8 as libc::c_uint
    } else if ColorMode == (0x2 as libc::c_uint) << 0 as libc::c_uint {
        color = ((*DMA2Dx).OCOLR & 0x7e0 as libc::c_uint) >> 5 as libc::c_uint
    } else if ColorMode ==
                  (0x1 as libc::c_uint) << 0 as libc::c_uint |
                      (0x2 as libc::c_uint) << 0 as libc::c_uint {
        color = ((*DMA2Dx).OCOLR & 0x3e0 as libc::c_uint) >> 5 as libc::c_uint
    } else {
        /* ColorMode = LL_DMA2D_OUTPUT_MODE_ARGB4444 */
        color = ((*DMA2Dx).OCOLR & 0xf0 as libc::c_uint) >> 4 as libc::c_uint
    }
    return color;
}
/* *
  * @brief  Return DMA2D output Red color.
  * @param  DMA2Dx DMA2D Instance.
  * @param  ColorMode This parameter can be one of the following values:
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_ARGB8888
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_RGB888
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_RGB565
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_ARGB1555
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_ARGB4444
  * @retval Output Red color value between Min_Data=0 and Max_Data=0xFF
  */
#[no_mangle]
pub unsafe extern "C" fn LL_DMA2D_GetOutputRedColor(mut DMA2Dx:
                                                        *mut DMA2D_TypeDef,
                                                    mut ColorMode: uint32_t)
 -> uint32_t {
    let mut color: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    /* DMA2D OCOLR register reading ------------------------------------------*/
    if ColorMode == 0 as libc::c_uint {
        color =
            ((*DMA2Dx).OCOLR & 0xff0000 as libc::c_uint) >> 16 as libc::c_uint
    } else if ColorMode == (0x1 as libc::c_uint) << 0 as libc::c_uint {
        color =
            ((*DMA2Dx).OCOLR & 0xff0000 as libc::c_uint) >> 16 as libc::c_uint
    } else if ColorMode == (0x2 as libc::c_uint) << 0 as libc::c_uint {
        color =
            ((*DMA2Dx).OCOLR & 0xf800 as libc::c_uint) >> 11 as libc::c_uint
    } else if ColorMode ==
                  (0x1 as libc::c_uint) << 0 as libc::c_uint |
                      (0x2 as libc::c_uint) << 0 as libc::c_uint {
        color =
            ((*DMA2Dx).OCOLR & 0x7c00 as libc::c_uint) >> 10 as libc::c_uint
    } else {
        /* ColorMode = LL_DMA2D_OUTPUT_MODE_ARGB4444 */
        color = ((*DMA2Dx).OCOLR & 0xf00 as libc::c_uint) >> 8 as libc::c_uint
    }
    return color;
}
/* *
  * @brief  Return DMA2D output Alpha color.
  * @param  DMA2Dx DMA2D Instance.
  * @param  ColorMode This parameter can be one of the following values:
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_ARGB8888
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_RGB888
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_RGB565
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_ARGB1555
  *         @arg @ref LL_DMA2D_OUTPUT_MODE_ARGB4444
  * @retval Output Alpha color value between Min_Data=0 and Max_Data=0xFF
  */
#[no_mangle]
pub unsafe extern "C" fn LL_DMA2D_GetOutputAlphaColor(mut DMA2Dx:
                                                          *mut DMA2D_TypeDef,
                                                      mut ColorMode: uint32_t)
 -> uint32_t {
    let mut color: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    /* DMA2D OCOLR register reading ------------------------------------------*/
    if ColorMode == 0 as libc::c_uint {
        color =
            ((*DMA2Dx).OCOLR & 0xff000000 as libc::c_uint) >>
                24 as libc::c_uint
    } else if ColorMode == (0x1 as libc::c_uint) << 0 as libc::c_uint ||
                  ColorMode == (0x2 as libc::c_uint) << 0 as libc::c_uint {
        color = 0 as libc::c_uint
    } else if ColorMode ==
                  (0x1 as libc::c_uint) << 0 as libc::c_uint |
                      (0x2 as libc::c_uint) << 0 as libc::c_uint {
        color =
            ((*DMA2Dx).OCOLR & 0x8000 as libc::c_uint) >> 15 as libc::c_uint
    } else {
        /* ColorMode = LL_DMA2D_OUTPUT_MODE_ARGB4444 */
        color =
            ((*DMA2Dx).OCOLR & 0xf000 as libc::c_uint) >> 12 as libc::c_uint
    }
    return color;
}
/* *
  * @brief  Configure DMA2D transfer size.
  * @param  DMA2Dx DMA2D Instance
  * @param  NbrOfLines Value between Min_Data=0 and Max_Data=0xFFFF
  * @param  NbrOfPixelsPerLines Value between Min_Data=0 and Max_Data=0x3FFF
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn LL_DMA2D_ConfigSize(mut DMA2Dx: *mut DMA2D_TypeDef,
                                             mut NbrOfLines: uint32_t,
                                             mut NbrOfPixelsPerLines:
                                                 uint32_t) {
    ::core::ptr::write_volatile(&mut (*DMA2Dx).NLR as *mut uint32_t,
                                (*DMA2Dx).NLR &
                                    !((0x3fff as libc::c_uint) <<
                                          16 as libc::c_uint |
                                          (0xffff as libc::c_uint) <<
                                              0 as libc::c_uint) |
                                    (NbrOfPixelsPerLines << 16 as libc::c_uint
                                         | NbrOfLines));
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* USE_FULL_LL_DRIVER */
/* *
  * @}
  */
/* defined (DMA2D) */
/* *
  * @}
  */
/* *
  * @}
  */
/* *
  * @}
  */
