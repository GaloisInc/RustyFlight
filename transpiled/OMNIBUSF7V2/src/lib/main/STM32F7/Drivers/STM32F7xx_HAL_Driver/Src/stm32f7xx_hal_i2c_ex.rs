use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
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
  * @brief Inter-integrated Circuit Interface
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct I2C_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub OAR1: uint32_t,
    pub OAR2: uint32_t,
    pub TIMINGR: uint32_t,
    pub TIMEOUTR: uint32_t,
    pub ISR: uint32_t,
    pub ICR: uint32_t,
    pub PECR: uint32_t,
    pub RXDR: uint32_t,
    pub TXDR: uint32_t,
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
  * @file    stm32f7xx_hal_i2c.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of I2C HAL module.
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
/* * @addtogroup I2C
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup I2C_Exported_Types I2C Exported Types
  * @{
  */
/* * @defgroup I2C_Configuration_Structure_definition I2C Configuration Structure definition
  * @brief  I2C Configuration Structure definition  
  * @{
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct I2C_InitTypeDef {
    pub Timing: uint32_t,
    pub OwnAddress1: uint32_t,
    pub AddressingMode: uint32_t,
    pub DualAddressMode: uint32_t,
    pub OwnAddress2: uint32_t,
    pub OwnAddress2Masks: uint32_t,
    pub GeneralCallMode: uint32_t,
    pub NoStretchMode: uint32_t,
}
/* * 
  * @}
  */
/* * @defgroup HAL_state_structure_definition HAL state structure definition
  * @brief  HAL State structure definition
  * @note  HAL I2C State value coding follow below described bitmap :\n
  *          b7-b6  Error information\n
  *             00 : No Error\n
  *             01 : Abort (Abort user request on going)\n
  *             10 : Timeout\n
  *             11 : Error\n
  *          b5     IP initilisation status\n
  *             0  : Reset (IP not initialized)\n
  *             1  : Init done (IP initialized and ready to use. HAL I2C Init function called)\n
  *          b4     (not used)\n
  *             x  : Should be set to 0\n
  *          b3\n
  *             0  : Ready or Busy (No Listen mode ongoing)\n
  *             1  : Listen (IP in Address Listen Mode)\n
  *          b2     Intrinsic process state\n
  *             0  : Ready\n
  *             1  : Busy (IP busy with some configuration or internal operations)\n
  *          b1     Rx state\n
  *             0  : Ready (no Rx operation ongoing)\n
  *             1  : Busy (Rx operation ongoing)\n
  *          b0     Tx state\n
  *             0  : Ready (no Tx operation ongoing)\n
  *             1  : Busy (Tx operation ongoing)
  * @{
  */
pub type HAL_I2C_StateTypeDef = libc::c_uint;
/* !< Error                                     */
/* !< Timeout state                             */
pub const HAL_I2C_STATE_ERROR: HAL_I2C_StateTypeDef = 224;
/* !< Abort user request ongoing                */
pub const HAL_I2C_STATE_TIMEOUT: HAL_I2C_StateTypeDef = 160;
/* !< Address Listen Mode and Data Reception
                                                 process is ongoing                         */
pub const HAL_I2C_STATE_ABORT: HAL_I2C_StateTypeDef = 96;
/* !< Address Listen Mode and Data Transmission
                                                 process is ongoing                         */
pub const HAL_I2C_STATE_BUSY_RX_LISTEN: HAL_I2C_StateTypeDef = 42;
/* !< Address Listen Mode is ongoing            */
pub const HAL_I2C_STATE_BUSY_TX_LISTEN: HAL_I2C_StateTypeDef = 41;
/* !< Data Reception process is ongoing         */
pub const HAL_I2C_STATE_LISTEN: HAL_I2C_StateTypeDef = 40;
/* !< Data Transmission process is ongoing      */
pub const HAL_I2C_STATE_BUSY_RX: HAL_I2C_StateTypeDef = 34;
/* !< An internal process is ongoing            */
pub const HAL_I2C_STATE_BUSY_TX: HAL_I2C_StateTypeDef = 33;
/* !< Peripheral Initialized and ready for use  */
pub const HAL_I2C_STATE_BUSY: HAL_I2C_StateTypeDef = 36;
/* !< Peripheral is not yet Initialized         */
pub const HAL_I2C_STATE_READY: HAL_I2C_StateTypeDef = 32;
pub const HAL_I2C_STATE_RESET: HAL_I2C_StateTypeDef = 0;
/* *
  * @}
  */
/* * @defgroup HAL_mode_structure_definition HAL mode structure definition
  * @brief  HAL Mode structure definition
  * @note  HAL I2C Mode value coding follow below described bitmap :\n
  *          b7     (not used)\n
  *             x  : Should be set to 0\n
  *          b6\n
  *             0  : None\n
  *             1  : Memory (HAL I2C communication is in Memory Mode)\n
  *          b5\n
  *             0  : None\n
  *             1  : Slave (HAL I2C communication is in Slave Mode)\n
  *          b4\n
  *             0  : None\n
  *             1  : Master (HAL I2C communication is in Master Mode)\n
  *          b3-b2-b1-b0  (not used)\n
  *             xxxx : Should be set to 0000
  * @{
  */
pub type HAL_I2C_ModeTypeDef = libc::c_uint;
/* !< I2C communication is in Memory Mode       */
/* !< I2C communication is in Slave Mode        */
pub const HAL_I2C_MODE_MEM: HAL_I2C_ModeTypeDef = 64;
/* !< I2C communication is in Master Mode       */
pub const HAL_I2C_MODE_SLAVE: HAL_I2C_ModeTypeDef = 32;
/* !< No I2C communication on going             */
pub const HAL_I2C_MODE_MASTER: HAL_I2C_ModeTypeDef = 16;
pub const HAL_I2C_MODE_NONE: HAL_I2C_ModeTypeDef = 0;
/* * 
  * @}
  */
/* * @defgroup I2C_Error_Code_definition I2C Error Code definition
  * @brief  I2C Error Code definition
  * @{
  */
/* !< No error              */
/* !< BERR error            */
/* !< ARLO error            */
/* !< ACKF error            */
/* !< OVR error             */
/* !< DMA transfer error    */
/* !< Timeout error         */
/* !< Size Management error */
/* *
  * @}
  */
/* * @defgroup I2C_handle_Structure_definition I2C handle Structure definition
  * @brief  I2C handle Structure definition
  * @{
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct __I2C_HandleTypeDef {
    pub Instance: *mut I2C_TypeDef,
    pub Init: I2C_InitTypeDef,
    pub pBuffPtr: *mut uint8_t,
    pub XferSize: uint16_t,
    pub XferCount: uint16_t,
    pub XferOptions: uint32_t,
    pub PreviousState: uint32_t,
    pub XferISR: Option<unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                             _: uint32_t, _: uint32_t)
                            -> HAL_StatusTypeDef>,
    pub hdmatx: *mut DMA_HandleTypeDef,
    pub hdmarx: *mut DMA_HandleTypeDef,
    pub Lock: HAL_LockTypeDef,
    pub State: HAL_I2C_StateTypeDef,
    pub Mode: HAL_I2C_ModeTypeDef,
    pub ErrorCode: uint32_t,
    pub AddrEventCount: uint32_t,
}
pub type I2C_HandleTypeDef = __I2C_HandleTypeDef;
/* !< I2C Address Event counter                 */
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_i2c_ex.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   I2C Extended HAL module driver.
  *          This file provides firmware functions to manage the following 
  *          functionalities of I2C Extended peripheral:
  *           + Extended features functions
  *
  @verbatim
  ==============================================================================
               ##### I2C peripheral Extended features  #####
  ==============================================================================

  [..] Comparing to other previous devices, the I2C interface for STM32F7xx
       devices contains the following additional features

       (+) Possibility to disable or enable Analog Noise Filter
       (+) Use of a configured Digital Noise Filter
       (+) Disable or enable Fast Mode Plus

                     ##### How to use this driver #####
  ==============================================================================
  [..] This driver provides functions to:
    (#) Configure I2C Analog noise filter using the function HAL_I2CEx_ConfigAnalogFilter()
    (#) Configure I2C Digital noise filter using the function HAL_I2CEx_ConfigDigitalFilter()
    (#) Configure the enable or disable of fast mode plus driving capability using the functions :
          (++) HAL_I2CEx_EnableFastModePlus()
          (++) HAL_I2CEx_DisableFastModePlus()
  @endverbatim
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
/* * @addtogroup STM32F7xx_HAL_Driver
  * @{
  */
/* * @defgroup I2CEx I2CEx
  * @brief I2C Extended HAL module driver
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @defgroup I2CEx_Exported_Functions I2C Extended Exported Functions
  * @{
  */
/* * @defgroup I2CEx_Exported_Functions_Group1 Extended features functions
  * @brief    Extended features functions
 *
@verbatim
 ===============================================================================
                      ##### Extended features functions #####
 ===============================================================================
    [..] This section provides functions allowing to:
      (+) Configure Noise Filters 
      (+) Configure Fast Mode Plus

@endverbatim
  * @{
  */
/* *
  * @brief  Configure I2C Analog noise filter.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2Cx peripheral.
  * @param  AnalogFilter New state of the Analog filter.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2CEx_ConfigAnalogFilter(mut hi2c:
                                                          *mut I2C_HandleTypeDef,
                                                      mut AnalogFilter:
                                                          uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY);
        /* Disable the selected I2C peripheral */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               0 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* Reset I2Cx ANOFF bit */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               12 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Set analog filter bit*/
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | AnalogFilter) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_READY);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_i2c_ex.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of I2C HAL Extended module.
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
/* * @addtogroup I2CEx
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* * @defgroup I2CEx_Exported_Constants I2C Extended Exported Constants
  * @{
  */
/* * @defgroup I2CEx_Analog_Filter I2C Extended Analog Filter
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2CEx_FastModePlus I2C Extended Fast Mode Plus
  * @{
  */
/* !< Fast Mode Plus not supported       */
/* !< Fast Mode Plus PB6 not supported   */
/* !< Fast Mode Plus PB7 not supported   */
/* !< Fast Mode Plus PB8 not supported   */
/* !< Fast Mode Plus PB9 not supported   */
/* !< Fast Mode Plus I2C1 not supported  */
/* !< Fast Mode Plus I2C2 not supported  */
/* !< Fast Mode Plus I2C3 not supported  */
/* !< Fast Mode Plus I2C4 not supported  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup I2CEx_Exported_Functions I2C Extended Exported Functions
  * @{
  */
/* * @addtogroup I2CEx_Exported_Functions_Group1 Extended features functions
  * @brief    Extended features functions
  * @{
  */
/* Peripheral Control functions  ************************************************/
/* *
  * @brief  Configure I2C Digital noise filter.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2Cx peripheral.
  * @param  DigitalFilter Coefficient of digital noise filter between Min_Data=0x00 and Max_Data=0x0F.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2CEx_ConfigDigitalFilter(mut hi2c:
                                                           *mut I2C_HandleTypeDef,
                                                       mut DigitalFilter:
                                                           uint32_t)
 -> HAL_StatusTypeDef {
    let mut tmpreg: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY);
        /* Disable the selected I2C peripheral */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               0 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* Get the old register value */
        tmpreg = (*(*hi2c).Instance).CR1;
        /* Reset I2Cx DNF bits [11:8] */
        tmpreg &= !((0xf as libc::c_uint) << 8 as libc::c_uint);
        /* Set I2Cx DNF coefficient */
        tmpreg |= DigitalFilter << 8 as libc::c_uint;
        /* Store the new register value */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as
                                        *mut uint32_t, tmpreg);
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_READY);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* HAL_I2C_MODULE_ENABLED */
/* *
  * @}
  */
/* *
  * @}
  */
/* SYSCFG_PMC_I2C1_FMP */
