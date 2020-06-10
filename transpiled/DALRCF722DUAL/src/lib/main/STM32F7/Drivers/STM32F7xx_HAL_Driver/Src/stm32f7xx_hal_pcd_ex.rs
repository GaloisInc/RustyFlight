use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
pub type C2RustUnnamed = libc::c_uint;
pub const ENABLE: C2RustUnnamed = 1;
pub const DISABLE: C2RustUnnamed = 0;
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
pub type PCD_LPM_MsgTypeDef = libc::c_uint;
pub const PCD_LPM_L1_ACTIVE: PCD_LPM_MsgTypeDef = 1;
pub const PCD_LPM_L0_ACTIVE: PCD_LPM_MsgTypeDef = 0;
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_pcd_ex.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   PCD HAL module driver.
  *          This file provides firmware functions to manage the following 
  *          functionalities of the USB Peripheral Controller:
  *           + Extended features functions
  *
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
/* * @defgroup PCDEx PCDEx
  * @brief PCD Extended HAL module driver
  * @{
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* * @defgroup PCDEx_Exported_Functions PCDEx Exported Functions
  * @{
  */
/* * @defgroup PCDEx_Exported_Functions_Group1 Peripheral Control functions
  * @brief    PCDEx control functions 
 *
@verbatim   
 ===============================================================================
                 ##### Extended features functions #####
 ===============================================================================  
    [..]  This section provides functions allowing to:
      (+) Update FIFO configuration

@endverbatim
  * @{
  */
/* *
  * @brief  Set Tx FIFO
  * @param  hpcd: PCD handle
  * @param  fifo: The number of Tx fifo
  * @param  size: Fifo size
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCDEx_SetTxFiFo(mut hpcd: *mut PCD_HandleTypeDef,
                                             mut fifo: uint8_t,
                                             mut size: uint16_t)
 -> HAL_StatusTypeDef {
    let mut i: uint8_t = 0 as libc::c_int as uint8_t;
    let mut Tx_Offset: uint32_t = 0 as libc::c_int as uint32_t;
    /*  TXn min size = 16 words. (n  : Transmit FIFO index)
      When a TxFIFO is not used, the Configuration should be as follows: 
          case 1 :  n > m    and Txn is not used    (n,m  : Transmit FIFO indexes)
         --> Txm can use the space allocated for Txn.
         case2  :  n < m    and Txn is not used    (n,m  : Transmit FIFO indexes)
         --> Txn should be configured with the minimum space of 16 words
     The FIFO is used optimally when used TxFIFOs are allocated in the top 
         of the FIFO.Ex: use EP1 and EP2 as IN instead of EP1 and EP3 as IN ones.
     When DMA is used 3n * FIFO locations should be reserved for internal DMA registers */
    Tx_Offset = (*(*hpcd).Instance).GRXFSIZ;
    if fifo as libc::c_int == 0 as libc::c_int {
        ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).DIEPTXF0_HNPTXFSIZ
                                        as *mut uint32_t,
                                    (size as uint32_t) << 16 as libc::c_int |
                                        Tx_Offset)
    } else {
        Tx_Offset =
            (Tx_Offset as
                 libc::c_uint).wrapping_add((*(*hpcd).Instance).DIEPTXF0_HNPTXFSIZ
                                                >> 16 as libc::c_int) as
                uint32_t as uint32_t;
        i = 0 as libc::c_int as uint8_t;
        while (i as libc::c_int) < fifo as libc::c_int - 1 as libc::c_int {
            Tx_Offset =
                (Tx_Offset as
                     libc::c_uint).wrapping_add((*(*hpcd).Instance).DIEPTXF[i
                                                                                as
                                                                                usize]
                                                    >> 16 as libc::c_int) as
                    uint32_t as uint32_t;
            i = i.wrapping_add(1)
        }
        /* Multiply Tx_Size by 2 to get higher performance */
        ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).DIEPTXF[(fifo as
                                                                          libc::c_int
                                                                          -
                                                                          1 as
                                                                              libc::c_int)
                                                                         as
                                                                         usize]
                                        as *mut uint32_t,
                                    (size as uint32_t) << 16 as libc::c_int |
                                        Tx_Offset)
    }
    return HAL_OK;
}
/* *
  * @brief  Set Rx FIFO
  * @param  hpcd: PCD handle
  * @param  size: Size of Rx fifo
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCDEx_SetRxFiFo(mut hpcd: *mut PCD_HandleTypeDef,
                                             mut size: uint16_t)
 -> HAL_StatusTypeDef {
    ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GRXFSIZ as
                                    *mut uint32_t, size as uint32_t);
    return HAL_OK;
}
/* *
  * @brief  Activate LPM Feature
  * @param  hpcd: PCD handle
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCDEx_ActivateLPM(mut hpcd:
                                                   *mut PCD_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut USBx: *mut USB_OTG_GlobalTypeDef = (*hpcd).Instance;
    (*hpcd).lpm_active = ENABLE as libc::c_int as uint32_t;
    (*hpcd).LPM_State = LPM_L0;
    ::core::ptr::write_volatile(&mut (*USBx).GINTMSK as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GINTMSK
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         27 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*USBx).GLPMCFG as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GLPMCFG
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((0x1 as libc::c_uint) <<
                                          0 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              1 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              28 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    return HAL_OK;
}
/* *
  * @brief  DeActivate LPM feature.
  * @param  hpcd: PCD handle
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCDEx_DeActivateLPM(mut hpcd:
                                                     *mut PCD_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut USBx: *mut USB_OTG_GlobalTypeDef = (*hpcd).Instance;
    (*hpcd).lpm_active = DISABLE as libc::c_int as uint32_t;
    ::core::ptr::write_volatile(&mut (*USBx).GINTMSK as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GINTMSK
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           27 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*USBx).GLPMCFG as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GLPMCFG
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               1 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               28 as libc::c_uint)) as
                                    uint32_t as uint32_t);
    return HAL_OK;
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_pcd_ex.h
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
/* * @addtogroup PCDEx
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* on */
/* LPM L1 sleep */
/* Exported constants --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup PCDEx_Exported_Functions PCDEx Exported Functions
  * @{
  */
/* * @addtogroup PCDEx_Exported_Functions_Group1 Peripheral Control functions
  * @{
  */
/* USB_OTG_GCCFG_BCDEN */
/* *
  * @brief  Send LPM message to user layer callback.
  * @param  hpcd: PCD handle
  * @param  msg: LPM message
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCDEx_LPM_Callback(mut hpcd:
                                                    *mut PCD_HandleTypeDef,
                                                mut msg: PCD_LPM_MsgTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCDEx_LPM_Callback could be implemented in the user file
   */
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* HAL_PCD_MODULE_ENABLED */
/* *
  * @}
  */
/* *
  * @}
  */
