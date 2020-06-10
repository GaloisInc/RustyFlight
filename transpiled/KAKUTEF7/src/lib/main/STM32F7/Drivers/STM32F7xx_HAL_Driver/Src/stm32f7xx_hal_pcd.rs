use ::libc;
extern "C" {
    #[no_mangle]
    fn HAL_RCC_GetHCLKFreq() -> uint32_t;
    /* Exported constants --------------------------------------------------------*/
    /* * @defgroup PCD_Exported_Constants PCD Exported Constants
  * @{
  */
    /* * @defgroup USB_Core_Mode_ USB Core Mode
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup USB_Core_Speed_   USB Core Speed
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup USB_Core_PHY_   USB Core PHY
  * @{
  */
    /* !< Value of USB HS PHY Tune */
    /* USB_HS_PHYC_TUNE_VALUE */
    /* *
  * @}
  */
    /* * @defgroup USB_Core_MPS_   USB Core MPS
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup USB_Core_Phy_Frequency_   USB Core Phy Frequency
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup USB_CORE_Frame_Interval_   USB CORE Frame Interval
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup USB_EP0_MPS_  USB EP0 MPS
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup USB_EP_Speed_  USB EP Speed
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup USB_EP_Type_  USB EP Type
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup USB_STS_Defines_   USB STS Defines
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup HCFG_SPEED_Defines_   HCFG SPEED Defines
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup HPRT0_PRTSPD_SPEED_Defines_  HPRT0 PRTSPD SPEED Defines
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
    #[no_mangle]
    fn USB_CoreInit(USBx: *mut USB_OTG_GlobalTypeDef,
                    Init: USB_OTG_CfgTypeDef) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn USB_DevInit(USBx: *mut USB_OTG_GlobalTypeDef, Init: USB_OTG_CfgTypeDef)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn USB_EnableGlobalInt(USBx: *mut USB_OTG_GlobalTypeDef)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn USB_DisableGlobalInt(USBx: *mut USB_OTG_GlobalTypeDef)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn USB_SetCurrentMode(USBx: *mut USB_OTG_GlobalTypeDef,
                          mode: USB_OTG_ModeTypeDef) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn USB_FlushRxFifo(USBx: *mut USB_OTG_GlobalTypeDef) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn USB_FlushTxFifo(USBx: *mut USB_OTG_GlobalTypeDef, num: uint32_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn USB_ActivateEndpoint(USBx: *mut USB_OTG_GlobalTypeDef,
                            ep: *mut USB_OTG_EPTypeDef) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn USB_DeactivateEndpoint(USBx: *mut USB_OTG_GlobalTypeDef,
                              ep: *mut USB_OTG_EPTypeDef)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn USB_EPStartXfer(USBx: *mut USB_OTG_GlobalTypeDef,
                       ep: *mut USB_OTG_EPTypeDef, dma: uint8_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn USB_EP0StartXfer(USBx: *mut USB_OTG_GlobalTypeDef,
                        ep: *mut USB_OTG_EPTypeDef, dma: uint8_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn USB_WritePacket(USBx: *mut USB_OTG_GlobalTypeDef, src: *mut uint8_t,
                       ch_ep_num: uint8_t, len: uint16_t, dma: uint8_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn USB_ReadPacket(USBx: *mut USB_OTG_GlobalTypeDef, dest: *mut uint8_t,
                      len: uint16_t) -> *mut libc::c_void;
    #[no_mangle]
    fn USB_EPSetStall(USBx: *mut USB_OTG_GlobalTypeDef,
                      ep: *mut USB_OTG_EPTypeDef) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn USB_EPClearStall(USBx: *mut USB_OTG_GlobalTypeDef,
                        ep: *mut USB_OTG_EPTypeDef) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn USB_SetDevAddress(USBx: *mut USB_OTG_GlobalTypeDef, address: uint8_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn USB_DevConnect(USBx: *mut USB_OTG_GlobalTypeDef) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn USB_DevDisconnect(USBx: *mut USB_OTG_GlobalTypeDef)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn USB_StopDevice(USBx: *mut USB_OTG_GlobalTypeDef) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn USB_ActivateSetup(USBx: *mut USB_OTG_GlobalTypeDef)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn USB_EP0_OutStart(USBx: *mut USB_OTG_GlobalTypeDef, dma: uint8_t,
                        psetup: *mut uint8_t) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn USB_GetDevSpeed(USBx: *mut USB_OTG_GlobalTypeDef) -> uint8_t;
    #[no_mangle]
    fn USB_GetMode(USBx: *mut USB_OTG_GlobalTypeDef) -> uint32_t;
    #[no_mangle]
    fn USB_ReadInterrupts(USBx: *mut USB_OTG_GlobalTypeDef) -> uint32_t;
    #[no_mangle]
    fn USB_ReadDevAllOutEpInterrupt(USBx: *mut USB_OTG_GlobalTypeDef)
     -> uint32_t;
    #[no_mangle]
    fn USB_ReadDevOutEPInterrupt(USBx: *mut USB_OTG_GlobalTypeDef,
                                 epnum: uint8_t) -> uint32_t;
    #[no_mangle]
    fn USB_ReadDevAllInEpInterrupt(USBx: *mut USB_OTG_GlobalTypeDef)
     -> uint32_t;
    #[no_mangle]
    fn USB_ReadDevInEPInterrupt(USBx: *mut USB_OTG_GlobalTypeDef,
                                epnum: uint8_t) -> uint32_t;
    #[no_mangle]
    fn HAL_PCDEx_ActivateLPM(hpcd: *mut PCD_HandleTypeDef)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_PCDEx_LPM_Callback(hpcd: *mut PCD_HandleTypeDef,
                              msg: PCD_LPM_MsgTypeDef);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USB_OTG_DeviceTypeDef {
    pub DCFG: uint32_t,
    pub DCTL: uint32_t,
    pub DSTS: uint32_t,
    pub Reserved0C: uint32_t,
    pub DIEPMSK: uint32_t,
    pub DOEPMSK: uint32_t,
    pub DAINT: uint32_t,
    pub DAINTMSK: uint32_t,
    pub Reserved20: uint32_t,
    pub Reserved9: uint32_t,
    pub DVBUSDIS: uint32_t,
    pub DVBUSPULSE: uint32_t,
    pub DTHRCTL: uint32_t,
    pub DIEPEMPMSK: uint32_t,
    pub DEACHINT: uint32_t,
    pub DEACHMSK: uint32_t,
    pub Reserved40: uint32_t,
    pub DINEP1MSK: uint32_t,
    pub Reserved44: [uint32_t; 15],
    pub DOUTEP1MSK: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USB_OTG_INEndpointTypeDef {
    pub DIEPCTL: uint32_t,
    pub Reserved04: uint32_t,
    pub DIEPINT: uint32_t,
    pub Reserved0C: uint32_t,
    pub DIEPTSIZ: uint32_t,
    pub DIEPDMA: uint32_t,
    pub DTXFSTS: uint32_t,
    pub Reserved18: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USB_OTG_OUTEndpointTypeDef {
    pub DOEPCTL: uint32_t,
    pub Reserved04: uint32_t,
    pub DOEPINT: uint32_t,
    pub Reserved0C: uint32_t,
    pub DOEPTSIZ: uint32_t,
    pub DOEPDMA: uint32_t,
    pub Reserved18: [uint32_t; 2],
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
  * @file    stm32f7xx_ll_usb.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of USB Core HAL module.
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
/* * @addtogroup STM32F7xx_HAL
  * @{
  */
/* * @addtogroup USB_Core
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief  USB Mode definition  
  */
pub type USB_OTG_ModeTypeDef = libc::c_uint;
pub const USB_OTG_DRD_MODE: USB_OTG_ModeTypeDef = 2;
pub const USB_OTG_HOST_MODE: USB_OTG_ModeTypeDef = 1;
pub const USB_OTG_DEVICE_MODE: USB_OTG_ModeTypeDef = 0;
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
pub type PCD_StateTypeDef = libc::c_uint;
pub const HAL_PCD_STATE_TIMEOUT: PCD_StateTypeDef = 4;
pub const HAL_PCD_STATE_BUSY: PCD_StateTypeDef = 3;
pub const HAL_PCD_STATE_ERROR: PCD_StateTypeDef = 2;
pub const HAL_PCD_STATE_READY: PCD_StateTypeDef = 1;
pub const HAL_PCD_STATE_RESET: PCD_StateTypeDef = 0;
pub type PCD_LPM_StateTypeDef = libc::c_uint;
pub const LPM_L3: PCD_LPM_StateTypeDef = 3;
pub const LPM_L2: PCD_LPM_StateTypeDef = 2;
pub const LPM_L1: PCD_LPM_StateTypeDef = 1;
pub const LPM_L0: PCD_LPM_StateTypeDef = 0;
pub type PCD_TypeDef = USB_OTG_GlobalTypeDef;
pub type PCD_InitTypeDef = USB_OTG_CfgTypeDef;
pub type PCD_EPTypeDef = USB_OTG_EPTypeDef;
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
pub type PCD_LPM_MsgTypeDef = libc::c_uint;
/* LPM L1 sleep */
/* on */
pub const PCD_LPM_L1_ACTIVE: PCD_LPM_MsgTypeDef = 1;
pub const PCD_LPM_L0_ACTIVE: PCD_LPM_MsgTypeDef = 0;
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @defgroup PCD_Exported_Functions PCD Exported Functions
  * @{
  */
/* * @defgroup PCD_Exported_Functions_Group1 Initialization and de-initialization functions 
 *  @brief    Initialization and Configuration functions 
 *
@verbatim    
 ===============================================================================
            ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
 
@endverbatim
  * @{
  */
/* *
  * @brief  Initializes the PCD according to the specified
  *         parameters in the PCD_InitTypeDef and create the associated handle.
  * @param  hpcd: PCD handle
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_Init(mut hpcd: *mut PCD_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut i: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the PCD handle allocation */
    if hpcd.is_null() { return HAL_ERROR }
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*hpcd).State as *mut PCD_StateTypeDef,
                                HAL_PCD_STATE_BUSY);
    /* Init the low level hardware : GPIO, CLOCK, NVIC... */
    HAL_PCD_MspInit(hpcd);
    /* Disable the Interrupts */
    USB_DisableGlobalInt((*hpcd).Instance);
    /*Init the Core (common init.) */
    USB_CoreInit((*hpcd).Instance, (*hpcd).Init);
    /* Force Device Mode*/
    USB_SetCurrentMode((*hpcd).Instance, USB_OTG_DEVICE_MODE);
    /* Init endpoints structures */
    i = 0 as libc::c_int as uint32_t;
    while i < 15 as libc::c_int as libc::c_uint {
        /* Init ep structure */
        (*hpcd).IN_ep[i as usize].is_in = 1 as libc::c_int as uint8_t;
        (*hpcd).IN_ep[i as usize].num = i as uint8_t;
        (*hpcd).IN_ep[i as usize].tx_fifo_num = i as uint16_t;
        /* Control until ep is activated */
        (*hpcd).IN_ep[i as usize].type_0 = 0 as libc::c_uint as uint8_t;
        (*hpcd).IN_ep[i as usize].maxpacket = 0 as libc::c_int as uint32_t;
        (*hpcd).IN_ep[i as usize].xfer_buff = 0 as *mut uint8_t;
        (*hpcd).IN_ep[i as usize].xfer_len = 0 as libc::c_int as uint32_t;
        i = i.wrapping_add(1)
    }
    i = 0 as libc::c_int as uint32_t;
    while i < 15 as libc::c_int as libc::c_uint {
        (*hpcd).OUT_ep[i as usize].is_in = 0 as libc::c_int as uint8_t;
        (*hpcd).OUT_ep[i as usize].num = i as uint8_t;
        (*hpcd).IN_ep[i as usize].tx_fifo_num = i as uint16_t;
        /* Control until ep is activated */
        (*hpcd).OUT_ep[i as usize].type_0 = 0 as libc::c_uint as uint8_t;
        (*hpcd).OUT_ep[i as usize].maxpacket = 0 as libc::c_int as uint32_t;
        (*hpcd).OUT_ep[i as usize].xfer_buff = 0 as *mut uint8_t;
        (*hpcd).OUT_ep[i as usize].xfer_len = 0 as libc::c_int as uint32_t;
        ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).DIEPTXF[i as
                                                                         usize]
                                        as *mut uint32_t,
                                    0 as libc::c_int as uint32_t);
        i = i.wrapping_add(1)
    }
    /* Init Device */
    USB_DevInit((*hpcd).Instance, (*hpcd).Init);
    ::core::ptr::write_volatile(&mut (*hpcd).State as *mut PCD_StateTypeDef,
                                HAL_PCD_STATE_READY);
    /* Activate LPM */
    if (*hpcd).Init.lpm_enable == 1 as libc::c_int as libc::c_uint {
        HAL_PCDEx_ActivateLPM(hpcd);
    }
    /* USB_OTG_GCCFG_BCDEN */
    USB_DevDisconnect((*hpcd).Instance);
    return HAL_OK;
}
/* *
  * @brief  DeInitializes the PCD peripheral. 
  * @param  hpcd: PCD handle
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_DeInit(mut hpcd: *mut PCD_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the PCD handle allocation */
    if hpcd.is_null() { return HAL_ERROR }
    ::core::ptr::write_volatile(&mut (*hpcd).State as *mut PCD_StateTypeDef,
                                HAL_PCD_STATE_BUSY);
    /* Stop Device */
    HAL_PCD_Stop(hpcd);
    /* DeInit the low level hardware */
    HAL_PCD_MspDeInit(hpcd);
    ::core::ptr::write_volatile(&mut (*hpcd).State as *mut PCD_StateTypeDef,
                                HAL_PCD_STATE_RESET);
    return HAL_OK;
}
/* *
  * @brief  Initializes the PCD MSP.
  * @param  hpcd: PCD handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_MspInit(mut hpcd: *mut PCD_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_MspInit could be implemented in the user file
   */
}
/* *
  * @brief  DeInitializes PCD MSP.
  * @param  hpcd: PCD handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_MspDeInit(mut hpcd: *mut PCD_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_MspDeInit could be implemented in the user file
   */
}
/* *
  * @}
  */
/* * @defgroup PCD_Exported_Functions_Group2 Input and Output operation functions
 *  @brief   Data transfers functions 
 *
@verbatim   
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================  
    [..]
    This subsection provides a set of functions allowing to manage the PCD data 
    transfers.

@endverbatim
  * @{
  */
/* *
  * @brief  Start The USB OTG Device.
  * @param  hpcd: PCD handle
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_Start(mut hpcd: *mut PCD_HandleTypeDef)
 -> HAL_StatusTypeDef {
    if (*hpcd).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hpcd).Lock = HAL_LOCKED }
    USB_DevConnect((*hpcd).Instance);
    USB_EnableGlobalInt((*hpcd).Instance);
    (*hpcd).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Stop The USB OTG Device.
  * @param  hpcd: PCD handle
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_Stop(mut hpcd: *mut PCD_HandleTypeDef)
 -> HAL_StatusTypeDef {
    if (*hpcd).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hpcd).Lock = HAL_LOCKED }
    USB_DisableGlobalInt((*hpcd).Instance);
    USB_StopDevice((*hpcd).Instance);
    USB_DevDisconnect((*hpcd).Instance);
    (*hpcd).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Handle PCD interrupt request.
  * @param  hpcd: PCD handle
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_IRQHandler(mut hpcd:
                                                *mut PCD_HandleTypeDef) {
    let mut USBx: *mut USB_OTG_GlobalTypeDef = (*hpcd).Instance;
    let mut i: uint32_t = 0 as libc::c_int as uint32_t;
    let mut ep_intr: uint32_t = 0 as libc::c_int as uint32_t;
    let mut epint: uint32_t = 0 as libc::c_int as uint32_t;
    let mut epnum: uint32_t = 0 as libc::c_int as uint32_t;
    let mut fifoemptymsk: uint32_t = 0 as libc::c_int as uint32_t;
    let mut temp: uint32_t = 0 as libc::c_int as uint32_t;
    let mut ep: *mut USB_OTG_EPTypeDef = 0 as *mut USB_OTG_EPTypeDef;
    let mut hclk: uint32_t = 200000000 as libc::c_int as uint32_t;
    /* ensure that we are in device mode */
    if USB_GetMode((*hpcd).Instance) == 0 as libc::c_uint {
        /* avoid spurious interrupt */
        if USB_ReadInterrupts((*hpcd).Instance) ==
               0 as libc::c_int as libc::c_uint {
            return
        }
        if USB_ReadInterrupts((*hpcd).Instance) &
               (0x1 as libc::c_uint) << 1 as libc::c_uint ==
               (0x1 as libc::c_uint) << 1 as libc::c_uint {
            /* incorrect mode, acknowledge the interrupt */
            ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GINTSTS as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            1 as libc::c_uint)
        }
        if USB_ReadInterrupts((*hpcd).Instance) &
               (0x1 as libc::c_uint) << 19 as libc::c_uint ==
               (0x1 as libc::c_uint) << 19 as libc::c_uint {
            epnum = 0 as libc::c_int as uint32_t;
            /* Read in the device interrupt bits */
            ep_intr = USB_ReadDevAllOutEpInterrupt((*hpcd).Instance);
            while ep_intr != 0 {
                if ep_intr & 0x1 as libc::c_int as libc::c_uint != 0 {
                    epint =
                        USB_ReadDevOutEPInterrupt((*hpcd).Instance,
                                                  epnum as uint8_t);
                    if epint & (0x1 as libc::c_uint) << 0 as libc::c_uint ==
                           (0x1 as libc::c_uint) << 0 as libc::c_uint {
                        ::core::ptr::write_volatile(&mut (*((USBx as
                                                                 uint32_t).wrapping_add(0xb00
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(epnum.wrapping_mul(0x20
                                                                                                                                              as
                                                                                                                                              libc::c_uint))
                                                                as
                                                                *mut USB_OTG_OUTEndpointTypeDef)).DOEPINT
                                                        as *mut uint32_t,
                                                    (0x1 as libc::c_uint) <<
                                                        0 as libc::c_uint);
                        if (*hpcd).Init.dma_enable ==
                               1 as libc::c_int as libc::c_uint {
                            (*hpcd).OUT_ep[epnum as usize].xfer_count =
                                (*hpcd).OUT_ep[epnum as
                                                   usize].maxpacket.wrapping_sub((*((USBx
                                                                                         as
                                                                                         uint32_t).wrapping_add(0xb00
                                                                                                                    as
                                                                                                                    libc::c_uint).wrapping_add(epnum.wrapping_mul(0x20
                                                                                                                                                                      as
                                                                                                                                                                      libc::c_uint))
                                                                                        as
                                                                                        *mut USB_OTG_OUTEndpointTypeDef)).DOEPTSIZ
                                                                                     &
                                                                                     (0x7ffff
                                                                                          as
                                                                                          libc::c_uint)
                                                                                         <<
                                                                                         0
                                                                                             as
                                                                                             libc::c_uint);
                            (*hpcd).OUT_ep[epnum as usize].xfer_buff =
                                (*hpcd).OUT_ep[epnum as
                                                   usize].xfer_buff.offset((*hpcd).OUT_ep[epnum
                                                                                              as
                                                                                              usize].maxpacket
                                                                               as
                                                                               isize)
                        }
                        HAL_PCD_DataOutStageCallback(hpcd, epnum as uint8_t);
                        if (*hpcd).Init.dma_enable ==
                               1 as libc::c_int as libc::c_uint {
                            if epnum == 0 as libc::c_int as libc::c_uint &&
                                   (*hpcd).OUT_ep[epnum as usize].xfer_len ==
                                       0 as libc::c_int as libc::c_uint {
                                /* this is ZLP, so prepare EP0 for next setup */
                                USB_EP0_OutStart((*hpcd).Instance,
                                                 1 as libc::c_int as uint8_t,
                                                 (*hpcd).Setup.as_mut_ptr() as
                                                     *mut uint8_t);
                            }
                        }
                    }
                    if epint & (0x1 as libc::c_uint) << 3 as libc::c_uint ==
                           (0x1 as libc::c_uint) << 3 as libc::c_uint {
                        /* Inform the upper layer that a setup packet is available */
                        HAL_PCD_SetupStageCallback(hpcd);
                        ::core::ptr::write_volatile(&mut (*((USBx as
                                                                 uint32_t).wrapping_add(0xb00
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(epnum.wrapping_mul(0x20
                                                                                                                                              as
                                                                                                                                              libc::c_uint))
                                                                as
                                                                *mut USB_OTG_OUTEndpointTypeDef)).DOEPINT
                                                        as *mut uint32_t,
                                                    (0x1 as libc::c_uint) <<
                                                        3 as libc::c_uint)
                    }
                    if epint & (0x1 as libc::c_uint) << 4 as libc::c_uint ==
                           (0x1 as libc::c_uint) << 4 as libc::c_uint {
                        ::core::ptr::write_volatile(&mut (*((USBx as
                                                                 uint32_t).wrapping_add(0xb00
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(epnum.wrapping_mul(0x20
                                                                                                                                              as
                                                                                                                                              libc::c_uint))
                                                                as
                                                                *mut USB_OTG_OUTEndpointTypeDef)).DOEPINT
                                                        as *mut uint32_t,
                                                    (0x1 as libc::c_uint) <<
                                                        4 as libc::c_uint)
                    }
                    /* Clear Status Phase Received interrupt */
                    if epint & (0x1 as libc::c_uint) << 5 as libc::c_uint ==
                           (0x1 as libc::c_uint) << 5 as libc::c_uint {
                        ::core::ptr::write_volatile(&mut (*((USBx as
                                                                 uint32_t).wrapping_add(0xb00
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(epnum.wrapping_mul(0x20
                                                                                                                                              as
                                                                                                                                              libc::c_uint))
                                                                as
                                                                *mut USB_OTG_OUTEndpointTypeDef)).DOEPINT
                                                        as *mut uint32_t,
                                                    (0x1 as libc::c_uint) <<
                                                        5 as libc::c_uint)
                    }
                }
                epnum = epnum.wrapping_add(1);
                ep_intr >>= 1 as libc::c_int
            }
        }
        if USB_ReadInterrupts((*hpcd).Instance) &
               (0x1 as libc::c_uint) << 18 as libc::c_uint ==
               (0x1 as libc::c_uint) << 18 as libc::c_uint {
            /* Read in the device interrupt bits */
            ep_intr = USB_ReadDevAllInEpInterrupt((*hpcd).Instance);
            epnum = 0 as libc::c_int as uint32_t;
            while ep_intr != 0 {
                if ep_intr & 0x1 as libc::c_int as libc::c_uint != 0 {
                    /* In ITR */
                    epint =
                        USB_ReadDevInEPInterrupt((*hpcd).Instance,
                                                 epnum as uint8_t);
                    if epint & (0x1 as libc::c_uint) << 0 as libc::c_uint ==
                           (0x1 as libc::c_uint) << 0 as libc::c_uint {
                        fifoemptymsk =
                            ((0x1 as libc::c_int) << epnum) as uint32_t;
                        let ref mut fresh0 =
                            (*((USBx as
                                    uint32_t).wrapping_add(0x800 as
                                                               libc::c_uint)
                                   as *mut USB_OTG_DeviceTypeDef)).DIEPEMPMSK;
                        ::core::ptr::write_volatile(fresh0,
                                                    (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                                as
                                                                                                *const uint32_t)
                                                         as libc::c_uint &
                                                         !fifoemptymsk) as
                                                        uint32_t as uint32_t);
                        ::core::ptr::write_volatile(&mut (*((USBx as
                                                                 uint32_t).wrapping_add(0x900
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(epnum.wrapping_mul(0x20
                                                                                                                                              as
                                                                                                                                              libc::c_uint))
                                                                as
                                                                *mut USB_OTG_INEndpointTypeDef)).DIEPINT
                                                        as *mut uint32_t,
                                                    (0x1 as libc::c_uint) <<
                                                        0 as libc::c_uint);
                        if (*hpcd).Init.dma_enable ==
                               1 as libc::c_int as libc::c_uint {
                            (*hpcd).IN_ep[epnum as usize].xfer_buff =
                                (*hpcd).IN_ep[epnum as
                                                  usize].xfer_buff.offset((*hpcd).IN_ep[epnum
                                                                                            as
                                                                                            usize].maxpacket
                                                                              as
                                                                              isize)
                        }
                        HAL_PCD_DataInStageCallback(hpcd, epnum as uint8_t);
                        if (*hpcd).Init.dma_enable ==
                               1 as libc::c_int as libc::c_uint {
                            /* this is ZLP, so prepare EP0 for next setup */
                            if epnum == 0 as libc::c_int as libc::c_uint &&
                                   (*hpcd).IN_ep[epnum as usize].xfer_len ==
                                       0 as libc::c_int as libc::c_uint {
                                /* prepare to rx more setup packets */
                                USB_EP0_OutStart((*hpcd).Instance,
                                                 1 as libc::c_int as uint8_t,
                                                 (*hpcd).Setup.as_mut_ptr() as
                                                     *mut uint8_t);
                            }
                        }
                    }
                    if epint & (0x1 as libc::c_uint) << 3 as libc::c_uint ==
                           (0x1 as libc::c_uint) << 3 as libc::c_uint {
                        ::core::ptr::write_volatile(&mut (*((USBx as
                                                                 uint32_t).wrapping_add(0x900
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(epnum.wrapping_mul(0x20
                                                                                                                                              as
                                                                                                                                              libc::c_uint))
                                                                as
                                                                *mut USB_OTG_INEndpointTypeDef)).DIEPINT
                                                        as *mut uint32_t,
                                                    (0x1 as libc::c_uint) <<
                                                        3 as libc::c_uint)
                    }
                    if epint & (0x1 as libc::c_uint) << 4 as libc::c_uint ==
                           (0x1 as libc::c_uint) << 4 as libc::c_uint {
                        ::core::ptr::write_volatile(&mut (*((USBx as
                                                                 uint32_t).wrapping_add(0x900
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(epnum.wrapping_mul(0x20
                                                                                                                                              as
                                                                                                                                              libc::c_uint))
                                                                as
                                                                *mut USB_OTG_INEndpointTypeDef)).DIEPINT
                                                        as *mut uint32_t,
                                                    (0x1 as libc::c_uint) <<
                                                        4 as libc::c_uint)
                    }
                    if epint & (0x1 as libc::c_uint) << 6 as libc::c_uint ==
                           (0x1 as libc::c_uint) << 6 as libc::c_uint {
                        ::core::ptr::write_volatile(&mut (*((USBx as
                                                                 uint32_t).wrapping_add(0x900
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(epnum.wrapping_mul(0x20
                                                                                                                                              as
                                                                                                                                              libc::c_uint))
                                                                as
                                                                *mut USB_OTG_INEndpointTypeDef)).DIEPINT
                                                        as *mut uint32_t,
                                                    (0x1 as libc::c_uint) <<
                                                        6 as libc::c_uint)
                    }
                    if epint & (0x1 as libc::c_uint) << 1 as libc::c_uint ==
                           (0x1 as libc::c_uint) << 1 as libc::c_uint {
                        ::core::ptr::write_volatile(&mut (*((USBx as
                                                                 uint32_t).wrapping_add(0x900
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(epnum.wrapping_mul(0x20
                                                                                                                                              as
                                                                                                                                              libc::c_uint))
                                                                as
                                                                *mut USB_OTG_INEndpointTypeDef)).DIEPINT
                                                        as *mut uint32_t,
                                                    (0x1 as libc::c_uint) <<
                                                        1 as libc::c_uint)
                    }
                    if epint & (0x1 as libc::c_uint) << 7 as libc::c_uint ==
                           (0x1 as libc::c_uint) << 7 as libc::c_uint {
                        PCD_WriteEmptyTxFifo(hpcd, epnum);
                    }
                }
                epnum = epnum.wrapping_add(1);
                ep_intr >>= 1 as libc::c_int
            }
        }
        /* Handle Resume Interrupt */
        if USB_ReadInterrupts((*hpcd).Instance) &
               (0x1 as libc::c_uint) << 31 as libc::c_uint ==
               (0x1 as libc::c_uint) << 31 as libc::c_uint {
            /* Clear the Remote Wake-up Signaling */
            let ref mut fresh1 =
                (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                       *mut USB_OTG_DeviceTypeDef)).DCTL;
            ::core::ptr::write_volatile(fresh1,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            if (*hpcd).LPM_State as libc::c_uint ==
                   LPM_L1 as libc::c_int as libc::c_uint {
                (*hpcd).LPM_State = LPM_L0;
                HAL_PCDEx_LPM_Callback(hpcd, PCD_LPM_L0_ACTIVE);
            } else { HAL_PCD_ResumeCallback(hpcd); }
            ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GINTSTS as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            31 as libc::c_uint)
        }
        /* Handle Suspend Interrupt */
        if USB_ReadInterrupts((*hpcd).Instance) &
               (0x1 as libc::c_uint) << 11 as libc::c_uint ==
               (0x1 as libc::c_uint) << 11 as libc::c_uint {
            if (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                      *mut USB_OTG_DeviceTypeDef)).DSTS &
                   (0x1 as libc::c_uint) << 0 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 0 as libc::c_uint {
                HAL_PCD_SuspendCallback(hpcd);
            }
            ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GINTSTS as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            11 as libc::c_uint)
        }
        /* Handle LPM Interrupt */
        if USB_ReadInterrupts((*hpcd).Instance) &
               (0x1 as libc::c_uint) << 27 as libc::c_uint ==
               (0x1 as libc::c_uint) << 27 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GINTSTS as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            27 as libc::c_uint);
            if (*hpcd).LPM_State as libc::c_uint ==
                   LPM_L0 as libc::c_int as libc::c_uint {
                (*hpcd).LPM_State = LPM_L1;
                (*hpcd).BESL =
                    ((*(*hpcd).Instance).GLPMCFG &
                         (0xf as libc::c_uint) << 2 as libc::c_uint) >>
                        2 as libc::c_int;
                HAL_PCDEx_LPM_Callback(hpcd, PCD_LPM_L1_ACTIVE);
            } else { HAL_PCD_SuspendCallback(hpcd); }
        }
        /* Handle Reset Interrupt */
        if USB_ReadInterrupts((*hpcd).Instance) &
               (0x1 as libc::c_uint) << 12 as libc::c_uint ==
               (0x1 as libc::c_uint) << 12 as libc::c_uint {
            let ref mut fresh2 =
                (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                       *mut USB_OTG_DeviceTypeDef)).DCTL;
            ::core::ptr::write_volatile(fresh2,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            USB_FlushTxFifo((*hpcd).Instance,
                            0x10 as libc::c_int as uint32_t);
            i = 0 as libc::c_int as uint32_t;
            while i < (*hpcd).Init.dev_endpoints {
                ::core::ptr::write_volatile(&mut (*((USBx as
                                                         uint32_t).wrapping_add(0x900
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(i.wrapping_mul(0x20
                                                                                                                                  as
                                                                                                                                  libc::c_uint))
                                                        as
                                                        *mut USB_OTG_INEndpointTypeDef)).DIEPINT
                                                as *mut uint32_t,
                                            0xff as libc::c_int as uint32_t);
                ::core::ptr::write_volatile(&mut (*((USBx as
                                                         uint32_t).wrapping_add(0xb00
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(i.wrapping_mul(0x20
                                                                                                                                  as
                                                                                                                                  libc::c_uint))
                                                        as
                                                        *mut USB_OTG_OUTEndpointTypeDef)).DOEPINT
                                                as *mut uint32_t,
                                            0xff as libc::c_int as uint32_t);
                i = i.wrapping_add(1)
            }
            ::core::ptr::write_volatile(&mut (*((USBx as
                                                     uint32_t).wrapping_add(0x800
                                                                                as
                                                                                libc::c_uint)
                                                    as
                                                    *mut USB_OTG_DeviceTypeDef)).DAINT
                                            as *mut uint32_t,
                                        0xffffffff as libc::c_uint);
            let ref mut fresh3 =
                (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                       *mut USB_OTG_DeviceTypeDef)).DAINTMSK;
            ::core::ptr::write_volatile(fresh3,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x10001 as libc::c_int as
                                                 libc::c_uint) as uint32_t as
                                            uint32_t);
            if (*hpcd).Init.use_dedicated_ep1 != 0 {
                let ref mut fresh4 =
                    (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint)
                           as *mut USB_OTG_DeviceTypeDef)).DOUTEP1MSK;
                ::core::ptr::write_volatile(fresh4,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 ((0x1 as libc::c_uint) <<
                                                      3 as libc::c_uint |
                                                      (0x1 as libc::c_uint) <<
                                                          0 as libc::c_uint |
                                                      (0x1 as libc::c_uint) <<
                                                          1 as libc::c_uint))
                                                as uint32_t as uint32_t);
                let ref mut fresh5 =
                    (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint)
                           as *mut USB_OTG_DeviceTypeDef)).DINEP1MSK;
                ::core::ptr::write_volatile(fresh5,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 ((0x1 as libc::c_uint) <<
                                                      3 as libc::c_uint |
                                                      (0x1 as libc::c_uint) <<
                                                          0 as libc::c_uint |
                                                      (0x1 as libc::c_uint) <<
                                                          1 as libc::c_uint))
                                                as uint32_t as uint32_t)
            } else {
                let ref mut fresh6 =
                    (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint)
                           as *mut USB_OTG_DeviceTypeDef)).DOEPMSK;
                ::core::ptr::write_volatile(fresh6,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 ((0x1 as libc::c_uint) <<
                                                      3 as libc::c_uint |
                                                      (0x1 as libc::c_uint) <<
                                                          0 as libc::c_uint |
                                                      (0x1 as libc::c_uint) <<
                                                          1 as libc::c_uint |
                                                      (0x1 as libc::c_uint) <<
                                                          5 as libc::c_uint))
                                                as uint32_t as uint32_t);
                let ref mut fresh7 =
                    (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint)
                           as *mut USB_OTG_DeviceTypeDef)).DIEPMSK;
                ::core::ptr::write_volatile(fresh7,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 ((0x1 as libc::c_uint) <<
                                                      3 as libc::c_uint |
                                                      (0x1 as libc::c_uint) <<
                                                          0 as libc::c_uint |
                                                      (0x1 as libc::c_uint) <<
                                                          1 as libc::c_uint))
                                                as uint32_t as uint32_t)
            }
            /* Set Default Address to 0 */
            let ref mut fresh8 =
                (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                       *mut USB_OTG_DeviceTypeDef)).DCFG;
            ::core::ptr::write_volatile(fresh8,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x7f as libc::c_uint) <<
                                                   4 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            /* setup EP0 to receive SETUP packets */
            USB_EP0_OutStart((*hpcd).Instance,
                             (*hpcd).Init.dma_enable as uint8_t,
                             (*hpcd).Setup.as_mut_ptr() as *mut uint8_t);
            ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GINTSTS as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            12 as libc::c_uint)
        }
        /* Handle Enumeration done Interrupt */
        if USB_ReadInterrupts((*hpcd).Instance) &
               (0x1 as libc::c_uint) << 13 as libc::c_uint ==
               (0x1 as libc::c_uint) << 13 as libc::c_uint {
            USB_ActivateSetup((*hpcd).Instance);
            ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GUSBCFG as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hpcd).Instance).GUSBCFG
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0xf as libc::c_uint) <<
                                                   10 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            if USB_GetDevSpeed((*hpcd).Instance) as libc::c_uint ==
                   0 as libc::c_uint {
                (*hpcd).Init.speed = 0 as libc::c_uint;
                (*hpcd).Init.ep0_mps = 512 as libc::c_uint;
                ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GUSBCFG
                                                as *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hpcd).Instance).GUSBCFG
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (9 as libc::c_uint) <<
                                                     10 as libc::c_int &
                                                     (0xf as libc::c_uint) <<
                                                         10 as libc::c_uint)
                                                as uint32_t as uint32_t)
            } else {
                (*hpcd).Init.speed = 3 as libc::c_uint;
                (*hpcd).Init.ep0_mps = 64 as libc::c_uint;
                /* The USBTRD is configured according to the tables below, depending on AHB frequency 
        used by application. In the low AHB frequency range it is used to stretch enough the USB response 
        time to IN tokens, the USB turnaround time, so to compensate for the longer AHB read access 
        latency to the Data FIFO */
                /* Get hclk frequency value */
                hclk = HAL_RCC_GetHCLKFreq();
                if hclk >= 14200000 as libc::c_int as libc::c_uint &&
                       hclk < 15000000 as libc::c_int as libc::c_uint {
                    /* hclk Clock Range between 14.2-15 MHz */
                    ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GUSBCFG
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hpcd).Instance).GUSBCFG
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint |
                                                     ((0xf as libc::c_int) <<
                                                          10 as libc::c_int)
                                                         as libc::c_uint &
                                                         (0xf as libc::c_uint)
                                                             <<
                                                             10 as
                                                                 libc::c_uint)
                                                    as uint32_t as uint32_t)
                } else if hclk >= 15000000 as libc::c_int as libc::c_uint &&
                              hclk < 16000000 as libc::c_int as libc::c_uint {
                    /* hclk Clock Range between 15-16 MHz */
                    ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GUSBCFG
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hpcd).Instance).GUSBCFG
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint |
                                                     ((0xe as libc::c_int) <<
                                                          10 as libc::c_int)
                                                         as libc::c_uint &
                                                         (0xf as libc::c_uint)
                                                             <<
                                                             10 as
                                                                 libc::c_uint)
                                                    as uint32_t as uint32_t)
                } else if hclk >= 16000000 as libc::c_int as libc::c_uint &&
                              hclk < 17200000 as libc::c_int as libc::c_uint {
                    /* hclk Clock Range between 16-17.2 MHz */
                    ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GUSBCFG
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hpcd).Instance).GUSBCFG
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint |
                                                     ((0xd as libc::c_int) <<
                                                          10 as libc::c_int)
                                                         as libc::c_uint &
                                                         (0xf as libc::c_uint)
                                                             <<
                                                             10 as
                                                                 libc::c_uint)
                                                    as uint32_t as uint32_t)
                } else if hclk >= 17200000 as libc::c_int as libc::c_uint &&
                              hclk < 18500000 as libc::c_int as libc::c_uint {
                    /* hclk Clock Range between 17.2-18.5 MHz */
                    ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GUSBCFG
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hpcd).Instance).GUSBCFG
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint |
                                                     ((0xc as libc::c_int) <<
                                                          10 as libc::c_int)
                                                         as libc::c_uint &
                                                         (0xf as libc::c_uint)
                                                             <<
                                                             10 as
                                                                 libc::c_uint)
                                                    as uint32_t as uint32_t)
                } else if hclk >= 18500000 as libc::c_int as libc::c_uint &&
                              hclk < 20000000 as libc::c_int as libc::c_uint {
                    /* hclk Clock Range between 18.5-20 MHz */
                    ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GUSBCFG
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hpcd).Instance).GUSBCFG
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint |
                                                     ((0xb as libc::c_int) <<
                                                          10 as libc::c_int)
                                                         as libc::c_uint &
                                                         (0xf as libc::c_uint)
                                                             <<
                                                             10 as
                                                                 libc::c_uint)
                                                    as uint32_t as uint32_t)
                } else if hclk >= 20000000 as libc::c_int as libc::c_uint &&
                              hclk < 21800000 as libc::c_int as libc::c_uint {
                    /* hclk Clock Range between 20-21.8 MHz */
                    ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GUSBCFG
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hpcd).Instance).GUSBCFG
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint |
                                                     ((0xa as libc::c_int) <<
                                                          10 as libc::c_int)
                                                         as libc::c_uint &
                                                         (0xf as libc::c_uint)
                                                             <<
                                                             10 as
                                                                 libc::c_uint)
                                                    as uint32_t as uint32_t)
                } else if hclk >= 21800000 as libc::c_int as libc::c_uint &&
                              hclk < 24000000 as libc::c_int as libc::c_uint {
                    /* hclk Clock Range between 21.8-24 MHz */
                    ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GUSBCFG
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hpcd).Instance).GUSBCFG
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint |
                                                     ((0x9 as libc::c_int) <<
                                                          10 as libc::c_int)
                                                         as libc::c_uint &
                                                         (0xf as libc::c_uint)
                                                             <<
                                                             10 as
                                                                 libc::c_uint)
                                                    as uint32_t as uint32_t)
                } else if hclk >= 24000000 as libc::c_int as libc::c_uint &&
                              hclk < 27700000 as libc::c_int as libc::c_uint {
                    /* hclk Clock Range between 24-27.7 MHz */
                    ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GUSBCFG
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hpcd).Instance).GUSBCFG
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint |
                                                     ((0x8 as libc::c_int) <<
                                                          10 as libc::c_int)
                                                         as libc::c_uint &
                                                         (0xf as libc::c_uint)
                                                             <<
                                                             10 as
                                                                 libc::c_uint)
                                                    as uint32_t as uint32_t)
                } else if hclk >= 27700000 as libc::c_int as libc::c_uint &&
                              hclk < 32000000 as libc::c_int as libc::c_uint {
                    /* hclk Clock Range between 27.7-32 MHz */
                    ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GUSBCFG
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hpcd).Instance).GUSBCFG
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint |
                                                     ((0x7 as libc::c_int) <<
                                                          10 as libc::c_int)
                                                         as libc::c_uint &
                                                         (0xf as libc::c_uint)
                                                             <<
                                                             10 as
                                                                 libc::c_uint)
                                                    as uint32_t as uint32_t)
                } else {
                    /* if(hclk >= 32000000) */
                    /* hclk Clock Range between 32-200 MHz */
                    ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GUSBCFG
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hpcd).Instance).GUSBCFG
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint |
                                                     ((0x6 as libc::c_int) <<
                                                          10 as libc::c_int)
                                                         as libc::c_uint &
                                                         (0xf as libc::c_uint)
                                                             <<
                                                             10 as
                                                                 libc::c_uint)
                                                    as uint32_t as uint32_t)
                }
            }
            HAL_PCD_ResetCallback(hpcd);
            ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GINTSTS as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            13 as libc::c_uint)
        }
        /* Handle RxQLevel Interrupt */
        if USB_ReadInterrupts((*hpcd).Instance) &
               (0x1 as libc::c_uint) << 4 as libc::c_uint ==
               (0x1 as libc::c_uint) << 4 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GINTMSK as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hpcd).Instance).GINTMSK
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   4 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            temp = (*USBx).GRXSTSP;
            ep =
                &mut *(*hpcd).OUT_ep.as_mut_ptr().offset((temp &
                                                              (0xf as
                                                                   libc::c_uint)
                                                                  <<
                                                                  0 as
                                                                      libc::c_uint)
                                                             as isize) as
                    *mut PCD_EPTypeDef;
            if (temp & (0xf as libc::c_uint) << 17 as libc::c_uint) >>
                   17 as libc::c_int == 2 as libc::c_uint {
                if temp & (0x7ff as libc::c_uint) << 4 as libc::c_uint !=
                       0 as libc::c_int as libc::c_uint {
                    USB_ReadPacket(USBx, (*ep).xfer_buff,
                                   ((temp &
                                         (0x7ff as libc::c_uint) <<
                                             4 as libc::c_uint) >>
                                        4 as libc::c_int) as uint16_t);
                    (*ep).xfer_buff =
                        (*ep).xfer_buff.offset(((temp &
                                                     (0x7ff as libc::c_uint)
                                                         << 4 as libc::c_uint)
                                                    >> 4 as libc::c_int) as
                                                   isize);
                    (*ep).xfer_count =
                        ((*ep).xfer_count as
                             libc::c_uint).wrapping_add((temp &
                                                             (0x7ff as
                                                                  libc::c_uint)
                                                                 <<
                                                                 4 as
                                                                     libc::c_uint)
                                                            >>
                                                            4 as libc::c_int)
                            as uint32_t as uint32_t
                }
            } else if (temp & (0xf as libc::c_uint) << 17 as libc::c_uint) >>
                          17 as libc::c_int == 6 as libc::c_uint {
                USB_ReadPacket(USBx,
                               (*hpcd).Setup.as_mut_ptr() as *mut uint8_t,
                               8 as libc::c_int as uint16_t);
                (*ep).xfer_count =
                    ((*ep).xfer_count as
                         libc::c_uint).wrapping_add((temp &
                                                         (0x7ff as
                                                              libc::c_uint) <<
                                                             4 as
                                                                 libc::c_uint)
                                                        >> 4 as libc::c_int)
                        as uint32_t as uint32_t
            }
            ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GINTMSK as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hpcd).Instance).GINTMSK
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 4 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        /* Handle SOF Interrupt */
        if USB_ReadInterrupts((*hpcd).Instance) &
               (0x1 as libc::c_uint) << 3 as libc::c_uint ==
               (0x1 as libc::c_uint) << 3 as libc::c_uint {
            HAL_PCD_SOFCallback(hpcd);
            ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GINTSTS as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            3 as libc::c_uint)
        }
        /* Handle Incomplete ISO IN Interrupt */
        if USB_ReadInterrupts((*hpcd).Instance) &
               (0x1 as libc::c_uint) << 20 as libc::c_uint ==
               (0x1 as libc::c_uint) << 20 as libc::c_uint {
            HAL_PCD_ISOINIncompleteCallback(hpcd, epnum as uint8_t);
            ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GINTSTS as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            20 as libc::c_uint)
        }
        /* Handle Incomplete ISO OUT Interrupt */
        if USB_ReadInterrupts((*hpcd).Instance) &
               (0x1 as libc::c_uint) << 21 as libc::c_uint ==
               (0x1 as libc::c_uint) << 21 as libc::c_uint {
            HAL_PCD_ISOOUTIncompleteCallback(hpcd, epnum as uint8_t);
            ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GINTSTS as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            21 as libc::c_uint)
        }
        /* Handle Connection event Interrupt */
        if USB_ReadInterrupts((*hpcd).Instance) &
               (0x1 as libc::c_uint) << 30 as libc::c_uint ==
               (0x1 as libc::c_uint) << 30 as libc::c_uint {
            HAL_PCD_ConnectCallback(hpcd);
            ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GINTSTS as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            30 as libc::c_uint)
        }
        /* Handle Disconnection event Interrupt */
        if USB_ReadInterrupts((*hpcd).Instance) &
               (0x1 as libc::c_uint) << 2 as libc::c_uint ==
               (0x1 as libc::c_uint) << 2 as libc::c_uint {
            temp = (*(*hpcd).Instance).GOTGINT;
            if temp & (0x1 as libc::c_uint) << 2 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 2 as libc::c_uint {
                HAL_PCD_DisconnectCallback(hpcd);
            }
            ::core::ptr::write_volatile(&mut (*(*hpcd).Instance).GOTGINT as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hpcd).Instance).GOTGINT
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint | temp) as
                                            uint32_t as uint32_t)
        }
    };
}
/* *
  * @brief  Data OUT stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: endpoint number  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_DataOutStageCallback(mut hpcd:
                                                          *mut PCD_HandleTypeDef,
                                                      mut epnum: uint8_t) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_DataOutStageCallback could be implemented in the user file
   */
}
/* *
  * @brief  Data IN stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: endpoint number  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_DataInStageCallback(mut hpcd:
                                                         *mut PCD_HandleTypeDef,
                                                     mut epnum: uint8_t) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_DataInStageCallback could be implemented in the user file
   */
}
/* *
  * @brief  Setup stage callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_SetupStageCallback(mut hpcd:
                                                        *mut PCD_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_SetupStageCallback could be implemented in the user file
   */
}
/* *
  * @brief  USB Start Of Frame callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_SOFCallback(mut hpcd:
                                                 *mut PCD_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_SOFCallback could be implemented in the user file
   */
}
/* *
  * @brief  USB Reset callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_ResetCallback(mut hpcd:
                                                   *mut PCD_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_ResetCallback could be implemented in the user file
   */
}
/* *
  * @brief  Suspend event callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_SuspendCallback(mut hpcd:
                                                     *mut PCD_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_SuspendCallback could be implemented in the user file
   */
}
/* *
  * @brief  Resume event callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_ResumeCallback(mut hpcd:
                                                    *mut PCD_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_ResumeCallback could be implemented in the user file
   */
}
/* *
  * @brief  Incomplete ISO OUT callback.
  * @param  hpcd: PCD handle
  * @param  epnum: endpoint number
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_ISOOUTIncompleteCallback(mut hpcd:
                                                              *mut PCD_HandleTypeDef,
                                                          mut epnum:
                                                              uint8_t) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_ISOOUTIncompleteCallback could be implemented in the user file
   */
}
/* *
  * @brief  Incomplete ISO IN  callback.
  * @param  hpcd: PCD handle
  * @param  epnum: endpoint number  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_ISOINIncompleteCallback(mut hpcd:
                                                             *mut PCD_HandleTypeDef,
                                                         mut epnum: uint8_t) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_ISOINIncompleteCallback could be implemented in the user file
   */
}
/* *
  * @brief  Connection event callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_ConnectCallback(mut hpcd:
                                                     *mut PCD_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_ConnectCallback could be implemented in the user file
   */
}
/* *
  * @brief  Disconnection event callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_DisconnectCallback(mut hpcd:
                                                        *mut PCD_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PCD_DisconnectCallback could be implemented in the user file
   */
}
/* *
  * @}
  */
/* * @defgroup PCD_Exported_Functions_Group3 Peripheral Control functions
 *  @brief   management functions 
 *
@verbatim   
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================  
    [..]
    This subsection provides a set of functions allowing to control the PCD data 
    transfers.

@endverbatim
  * @{
  */
/* *
  * @brief  Connect the USB device.
  * @param  hpcd: PCD handle
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_DevConnect(mut hpcd: *mut PCD_HandleTypeDef)
 -> HAL_StatusTypeDef {
    if (*hpcd).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hpcd).Lock = HAL_LOCKED }
    USB_DevConnect((*hpcd).Instance);
    (*hpcd).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Disconnect the USB device.
  * @param  hpcd: PCD handle
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_DevDisconnect(mut hpcd:
                                                   *mut PCD_HandleTypeDef)
 -> HAL_StatusTypeDef {
    if (*hpcd).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hpcd).Lock = HAL_LOCKED }
    USB_DevDisconnect((*hpcd).Instance);
    (*hpcd).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Set the USB Device address. 
  * @param  hpcd: PCD handle
  * @param  address: new device address
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_SetAddress(mut hpcd: *mut PCD_HandleTypeDef,
                                            mut address: uint8_t)
 -> HAL_StatusTypeDef {
    if (*hpcd).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hpcd).Lock = HAL_LOCKED }
    USB_SetDevAddress((*hpcd).Instance, address);
    (*hpcd).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Open and configure an endpoint.
  * @param  hpcd: PCD handle
  * @param  ep_addr: endpoint address
  * @param  ep_mps: endpoint max packet size
  * @param  ep_type: endpoint type   
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_EP_Open(mut hpcd: *mut PCD_HandleTypeDef,
                                         mut ep_addr: uint8_t,
                                         mut ep_mps: uint16_t,
                                         mut ep_type: uint8_t)
 -> HAL_StatusTypeDef {
    let mut ret: HAL_StatusTypeDef = HAL_OK;
    let mut ep: *mut USB_OTG_EPTypeDef = 0 as *mut USB_OTG_EPTypeDef;
    if ep_addr as libc::c_int & 0x80 as libc::c_int == 0x80 as libc::c_int {
        ep =
            &mut *(*hpcd).IN_ep.as_mut_ptr().offset((ep_addr as libc::c_int &
                                                         0x7f as libc::c_int)
                                                        as isize) as
                *mut PCD_EPTypeDef
    } else {
        ep =
            &mut *(*hpcd).OUT_ep.as_mut_ptr().offset((ep_addr as libc::c_int &
                                                          0x7f as libc::c_int)
                                                         as isize) as
                *mut PCD_EPTypeDef
    }
    (*ep).num = (ep_addr as libc::c_int & 0x7f as libc::c_int) as uint8_t;
    (*ep).is_in =
        (0x80 as libc::c_int & ep_addr as libc::c_int != 0 as libc::c_int) as
            libc::c_int as uint8_t;
    (*ep).maxpacket = ep_mps as uint32_t;
    (*ep).type_0 = ep_type;
    if (*ep).is_in != 0 {
        /* Assign a Tx FIFO */
        (*ep).tx_fifo_num = (*ep).num as uint16_t
    }
    /* Set initial data PID. */
    if ep_type as libc::c_uint == 2 as libc::c_uint {
        (*ep).data_pid_start = 0 as libc::c_int as uint8_t
    }
    if (*hpcd).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hpcd).Lock = HAL_LOCKED }
    USB_ActivateEndpoint((*hpcd).Instance, ep);
    (*hpcd).Lock = HAL_UNLOCKED;
    return ret;
}
/* *
  * @brief  Deactivate an endpoint.
  * @param  hpcd: PCD handle
  * @param  ep_addr: endpoint address
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_EP_Close(mut hpcd: *mut PCD_HandleTypeDef,
                                          mut ep_addr: uint8_t)
 -> HAL_StatusTypeDef {
    let mut ep: *mut USB_OTG_EPTypeDef = 0 as *mut USB_OTG_EPTypeDef;
    if ep_addr as libc::c_int & 0x80 as libc::c_int == 0x80 as libc::c_int {
        ep =
            &mut *(*hpcd).IN_ep.as_mut_ptr().offset((ep_addr as libc::c_int &
                                                         0x7f as libc::c_int)
                                                        as isize) as
                *mut PCD_EPTypeDef
    } else {
        ep =
            &mut *(*hpcd).OUT_ep.as_mut_ptr().offset((ep_addr as libc::c_int &
                                                          0x7f as libc::c_int)
                                                         as isize) as
                *mut PCD_EPTypeDef
    }
    (*ep).num = (ep_addr as libc::c_int & 0x7f as libc::c_int) as uint8_t;
    (*ep).is_in =
        (0x80 as libc::c_int & ep_addr as libc::c_int != 0 as libc::c_int) as
            libc::c_int as uint8_t;
    if (*hpcd).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hpcd).Lock = HAL_LOCKED }
    USB_DeactivateEndpoint((*hpcd).Instance, ep);
    (*hpcd).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Receive an amount of data.  
  * @param  hpcd: PCD handle
  * @param  ep_addr: endpoint address
  * @param  pBuf: pointer to the reception buffer   
  * @param  len: amount of data to be received
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_EP_Receive(mut hpcd: *mut PCD_HandleTypeDef,
                                            mut ep_addr: uint8_t,
                                            mut pBuf: *mut uint8_t,
                                            mut len: uint32_t)
 -> HAL_StatusTypeDef {
    let mut ep: *mut USB_OTG_EPTypeDef = 0 as *mut USB_OTG_EPTypeDef;
    ep =
        &mut *(*hpcd).OUT_ep.as_mut_ptr().offset((ep_addr as libc::c_int &
                                                      0x7f as libc::c_int) as
                                                     isize) as
            *mut PCD_EPTypeDef;
    /*setup and start the Xfer */
    (*ep).xfer_buff = pBuf;
    (*ep).xfer_len = len;
    (*ep).xfer_count = 0 as libc::c_int as uint32_t;
    (*ep).is_in = 0 as libc::c_int as uint8_t;
    (*ep).num = (ep_addr as libc::c_int & 0x7f as libc::c_int) as uint8_t;
    if (*hpcd).Init.dma_enable == 1 as libc::c_int as libc::c_uint {
        (*ep).dma_addr = pBuf as uint32_t
    }
    if ep_addr as libc::c_int & 0x7f as libc::c_int == 0 as libc::c_int {
        USB_EP0StartXfer((*hpcd).Instance, ep,
                         (*hpcd).Init.dma_enable as uint8_t);
    } else {
        USB_EPStartXfer((*hpcd).Instance, ep,
                        (*hpcd).Init.dma_enable as uint8_t);
    }
    return HAL_OK;
}
/* *
  * @brief  Get Received Data Size.
  * @param  hpcd: PCD handle
  * @param  ep_addr: endpoint address
  * @retval Data Size
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_EP_GetRxCount(mut hpcd:
                                                   *mut PCD_HandleTypeDef,
                                               mut ep_addr: uint8_t)
 -> uint16_t {
    return (*hpcd).OUT_ep[(ep_addr as libc::c_int & 0xf as libc::c_int) as
                              usize].xfer_count as uint16_t;
}
/* *
  * @brief  Send an amount of data.  
  * @param  hpcd: PCD handle
  * @param  ep_addr: endpoint address
  * @param  pBuf: pointer to the transmission buffer   
  * @param  len: amount of data to be sent
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_EP_Transmit(mut hpcd: *mut PCD_HandleTypeDef,
                                             mut ep_addr: uint8_t,
                                             mut pBuf: *mut uint8_t,
                                             mut len: uint32_t)
 -> HAL_StatusTypeDef {
    let mut ep: *mut USB_OTG_EPTypeDef = 0 as *mut USB_OTG_EPTypeDef;
    ep =
        &mut *(*hpcd).IN_ep.as_mut_ptr().offset((ep_addr as libc::c_int &
                                                     0x7f as libc::c_int) as
                                                    isize) as
            *mut PCD_EPTypeDef;
    /*setup and start the Xfer */
    (*ep).xfer_buff = pBuf;
    (*ep).xfer_len = len;
    (*ep).xfer_count = 0 as libc::c_int as uint32_t;
    (*ep).is_in = 1 as libc::c_int as uint8_t;
    (*ep).num = (ep_addr as libc::c_int & 0x7f as libc::c_int) as uint8_t;
    if (*hpcd).Init.dma_enable == 1 as libc::c_int as libc::c_uint {
        (*ep).dma_addr = pBuf as uint32_t
    }
    if ep_addr as libc::c_int & 0x7f as libc::c_int == 0 as libc::c_int {
        USB_EP0StartXfer((*hpcd).Instance, ep,
                         (*hpcd).Init.dma_enable as uint8_t);
    } else {
        USB_EPStartXfer((*hpcd).Instance, ep,
                        (*hpcd).Init.dma_enable as uint8_t);
    }
    return HAL_OK;
}
/* *
  * @brief  Set a STALL condition over an endpoint.
  * @param  hpcd: PCD handle
  * @param  ep_addr: endpoint address
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_EP_SetStall(mut hpcd: *mut PCD_HandleTypeDef,
                                             mut ep_addr: uint8_t)
 -> HAL_StatusTypeDef {
    let mut ep: *mut USB_OTG_EPTypeDef = 0 as *mut USB_OTG_EPTypeDef;
    if 0x80 as libc::c_int & ep_addr as libc::c_int == 0x80 as libc::c_int {
        ep =
            &mut *(*hpcd).IN_ep.as_mut_ptr().offset((ep_addr as libc::c_int &
                                                         0x7f as libc::c_int)
                                                        as isize) as
                *mut PCD_EPTypeDef
    } else {
        ep =
            &mut *(*hpcd).OUT_ep.as_mut_ptr().offset(ep_addr as isize) as
                *mut PCD_EPTypeDef
    }
    (*ep).is_stall = 1 as libc::c_int as uint8_t;
    (*ep).num = (ep_addr as libc::c_int & 0x7f as libc::c_int) as uint8_t;
    (*ep).is_in =
        (ep_addr as libc::c_int & 0x80 as libc::c_int == 0x80 as libc::c_int)
            as libc::c_int as uint8_t;
    if (*hpcd).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hpcd).Lock = HAL_LOCKED }
    USB_EPSetStall((*hpcd).Instance, ep);
    if ep_addr as libc::c_int & 0x7f as libc::c_int == 0 as libc::c_int {
        USB_EP0_OutStart((*hpcd).Instance, (*hpcd).Init.dma_enable as uint8_t,
                         (*hpcd).Setup.as_mut_ptr() as *mut uint8_t);
    }
    (*hpcd).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Clear a STALL condition over in an endpoint.
  * @param  hpcd: PCD handle
  * @param  ep_addr: endpoint address
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_EP_ClrStall(mut hpcd: *mut PCD_HandleTypeDef,
                                             mut ep_addr: uint8_t)
 -> HAL_StatusTypeDef {
    let mut ep: *mut USB_OTG_EPTypeDef = 0 as *mut USB_OTG_EPTypeDef;
    if 0x80 as libc::c_int & ep_addr as libc::c_int == 0x80 as libc::c_int {
        ep =
            &mut *(*hpcd).IN_ep.as_mut_ptr().offset((ep_addr as libc::c_int &
                                                         0x7f as libc::c_int)
                                                        as isize) as
                *mut PCD_EPTypeDef
    } else {
        ep =
            &mut *(*hpcd).OUT_ep.as_mut_ptr().offset(ep_addr as isize) as
                *mut PCD_EPTypeDef
    }
    (*ep).is_stall = 0 as libc::c_int as uint8_t;
    (*ep).num = (ep_addr as libc::c_int & 0x7f as libc::c_int) as uint8_t;
    (*ep).is_in =
        (ep_addr as libc::c_int & 0x80 as libc::c_int == 0x80 as libc::c_int)
            as libc::c_int as uint8_t;
    if (*hpcd).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hpcd).Lock = HAL_LOCKED }
    USB_EPClearStall((*hpcd).Instance, ep);
    (*hpcd).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Flush an endpoint.
  * @param  hpcd: PCD handle
  * @param  ep_addr: endpoint address
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_EP_Flush(mut hpcd: *mut PCD_HandleTypeDef,
                                          mut ep_addr: uint8_t)
 -> HAL_StatusTypeDef {
    if (*hpcd).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hpcd).Lock = HAL_LOCKED }
    if ep_addr as libc::c_int & 0x80 as libc::c_int == 0x80 as libc::c_int {
        USB_FlushTxFifo((*hpcd).Instance,
                        (ep_addr as libc::c_int & 0x7f as libc::c_int) as
                            uint32_t);
    } else { USB_FlushRxFifo((*hpcd).Instance); }
    (*hpcd).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Activate remote wakeup signalling.
  * @param  hpcd: PCD handle
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_ActivateRemoteWakeup(mut hpcd:
                                                          *mut PCD_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut USBx: *mut USB_OTG_GlobalTypeDef = (*hpcd).Instance;
    if (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
              *mut USB_OTG_DeviceTypeDef)).DSTS &
           (0x1 as libc::c_uint) << 0 as libc::c_uint ==
           (0x1 as libc::c_uint) << 0 as libc::c_uint {
        /* Activate Remote wakeup signaling */
        let ref mut fresh9 =
            (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                   *mut USB_OTG_DeviceTypeDef)).DCTL;
        ::core::ptr::write_volatile(fresh9,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    return HAL_OK;
}
/* *
  * @brief  De-activate remote wakeup signalling.
  * @param  hpcd: PCD handle
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_DeActivateRemoteWakeup(mut hpcd:
                                                            *mut PCD_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut USBx: *mut USB_OTG_GlobalTypeDef = (*hpcd).Instance;
    /* De-activate Remote wakeup signaling */
    let ref mut fresh10 =
        (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
               *mut USB_OTG_DeviceTypeDef)).DCTL;
    ::core::ptr::write_volatile(fresh10,
                                (::core::ptr::read_volatile::<uint32_t>(fresh10
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    return HAL_OK;
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
/* Device LPM suspend state */
/* on */
/* LPM L1 sleep */
/* suspend */
/* off */
/* * 
  * @brief  PCD Handle Structure definition  
  */
/* !< Register base address              */
/* !< PCD required parameters            */
/* !< IN endpoint parameters             */
/* !< OUT endpoint parameters            */
/* !< PCD peripheral status              */
/* !< PCD communication state            */
/* !< Setup packet buffer                */
/* !< LPM State                          */
/* !< Enable or disable the Link Power Management .                                  
                                        This parameter can be set to ENABLE or DISABLE */
/* !< Enable or disable Battery charging.                                  
                                        This parameter can be set to ENABLE or DISABLE                      */
/* !< Pointer to upper stack Handler */
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
/* *
  * @}
  */
/* I/O operation functions  ***************************************************/
/* Non-Blocking mode: Interrupt */
/* * @addtogroup PCD_Exported_Functions_Group2 Input and Output operation functions
  * @{
  */
/* *
  * @}
  */
/* Peripheral Control functions  **********************************************/
/* * @addtogroup PCD_Exported_Functions_Group3 Peripheral Control functions
  * @{
  */
/* *
  * @}
  */
/* Peripheral State functions  ************************************************/
/* * @addtogroup PCD_Exported_Functions_Group4 Peripheral State functions
  * @{
  */
/* *
  * @}
  */
/* * @defgroup PCD_Exported_Functions_Group4 Peripheral State functions 
 *  @brief   Peripheral State functions 
 *
@verbatim   
 ===============================================================================
                      ##### Peripheral State functions #####
 ===============================================================================  
    [..]
    This subsection permits to get in run-time the status of the peripheral 
    and the data flow.

@endverbatim
  * @{
  */
/* *
  * @brief  Return the PCD handle state.
  * @param  hpcd: PCD handle
  * @retval HAL state
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_PCD_GetState(mut hpcd: *mut PCD_HandleTypeDef)
 -> PCD_StateTypeDef {
    return (*hpcd).State;
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_pcd.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   PCD HAL module driver.
  *          This file provides firmware functions to manage the following 
  *          functionalities of the USB Peripheral Controller:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions 
  *           + Peripheral State functions
  *         
  @verbatim
  ==============================================================================
                    ##### How to use this driver #####
  ==============================================================================
    [..]
      The PCD HAL driver can be used as follows:

     (#) Declare a PCD_HandleTypeDef handle structure, for example:
         PCD_HandleTypeDef  hpcd;
        
     (#) Fill parameters of Init structure in HCD handle
  
     (#) Call HAL_PCD_Init() API to initialize the PCD peripheral (Core, Device core, ...) 

     (#) Initialize the PCD low level resources through the HAL_PCD_MspInit() API:
         (##) Enable the PCD/USB Low Level interface clock using 
              (+++) __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
              (+++) __HAL_RCC_USB_OTG_HS_CLK_ENABLE(); (For High Speed Mode)
           
         (##) Initialize the related GPIO clocks
         (##) Configure PCD pin-out
         (##) Configure PCD NVIC interrupt
    
     (#)Associate the Upper USB device stack to the HAL PCD Driver:
         (##) hpcd.pData = pdev;

     (#)Enable PCD transmission and reception:
         (##) HAL_PCD_Start();

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
/* * @defgroup PCD PCD
  * @brief PCD HAL module driver
  * @{
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* * @defgroup PCD_Private_Macros PCD Private Macros
  * @{
  */
/* *
  * @}
  */
/* Private functions prototypes ----------------------------------------------*/
/* * @defgroup PCD_Private_Functions PCD Private Functions
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Private functions ---------------------------------------------------------*/
/* * @addtogroup PCD_Private_Functions
  * @{
  */
/* *
  * @brief  Check FIFO for the next packet to be loaded.
  * @param  hpcd: PCD handle
  * @param  epnum : endpoint number   
  * @retval HAL status
  */
unsafe extern "C" fn PCD_WriteEmptyTxFifo(mut hpcd: *mut PCD_HandleTypeDef,
                                          mut epnum: uint32_t)
 -> HAL_StatusTypeDef {
    let mut USBx: *mut USB_OTG_GlobalTypeDef = (*hpcd).Instance;
    let mut ep: *mut USB_OTG_EPTypeDef = 0 as *mut USB_OTG_EPTypeDef;
    let mut len: int32_t = 0 as libc::c_int;
    let mut len32b: uint32_t = 0;
    let mut fifoemptymsk: uint32_t = 0 as libc::c_int as uint32_t;
    ep =
        &mut *(*hpcd).IN_ep.as_mut_ptr().offset(epnum as isize) as
            *mut PCD_EPTypeDef;
    len = (*ep).xfer_len.wrapping_sub((*ep).xfer_count) as int32_t;
    if len > (*ep).maxpacket as int32_t { len = (*ep).maxpacket as int32_t }
    len32b = ((len + 3 as libc::c_int) / 4 as libc::c_int) as uint32_t;
    while (*((USBx as
                  uint32_t).wrapping_add(0x900 as
                                             libc::c_uint).wrapping_add(epnum.wrapping_mul(0x20
                                                                                               as
                                                                                               libc::c_uint))
                 as *mut USB_OTG_INEndpointTypeDef)).DTXFSTS &
              (0xffff as libc::c_uint) << 0 as libc::c_uint > len32b &&
              (*ep).xfer_count < (*ep).xfer_len &&
              (*ep).xfer_len != 0 as libc::c_int as libc::c_uint {
        /* Write the FIFO */
        len = (*ep).xfer_len.wrapping_sub((*ep).xfer_count) as int32_t;
        if len > (*ep).maxpacket as int32_t {
            len = (*ep).maxpacket as int32_t
        }
        len32b = ((len + 3 as libc::c_int) / 4 as libc::c_int) as uint32_t;
        USB_WritePacket(USBx, (*ep).xfer_buff, epnum as uint8_t,
                        len as uint16_t, (*hpcd).Init.dma_enable as uint8_t);
        (*ep).xfer_buff = (*ep).xfer_buff.offset(len as isize);
        (*ep).xfer_count =
            ((*ep).xfer_count as
                 libc::c_uint).wrapping_add(len as libc::c_uint) as uint32_t
                as uint32_t
    }
    if len <= 0 as libc::c_int {
        fifoemptymsk = ((0x1 as libc::c_int) << epnum) as uint32_t;
        let ref mut fresh11 =
            (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                   *mut USB_OTG_DeviceTypeDef)).DIEPEMPMSK;
        ::core::ptr::write_volatile(fresh11,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh11
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !fifoemptymsk) as
                                        uint32_t as uint32_t)
    }
    return HAL_OK;
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
