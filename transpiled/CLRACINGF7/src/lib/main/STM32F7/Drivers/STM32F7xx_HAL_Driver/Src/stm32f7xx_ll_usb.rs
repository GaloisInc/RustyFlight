use ::libc;
extern "C" {
    /* *
  ******************************************************************************
  * @file    stm32f7xx_hal.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   This file contains all the functions prototypes for the HAL 
  *          module driver.
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
    /* * @addtogroup HAL
  * @{
  */
    /* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* * @defgroup HAL_Exported_Constants HAL Exported Constants
  * @{
  */
    /* * @defgroup SYSCFG_BootMode Boot Mode
  * @{
  */
    /* *
  * @}
  */
    /* *
  * @}
  */
    /* Exported macro ------------------------------------------------------------*/
/* * @defgroup HAL_Exported_Macros HAL Exported Macros
  * @{
  */
    /* * @brief  Freeze/Unfreeze Peripherals in Debug mode 
  */
    /* * @brief  FMC (NOR/RAM) mapped at 0x60000000 and SDRAM mapped at 0xC0000000
  */
    /* * @brief  FMC/SDRAM  mapped at 0x60000000 (NOR/RAM) mapped at 0xC0000000
  */
    /* *
  * @brief  Return the memory boot mapping as configured by user.
  * @retval The boot mode as configured by user. The returned value can be one
  *         of the following values:
  *           @arg @ref SYSCFG_MEM_BOOT_ADD0
  *           @arg @ref SYSCFG_MEM_BOOT_ADD1
  */
    /* STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
    /* *
  * @}
  */
    /* Exported functions --------------------------------------------------------*/
/* * @addtogroup HAL_Exported_Functions
  * @{
  */
/* * @addtogroup HAL_Exported_Functions_Group1
  * @{
  */
/* Initialization and de-initialization functions  ******************************/
    /* *
  * @}
  */
    /* * @addtogroup HAL_Exported_Functions_Group2
  * @{
  */ 
/* Peripheral Control functions  ************************************************/
    #[no_mangle]
    fn HAL_Delay(Delay: uint32_t);
}
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
/* *
  * @brief USB_OTG_device_Registers
  */
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
/* *
  * @brief USB_OTG_IN_Endpoint-Specific_Register
  */
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
/* *
  * @brief USB_OTG_OUT_Endpoint-Specific_Registers
  */
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
  * @brief USB_OTG_Host_Mode_Register_Structures
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USB_OTG_HostTypeDef {
    pub HCFG: uint32_t,
    pub HFIR: uint32_t,
    pub HFNUM: uint32_t,
    pub Reserved40C: uint32_t,
    pub HPTXSTS: uint32_t,
    pub HAINT: uint32_t,
    pub HAINTMSK: uint32_t,
}
/* *
  * @brief USB_OTG_Host_Channel_Specific_Registers
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USB_OTG_HostChannelTypeDef {
    pub HCCHAR: uint32_t,
    pub HCSPLT: uint32_t,
    pub HCINT: uint32_t,
    pub HCINTMSK: uint32_t,
    pub HCTSIZ: uint32_t,
    pub HCDMA: uint32_t,
    pub Reserved: [uint32_t; 2],
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
pub type USB_OTG_ModeTypeDef = libc::c_uint;
pub const USB_OTG_DRD_MODE: USB_OTG_ModeTypeDef = 2;
pub const USB_OTG_HOST_MODE: USB_OTG_ModeTypeDef = 1;
pub const USB_OTG_DEVICE_MODE: USB_OTG_ModeTypeDef = 0;
pub type USB_OTG_URBStateTypeDef = libc::c_uint;
pub const URB_STALL: USB_OTG_URBStateTypeDef = 5;
pub const URB_ERROR: USB_OTG_URBStateTypeDef = 4;
pub const URB_NYET: USB_OTG_URBStateTypeDef = 3;
pub const URB_NOTREADY: USB_OTG_URBStateTypeDef = 2;
pub const URB_DONE: USB_OTG_URBStateTypeDef = 1;
pub const URB_IDLE: USB_OTG_URBStateTypeDef = 0;
pub type USB_OTG_HCStateTypeDef = libc::c_uint;
pub const HC_DATATGLERR: USB_OTG_HCStateTypeDef = 8;
pub const HC_BBLERR: USB_OTG_HCStateTypeDef = 7;
pub const HC_XACTERR: USB_OTG_HCStateTypeDef = 6;
pub const HC_STALL: USB_OTG_HCStateTypeDef = 5;
pub const HC_NYET: USB_OTG_HCStateTypeDef = 4;
pub const HC_NAK: USB_OTG_HCStateTypeDef = 3;
pub const HC_HALTED: USB_OTG_HCStateTypeDef = 2;
pub const HC_XFRC: USB_OTG_HCStateTypeDef = 1;
pub const HC_IDLE: USB_OTG_HCStateTypeDef = 0;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USB_OTG_HCTypeDef {
    pub dev_addr: uint8_t,
    pub ch_num: uint8_t,
    pub ep_num: uint8_t,
    pub ep_is_in: uint8_t,
    pub speed: uint8_t,
    pub do_ping: uint8_t,
    pub process_ping: uint8_t,
    pub ep_type: uint8_t,
    pub max_packet: uint16_t,
    pub data_pid: uint8_t,
    pub xfer_buff: *mut uint8_t,
    pub xfer_len: uint32_t,
    pub xfer_count: uint32_t,
    pub toggle_in: uint8_t,
    pub toggle_out: uint8_t,
    pub dma_addr: uint32_t,
    pub ErrCnt: uint32_t,
    pub urb_state: USB_OTG_URBStateTypeDef,
    pub state: USB_OTG_HCStateTypeDef,
}
/* Exported functions --------------------------------------------------------*/
/* * @defgroup LL_USB_Exported_Functions USB Low Layer Exported Functions
  * @{
  */
/* * @defgroup LL_USB_Group1 Initialization/de-initialization functions 
 *  @brief    Initialization and Configuration functions 
 *
@verbatim    
 ===============================================================================
              ##### Initialization/de-initialization functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
 
@endverbatim
  * @{
  */
/* *
  * @brief  Initializes the USB Core
  * @param  USBx: USB Instance
  * @param  cfg : pointer to a USB_OTG_CfgTypeDef structure that contains
  *         the configuration information for the specified USBx peripheral.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_CoreInit(mut USBx: *mut USB_OTG_GlobalTypeDef,
                                      mut cfg: USB_OTG_CfgTypeDef)
 -> HAL_StatusTypeDef {
    if cfg.phy_itface == 1 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*USBx).GCCFG as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GCCFG
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               16 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Init The ULPI Interface */
        ::core::ptr::write_volatile(&mut (*USBx).GUSBCFG as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GUSBCFG
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               22 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   17 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   6 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Select vbus source */
        ::core::ptr::write_volatile(&mut (*USBx).GUSBCFG as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GUSBCFG
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               20 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   21 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        if cfg.use_external_vbus == 1 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*USBx).GUSBCFG as *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GUSBCFG
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 20 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        /* Reset after a PHY select  */
        USB_CoreReset(USBx);
    } else {
        /* FS interface (embedded Phy) */
        /* Select FS Embedded PHY */
        ::core::ptr::write_volatile(&mut (*USBx).GUSBCFG as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GUSBCFG
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             6 as libc::c_uint) as uint32_t as
                                        uint32_t);
        USB_CoreReset(USBx);
        ::core::ptr::write_volatile(&mut (*USBx).GCCFG as *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        16 as libc::c_uint)
    }
    if cfg.dma_enable == ENABLE as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*USBx).GAHBCFG as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GAHBCFG
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x3 as libc::c_uint) <<
                                             1 as libc::c_uint) as uint32_t as
                                        uint32_t);
        ::core::ptr::write_volatile(&mut (*USBx).GAHBCFG as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GAHBCFG
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             5 as libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    return HAL_OK;
}
/* Reset after a PHY select and set Host mode */
/* Deactivate the power down*/
/* *
  * @brief  USB_EnableGlobalInt
  *         Enables the controller's Global Int in the AHB Config reg
  * @param  USBx : Selected device
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_EnableGlobalInt(mut USBx:
                                                 *mut USB_OTG_GlobalTypeDef)
 -> HAL_StatusTypeDef {
    ::core::ptr::write_volatile(&mut (*USBx).GAHBCFG as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GAHBCFG
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    return HAL_OK;
}
/* *
  * @brief  USB_DisableGlobalInt
  *         Disable the controller's Global Int in the AHB Config reg
  * @param  USBx : Selected device
  * @retval HAL status
*/
#[no_mangle]
pub unsafe extern "C" fn USB_DisableGlobalInt(mut USBx:
                                                  *mut USB_OTG_GlobalTypeDef)
 -> HAL_StatusTypeDef {
    ::core::ptr::write_volatile(&mut (*USBx).GAHBCFG as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GAHBCFG
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    return HAL_OK;
}
/* *
  * @brief  USB_SetCurrentMode : Set functional mode
  * @param  USBx : Selected device
  * @param  mode :  current core mode
  *          This parameter can be one of these values:
  *            @arg USB_OTG_DEVICE_MODE: Peripheral mode
  *            @arg USB_OTG_HOST_MODE: Host mode
  *            @arg USB_OTG_DRD_MODE: Dual Role Device mode  
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_SetCurrentMode(mut USBx:
                                                *mut USB_OTG_GlobalTypeDef,
                                            mut mode: USB_OTG_ModeTypeDef)
 -> HAL_StatusTypeDef {
    ::core::ptr::write_volatile(&mut (*USBx).GUSBCFG as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GUSBCFG
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           29 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               30 as libc::c_uint)) as
                                    uint32_t as uint32_t);
    if mode as libc::c_uint ==
           USB_OTG_HOST_MODE as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*USBx).GUSBCFG as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GUSBCFG
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             29 as libc::c_uint) as uint32_t
                                        as uint32_t)
    } else if mode as libc::c_uint ==
                  USB_OTG_DEVICE_MODE as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*USBx).GUSBCFG as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GUSBCFG
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             30 as libc::c_uint) as uint32_t
                                        as uint32_t)
    }
    HAL_Delay(50 as libc::c_int as uint32_t);
    return HAL_OK;
}
/* *
  * @brief  USB_DevInit : Initializes the USB_OTG controller registers 
  *         for device mode
  * @param  USBx : Selected device
  * @param  cfg  : pointer to a USB_OTG_CfgTypeDef structure that contains
  *         the configuration information for the specified USBx peripheral.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_DevInit(mut USBx: *mut USB_OTG_GlobalTypeDef,
                                     mut cfg: USB_OTG_CfgTypeDef)
 -> HAL_StatusTypeDef {
    let mut i: uint32_t = 0 as libc::c_int as uint32_t;
    /*Activate VBUS Sensing B */
    ::core::ptr::write_volatile(&mut (*USBx).GCCFG as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GCCFG
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         21 as libc::c_uint) as uint32_t as
                                    uint32_t);
    if cfg.vbus_sensing_enable == 0 as libc::c_int as libc::c_uint {
        /* Deactivate VBUS Sensing B */
        ::core::ptr::write_volatile(&mut (*USBx).GCCFG as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GCCFG
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               21 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* B-peripheral session valid override enable*/
        ::core::ptr::write_volatile(&mut (*USBx).GOTGCTL as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GOTGCTL
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             6 as libc::c_uint) as uint32_t as
                                        uint32_t);
        ::core::ptr::write_volatile(&mut (*USBx).GOTGCTL as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GOTGCTL
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             7 as libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    /* Restart the Phy Clock */
    ::core::ptr::write_volatile((USBx as
                                     uint32_t).wrapping_add(0xe00 as
                                                                libc::c_uint)
                                    as *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    /* Device mode configuration */
    let ref mut fresh0 =
        (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
               *mut USB_OTG_DeviceTypeDef)).DCFG;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0 as libc::c_uint) as
                                    uint32_t as uint32_t);
    if cfg.phy_itface == 1 as libc::c_uint {
        if cfg.speed == 0 as libc::c_uint {
            /* Set High speed phy */
            USB_SetDevSpeed(USBx, 0 as libc::c_uint as uint8_t);
        } else {
            /* set High speed phy in Full speed mode */
            USB_SetDevSpeed(USBx, 1 as libc::c_uint as uint8_t);
        }
    } else if cfg.phy_itface == 3 as libc::c_uint {
        if cfg.speed == 0 as libc::c_uint {
            /* Set High speed phy */
            USB_SetDevSpeed(USBx, 0 as libc::c_uint as uint8_t);
        } else {
            /* set High speed phy in Full speed mode */
            USB_SetDevSpeed(USBx, 1 as libc::c_uint as uint8_t);
        }
    } else {
        /* Set Full speed phy */
        USB_SetDevSpeed(USBx, 3 as libc::c_uint as uint8_t);
    }
    /* Flush the FIFOs */
    USB_FlushTxFifo(USBx, 0x10 as libc::c_int as uint32_t); /* all Tx FIFOs */
    USB_FlushRxFifo(USBx);
    /* Clear all pending Device Interrupts */
    ::core::ptr::write_volatile(&mut (*((USBx as
                                             uint32_t).wrapping_add(0x800 as
                                                                        libc::c_uint)
                                            as
                                            *mut USB_OTG_DeviceTypeDef)).DIEPMSK
                                    as *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((USBx as
                                             uint32_t).wrapping_add(0x800 as
                                                                        libc::c_uint)
                                            as
                                            *mut USB_OTG_DeviceTypeDef)).DOEPMSK
                                    as *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((USBx as
                                             uint32_t).wrapping_add(0x800 as
                                                                        libc::c_uint)
                                            as
                                            *mut USB_OTG_DeviceTypeDef)).DAINT
                                    as *mut uint32_t,
                                0xffffffff as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*((USBx as
                                             uint32_t).wrapping_add(0x800 as
                                                                        libc::c_uint)
                                            as
                                            *mut USB_OTG_DeviceTypeDef)).DAINTMSK
                                    as *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    i = 0 as libc::c_int as uint32_t;
    while i < cfg.dev_endpoints {
        if (*((USBx as
                   uint32_t).wrapping_add(0x900 as
                                              libc::c_uint).wrapping_add(i.wrapping_mul(0x20
                                                                                            as
                                                                                            libc::c_uint))
                  as *mut USB_OTG_INEndpointTypeDef)).DIEPCTL &
               (0x1 as libc::c_uint) << 31 as libc::c_uint ==
               (0x1 as libc::c_uint) << 31 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*((USBx as
                                                     uint32_t).wrapping_add(0x900
                                                                                as
                                                                                libc::c_uint).wrapping_add(i.wrapping_mul(0x20
                                                                                                                              as
                                                                                                                              libc::c_uint))
                                                    as
                                                    *mut USB_OTG_INEndpointTypeDef)).DIEPCTL
                                            as *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            30 as libc::c_uint |
                                            (0x1 as libc::c_uint) <<
                                                27 as libc::c_uint)
        } else {
            ::core::ptr::write_volatile(&mut (*((USBx as
                                                     uint32_t).wrapping_add(0x900
                                                                                as
                                                                                libc::c_uint).wrapping_add(i.wrapping_mul(0x20
                                                                                                                              as
                                                                                                                              libc::c_uint))
                                                    as
                                                    *mut USB_OTG_INEndpointTypeDef)).DIEPCTL
                                            as *mut uint32_t,
                                        0 as libc::c_int as uint32_t)
        }
        ::core::ptr::write_volatile(&mut (*((USBx as
                                                 uint32_t).wrapping_add(0x900
                                                                            as
                                                                            libc::c_uint).wrapping_add(i.wrapping_mul(0x20
                                                                                                                          as
                                                                                                                          libc::c_uint))
                                                as
                                                *mut USB_OTG_INEndpointTypeDef)).DIEPTSIZ
                                        as *mut uint32_t,
                                    0 as libc::c_int as uint32_t);
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
        i = i.wrapping_add(1)
    }
    i = 0 as libc::c_int as uint32_t;
    while i < cfg.dev_endpoints {
        if (*((USBx as
                   uint32_t).wrapping_add(0xb00 as
                                              libc::c_uint).wrapping_add(i.wrapping_mul(0x20
                                                                                            as
                                                                                            libc::c_uint))
                  as *mut USB_OTG_OUTEndpointTypeDef)).DOEPCTL &
               (0x1 as libc::c_uint) << 31 as libc::c_uint ==
               (0x1 as libc::c_uint) << 31 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*((USBx as
                                                     uint32_t).wrapping_add(0xb00
                                                                                as
                                                                                libc::c_uint).wrapping_add(i.wrapping_mul(0x20
                                                                                                                              as
                                                                                                                              libc::c_uint))
                                                    as
                                                    *mut USB_OTG_OUTEndpointTypeDef)).DOEPCTL
                                            as *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            30 as libc::c_uint |
                                            (0x1 as libc::c_uint) <<
                                                27 as libc::c_uint)
        } else {
            ::core::ptr::write_volatile(&mut (*((USBx as
                                                     uint32_t).wrapping_add(0xb00
                                                                                as
                                                                                libc::c_uint).wrapping_add(i.wrapping_mul(0x20
                                                                                                                              as
                                                                                                                              libc::c_uint))
                                                    as
                                                    *mut USB_OTG_OUTEndpointTypeDef)).DOEPCTL
                                            as *mut uint32_t,
                                        0 as libc::c_int as uint32_t)
        }
        ::core::ptr::write_volatile(&mut (*((USBx as
                                                 uint32_t).wrapping_add(0xb00
                                                                            as
                                                                            libc::c_uint).wrapping_add(i.wrapping_mul(0x20
                                                                                                                          as
                                                                                                                          libc::c_uint))
                                                as
                                                *mut USB_OTG_OUTEndpointTypeDef)).DOEPTSIZ
                                        as *mut uint32_t,
                                    0 as libc::c_int as uint32_t);
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
    let ref mut fresh1 =
        (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
               *mut USB_OTG_DeviceTypeDef)).DIEPMSK;
    ::core::ptr::write_volatile(fresh1,
                                (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           8 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    if cfg.dma_enable == 1 as libc::c_int as libc::c_uint {
        /*Set threshold parameters */
        ::core::ptr::write_volatile(&mut (*((USBx as
                                                 uint32_t).wrapping_add(0x800
                                                                            as
                                                                            libc::c_uint)
                                                as
                                                *mut USB_OTG_DeviceTypeDef)).DTHRCTL
                                        as *mut uint32_t,
                                    (0x40 as libc::c_uint) <<
                                        2 as libc::c_uint |
                                        (0x40 as libc::c_uint) <<
                                            17 as libc::c_uint);
        let ref mut fresh2 =
            (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                   *mut USB_OTG_DeviceTypeDef)).DTHRCTL;
        ::core::ptr::write_volatile(fresh2,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              16 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  1 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  0 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        i =
            (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                   *mut USB_OTG_DeviceTypeDef)).DTHRCTL
    }
    /* Disable all interrupts. */
    ::core::ptr::write_volatile(&mut (*USBx).GINTMSK as *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    /* Clear any pending interrupts */
    ::core::ptr::write_volatile(&mut (*USBx).GINTSTS as *mut uint32_t,
                                0xbfffffff as libc::c_uint);
    /* Enable the common interrupts */
    if cfg.dma_enable == DISABLE as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*USBx).GINTMSK as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GINTMSK
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             4 as libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    /* Enable interrupts matching to the Device mode ONLY */
    ::core::ptr::write_volatile(&mut (*USBx).GINTMSK as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GINTMSK
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((0x1 as libc::c_uint) <<
                                          11 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              12 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              13 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              18 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              19 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              20 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              21 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              31 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    if cfg.Sof_enable != 0 {
        ::core::ptr::write_volatile(&mut (*USBx).GINTMSK as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GINTMSK
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             3 as libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    if cfg.vbus_sensing_enable == ENABLE as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*USBx).GINTMSK as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GINTMSK
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              30 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  2 as libc::c_uint)) as
                                        uint32_t as uint32_t)
    }
    return HAL_OK;
}
/* *
  * @brief  USB_OTG_FlushTxFifo : Flush a Tx FIFO
  * @param  USBx : Selected device
  * @param  num : FIFO number
  *         This parameter can be a value from 1 to 15
            15 means Flush all Tx FIFOs
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_FlushTxFifo(mut USBx: *mut USB_OTG_GlobalTypeDef,
                                         mut num: uint32_t)
 -> HAL_StatusTypeDef {
    let mut count: uint32_t = 0 as libc::c_int as uint32_t;
    ::core::ptr::write_volatile(&mut (*USBx).GRSTCTL as *mut uint32_t,
                                (0x1 as libc::c_uint) << 5 as libc::c_uint |
                                    num << 6 as libc::c_int);
    loop  {
        count = count.wrapping_add(1);
        if count > 200000 as libc::c_int as libc::c_uint {
            return HAL_TIMEOUT
        }
        if !((*USBx).GRSTCTL & (0x1 as libc::c_uint) << 5 as libc::c_uint ==
                 (0x1 as libc::c_uint) << 5 as libc::c_uint) {
            break ;
        }
    }
    return HAL_OK;
}
/* *
  * @brief  USB_FlushRxFifo : Flush Rx FIFO
  * @param  USBx : Selected device
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_FlushRxFifo(mut USBx: *mut USB_OTG_GlobalTypeDef)
 -> HAL_StatusTypeDef {
    let mut count: uint32_t = 0 as libc::c_int as uint32_t;
    ::core::ptr::write_volatile(&mut (*USBx).GRSTCTL as *mut uint32_t,
                                (0x1 as libc::c_uint) << 4 as libc::c_uint);
    loop  {
        count = count.wrapping_add(1);
        if count > 200000 as libc::c_int as libc::c_uint {
            return HAL_TIMEOUT
        }
        if !((*USBx).GRSTCTL & (0x1 as libc::c_uint) << 4 as libc::c_uint ==
                 (0x1 as libc::c_uint) << 4 as libc::c_uint) {
            break ;
        }
    }
    return HAL_OK;
}
/* *
  * @brief  USB_SetDevSpeed :Initializes the DevSpd field of DCFG register 
  *         depending the PHY type and the enumeration speed of the device.
  * @param  USBx : Selected device
  * @param  speed : device speed
  *          This parameter can be one of these values:
  *            @arg USB_OTG_SPEED_HIGH: High speed mode
  *            @arg USB_OTG_SPEED_HIGH_IN_FULL: High speed core in Full Speed mode
  *            @arg USB_OTG_SPEED_FULL: Full speed mode
  *            @arg USB_OTG_SPEED_LOW: Low speed mode
  * @retval  Hal status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_SetDevSpeed(mut USBx: *mut USB_OTG_GlobalTypeDef,
                                         mut speed: uint8_t)
 -> HAL_StatusTypeDef {
    let ref mut fresh3 =
        (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
               *mut USB_OTG_DeviceTypeDef)).DCFG;
    ::core::ptr::write_volatile(fresh3,
                                (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | speed as libc::c_uint)
                                    as uint32_t as uint32_t);
    return HAL_OK;
}
/* *
  * @brief  USB_GetDevSpeed :Return the  Dev Speed 
  * @param  USBx : Selected device
  * @retval speed : device speed
  *          This parameter can be one of these values:
  *            @arg USB_OTG_SPEED_HIGH: High speed mode
  *            @arg USB_OTG_SPEED_FULL: Full speed mode
  *            @arg USB_OTG_SPEED_LOW: Low speed mode
  */
#[no_mangle]
pub unsafe extern "C" fn USB_GetDevSpeed(mut USBx: *mut USB_OTG_GlobalTypeDef)
 -> uint8_t {
    let mut speed: uint8_t = 0 as libc::c_int as uint8_t;
    if (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
              *mut USB_OTG_DeviceTypeDef)).DSTS &
           (0x3 as libc::c_uint) << 1 as libc::c_uint ==
           ((0 as libc::c_int) << 1 as libc::c_int) as libc::c_uint {
        speed = 0 as libc::c_uint as uint8_t
    } else if (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                     *mut USB_OTG_DeviceTypeDef)).DSTS &
                  (0x3 as libc::c_uint) << 1 as libc::c_uint ==
                  ((1 as libc::c_int) << 1 as libc::c_int) as libc::c_uint ||
                  (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                         *mut USB_OTG_DeviceTypeDef)).DSTS &
                      (0x3 as libc::c_uint) << 1 as libc::c_uint ==
                      ((3 as libc::c_int) << 1 as libc::c_int) as libc::c_uint
     {
        speed = 3 as libc::c_uint as uint8_t
    } else if (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                     *mut USB_OTG_DeviceTypeDef)).DSTS &
                  (0x3 as libc::c_uint) << 1 as libc::c_uint ==
                  ((2 as libc::c_int) << 1 as libc::c_int) as libc::c_uint {
        speed = 2 as libc::c_uint as uint8_t
    }
    return speed;
}
/* *
  * @brief  Activate and configure an endpoint
  * @param  USBx : Selected device
  * @param  ep: pointer to endpoint structure
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_ActivateEndpoint(mut USBx:
                                                  *mut USB_OTG_GlobalTypeDef,
                                              mut ep: *mut USB_OTG_EPTypeDef)
 -> HAL_StatusTypeDef {
    if (*ep).is_in as libc::c_int == 1 as libc::c_int {
        let ref mut fresh4 =
            (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                   *mut USB_OTG_DeviceTypeDef)).DAINTMSK;
        ::core::ptr::write_volatile(fresh4,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0xffff as libc::c_uint) <<
                                             0 as libc::c_uint &
                                             ((1 as libc::c_int) <<
                                                  (*ep).num as libc::c_int) as
                                                 libc::c_uint) as uint32_t as
                                        uint32_t);
        if (*((USBx as
                   uint32_t).wrapping_add(0x900 as
                                              libc::c_uint).wrapping_add(((*ep).num
                                                                              as
                                                                              libc::c_uint).wrapping_mul(0x20
                                                                                                             as
                                                                                                             libc::c_uint))
                  as *mut USB_OTG_INEndpointTypeDef)).DIEPCTL &
               (0x1 as libc::c_uint) << 15 as libc::c_uint ==
               0 as libc::c_int as libc::c_uint {
            let ref mut fresh5 =
                (*((USBx as
                        uint32_t).wrapping_add(0x900 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_INEndpointTypeDef)).DIEPCTL;
            ::core::ptr::write_volatile(fresh5,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             ((*ep).maxpacket &
                                                  (0x7ff as libc::c_uint) <<
                                                      0 as libc::c_uint |
                                                  (((*ep).type_0 as
                                                        libc::c_int) <<
                                                       18 as libc::c_int) as
                                                      libc::c_uint |
                                                  (((*ep).num as libc::c_int)
                                                       << 22 as libc::c_int)
                                                      as libc::c_uint |
                                                  (0x1 as libc::c_uint) <<
                                                      28 as libc::c_uint |
                                                  (0x1 as libc::c_uint) <<
                                                      15 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    } else {
        let ref mut fresh6 =
            (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                   *mut USB_OTG_DeviceTypeDef)).DAINTMSK;
        ::core::ptr::write_volatile(fresh6,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0xffff as libc::c_uint) <<
                                             16 as libc::c_uint &
                                             (((1 as libc::c_int) <<
                                                   (*ep).num as libc::c_int)
                                                  << 16 as libc::c_int) as
                                                 libc::c_uint) as uint32_t as
                                        uint32_t);
        if (*((USBx as
                   uint32_t).wrapping_add(0xb00 as
                                              libc::c_uint).wrapping_add(((*ep).num
                                                                              as
                                                                              libc::c_uint).wrapping_mul(0x20
                                                                                                             as
                                                                                                             libc::c_uint))
                  as *mut USB_OTG_OUTEndpointTypeDef)).DOEPCTL &
               (0x1 as libc::c_uint) << 15 as libc::c_uint ==
               0 as libc::c_int as libc::c_uint {
            let ref mut fresh7 =
                (*((USBx as
                        uint32_t).wrapping_add(0xb00 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_OUTEndpointTypeDef)).DOEPCTL;
            ::core::ptr::write_volatile(fresh7,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             ((*ep).maxpacket &
                                                  (0x7ff as libc::c_uint) <<
                                                      0 as libc::c_uint |
                                                  (((*ep).type_0 as
                                                        libc::c_int) <<
                                                       18 as libc::c_int) as
                                                      libc::c_uint |
                                                  (0x1 as libc::c_uint) <<
                                                      28 as libc::c_uint |
                                                  (0x1 as libc::c_uint) <<
                                                      15 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    return HAL_OK;
}
/* *
  * @brief  Activate and configure a dedicated endpoint
  * @param  USBx : Selected device
  * @param  ep: pointer to endpoint structure
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_ActivateDedicatedEndpoint(mut USBx:
                                                           *mut USB_OTG_GlobalTypeDef,
                                                       mut ep:
                                                           *mut USB_OTG_EPTypeDef)
 -> HAL_StatusTypeDef {
    static mut debug: uint32_t = 0 as libc::c_int as uint32_t;
    /* Read DEPCTLn register */
    if (*ep).is_in as libc::c_int == 1 as libc::c_int {
        if (*((USBx as
                   uint32_t).wrapping_add(0x900 as
                                              libc::c_uint).wrapping_add(((*ep).num
                                                                              as
                                                                              libc::c_uint).wrapping_mul(0x20
                                                                                                             as
                                                                                                             libc::c_uint))
                  as *mut USB_OTG_INEndpointTypeDef)).DIEPCTL &
               (0x1 as libc::c_uint) << 15 as libc::c_uint ==
               0 as libc::c_int as libc::c_uint {
            let ref mut fresh8 =
                (*((USBx as
                        uint32_t).wrapping_add(0x900 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_INEndpointTypeDef)).DIEPCTL;
            ::core::ptr::write_volatile(fresh8,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             ((*ep).maxpacket &
                                                  (0x7ff as libc::c_uint) <<
                                                      0 as libc::c_uint |
                                                  (((*ep).type_0 as
                                                        libc::c_int) <<
                                                       18 as libc::c_int) as
                                                      libc::c_uint |
                                                  (((*ep).num as libc::c_int)
                                                       << 22 as libc::c_int)
                                                      as libc::c_uint |
                                                  (0x1 as libc::c_uint) <<
                                                      28 as libc::c_uint |
                                                  (0x1 as libc::c_uint) <<
                                                      15 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        ::core::ptr::write_volatile(&mut debug as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&debug
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((*ep).maxpacket &
                                              (0x7ff as libc::c_uint) <<
                                                  0 as libc::c_uint |
                                              (((*ep).type_0 as libc::c_int)
                                                   << 18 as libc::c_int) as
                                                  libc::c_uint |
                                              (((*ep).num as libc::c_int) <<
                                                   22 as libc::c_int) as
                                                  libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  28 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  15 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        let ref mut fresh9 =
            (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                   *mut USB_OTG_DeviceTypeDef)).DEACHMSK;
        ::core::ptr::write_volatile(fresh9,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0xffff as libc::c_uint) <<
                                             0 as libc::c_uint &
                                             ((1 as libc::c_int) <<
                                                  (*ep).num as libc::c_int) as
                                                 libc::c_uint) as uint32_t as
                                        uint32_t)
    } else {
        if (*((USBx as
                   uint32_t).wrapping_add(0xb00 as
                                              libc::c_uint).wrapping_add(((*ep).num
                                                                              as
                                                                              libc::c_uint).wrapping_mul(0x20
                                                                                                             as
                                                                                                             libc::c_uint))
                  as *mut USB_OTG_OUTEndpointTypeDef)).DOEPCTL &
               (0x1 as libc::c_uint) << 15 as libc::c_uint ==
               0 as libc::c_int as libc::c_uint {
            let ref mut fresh10 =
                (*((USBx as
                        uint32_t).wrapping_add(0xb00 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_OUTEndpointTypeDef)).DOEPCTL;
            ::core::ptr::write_volatile(fresh10,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh10
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             ((*ep).maxpacket &
                                                  (0x7ff as libc::c_uint) <<
                                                      0 as libc::c_uint |
                                                  (((*ep).type_0 as
                                                        libc::c_int) <<
                                                       18 as libc::c_int) as
                                                      libc::c_uint |
                                                  (((*ep).num as libc::c_int)
                                                       << 22 as libc::c_int)
                                                      as libc::c_uint |
                                                  (0x1 as libc::c_uint) <<
                                                      15 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut debug as *mut uint32_t,
                                        (USBx as
                                             uint32_t).wrapping_add(0xb00 as
                                                                        libc::c_uint).wrapping_add((0
                                                                                                        as
                                                                                                        libc::c_int
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_mul(0x20
                                                                                                                                       as
                                                                                                                                       libc::c_uint)));
            ::core::ptr::write_volatile(&mut debug as *mut uint32_t,
                                        &mut (*((USBx as
                                                     uint32_t).wrapping_add(0xb00
                                                                                as
                                                                                libc::c_uint).wrapping_add(((*ep).num
                                                                                                                as
                                                                                                                libc::c_uint).wrapping_mul(0x20
                                                                                                                                               as
                                                                                                                                               libc::c_uint))
                                                    as
                                                    *mut USB_OTG_OUTEndpointTypeDef)).DOEPCTL
                                            as *mut uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut debug as *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&debug
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             ((*ep).maxpacket &
                                                  (0x7ff as libc::c_uint) <<
                                                      0 as libc::c_uint |
                                                  (((*ep).type_0 as
                                                        libc::c_int) <<
                                                       18 as libc::c_int) as
                                                      libc::c_uint |
                                                  (((*ep).num as libc::c_int)
                                                       << 22 as libc::c_int)
                                                      as libc::c_uint |
                                                  (0x1 as libc::c_uint) <<
                                                      15 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        let ref mut fresh11 =
            (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                   *mut USB_OTG_DeviceTypeDef)).DEACHMSK;
        ::core::ptr::write_volatile(fresh11,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh11
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0xffff as libc::c_uint) <<
                                             16 as libc::c_uint &
                                             (((1 as libc::c_int) <<
                                                   (*ep).num as libc::c_int)
                                                  << 16 as libc::c_int) as
                                                 libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    return HAL_OK;
}
/* *
  * @brief  De-activate and de-initialize an endpoint
  * @param  USBx : Selected device
  * @param  ep: pointer to endpoint structure
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_DeactivateEndpoint(mut USBx:
                                                    *mut USB_OTG_GlobalTypeDef,
                                                mut ep:
                                                    *mut USB_OTG_EPTypeDef)
 -> HAL_StatusTypeDef {
    /* Read DEPCTLn register */
    if (*ep).is_in as libc::c_int == 1 as libc::c_int {
        let ref mut fresh12 =
            (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                   *mut USB_OTG_DeviceTypeDef)).DEACHMSK;
        ::core::ptr::write_volatile(fresh12,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh12
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0xffff as libc::c_uint) <<
                                               0 as libc::c_uint &
                                               ((1 as libc::c_int) <<
                                                    (*ep).num as libc::c_int)
                                                   as libc::c_uint)) as
                                        uint32_t as uint32_t);
        let ref mut fresh13 =
            (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                   *mut USB_OTG_DeviceTypeDef)).DAINTMSK;
        ::core::ptr::write_volatile(fresh13,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh13
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0xffff as libc::c_uint) <<
                                               0 as libc::c_uint &
                                               ((1 as libc::c_int) <<
                                                    (*ep).num as libc::c_int)
                                                   as libc::c_uint)) as
                                        uint32_t as uint32_t);
        let ref mut fresh14 =
            (*((USBx as
                    uint32_t).wrapping_add(0x900 as
                                               libc::c_uint).wrapping_add(((*ep).num
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_INEndpointTypeDef)).DIEPCTL;
        ::core::ptr::write_volatile(fresh14,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh14
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               15 as libc::c_uint)) as
                                        uint32_t as uint32_t)
    } else {
        let ref mut fresh15 =
            (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                   *mut USB_OTG_DeviceTypeDef)).DEACHMSK;
        ::core::ptr::write_volatile(fresh15,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh15
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0xffff as libc::c_uint) <<
                                               16 as libc::c_uint &
                                               (((1 as libc::c_int) <<
                                                     (*ep).num as libc::c_int)
                                                    << 16 as libc::c_int) as
                                                   libc::c_uint)) as uint32_t
                                        as uint32_t);
        let ref mut fresh16 =
            (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                   *mut USB_OTG_DeviceTypeDef)).DAINTMSK;
        ::core::ptr::write_volatile(fresh16,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh16
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0xffff as libc::c_uint) <<
                                               16 as libc::c_uint &
                                               (((1 as libc::c_int) <<
                                                     (*ep).num as libc::c_int)
                                                    << 16 as libc::c_int) as
                                                   libc::c_uint)) as uint32_t
                                        as uint32_t);
        let ref mut fresh17 =
            (*((USBx as
                    uint32_t).wrapping_add(0xb00 as
                                               libc::c_uint).wrapping_add(((*ep).num
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_OUTEndpointTypeDef)).DOEPCTL;
        ::core::ptr::write_volatile(fresh17,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh17
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               15 as libc::c_uint)) as
                                        uint32_t as uint32_t)
    }
    return HAL_OK;
}
/* *
  * @brief  De-activate and de-initialize a dedicated endpoint
  * @param  USBx : Selected device
  * @param  ep: pointer to endpoint structure
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_DeactivateDedicatedEndpoint(mut USBx:
                                                             *mut USB_OTG_GlobalTypeDef,
                                                         mut ep:
                                                             *mut USB_OTG_EPTypeDef)
 -> HAL_StatusTypeDef {
    /* Read DEPCTLn register */
    if (*ep).is_in as libc::c_int == 1 as libc::c_int {
        let ref mut fresh18 =
            (*((USBx as
                    uint32_t).wrapping_add(0x900 as
                                               libc::c_uint).wrapping_add(((*ep).num
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_INEndpointTypeDef)).DIEPCTL;
        ::core::ptr::write_volatile(fresh18,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh18
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               15 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        let ref mut fresh19 =
            (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                   *mut USB_OTG_DeviceTypeDef)).DAINTMSK;
        ::core::ptr::write_volatile(fresh19,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh19
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0xffff as libc::c_uint) <<
                                               0 as libc::c_uint &
                                               ((1 as libc::c_int) <<
                                                    (*ep).num as libc::c_int)
                                                   as libc::c_uint)) as
                                        uint32_t as uint32_t)
    } else {
        let ref mut fresh20 =
            (*((USBx as
                    uint32_t).wrapping_add(0xb00 as
                                               libc::c_uint).wrapping_add(((*ep).num
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_OUTEndpointTypeDef)).DOEPCTL;
        ::core::ptr::write_volatile(fresh20,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh20
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               15 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        let ref mut fresh21 =
            (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                   *mut USB_OTG_DeviceTypeDef)).DAINTMSK;
        ::core::ptr::write_volatile(fresh21,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh21
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0xffff as libc::c_uint) <<
                                               16 as libc::c_uint &
                                               (((1 as libc::c_int) <<
                                                     (*ep).num as libc::c_int)
                                                    << 16 as libc::c_int) as
                                                   libc::c_uint)) as uint32_t
                                        as uint32_t)
    }
    return HAL_OK;
}
/* *
  * @brief  USB_EPStartXfer : setup and starts a transfer over an EP
  * @param  USBx : Selected device
  * @param  ep: pointer to endpoint structure
  * @param  dma: USB dma enabled or disabled 
  *          This parameter can be one of these values:
  *           0 : DMA feature not used 
  *           1 : DMA feature used  
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_EPStartXfer(mut USBx: *mut USB_OTG_GlobalTypeDef,
                                         mut ep: *mut USB_OTG_EPTypeDef,
                                         mut dma: uint8_t)
 -> HAL_StatusTypeDef {
    let mut pktcnt: uint16_t = 0 as libc::c_int as uint16_t;
    /* IN endpoint */
    if (*ep).is_in as libc::c_int == 1 as libc::c_int {
        /* Zero Length Packet? */
        if (*ep).xfer_len == 0 as libc::c_int as libc::c_uint {
            let ref mut fresh22 =
                (*((USBx as
                        uint32_t).wrapping_add(0x900 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_INEndpointTypeDef)).DIEPTSIZ;
            ::core::ptr::write_volatile(fresh22,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh22
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x3ff as libc::c_uint) <<
                                                   19 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            let ref mut fresh23 =
                (*((USBx as
                        uint32_t).wrapping_add(0x900 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_INEndpointTypeDef)).DIEPTSIZ;
            ::core::ptr::write_volatile(fresh23,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh23
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x3ff as libc::c_uint) <<
                                                 19 as libc::c_uint &
                                                 ((1 as libc::c_int) <<
                                                      19 as libc::c_int) as
                                                     libc::c_uint) as uint32_t
                                            as uint32_t);
            let ref mut fresh24 =
                (*((USBx as
                        uint32_t).wrapping_add(0x900 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_INEndpointTypeDef)).DIEPTSIZ;
            ::core::ptr::write_volatile(fresh24,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh24
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x7ffff as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        } else {
            /* Program the transfer size and packet count
      * as follows: xfersize = N * maxpacket +
      * short_packet pktcnt = N + (short_packet
      * exist ? 1 : 0)
      */
            let ref mut fresh25 =
                (*((USBx as
                        uint32_t).wrapping_add(0x900 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_INEndpointTypeDef)).DIEPTSIZ;
            ::core::ptr::write_volatile(fresh25,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh25
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x7ffff as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            let ref mut fresh26 =
                (*((USBx as
                        uint32_t).wrapping_add(0x900 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_INEndpointTypeDef)).DIEPTSIZ;
            ::core::ptr::write_volatile(fresh26,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh26
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x3ff as libc::c_uint) <<
                                                   19 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            let ref mut fresh27 =
                (*((USBx as
                        uint32_t).wrapping_add(0x900 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_INEndpointTypeDef)).DIEPTSIZ;
            ::core::ptr::write_volatile(fresh27,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh27
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x3ff as libc::c_uint) <<
                                                 19 as libc::c_uint &
                                                 (*ep).xfer_len.wrapping_add((*ep).maxpacket).wrapping_sub(1
                                                                                                               as
                                                                                                               libc::c_int
                                                                                                               as
                                                                                                               libc::c_uint).wrapping_div((*ep).maxpacket)
                                                     << 19 as libc::c_int) as
                                            uint32_t as uint32_t);
            let ref mut fresh28 =
                (*((USBx as
                        uint32_t).wrapping_add(0x900 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_INEndpointTypeDef)).DIEPTSIZ;
            ::core::ptr::write_volatile(fresh28,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh28
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x7ffff as libc::c_uint) <<
                                                 0 as libc::c_uint &
                                                 (*ep).xfer_len) as uint32_t
                                            as uint32_t);
            if (*ep).type_0 as libc::c_uint == 1 as libc::c_uint {
                let ref mut fresh29 =
                    (*((USBx as
                            uint32_t).wrapping_add(0x900 as
                                                       libc::c_uint).wrapping_add(((*ep).num
                                                                                       as
                                                                                       libc::c_uint).wrapping_mul(0x20
                                                                                                                      as
                                                                                                                      libc::c_uint))
                           as *mut USB_OTG_INEndpointTypeDef)).DIEPTSIZ;
                ::core::ptr::write_volatile(fresh29,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh29
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x3 as libc::c_uint) <<
                                                       29 as libc::c_uint)) as
                                                uint32_t as uint32_t);
                let ref mut fresh30 =
                    (*((USBx as
                            uint32_t).wrapping_add(0x900 as
                                                       libc::c_uint).wrapping_add(((*ep).num
                                                                                       as
                                                                                       libc::c_uint).wrapping_mul(0x20
                                                                                                                      as
                                                                                                                      libc::c_uint))
                           as *mut USB_OTG_INEndpointTypeDef)).DIEPTSIZ;
                ::core::ptr::write_volatile(fresh30,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh30
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x3 as libc::c_uint) <<
                                                     29 as libc::c_uint &
                                                     ((1 as libc::c_int) <<
                                                          29 as libc::c_int)
                                                         as libc::c_uint) as
                                                uint32_t as uint32_t)
            }
        }
        if dma as libc::c_int == 1 as libc::c_int {
            ::core::ptr::write_volatile(&mut (*((USBx as
                                                     uint32_t).wrapping_add(0x900
                                                                                as
                                                                                libc::c_uint).wrapping_add(((*ep).num
                                                                                                                as
                                                                                                                libc::c_uint).wrapping_mul(0x20
                                                                                                                                               as
                                                                                                                                               libc::c_uint))
                                                    as
                                                    *mut USB_OTG_INEndpointTypeDef)).DIEPDMA
                                            as *mut uint32_t, (*ep).dma_addr)
        } else if (*ep).type_0 as libc::c_uint != 1 as libc::c_uint {
            /* Enable the Tx FIFO Empty Interrupt for this EP */
            if (*ep).xfer_len > 0 as libc::c_int as libc::c_uint {
                let ref mut fresh31 =
                    (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint)
                           as *mut USB_OTG_DeviceTypeDef)).DIEPEMPMSK;
                ::core::ptr::write_volatile(fresh31,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh31
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 ((1 as libc::c_int) <<
                                                      (*ep).num as
                                                          libc::c_int) as
                                                     libc::c_uint) as uint32_t
                                                as uint32_t)
            }
        }
        if (*ep).type_0 as libc::c_uint == 1 as libc::c_uint {
            if (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                      *mut USB_OTG_DeviceTypeDef)).DSTS &
                   ((1 as libc::c_int) << 8 as libc::c_int) as libc::c_uint ==
                   0 as libc::c_int as libc::c_uint {
                let ref mut fresh32 =
                    (*((USBx as
                            uint32_t).wrapping_add(0x900 as
                                                       libc::c_uint).wrapping_add(((*ep).num
                                                                                       as
                                                                                       libc::c_uint).wrapping_mul(0x20
                                                                                                                      as
                                                                                                                      libc::c_uint))
                           as *mut USB_OTG_INEndpointTypeDef)).DIEPCTL;
                ::core::ptr::write_volatile(fresh32,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh32
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     29 as libc::c_uint) as
                                                uint32_t as uint32_t)
            } else {
                let ref mut fresh33 =
                    (*((USBx as
                            uint32_t).wrapping_add(0x900 as
                                                       libc::c_uint).wrapping_add(((*ep).num
                                                                                       as
                                                                                       libc::c_uint).wrapping_mul(0x20
                                                                                                                      as
                                                                                                                      libc::c_uint))
                           as *mut USB_OTG_INEndpointTypeDef)).DIEPCTL;
                ::core::ptr::write_volatile(fresh33,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh33
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     28 as libc::c_uint) as
                                                uint32_t as uint32_t)
            }
        }
        /* EP enable, IN data in FIFO */
        let ref mut fresh34 =
            (*((USBx as
                    uint32_t).wrapping_add(0x900 as
                                               libc::c_uint).wrapping_add(((*ep).num
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_INEndpointTypeDef)).DIEPCTL;
        ::core::ptr::write_volatile(fresh34,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh34
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              26 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  31 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        if (*ep).type_0 as libc::c_uint == 1 as libc::c_uint {
            USB_WritePacket(USBx, (*ep).xfer_buff, (*ep).num,
                            (*ep).xfer_len as uint16_t, dma);
        }
    } else {
        /* OUT endpoint */
        /* Program the transfer size and packet count as follows:
    * pktcnt = N
    * xfersize = N * maxpacket
    */
        let ref mut fresh35 =
            (*((USBx as
                    uint32_t).wrapping_add(0xb00 as
                                               libc::c_uint).wrapping_add(((*ep).num
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_OUTEndpointTypeDef)).DOEPTSIZ;
        ::core::ptr::write_volatile(fresh35,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh35
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x7ffff as libc::c_uint) <<
                                               0 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        let ref mut fresh36 =
            (*((USBx as
                    uint32_t).wrapping_add(0xb00 as
                                               libc::c_uint).wrapping_add(((*ep).num
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_OUTEndpointTypeDef)).DOEPTSIZ;
        ::core::ptr::write_volatile(fresh36,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh36
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x3ff as libc::c_uint) <<
                                               19 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        if (*ep).xfer_len == 0 as libc::c_int as libc::c_uint {
            let ref mut fresh37 =
                (*((USBx as
                        uint32_t).wrapping_add(0xb00 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_OUTEndpointTypeDef)).DOEPTSIZ;
            ::core::ptr::write_volatile(fresh37,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh37
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x7ffff as libc::c_uint) <<
                                                 0 as libc::c_uint &
                                                 (*ep).maxpacket) as uint32_t
                                            as uint32_t);
            let ref mut fresh38 =
                (*((USBx as
                        uint32_t).wrapping_add(0xb00 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_OUTEndpointTypeDef)).DOEPTSIZ;
            ::core::ptr::write_volatile(fresh38,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh38
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x3ff as libc::c_uint) <<
                                                 19 as libc::c_uint &
                                                 ((1 as libc::c_int) <<
                                                      19 as libc::c_int) as
                                                     libc::c_uint) as uint32_t
                                            as uint32_t)
        } else {
            pktcnt =
                (*ep).xfer_len.wrapping_add((*ep).maxpacket).wrapping_sub(1 as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint).wrapping_div((*ep).maxpacket)
                    as uint16_t;
            let ref mut fresh39 =
                (*((USBx as
                        uint32_t).wrapping_add(0xb00 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_OUTEndpointTypeDef)).DOEPTSIZ;
            ::core::ptr::write_volatile(fresh39,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh39
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x3ff as libc::c_uint) <<
                                                 19 as libc::c_uint &
                                                 ((pktcnt as libc::c_int) <<
                                                      19 as libc::c_int) as
                                                     libc::c_uint) as uint32_t
                                            as uint32_t);
            let ref mut fresh40 =
                (*((USBx as
                        uint32_t).wrapping_add(0xb00 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_OUTEndpointTypeDef)).DOEPTSIZ;
            ::core::ptr::write_volatile(fresh40,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh40
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x7ffff as libc::c_uint) <<
                                                 0 as libc::c_uint &
                                                 (*ep).maxpacket.wrapping_mul(pktcnt
                                                                                  as
                                                                                  libc::c_uint))
                                            as uint32_t as uint32_t)
        }
        if dma as libc::c_int == 1 as libc::c_int {
            ::core::ptr::write_volatile(&mut (*((USBx as
                                                     uint32_t).wrapping_add(0xb00
                                                                                as
                                                                                libc::c_uint).wrapping_add(((*ep).num
                                                                                                                as
                                                                                                                libc::c_uint).wrapping_mul(0x20
                                                                                                                                               as
                                                                                                                                               libc::c_uint))
                                                    as
                                                    *mut USB_OTG_OUTEndpointTypeDef)).DOEPDMA
                                            as *mut uint32_t,
                                        (*ep).xfer_buff as uint32_t)
        }
        if (*ep).type_0 as libc::c_uint == 1 as libc::c_uint {
            if (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                      *mut USB_OTG_DeviceTypeDef)).DSTS &
                   ((1 as libc::c_int) << 8 as libc::c_int) as libc::c_uint ==
                   0 as libc::c_int as libc::c_uint {
                let ref mut fresh41 =
                    (*((USBx as
                            uint32_t).wrapping_add(0xb00 as
                                                       libc::c_uint).wrapping_add(((*ep).num
                                                                                       as
                                                                                       libc::c_uint).wrapping_mul(0x20
                                                                                                                      as
                                                                                                                      libc::c_uint))
                           as *mut USB_OTG_OUTEndpointTypeDef)).DOEPCTL;
                ::core::ptr::write_volatile(fresh41,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh41
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     29 as libc::c_uint) as
                                                uint32_t as uint32_t)
            } else {
                let ref mut fresh42 =
                    (*((USBx as
                            uint32_t).wrapping_add(0xb00 as
                                                       libc::c_uint).wrapping_add(((*ep).num
                                                                                       as
                                                                                       libc::c_uint).wrapping_mul(0x20
                                                                                                                      as
                                                                                                                      libc::c_uint))
                           as *mut USB_OTG_OUTEndpointTypeDef)).DOEPCTL;
                ::core::ptr::write_volatile(fresh42,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh42
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     28 as libc::c_uint) as
                                                uint32_t as uint32_t)
            }
        }
        let ref mut fresh43 =
            (*((USBx as
                    uint32_t).wrapping_add(0xb00 as
                                               libc::c_uint).wrapping_add(((*ep).num
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_OUTEndpointTypeDef)).DOEPCTL;
        ::core::ptr::write_volatile(fresh43,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh43
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              26 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  31 as libc::c_uint)) as
                                        uint32_t as uint32_t)
    }
    return HAL_OK;
}
/* EP enable */
/* *
  * @brief  USB_EP0StartXfer : setup and starts a transfer over the EP  0
  * @param  USBx : Selected device
  * @param  ep: pointer to endpoint structure
  * @param  dma: USB dma enabled or disabled 
  *          This parameter can be one of these values:
  *           0 : DMA feature not used 
  *           1 : DMA feature used  
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_EP0StartXfer(mut USBx:
                                              *mut USB_OTG_GlobalTypeDef,
                                          mut ep: *mut USB_OTG_EPTypeDef,
                                          mut dma: uint8_t)
 -> HAL_StatusTypeDef {
    /* IN endpoint */
    if (*ep).is_in as libc::c_int == 1 as libc::c_int {
        /* Zero Length Packet? */
        if (*ep).xfer_len == 0 as libc::c_int as libc::c_uint {
            let ref mut fresh44 =
                (*((USBx as
                        uint32_t).wrapping_add(0x900 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_INEndpointTypeDef)).DIEPTSIZ;
            ::core::ptr::write_volatile(fresh44,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh44
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x3ff as libc::c_uint) <<
                                                   19 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            let ref mut fresh45 =
                (*((USBx as
                        uint32_t).wrapping_add(0x900 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_INEndpointTypeDef)).DIEPTSIZ;
            ::core::ptr::write_volatile(fresh45,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh45
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x3ff as libc::c_uint) <<
                                                 19 as libc::c_uint &
                                                 ((1 as libc::c_int) <<
                                                      19 as libc::c_int) as
                                                     libc::c_uint) as uint32_t
                                            as uint32_t);
            let ref mut fresh46 =
                (*((USBx as
                        uint32_t).wrapping_add(0x900 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_INEndpointTypeDef)).DIEPTSIZ;
            ::core::ptr::write_volatile(fresh46,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh46
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x7ffff as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        } else {
            /* Program the transfer size and packet count
      * as follows: xfersize = N * maxpacket +
      * short_packet pktcnt = N + (short_packet
      * exist ? 1 : 0)
      */
            let ref mut fresh47 =
                (*((USBx as
                        uint32_t).wrapping_add(0x900 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_INEndpointTypeDef)).DIEPTSIZ;
            ::core::ptr::write_volatile(fresh47,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh47
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x7ffff as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            let ref mut fresh48 =
                (*((USBx as
                        uint32_t).wrapping_add(0x900 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_INEndpointTypeDef)).DIEPTSIZ;
            ::core::ptr::write_volatile(fresh48,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh48
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x3ff as libc::c_uint) <<
                                                   19 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            if (*ep).xfer_len > (*ep).maxpacket {
                (*ep).xfer_len = (*ep).maxpacket
            }
            let ref mut fresh49 =
                (*((USBx as
                        uint32_t).wrapping_add(0x900 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_INEndpointTypeDef)).DIEPTSIZ;
            ::core::ptr::write_volatile(fresh49,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh49
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x3ff as libc::c_uint) <<
                                                 19 as libc::c_uint &
                                                 ((1 as libc::c_int) <<
                                                      19 as libc::c_int) as
                                                     libc::c_uint) as uint32_t
                                            as uint32_t);
            let ref mut fresh50 =
                (*((USBx as
                        uint32_t).wrapping_add(0x900 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_INEndpointTypeDef)).DIEPTSIZ;
            ::core::ptr::write_volatile(fresh50,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh50
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x7ffff as libc::c_uint) <<
                                                 0 as libc::c_uint &
                                                 (*ep).xfer_len) as uint32_t
                                            as uint32_t)
        }
        /* EP enable, IN data in FIFO */
        let ref mut fresh51 =
            (*((USBx as
                    uint32_t).wrapping_add(0x900 as
                                               libc::c_uint).wrapping_add(((*ep).num
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_INEndpointTypeDef)).DIEPCTL;
        ::core::ptr::write_volatile(fresh51,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh51
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              26 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  31 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        if dma as libc::c_int == 1 as libc::c_int {
            ::core::ptr::write_volatile(&mut (*((USBx as
                                                     uint32_t).wrapping_add(0x900
                                                                                as
                                                                                libc::c_uint).wrapping_add(((*ep).num
                                                                                                                as
                                                                                                                libc::c_uint).wrapping_mul(0x20
                                                                                                                                               as
                                                                                                                                               libc::c_uint))
                                                    as
                                                    *mut USB_OTG_INEndpointTypeDef)).DIEPDMA
                                            as *mut uint32_t, (*ep).dma_addr)
        } else if (*ep).xfer_len > 0 as libc::c_uint {
            let ref mut fresh52 =
                (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
                       *mut USB_OTG_DeviceTypeDef)).DIEPEMPMSK;
            ::core::ptr::write_volatile(fresh52,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh52
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (1 as libc::c_uint) <<
                                                 (*ep).num as libc::c_int) as
                                            uint32_t as uint32_t)
        }
    } else {
        /* Enable the Tx FIFO Empty Interrupt for this EP */
        /* OUT endpoint */
        /* Program the transfer size and packet count as follows:
    * pktcnt = N
    * xfersize = N * maxpacket
    */
        let ref mut fresh53 =
            (*((USBx as
                    uint32_t).wrapping_add(0xb00 as
                                               libc::c_uint).wrapping_add(((*ep).num
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_OUTEndpointTypeDef)).DOEPTSIZ;
        ::core::ptr::write_volatile(fresh53,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh53
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x7ffff as libc::c_uint) <<
                                               0 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        let ref mut fresh54 =
            (*((USBx as
                    uint32_t).wrapping_add(0xb00 as
                                               libc::c_uint).wrapping_add(((*ep).num
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_OUTEndpointTypeDef)).DOEPTSIZ;
        ::core::ptr::write_volatile(fresh54,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh54
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x3ff as libc::c_uint) <<
                                               19 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        if (*ep).xfer_len > 0 as libc::c_int as libc::c_uint {
            (*ep).xfer_len = (*ep).maxpacket
        }
        let ref mut fresh55 =
            (*((USBx as
                    uint32_t).wrapping_add(0xb00 as
                                               libc::c_uint).wrapping_add(((*ep).num
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_OUTEndpointTypeDef)).DOEPTSIZ;
        ::core::ptr::write_volatile(fresh55,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh55
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x3ff as libc::c_uint) <<
                                             19 as libc::c_uint &
                                             ((1 as libc::c_int) <<
                                                  19 as libc::c_int) as
                                                 libc::c_uint) as uint32_t as
                                        uint32_t);
        let ref mut fresh56 =
            (*((USBx as
                    uint32_t).wrapping_add(0xb00 as
                                               libc::c_uint).wrapping_add(((*ep).num
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_OUTEndpointTypeDef)).DOEPTSIZ;
        ::core::ptr::write_volatile(fresh56,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh56
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x7ffff as libc::c_uint) <<
                                             0 as libc::c_uint &
                                             (*ep).maxpacket) as uint32_t as
                                        uint32_t);
        if dma as libc::c_int == 1 as libc::c_int {
            ::core::ptr::write_volatile(&mut (*((USBx as
                                                     uint32_t).wrapping_add(0xb00
                                                                                as
                                                                                libc::c_uint).wrapping_add(((*ep).num
                                                                                                                as
                                                                                                                libc::c_uint).wrapping_mul(0x20
                                                                                                                                               as
                                                                                                                                               libc::c_uint))
                                                    as
                                                    *mut USB_OTG_OUTEndpointTypeDef)).DOEPDMA
                                            as *mut uint32_t,
                                        (*ep).xfer_buff as uint32_t)
        }
        let ref mut fresh57 =
            (*((USBx as
                    uint32_t).wrapping_add(0xb00 as
                                               libc::c_uint).wrapping_add(((*ep).num
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_OUTEndpointTypeDef)).DOEPCTL;
        ::core::ptr::write_volatile(fresh57,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh57
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              26 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  31 as libc::c_uint)) as
                                        uint32_t as uint32_t)
    }
    return HAL_OK;
}
/* EP enable */
/* *
  * @brief  USB_WritePacket : Writes a packet into the Tx FIFO associated 
  *         with the EP/channel
  * @param  USBx : Selected device           
  * @param  src :  pointer to source buffer
  * @param  ch_ep_num : endpoint or host channel number
  * @param  len : Number of bytes to write
  * @param  dma: USB dma enabled or disabled 
  *          This parameter can be one of these values:
  *           0 : DMA feature not used 
  *           1 : DMA feature used  
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_WritePacket(mut USBx: *mut USB_OTG_GlobalTypeDef,
                                         mut src: *mut uint8_t,
                                         mut ch_ep_num: uint8_t,
                                         mut len: uint16_t, mut dma: uint8_t)
 -> HAL_StatusTypeDef {
    let mut count32b: uint32_t = 0 as libc::c_int as uint32_t;
    let mut i: uint32_t = 0 as libc::c_int as uint32_t;
    if dma as libc::c_int == 0 as libc::c_int {
        count32b =
            ((len as libc::c_int + 3 as libc::c_int) / 4 as libc::c_int) as
                uint32_t;
        i = 0 as libc::c_int as uint32_t;
        while i < count32b {
            ::core::ptr::write_volatile((USBx as
                                             uint32_t).wrapping_add(0x1000 as
                                                                        libc::c_uint).wrapping_add((ch_ep_num
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_mul(0x1000
                                                                                                                                       as
                                                                                                                                       libc::c_uint))
                                            as *mut uint32_t,
                                        *(src as *mut uint32_t));
            i = i.wrapping_add(1);
            src = src.offset(4 as libc::c_int as isize)
        }
    }
    return HAL_OK;
}
/* *
  * @brief  USB_ReadPacket : read a packet from the Tx FIFO associated 
  *         with the EP/channel
  * @param  USBx : Selected device  
  * @param  src : source pointer
  * @param  ch_ep_num : endpoint or host channel number
  * @param  len : Number of bytes to read
  * @param  dma: USB dma enabled or disabled 
  *          This parameter can be one of these values:
  *           0 : DMA feature not used 
  *           1 : DMA feature used  
  * @retval pointer to destination buffer
  */
#[no_mangle]
pub unsafe extern "C" fn USB_ReadPacket(mut USBx: *mut USB_OTG_GlobalTypeDef,
                                        mut dest: *mut uint8_t,
                                        mut len: uint16_t)
 -> *mut libc::c_void {
    let mut i: uint32_t = 0 as libc::c_int as uint32_t;
    let mut count32b: uint32_t =
        ((len as libc::c_int + 3 as libc::c_int) / 4 as libc::c_int) as
            uint32_t;
    i = 0 as libc::c_int as uint32_t;
    while i < count32b {
        *(dest as *mut uint32_t) =
            *((USBx as
                   uint32_t).wrapping_add(0x1000 as
                                              libc::c_uint).wrapping_add((0 as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint).wrapping_mul(0x1000
                                                                                                             as
                                                                                                             libc::c_uint))
                  as *mut uint32_t);
        i = i.wrapping_add(1);
        dest = dest.offset(4 as libc::c_int as isize)
    }
    return dest as *mut libc::c_void;
}
/* *
  * @brief  USB_EPSetStall : set a stall condition over an EP
  * @param  USBx : Selected device
  * @param  ep: pointer to endpoint structure   
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_EPSetStall(mut USBx: *mut USB_OTG_GlobalTypeDef,
                                        mut ep: *mut USB_OTG_EPTypeDef)
 -> HAL_StatusTypeDef {
    if (*ep).is_in as libc::c_int == 1 as libc::c_int {
        if (*((USBx as
                   uint32_t).wrapping_add(0x900 as
                                              libc::c_uint).wrapping_add(((*ep).num
                                                                              as
                                                                              libc::c_uint).wrapping_mul(0x20
                                                                                                             as
                                                                                                             libc::c_uint))
                  as *mut USB_OTG_INEndpointTypeDef)).DIEPCTL &
               (0x1 as libc::c_uint) << 31 as libc::c_uint ==
               0 as libc::c_int as libc::c_uint {
            let ref mut fresh58 =
                (*((USBx as
                        uint32_t).wrapping_add(0x900 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_INEndpointTypeDef)).DIEPCTL;
            ::core::ptr::write_volatile(fresh58,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh58
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   30 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        let ref mut fresh59 =
            (*((USBx as
                    uint32_t).wrapping_add(0x900 as
                                               libc::c_uint).wrapping_add(((*ep).num
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_INEndpointTypeDef)).DIEPCTL;
        ::core::ptr::write_volatile(fresh59,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh59
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             21 as libc::c_uint) as uint32_t
                                        as uint32_t)
    } else {
        if (*((USBx as
                   uint32_t).wrapping_add(0xb00 as
                                              libc::c_uint).wrapping_add(((*ep).num
                                                                              as
                                                                              libc::c_uint).wrapping_mul(0x20
                                                                                                             as
                                                                                                             libc::c_uint))
                  as *mut USB_OTG_OUTEndpointTypeDef)).DOEPCTL &
               (0x1 as libc::c_uint) << 31 as libc::c_uint ==
               0 as libc::c_int as libc::c_uint {
            let ref mut fresh60 =
                (*((USBx as
                        uint32_t).wrapping_add(0xb00 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_OUTEndpointTypeDef)).DOEPCTL;
            ::core::ptr::write_volatile(fresh60,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh60
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   30 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        let ref mut fresh61 =
            (*((USBx as
                    uint32_t).wrapping_add(0xb00 as
                                               libc::c_uint).wrapping_add(((*ep).num
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_OUTEndpointTypeDef)).DOEPCTL;
        ::core::ptr::write_volatile(fresh61,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh61
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             21 as libc::c_uint) as uint32_t
                                        as uint32_t)
    }
    return HAL_OK;
}
/* *
  * @brief  USB_EPClearStall : Clear a stall condition over an EP
  * @param  USBx : Selected device
  * @param  ep: pointer to endpoint structure   
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_EPClearStall(mut USBx:
                                              *mut USB_OTG_GlobalTypeDef,
                                          mut ep: *mut USB_OTG_EPTypeDef)
 -> HAL_StatusTypeDef {
    if (*ep).is_in as libc::c_int == 1 as libc::c_int {
        let ref mut fresh62 =
            (*((USBx as
                    uint32_t).wrapping_add(0x900 as
                                               libc::c_uint).wrapping_add(((*ep).num
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_INEndpointTypeDef)).DIEPCTL;
        ::core::ptr::write_volatile(fresh62,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh62
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               21 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        if (*ep).type_0 as libc::c_uint == 3 as libc::c_uint ||
               (*ep).type_0 as libc::c_uint == 2 as libc::c_uint {
            let ref mut fresh63 =
                (*((USBx as
                        uint32_t).wrapping_add(0x900 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_INEndpointTypeDef)).DIEPCTL;
            ::core::ptr::write_volatile(fresh63,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh63
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 28 as libc::c_uint) as
                                            uint32_t as uint32_t)
            /* DATA0 */
        }
    } else {
        let ref mut fresh64 =
            (*((USBx as
                    uint32_t).wrapping_add(0xb00 as
                                               libc::c_uint).wrapping_add(((*ep).num
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_OUTEndpointTypeDef)).DOEPCTL;
        ::core::ptr::write_volatile(fresh64,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh64
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               21 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        if (*ep).type_0 as libc::c_uint == 3 as libc::c_uint ||
               (*ep).type_0 as libc::c_uint == 2 as libc::c_uint {
            let ref mut fresh65 =
                (*((USBx as
                        uint32_t).wrapping_add(0xb00 as
                                                   libc::c_uint).wrapping_add(((*ep).num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_OUTEndpointTypeDef)).DOEPCTL;
            ::core::ptr::write_volatile(fresh65,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh65
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 28 as libc::c_uint) as
                                            uint32_t as uint32_t)
            /* DATA0 */
        }
    }
    return HAL_OK;
}
/* *
  * @brief  USB_StopDevice : Stop the usb device mode
  * @param  USBx : Selected device
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_StopDevice(mut USBx: *mut USB_OTG_GlobalTypeDef)
 -> HAL_StatusTypeDef {
    let mut i: uint32_t = 0;
    /* Clear Pending interrupt */
    i = 0 as libc::c_int as uint32_t;
    while i < 15 as libc::c_int as libc::c_uint {
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
                                             uint32_t).wrapping_add(0x800 as
                                                                        libc::c_uint)
                                            as
                                            *mut USB_OTG_DeviceTypeDef)).DAINT
                                    as *mut uint32_t,
                                0xffffffff as libc::c_uint);
    /* Clear interrupt masks */
    ::core::ptr::write_volatile(&mut (*((USBx as
                                             uint32_t).wrapping_add(0x800 as
                                                                        libc::c_uint)
                                            as
                                            *mut USB_OTG_DeviceTypeDef)).DIEPMSK
                                    as *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((USBx as
                                             uint32_t).wrapping_add(0x800 as
                                                                        libc::c_uint)
                                            as
                                            *mut USB_OTG_DeviceTypeDef)).DOEPMSK
                                    as *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((USBx as
                                             uint32_t).wrapping_add(0x800 as
                                                                        libc::c_uint)
                                            as
                                            *mut USB_OTG_DeviceTypeDef)).DAINTMSK
                                    as *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    /* Flush the FIFO */
    USB_FlushRxFifo(USBx);
    USB_FlushTxFifo(USBx, 0x10 as libc::c_int as uint32_t);
    return HAL_OK;
}
/* *
  * @brief  USB_SetDevAddress : Stop the usb device mode
  * @param  USBx : Selected device
  * @param  address : new device address to be assigned
  *          This parameter can be a value from 0 to 255
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_SetDevAddress(mut USBx:
                                               *mut USB_OTG_GlobalTypeDef,
                                           mut address: uint8_t)
 -> HAL_StatusTypeDef {
    let ref mut fresh66 =
        (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
               *mut USB_OTG_DeviceTypeDef)).DCFG;
    ::core::ptr::write_volatile(fresh66,
                                (::core::ptr::read_volatile::<uint32_t>(fresh66
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x7f as libc::c_uint) <<
                                           4 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    let ref mut fresh67 =
        (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
               *mut USB_OTG_DeviceTypeDef)).DCFG;
    ::core::ptr::write_volatile(fresh67,
                                (::core::ptr::read_volatile::<uint32_t>(fresh67
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((address as libc::c_int) <<
                                          4 as libc::c_int) as libc::c_uint &
                                         (0x7f as libc::c_uint) <<
                                             4 as libc::c_uint) as uint32_t as
                                    uint32_t);
    return HAL_OK;
}
/* *
  * @brief  USB_DevConnect : Connect the USB device by enabling the pull-up/pull-down
  * @param  USBx : Selected device
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_DevConnect(mut USBx: *mut USB_OTG_GlobalTypeDef)
 -> HAL_StatusTypeDef {
    let ref mut fresh68 =
        (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
               *mut USB_OTG_DeviceTypeDef)).DCTL;
    ::core::ptr::write_volatile(fresh68,
                                (::core::ptr::read_volatile::<uint32_t>(fresh68
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           1 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    HAL_Delay(3 as libc::c_int as uint32_t);
    return HAL_OK;
}
/* *
  * @brief  USB_DevDisconnect : Disconnect the USB device by disabling the pull-up/pull-down
  * @param  USBx : Selected device
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_DevDisconnect(mut USBx:
                                               *mut USB_OTG_GlobalTypeDef)
 -> HAL_StatusTypeDef {
    let ref mut fresh69 =
        (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
               *mut USB_OTG_DeviceTypeDef)).DCTL;
    ::core::ptr::write_volatile(fresh69,
                                (::core::ptr::read_volatile::<uint32_t>(fresh69
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         1 as libc::c_uint) as uint32_t as
                                    uint32_t);
    HAL_Delay(3 as libc::c_int as uint32_t);
    return HAL_OK;
}
/* *
  * @brief  USB_ReadInterrupts: return the global USB interrupt status
  * @param  USBx : Selected device
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_ReadInterrupts(mut USBx:
                                                *mut USB_OTG_GlobalTypeDef)
 -> uint32_t {
    let mut v: uint32_t = 0 as libc::c_int as uint32_t;
    v = (*USBx).GINTSTS;
    v &= (*USBx).GINTMSK;
    return v;
}
/* *
  * @brief  USB_ReadDevAllOutEpInterrupt: return the USB device OUT endpoints interrupt status
  * @param  USBx : Selected device
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_ReadDevAllOutEpInterrupt(mut USBx:
                                                          *mut USB_OTG_GlobalTypeDef)
 -> uint32_t {
    let mut v: uint32_t = 0;
    v =
        (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
               *mut USB_OTG_DeviceTypeDef)).DAINT;
    v &=
        (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
               *mut USB_OTG_DeviceTypeDef)).DAINTMSK;
    return (v & 0xffff0000 as libc::c_uint) >> 16 as libc::c_int;
}
/* *
  * @brief  USB_ReadDevAllInEpInterrupt: return the USB device IN endpoints interrupt status
  * @param  USBx : Selected device
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_ReadDevAllInEpInterrupt(mut USBx:
                                                         *mut USB_OTG_GlobalTypeDef)
 -> uint32_t {
    let mut v: uint32_t = 0;
    v =
        (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
               *mut USB_OTG_DeviceTypeDef)).DAINT;
    v &=
        (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
               *mut USB_OTG_DeviceTypeDef)).DAINTMSK;
    return v & 0xffff as libc::c_int as libc::c_uint;
}
/* *
  * @brief  Returns Device OUT EP Interrupt register
  * @param  USBx : Selected device
  * @param  epnum : endpoint number
  *          This parameter can be a value from 0 to 15
  * @retval Device OUT EP Interrupt register
  */
#[no_mangle]
pub unsafe extern "C" fn USB_ReadDevOutEPInterrupt(mut USBx:
                                                       *mut USB_OTG_GlobalTypeDef,
                                                   mut epnum: uint8_t)
 -> uint32_t {
    let mut v: uint32_t = 0;
    v =
        (*((USBx as
                uint32_t).wrapping_add(0xb00 as
                                           libc::c_uint).wrapping_add((epnum
                                                                           as
                                                                           libc::c_uint).wrapping_mul(0x20
                                                                                                          as
                                                                                                          libc::c_uint))
               as *mut USB_OTG_OUTEndpointTypeDef)).DOEPINT;
    v &=
        (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
               *mut USB_OTG_DeviceTypeDef)).DOEPMSK;
    return v;
}
/* *
  * @brief  Returns Device IN EP Interrupt register
  * @param  USBx : Selected device
  * @param  epnum : endpoint number
  *          This parameter can be a value from 0 to 15
  * @retval Device IN EP Interrupt register
  */
#[no_mangle]
pub unsafe extern "C" fn USB_ReadDevInEPInterrupt(mut USBx:
                                                      *mut USB_OTG_GlobalTypeDef,
                                                  mut epnum: uint8_t)
 -> uint32_t {
    let mut v: uint32_t = 0;
    let mut msk: uint32_t = 0;
    let mut emp: uint32_t = 0;
    msk =
        (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
               *mut USB_OTG_DeviceTypeDef)).DIEPMSK;
    emp =
        (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
               *mut USB_OTG_DeviceTypeDef)).DIEPEMPMSK;
    msk |=
        (emp >> epnum as libc::c_int & 0x1 as libc::c_int as libc::c_uint) <<
            7 as libc::c_int;
    v =
        (*((USBx as
                uint32_t).wrapping_add(0x900 as
                                           libc::c_uint).wrapping_add((epnum
                                                                           as
                                                                           libc::c_uint).wrapping_mul(0x20
                                                                                                          as
                                                                                                          libc::c_uint))
               as *mut USB_OTG_INEndpointTypeDef)).DIEPINT & msk;
    return v;
}
/* *
  * @brief  USB_ClearInterrupts: clear a USB interrupt
  * @param  USBx : Selected device
  * @param  interrupt : interrupt flag
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn USB_ClearInterrupts(mut USBx:
                                                 *mut USB_OTG_GlobalTypeDef,
                                             mut interrupt: uint32_t) {
    ::core::ptr::write_volatile(&mut (*USBx).GINTSTS as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GINTSTS
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | interrupt) as uint32_t
                                    as uint32_t);
}
/* *
  * @brief  Returns USB core mode
  * @param  USBx : Selected device
  * @retval return core mode : Host or Device
  *          This parameter can be one of these values:
  *           0 : Host 
  *           1 : Device
  */
#[no_mangle]
pub unsafe extern "C" fn USB_GetMode(mut USBx: *mut USB_OTG_GlobalTypeDef)
 -> uint32_t {
    return (*USBx).GINTSTS & 0x1 as libc::c_int as libc::c_uint;
}
/* *
  * @brief  Activate EP0 for Setup transactions
  * @param  USBx : Selected device
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_ActivateSetup(mut USBx:
                                               *mut USB_OTG_GlobalTypeDef)
 -> HAL_StatusTypeDef {
    /* Set the MPS of the IN EP based on the enumeration speed */
    let ref mut fresh70 =
        (*((USBx as
                uint32_t).wrapping_add(0x900 as
                                           libc::c_uint).wrapping_add((0 as
                                                                           libc::c_int
                                                                           as
                                                                           libc::c_uint).wrapping_mul(0x20
                                                                                                          as
                                                                                                          libc::c_uint))
               as *mut USB_OTG_INEndpointTypeDef)).DIEPCTL;
    ::core::ptr::write_volatile(fresh70,
                                (::core::ptr::read_volatile::<uint32_t>(fresh70
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x7ff as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    if (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
              *mut USB_OTG_DeviceTypeDef)).DSTS &
           (0x3 as libc::c_uint) << 1 as libc::c_uint ==
           ((2 as libc::c_int) << 1 as libc::c_int) as libc::c_uint {
        let ref mut fresh71 =
            (*((USBx as
                    uint32_t).wrapping_add(0x900 as
                                               libc::c_uint).wrapping_add((0
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_INEndpointTypeDef)).DIEPCTL;
        ::core::ptr::write_volatile(fresh71,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh71
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         3 as libc::c_int as libc::c_uint) as
                                        uint32_t as uint32_t)
    }
    let ref mut fresh72 =
        (*((USBx as uint32_t).wrapping_add(0x800 as libc::c_uint) as
               *mut USB_OTG_DeviceTypeDef)).DCTL;
    ::core::ptr::write_volatile(fresh72,
                                (::core::ptr::read_volatile::<uint32_t>(fresh72
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         8 as libc::c_uint) as uint32_t as
                                    uint32_t);
    return HAL_OK;
}
/* *
  * @brief  Prepare the EP0 to start the first control setup
  * @param  USBx : Selected device
  * @param  dma: USB dma enabled or disabled 
  *          This parameter can be one of these values:
  *           0 : DMA feature not used 
  *           1 : DMA feature used  
  * @param  psetup : pointer to setup packet
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_EP0_OutStart(mut USBx:
                                              *mut USB_OTG_GlobalTypeDef,
                                          mut dma: uint8_t,
                                          mut psetup: *mut uint8_t)
 -> HAL_StatusTypeDef {
    ::core::ptr::write_volatile(&mut (*((USBx as
                                             uint32_t).wrapping_add(0xb00 as
                                                                        libc::c_uint).wrapping_add((0
                                                                                                        as
                                                                                                        libc::c_int
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_mul(0x20
                                                                                                                                       as
                                                                                                                                       libc::c_uint))
                                            as
                                            *mut USB_OTG_OUTEndpointTypeDef)).DOEPTSIZ
                                    as *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    let ref mut fresh73 =
        (*((USBx as
                uint32_t).wrapping_add(0xb00 as
                                           libc::c_uint).wrapping_add((0 as
                                                                           libc::c_int
                                                                           as
                                                                           libc::c_uint).wrapping_mul(0x20
                                                                                                          as
                                                                                                          libc::c_uint))
               as *mut USB_OTG_OUTEndpointTypeDef)).DOEPTSIZ;
    ::core::ptr::write_volatile(fresh73,
                                (::core::ptr::read_volatile::<uint32_t>(fresh73
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x3ff as libc::c_uint) <<
                                         19 as libc::c_uint &
                                         ((1 as libc::c_int) <<
                                              19 as libc::c_int) as
                                             libc::c_uint) as uint32_t as
                                    uint32_t);
    let ref mut fresh74 =
        (*((USBx as
                uint32_t).wrapping_add(0xb00 as
                                           libc::c_uint).wrapping_add((0 as
                                                                           libc::c_int
                                                                           as
                                                                           libc::c_uint).wrapping_mul(0x20
                                                                                                          as
                                                                                                          libc::c_uint))
               as *mut USB_OTG_OUTEndpointTypeDef)).DOEPTSIZ;
    ::core::ptr::write_volatile(fresh74,
                                (::core::ptr::read_volatile::<uint32_t>(fresh74
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (3 as libc::c_int * 8 as libc::c_int) as
                                         libc::c_uint) as uint32_t as
                                    uint32_t);
    let ref mut fresh75 =
        (*((USBx as
                uint32_t).wrapping_add(0xb00 as
                                           libc::c_uint).wrapping_add((0 as
                                                                           libc::c_int
                                                                           as
                                                                           libc::c_uint).wrapping_mul(0x20
                                                                                                          as
                                                                                                          libc::c_uint))
               as *mut USB_OTG_OUTEndpointTypeDef)).DOEPTSIZ;
    ::core::ptr::write_volatile(fresh75,
                                (::core::ptr::read_volatile::<uint32_t>(fresh75
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x3 as libc::c_uint) <<
                                         29 as libc::c_uint) as uint32_t as
                                    uint32_t);
    if dma as libc::c_int == 1 as libc::c_int {
        ::core::ptr::write_volatile(&mut (*((USBx as
                                                 uint32_t).wrapping_add(0xb00
                                                                            as
                                                                            libc::c_uint).wrapping_add((0
                                                                                                            as
                                                                                                            libc::c_int
                                                                                                            as
                                                                                                            libc::c_uint).wrapping_mul(0x20
                                                                                                                                           as
                                                                                                                                           libc::c_uint))
                                                as
                                                *mut USB_OTG_OUTEndpointTypeDef)).DOEPDMA
                                        as *mut uint32_t, psetup as uint32_t);
        /* EP enable */
        ::core::ptr::write_volatile(&mut (*((USBx as
                                                 uint32_t).wrapping_add(0xb00
                                                                            as
                                                                            libc::c_uint).wrapping_add((0
                                                                                                            as
                                                                                                            libc::c_int
                                                                                                            as
                                                                                                            libc::c_uint).wrapping_mul(0x20
                                                                                                                                           as
                                                                                                                                           libc::c_uint))
                                                as
                                                *mut USB_OTG_OUTEndpointTypeDef)).DOEPCTL
                                        as *mut uint32_t,
                                    0x80008000 as libc::c_uint)
    }
    return HAL_OK;
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_ll_usb.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   USB Low Layer HAL module driver.
  *    
  *          This file provides firmware functions to manage the following 
  *          functionalities of the USB Peripheral Controller:
  *           + Initialization/de-initialization functions
  *           + I/O operation functions
  *           + Peripheral Control functions 
  *           + Peripheral State functions
  *         
  @verbatim
  ==============================================================================
                    ##### How to use this driver #####
  ==============================================================================
    [..]
      (#) Fill parameters of Init structure in USB_OTG_CfgTypeDef structure.
  
      (#) Call USB_CoreInit() API to initialize the USB Core peripheral.

      (#) The upper HAL HCD/PCD driver will call the right routines for its internal processes.

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
/* * @addtogroup STM32F7xx_LL_USB_DRIVER
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* *
  * @brief  Reset the USB Core (needed after USB clock settings change)
  * @param  USBx : Selected device
  * @retval HAL status
  */
unsafe extern "C" fn USB_CoreReset(mut USBx: *mut USB_OTG_GlobalTypeDef)
 -> HAL_StatusTypeDef {
    let mut count: uint32_t = 0 as libc::c_int as uint32_t;
    loop 
         /* Wait for AHB master IDLE state. */
         {
        count = count.wrapping_add(1);
        if count > 200000 as libc::c_int as libc::c_uint {
            return HAL_TIMEOUT
        }
        if !((*USBx).GRSTCTL & (0x1 as libc::c_uint) << 31 as libc::c_uint ==
                 0 as libc::c_int as libc::c_uint) {
            break ;
        }
    }
    /* Core Soft Reset */
    count = 0 as libc::c_int as uint32_t;
    ::core::ptr::write_volatile(&mut (*USBx).GRSTCTL as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GRSTCTL
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    loop  {
        count = count.wrapping_add(1);
        if count > 200000 as libc::c_int as libc::c_uint {
            return HAL_TIMEOUT
        }
        if !((*USBx).GRSTCTL & (0x1 as libc::c_uint) << 0 as libc::c_uint ==
                 (0x1 as libc::c_uint) << 0 as libc::c_uint) {
            break ;
        }
    }
    return HAL_OK;
}
/* USB_HS_PHYC */
/* *
  * @brief  USB_HostInit : Initializes the USB OTG controller registers 
  *         for Host mode 
  * @param  USBx : Selected device
  * @param  cfg  : pointer to a USB_OTG_CfgTypeDef structure that contains
  *         the configuration information for the specified USBx peripheral.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_HostInit(mut USBx: *mut USB_OTG_GlobalTypeDef,
                                      mut cfg: USB_OTG_CfgTypeDef)
 -> HAL_StatusTypeDef {
    let mut i: uint32_t = 0;
    /* Restart the Phy Clock */
    ::core::ptr::write_volatile((USBx as
                                     uint32_t).wrapping_add(0xe00 as
                                                                libc::c_uint)
                                    as *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    /*Activate VBUS Sensing B */
    ::core::ptr::write_volatile(&mut (*USBx).GCCFG as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GCCFG
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         21 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Disable the FS/LS support mode only */
    if cfg.speed == 3 as libc::c_uint &&
           USBx != 0x50000000 as libc::c_uint as *mut USB_OTG_GlobalTypeDef {
        let ref mut fresh76 =
            (*((USBx as uint32_t).wrapping_add(0x400 as libc::c_uint) as
                   *mut USB_OTG_HostTypeDef)).HCFG;
        ::core::ptr::write_volatile(fresh76,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh76
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             2 as libc::c_uint) as uint32_t as
                                        uint32_t)
    } else {
        let ref mut fresh77 =
            (*((USBx as uint32_t).wrapping_add(0x400 as libc::c_uint) as
                   *mut USB_OTG_HostTypeDef)).HCFG;
        ::core::ptr::write_volatile(fresh77,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh77
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               2 as libc::c_uint)) as uint32_t
                                        as uint32_t)
    }
    /* Make sure the FIFOs are flushed. */
    USB_FlushTxFifo(USBx, 0x10 as libc::c_int as uint32_t); /* all Tx FIFOs */
    USB_FlushRxFifo(USBx);
    /* Clear all pending HC Interrupts */
    i = 0 as libc::c_int as uint32_t;
    while i < cfg.Host_channels {
        ::core::ptr::write_volatile(&mut (*((USBx as
                                                 uint32_t).wrapping_add(0x500
                                                                            as
                                                                            libc::c_uint).wrapping_add(i.wrapping_mul(0x20
                                                                                                                          as
                                                                                                                          libc::c_uint))
                                                as
                                                *mut USB_OTG_HostChannelTypeDef)).HCINT
                                        as *mut uint32_t,
                                    0xffffffff as libc::c_uint);
        ::core::ptr::write_volatile(&mut (*((USBx as
                                                 uint32_t).wrapping_add(0x500
                                                                            as
                                                                            libc::c_uint).wrapping_add(i.wrapping_mul(0x20
                                                                                                                          as
                                                                                                                          libc::c_uint))
                                                as
                                                *mut USB_OTG_HostChannelTypeDef)).HCINTMSK
                                        as *mut uint32_t,
                                    0 as libc::c_int as uint32_t);
        i = i.wrapping_add(1)
    }
    /* Enable VBUS driving */
    USB_DriveVbus(USBx, 1 as libc::c_int as uint8_t);
    HAL_Delay(200 as libc::c_int as uint32_t);
    /* Disable all interrupts. */
    ::core::ptr::write_volatile(&mut (*USBx).GINTMSK as *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    /* Clear any pending interrupts */
    ::core::ptr::write_volatile(&mut (*USBx).GINTSTS as *mut uint32_t,
                                0xffffffff as libc::c_uint);
    if USBx == 0x50000000 as libc::c_uint as *mut USB_OTG_GlobalTypeDef {
        /* set Rx FIFO size */
        ::core::ptr::write_volatile(&mut (*USBx).GRXFSIZ as *mut uint32_t,
                                    0x80 as libc::c_int as uint32_t);
        ::core::ptr::write_volatile(&mut (*USBx).DIEPTXF0_HNPTXFSIZ as
                                        *mut uint32_t,
                                    ((0x60 as libc::c_int) <<
                                         16 as libc::c_int) as libc::c_uint &
                                        (0xffff as libc::c_uint) <<
                                            16 as libc::c_uint |
                                        0x80 as libc::c_int as libc::c_uint);
        ::core::ptr::write_volatile(&mut (*USBx).HPTXFSIZ as *mut uint32_t,
                                    ((0x40 as libc::c_int) <<
                                         16 as libc::c_int) as libc::c_uint &
                                        (0xffff as libc::c_uint) <<
                                            16 as libc::c_uint |
                                        0xe0 as libc::c_int as libc::c_uint)
    } else {
        /* set Rx FIFO size */
        ::core::ptr::write_volatile(&mut (*USBx).GRXFSIZ as *mut uint32_t,
                                    0x200 as libc::c_int as uint32_t);
        ::core::ptr::write_volatile(&mut (*USBx).DIEPTXF0_HNPTXFSIZ as
                                        *mut uint32_t,
                                    ((0x100 as libc::c_int) <<
                                         16 as libc::c_int) as libc::c_uint &
                                        (0xffff as libc::c_uint) <<
                                            16 as libc::c_uint |
                                        0x200 as libc::c_int as libc::c_uint);
        ::core::ptr::write_volatile(&mut (*USBx).HPTXFSIZ as *mut uint32_t,
                                    ((0xe0 as libc::c_int) <<
                                         16 as libc::c_int) as libc::c_uint &
                                        (0xffff as libc::c_uint) <<
                                            16 as libc::c_uint |
                                        0x300 as libc::c_int as libc::c_uint)
    }
    /* Enable the common interrupts */
    if cfg.dma_enable == DISABLE as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*USBx).GINTMSK as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GINTMSK
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             4 as libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    /* Enable interrupts matching to the Host mode ONLY */
    ::core::ptr::write_volatile(&mut (*USBx).GINTMSK as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GINTMSK
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((0x1 as libc::c_uint) <<
                                          24 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              25 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              3 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              29 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              21 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              31 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    return HAL_OK;
}
/* *
  * @brief  USB_InitFSLSPClkSel : Initializes the FSLSPClkSel field of the 
  *         HCFG register on the PHY type and set the right frame interval
  * @param  USBx : Selected device
  * @param  freq : clock frequency
  *          This parameter can be one of these values:
  *           HCFG_48_MHZ : Full Speed 48 MHz Clock 
  *           HCFG_6_MHZ : Low Speed 6 MHz Clock 
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn USB_InitFSLSPClkSel(mut USBx:
                                                 *mut USB_OTG_GlobalTypeDef,
                                             mut freq: uint8_t)
 -> HAL_StatusTypeDef {
    let ref mut fresh78 =
        (*((USBx as uint32_t).wrapping_add(0x400 as libc::c_uint) as
               *mut USB_OTG_HostTypeDef)).HCFG;
    ::core::ptr::write_volatile(fresh78,
                                (::core::ptr::read_volatile::<uint32_t>(fresh78
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x3 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    let ref mut fresh79 =
        (*((USBx as uint32_t).wrapping_add(0x400 as libc::c_uint) as
               *mut USB_OTG_HostTypeDef)).HCFG;
    ::core::ptr::write_volatile(fresh79,
                                (::core::ptr::read_volatile::<uint32_t>(fresh79
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     freq as libc::c_uint &
                                         (0x3 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    if freq as libc::c_uint == 1 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*((USBx as
                                                 uint32_t).wrapping_add(0x400
                                                                            as
                                                                            libc::c_uint)
                                                as
                                                *mut USB_OTG_HostTypeDef)).HFIR
                                        as *mut uint32_t,
                                    48000 as libc::c_int as uint32_t)
    } else if freq as libc::c_uint == 2 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*((USBx as
                                                 uint32_t).wrapping_add(0x400
                                                                            as
                                                                            libc::c_uint)
                                                as
                                                *mut USB_OTG_HostTypeDef)).HFIR
                                        as *mut uint32_t,
                                    6000 as libc::c_int as uint32_t)
    }
    return HAL_OK;
}
/* *
* @brief  USB_OTG_ResetPort : Reset Host Port
  * @param  USBx : Selected device
  * @retval HAL status
  * @note : (1)The application must wait at least 10 ms
  *   before clearing the reset bit.
  */
#[no_mangle]
pub unsafe extern "C" fn USB_ResetPort(mut USBx: *mut USB_OTG_GlobalTypeDef)
 -> HAL_StatusTypeDef {
    let mut hprt0: uint32_t = 0; /* See Note #1 */
    ::core::ptr::write_volatile(&mut hprt0 as *mut uint32_t,
                                *((USBx as
                                       uint32_t).wrapping_add(0x440 as
                                                                  libc::c_uint)
                                      as *mut uint32_t));
    ::core::ptr::write_volatile(&mut hprt0 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&hprt0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           2 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               1 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               3 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               5 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    ::core::ptr::write_volatile((USBx as
                                     uint32_t).wrapping_add(0x440 as
                                                                libc::c_uint)
                                    as *mut uint32_t,
                                (0x1 as libc::c_uint) << 8 as libc::c_uint |
                                    hprt0);
    HAL_Delay(100 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile((USBx as
                                     uint32_t).wrapping_add(0x440 as
                                                                libc::c_uint)
                                    as *mut uint32_t,
                                !((0x1 as libc::c_uint) << 8 as libc::c_uint)
                                    & hprt0);
    HAL_Delay(10 as libc::c_int as uint32_t);
    return HAL_OK;
}
/* *
  * @brief  USB_DriveVbus : activate or de-activate vbus
  * @param  state : VBUS state
  *          This parameter can be one of these values:
  *           0 : VBUS Active 
  *           1 : VBUS Inactive
  * @retval HAL status
*/
#[no_mangle]
pub unsafe extern "C" fn USB_DriveVbus(mut USBx: *mut USB_OTG_GlobalTypeDef,
                                       mut state: uint8_t)
 -> HAL_StatusTypeDef {
    let mut hprt0: uint32_t = 0;
    ::core::ptr::write_volatile(&mut hprt0 as *mut uint32_t,
                                *((USBx as
                                       uint32_t).wrapping_add(0x440 as
                                                                  libc::c_uint)
                                      as *mut uint32_t));
    ::core::ptr::write_volatile(&mut hprt0 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&hprt0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           2 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               1 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               3 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               5 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    if hprt0 & (0x1 as libc::c_uint) << 12 as libc::c_uint ==
           0 as libc::c_int as libc::c_uint &&
           state as libc::c_int == 1 as libc::c_int {
        ::core::ptr::write_volatile((USBx as
                                         uint32_t).wrapping_add(0x440 as
                                                                    libc::c_uint)
                                        as *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        12 as libc::c_uint | hprt0)
    }
    if hprt0 & (0x1 as libc::c_uint) << 12 as libc::c_uint ==
           (0x1 as libc::c_uint) << 12 as libc::c_uint &&
           state as libc::c_int == 0 as libc::c_int {
        ::core::ptr::write_volatile((USBx as
                                         uint32_t).wrapping_add(0x440 as
                                                                    libc::c_uint)
                                        as *mut uint32_t,
                                    !((0x1 as libc::c_uint) <<
                                          12 as libc::c_uint) & hprt0)
    }
    return HAL_OK;
}
/* *
  * @brief  Return Host Core speed
  * @param  USBx : Selected device
  * @retval speed : Host speed
  *          This parameter can be one of these values:
  *            @arg USB_OTG_SPEED_HIGH: High speed mode
  *            @arg USB_OTG_SPEED_FULL: Full speed mode
  *            @arg USB_OTG_SPEED_LOW: Low speed mode
  */
#[no_mangle]
pub unsafe extern "C" fn USB_GetHostSpeed(mut USBx:
                                              *mut USB_OTG_GlobalTypeDef)
 -> uint32_t {
    let mut hprt0: uint32_t = 0;
    ::core::ptr::write_volatile(&mut hprt0 as *mut uint32_t,
                                *((USBx as
                                       uint32_t).wrapping_add(0x440 as
                                                                  libc::c_uint)
                                      as *mut uint32_t));
    return (hprt0 & (0x3 as libc::c_uint) << 17 as libc::c_uint) >>
               17 as libc::c_int;
}
/* *
  * @brief  Return Host Current Frame number
  * @param  USBx : Selected device
  * @retval current frame number
*/
#[no_mangle]
pub unsafe extern "C" fn USB_GetCurrentFrame(mut USBx:
                                                 *mut USB_OTG_GlobalTypeDef)
 -> uint32_t {
    return (*((USBx as uint32_t).wrapping_add(0x400 as libc::c_uint) as
                  *mut USB_OTG_HostTypeDef)).HFNUM &
               (0xffff as libc::c_uint) << 0 as libc::c_uint;
}
/* *
  * @brief  Initialize a host channel
  * @param  USBx : Selected device
  * @param  ch_num : Channel number
  *         This parameter can be a value from 1 to 15
  * @param  epnum : Endpoint number
  *          This parameter can be a value from 1 to 15
  * @param  dev_address : Current device address
  *          This parameter can be a value from 0 to 255
  * @param  speed : Current device speed
  *          This parameter can be one of these values:
  *            @arg USB_OTG_SPEED_HIGH: High speed mode
  *            @arg USB_OTG_SPEED_FULL: Full speed mode
  *            @arg USB_OTG_SPEED_LOW: Low speed mode
  * @param  ep_type : Endpoint Type
  *          This parameter can be one of these values:
  *            @arg EP_TYPE_CTRL: Control type
  *            @arg EP_TYPE_ISOC: Isochronous type
  *            @arg EP_TYPE_BULK: Bulk type
  *            @arg EP_TYPE_INTR: Interrupt type
  * @param  mps : Max Packet Size
  *          This parameter can be a value from 0 to32K
  * @retval HAL state
  */
#[no_mangle]
pub unsafe extern "C" fn USB_HC_Init(mut USBx: *mut USB_OTG_GlobalTypeDef,
                                     mut ch_num: uint8_t, mut epnum: uint8_t,
                                     mut dev_address: uint8_t,
                                     mut speed: uint8_t, mut ep_type: uint8_t,
                                     mut mps: uint16_t) -> HAL_StatusTypeDef {
    /* Clear old interrupt conditions for this host channel. */
    ::core::ptr::write_volatile(&mut (*((USBx as
                                             uint32_t).wrapping_add(0x500 as
                                                                        libc::c_uint).wrapping_add((ch_num
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_mul(0x20
                                                                                                                                       as
                                                                                                                                       libc::c_uint))
                                            as
                                            *mut USB_OTG_HostChannelTypeDef)).HCINT
                                    as *mut uint32_t,
                                0xffffffff as libc::c_uint);
    /* Enable channel interrupts required for this transfer. */
    match ep_type as libc::c_int {
        0 | 2 => {
            ::core::ptr::write_volatile(&mut (*((USBx as
                                                     uint32_t).wrapping_add(0x500
                                                                                as
                                                                                libc::c_uint).wrapping_add((ch_num
                                                                                                                as
                                                                                                                libc::c_uint).wrapping_mul(0x20
                                                                                                                                               as
                                                                                                                                               libc::c_uint))
                                                    as
                                                    *mut USB_OTG_HostChannelTypeDef)).HCINTMSK
                                            as *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            0 as libc::c_uint |
                                            (0x1 as libc::c_uint) <<
                                                3 as libc::c_uint |
                                            (0x1 as libc::c_uint) <<
                                                7 as libc::c_uint |
                                            (0x1 as libc::c_uint) <<
                                                10 as libc::c_uint |
                                            (0x1 as libc::c_uint) <<
                                                2 as libc::c_uint |
                                            (0x1 as libc::c_uint) <<
                                                4 as libc::c_uint);
            if epnum as libc::c_int & 0x80 as libc::c_int != 0 {
                let ref mut fresh80 =
                    (*((USBx as
                            uint32_t).wrapping_add(0x500 as
                                                       libc::c_uint).wrapping_add((ch_num
                                                                                       as
                                                                                       libc::c_uint).wrapping_mul(0x20
                                                                                                                      as
                                                                                                                      libc::c_uint))
                           as *mut USB_OTG_HostChannelTypeDef)).HCINTMSK;
                ::core::ptr::write_volatile(fresh80,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh80
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     8 as libc::c_uint) as
                                                uint32_t as uint32_t)
            } else if USBx !=
                          0x50000000 as libc::c_uint as
                              *mut USB_OTG_GlobalTypeDef {
                let ref mut fresh81 =
                    (*((USBx as
                            uint32_t).wrapping_add(0x500 as
                                                       libc::c_uint).wrapping_add((ch_num
                                                                                       as
                                                                                       libc::c_uint).wrapping_mul(0x20
                                                                                                                      as
                                                                                                                      libc::c_uint))
                           as *mut USB_OTG_HostChannelTypeDef)).HCINTMSK;
                ::core::ptr::write_volatile(fresh81,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh81
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 ((0x1 as libc::c_uint) <<
                                                      6 as libc::c_uint |
                                                      (0x1 as libc::c_uint) <<
                                                          5 as libc::c_uint))
                                                as uint32_t as uint32_t)
            }
        }
        3 => {
            ::core::ptr::write_volatile(&mut (*((USBx as
                                                     uint32_t).wrapping_add(0x500
                                                                                as
                                                                                libc::c_uint).wrapping_add((ch_num
                                                                                                                as
                                                                                                                libc::c_uint).wrapping_mul(0x20
                                                                                                                                               as
                                                                                                                                               libc::c_uint))
                                                    as
                                                    *mut USB_OTG_HostChannelTypeDef)).HCINTMSK
                                            as *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            0 as libc::c_uint |
                                            (0x1 as libc::c_uint) <<
                                                3 as libc::c_uint |
                                            (0x1 as libc::c_uint) <<
                                                7 as libc::c_uint |
                                            (0x1 as libc::c_uint) <<
                                                10 as libc::c_uint |
                                            (0x1 as libc::c_uint) <<
                                                4 as libc::c_uint |
                                            (0x1 as libc::c_uint) <<
                                                2 as libc::c_uint |
                                            (0x1 as libc::c_uint) <<
                                                9 as libc::c_uint);
            if epnum as libc::c_int & 0x80 as libc::c_int != 0 {
                let ref mut fresh82 =
                    (*((USBx as
                            uint32_t).wrapping_add(0x500 as
                                                       libc::c_uint).wrapping_add((ch_num
                                                                                       as
                                                                                       libc::c_uint).wrapping_mul(0x20
                                                                                                                      as
                                                                                                                      libc::c_uint))
                           as *mut USB_OTG_HostChannelTypeDef)).HCINTMSK;
                ::core::ptr::write_volatile(fresh82,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh82
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     8 as libc::c_uint) as
                                                uint32_t as uint32_t)
            }
        }
        1 => {
            ::core::ptr::write_volatile(&mut (*((USBx as
                                                     uint32_t).wrapping_add(0x500
                                                                                as
                                                                                libc::c_uint).wrapping_add((ch_num
                                                                                                                as
                                                                                                                libc::c_uint).wrapping_mul(0x20
                                                                                                                                               as
                                                                                                                                               libc::c_uint))
                                                    as
                                                    *mut USB_OTG_HostChannelTypeDef)).HCINTMSK
                                            as *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            0 as libc::c_uint |
                                            (0x1 as libc::c_uint) <<
                                                5 as libc::c_uint |
                                            (0x1 as libc::c_uint) <<
                                                2 as libc::c_uint |
                                            (0x1 as libc::c_uint) <<
                                                9 as libc::c_uint);
            if epnum as libc::c_int & 0x80 as libc::c_int != 0 {
                let ref mut fresh83 =
                    (*((USBx as
                            uint32_t).wrapping_add(0x500 as
                                                       libc::c_uint).wrapping_add((ch_num
                                                                                       as
                                                                                       libc::c_uint).wrapping_mul(0x20
                                                                                                                      as
                                                                                                                      libc::c_uint))
                           as *mut USB_OTG_HostChannelTypeDef)).HCINTMSK;
                ::core::ptr::write_volatile(fresh83,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh83
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 ((0x1 as libc::c_uint) <<
                                                      7 as libc::c_uint |
                                                      (0x1 as libc::c_uint) <<
                                                          8 as libc::c_uint))
                                                as uint32_t as uint32_t)
            }
        }
        _ => { }
    }
    /* Enable the top level host channel interrupt. */
    let ref mut fresh84 =
        (*((USBx as uint32_t).wrapping_add(0x400 as libc::c_uint) as
               *mut USB_OTG_HostTypeDef)).HAINTMSK;
    ::core::ptr::write_volatile(fresh84,
                                (::core::ptr::read_volatile::<uint32_t>(fresh84
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((1 as libc::c_int) <<
                                          ch_num as libc::c_int) as
                                         libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Make sure host channel interrupts are enabled. */
    ::core::ptr::write_volatile(&mut (*USBx).GINTMSK as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GINTMSK
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         25 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Program the HCCHAR register */
    ::core::ptr::write_volatile(&mut (*((USBx as
                                             uint32_t).wrapping_add(0x500 as
                                                                        libc::c_uint).wrapping_add((ch_num
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_mul(0x20
                                                                                                                                       as
                                                                                                                                       libc::c_uint))
                                            as
                                            *mut USB_OTG_HostChannelTypeDef)).HCCHAR
                                    as *mut uint32_t,
                                ((dev_address as libc::c_int) <<
                                     22 as libc::c_int) as libc::c_uint &
                                    (0x7f as libc::c_uint) <<
                                        22 as libc::c_uint |
                                    ((epnum as libc::c_int &
                                          0x7f as libc::c_int) <<
                                         11 as libc::c_int) as libc::c_uint &
                                        (0xf as libc::c_uint) <<
                                            11 as libc::c_uint |
                                    (((epnum as libc::c_int &
                                           0x80 as libc::c_int ==
                                           0x80 as libc::c_int) as
                                          libc::c_int) << 15 as libc::c_int)
                                        as libc::c_uint &
                                        (0x1 as libc::c_uint) <<
                                            15 as libc::c_uint |
                                    (((speed as libc::c_uint ==
                                           2 as libc::c_uint) as libc::c_int)
                                         << 17 as libc::c_int) as libc::c_uint
                                        &
                                        (0x1 as libc::c_uint) <<
                                            17 as libc::c_uint |
                                    ((ep_type as libc::c_int) <<
                                         18 as libc::c_int) as libc::c_uint &
                                        (0x3 as libc::c_uint) <<
                                            18 as libc::c_uint |
                                    mps as libc::c_uint &
                                        (0x7ff as libc::c_uint) <<
                                            0 as libc::c_uint);
    if ep_type as libc::c_uint == 3 as libc::c_uint {
        let ref mut fresh85 =
            (*((USBx as
                    uint32_t).wrapping_add(0x500 as
                                               libc::c_uint).wrapping_add((ch_num
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_HostChannelTypeDef)).HCCHAR;
        ::core::ptr::write_volatile(fresh85,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh85
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             29 as libc::c_uint) as uint32_t
                                        as uint32_t)
    }
    return HAL_OK;
}
/* *
  * @brief  Start a transfer over a host channel
  * @param  USBx : Selected device
  * @param  hc : pointer to host channel structure
  * @param  dma: USB dma enabled or disabled 
  *          This parameter can be one of these values:
  *           0 : DMA feature not used 
  *           1 : DMA feature used  
  * @retval HAL state
  */
#[no_mangle]
pub unsafe extern "C" fn USB_HC_StartXfer(mut USBx:
                                              *mut USB_OTG_GlobalTypeDef,
                                          mut hc: *mut USB_OTG_HCTypeDef,
                                          mut dma: uint8_t)
 -> HAL_StatusTypeDef {
    static mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    let mut is_oddframe: uint8_t = 0 as libc::c_int as uint8_t;
    let mut len_words: uint16_t = 0 as libc::c_int as uint16_t;
    let mut num_packets: uint16_t = 0 as libc::c_int as uint16_t;
    let mut max_hc_pkt_count: uint16_t = 256 as libc::c_int as uint16_t;
    if USBx != 0x50000000 as libc::c_uint as *mut USB_OTG_GlobalTypeDef &&
           (*hc).speed as libc::c_uint == 0 as libc::c_uint {
        if dma as libc::c_int == 0 as libc::c_int &&
               (*hc).do_ping as libc::c_int == 1 as libc::c_int {
            USB_DoPing(USBx, (*hc).ch_num);
            return HAL_OK
        } else {
            if dma as libc::c_int == 1 as libc::c_int {
                let ref mut fresh86 =
                    (*((USBx as
                            uint32_t).wrapping_add(0x500 as
                                                       libc::c_uint).wrapping_add(((*hc).ch_num
                                                                                       as
                                                                                       libc::c_uint).wrapping_mul(0x20
                                                                                                                      as
                                                                                                                      libc::c_uint))
                           as *mut USB_OTG_HostChannelTypeDef)).HCINTMSK;
                ::core::ptr::write_volatile(fresh86,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh86
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       6 as libc::c_uint |
                                                       (0x1 as libc::c_uint)
                                                           <<
                                                           5 as libc::c_uint))
                                                as uint32_t as uint32_t);
                (*hc).do_ping = 0 as libc::c_int as uint8_t
            }
        }
    }
    /* Compute the expected number of packets associated to the transfer */
    if (*hc).xfer_len > 0 as libc::c_int as libc::c_uint {
        num_packets =
            (*hc).xfer_len.wrapping_add((*hc).max_packet as
                                            libc::c_uint).wrapping_sub(1 as
                                                                           libc::c_int
                                                                           as
                                                                           libc::c_uint).wrapping_div((*hc).max_packet
                                                                                                          as
                                                                                                          libc::c_uint)
                as uint16_t;
        if num_packets as libc::c_int > max_hc_pkt_count as libc::c_int {
            num_packets = max_hc_pkt_count;
            (*hc).xfer_len =
                (num_packets as libc::c_int * (*hc).max_packet as libc::c_int)
                    as uint32_t
        }
    } else { num_packets = 1 as libc::c_int as uint16_t }
    if (*hc).ep_is_in != 0 {
        (*hc).xfer_len =
            (num_packets as libc::c_int * (*hc).max_packet as libc::c_int) as
                uint32_t
    }
    /* Initialize the HCTSIZn register */
    ::core::ptr::write_volatile(&mut (*((USBx as
                                             uint32_t).wrapping_add(0x500 as
                                                                        libc::c_uint).wrapping_add(((*hc).ch_num
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_mul(0x20
                                                                                                                                       as
                                                                                                                                       libc::c_uint))
                                            as
                                            *mut USB_OTG_HostChannelTypeDef)).HCTSIZ
                                    as *mut uint32_t,
                                (*hc).xfer_len &
                                    (0x7ffff as libc::c_uint) <<
                                        0 as libc::c_uint |
                                    ((num_packets as libc::c_int) <<
                                         19 as libc::c_int) as libc::c_uint &
                                        (0x3ff as libc::c_uint) <<
                                            19 as libc::c_uint |
                                    (((*hc).data_pid as libc::c_int) <<
                                         29 as libc::c_int) as libc::c_uint &
                                        (0x3 as libc::c_uint) <<
                                            29 as libc::c_uint);
    if dma != 0 {
        /* xfer_buff MUST be 32-bits aligned */
        ::core::ptr::write_volatile(&mut (*((USBx as
                                                 uint32_t).wrapping_add(0x500
                                                                            as
                                                                            libc::c_uint).wrapping_add(((*hc).ch_num
                                                                                                            as
                                                                                                            libc::c_uint).wrapping_mul(0x20
                                                                                                                                           as
                                                                                                                                           libc::c_uint))
                                                as
                                                *mut USB_OTG_HostChannelTypeDef)).HCDMA
                                        as *mut uint32_t,
                                    (*hc).xfer_buff as uint32_t)
    }
    is_oddframe =
        if (*((USBx as uint32_t).wrapping_add(0x400 as libc::c_uint) as
                  *mut USB_OTG_HostTypeDef)).HFNUM &
               0x1 as libc::c_int as libc::c_uint != 0 {
            0 as libc::c_int
        } else { 1 as libc::c_int } as uint8_t;
    let ref mut fresh87 =
        (*((USBx as
                uint32_t).wrapping_add(0x500 as
                                           libc::c_uint).wrapping_add(((*hc).ch_num
                                                                           as
                                                                           libc::c_uint).wrapping_mul(0x20
                                                                                                          as
                                                                                                          libc::c_uint))
               as *mut USB_OTG_HostChannelTypeDef)).HCCHAR;
    ::core::ptr::write_volatile(fresh87,
                                (::core::ptr::read_volatile::<uint32_t>(fresh87
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           29 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    let ref mut fresh88 =
        (*((USBx as
                uint32_t).wrapping_add(0x500 as
                                           libc::c_uint).wrapping_add(((*hc).ch_num
                                                                           as
                                                                           libc::c_uint).wrapping_mul(0x20
                                                                                                          as
                                                                                                          libc::c_uint))
               as *mut USB_OTG_HostChannelTypeDef)).HCCHAR;
    ::core::ptr::write_volatile(fresh88,
                                (::core::ptr::read_volatile::<uint32_t>(fresh88
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((is_oddframe as libc::c_int) <<
                                          29 as libc::c_int) as libc::c_uint)
                                    as uint32_t as uint32_t);
    /* Set host channel enable */
    ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                (*((USBx as
                                        uint32_t).wrapping_add(0x500 as
                                                                   libc::c_uint).wrapping_add(((*hc).ch_num
                                                                                                   as
                                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                                  as
                                                                                                                                  libc::c_uint))
                                       as
                                       *mut USB_OTG_HostChannelTypeDef)).HCCHAR);
    ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&tmpreg
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           30 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&tmpreg
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         31 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*((USBx as
                                             uint32_t).wrapping_add(0x500 as
                                                                        libc::c_uint).wrapping_add(((*hc).ch_num
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_mul(0x20
                                                                                                                                       as
                                                                                                                                       libc::c_uint))
                                            as
                                            *mut USB_OTG_HostChannelTypeDef)).HCCHAR
                                    as *mut uint32_t, tmpreg);
    if dma as libc::c_int == 0 as libc::c_int {
        /* Slave mode */
        if (*hc).ep_is_in as libc::c_int == 0 as libc::c_int &&
               (*hc).xfer_len > 0 as libc::c_int as libc::c_uint {
            match (*hc).ep_type as libc::c_int {
                0 | 2 => {
                    /* Non periodic transfer */
                    len_words =
                        (*hc).xfer_len.wrapping_add(3 as libc::c_int as
                                                        libc::c_uint).wrapping_div(4
                                                                                       as
                                                                                       libc::c_int
                                                                                       as
                                                                                       libc::c_uint)
                            as uint16_t;
                    /* check if there is enough space in FIFO space */
                    if len_words as libc::c_uint >
                           (*USBx).HNPTXSTS &
                               0xffff as libc::c_int as libc::c_uint {
                        /* need to process data in nptxfempty interrupt */
                        ::core::ptr::write_volatile(&mut (*USBx).GINTMSK as
                                                        *mut uint32_t,
                                                    (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GINTMSK
                                                                                                as
                                                                                                *const uint32_t)
                                                         as libc::c_uint |
                                                         (0x1 as libc::c_uint)
                                                             <<
                                                             5 as
                                                                 libc::c_uint)
                                                        as uint32_t as
                                                        uint32_t)
                    }
                }
                3 | 1 => {
                    /* Periodic transfer */
                    len_words =
                        (*hc).xfer_len.wrapping_add(3 as libc::c_int as
                                                        libc::c_uint).wrapping_div(4
                                                                                       as
                                                                                       libc::c_int
                                                                                       as
                                                                                       libc::c_uint)
                            as uint16_t;
                    /* check if there is enough space in FIFO space */
                    if len_words as libc::c_uint >
                           (*((USBx as
                                   uint32_t).wrapping_add(0x400 as
                                                              libc::c_uint) as
                                  *mut USB_OTG_HostTypeDef)).HPTXSTS &
                               0xffff as libc::c_int as libc::c_uint {
                        /* split the transfer */
                        /* need to process data in ptxfempty interrupt */
                        ::core::ptr::write_volatile(&mut (*USBx).GINTMSK as
                                                        *mut uint32_t,
                                                    (::core::ptr::read_volatile::<uint32_t>(&(*USBx).GINTMSK
                                                                                                as
                                                                                                *const uint32_t)
                                                         as libc::c_uint |
                                                         (0x1 as libc::c_uint)
                                                             <<
                                                             26 as
                                                                 libc::c_uint)
                                                        as uint32_t as
                                                        uint32_t)
                    }
                }
                _ => { }
            }
            /* Write packet into the Tx FIFO. */
            USB_WritePacket(USBx, (*hc).xfer_buff, (*hc).ch_num,
                            (*hc).xfer_len as uint16_t,
                            0 as libc::c_int as uint8_t);
        }
    }
    return HAL_OK;
}
/* *
  * @brief Read all host channel interrupts status
  * @param  USBx : Selected device
  * @retval HAL state
  */
#[no_mangle]
pub unsafe extern "C" fn USB_HC_ReadInterrupt(mut USBx:
                                                  *mut USB_OTG_GlobalTypeDef)
 -> uint32_t {
    return (*((USBx as uint32_t).wrapping_add(0x400 as libc::c_uint) as
                  *mut USB_OTG_HostTypeDef)).HAINT &
               0xffff as libc::c_int as libc::c_uint;
}
/* *
  * @brief  Halt a host channel
  * @param  USBx : Selected device
  * @param  hc_num : Host Channel number
  *         This parameter can be a value from 1 to 15
  * @retval HAL state
  */
#[no_mangle]
pub unsafe extern "C" fn USB_HC_Halt(mut USBx: *mut USB_OTG_GlobalTypeDef,
                                     mut hc_num: uint8_t)
 -> HAL_StatusTypeDef {
    let mut count: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check for space in the request queue to issue the halt. */
    if ((*((USBx as
                uint32_t).wrapping_add(0x500 as
                                           libc::c_uint).wrapping_add((hc_num
                                                                           as
                                                                           libc::c_uint).wrapping_mul(0x20
                                                                                                          as
                                                                                                          libc::c_uint))
               as *mut USB_OTG_HostChannelTypeDef)).HCCHAR &
            (0x3 as libc::c_uint) << 18 as libc::c_uint) >> 18 as libc::c_int
           == 0 as libc::c_uint ||
           ((*((USBx as
                    uint32_t).wrapping_add(0x500 as
                                               libc::c_uint).wrapping_add((hc_num
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_HostChannelTypeDef)).HCCHAR &
                (0x3 as libc::c_uint) << 18 as libc::c_uint) >>
               18 as libc::c_int == 2 as libc::c_uint {
        let ref mut fresh89 =
            (*((USBx as
                    uint32_t).wrapping_add(0x500 as
                                               libc::c_uint).wrapping_add((hc_num
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_HostChannelTypeDef)).HCCHAR;
        ::core::ptr::write_volatile(fresh89,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh89
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             30 as libc::c_uint) as uint32_t
                                        as uint32_t);
        if (*USBx).HNPTXSTS & 0xffff as libc::c_int as libc::c_uint ==
               0 as libc::c_int as libc::c_uint {
            let ref mut fresh90 =
                (*((USBx as
                        uint32_t).wrapping_add(0x500 as
                                                   libc::c_uint).wrapping_add((hc_num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_HostChannelTypeDef)).HCCHAR;
            ::core::ptr::write_volatile(fresh90,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh90
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   31 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            let ref mut fresh91 =
                (*((USBx as
                        uint32_t).wrapping_add(0x500 as
                                                   libc::c_uint).wrapping_add((hc_num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_HostChannelTypeDef)).HCCHAR;
            ::core::ptr::write_volatile(fresh91,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh91
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 31 as libc::c_uint) as
                                            uint32_t as uint32_t);
            let ref mut fresh92 =
                (*((USBx as
                        uint32_t).wrapping_add(0x500 as
                                                   libc::c_uint).wrapping_add((hc_num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_HostChannelTypeDef)).HCCHAR;
            ::core::ptr::write_volatile(fresh92,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh92
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   15 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            loop  {
                count = count.wrapping_add(1);
                if count > 1000 as libc::c_int as libc::c_uint { break ; }
                if !((*((USBx as
                             uint32_t).wrapping_add(0x500 as
                                                        libc::c_uint).wrapping_add((hc_num
                                                                                        as
                                                                                        libc::c_uint).wrapping_mul(0x20
                                                                                                                       as
                                                                                                                       libc::c_uint))
                            as *mut USB_OTG_HostChannelTypeDef)).HCCHAR &
                         (0x1 as libc::c_uint) << 31 as libc::c_uint ==
                         (0x1 as libc::c_uint) << 31 as libc::c_uint) {
                    break ;
                }
            }
        } else {
            let ref mut fresh93 =
                (*((USBx as
                        uint32_t).wrapping_add(0x500 as
                                                   libc::c_uint).wrapping_add((hc_num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_HostChannelTypeDef)).HCCHAR;
            ::core::ptr::write_volatile(fresh93,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh93
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 31 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
    } else {
        let ref mut fresh94 =
            (*((USBx as
                    uint32_t).wrapping_add(0x500 as
                                               libc::c_uint).wrapping_add((hc_num
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_HostChannelTypeDef)).HCCHAR;
        ::core::ptr::write_volatile(fresh94,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh94
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             30 as libc::c_uint) as uint32_t
                                        as uint32_t);
        if (*((USBx as uint32_t).wrapping_add(0x400 as libc::c_uint) as
                  *mut USB_OTG_HostTypeDef)).HPTXSTS &
               0xffff as libc::c_int as libc::c_uint ==
               0 as libc::c_int as libc::c_uint {
            let ref mut fresh95 =
                (*((USBx as
                        uint32_t).wrapping_add(0x500 as
                                                   libc::c_uint).wrapping_add((hc_num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_HostChannelTypeDef)).HCCHAR;
            ::core::ptr::write_volatile(fresh95,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh95
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   31 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            let ref mut fresh96 =
                (*((USBx as
                        uint32_t).wrapping_add(0x500 as
                                                   libc::c_uint).wrapping_add((hc_num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_HostChannelTypeDef)).HCCHAR;
            ::core::ptr::write_volatile(fresh96,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh96
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 31 as libc::c_uint) as
                                            uint32_t as uint32_t);
            let ref mut fresh97 =
                (*((USBx as
                        uint32_t).wrapping_add(0x500 as
                                                   libc::c_uint).wrapping_add((hc_num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_HostChannelTypeDef)).HCCHAR;
            ::core::ptr::write_volatile(fresh97,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh97
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   15 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            loop  {
                count = count.wrapping_add(1);
                if count > 1000 as libc::c_int as libc::c_uint { break ; }
                if !((*((USBx as
                             uint32_t).wrapping_add(0x500 as
                                                        libc::c_uint).wrapping_add((hc_num
                                                                                        as
                                                                                        libc::c_uint).wrapping_mul(0x20
                                                                                                                       as
                                                                                                                       libc::c_uint))
                            as *mut USB_OTG_HostChannelTypeDef)).HCCHAR &
                         (0x1 as libc::c_uint) << 31 as libc::c_uint ==
                         (0x1 as libc::c_uint) << 31 as libc::c_uint) {
                    break ;
                }
            }
        } else {
            let ref mut fresh98 =
                (*((USBx as
                        uint32_t).wrapping_add(0x500 as
                                                   libc::c_uint).wrapping_add((hc_num
                                                                                   as
                                                                                   libc::c_uint).wrapping_mul(0x20
                                                                                                                  as
                                                                                                                  libc::c_uint))
                       as *mut USB_OTG_HostChannelTypeDef)).HCCHAR;
            ::core::ptr::write_volatile(fresh98,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh98
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 31 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
    }
    return HAL_OK;
}
/* *
  * @brief  Initiate Do Ping protocol
  * @param  USBx : Selected device
  * @param  hc_num : Host Channel number
  *         This parameter can be a value from 1 to 15
  * @retval HAL state
  */
#[no_mangle]
pub unsafe extern "C" fn USB_DoPing(mut USBx: *mut USB_OTG_GlobalTypeDef,
                                    mut ch_num: uint8_t)
 -> HAL_StatusTypeDef {
    let mut num_packets: uint8_t = 1 as libc::c_int as uint8_t;
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    ::core::ptr::write_volatile(&mut (*((USBx as
                                             uint32_t).wrapping_add(0x500 as
                                                                        libc::c_uint).wrapping_add((ch_num
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_mul(0x20
                                                                                                                                       as
                                                                                                                                       libc::c_uint))
                                            as
                                            *mut USB_OTG_HostChannelTypeDef)).HCTSIZ
                                    as *mut uint32_t,
                                ((num_packets as libc::c_int) <<
                                     19 as libc::c_int) as libc::c_uint &
                                    (0x3ff as libc::c_uint) <<
                                        19 as libc::c_uint |
                                    (0x1 as libc::c_uint) <<
                                        31 as libc::c_uint);
    /* Set host channel enable */
    tmpreg =
        (*((USBx as
                uint32_t).wrapping_add(0x500 as
                                           libc::c_uint).wrapping_add((ch_num
                                                                           as
                                                                           libc::c_uint).wrapping_mul(0x20
                                                                                                          as
                                                                                                          libc::c_uint))
               as *mut USB_OTG_HostChannelTypeDef)).HCCHAR;
    tmpreg &= !((0x1 as libc::c_uint) << 30 as libc::c_uint);
    tmpreg |= (0x1 as libc::c_uint) << 31 as libc::c_uint;
    ::core::ptr::write_volatile(&mut (*((USBx as
                                             uint32_t).wrapping_add(0x500 as
                                                                        libc::c_uint).wrapping_add((ch_num
                                                                                                        as
                                                                                                        libc::c_uint).wrapping_mul(0x20
                                                                                                                                       as
                                                                                                                                       libc::c_uint))
                                            as
                                            *mut USB_OTG_HostChannelTypeDef)).HCCHAR
                                    as *mut uint32_t, tmpreg);
    return HAL_OK;
}
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
/* * 
  * @brief  URB States definition  
  */
/* * 
  * @brief  Host channel States  definition  
  */
/* * 
  * @brief  PCD Initialization Structure definition  
  */
/* !< Device Endpoints number.
                                      This parameter depends on the used USB core.   
                                      This parameter must be a number between Min_Data = 1 and Max_Data = 15 */
/* !< Host Channels number.
                                      This parameter Depends on the used USB core.   
                                      This parameter must be a number between Min_Data = 1 and Max_Data = 15 */
/* !< USB Core speed.
                                      This parameter can be any value of @ref USB_Core_Speed_                */
/* !< Enable or disable of the USB embedded DMA.                             */
/* !< Set the Endpoint 0 Max Packet size. 
                                      This parameter can be any value of @ref USB_EP0_MPS_                   */
/* !< Select the used PHY interface.
                                      This parameter can be any value of @ref USB_Core_PHY_                  */
/* !< Enable or disable the output of the SOF signal.                        */
/* !< Enable or disable the low power mode.                                  */
/* !< Enable or disable Link Power Management.                               */
/* !< Enable or disable Battery charging.                                 */
/* !< Enable or disable the VBUS Sensing feature.                            */
/* !< Enable or disable the use of the dedicated EP1 interrupt.              */
/* !< Enable or disable the use of the external VBUS.                        */
/* !< Endpoint number
                                This parameter must be a number between Min_Data = 1 and Max_Data = 15    */
/* !< Endpoint direction
                                This parameter must be a number between Min_Data = 0 and Max_Data = 1     */
/* !< Endpoint stall condition
                                This parameter must be a number between Min_Data = 0 and Max_Data = 1     */
/* !< Endpoint type
                                 This parameter can be any value of @ref USB_EP_Type_                     */
/* !< Initial data PID
                                This parameter must be a number between Min_Data = 0 and Max_Data = 1     */
/* !< IFrame parity
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 1    */
/* !< Transmission FIFO number
                                 This parameter must be a number between Min_Data = 1 and Max_Data = 15   */
/* !< Endpoint Max packet size
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 64KB */
/* !< Pointer to transfer buffer                                               */
/* !< 32 bits aligned transfer buffer address                                  */
/* !< Current transfer length                                                  */
/* !< Partial transfer length in case of multi packet transfer                 */
/* !< USB device address.
                                This parameter must be a number between Min_Data = 1 and Max_Data = 255    */
/* !< Host channel number.
                                This parameter must be a number between Min_Data = 1 and Max_Data = 15     */
/* !< Endpoint number.
                                This parameter must be a number between Min_Data = 1 and Max_Data = 15     */
/* !< Endpoint direction
                                This parameter must be a number between Min_Data = 0 and Max_Data = 1      */
/* !< USB Host speed.
                                This parameter can be any value of @ref USB_Core_Speed_                    */
/* !< Enable or disable the use of the PING protocol for HS mode.                */
/* !< Execute the PING protocol for HS mode.                                     */
/* !< Endpoint Type.
                                This parameter can be any value of @ref USB_EP_Type_                       */
/* !< Endpoint Max packet size.
                                This parameter must be a number between Min_Data = 0 and Max_Data = 64KB   */
/* !< Initial data PID.
                                This parameter must be a number between Min_Data = 0 and Max_Data = 1      */
/* !< Pointer to transfer buffer.                                                */
/* !< Current transfer length.                                                   */
/* !< Partial transfer length in case of multi packet transfer.                  */
/* !< IN transfer current toggle flag.
                                This parameter must be a number between Min_Data = 0 and Max_Data = 1      */
/* !< OUT transfer current toggle flag
                                This parameter must be a number between Min_Data = 0 and Max_Data = 1      */
/* !< 32 bits aligned transfer buffer address.                                   */
/* !< Host channel error count.*/
/* !< URB state. 
                                           This parameter can be any value of @ref USB_OTG_URBStateTypeDef */
/* !< Host Channel state. 
                                           This parameter can be any value of @ref USB_OTG_HCStateTypeDef  */
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
/* *
  * @brief  Stop Host Core
  * @param  USBx : Selected device
  * @retval HAL state
  */
#[no_mangle]
pub unsafe extern "C" fn USB_StopHost(mut USBx: *mut USB_OTG_GlobalTypeDef)
 -> HAL_StatusTypeDef {
    let mut i: uint8_t = 0;
    let mut count: uint32_t = 0 as libc::c_int as uint32_t;
    let mut value: uint32_t = 0;
    USB_DisableGlobalInt(USBx);
    /* Flush FIFO */
    USB_FlushTxFifo(USBx, 0x10 as libc::c_int as uint32_t);
    USB_FlushRxFifo(USBx);
    /* Flush out any leftover queued requests. */
    i = 0 as libc::c_int as uint8_t;
    while i as libc::c_int <= 15 as libc::c_int {
        value =
            (*((USBx as
                    uint32_t).wrapping_add(0x500 as
                                               libc::c_uint).wrapping_add((i
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_HostChannelTypeDef)).HCCHAR;
        value |= (0x1 as libc::c_uint) << 30 as libc::c_uint;
        value &= !((0x1 as libc::c_uint) << 31 as libc::c_uint);
        value &= !((0x1 as libc::c_uint) << 15 as libc::c_uint);
        ::core::ptr::write_volatile(&mut (*((USBx as
                                                 uint32_t).wrapping_add(0x500
                                                                            as
                                                                            libc::c_uint).wrapping_add((i
                                                                                                            as
                                                                                                            libc::c_uint).wrapping_mul(0x20
                                                                                                                                           as
                                                                                                                                           libc::c_uint))
                                                as
                                                *mut USB_OTG_HostChannelTypeDef)).HCCHAR
                                        as *mut uint32_t, value);
        i = i.wrapping_add(1)
    }
    /* Halt all channels to put them into a known state. */
    i = 0 as libc::c_int as uint8_t;
    while i as libc::c_int <= 15 as libc::c_int {
        value =
            (*((USBx as
                    uint32_t).wrapping_add(0x500 as
                                               libc::c_uint).wrapping_add((i
                                                                               as
                                                                               libc::c_uint).wrapping_mul(0x20
                                                                                                              as
                                                                                                              libc::c_uint))
                   as *mut USB_OTG_HostChannelTypeDef)).HCCHAR;
        value |= (0x1 as libc::c_uint) << 30 as libc::c_uint;
        value |= (0x1 as libc::c_uint) << 31 as libc::c_uint;
        value &= !((0x1 as libc::c_uint) << 15 as libc::c_uint);
        ::core::ptr::write_volatile(&mut (*((USBx as
                                                 uint32_t).wrapping_add(0x500
                                                                            as
                                                                            libc::c_uint).wrapping_add((i
                                                                                                            as
                                                                                                            libc::c_uint).wrapping_mul(0x20
                                                                                                                                           as
                                                                                                                                           libc::c_uint))
                                                as
                                                *mut USB_OTG_HostChannelTypeDef)).HCCHAR
                                        as *mut uint32_t, value);
        loop  {
            count = count.wrapping_add(1);
            if count > 1000 as libc::c_int as libc::c_uint { break ; }
            if !((*((USBx as
                         uint32_t).wrapping_add(0x500 as
                                                    libc::c_uint).wrapping_add((i
                                                                                    as
                                                                                    libc::c_uint).wrapping_mul(0x20
                                                                                                                   as
                                                                                                                   libc::c_uint))
                        as *mut USB_OTG_HostChannelTypeDef)).HCCHAR &
                     (0x1 as libc::c_uint) << 31 as libc::c_uint ==
                     (0x1 as libc::c_uint) << 31 as libc::c_uint) {
                break ;
            }
        }
        i = i.wrapping_add(1)
    }
    /* Clear any pending Host interrupts */
    ::core::ptr::write_volatile(&mut (*((USBx as
                                             uint32_t).wrapping_add(0x400 as
                                                                        libc::c_uint)
                                            as
                                            *mut USB_OTG_HostTypeDef)).HAINT
                                    as *mut uint32_t,
                                0xffffffff as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*USBx).GINTSTS as *mut uint32_t,
                                0xffffffff as libc::c_uint);
    USB_EnableGlobalInt(USBx);
    return HAL_OK;
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* defined (HAL_PCD_MODULE_ENABLED) || defined (HAL_HCD_MODULE_ENABLED) */
/* *
  * @}
  */
