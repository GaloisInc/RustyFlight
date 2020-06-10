use ::libc;
extern "C" {
    #[no_mangle]
    static mut blackboxConfig_System: blackboxConfig_t;
    // declare available IO pins. Available pins are specified per target
    #[no_mangle]
    fn IORead(io: IO_t) -> bool;
    // unimplemented
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    static mut statusLedConfig_System: statusLedConfig_t;
    // Helpful macros
    #[no_mangle]
    fn ledInit(statusLedConfig_0: *const statusLedConfig_t);
    #[no_mangle]
    fn delay(ms: timeMs_t);
    #[no_mangle]
    static mut mpuResetFn: mpuResetFnPtr;
    #[no_mangle]
    static mut usbDevConfig_System: usbDev_t;
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
    #[no_mangle]
    fn USBD_Init(pdev: *mut USBD_HandleTypeDef,
                 pdesc: *mut USBD_DescriptorsTypeDef, id: uint8_t)
     -> USBD_StatusTypeDef;
    /* *
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/Inc/usbd_desc.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-April-2016
  * @brief   Header for usbd_desc.c module
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
    /* Define to prevent recursive inclusion -------------------------------------*/
    /* Includes ------------------------------------------------------------------*/
    /* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
    /* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
    #[no_mangle]
    static mut VCP_Desc: USBD_DescriptorsTypeDef;
    #[no_mangle]
    fn USBD_RegisterClass(pdev: *mut USBD_HandleTypeDef,
                          pclass: *mut USBD_ClassTypeDef)
     -> USBD_StatusTypeDef;
    #[no_mangle]
    fn USBD_Start(pdev: *mut USBD_HandleTypeDef) -> USBD_StatusTypeDef;
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
    /* Structure for MSC process */
    #[no_mangle]
    static mut USBD_MSC: USBD_ClassTypeDef;
    #[no_mangle]
    fn USBD_MSC_RegisterStorage(pdev: *mut USBD_HandleTypeDef,
                                fops: *mut USBD_StorageTypeDef) -> uint8_t;
    #[no_mangle]
    static mut USBD_MSC_EMFAT_fops: USBD_StorageTypeDef;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type IRQn_Type = libc::c_int;
pub const SPDIF_RX_IRQn: IRQn_Type = 97;
pub const I2C4_ER_IRQn: IRQn_Type = 96;
pub const I2C4_EV_IRQn: IRQn_Type = 95;
pub const CEC_IRQn: IRQn_Type = 94;
pub const LPTIM1_IRQn: IRQn_Type = 93;
pub const QUADSPI_IRQn: IRQn_Type = 92;
pub const SAI2_IRQn: IRQn_Type = 91;
pub const DMA2D_IRQn: IRQn_Type = 90;
pub const SAI1_IRQn: IRQn_Type = 87;
pub const SPI6_IRQn: IRQn_Type = 86;
pub const SPI5_IRQn: IRQn_Type = 85;
pub const SPI4_IRQn: IRQn_Type = 84;
pub const UART8_IRQn: IRQn_Type = 83;
pub const UART7_IRQn: IRQn_Type = 82;
pub const FPU_IRQn: IRQn_Type = 81;
pub const RNG_IRQn: IRQn_Type = 80;
pub const DCMI_IRQn: IRQn_Type = 78;
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
pub const CAN2_SCE_IRQn: IRQn_Type = 66;
pub const CAN2_RX1_IRQn: IRQn_Type = 65;
pub const CAN2_RX0_IRQn: IRQn_Type = 64;
pub const CAN2_TX_IRQn: IRQn_Type = 63;
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
#[derive(Copy, Clone)]
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
pub type timeMs_t = uint32_t;
pub type BlackboxDevice = libc::c_uint;
pub const BLACKBOX_DEVICE_SERIAL: BlackboxDevice = 3;
pub const BLACKBOX_DEVICE_FLASH: BlackboxDevice = 1;
pub const BLACKBOX_DEVICE_NONE: BlackboxDevice = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct blackboxConfig_s {
    pub p_ratio: uint16_t,
    pub device: uint8_t,
    pub record_acc: uint8_t,
    pub mode: uint8_t,
}
pub type blackboxConfig_t = blackboxConfig_s;
// I-frame interval / P-frame interval
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
pub type resourceOwner_e = libc::c_uint;
pub const OWNER_TOTAL_COUNT: resourceOwner_e = 55;
pub const OWNER_SPI_PREINIT_OPU: resourceOwner_e = 54;
pub const OWNER_SPI_PREINIT_IPU: resourceOwner_e = 53;
pub const OWNER_USB_MSC_PIN: resourceOwner_e = 52;
pub const OWNER_PINIO: resourceOwner_e = 51;
pub const OWNER_RX_SPI: resourceOwner_e = 50;
pub const OWNER_RANGEFINDER: resourceOwner_e = 49;
pub const OWNER_TIMUP: resourceOwner_e = 48;
pub const OWNER_CAMERA_CONTROL: resourceOwner_e = 47;
pub const OWNER_ESCSERIAL: resourceOwner_e = 46;
pub const OWNER_RX_BIND_PLUG: resourceOwner_e = 45;
pub const OWNER_COMPASS_CS: resourceOwner_e = 44;
pub const OWNER_VTX: resourceOwner_e = 43;
pub const OWNER_TRANSPONDER: resourceOwner_e = 42;
pub const OWNER_LED_STRIP: resourceOwner_e = 41;
pub const OWNER_INVERTER: resourceOwner_e = 40;
pub const OWNER_RX_BIND: resourceOwner_e = 39;
pub const OWNER_OSD: resourceOwner_e = 38;
pub const OWNER_BEEPER: resourceOwner_e = 37;
pub const OWNER_USB_DETECT: resourceOwner_e = 36;
pub const OWNER_USB: resourceOwner_e = 35;
pub const OWNER_COMPASS_EXTI: resourceOwner_e = 34;
pub const OWNER_BARO_EXTI: resourceOwner_e = 33;
pub const OWNER_MPU_EXTI: resourceOwner_e = 32;
pub const OWNER_SPI_CS: resourceOwner_e = 31;
pub const OWNER_RX_SPI_CS: resourceOwner_e = 30;
pub const OWNER_OSD_CS: resourceOwner_e = 29;
pub const OWNER_MPU_CS: resourceOwner_e = 28;
pub const OWNER_BARO_CS: resourceOwner_e = 27;
pub const OWNER_FLASH_CS: resourceOwner_e = 26;
pub const OWNER_SDCARD_DETECT: resourceOwner_e = 25;
pub const OWNER_SDCARD_CS: resourceOwner_e = 24;
pub const OWNER_SDCARD: resourceOwner_e = 23;
pub const OWNER_I2C_SDA: resourceOwner_e = 22;
pub const OWNER_I2C_SCL: resourceOwner_e = 21;
pub const OWNER_SPI_MOSI: resourceOwner_e = 20;
pub const OWNER_SPI_MISO: resourceOwner_e = 19;
pub const OWNER_SPI_SCK: resourceOwner_e = 18;
pub const OWNER_SYSTEM: resourceOwner_e = 17;
pub const OWNER_SONAR_ECHO: resourceOwner_e = 16;
pub const OWNER_SONAR_TRIGGER: resourceOwner_e = 15;
pub const OWNER_TIMER: resourceOwner_e = 14;
pub const OWNER_PINDEBUG: resourceOwner_e = 13;
pub const OWNER_SERIAL_RX: resourceOwner_e = 12;
pub const OWNER_SERIAL_TX: resourceOwner_e = 11;
pub const OWNER_ADC_RSSI: resourceOwner_e = 10;
pub const OWNER_ADC_EXT: resourceOwner_e = 9;
pub const OWNER_ADC_CURR: resourceOwner_e = 8;
pub const OWNER_ADC_BATT: resourceOwner_e = 7;
pub const OWNER_ADC: resourceOwner_e = 6;
pub const OWNER_LED: resourceOwner_e = 5;
pub const OWNER_SERVO: resourceOwner_e = 4;
pub const OWNER_MOTOR: resourceOwner_e = 3;
pub const OWNER_PPMINPUT: resourceOwner_e = 2;
pub const OWNER_PWMINPUT: resourceOwner_e = 1;
pub const OWNER_FREE: resourceOwner_e = 0;
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
// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
pub type ioTag_t = uint8_t;
// packet tag to specify IO pin
pub type IO_t = *mut libc::c_void;
// type specifying IO pin. Currently ioRec_t pointer, but this may change
// NONE initializer for ioTag_t variables
// NONE initializer for IO_t variable
// both ioTag_t and IO_t are guarantied to be zero if pinid is NONE (no pin)
// this simplifies initialization (globals are zeroed on start) and allows
//  omitting unused fields in structure initializers.
// it is also possible to use IO_t and ioTag_t as boolean value
//   TODO - this may conflict with requirement to generate warning/error on IO_t - ioTag_t assignment
//   IO_t being pointer is only possibility I know of ..
// pin config handling
// pin config is packed into ioConfig_t to decrease memory requirements
// IOCFG_x macros are defined for common combinations for all CPUs; this
//  helps masking CPU differences
pub type ioConfig_t = uint8_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct statusLedConfig_s {
    pub ioTags: [ioTag_t; 3],
    pub inversion: uint8_t,
}
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
pub type statusLedConfig_t = statusLedConfig_s;
pub type usbDev_t = usbDev_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct usbDev_s {
    pub type_0: uint8_t,
    pub mscButtonPin: ioTag_t,
    pub mscButtonUsePullup: uint8_t,
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
/* This is stupid, any nice solution to handle multiple interfaces
   * would be much apriciated. Or at least a flow how this should be rewritten instead.
   */
/* *
  ******************************************************************************
  * @file    usbd_msc.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   Header for the usbd_msc.c file
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
/* * @addtogroup USBD_MSC_BOT
  * @{
  */
/* * @defgroup USBD_MSC
  * @brief This file is the Header file for usbd_msc.c
  * @{
  */
/* * @defgroup USBD_BOT_Exported_Defines
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USB_CORE_Exported_Types
  * @{
  */
pub type USBD_StorageTypeDef = _USBD_STORAGE;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct _USBD_STORAGE {
    pub Init: Option<unsafe extern "C" fn(_: uint8_t) -> int8_t>,
    pub GetCapacity: Option<unsafe extern "C" fn(_: uint8_t, _: *mut uint32_t,
                                                 _: *mut uint16_t) -> int8_t>,
    pub IsReady: Option<unsafe extern "C" fn(_: uint8_t) -> int8_t>,
    pub IsWriteProtected: Option<unsafe extern "C" fn(_: uint8_t) -> int8_t>,
    pub Read: Option<unsafe extern "C" fn(_: uint8_t, _: *mut uint8_t,
                                          _: uint32_t, _: uint16_t)
                         -> int8_t>,
    pub Write: Option<unsafe extern "C" fn(_: uint8_t, _: *mut uint8_t,
                                           _: uint32_t, _: uint16_t)
                          -> int8_t>,
    pub GetMaxLun: Option<unsafe extern "C" fn() -> int8_t>,
    pub pInquiry: *mut int8_t,
}
pub type mpuResetFnPtr = Option<unsafe extern "C" fn() -> ()>;
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
unsafe extern "C" fn __NVIC_SetPriority(mut IRQn: IRQn_Type,
                                        mut priority: uint32_t) {
    if IRQn as int32_t >= 0 as libc::c_int {
        ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                                 libc::c_ulong).wrapping_add(0x100
                                                                                 as
                                                                                 libc::c_ulong)
                                                as
                                                *mut NVIC_Type)).IP[IRQn as
                                                                        uint32_t
                                                                        as
                                                                        usize]
                                        as *mut uint8_t,
                                    (priority <<
                                         (8 as
                                              libc::c_uint).wrapping_sub(4 as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint)
                                         & 0xff as libc::c_ulong as uint32_t)
                                        as uint8_t)
    } else {
        ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                                 libc::c_ulong).wrapping_add(0xd00
                                                                                 as
                                                                                 libc::c_ulong)
                                                as
                                                *mut SCB_Type)).SHPR[(IRQn as
                                                                          uint32_t
                                                                          as
                                                                          libc::c_ulong
                                                                          &
                                                                          0xf
                                                                              as
                                                                              libc::c_ulong).wrapping_sub(4
                                                                                                              as
                                                                                                              libc::c_ulong)
                                                                         as
                                                                         usize]
                                        as *mut uint8_t,
                                    (priority <<
                                         (8 as
                                              libc::c_uint).wrapping_sub(4 as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint)
                                         & 0xff as libc::c_ulong as uint32_t)
                                        as uint8_t)
    };
}
#[inline]
unsafe extern "C" fn __NVIC_DisableIRQ(mut IRQn: IRQn_Type) {
    if IRQn as int32_t >= 0 as libc::c_int {
        ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                                 libc::c_ulong).wrapping_add(0x100
                                                                                 as
                                                                                 libc::c_ulong)
                                                as
                                                *mut NVIC_Type)).ICER[(IRQn as
                                                                           uint32_t
                                                                           >>
                                                                           5
                                                                               as
                                                                               libc::c_ulong)
                                                                          as
                                                                          usize]
                                        as *mut uint32_t,
                                    ((1 as libc::c_ulong) <<
                                         (IRQn as uint32_t as libc::c_ulong &
                                              0x1f as libc::c_ulong)) as
                                        uint32_t);
        __DSB();
        __ISB();
    };
}
#[inline]
unsafe extern "C" fn __NVIC_EnableIRQ(mut IRQn: IRQn_Type) {
    if IRQn as int32_t >= 0 as libc::c_int {
        ::core::ptr::write_volatile(&mut (*((0xe000e000 as
                                                 libc::c_ulong).wrapping_add(0x100
                                                                                 as
                                                                                 libc::c_ulong)
                                                as
                                                *mut NVIC_Type)).ISER[(IRQn as
                                                                           uint32_t
                                                                           >>
                                                                           5
                                                                               as
                                                                               libc::c_ulong)
                                                                          as
                                                                          usize]
                                        as *mut uint32_t,
                                    ((1 as libc::c_ulong) <<
                                         (IRQn as uint32_t as libc::c_ulong &
                                              0x1f as libc::c_ulong)) as
                                        uint32_t)
    };
}
#[inline(always)]
unsafe extern "C" fn __DSB() { asm!("dsb 0xF" : : : "memory" : "volatile"); }
#[inline(always)]
unsafe extern "C" fn __ISB() { asm!("isb 0xF" : : : "memory" : "volatile"); }
#[inline(always)]
unsafe extern "C" fn __disable_irq() {
    asm!("cpsid i" : : : "memory" : "volatile");
}
#[inline]
unsafe extern "C" fn blackboxConfig() -> *const blackboxConfig_t {
    return &mut blackboxConfig_System;
}
#[inline]
unsafe extern "C" fn statusLedConfig() -> *const statusLedConfig_t {
    return &mut statusLedConfig_System;
}
#[inline]
unsafe extern "C" fn usbDevConfig() -> *const usbDev_t {
    return &mut usbDevConfig_System;
}
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
/*
 * Author: Chris Hockuba (https://github.com/conkerkh)
 *
 */
#[no_mangle]
pub static mut USBD_Device: USBD_HandleTypeDef =
    USBD_HandleTypeDef{id: 0,
                       dev_config: 0,
                       dev_default_config: 0,
                       dev_config_status: 0,
                       dev_speed: USBD_SPEED_HIGH,
                       ep_in:
                           [USBD_EndpointTypeDef{status: 0,
                                                 total_length: 0,
                                                 rem_length: 0,
                                                 maxpacket: 0,}; 15],
                       ep_out:
                           [USBD_EndpointTypeDef{status: 0,
                                                 total_length: 0,
                                                 rem_length: 0,
                                                 maxpacket: 0,}; 15],
                       ep0_state: 0,
                       ep0_data_len: 0,
                       dev_state: 0,
                       dev_old_state: 0,
                       dev_address: 0,
                       dev_connection_status: 0,
                       dev_test_mode: 0,
                       dev_remote_wakeup: 0,
                       request:
                           USBD_SetupReqTypedef{bmRequest: 0,
                                                bRequest: 0,
                                                wValue: 0,
                                                wIndex: 0,
                                                wLength: 0,},
                       pDesc:
                           0 as *const USBD_DescriptorsTypeDef as
                               *mut USBD_DescriptorsTypeDef,
                       pClass:
                           0 as *const USBD_ClassTypeDef as
                               *mut USBD_ClassTypeDef,
                       pCDC_ClassData:
                           0 as *const libc::c_void as *mut libc::c_void,
                       pCDC_UserData:
                           0 as *const libc::c_void as *mut libc::c_void,
                       pHID_ClassData:
                           0 as *const libc::c_void as *mut libc::c_void,
                       pHID_UserData:
                           0 as *const libc::c_void as *mut libc::c_void,
                       pMSC_ClassData:
                           0 as *const libc::c_void as *mut libc::c_void,
                       pMSC_UserData:
                           0 as *const libc::c_void as *mut libc::c_void,
                       pData: 0 as *const libc::c_void as *mut libc::c_void,};
static mut mscButton: IO_t = 0 as *const libc::c_void as *mut libc::c_void;
#[no_mangle]
pub unsafe extern "C" fn mscInit() {
    if (*usbDevConfig()).mscButtonPin != 0 {
        mscButton = IOGetByTag((*usbDevConfig()).mscButtonPin);
        IOInit(mscButton, OWNER_USB_MSC_PIN, 0 as libc::c_int as uint8_t);
        if (*usbDevConfig()).mscButtonUsePullup != 0 {
            IOConfigGPIO(mscButton,
                         (0 as libc::c_uint |
                              (0 as libc::c_uint) << 2 as libc::c_int |
                              (0x1 as libc::c_uint) << 5 as libc::c_int) as
                             ioConfig_t);
        } else {
            IOConfigGPIO(mscButton,
                         (0 as libc::c_uint |
                              (0 as libc::c_uint) << 2 as libc::c_int |
                              (0x2 as libc::c_uint) << 5 as libc::c_int) as
                             ioConfig_t);
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn mscStart() -> uint8_t {
    ledInit(statusLedConfig());
    //Start USB
    usbGenerateDisconnectPulse();
    IOInit(IOGetByTag(((0 as libc::c_int + 1 as libc::c_int) <<
                           4 as libc::c_int | 11 as libc::c_int) as ioTag_t),
           OWNER_USB, 0 as libc::c_int as uint8_t);
    IOInit(IOGetByTag(((0 as libc::c_int + 1 as libc::c_int) <<
                           4 as libc::c_int | 12 as libc::c_int) as ioTag_t),
           OWNER_USB, 0 as libc::c_int as uint8_t);
    USBD_Init(&mut USBD_Device, &mut VCP_Desc, 0 as libc::c_int as uint8_t);
    /* * Regsiter class */
    USBD_RegisterClass(&mut USBD_Device, &mut USBD_MSC);
    /* * Register interface callbacks */
    match (*blackboxConfig()).device as libc::c_int {
        1 => {
            USBD_MSC_RegisterStorage(&mut USBD_Device,
                                     &mut USBD_MSC_EMFAT_fops);
        }
        _ => { return 1 as libc::c_int as uint8_t }
    }
    USBD_Start(&mut USBD_Device);
    // NVIC configuration for SYSTick
    __NVIC_DisableIRQ(SysTick_IRQn);
    __NVIC_SetPriority(SysTick_IRQn,
                       (((0 as libc::c_int) <<
                             (4 as libc::c_int as
                                  libc::c_uint).wrapping_sub((7 as libc::c_int
                                                                  as
                                                                  libc::c_uint).wrapping_sub(0x5
                                                                                                 as
                                                                                                 libc::c_uint))
                             |
                             0 as libc::c_int &
                                 0xf as libc::c_int >>
                                     (7 as libc::c_int as
                                          libc::c_uint).wrapping_sub(0x5 as
                                                                         libc::c_uint))
                            << 4 as libc::c_int & 0xf0 as libc::c_int) as
                           uint32_t);
    __NVIC_EnableIRQ(SysTick_IRQn);
    return 0 as libc::c_int as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn mscCheckBoot() -> bool {
    if *(0x40024000 as libc::c_uint as
             *mut uint32_t).offset(16 as libc::c_int as isize) ==
           0xdddd1010 as libc::c_uint {
        return 1 as libc::c_int != 0
    }
    return 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn mscCheckButton() -> bool {
    let mut result: bool = 0 as libc::c_int != 0;
    if !mscButton.is_null() {
        let mut state: uint8_t = IORead(mscButton) as uint8_t;
        if (*usbDevConfig()).mscButtonUsePullup != 0 {
            result = state as libc::c_int == 0 as libc::c_int
        } else { result = state as libc::c_int == 1 as libc::c_int }
    }
    return result;
}
#[no_mangle]
pub unsafe extern "C" fn mscWaitForButton() {
    // In order to exit MSC mode simply disconnect the board, or push the button again.
    while mscCheckButton() { }
    delay(20 as libc::c_int as timeMs_t);
    loop  {
        asm!("NOP" : : :);
        if mscCheckButton() {
            *(0x2001fff0 as libc::c_int as *mut uint32_t) =
                0xffffffff as libc::c_uint;
            delay(1 as libc::c_int as timeMs_t);
            __NVIC_SystemReset();
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn systemResetToMsc() {
    if mpuResetFn.is_some() {
        mpuResetFn.expect("non-null function pointer")();
    }
    ::core::ptr::write_volatile((0x40024000 as libc::c_uint as
                                     *mut uint32_t).offset(16 as libc::c_int
                                                               as isize),
                                0xdddd1010 as libc::c_uint);
    __disable_irq();
    __NVIC_SystemReset();
}
