use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn ADC_Init(ADCx: *mut ADC_TypeDef, ADC_InitStruct: *mut ADC_InitTypeDef);
    #[no_mangle]
    fn ADC_StructInit(ADC_InitStruct: *mut ADC_InitTypeDef);
    #[no_mangle]
    fn ADC_Cmd(ADCx: *mut ADC_TypeDef, NewState: FunctionalState);
    #[no_mangle]
    fn ADC_DMACmd(ADCx: *mut ADC_TypeDef, NewState: FunctionalState);
    #[no_mangle]
    fn ADC_ResetCalibration(ADCx: *mut ADC_TypeDef);
    #[no_mangle]
    fn ADC_GetResetCalibrationStatus(ADCx: *mut ADC_TypeDef) -> FlagStatus;
    #[no_mangle]
    fn ADC_StartCalibration(ADCx: *mut ADC_TypeDef);
    #[no_mangle]
    fn ADC_GetCalibrationStatus(ADCx: *mut ADC_TypeDef) -> FlagStatus;
    #[no_mangle]
    fn ADC_SoftwareStartConvCmd(ADCx: *mut ADC_TypeDef,
                                NewState: FunctionalState);
    #[no_mangle]
    fn ADC_RegularChannelConfig(ADCx: *mut ADC_TypeDef, ADC_Channel: uint8_t,
                                Rank: uint8_t, ADC_SampleTime: uint8_t);
    /* *
  * @}
  */
    /* * @defgroup DMA_Exported_Constants
  * @{
  */
    /* * @defgroup DMA_data_transfer_direction 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup DMA_peripheral_incremented_mode 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup DMA_memory_incremented_mode 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup DMA_peripheral_data_size 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup DMA_memory_data_size 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup DMA_circular_normal_mode 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup DMA_priority_level 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup DMA_memory_to_memory 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup DMA_interrupts_definition 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup DMA_flags_definition 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup DMA_Buffer_Size 
  * @{
  */
    /* *
  * @}
  */
    /* *
  * @}
  */
    /* * @defgroup DMA_Exported_Macros
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup DMA_Exported_Functions
  * @{
  */
    #[no_mangle]
    fn DMA_DeInit(DMAy_Channelx: *mut DMA_Channel_TypeDef);
    #[no_mangle]
    fn DMA_Init(DMAy_Channelx: *mut DMA_Channel_TypeDef,
                DMA_InitStruct: *mut DMA_InitTypeDef);
    #[no_mangle]
    fn DMA_StructInit(DMA_InitStruct: *mut DMA_InitTypeDef);
    #[no_mangle]
    fn DMA_Cmd(DMAy_Channelx: *mut DMA_Channel_TypeDef,
               NewState: FunctionalState);
    /* STM32F10X_CL */
    #[no_mangle]
    fn RCC_ADCCLKConfig(RCC_PCLK2: uint32_t);
    #[no_mangle]
    fn adcDeviceByInstance(instance: *mut ADC_TypeDef) -> ADCDevice;
    #[no_mangle]
    static mut adcOperatingConfig: [adcOperatingConfig_t; 4];
    #[no_mangle]
    static mut adcValues: [uint16_t; 4];
    #[no_mangle]
    fn adcChannelByTag(ioTag: ioTag_t) -> uint8_t;
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn RCC_ClockCmd(periphTag: rccPeriphTag_t, NewState: FunctionalState);
    #[no_mangle]
    fn dmaGetIdentifier(channel: *const DMA_Channel_TypeDef)
     -> dmaIdentifier_e;
    #[no_mangle]
    fn dmaInit(identifier: dmaIdentifier_e, owner: resourceOwner_e,
               resourceIndex: uint8_t);
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* !< Read Only */
pub type FlagStatus = libc::c_uint;
pub const SET: FlagStatus = 1;
pub const RESET: FlagStatus = 0;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
/* !< STM32F10x Standard Peripheral Library old definitions (maintained for legacy purpose) */
/* *
  * @}
  */
/* * @addtogroup Peripheral_registers_structures
  * @{
  */
/* * 
  * @brief Analog to Digital Converter  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ADC_TypeDef {
    pub SR: uint32_t,
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub SMPR1: uint32_t,
    pub SMPR2: uint32_t,
    pub JOFR1: uint32_t,
    pub JOFR2: uint32_t,
    pub JOFR3: uint32_t,
    pub JOFR4: uint32_t,
    pub HTR: uint32_t,
    pub LTR: uint32_t,
    pub SQR1: uint32_t,
    pub SQR2: uint32_t,
    pub SQR3: uint32_t,
    pub JSQR: uint32_t,
    pub JDR1: uint32_t,
    pub JDR2: uint32_t,
    pub JDR3: uint32_t,
    pub JDR4: uint32_t,
    pub DR: uint32_t,
}
/* * 
  * @brief DMA Controller
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_Channel_TypeDef {
    pub CCR: uint32_t,
    pub CNDTR: uint32_t,
    pub CPAR: uint32_t,
    pub CMAR: uint32_t,
}
/* *
  ******************************************************************************
  * @file    stm32f10x_adc.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the ADC firmware 
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
/* * @addtogroup ADC
  * @{
  */
/* * @defgroup ADC_Exported_Types
  * @{
  */
/* * 
  * @brief  ADC Init structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ADC_InitTypeDef {
    pub ADC_Mode: uint32_t,
    pub ADC_ScanConvMode: FunctionalState,
    pub ADC_ContinuousConvMode: FunctionalState,
    pub ADC_ExternalTrigConv: uint32_t,
    pub ADC_DataAlign: uint32_t,
    pub ADC_NbrOfChannel: uint8_t,
}
/* *
  ******************************************************************************
  * @file    stm32f10x_dma.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the DMA firmware 
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
/* * @addtogroup DMA
  * @{
  */
/* * @defgroup DMA_Exported_Types
  * @{
  */
/* * 
  * @brief  DMA Init structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_InitTypeDef {
    pub DMA_PeripheralBaseAddr: uint32_t,
    pub DMA_MemoryBaseAddr: uint32_t,
    pub DMA_DIR: uint32_t,
    pub DMA_BufferSize: uint32_t,
    pub DMA_PeripheralInc: uint32_t,
    pub DMA_MemoryInc: uint32_t,
    pub DMA_PeripheralDataSize: uint32_t,
    pub DMA_MemoryDataSize: uint32_t,
    pub DMA_Mode: uint32_t,
    pub DMA_Priority: uint32_t,
    pub DMA_M2M: uint32_t,
}
/* * 
  * @brief  Configuration Mode enumeration  
  */
pub type C2RustUnnamed = libc::c_uint;
pub const GPIO_Mode_AF_PP: C2RustUnnamed = 24;
pub const GPIO_Mode_AF_OD: C2RustUnnamed = 28;
pub const GPIO_Mode_Out_PP: C2RustUnnamed = 16;
pub const GPIO_Mode_Out_OD: C2RustUnnamed = 20;
pub const GPIO_Mode_IPU: C2RustUnnamed = 72;
pub const GPIO_Mode_IPD: C2RustUnnamed = 40;
pub const GPIO_Mode_IN_FLOATING: C2RustUnnamed = 4;
pub const GPIO_Mode_AIN: C2RustUnnamed = 0;
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
pub type rccPeriphTag_t = uint8_t;
pub type ADCDevice = libc::c_int;
pub const ADCDEV_COUNT: ADCDevice = 1;
pub const ADCDEV_1: ADCDevice = 0;
pub const ADCINVALID: ADCDevice = -1;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const ADC_CHANNEL_COUNT: C2RustUnnamed_0 = 4;
pub const ADC_RSSI: C2RustUnnamed_0 = 3;
pub const ADC_EXTERNAL1: C2RustUnnamed_0 = 2;
pub const ADC_CURRENT: C2RustUnnamed_0 = 1;
pub const ADC_BATTERY: C2RustUnnamed_0 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct adcOperatingConfig_s {
    pub tag: ioTag_t,
    pub adcChannel: uint8_t,
    pub dmaIndex: uint8_t,
    pub enabled: bool,
    pub sampleTime: uint8_t,
}
pub type adcOperatingConfig_t = adcOperatingConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct adcConfig_s {
    pub vbat: adcChannelConfig_t,
    pub rssi: adcChannelConfig_t,
    pub current: adcChannelConfig_t,
    pub external1: adcChannelConfig_t,
    pub device: int8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct adcChannelConfig_t {
    pub enabled: bool,
    pub ioTag: ioTag_t,
}
pub type adcConfig_t = adcConfig_s;
// ADC1_INxx channel number
// index into DMA buffer in case of sparse channels
// ADCDevice
// Encoding for adcTagMap_t.devices
pub type adcDevice_t = adcDevice_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct adcDevice_s {
    pub ADCx: *mut ADC_TypeDef,
    pub rccADC: rccPeriphTag_t,
    pub DMAy_Channelx: *mut DMA_Channel_TypeDef,
}
pub const RCC_APB2: rcc_reg = 2;
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
pub type dmaIdentifier_e = libc::c_uint;
pub const DMA_LAST_HANDLER: dmaIdentifier_e = 7;
pub const DMA1_CH7_HANDLER: dmaIdentifier_e = 7;
pub const DMA1_CH6_HANDLER: dmaIdentifier_e = 6;
pub const DMA1_CH5_HANDLER: dmaIdentifier_e = 5;
pub const DMA1_CH4_HANDLER: dmaIdentifier_e = 4;
pub const DMA1_CH3_HANDLER: dmaIdentifier_e = 3;
pub const DMA1_CH2_HANDLER: dmaIdentifier_e = 2;
pub const DMA1_CH1_HANDLER: dmaIdentifier_e = 1;
pub const DMA_NONE: dmaIdentifier_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct adcTagMap_s {
    pub tag: ioTag_t,
    pub channel: uint8_t,
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
pub type adcTagMap_t = adcTagMap_s;
// F1 pins have uniform connection to ADC instances
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
pub type rcc_reg = libc::c_uint;
pub const RCC_AHB1: rcc_reg = 4;
pub const RCC_APB1: rcc_reg = 3;
// make sure that default value (0) does not enable anything
pub const RCC_AHB: rcc_reg = 1;
pub const RCC_EMPTY: rcc_reg = 0;
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
// Initialized in run_static_initializers
#[no_mangle]
pub static mut adcHardware: [adcDevice_t; 1] =
    [adcDevice_t{ADCx: 0 as *mut ADC_TypeDef,
                 rccADC: 0,
                 DMAy_Channelx: 0 as *mut DMA_Channel_TypeDef,}; 1];
#[no_mangle]
pub static mut adcTagMap: [adcTagMap_t; 10] =
    [{
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 0 as libc::c_int) as
                                 ioTag_t,
                         channel: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 1 as libc::c_int) as
                                 ioTag_t,
                         channel: 0x1 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 2 as libc::c_int) as
                                 ioTag_t,
                         channel: 0x2 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 3 as libc::c_int) as
                                 ioTag_t,
                         channel: 0x3 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 4 as libc::c_int) as
                                 ioTag_t,
                         channel: 0x4 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 5 as libc::c_int) as
                                 ioTag_t,
                         channel: 0x5 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 6 as libc::c_int) as
                                 ioTag_t,
                         channel: 0x6 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 7 as libc::c_int) as
                                 ioTag_t,
                         channel: 0x7 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((1 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 0 as libc::c_int) as
                                 ioTag_t,
                         channel: 0x8 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((1 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 1 as libc::c_int) as
                                 ioTag_t,
                         channel: 0x9 as libc::c_int as uint8_t,};
         init
     }];
// Driver for STM32F103CB onboard ADC
//
// Naze32
// Battery Voltage (VBAT) is connected to PA4 (ADC1_IN4) with 10k:1k divider
// RSSI ADC uses CH2 (PA1, ADC1_IN1)
// Current ADC uses CH8 (PB1, ADC1_IN9)
//
// NAZE rev.5 hardware has PA5 (ADC1_IN5) on breakout pad on bottom of board
//
#[no_mangle]
pub unsafe extern "C" fn adcInit(mut config: *const adcConfig_t) {
    let mut configuredAdcChannels: uint8_t = 0 as libc::c_int as uint8_t;
    memset(&mut adcOperatingConfig as *mut [adcOperatingConfig_t; 4] as
               *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<[adcOperatingConfig_t; 4]>() as
               libc::c_ulong);
    if (*config).vbat.enabled {
        adcOperatingConfig[ADC_BATTERY as libc::c_int as usize].tag =
            (*config).vbat.ioTag
    }
    if (*config).rssi.enabled {
        adcOperatingConfig[ADC_RSSI as libc::c_int as usize].tag =
            (*config).rssi.ioTag
        //RSSI_ADC_CHANNEL;
    }
    if (*config).external1.enabled {
        adcOperatingConfig[ADC_EXTERNAL1 as libc::c_int as usize].tag =
            (*config).external1.ioTag
        //EXTERNAL1_ADC_CHANNEL;
    }
    if (*config).current.enabled {
        adcOperatingConfig[ADC_CURRENT as libc::c_int as usize].tag =
            (*config).current.ioTag
        //CURRENT_METER_ADC_CHANNEL;
    } // 9MHz from 72MHz APB2 clock(HSE), 8MHz from 64MHz (HSI)
    let mut device: ADCDevice =
        adcDeviceByInstance((0x40000000 as libc::c_int as
                                 uint32_t).wrapping_add(0x10000 as libc::c_int
                                                            as
                                                            libc::c_uint).wrapping_add(0x2400
                                                                                           as
                                                                                           libc::c_int
                                                                                           as
                                                                                           libc::c_uint)
                                as *mut ADC_TypeDef);
    if device as libc::c_int == ADCINVALID as libc::c_int { return }
    let adc: adcDevice_t = adcHardware[device as usize];
    let mut adcActive: bool = 0 as libc::c_int != 0;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < ADC_CHANNEL_COUNT as libc::c_int {
        if !(adcOperatingConfig[i as usize].tag == 0) {
            adcActive = 1 as libc::c_int != 0;
            IOInit(IOGetByTag(adcOperatingConfig[i as usize].tag),
                   (OWNER_ADC_BATT as libc::c_int + i) as resourceOwner_e,
                   0 as libc::c_int as uint8_t);
            IOConfigGPIO(IOGetByTag(adcOperatingConfig[i as usize].tag),
                         (GPIO_Mode_AIN as libc::c_int | 0 as libc::c_int) as
                             ioConfig_t);
            adcOperatingConfig[i as usize].adcChannel =
                adcChannelByTag(adcOperatingConfig[i as usize].tag);
            let fresh0 = configuredAdcChannels;
            configuredAdcChannels = configuredAdcChannels.wrapping_add(1);
            adcOperatingConfig[i as usize].dmaIndex = fresh0;
            adcOperatingConfig[i as usize].sampleTime =
                0x7 as libc::c_int as uint8_t;
            adcOperatingConfig[i as usize].enabled = 1 as libc::c_int != 0
        }
        i += 1
    }
    if !adcActive { return }
    RCC_ADCCLKConfig(0xc000 as libc::c_int as uint32_t);
    RCC_ClockCmd(adc.rccADC, ENABLE);
    dmaInit(dmaGetIdentifier(adc.DMAy_Channelx), OWNER_ADC,
            0 as libc::c_int as uint8_t);
    DMA_DeInit(adc.DMAy_Channelx);
    let mut DMA_InitStructure: DMA_InitTypeDef =
        DMA_InitTypeDef{DMA_PeripheralBaseAddr: 0,
                        DMA_MemoryBaseAddr: 0,
                        DMA_DIR: 0,
                        DMA_BufferSize: 0,
                        DMA_PeripheralInc: 0,
                        DMA_MemoryInc: 0,
                        DMA_PeripheralDataSize: 0,
                        DMA_MemoryDataSize: 0,
                        DMA_Mode: 0,
                        DMA_Priority: 0,
                        DMA_M2M: 0,};
    DMA_StructInit(&mut DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr =
        &mut (*adc.ADCx).DR as *mut uint32_t as uint32_t;
    DMA_InitStructure.DMA_MemoryBaseAddr = adcValues.as_mut_ptr() as uint32_t;
    DMA_InitStructure.DMA_DIR = 0 as libc::c_int as uint32_t;
    DMA_InitStructure.DMA_BufferSize = configuredAdcChannels as uint32_t;
    DMA_InitStructure.DMA_PeripheralInc = 0 as libc::c_int as uint32_t;
    DMA_InitStructure.DMA_MemoryInc =
        if configuredAdcChannels as libc::c_int > 1 as libc::c_int {
            0x80 as libc::c_int as uint32_t
        } else { 0 as libc::c_int as uint32_t };
    DMA_InitStructure.DMA_PeripheralDataSize =
        0x100 as libc::c_int as uint32_t;
    DMA_InitStructure.DMA_MemoryDataSize = 0x400 as libc::c_int as uint32_t;
    DMA_InitStructure.DMA_Mode = 0x20 as libc::c_int as uint32_t;
    DMA_InitStructure.DMA_Priority = 0x2000 as libc::c_int as uint32_t;
    DMA_InitStructure.DMA_M2M = 0 as libc::c_int as uint32_t;
    DMA_Init(adc.DMAy_Channelx, &mut DMA_InitStructure);
    DMA_Cmd(adc.DMAy_Channelx, ENABLE);
    let mut ADC_InitStructure: ADC_InitTypeDef =
        ADC_InitTypeDef{ADC_Mode: 0,
                        ADC_ScanConvMode: DISABLE,
                        ADC_ContinuousConvMode: DISABLE,
                        ADC_ExternalTrigConv: 0,
                        ADC_DataAlign: 0,
                        ADC_NbrOfChannel: 0,};
    ADC_StructInit(&mut ADC_InitStructure);
    ADC_InitStructure.ADC_Mode = 0 as libc::c_int as uint32_t;
    ADC_InitStructure.ADC_ScanConvMode =
        if configuredAdcChannels as libc::c_int > 1 as libc::c_int {
            ENABLE as libc::c_int
        } else { DISABLE as libc::c_int } as FunctionalState;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv =
        0xe0000 as libc::c_int as uint32_t;
    ADC_InitStructure.ADC_DataAlign = 0 as libc::c_int as uint32_t;
    ADC_InitStructure.ADC_NbrOfChannel = configuredAdcChannels;
    ADC_Init(adc.ADCx, &mut ADC_InitStructure);
    let mut rank: uint8_t = 1 as libc::c_int as uint8_t;
    let mut i_0: libc::c_int = 0 as libc::c_int;
    while i_0 < ADC_CHANNEL_COUNT as libc::c_int {
        if adcOperatingConfig[i_0 as usize].enabled {
            let fresh1 = rank;
            rank = rank.wrapping_add(1);
            ADC_RegularChannelConfig(adc.ADCx,
                                     adcOperatingConfig[i_0 as
                                                            usize].adcChannel,
                                     fresh1,
                                     adcOperatingConfig[i_0 as
                                                            usize].sampleTime);
        }
        i_0 += 1
    }
    ADC_DMACmd(adc.ADCx, ENABLE);
    ADC_Cmd(adc.ADCx, ENABLE);
    ADC_ResetCalibration(adc.ADCx);
    while ADC_GetResetCalibrationStatus(adc.ADCx) as u64 != 0 { }
    ADC_StartCalibration(adc.ADCx);
    while ADC_GetCalibrationStatus(adc.ADCx) as u64 != 0 { }
    ADC_SoftwareStartConvCmd(adc.ADCx, ENABLE);
}
unsafe extern "C" fn run_static_initializers() {
    adcHardware =
        [{
             let mut init =
                 adcDevice_s{ADCx:
                                 (0x40000000 as libc::c_int as
                                      uint32_t).wrapping_add(0x10000 as
                                                                 libc::c_int
                                                                 as
                                                                 libc::c_uint).wrapping_add(0x2400
                                                                                                as
                                                                                                libc::c_int
                                                                                                as
                                                                                                libc::c_uint)
                                     as *mut ADC_TypeDef,
                             rccADC:
                                 (((RCC_APB2 as libc::c_int) <<
                                       5 as libc::c_int) as libc::c_long |
                                      (16 as libc::c_int *
                                           (0x200 as libc::c_int as uint32_t
                                                as libc::c_long >
                                                65535 as libc::c_long) as
                                               libc::c_int) as libc::c_long +
                                          ((8 as libc::c_int *
                                                (0x200 as libc::c_int as
                                                     uint32_t as libc::c_long
                                                     * 1 as libc::c_long >>
                                                     16 as libc::c_int *
                                                         (0x200 as libc::c_int
                                                              as uint32_t as
                                                              libc::c_long >
                                                              65535 as
                                                                  libc::c_long)
                                                             as libc::c_int >
                                                     255 as libc::c_int as
                                                         libc::c_long) as
                                                    libc::c_int) as
                                               libc::c_long +
                                               (8 as libc::c_int as
                                                    libc::c_long -
                                                    90 as libc::c_int as
                                                        libc::c_long /
                                                        ((0x200 as libc::c_int
                                                              as uint32_t as
                                                              libc::c_long *
                                                              1 as
                                                                  libc::c_long
                                                              >>
                                                              16 as
                                                                  libc::c_int
                                                                  *
                                                                  (0x200 as
                                                                       libc::c_int
                                                                       as
                                                                       uint32_t
                                                                       as
                                                                       libc::c_long
                                                                       >
                                                                       65535
                                                                           as
                                                                           libc::c_long)
                                                                      as
                                                                      libc::c_int
                                                              >>
                                                              8 as libc::c_int
                                                                  *
                                                                  (0x200 as
                                                                       libc::c_int
                                                                       as
                                                                       uint32_t
                                                                       as
                                                                       libc::c_long
                                                                       *
                                                                       1 as
                                                                           libc::c_long
                                                                       >>
                                                                       16 as
                                                                           libc::c_int
                                                                           *
                                                                           (0x200
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                uint32_t
                                                                                as
                                                                                libc::c_long
                                                                                >
                                                                                65535
                                                                                    as
                                                                                    libc::c_long)
                                                                               as
                                                                               libc::c_int
                                                                       >
                                                                       255 as
                                                                           libc::c_int
                                                                           as
                                                                           libc::c_long)
                                                                      as
                                                                      libc::c_int)
                                                             /
                                                             4 as libc::c_int
                                                                 as
                                                                 libc::c_long
                                                             +
                                                             14 as libc::c_int
                                                                 as
                                                                 libc::c_long
                                                             |
                                                             1 as libc::c_int
                                                                 as
                                                                 libc::c_long)
                                                    -
                                                    2 as libc::c_int as
                                                        libc::c_long /
                                                        ((0x200 as libc::c_int
                                                              as uint32_t as
                                                              libc::c_long *
                                                              1 as
                                                                  libc::c_long
                                                              >>
                                                              16 as
                                                                  libc::c_int
                                                                  *
                                                                  (0x200 as
                                                                       libc::c_int
                                                                       as
                                                                       uint32_t
                                                                       as
                                                                       libc::c_long
                                                                       >
                                                                       65535
                                                                           as
                                                                           libc::c_long)
                                                                      as
                                                                      libc::c_int
                                                              >>
                                                              8 as libc::c_int
                                                                  *
                                                                  (0x200 as
                                                                       libc::c_int
                                                                       as
                                                                       uint32_t
                                                                       as
                                                                       libc::c_long
                                                                       *
                                                                       1 as
                                                                           libc::c_long
                                                                       >>
                                                                       16 as
                                                                           libc::c_int
                                                                           *
                                                                           (0x200
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                uint32_t
                                                                                as
                                                                                libc::c_long
                                                                                >
                                                                                65535
                                                                                    as
                                                                                    libc::c_long)
                                                                               as
                                                                               libc::c_int
                                                                       >
                                                                       255 as
                                                                           libc::c_int
                                                                           as
                                                                           libc::c_long)
                                                                      as
                                                                      libc::c_int)
                                                             /
                                                             2 as libc::c_int
                                                                 as
                                                                 libc::c_long
                                                             +
                                                             1 as libc::c_int
                                                                 as
                                                                 libc::c_long))))
                                     as rccPeriphTag_t,
                             DMAy_Channelx:
                                 (0x40000000 as libc::c_int as
                                      uint32_t).wrapping_add(0x20000 as
                                                                 libc::c_int
                                                                 as
                                                                 libc::c_uint).wrapping_add(0x8
                                                                                                as
                                                                                                libc::c_int
                                                                                                as
                                                                                                libc::c_uint)
                                     as *mut DMA_Channel_TypeDef,};
             init
         }]
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
