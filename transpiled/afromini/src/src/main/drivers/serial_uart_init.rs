use core;
use libc;
extern "C" {
    /* Exported constants --------------------------------------------------------*/
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
    /* *
  * @}
  */
    /* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
    /* Function used to set the DMA configuration to the default reset state ******/
    #[no_mangle]
    fn DMA_DeInit(DMAy_Channelx: *mut DMA_Channel_TypeDef);
    /* Initialization and Configuration functions *********************************/
    #[no_mangle]
    fn DMA_Init(DMAy_Channelx: *mut DMA_Channel_TypeDef,
                DMA_InitStruct: *mut DMA_InitTypeDef);
    #[no_mangle]
    fn DMA_StructInit(DMA_InitStruct: *mut DMA_InitTypeDef);
    #[no_mangle]
    fn DMA_Cmd(DMAy_Channelx: *mut DMA_Channel_TypeDef,
               NewState: FunctionalState);
    /* Data Counter functions******************************************************/
    #[no_mangle]
    fn DMA_SetCurrDataCounter(DMAy_Channelx: *mut DMA_Channel_TypeDef,
                              DataNumber: uint16_t);
    #[no_mangle]
    fn DMA_GetCurrDataCounter(DMAy_Channelx: *mut DMA_Channel_TypeDef)
     -> uint16_t;
    /* Interrupts and flags management functions **********************************/
    #[no_mangle]
    fn DMA_ITConfig(DMAy_Channelx: *mut DMA_Channel_TypeDef, DMA_IT: uint32_t,
                    NewState: FunctionalState);
    #[no_mangle]
    fn USART_Init(USARTx: *mut USART_TypeDef,
                  USART_InitStruct: *mut USART_InitTypeDef);
    #[no_mangle]
    fn USART_Cmd(USARTx: *mut USART_TypeDef, NewState: FunctionalState);
    #[no_mangle]
    fn USART_InvPinCmd(USARTx: *mut USART_TypeDef, USART_InvPin: uint32_t,
                       NewState: FunctionalState);
    /* Half-duplex mode function **************************************************/
    #[no_mangle]
    fn USART_HalfDuplexCmd(USARTx: *mut USART_TypeDef,
                           NewState: FunctionalState);
    /* DMA transfers management functions *****************************************/
    #[no_mangle]
    fn USART_DMACmd(USARTx: *mut USART_TypeDef, USART_DMAReq: uint32_t,
                    NewState: FunctionalState);
    /* Interrupts and flags management functions **********************************/
    #[no_mangle]
    fn USART_ITConfig(USARTx: *mut USART_TypeDef, USART_IT: uint32_t,
                      NewState: FunctionalState);
    #[no_mangle]
    fn USART_ClearITPendingBit(USARTx: *mut USART_TypeDef,
                               USART_IT: uint32_t);
    #[no_mangle]
    fn serialUART(device: UARTDevice_e, baudRate: uint32_t, mode: portMode_e,
                  options: portOptions_e) -> *mut uartPort_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct DMA_Channel_TypeDef {
    pub CCR: uint32_t,
    pub CNDTR: uint32_t,
    pub CPAR: uint32_t,
    pub CMAR: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct USART_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub CR3: uint32_t,
    pub BRR: uint16_t,
    pub RESERVED1: uint16_t,
    pub GTPR: uint16_t,
    pub RESERVED2: uint16_t,
    pub RTOR: uint32_t,
    pub RQR: uint16_t,
    pub RESERVED3: uint16_t,
    pub ISR: uint32_t,
    pub ICR: uint32_t,
    pub RDR: uint16_t,
    pub RESERVED4: uint16_t,
    pub TDR: uint16_t,
    pub RESERVED5: uint16_t,
}
/* *
  ******************************************************************************
  * @file    stm32f30x_dma.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the DMA firmware
  *          library.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
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
/* * @addtogroup STM32F30x_StdPeriph_Driver
  * @{
  */
/* * @addtogroup DMA
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief  DMA Init structures definition
  */
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct DMA_InitTypeDef {
    pub DMA_PeripheralBaseAddr: uint32_t,
    pub DMA_MemoryBaseAddr: uint32_t,
    pub DMA_DIR: uint32_t,
    pub DMA_BufferSize: uint16_t,
    pub DMA_PeripheralInc: uint32_t,
    pub DMA_MemoryInc: uint32_t,
    pub DMA_PeripheralDataSize: uint32_t,
    pub DMA_MemoryDataSize: uint32_t,
    pub DMA_Mode: uint32_t,
    pub DMA_Priority: uint32_t,
    pub DMA_M2M: uint32_t,
    /* !< Specifies if the DMAy Channelx will be used in memory-to-memory transfer.
                                        This parameter can be a value of @ref DMA_memory_to_memory            */
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct USART_InitTypeDef {
    pub USART_BaudRate: uint32_t,
    pub USART_WordLength: uint32_t,
    pub USART_StopBits: uint32_t,
    pub USART_Parity: uint32_t,
    pub USART_Mode: uint32_t,
    pub USART_HardwareFlowControl: uint32_t,
}
pub type portMode_e = libc::c_uint;
pub const MODE_RXTX: portMode_e = 3;
pub const MODE_TX: portMode_e = 2;
pub const MODE_RX: portMode_e = 1;
pub type portOptions_e = libc::c_uint;
pub const SERIAL_BIDIR_NOPULL: portOptions_e = 32;
pub const SERIAL_BIDIR_PP: portOptions_e = 16;
pub const SERIAL_BIDIR_OD: portOptions_e = 0;
pub const SERIAL_BIDIR: portOptions_e = 8;
pub const SERIAL_UNIDIR: portOptions_e = 0;
pub const SERIAL_PARITY_EVEN: portOptions_e = 4;
pub const SERIAL_PARITY_NO: portOptions_e = 0;
pub const SERIAL_STOPBITS_2: portOptions_e = 2;
pub const SERIAL_STOPBITS_1: portOptions_e = 0;
pub const SERIAL_INVERTED: portOptions_e = 1;
pub const SERIAL_NOT_INVERTED: portOptions_e = 0;
// Define known line control states which may be passed up by underlying serial driver callback
pub type serialReceiveCallbackPtr
    =
    Option<unsafe extern "C" fn(_: uint16_t, _: *mut libc::c_void) -> ()>;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct serialPort_s {
    pub vTable: *const serialPortVTable,
    pub mode: portMode_e,
    pub options: portOptions_e,
    pub baudRate: uint32_t,
    pub rxBufferSize: uint32_t,
    pub txBufferSize: uint32_t,
    pub rxBuffer: *mut uint8_t,
    pub txBuffer: *mut uint8_t,
    pub rxBufferHead: uint32_t,
    pub rxBufferTail: uint32_t,
    pub txBufferHead: uint32_t,
    pub txBufferTail: uint32_t,
    pub rxCallback: serialReceiveCallbackPtr,
    pub rxCallbackData: *mut libc::c_void,
    pub identifier: uint8_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct serialPortVTable {
    pub serialWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                 _: uint8_t) -> ()>,
    pub serialTotalRxWaiting: Option<unsafe extern "C" fn(_:
                                                              *const serialPort_t)
                                         -> uint32_t>,
    pub serialTotalTxFree: Option<unsafe extern "C" fn(_: *const serialPort_t)
                                      -> uint32_t>,
    pub serialRead: Option<unsafe extern "C" fn(_: *mut serialPort_t)
                               -> uint8_t>,
    pub serialSetBaudRate: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                       _: uint32_t) -> ()>,
    pub isSerialTransmitBufferEmpty: Option<unsafe extern "C" fn(_:
                                                                     *const serialPort_t)
                                                -> bool>,
    pub setMode: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                             _: portMode_e) -> ()>,
    pub setCtrlLineStateCb: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                        _:
                                                            Option<unsafe extern "C" fn(_:
                                                                                            *mut libc::c_void,
                                                                                        _:
                                                                                            uint16_t)
                                                                       -> ()>,
                                                        _: *mut libc::c_void)
                                       -> ()>,
    pub setBaudRateCb: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                   _:
                                                       Option<unsafe extern "C" fn(_:
                                                                                       *mut serialPort_t,
                                                                                   _:
                                                                                       uint32_t)
                                                                  -> ()>,
                                                   _: *mut serialPort_t)
                                  -> ()>,
    pub writeBuf: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                              _: *const libc::c_void,
                                              _: libc::c_int) -> ()>,
    pub beginWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t) -> ()>,
    pub endWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t) -> ()>,
}
// used by serial drivers to return frames to app
pub type serialPort_t = serialPort_s;
pub type UARTDevice_e = libc::c_uint;
pub const UARTDEV_8: UARTDevice_e = 7;
pub const UARTDEV_7: UARTDevice_e = 6;
pub const UARTDEV_6: UARTDevice_e = 5;
pub const UARTDEV_5: UARTDevice_e = 4;
pub const UARTDEV_4: UARTDevice_e = 3;
pub const UARTDEV_3: UARTDevice_e = 2;
pub const UARTDEV_2: UARTDevice_e = 1;
pub const UARTDEV_1: UARTDevice_e = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct uartPort_s {
    pub port: serialPort_t,
    pub rxDMAChannel: *mut DMA_Channel_TypeDef,
    pub txDMAChannel: *mut DMA_Channel_TypeDef,
    pub rxDMAIrq: uint32_t,
    pub txDMAIrq: uint32_t,
    pub rxDMAPos: uint32_t,
    pub txDMAPeripheralBaseAddr: uint32_t,
    pub rxDMAPeripheralBaseAddr: uint32_t,
    pub USARTx: *mut USART_TypeDef,
    pub txDMAEmpty: bool,
}
pub type uartPort_t = uartPort_s;
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
 * Initialization part of serial_uart.c
 */
/*
 * Authors:
 * jflyper - Refactoring, cleanup and made pin-configurable
 * Dominic Clifton - Serial port abstraction, Separation of common STM32 code for cleanflight, various cleanups.
 * Hamasaki/Timecop - Initial baseflight code
*/
unsafe extern "C" fn usartConfigurePinInversion(mut uartPort:
                                                    *mut uartPort_t) {
    let mut inverted: bool =
        (*uartPort).port.options as libc::c_uint &
            SERIAL_INVERTED as libc::c_int as libc::c_uint != 0;
    let mut inversionPins: uint32_t = 0i32 as uint32_t;
    if (*uartPort).port.mode as libc::c_uint &
           MODE_TX as libc::c_int as libc::c_uint != 0 {
        inversionPins |= 0x20000i32 as uint32_t
    }
    if (*uartPort).port.mode as libc::c_uint &
           MODE_RX as libc::c_int as libc::c_uint != 0 {
        inversionPins |= 0x10000i32 as uint32_t
    }
    USART_InvPinCmd((*uartPort).USARTx, inversionPins,
                    if inverted as libc::c_int != 0 {
                        ENABLE as libc::c_int
                    } else { DISABLE as libc::c_int } as FunctionalState);
}
#[no_mangle]
pub unsafe extern "C" fn uartReconfigure(mut uartPort: *mut uartPort_t) {
    let mut USART_InitStructure: USART_InitTypeDef =
        USART_InitTypeDef{USART_BaudRate: 0,
                          USART_WordLength: 0,
                          USART_StopBits: 0,
                          USART_Parity: 0,
                          USART_Mode: 0,
                          USART_HardwareFlowControl: 0,};
    USART_Cmd((*uartPort).USARTx, DISABLE);
    USART_InitStructure.USART_BaudRate = (*uartPort).port.baudRate;
    // according to the stm32 documentation wordlen has to be 9 for parity bits
    // this does not seem to matter for rx but will give bad data on tx!
    // This seems to cause RX to break on STM32F1, see https://github.com/betaflight/betaflight/pull/1654
    if (*uartPort).port.options as libc::c_uint &
           SERIAL_PARITY_EVEN as libc::c_int as libc::c_uint != 0 {
        USART_InitStructure.USART_WordLength = 0x1000i32 as uint32_t
    } else { USART_InitStructure.USART_WordLength = 0i32 as uint32_t }
    USART_InitStructure.USART_StopBits =
        if (*uartPort).port.options as libc::c_uint &
               SERIAL_STOPBITS_2 as libc::c_int as libc::c_uint != 0 {
            0x2000i32 as uint32_t
        } else { 0i32 as uint32_t };
    USART_InitStructure.USART_Parity =
        if (*uartPort).port.options as libc::c_uint &
               SERIAL_PARITY_EVEN as libc::c_int as libc::c_uint != 0 {
            0x400i32 as uint32_t
        } else { 0i32 as uint32_t };
    USART_InitStructure.USART_HardwareFlowControl = 0i32 as uint32_t;
    USART_InitStructure.USART_Mode = 0i32 as uint32_t;
    if (*uartPort).port.mode as libc::c_uint &
           MODE_RX as libc::c_int as libc::c_uint != 0 {
        USART_InitStructure.USART_Mode |= 0x4i32 as uint32_t
    }
    if (*uartPort).port.mode as libc::c_uint &
           MODE_TX as libc::c_int as libc::c_uint != 0 {
        USART_InitStructure.USART_Mode |= 0x8i32 as uint32_t
    }
    USART_Init((*uartPort).USARTx, &mut USART_InitStructure);
    usartConfigurePinInversion(uartPort);
    if (*uartPort).port.options as libc::c_uint &
           SERIAL_BIDIR as libc::c_int as libc::c_uint != 0 {
        USART_HalfDuplexCmd((*uartPort).USARTx, ENABLE);
    } else { USART_HalfDuplexCmd((*uartPort).USARTx, DISABLE); }
    USART_Cmd((*uartPort).USARTx, ENABLE);
}
#[no_mangle]
pub unsafe extern "C" fn uartOpen(mut device: UARTDevice_e,
                                  mut rxCallback: serialReceiveCallbackPtr,
                                  mut rxCallbackData: *mut libc::c_void,
                                  mut baudRate: uint32_t,
                                  mut mode: portMode_e,
                                  mut options: portOptions_e)
 -> *mut serialPort_t {
    let mut s: *mut uartPort_t = serialUART(device, baudRate, mode, options);
    if s.is_null() { return s as *mut serialPort_t }
    (*s).txDMAEmpty = 1i32 != 0;
    // common serial initialisation code should move to serialPort::init()
    (*s).port.rxBufferTail = 0i32 as uint32_t;
    (*s).port.rxBufferHead = (*s).port.rxBufferTail;
    (*s).port.txBufferTail = 0i32 as uint32_t;
    (*s).port.txBufferHead = (*s).port.txBufferTail;
    // callback works for IRQ-based RX ONLY
    (*s).port.rxCallback = rxCallback;
    (*s).port.rxCallbackData = rxCallbackData;
    (*s).port.mode = mode;
    (*s).port.baudRate = baudRate;
    (*s).port.options = options;
    uartReconfigure(s);
    // Receive DMA or IRQ
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
    if mode as libc::c_uint & MODE_RX as libc::c_int as libc::c_uint != 0 {
        if !(*s).rxDMAChannel.is_null() {
            DMA_StructInit(&mut DMA_InitStructure);
            DMA_InitStructure.DMA_PeripheralBaseAddr =
                (*s).rxDMAPeripheralBaseAddr;
            DMA_InitStructure.DMA_Priority = 0x1000i32 as uint32_t;
            DMA_InitStructure.DMA_M2M = 0i32 as uint32_t;
            DMA_InitStructure.DMA_PeripheralInc = 0i32 as uint32_t;
            DMA_InitStructure.DMA_PeripheralDataSize = 0i32 as uint32_t;
            DMA_InitStructure.DMA_MemoryInc = 0x80i32 as uint32_t;
            DMA_InitStructure.DMA_MemoryDataSize = 0i32 as uint32_t;
            DMA_InitStructure.DMA_BufferSize =
                (*s).port.rxBufferSize as uint16_t;
            DMA_InitStructure.DMA_DIR = 0i32 as uint32_t;
            DMA_InitStructure.DMA_Mode = 0x20i32 as uint32_t;
            DMA_InitStructure.DMA_MemoryBaseAddr =
                (*s).port.rxBuffer as uint32_t;
            DMA_DeInit((*s).rxDMAChannel);
            DMA_Init((*s).rxDMAChannel, &mut DMA_InitStructure);
            DMA_Cmd((*s).rxDMAChannel, ENABLE);
            USART_DMACmd((*s).USARTx, 0x40i32 as uint32_t, ENABLE);
            (*s).rxDMAPos =
                DMA_GetCurrDataCounter((*s).rxDMAChannel) as uint32_t
        } else {
            USART_ClearITPendingBit((*s).USARTx, 0x50105i32 as uint32_t);
            USART_ITConfig((*s).USARTx, 0x50105i32 as uint32_t, ENABLE);
        }
    }
    // Transmit DMA or IRQ
    if mode as libc::c_uint & MODE_TX as libc::c_int as libc::c_uint != 0 {
        if !(*s).txDMAChannel.is_null() {
            DMA_StructInit(&mut DMA_InitStructure);
            DMA_InitStructure.DMA_PeripheralBaseAddr =
                (*s).txDMAPeripheralBaseAddr;
            DMA_InitStructure.DMA_Priority = 0x1000i32 as uint32_t;
            DMA_InitStructure.DMA_M2M = 0i32 as uint32_t;
            DMA_InitStructure.DMA_PeripheralInc = 0i32 as uint32_t;
            DMA_InitStructure.DMA_PeripheralDataSize = 0i32 as uint32_t;
            DMA_InitStructure.DMA_MemoryInc = 0x80i32 as uint32_t;
            DMA_InitStructure.DMA_MemoryDataSize = 0i32 as uint32_t;
            DMA_InitStructure.DMA_BufferSize =
                (*s).port.txBufferSize as uint16_t;
            DMA_InitStructure.DMA_DIR = 0x10i32 as uint32_t;
            DMA_InitStructure.DMA_Mode = 0i32 as uint32_t;
            DMA_DeInit((*s).txDMAChannel);
            DMA_Init((*s).txDMAChannel, &mut DMA_InitStructure);
            DMA_ITConfig((*s).txDMAChannel, 0x2i32 as uint32_t, ENABLE);
            DMA_SetCurrDataCounter((*s).txDMAChannel, 0i32 as uint16_t);
            ::core::ptr::write_volatile(&mut (*(*s).txDMAChannel).CNDTR as
                                            *mut uint32_t, 0i32 as uint32_t);
            USART_DMACmd((*s).USARTx, 0x80i32 as uint32_t, ENABLE);
        } else {
            USART_ITConfig((*s).USARTx, 0x70107i32 as uint32_t, ENABLE);
        }
    }
    USART_Cmd((*s).USARTx, ENABLE);
    return s as *mut serialPort_t;
}
