use ::libc;
extern "C" {
    #[no_mangle]
    static uartHardware: [uartHardware_t; 0];
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USART_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub CR3: uint32_t,
    pub BRR: uint32_t,
    pub GTPR: uint32_t,
    pub RTOR: uint32_t,
    pub RQR: uint32_t,
    pub ISR: uint32_t,
    pub ICR: uint32_t,
    pub RDR: uint32_t,
    pub TDR: uint32_t,
}
pub type size_t = libc::c_ulong;
pub type HAL_LockTypeDef = libc::c_uint;
pub const HAL_LOCKED: HAL_LockTypeDef = 1;
pub const HAL_UNLOCKED: HAL_LockTypeDef = 0;
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
pub type HAL_DMA_StateTypeDef = libc::c_uint;
pub const HAL_DMA_STATE_ABORT: HAL_DMA_StateTypeDef = 5;
pub const HAL_DMA_STATE_ERROR: HAL_DMA_StateTypeDef = 4;
pub const HAL_DMA_STATE_TIMEOUT: HAL_DMA_StateTypeDef = 3;
pub const HAL_DMA_STATE_BUSY: HAL_DMA_StateTypeDef = 2;
pub const HAL_DMA_STATE_READY: HAL_DMA_StateTypeDef = 1;
pub const HAL_DMA_STATE_RESET: HAL_DMA_StateTypeDef = 0;
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
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_uart.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of UART HAL module.
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
/* * @addtogroup UART
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup UART_Exported_Types UART Exported Types
  * @{
  */
/* *
  * @brief UART Init Structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct UART_InitTypeDef {
    pub BaudRate: uint32_t,
    pub WordLength: uint32_t,
    pub StopBits: uint32_t,
    pub Parity: uint32_t,
    pub Mode: uint32_t,
    pub HwFlowCtl: uint32_t,
    pub OverSampling: uint32_t,
    pub OneBitSampling: uint32_t,
}
/* *
  * @brief  UART Advanced Features initialization structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct UART_AdvFeatureInitTypeDef {
    pub AdvFeatureInit: uint32_t,
    pub TxPinLevelInvert: uint32_t,
    pub RxPinLevelInvert: uint32_t,
    pub DataInvert: uint32_t,
    pub Swap: uint32_t,
    pub OverrunDisable: uint32_t,
    pub DMADisableonRxError: uint32_t,
    pub AutoBaudRateEnable: uint32_t,
    pub AutoBaudRateMode: uint32_t,
    pub MSBFirst: uint32_t,
}
/* *
  * @brief HAL UART State structures definition
  * @note  HAL UART State value is a combination of 2 different substates: gState and RxState.
  *        - gState contains UART state information related to global Handle management 
  *          and also information related to Tx operations.
  *          gState value coding follow below described bitmap :
  *          b7-b6  Error information 
  *             00 : No Error
  *             01 : (Not Used)
  *             10 : Timeout
  *             11 : Error
  *          b5     IP initilisation status
  *             0  : Reset (IP not initialized)
  *             1  : Init done (IP not initialized. HAL UART Init function already called)
  *          b4-b3  (not used)
  *             xx : Should be set to 00
  *          b2     Intrinsic process state
  *             0  : Ready
  *             1  : Busy (IP busy with some configuration or internal operations)
  *          b1     (not used)
  *             x  : Should be set to 0
  *          b0     Tx state
  *             0  : Ready (no Tx operation ongoing)
  *             1  : Busy (Tx operation ongoing)
  *        - RxState contains information related to Rx operations.
  *          RxState value coding follow below described bitmap :
  *          b7-b6  (not used)
  *             xx : Should be set to 00
  *          b5     IP initilisation status
  *             0  : Reset (IP not initialized)
  *             1  : Init done (IP not initialized)
  *          b4-b2  (not used)
  *            xxx : Should be set to 000
  *          b1     Rx state
  *             0  : Ready (no Rx operation ongoing)
  *             1  : Busy (Rx operation ongoing)
  *          b0     (not used)
  *             x  : Should be set to 0.
  */
pub type HAL_UART_StateTypeDef = libc::c_uint;
/* !< Error
                                                   Value is allowed for gState only */
/* !< Timeout state
                                                   Value is allowed for gState only */
pub const HAL_UART_STATE_ERROR: HAL_UART_StateTypeDef = 224;
/* !< Data Transmission and Reception process is ongoing
                                                   Not to be used for neither gState nor RxState.
                                                   Value is result of combination (Or) between gState and RxState values */
pub const HAL_UART_STATE_TIMEOUT: HAL_UART_StateTypeDef = 160;
/* !< Data Reception process is ongoing
                                                   Value is allowed for RxState only */
pub const HAL_UART_STATE_BUSY_TX_RX: HAL_UART_StateTypeDef = 35;
/* !< Data Transmission process is ongoing
                                                   Value is allowed for gState only */
pub const HAL_UART_STATE_BUSY_RX: HAL_UART_StateTypeDef = 34;
/* !< an internal process is ongoing 
                                                   Value is allowed for gState only */
pub const HAL_UART_STATE_BUSY_TX: HAL_UART_StateTypeDef = 33;
/* !< Peripheral Initialized and ready for use
                                                   Value is allowed for gState and RxState */
pub const HAL_UART_STATE_BUSY: HAL_UART_StateTypeDef = 36;
/* !< Peripheral is not initialized
                                                   Value is allowed for gState and RxState */
pub const HAL_UART_STATE_READY: HAL_UART_StateTypeDef = 32;
pub const HAL_UART_STATE_RESET: HAL_UART_StateTypeDef = 0;
/* *
  * @brief  UART handle Structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct UART_HandleTypeDef {
    pub Instance: *mut USART_TypeDef,
    pub Init: UART_InitTypeDef,
    pub AdvancedInit: UART_AdvFeatureInitTypeDef,
    pub pTxBuffPtr: *mut uint8_t,
    pub TxXferSize: uint16_t,
    pub TxXferCount: uint16_t,
    pub pRxBuffPtr: *mut uint8_t,
    pub RxXferSize: uint16_t,
    pub RxXferCount: uint16_t,
    pub Mask: uint16_t,
    pub hdmatx: *mut DMA_HandleTypeDef,
    pub hdmarx: *mut DMA_HandleTypeDef,
    pub Lock: HAL_LockTypeDef,
    pub gState: HAL_UART_StateTypeDef,
    pub RxState: HAL_UART_StateTypeDef,
    pub ErrorCode: uint32_t,
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
pub type rccPeriphTag_t = uint8_t;
pub type ioTag_t = uint8_t;
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
#[derive(Copy, Clone)]
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
#[derive(Copy, Clone)]
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPinConfig_s {
    pub ioTagTx: [ioTag_t; 12],
    pub ioTagRx: [ioTag_t; 12],
    pub ioTagInverter: [ioTag_t; 12],
}
pub type serialPinConfig_t = serialPinConfig_s;
pub type UARTDevice_e = libc::c_uint;
pub const UARTDEV_8: UARTDevice_e = 7;
pub const UARTDEV_7: UARTDevice_e = 6;
pub const UARTDEV_6: UARTDevice_e = 5;
pub const UARTDEV_5: UARTDevice_e = 4;
pub const UARTDEV_4: UARTDevice_e = 3;
pub const UARTDEV_3: UARTDevice_e = 2;
pub const UARTDEV_2: UARTDevice_e = 1;
pub const UARTDEV_1: UARTDevice_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct uartPort_s {
    pub port: serialPort_t,
    pub rxDMAHandle: DMA_HandleTypeDef,
    pub txDMAHandle: DMA_HandleTypeDef,
    pub rxDMAStream: *mut DMA_Stream_TypeDef,
    pub txDMAStream: *mut DMA_Stream_TypeDef,
    pub rxDMAChannel: uint32_t,
    pub txDMAChannel: uint32_t,
    pub rxDMAIrq: uint32_t,
    pub txDMAIrq: uint32_t,
    pub rxDMAPos: uint32_t,
    pub txDMAPeripheralBaseAddr: uint32_t,
    pub rxDMAPeripheralBaseAddr: uint32_t,
    pub Handle: UART_HandleTypeDef,
    pub USARTx: *mut USART_TypeDef,
    pub txDMAEmpty: bool,
}
pub type uartPort_t = uartPort_s;
// All USARTs can also be used as UART, and we use them only as UART.
// uartDevice_t is an actual device instance.
// XXX Instances are allocated for uarts defined by USE_UARTx atm.
pub type uartDevice_t = uartDevice_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct uartDevice_s {
    pub port: uartPort_t,
    pub hardware: *const uartHardware_t,
    pub rx: ioTag_t,
    pub tx: ioTag_t,
    pub rxBuffer: [uint8_t; 128],
    pub txBuffer: [uint8_t; 256],
}
pub type uartHardware_t = uartHardware_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct uartHardware_s {
    pub device: UARTDevice_e,
    pub reg: *mut USART_TypeDef,
    pub DMAChannel: uint32_t,
    pub txDMAStream: *mut DMA_Stream_TypeDef,
    pub rxDMAStream: *mut DMA_Stream_TypeDef,
    pub rxPins: [ioTag_t; 3],
    pub txPins: [ioTag_t; 3],
    pub rcc_ahb1: uint32_t,
    pub rcc_apb2: rccPeriphTag_t,
    pub rcc_apb1: rccPeriphTag_t,
    pub af: uint8_t,
    pub txIrq: uint8_t,
    pub rxIrq: uint8_t,
    pub txPriority: uint8_t,
    pub rxPriority: uint8_t,
}
// XXX Not required for full allocation
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
 * UART pin configuration common to all MCUs.
 */
/*
 * Authors:
 * jflyper - Created as a part of configurable UART/refactoring.
*/
#[no_mangle]
#[link_section = ".fastram_bss"]
pub static mut uartDevice: [uartDevice_t; 8] =
    [uartDevice_t{port:
                      uartPort_t{port:
                                     serialPort_t{vTable:
                                                      0 as
                                                          *const serialPortVTable,
                                                  mode: 0 as portMode_e,
                                                  options:
                                                      SERIAL_NOT_INVERTED,
                                                  baudRate: 0,
                                                  rxBufferSize: 0,
                                                  txBufferSize: 0,
                                                  rxBuffer:
                                                      0 as *const uint8_t as
                                                          *mut uint8_t,
                                                  txBuffer:
                                                      0 as *const uint8_t as
                                                          *mut uint8_t,
                                                  rxBufferHead: 0,
                                                  rxBufferTail: 0,
                                                  txBufferHead: 0,
                                                  txBufferTail: 0,
                                                  rxCallback: None,
                                                  rxCallbackData:
                                                      0 as *const libc::c_void
                                                          as
                                                          *mut libc::c_void,
                                                  identifier: 0,},
                                 rxDMAHandle:
                                     DMA_HandleTypeDef{Instance:
                                                           0 as
                                                               *const DMA_Stream_TypeDef
                                                               as
                                                               *mut DMA_Stream_TypeDef,
                                                       Init:
                                                           DMA_InitTypeDef{Channel:
                                                                               0,
                                                                           Direction:
                                                                               0,
                                                                           PeriphInc:
                                                                               0,
                                                                           MemInc:
                                                                               0,
                                                                           PeriphDataAlignment:
                                                                               0,
                                                                           MemDataAlignment:
                                                                               0,
                                                                           Mode:
                                                                               0,
                                                                           Priority:
                                                                               0,
                                                                           FIFOMode:
                                                                               0,
                                                                           FIFOThreshold:
                                                                               0,
                                                                           MemBurst:
                                                                               0,
                                                                           PeriphBurst:
                                                                               0,},
                                                       Lock: HAL_UNLOCKED,
                                                       State:
                                                           HAL_DMA_STATE_RESET,
                                                       Parent:
                                                           0 as
                                                               *const libc::c_void
                                                               as
                                                               *mut libc::c_void,
                                                       XferCpltCallback: None,
                                                       XferHalfCpltCallback:
                                                           None,
                                                       XferM1CpltCallback:
                                                           None,
                                                       XferM1HalfCpltCallback:
                                                           None,
                                                       XferErrorCallback:
                                                           None,
                                                       XferAbortCallback:
                                                           None,
                                                       ErrorCode: 0,
                                                       StreamBaseAddress: 0,
                                                       StreamIndex: 0,},
                                 txDMAHandle:
                                     DMA_HandleTypeDef{Instance:
                                                           0 as
                                                               *const DMA_Stream_TypeDef
                                                               as
                                                               *mut DMA_Stream_TypeDef,
                                                       Init:
                                                           DMA_InitTypeDef{Channel:
                                                                               0,
                                                                           Direction:
                                                                               0,
                                                                           PeriphInc:
                                                                               0,
                                                                           MemInc:
                                                                               0,
                                                                           PeriphDataAlignment:
                                                                               0,
                                                                           MemDataAlignment:
                                                                               0,
                                                                           Mode:
                                                                               0,
                                                                           Priority:
                                                                               0,
                                                                           FIFOMode:
                                                                               0,
                                                                           FIFOThreshold:
                                                                               0,
                                                                           MemBurst:
                                                                               0,
                                                                           PeriphBurst:
                                                                               0,},
                                                       Lock: HAL_UNLOCKED,
                                                       State:
                                                           HAL_DMA_STATE_RESET,
                                                       Parent:
                                                           0 as
                                                               *const libc::c_void
                                                               as
                                                               *mut libc::c_void,
                                                       XferCpltCallback: None,
                                                       XferHalfCpltCallback:
                                                           None,
                                                       XferM1CpltCallback:
                                                           None,
                                                       XferM1HalfCpltCallback:
                                                           None,
                                                       XferErrorCallback:
                                                           None,
                                                       XferAbortCallback:
                                                           None,
                                                       ErrorCode: 0,
                                                       StreamBaseAddress: 0,
                                                       StreamIndex: 0,},
                                 rxDMAStream:
                                     0 as *const DMA_Stream_TypeDef as
                                         *mut DMA_Stream_TypeDef,
                                 txDMAStream:
                                     0 as *const DMA_Stream_TypeDef as
                                         *mut DMA_Stream_TypeDef,
                                 rxDMAChannel: 0,
                                 txDMAChannel: 0,
                                 rxDMAIrq: 0,
                                 txDMAIrq: 0,
                                 rxDMAPos: 0,
                                 txDMAPeripheralBaseAddr: 0,
                                 rxDMAPeripheralBaseAddr: 0,
                                 Handle:
                                     UART_HandleTypeDef{Instance:
                                                            0 as
                                                                *const USART_TypeDef
                                                                as
                                                                *mut USART_TypeDef,
                                                        Init:
                                                            UART_InitTypeDef{BaudRate:
                                                                                 0,
                                                                             WordLength:
                                                                                 0,
                                                                             StopBits:
                                                                                 0,
                                                                             Parity:
                                                                                 0,
                                                                             Mode:
                                                                                 0,
                                                                             HwFlowCtl:
                                                                                 0,
                                                                             OverSampling:
                                                                                 0,
                                                                             OneBitSampling:
                                                                                 0,},
                                                        AdvancedInit:
                                                            UART_AdvFeatureInitTypeDef{AdvFeatureInit:
                                                                                           0,
                                                                                       TxPinLevelInvert:
                                                                                           0,
                                                                                       RxPinLevelInvert:
                                                                                           0,
                                                                                       DataInvert:
                                                                                           0,
                                                                                       Swap:
                                                                                           0,
                                                                                       OverrunDisable:
                                                                                           0,
                                                                                       DMADisableonRxError:
                                                                                           0,
                                                                                       AutoBaudRateEnable:
                                                                                           0,
                                                                                       AutoBaudRateMode:
                                                                                           0,
                                                                                       MSBFirst:
                                                                                           0,},
                                                        pTxBuffPtr:
                                                            0 as
                                                                *const uint8_t
                                                                as
                                                                *mut uint8_t,
                                                        TxXferSize: 0,
                                                        TxXferCount: 0,
                                                        pRxBuffPtr:
                                                            0 as
                                                                *const uint8_t
                                                                as
                                                                *mut uint8_t,
                                                        RxXferSize: 0,
                                                        RxXferCount: 0,
                                                        Mask: 0,
                                                        hdmatx:
                                                            0 as
                                                                *const DMA_HandleTypeDef
                                                                as
                                                                *mut DMA_HandleTypeDef,
                                                        hdmarx:
                                                            0 as
                                                                *const DMA_HandleTypeDef
                                                                as
                                                                *mut DMA_HandleTypeDef,
                                                        Lock: HAL_UNLOCKED,
                                                        gState:
                                                            HAL_UART_STATE_RESET,
                                                        RxState:
                                                            HAL_UART_STATE_RESET,
                                                        ErrorCode: 0,},
                                 USARTx:
                                     0 as *const USART_TypeDef as
                                         *mut USART_TypeDef,
                                 txDMAEmpty: false,},
                  hardware: 0 as *const uartHardware_t,
                  rx: 0,
                  tx: 0,
                  rxBuffer: [0; 128],
                  txBuffer: [0; 256],}; 8];
// Only those configured in target.h
#[no_mangle]
#[link_section = ".fastram_bss"]
pub static mut uartDevmap: [*mut uartDevice_t; 8] =
    [0 as *const uartDevice_t as *mut uartDevice_t; 8];
// Full array
#[no_mangle]
pub unsafe extern "C" fn uartPinConfigure(mut pSerialPinConfig:
                                              *const serialPinConfig_t) {
    let mut uartdev: *mut uartDevice_t = uartDevice.as_mut_ptr();
    let mut hindex: size_t = 0 as libc::c_int as size_t;
    while hindex <
              (1 as libc::c_int + 1 as libc::c_int + 1 as libc::c_int +
                   1 as libc::c_int + 1 as libc::c_int + 1 as libc::c_int +
                   1 as libc::c_int + 1 as libc::c_int) as libc::c_ulong {
        let mut hardware: *const uartHardware_t =
            &*uartHardware.as_ptr().offset(hindex as isize) as
                *const uartHardware_t;
        let device: UARTDevice_e = (*hardware).device;
        let mut pindex: libc::c_int = 0 as libc::c_int;
        while pindex < 3 as libc::c_int {
            if (*hardware).rxPins[pindex as usize] as libc::c_int != 0 &&
                   (*hardware).rxPins[pindex as usize] as libc::c_int ==
                       (*pSerialPinConfig).ioTagRx[device as usize] as
                           libc::c_int {
                (*uartdev).rx = (*pSerialPinConfig).ioTagRx[device as usize]
            }
            if (*hardware).txPins[pindex as usize] as libc::c_int != 0 &&
                   (*hardware).txPins[pindex as usize] as libc::c_int ==
                       (*pSerialPinConfig).ioTagTx[device as usize] as
                           libc::c_int {
                (*uartdev).tx = (*pSerialPinConfig).ioTagTx[device as usize]
            }
            pindex += 1
        }
        if (*uartdev).rx as libc::c_int != 0 ||
               (*uartdev).tx as libc::c_int != 0 {
            (*uartdev).hardware = hardware;
            let fresh0 = uartdev;
            uartdev = uartdev.offset(1);
            uartDevmap[device as usize] = fresh0
        }
        hindex = hindex.wrapping_add(1)
    };
}
