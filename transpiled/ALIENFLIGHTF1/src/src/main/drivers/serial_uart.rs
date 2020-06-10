use ::libc;
extern "C" {
    #[no_mangle]
    fn DMA_Cmd(DMAy_Channelx: *mut DMA_Channel_TypeDef,
               NewState: FunctionalState);
    #[no_mangle]
    fn USART_ITConfig(USARTx: *mut USART_TypeDef, USART_IT: uint16_t,
                      NewState: FunctionalState);
    #[no_mangle]
    static mut uartDevmap: [*mut uartDevice_t; 0];
    #[no_mangle]
    fn uartIrqHandler(s: *mut uartPort_t);
    #[no_mangle]
    fn uartReconfigure(uartPort: *mut uartPort_t);
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
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USART_TypeDef {
    pub SR: uint16_t,
    pub RESERVED0: uint16_t,
    pub DR: uint16_t,
    pub RESERVED1: uint16_t,
    pub BRR: uint16_t,
    pub RESERVED2: uint16_t,
    pub CR1: uint16_t,
    pub RESERVED3: uint16_t,
    pub CR2: uint16_t,
    pub RESERVED4: uint16_t,
    pub CR3: uint16_t,
    pub RESERVED5: uint16_t,
    pub GTPR: uint16_t,
    pub RESERVED6: uint16_t,
}
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
pub type serialPort_t = serialPort_s;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct uartHardware_s {
    pub device: UARTDevice_e,
    pub reg: *mut USART_TypeDef,
    pub txDMAChannel: *mut DMA_Channel_TypeDef,
    pub rxDMAChannel: *mut DMA_Channel_TypeDef,
    pub rxPins: [ioTag_t; 3],
    pub txPins: [ioTag_t; 3],
    pub rcc: rccPeriphTag_t,
    pub af: uint8_t,
    pub irqn: uint8_t,
    pub txPriority: uint8_t,
    pub rxPriority: uint8_t,
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
// Configuration constants
// Count number of configured UARTs
pub type uartHardware_t = uartHardware_s;
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
// XXX Not required for full allocation
// uartDevice_t is an actual device instance.
// XXX Instances are allocated for uarts defined by USE_UARTx atm.
pub type uartDevice_t = uartDevice_s;
// set BASEPRI_MAX, with global memory barrier, returns true
#[inline]
unsafe extern "C" fn __basepriSetMemRetVal(mut prio: uint8_t) -> uint8_t {
    __set_BASEPRI_MAX(prio as libc::c_int);
    return 1 as libc::c_int as uint8_t;
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
 * Authors:
 * jflyper - Refactoring, cleanup and made pin-configurable
 * Dominic Clifton - Serial port abstraction, Separation of common STM32 code for cleanflight, various cleanups.
 * Hamasaki/Timecop - Initial baseflight code
*/
unsafe extern "C" fn uartSetBaudRate(mut instance: *mut serialPort_t,
                                     mut baudRate: uint32_t) {
    let mut uartPort: *mut uartPort_t = instance as *mut uartPort_t;
    (*uartPort).port.baudRate = baudRate;
    uartReconfigure(uartPort);
}
unsafe extern "C" fn uartSetMode(mut instance: *mut serialPort_t,
                                 mut mode: portMode_e) {
    let mut uartPort: *mut uartPort_t = instance as *mut uartPort_t;
    (*uartPort).port.mode = mode;
    uartReconfigure(uartPort);
}
#[no_mangle]
pub unsafe extern "C" fn uartTryStartTxDMA(mut s: *mut uartPort_t) {
    // uartTryStartTxDMA must be protected, since it is called from
    // uartWrite and handleUsartTxDma (an ISR).
    let mut __basepri_save: uint8_t = __get_BASEPRI() as uint8_t;
    let mut __ToDo: uint8_t =
        __basepriSetMemRetVal((((1 as libc::c_int) <<
                                    (4 as libc::c_int as
                                         libc::c_uint).wrapping_sub((7 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint).wrapping_sub(0x500
                                                                                                        as
                                                                                                        libc::c_int
                                                                                                        as
                                                                                                        uint32_t
                                                                                                        >>
                                                                                                        8
                                                                                                            as
                                                                                                            libc::c_int))
                                    |
                                    1 as libc::c_int &
                                        0xf as libc::c_int >>
                                            (7 as libc::c_int as
                                                 libc::c_uint).wrapping_sub(0x500
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                uint32_t
                                                                                >>
                                                                                8
                                                                                    as
                                                                                    libc::c_int))
                                   << 4 as libc::c_int & 0xf0 as libc::c_int)
                                  as uint8_t);
    while __ToDo != 0 {
        if (*(*s).txDMAChannel).CCR & 1 as libc::c_int as libc::c_uint != 0 {
            // DMA is already in progress
            return
        }
        // For F1 (and F4), there are cases that CNDTR (NDTR for F4) is non-zero upon TC interrupt.
        // We couldn't find out the root cause, so mask the case here.
        // F3 is not confirmed to be vulnerable, but not excluded as a safety.
        if !((*(*s).txDMAChannel).CNDTR != 0) {
            if (*s).port.txBufferHead == (*s).port.txBufferTail {
                // No more data to transmit.
                (*s).txDMAEmpty = 1 as libc::c_int != 0;
                return
            }
            // Start a new transaction.
            ::core::ptr::write_volatile(&mut (*(*s).txDMAChannel).CMAR as
                                            *mut uint32_t,
                                        &mut *(*s).port.txBuffer.offset((*s).port.txBufferTail
                                                                            as
                                                                            isize)
                                            as *mut uint8_t as uint32_t);
            if (*s).port.txBufferHead > (*s).port.txBufferTail {
                ::core::ptr::write_volatile(&mut (*(*s).txDMAChannel).CNDTR as
                                                *mut uint32_t,
                                            (*s).port.txBufferHead.wrapping_sub((*s).port.txBufferTail));
                (*s).port.txBufferTail = (*s).port.txBufferHead
            } else {
                ::core::ptr::write_volatile(&mut (*(*s).txDMAChannel).CNDTR as
                                                *mut uint32_t,
                                            (*s).port.txBufferSize.wrapping_sub((*s).port.txBufferTail));
                (*s).port.txBufferTail = 0 as libc::c_int as uint32_t
            }
            (*s).txDMAEmpty = 0 as libc::c_int != 0
        }
        // Possible premature TC case.
        DMA_Cmd((*s).txDMAChannel, ENABLE);
        __ToDo = 0 as libc::c_int as uint8_t
    };
}
unsafe extern "C" fn uartTotalRxBytesWaiting(mut instance:
                                                 *const serialPort_t)
 -> uint32_t {
    let mut s: *const uartPort_t = instance as *const uartPort_t;
    if !(*s).rxDMAChannel.is_null() {
        let mut rxDMAHead: uint32_t = (*(*s).rxDMAChannel).CNDTR;
        if rxDMAHead >= (*s).rxDMAPos {
            return rxDMAHead.wrapping_sub((*s).rxDMAPos)
        } else {
            return (*s).port.rxBufferSize.wrapping_add(rxDMAHead).wrapping_sub((*s).rxDMAPos)
        }
    }
    if (*s).port.rxBufferHead >= (*s).port.rxBufferTail {
        return (*s).port.rxBufferHead.wrapping_sub((*s).port.rxBufferTail)
    } else {
        return (*s).port.rxBufferSize.wrapping_add((*s).port.rxBufferHead).wrapping_sub((*s).port.rxBufferTail)
    };
}
unsafe extern "C" fn uartTotalTxBytesFree(mut instance: *const serialPort_t)
 -> uint32_t {
    let mut s: *const uartPort_t = instance as *const uartPort_t;
    let mut bytesUsed: uint32_t = 0;
    if (*s).port.txBufferHead >= (*s).port.txBufferTail {
        bytesUsed =
            (*s).port.txBufferHead.wrapping_sub((*s).port.txBufferTail)
    } else {
        bytesUsed =
            (*s).port.txBufferSize.wrapping_add((*s).port.txBufferHead).wrapping_sub((*s).port.txBufferTail)
    }
    if !(*s).txDMAChannel.is_null() {
        /*
         * When we queue up a DMA request, we advance the Tx buffer tail before the transfer finishes, so we must add
         * the remaining size of that in-progress transfer here instead:
         */
        bytesUsed =
            (bytesUsed as
                 libc::c_uint).wrapping_add((*(*s).txDMAChannel).CNDTR) as
                uint32_t as uint32_t;
        /*
         * If the Tx buffer is being written to very quickly, we might have advanced the head into the buffer
         * space occupied by the current DMA transfer. In that case the "bytesUsed" total will actually end up larger
         * than the total Tx buffer size, because we'll end up transmitting the same buffer region twice. (So we'll be
         * transmitting a garbage mixture of old and new bytes).
         *
         * Be kind to callers and pretend like our buffer can only ever be 100% full.
         */
        if bytesUsed >=
               (*s).port.txBufferSize.wrapping_sub(1 as libc::c_int as
                                                       libc::c_uint) {
            return 0 as libc::c_int as uint32_t
        }
    }
    return (*s).port.txBufferSize.wrapping_sub(1 as libc::c_int as
                                                   libc::c_uint).wrapping_sub(bytesUsed);
}
unsafe extern "C" fn isUartTransmitBufferEmpty(mut instance:
                                                   *const serialPort_t)
 -> bool {
    let mut s: *const uartPort_t = instance as *const uartPort_t;
    if !(*s).txDMAChannel.is_null() {
        return (*s).txDMAEmpty
    } else { return (*s).port.txBufferTail == (*s).port.txBufferHead };
}
unsafe extern "C" fn uartRead(mut instance: *mut serialPort_t) -> uint8_t {
    let mut ch: uint8_t = 0;
    let mut s: *mut uartPort_t = instance as *mut uartPort_t;
    if !(*s).rxDMAChannel.is_null() {
        ch =
            *(*s).port.rxBuffer.offset((*s).port.rxBufferSize.wrapping_sub((*s).rxDMAPos)
                                           as isize);
        (*s).rxDMAPos = (*s).rxDMAPos.wrapping_sub(1);
        if (*s).rxDMAPos == 0 as libc::c_int as libc::c_uint {
            (*s).rxDMAPos = (*s).port.rxBufferSize
        }
    } else {
        ch = *(*s).port.rxBuffer.offset((*s).port.rxBufferTail as isize);
        if (*s).port.rxBufferTail.wrapping_add(1 as libc::c_int as
                                                   libc::c_uint) >=
               (*s).port.rxBufferSize {
            (*s).port.rxBufferTail = 0 as libc::c_int as uint32_t
        } else {
            (*s).port.rxBufferTail = (*s).port.rxBufferTail.wrapping_add(1)
        }
    }
    return ch;
}
unsafe extern "C" fn uartWrite(mut instance: *mut serialPort_t,
                               mut ch: uint8_t) {
    let mut s: *mut uartPort_t = instance as *mut uartPort_t;
    ::core::ptr::write_volatile((*s).port.txBuffer.offset((*s).port.txBufferHead
                                                              as isize), ch);
    if (*s).port.txBufferHead.wrapping_add(1 as libc::c_int as libc::c_uint)
           >= (*s).port.txBufferSize {
        (*s).port.txBufferHead = 0 as libc::c_int as uint32_t
    } else { (*s).port.txBufferHead = (*s).port.txBufferHead.wrapping_add(1) }
    if !(*s).txDMAChannel.is_null() {
        uartTryStartTxDMA(s);
    } else {
        USART_ITConfig((*s).USARTx, 0x727 as libc::c_int as uint16_t, ENABLE);
    };
}
#[no_mangle]
pub static mut uartVTable: [serialPortVTable; 1] =
    unsafe {
        [{
             let mut init =
                 serialPortVTable{serialWrite:
                                      Some(uartWrite as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t,
                                                                    _:
                                                                        uint8_t)
                                                   -> ()),
                                  serialTotalRxWaiting:
                                      Some(uartTotalRxBytesWaiting as
                                               unsafe extern "C" fn(_:
                                                                        *const serialPort_t)
                                                   -> uint32_t),
                                  serialTotalTxFree:
                                      Some(uartTotalTxBytesFree as
                                               unsafe extern "C" fn(_:
                                                                        *const serialPort_t)
                                                   -> uint32_t),
                                  serialRead:
                                      Some(uartRead as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t)
                                                   -> uint8_t),
                                  serialSetBaudRate:
                                      Some(uartSetBaudRate as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t,
                                                                    _:
                                                                        uint32_t)
                                                   -> ()),
                                  isSerialTransmitBufferEmpty:
                                      Some(isUartTransmitBufferEmpty as
                                               unsafe extern "C" fn(_:
                                                                        *const serialPort_t)
                                                   -> bool),
                                  setMode:
                                      Some(uartSetMode as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t,
                                                                    _:
                                                                        portMode_e)
                                                   -> ()),
                                  setCtrlLineStateCb: None,
                                  setBaudRateCb: None,
                                  writeBuf: None,
                                  beginWrite: None,
                                  endWrite: None,};
             init
         }]
    };
// USART1 Rx/Tx IRQ Handler
#[no_mangle]
pub unsafe extern "C" fn USART1_IRQHandler() {
    let mut s: *mut uartPort_t =
        &mut (**uartDevmap.as_mut_ptr().offset(UARTDEV_1 as libc::c_int as
                                                   isize)).port;
    uartIrqHandler(s);
}
// USART2 Rx/Tx IRQ Handler
#[no_mangle]
pub unsafe extern "C" fn USART2_IRQHandler() {
    let mut s: *mut uartPort_t =
        &mut (**uartDevmap.as_mut_ptr().offset(UARTDEV_2 as libc::c_int as
                                                   isize)).port;
    uartIrqHandler(s);
}
// USART3 Rx/Tx IRQ Handler
#[no_mangle]
pub unsafe extern "C" fn USART3_IRQHandler() {
    let mut s: *mut uartPort_t =
        &mut (**uartDevmap.as_mut_ptr().offset(UARTDEV_3 as libc::c_int as
                                                   isize)).port;
    uartIrqHandler(s);
}
