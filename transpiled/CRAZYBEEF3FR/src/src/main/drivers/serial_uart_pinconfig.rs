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
pub type size_t = libc::c_ulong;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPinConfig_s {
    pub ioTagTx: [ioTag_t; 10],
    pub ioTagRx: [ioTag_t; 10],
    pub ioTagInverter: [ioTag_t; 10],
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
    pub txDMAChannel: *mut DMA_Channel_TypeDef,
    pub rxDMAChannel: *mut DMA_Channel_TypeDef,
    pub rxPins: [ioTag_t; 4],
    pub txPins: [ioTag_t; 4],
    pub rcc: rccPeriphTag_t,
    pub af: uint8_t,
    pub irqn: uint8_t,
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
pub static mut uartDevice: [uartDevice_t; 1] =
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
                                 rxDMAChannel:
                                     0 as *const DMA_Channel_TypeDef as
                                         *mut DMA_Channel_TypeDef,
                                 txDMAChannel:
                                     0 as *const DMA_Channel_TypeDef as
                                         *mut DMA_Channel_TypeDef,
                                 rxDMAIrq: 0,
                                 txDMAIrq: 0,
                                 rxDMAPos: 0,
                                 txDMAPeripheralBaseAddr: 0,
                                 rxDMAPeripheralBaseAddr: 0,
                                 USARTx:
                                     0 as *const USART_TypeDef as
                                         *mut USART_TypeDef,
                                 txDMAEmpty: false,},
                  hardware: 0 as *const uartHardware_t,
                  rx: 0,
                  tx: 0,
                  rxBuffer: [0; 128],
                  txBuffer: [0; 256],}; 1];
// Only those configured in target.h
#[no_mangle]
pub static mut uartDevmap: [*mut uartDevice_t; 5] =
    [0 as *const uartDevice_t as *mut uartDevice_t; 5];
// Full array
#[no_mangle]
pub unsafe extern "C" fn uartPinConfigure(mut pSerialPinConfig:
                                              *const serialPinConfig_t) {
    let mut uartdev: *mut uartDevice_t = uartDevice.as_mut_ptr();
    let mut hindex: size_t = 0 as libc::c_int as size_t;
    while hindex <
              (0 as libc::c_int + 0 as libc::c_int + 1 as libc::c_int +
                   0 as libc::c_int + 0 as libc::c_int + 0 as libc::c_int +
                   0 as libc::c_int + 0 as libc::c_int) as libc::c_ulong {
        let mut hardware: *const uartHardware_t =
            &*uartHardware.as_ptr().offset(hindex as isize) as
                *const uartHardware_t;
        let device: UARTDevice_e = (*hardware).device;
        let mut pindex: libc::c_int = 0 as libc::c_int;
        while pindex < 4 as libc::c_int {
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
