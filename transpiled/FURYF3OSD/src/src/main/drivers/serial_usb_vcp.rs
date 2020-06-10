use ::libc;
extern "C" {
    /* *
  ******************************************************************************
  * @file    usb_init.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Initialization routines & global variables
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
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
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
    #[no_mangle]
    fn USB_Init();
    /* *
 ******************************************************************************
 * @file    hw_config.h
 * @author  MCD Application Team
 * @version V4.0.0
 * @date    21-January-2013
 * @brief   Hardware Configuration & Setup
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
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
//#include "platform_config.h"
    /* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
    /* Exported functions ------------------------------------------------------- */
    #[no_mangle]
    fn Set_System();
    #[no_mangle]
    fn Set_USBClock();
    #[no_mangle]
    fn USB_Interrupts_Config();
    #[no_mangle]
    fn CDC_Send_DATA(ptrBuffer: *const uint8_t, sendLength: uint32_t)
     -> uint32_t;
    // HJI
    #[no_mangle]
    fn CDC_Send_FreeBytes() -> uint32_t;
    #[no_mangle]
    fn CDC_Receive_DATA(recvBuf: *mut uint8_t, len: uint32_t) -> uint32_t;
    // HJI
    #[no_mangle]
    fn CDC_Receive_BytesAvailable() -> uint32_t;
    #[no_mangle]
    fn usbIsConfigured() -> uint8_t;
    // HJI
    #[no_mangle]
    fn usbIsConnected() -> uint8_t;
    // HJI
    #[no_mangle]
    fn CDC_BaudRate() -> uint32_t;
    #[no_mangle]
    fn CDC_SetCtrlLineStateCb(cb:
                                  Option<unsafe extern "C" fn(_:
                                                                  *mut libc::c_void,
                                                              _: uint16_t)
                                             -> ()>,
                              context: *mut libc::c_void);
    #[no_mangle]
    fn CDC_SetBaudRateCb(cb:
                             Option<unsafe extern "C" fn(_: *mut libc::c_void,
                                                         _: uint32_t) -> ()>,
                         context: *mut libc::c_void);
    #[no_mangle]
    fn millis() -> timeMs_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
// millisecond time
pub type timeMs_t = uint32_t;
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
pub struct vcpPort_t {
    pub port: serialPort_t,
    pub txBuf: [uint8_t; 20],
    pub txAt: uint8_t,
    pub buffering: bool,
}
static mut vcpPort: vcpPort_t =
    vcpPort_t{port:
                  serialPort_t{vTable: 0 as *const serialPortVTable,
                               mode: 0 as portMode_e,
                               options: SERIAL_NOT_INVERTED,
                               baudRate: 0,
                               rxBufferSize: 0,
                               txBufferSize: 0,
                               rxBuffer: 0 as *const uint8_t as *mut uint8_t,
                               txBuffer: 0 as *const uint8_t as *mut uint8_t,
                               rxBufferHead: 0,
                               rxBufferTail: 0,
                               txBufferHead: 0,
                               txBufferTail: 0,
                               rxCallback: None,
                               rxCallbackData:
                                   0 as *const libc::c_void as
                                       *mut libc::c_void,
                               identifier: 0,},
              txBuf: [0; 20],
              txAt: 0,
              buffering: false,};
unsafe extern "C" fn usbVcpSetBaudRate(mut instance: *mut serialPort_t,
                                       mut baudRate: uint32_t) {
    // TODO implement
}
unsafe extern "C" fn usbVcpSetMode(mut instance: *mut serialPort_t,
                                   mut mode: portMode_e) {
    // TODO implement
}
unsafe extern "C" fn usbVcpSetCtrlLineStateCb(mut instance: *mut serialPort_t,
                                              mut cb:
                                                  Option<unsafe extern "C" fn(_:
                                                                                  *mut libc::c_void,
                                                                              _:
                                                                                  uint16_t)
                                                             -> ()>,
                                              mut context:
                                                  *mut libc::c_void) {
    // Register upper driver control line state callback routine with USB driver
    CDC_SetCtrlLineStateCb(cb, context);
}
unsafe extern "C" fn usbVcpSetBaudRateCb(mut instance: *mut serialPort_t,
                                         mut cb:
                                             Option<unsafe extern "C" fn(_:
                                                                             *mut serialPort_t,
                                                                         _:
                                                                             uint32_t)
                                                        -> ()>,
                                         mut context: *mut serialPort_t) {
    // Register upper driver baud rate callback routine with USB driver
    CDC_SetBaudRateCb(::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                               *mut serialPort_t,
                                                                           _:
                                                                               uint32_t)
                                                          -> ()>,
                                               Option<unsafe extern "C" fn(_:
                                                                               *mut libc::c_void,
                                                                           _:
                                                                               uint32_t)
                                                          -> ()>>(cb),
                      context as *mut libc::c_void);
}
unsafe extern "C" fn isUsbVcpTransmitBufferEmpty(mut instance:
                                                     *const serialPort_t)
 -> bool {
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn usbVcpAvailable(mut instance: *const serialPort_t)
 -> uint32_t {
    return CDC_Receive_BytesAvailable();
}
unsafe extern "C" fn usbVcpRead(mut instance: *mut serialPort_t) -> uint8_t {
    let mut buf: [uint8_t; 1] = [0; 1];
    loop  {
        if CDC_Receive_DATA(buf.as_mut_ptr(), 1 as libc::c_int as uint32_t) !=
               0 {
            return buf[0 as libc::c_int as usize]
        }
    };
}
unsafe extern "C" fn usbVcpWriteBuf(mut instance: *mut serialPort_t,
                                    mut data: *const libc::c_void,
                                    mut count: libc::c_int) {
    if !(usbIsConnected() as libc::c_int != 0 &&
             usbIsConfigured() as libc::c_int != 0) {
        return
    }
    let mut start: uint32_t = millis();
    let mut p: *const uint8_t = data as *const uint8_t;
    while count > 0 as libc::c_int {
        let mut txed: uint32_t = CDC_Send_DATA(p, count as uint32_t);
        count =
            (count as libc::c_uint).wrapping_sub(txed) as libc::c_int as
                libc::c_int;
        p = p.offset(txed as isize);
        if millis().wrapping_sub(start) > 50 as libc::c_int as libc::c_uint {
            break ;
        }
    };
}
unsafe extern "C" fn usbVcpFlush(mut port: *mut vcpPort_t) -> bool {
    let mut count: uint32_t = (*port).txAt as uint32_t;
    (*port).txAt = 0 as libc::c_int as uint8_t;
    if count == 0 as libc::c_int as libc::c_uint {
        return 1 as libc::c_int != 0
    }
    if usbIsConnected() == 0 || usbIsConfigured() == 0 {
        return 0 as libc::c_int != 0
    }
    let mut start: uint32_t = millis();
    let mut p: *mut uint8_t = (*port).txBuf.as_mut_ptr();
    while count > 0 as libc::c_int as libc::c_uint {
        let mut txed: uint32_t = CDC_Send_DATA(p, count);
        count =
            (count as libc::c_uint).wrapping_sub(txed) as uint32_t as
                uint32_t;
        p = p.offset(txed as isize);
        if millis().wrapping_sub(start) > 50 as libc::c_int as libc::c_uint {
            break ;
        }
    }
    return count == 0 as libc::c_int as libc::c_uint;
}
unsafe extern "C" fn usbVcpWrite(mut instance: *mut serialPort_t,
                                 mut c: uint8_t) {
    let mut port: *mut vcpPort_t =
        ({
             let mut __mptr: *const serialPort_t = instance;
             (__mptr as
                  *mut libc::c_char).offset(-(0 as libc::c_ulong as isize)) as
                 *mut vcpPort_t
         });
    let fresh0 = (*port).txAt;
    (*port).txAt = (*port).txAt.wrapping_add(1);
    (*port).txBuf[fresh0 as usize] = c;
    if !(*port).buffering ||
           (*port).txAt as libc::c_ulong >=
               (::core::mem::size_of::<[uint8_t; 20]>() as
                    libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>()
                                                    as libc::c_ulong) {
        usbVcpFlush(port);
    };
}
unsafe extern "C" fn usbVcpBeginWrite(mut instance: *mut serialPort_t) {
    let mut port: *mut vcpPort_t =
        ({
             let mut __mptr: *const serialPort_t = instance;
             (__mptr as
                  *mut libc::c_char).offset(-(0 as libc::c_ulong as isize)) as
                 *mut vcpPort_t
         });
    (*port).buffering = 1 as libc::c_int != 0;
}
unsafe extern "C" fn usbTxBytesFree(mut instance: *const serialPort_t)
 -> uint32_t {
    return CDC_Send_FreeBytes();
}
unsafe extern "C" fn usbVcpEndWrite(mut instance: *mut serialPort_t) {
    let mut port: *mut vcpPort_t =
        ({
             let mut __mptr: *const serialPort_t = instance;
             (__mptr as
                  *mut libc::c_char).offset(-(0 as libc::c_ulong as isize)) as
                 *mut vcpPort_t
         });
    (*port).buffering = 0 as libc::c_int != 0;
    usbVcpFlush(port);
}
static mut usbVTable: [serialPortVTable; 1] =
    unsafe {
        [{
             let mut init =
                 serialPortVTable{serialWrite:
                                      Some(usbVcpWrite as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t,
                                                                    _:
                                                                        uint8_t)
                                                   -> ()),
                                  serialTotalRxWaiting:
                                      Some(usbVcpAvailable as
                                               unsafe extern "C" fn(_:
                                                                        *const serialPort_t)
                                                   -> uint32_t),
                                  serialTotalTxFree:
                                      Some(usbTxBytesFree as
                                               unsafe extern "C" fn(_:
                                                                        *const serialPort_t)
                                                   -> uint32_t),
                                  serialRead:
                                      Some(usbVcpRead as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t)
                                                   -> uint8_t),
                                  serialSetBaudRate:
                                      Some(usbVcpSetBaudRate as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t,
                                                                    _:
                                                                        uint32_t)
                                                   -> ()),
                                  isSerialTransmitBufferEmpty:
                                      Some(isUsbVcpTransmitBufferEmpty as
                                               unsafe extern "C" fn(_:
                                                                        *const serialPort_t)
                                                   -> bool),
                                  setMode:
                                      Some(usbVcpSetMode as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t,
                                                                    _:
                                                                        portMode_e)
                                                   -> ()),
                                  setCtrlLineStateCb:
                                      Some(usbVcpSetCtrlLineStateCb as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t,
                                                                    _:
                                                                        Option<unsafe extern "C" fn(_:
                                                                                                        *mut libc::c_void,
                                                                                                    _:
                                                                                                        uint16_t)
                                                                                   ->
                                                                                       ()>,
                                                                    _:
                                                                        *mut libc::c_void)
                                                   -> ()),
                                  setBaudRateCb:
                                      Some(usbVcpSetBaudRateCb as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t,
                                                                    _:
                                                                        Option<unsafe extern "C" fn(_:
                                                                                                        *mut serialPort_t,
                                                                                                    _:
                                                                                                        uint32_t)
                                                                                   ->
                                                                                       ()>,
                                                                    _:
                                                                        *mut serialPort_t)
                                                   -> ()),
                                  writeBuf:
                                      Some(usbVcpWriteBuf as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t,
                                                                    _:
                                                                        *const libc::c_void,
                                                                    _:
                                                                        libc::c_int)
                                                   -> ()),
                                  beginWrite:
                                      Some(usbVcpBeginWrite as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t)
                                                   -> ()),
                                  endWrite:
                                      Some(usbVcpEndWrite as
                                               unsafe extern "C" fn(_:
                                                                        *mut serialPort_t)
                                                   -> ()),};
             init
         }]
    };
#[no_mangle]
pub unsafe extern "C" fn usbVcpOpen() -> *mut serialPort_t {
    let mut s: *mut vcpPort_t = 0 as *mut vcpPort_t;
    Set_System();
    Set_USBClock();
    USB_Init();
    USB_Interrupts_Config();
    s = &mut vcpPort;
    (*s).port.vTable = usbVTable.as_ptr();
    return s as *mut serialPort_t;
}
#[no_mangle]
pub unsafe extern "C" fn usbVcpGetBaudRate(mut instance: *mut serialPort_t)
 -> uint32_t {
    return CDC_BaudRate();
}
#[no_mangle]
pub unsafe extern "C" fn usbVcpIsConnected() -> uint8_t {
    return usbIsConnected();
}
