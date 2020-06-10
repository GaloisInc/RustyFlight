use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
#[no_mangle]
pub unsafe extern "C" fn serialPrint(mut instance: *mut serialPort_t,
                                     mut str: *const libc::c_char) {
    let mut ch: uint8_t = 0;
    loop  {
        let fresh0 = str;
        str = str.offset(1);
        ch = *fresh0 as uint8_t;
        if !(ch as libc::c_int != 0 as libc::c_int) { break ; }
        serialWrite(instance, ch);
    };
}
#[no_mangle]
pub unsafe extern "C" fn serialGetBaudRate(mut instance: *mut serialPort_t)
 -> uint32_t {
    return (*instance).baudRate;
}
#[no_mangle]
pub unsafe extern "C" fn serialWrite(mut instance: *mut serialPort_t,
                                     mut ch: uint8_t) {
    (*(*instance).vTable).serialWrite.expect("non-null function pointer")(instance,
                                                                          ch);
}
#[no_mangle]
pub unsafe extern "C" fn serialWriteBuf(mut instance: *mut serialPort_t,
                                        mut data: *const uint8_t,
                                        mut count: libc::c_int) {
    if (*(*instance).vTable).writeBuf.is_some() {
        (*(*instance).vTable).writeBuf.expect("non-null function pointer")(instance,
                                                                           data
                                                                               as
                                                                               *const libc::c_void,
                                                                           count);
    } else {
        let mut p: *const uint8_t = data;
        while count > 0 as libc::c_int {
            while serialTxBytesFree(instance) == 0 { }
            serialWrite(instance, *p);
            count -= 1;
            p = p.offset(1)
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn serialRxBytesWaiting(mut instance:
                                                  *const serialPort_t)
 -> uint32_t {
    return (*(*instance).vTable).serialTotalRxWaiting.expect("non-null function pointer")(instance);
}
#[no_mangle]
pub unsafe extern "C" fn serialTxBytesFree(mut instance: *const serialPort_t)
 -> uint32_t {
    return (*(*instance).vTable).serialTotalTxFree.expect("non-null function pointer")(instance);
}
// Specified baud rate may not be allowed by an implementation, use serialGetBaudRate to determine actual baud rate in use.
// Optional functions used to buffer large writes.
#[no_mangle]
pub unsafe extern "C" fn serialRead(mut instance: *mut serialPort_t)
 -> uint8_t {
    return (*(*instance).vTable).serialRead.expect("non-null function pointer")(instance);
}
#[no_mangle]
pub unsafe extern "C" fn serialSetBaudRate(mut instance: *mut serialPort_t,
                                           mut baudRate: uint32_t) {
    (*(*instance).vTable).serialSetBaudRate.expect("non-null function pointer")(instance,
                                                                                baudRate);
}
#[no_mangle]
pub unsafe extern "C" fn isSerialTransmitBufferEmpty(mut instance:
                                                         *const serialPort_t)
 -> bool {
    return (*(*instance).vTable).isSerialTransmitBufferEmpty.expect("non-null function pointer")(instance);
}
#[no_mangle]
pub unsafe extern "C" fn serialSetMode(mut instance: *mut serialPort_t,
                                       mut mode: portMode_e) {
    (*(*instance).vTable).setMode.expect("non-null function pointer")(instance,
                                                                      mode);
}
#[no_mangle]
pub unsafe extern "C" fn serialSetCtrlLineStateCb(mut serialPort:
                                                      *mut serialPort_t,
                                                  mut cb:
                                                      Option<unsafe extern "C" fn(_:
                                                                                      *mut libc::c_void,
                                                                                  _:
                                                                                      uint16_t)
                                                                 -> ()>,
                                                  mut context:
                                                      *mut libc::c_void) {
    // If a callback routine for changes to control line state is supported by the underlying
    // driver, then set the callback.
    if (*(*serialPort).vTable).setCtrlLineStateCb.is_some() {
        (*(*serialPort).vTable).setCtrlLineStateCb.expect("non-null function pointer")(serialPort,
                                                                                       cb,
                                                                                       context);
    };
}
#[no_mangle]
pub unsafe extern "C" fn serialSetBaudRateCb(mut serialPort:
                                                 *mut serialPort_t,
                                             mut cb:
                                                 Option<unsafe extern "C" fn(_:
                                                                                 *mut serialPort_t,
                                                                             _:
                                                                                 uint32_t)
                                                            -> ()>,
                                             mut context: *mut serialPort_t) {
    // If a callback routine for changes to baud rate is supported by the underlying
    // driver, then set the callback.
    if (*(*serialPort).vTable).setBaudRateCb.is_some() {
        (*(*serialPort).vTable).setBaudRateCb.expect("non-null function pointer")(serialPort,
                                                                                  cb,
                                                                                  context);
    };
}
// A shim that adapts the bufWriter API to the serialWriteBuf() API.
#[no_mangle]
pub unsafe extern "C" fn serialWriteBufShim(mut instance: *mut libc::c_void,
                                            mut data: *const uint8_t,
                                            mut count: libc::c_int) {
    serialWriteBuf(instance as *mut serialPort_t, data, count);
}
#[no_mangle]
pub unsafe extern "C" fn serialBeginWrite(mut instance: *mut serialPort_t) {
    if (*(*instance).vTable).beginWrite.is_some() {
        (*(*instance).vTable).beginWrite.expect("non-null function pointer")(instance);
    };
}
#[no_mangle]
pub unsafe extern "C" fn serialEndWrite(mut instance: *mut serialPort_t) {
    if (*(*instance).vTable).endWrite.is_some() {
        (*(*instance).vTable).endWrite.expect("non-null function pointer")(instance);
    };
}
