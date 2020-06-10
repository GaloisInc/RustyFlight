use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn delay(ms: timeMs_t);
    #[no_mangle]
    static mut serialPinConfig_System: serialPinConfig_t;
    #[no_mangle]
    fn serialWrite(instance: *mut serialPort_t, ch: uint8_t);
    #[no_mangle]
    fn serialRxBytesWaiting(instance: *const serialPort_t) -> uint32_t;
    #[no_mangle]
    fn serialTxBytesFree(instance: *const serialPort_t) -> uint32_t;
    #[no_mangle]
    fn serialRead(instance: *mut serialPort_t) -> uint8_t;
    #[no_mangle]
    fn isSerialTransmitBufferEmpty(instance: *const serialPort_t) -> bool;
    #[no_mangle]
    fn uartOpen(device: UARTDevice_e, rxCallback: serialReceiveCallbackPtr,
                rxCallbackData: *mut libc::c_void, baudRate: uint32_t,
                mode: portMode_e, options: portOptions_e)
     -> *mut serialPort_t;
    #[no_mangle]
    fn ledSet(led: libc::c_int, state: bool);
    #[no_mangle]
    fn telemetryCheckRxPortShared(portConfig: *const serialPortConfig_t)
     -> bool;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
pub type pgn_t = uint16_t;
pub type C2RustUnnamed = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed = 4095;
// function that resets a single parameter group instance
pub type pgResetFunc
    =
    unsafe extern "C" fn(_: *mut libc::c_void, _: libc::c_int) -> ();
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pgRegistry_s {
    pub pgn: pgn_t,
    pub size: uint16_t,
    pub address: *mut uint8_t,
    pub copy: *mut uint8_t,
    pub ptr: *mut *mut uint8_t,
    pub reset: C2RustUnnamed_0,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_0 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<pgResetFunc>,
}
pub type pgRegistry_t = pgRegistry_s;
/* base */
/* size */
// The parameter group number, the top 4 bits are reserved for version
// Size of the group in RAM, the top 4 bits are reserved for flags
// Address of the group in RAM.
// Address of the copy in RAM.
// The pointer to update after loading the record into ram.
// Pointer to init template
// Pointer to pgResetFunc
// millisecond time
pub type timeMs_t = uint32_t;
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
pub type portSharing_e = libc::c_uint;
pub const PORTSHARING_SHARED: portSharing_e = 2;
pub const PORTSHARING_NOT_SHARED: portSharing_e = 1;
pub const PORTSHARING_UNUSED: portSharing_e = 0;
pub type serialPortFunction_e = libc::c_uint;
pub const FUNCTION_LIDAR_TF: serialPortFunction_e = 32768;
pub const FUNCTION_RCDEVICE: serialPortFunction_e = 16384;
pub const FUNCTION_VTX_TRAMP: serialPortFunction_e = 8192;
pub const FUNCTION_TELEMETRY_IBUS: serialPortFunction_e = 4096;
pub const FUNCTION_VTX_SMARTAUDIO: serialPortFunction_e = 2048;
pub const FUNCTION_ESC_SENSOR: serialPortFunction_e = 1024;
pub const FUNCTION_TELEMETRY_MAVLINK: serialPortFunction_e = 512;
pub const FUNCTION_BLACKBOX: serialPortFunction_e = 128;
pub const FUNCTION_RX_SERIAL: serialPortFunction_e = 64;
pub const FUNCTION_TELEMETRY_SMARTPORT: serialPortFunction_e = 32;
pub const FUNCTION_TELEMETRY_LTM: serialPortFunction_e = 16;
pub const FUNCTION_TELEMETRY_HOTT: serialPortFunction_e = 8;
pub const FUNCTION_TELEMETRY_FRSKY_HUB: serialPortFunction_e = 4;
pub const FUNCTION_GPS: serialPortFunction_e = 2;
pub const FUNCTION_MSP: serialPortFunction_e = 1;
pub const FUNCTION_NONE: serialPortFunction_e = 0;
pub type baudRate_e = libc::c_uint;
pub const BAUD_2470000: baudRate_e = 15;
pub const BAUD_2000000: baudRate_e = 14;
pub const BAUD_1500000: baudRate_e = 13;
pub const BAUD_1000000: baudRate_e = 12;
pub const BAUD_921600: baudRate_e = 11;
pub const BAUD_500000: baudRate_e = 10;
pub const BAUD_460800: baudRate_e = 9;
pub const BAUD_400000: baudRate_e = 8;
pub const BAUD_250000: baudRate_e = 7;
pub const BAUD_230400: baudRate_e = 6;
pub const BAUD_115200: baudRate_e = 5;
pub const BAUD_57600: baudRate_e = 4;
pub const BAUD_38400: baudRate_e = 3;
pub const BAUD_19200: baudRate_e = 2;
pub const BAUD_9600: baudRate_e = 1;
pub const BAUD_AUTO: baudRate_e = 0;
pub type serialPortIdentifier_e = libc::c_int;
pub const SERIAL_PORT_SOFTSERIAL2: serialPortIdentifier_e = 31;
pub const SERIAL_PORT_SOFTSERIAL1: serialPortIdentifier_e = 30;
pub const SERIAL_PORT_USB_VCP: serialPortIdentifier_e = 20;
pub const SERIAL_PORT_USART8: serialPortIdentifier_e = 7;
pub const SERIAL_PORT_USART7: serialPortIdentifier_e = 6;
pub const SERIAL_PORT_USART6: serialPortIdentifier_e = 5;
pub const SERIAL_PORT_UART5: serialPortIdentifier_e = 4;
pub const SERIAL_PORT_UART4: serialPortIdentifier_e = 3;
pub const SERIAL_PORT_USART3: serialPortIdentifier_e = 2;
pub const SERIAL_PORT_USART2: serialPortIdentifier_e = 1;
pub const SERIAL_PORT_USART1: serialPortIdentifier_e = 0;
pub const SERIAL_PORT_NONE: serialPortIdentifier_e = -1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPortUsage_s {
    pub serialPort: *mut serialPort_t,
    pub function: serialPortFunction_e,
    pub identifier: serialPortIdentifier_e,
}
//
// runtime
//
pub type serialPortUsage_t = serialPortUsage_s;
//
// configuration
//
pub type serialPortConfig_t = serialPortConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPortConfig_s {
    pub functionMask: uint16_t,
    pub identifier: serialPortIdentifier_e,
    pub msp_baudrateIndex: uint8_t,
    pub gps_baudrateIndex: uint8_t,
    pub blackbox_baudrateIndex: uint8_t,
    pub telemetry_baudrateIndex: uint8_t,
}
pub type serialConfig_t = serialConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialConfig_s {
    pub portConfigs: [serialPortConfig_t; 2],
    pub serial_update_rate_hz: uint16_t,
    pub reboot_character: uint8_t,
}
pub type serialConsumer = unsafe extern "C" fn(_: uint8_t) -> ();
pub type findSerialPortConfigState_t = findSerialPortConfigState_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct findSerialPortConfigState_s {
    pub lastIndex: uint8_t,
}
#[inline]
unsafe extern "C" fn serialPinConfig() -> *const serialPinConfig_t {
    return &mut serialPinConfig_System;
}
#[inline]
unsafe extern "C" fn serialConfig() -> *const serialConfig_t {
    return &mut serialConfig_System;
}
#[inline]
unsafe extern "C" fn serialConfigMutable() -> *mut serialConfig_t {
    return &mut serialConfig_System;
}
// not used for all telemetry systems, e.g. HoTT only works at 19200.
// which byte is used to reboot. Default 'R', could be changed carefully to something else.
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
static mut serialPortUsageList: [serialPortUsage_t; 2] =
    [serialPortUsage_t{serialPort:
                           0 as *const serialPort_t as *mut serialPort_t,
                       function: FUNCTION_NONE,
                       identifier: SERIAL_PORT_USART1,}; 2];
#[no_mangle]
pub static mut serialPortIdentifiers: [serialPortIdentifier_e; 2] =
    [SERIAL_PORT_USART1, SERIAL_PORT_USART2];
static mut serialPortCount: uint8_t = 0;
#[no_mangle]
pub static mut baudRates: [uint32_t; 16] =
    [0 as libc::c_int as uint32_t, 9600 as libc::c_int as uint32_t,
     19200 as libc::c_int as uint32_t, 38400 as libc::c_int as uint32_t,
     57600 as libc::c_int as uint32_t, 115200 as libc::c_int as uint32_t,
     230400 as libc::c_int as uint32_t, 250000 as libc::c_int as uint32_t,
     400000 as libc::c_int as uint32_t, 460800 as libc::c_int as uint32_t,
     500000 as libc::c_int as uint32_t, 921600 as libc::c_int as uint32_t,
     1000000 as libc::c_int as uint32_t, 1500000 as libc::c_int as uint32_t,
     2000000 as libc::c_int as uint32_t, 2470000 as libc::c_int as uint32_t];
#[no_mangle]
pub static mut serialConfig_System: serialConfig_t =
    serialConfig_t{portConfigs:
                       [serialPortConfig_t{functionMask: 0,
                                           identifier: SERIAL_PORT_USART1,
                                           msp_baudrateIndex: 0,
                                           gps_baudrateIndex: 0,
                                           blackbox_baudrateIndex: 0,
                                           telemetry_baudrateIndex: 0,}; 2],
                   serial_update_rate_hz: 0,
                   reboot_character: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut serialConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (13 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<serialConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &serialConfig_System as *const serialConfig_t
                                     as *mut serialConfig_t as *mut uint8_t,
                             copy:
                                 &serialConfig_Copy as *const serialConfig_t
                                     as *mut serialConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut serialConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<pgResetFunc>>(Some(pgResetFn_serialConfig
                                                                                                            as
                                                                                                            unsafe extern "C" fn(_:
                                                                                                                                     *mut serialConfig_t)
                                                                                                                ->
                                                                                                                    ())),},};
            init
        }
    };
#[no_mangle]
pub static mut serialConfig_Copy: serialConfig_t =
    serialConfig_t{portConfigs:
                       [serialPortConfig_t{functionMask: 0,
                                           identifier: SERIAL_PORT_USART1,
                                           msp_baudrateIndex: 0,
                                           gps_baudrateIndex: 0,
                                           blackbox_baudrateIndex: 0,
                                           telemetry_baudrateIndex: 0,}; 2],
                   serial_update_rate_hz: 0,
                   reboot_character: 0,};
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_serialConfig(mut serialConfig_0:
                                                    *mut serialConfig_t) {
    memset(serialConfig_0 as *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<serialConfig_t>() as libc::c_ulong);
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 2 as libc::c_int {
        (*serialConfig_0).portConfigs[i as usize].identifier =
            serialPortIdentifiers[i as usize];
        (*serialConfig_0).portConfigs[i as usize].msp_baudrateIndex =
            BAUD_115200 as libc::c_int as uint8_t;
        (*serialConfig_0).portConfigs[i as usize].gps_baudrateIndex =
            BAUD_57600 as libc::c_int as uint8_t;
        (*serialConfig_0).portConfigs[i as usize].telemetry_baudrateIndex =
            BAUD_AUTO as libc::c_int as uint8_t;
        (*serialConfig_0).portConfigs[i as usize].blackbox_baudrateIndex =
            BAUD_115200 as libc::c_int as uint8_t;
        i += 1
    }
    (*serialConfig_0).portConfigs[0 as libc::c_int as usize].functionMask =
        FUNCTION_MSP as libc::c_int as uint16_t;
    let mut serialRxUartConfig: *mut serialPortConfig_t =
        serialFindPortConfiguration(SERIAL_PORT_USART2);
    if !serialRxUartConfig.is_null() {
        (*serialRxUartConfig).functionMask =
            FUNCTION_RX_SERIAL as libc::c_int as uint16_t
    }
    (*serialConfig_0).reboot_character = 'R' as i32 as uint8_t;
    (*serialConfig_0).serial_update_rate_hz = 100 as libc::c_int as uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn lookupBaudRateIndex(mut baudRate: uint32_t)
 -> baudRate_e {
    let mut index: uint8_t = 0;
    index = 0 as libc::c_int as uint8_t;
    while (index as libc::c_ulong) <
              (::core::mem::size_of::<[uint32_t; 16]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<uint32_t>()
                                                   as libc::c_ulong) {
        if baudRates[index as usize] == baudRate {
            return index as baudRate_e
        }
        index = index.wrapping_add(1)
    }
    return BAUD_AUTO;
}
#[no_mangle]
pub unsafe extern "C" fn findSerialPortIndexByIdentifier(mut identifier:
                                                             serialPortIdentifier_e)
 -> libc::c_int {
    let mut index: libc::c_int = 0 as libc::c_int;
    while index < 2 as libc::c_int {
        if serialPortIdentifiers[index as usize] as libc::c_int ==
               identifier as libc::c_int {
            return index
        }
        index += 1
    }
    return -(1 as libc::c_int);
}
// !!TODO remove need for this
#[no_mangle]
pub unsafe extern "C" fn findSerialPortUsageByIdentifier(mut identifier:
                                                             serialPortIdentifier_e)
 -> *mut serialPortUsage_t {
    let mut index: uint8_t = 0;
    index = 0 as libc::c_int as uint8_t;
    while (index as libc::c_int) < 2 as libc::c_int {
        let mut candidate: *mut serialPortUsage_t =
            &mut *serialPortUsageList.as_mut_ptr().offset(index as isize) as
                *mut serialPortUsage_t;
        if (*candidate).identifier as libc::c_int == identifier as libc::c_int
           {
            return candidate
        }
        index = index.wrapping_add(1)
    }
    return 0 as *mut serialPortUsage_t;
}
#[no_mangle]
pub unsafe extern "C" fn findSerialPortUsageByPort(mut serialPort:
                                                       *mut serialPort_t)
 -> *mut serialPortUsage_t {
    let mut index: uint8_t = 0;
    index = 0 as libc::c_int as uint8_t;
    while (index as libc::c_int) < 2 as libc::c_int {
        let mut candidate: *mut serialPortUsage_t =
            &mut *serialPortUsageList.as_mut_ptr().offset(index as isize) as
                *mut serialPortUsage_t;
        if (*candidate).serialPort == serialPort { return candidate }
        index = index.wrapping_add(1)
    }
    return 0 as *mut serialPortUsage_t;
}
static mut findSerialPortConfigState: findSerialPortConfigState_t =
    findSerialPortConfigState_t{lastIndex: 0,};
#[no_mangle]
pub unsafe extern "C" fn findSerialPortConfig(mut function:
                                                  serialPortFunction_e)
 -> *mut serialPortConfig_t {
    memset(&mut findSerialPortConfigState as *mut findSerialPortConfigState_t
               as *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<findSerialPortConfigState_t>() as
               libc::c_ulong);
    return findNextSerialPortConfig(function);
}
#[no_mangle]
pub unsafe extern "C" fn findNextSerialPortConfig(mut function:
                                                      serialPortFunction_e)
 -> *mut serialPortConfig_t {
    while (findSerialPortConfigState.lastIndex as libc::c_int) <
              2 as libc::c_int {
        let fresh0 = findSerialPortConfigState.lastIndex;
        findSerialPortConfigState.lastIndex =
            findSerialPortConfigState.lastIndex.wrapping_add(1);
        let mut candidate: *mut serialPortConfig_t =
            &mut *(*(serialConfigMutable as
                         unsafe extern "C" fn()
                             ->
                                 *mut serialConfig_t)()).portConfigs.as_mut_ptr().offset(fresh0
                                                                                             as
                                                                                             isize)
                as *mut serialPortConfig_t;
        if (*candidate).functionMask as libc::c_uint &
               function as libc::c_uint != 0 {
            return candidate
        }
    }
    return 0 as *mut serialPortConfig_t;
}
#[no_mangle]
pub unsafe extern "C" fn determinePortSharing(mut portConfig:
                                                  *const serialPortConfig_t,
                                              mut function:
                                                  serialPortFunction_e)
 -> portSharing_e {
    if portConfig.is_null() ||
           (*portConfig).functionMask as libc::c_uint &
               function as libc::c_uint == 0 as libc::c_int as libc::c_uint {
        return PORTSHARING_UNUSED
    }
    return if (*portConfig).functionMask as libc::c_uint ==
                  function as libc::c_uint {
               PORTSHARING_NOT_SHARED as libc::c_int
           } else { PORTSHARING_SHARED as libc::c_int } as portSharing_e;
}
#[no_mangle]
pub unsafe extern "C" fn isSerialPortShared(mut portConfig:
                                                *const serialPortConfig_t,
                                            mut functionMask: uint16_t,
                                            mut sharedWithFunction:
                                                serialPortFunction_e)
 -> bool {
    return !portConfig.is_null() &&
               (*portConfig).functionMask as libc::c_uint &
                   sharedWithFunction as libc::c_uint != 0 &&
               (*portConfig).functionMask as libc::c_int &
                   functionMask as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn findSharedSerialPort(mut functionMask: uint16_t,
                                              mut sharedWithFunction:
                                                  serialPortFunction_e)
 -> *mut serialPort_t {
    let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while i < 2 as libc::c_int as libc::c_uint {
        let mut candidate: *const serialPortConfig_t =
            &*(*(serialConfig as
                     unsafe extern "C" fn()
                         ->
                             *const serialConfig_t)()).portConfigs.as_ptr().offset(i
                                                                                       as
                                                                                       isize)
                as *const serialPortConfig_t;
        if isSerialPortShared(candidate, functionMask, sharedWithFunction) {
            let mut serialPortUsage: *const serialPortUsage_t =
                findSerialPortUsageByIdentifier((*candidate).identifier);
            if !serialPortUsage.is_null() {
                return (*serialPortUsage).serialPort
            }
        }
        i = i.wrapping_add(1)
    }
    return 0 as *mut serialPort_t;
}
#[no_mangle]
pub unsafe extern "C" fn isSerialConfigValid(mut serialConfigToCheck:
                                                 *const serialConfig_t)
 -> bool {
    /*
     * rules:
     * - 1 MSP port minimum, max MSP ports is defined and must be adhered to.
     * - MSP is allowed to be shared with EITHER any telemetry OR blackbox.
     *   (using either / or, switching based on armed / disarmed or the AUX channel configured for BOXTELEMETRY)
     * - serial RX and FrSky / LTM / MAVLink telemetry can be shared
     *   (serial RX using RX line, telemetry using TX line)
     * - No other sharing combinations are valid.
     */
    let mut mspPortCount: uint8_t = 0 as libc::c_int as uint8_t;
    let mut index: libc::c_int = 0 as libc::c_int;
    while index < 2 as libc::c_int {
        let mut portConfig: *const serialPortConfig_t =
            &*(*serialConfigToCheck).portConfigs.as_ptr().offset(index as
                                                                     isize) as
                *const serialPortConfig_t;
        if (*portConfig).functionMask as libc::c_int &
               FUNCTION_MSP as libc::c_int != 0 {
            mspPortCount = mspPortCount.wrapping_add(1)
        }
        let mut bitCount: uint8_t =
            (((*portConfig).functionMask as libc::c_int -
                  ((*portConfig).functionMask as libc::c_int >>
                       1 as libc::c_int & 0x77777777 as libc::c_int) -
                  ((*portConfig).functionMask as libc::c_int >>
                       2 as libc::c_int & 0x33333333 as libc::c_int) -
                  ((*portConfig).functionMask as libc::c_int >>
                       3 as libc::c_int & 0x11111111 as libc::c_int) +
                  ((*portConfig).functionMask as libc::c_int -
                       ((*portConfig).functionMask as libc::c_int >>
                            1 as libc::c_int & 0x77777777 as libc::c_int) -
                       ((*portConfig).functionMask as libc::c_int >>
                            2 as libc::c_int & 0x33333333 as libc::c_int) -
                       ((*portConfig).functionMask as libc::c_int >>
                            3 as libc::c_int & 0x11111111 as libc::c_int) >>
                       4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
                 255 as libc::c_int) as uint8_t;
        if bitCount as libc::c_int > 1 as libc::c_int {
            // shared
            if bitCount as libc::c_int > 2 as libc::c_int {
                return 0 as libc::c_int != 0
            }
            if !((*portConfig).functionMask as libc::c_int &
                     FUNCTION_MSP as libc::c_int != 0 &&
                     (*portConfig).functionMask as libc::c_int &
                         (FUNCTION_BLACKBOX as libc::c_int |
                              (FUNCTION_TELEMETRY_FRSKY_HUB as libc::c_int |
                                   FUNCTION_TELEMETRY_LTM as libc::c_int |
                                   FUNCTION_TELEMETRY_MAVLINK as libc::c_int |
                                   FUNCTION_TELEMETRY_HOTT as libc::c_int |
                                   FUNCTION_TELEMETRY_SMARTPORT as
                                       libc::c_int)) != 0) {
                if telemetryCheckRxPortShared(portConfig) {
                } else {
                    // some other combination
                    return 0 as libc::c_int != 0
                }
            }
        }
        index += 1
    }
    if mspPortCount as libc::c_int == 0 as libc::c_int ||
           mspPortCount as libc::c_int > 3 as libc::c_int {
        return 0 as libc::c_int != 0
    }
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn serialFindPortConfiguration(mut identifier:
                                                         serialPortIdentifier_e)
 -> *mut serialPortConfig_t {
    let mut index: libc::c_int = 0 as libc::c_int;
    while index < 2 as libc::c_int {
        let mut candidate: *mut serialPortConfig_t =
            &mut *(*(serialConfigMutable as
                         unsafe extern "C" fn()
                             ->
                                 *mut serialConfig_t)()).portConfigs.as_mut_ptr().offset(index
                                                                                             as
                                                                                             isize)
                as *mut serialPortConfig_t;
        if (*candidate).identifier as libc::c_int == identifier as libc::c_int
           {
            return candidate
        }
        index += 1
    }
    return 0 as *mut serialPortConfig_t;
}
#[no_mangle]
pub unsafe extern "C" fn doesConfigurationUsePort(mut identifier:
                                                      serialPortIdentifier_e)
 -> bool {
    let mut candidate: *mut serialPortConfig_t =
        serialFindPortConfiguration(identifier);
    return !candidate.is_null() &&
               (*candidate).functionMask as libc::c_int != 0;
}
//
// runtime
//
#[no_mangle]
pub unsafe extern "C" fn openSerialPort(mut identifier:
                                            serialPortIdentifier_e,
                                        mut function: serialPortFunction_e,
                                        mut rxCallback:
                                            serialReceiveCallbackPtr,
                                        mut rxCallbackData: *mut libc::c_void,
                                        mut baudRate: uint32_t,
                                        mut mode: portMode_e,
                                        mut options: portOptions_e)
 -> *mut serialPort_t {
    let mut serialPortUsage: *mut serialPortUsage_t =
        findSerialPortUsageByIdentifier(identifier);
    if serialPortUsage.is_null() ||
           (*serialPortUsage).function as libc::c_uint !=
               FUNCTION_NONE as libc::c_int as libc::c_uint {
        // not available / already in use
        return 0 as *mut serialPort_t
    }
    let mut serialPort: *mut serialPort_t = 0 as *mut serialPort_t;
    match identifier as libc::c_int {
        0 | 1 => {
            serialPort =
                uartOpen((identifier as libc::c_int -
                              SERIAL_PORT_USART1 as libc::c_int +
                              UARTDEV_1 as libc::c_int) as UARTDevice_e,
                         rxCallback, rxCallbackData, baudRate, mode, options)
        }
        _ => { }
    }
    if serialPort.is_null() { return 0 as *mut serialPort_t }
    (*serialPort).identifier = identifier as uint8_t;
    (*serialPortUsage).function = function;
    (*serialPortUsage).serialPort = serialPort;
    return serialPort;
}
#[no_mangle]
pub unsafe extern "C" fn closeSerialPort(mut serialPort: *mut serialPort_t) {
    let mut serialPortUsage: *mut serialPortUsage_t =
        findSerialPortUsageByPort(serialPort);
    if serialPortUsage.is_null() {
        // already closed
        return
    }
    // TODO wait until data has been transmitted.
    (*serialPort).rxCallback = None;
    (*serialPortUsage).function = FUNCTION_NONE;
    (*serialPortUsage).serialPort = 0 as *mut serialPort_t;
}
//
// configuration
//
#[no_mangle]
pub unsafe extern "C" fn serialInit(mut softserialEnabled: bool,
                                    mut serialPortToDisable:
                                        serialPortIdentifier_e) {
    serialPortCount = 2 as libc::c_int as uint8_t;
    memset(&mut serialPortUsageList as *mut [serialPortUsage_t; 2] as
               *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<[serialPortUsage_t; 2]>() as libc::c_ulong);
    let mut index: libc::c_int = 0 as libc::c_int;
    while index < 2 as libc::c_int {
        serialPortUsageList[index as usize].identifier =
            serialPortIdentifiers[index as usize];
        if serialPortToDisable as libc::c_int !=
               SERIAL_PORT_NONE as libc::c_int {
            if serialPortUsageList[index as usize].identifier as libc::c_int
                   == serialPortToDisable as libc::c_int {
                serialPortUsageList[index as usize].identifier =
                    SERIAL_PORT_NONE;
                serialPortCount = serialPortCount.wrapping_sub(1)
            }
        } else if serialPortUsageList[index as usize].identifier as
                      libc::c_int <= SERIAL_PORT_USART8 as libc::c_int {
            let mut resourceIndex: libc::c_int =
                if serialPortUsageList[index as usize].identifier as
                       libc::c_int <= SERIAL_PORT_USART8 as libc::c_int {
                    serialPortUsageList[index as usize].identifier as
                        libc::c_int
                } else {
                    (10 as libc::c_int) +
                        (serialPortUsageList[index as usize].identifier as
                             libc::c_int -
                             SERIAL_PORT_SOFTSERIAL1 as libc::c_int)
                };
            if !((*serialPinConfig()).ioTagTx[resourceIndex as usize] as
                     libc::c_int != 0 ||
                     (*serialPinConfig()).ioTagRx[resourceIndex as usize] as
                         libc::c_int != 0) {
                serialPortUsageList[index as usize].identifier =
                    SERIAL_PORT_NONE;
                serialPortCount = serialPortCount.wrapping_sub(1)
            }
        } else if serialPortUsageList[index as usize].identifier as
                      libc::c_int == SERIAL_PORT_SOFTSERIAL1 as libc::c_int ||
                      serialPortUsageList[index as usize].identifier as
                          libc::c_int ==
                          SERIAL_PORT_SOFTSERIAL2 as libc::c_int {
            serialPortUsageList[index as usize].identifier = SERIAL_PORT_NONE;
            serialPortCount = serialPortCount.wrapping_sub(1)
        }
        index += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn serialRemovePort(mut identifier:
                                              serialPortIdentifier_e) {
    let mut index: uint8_t = 0 as libc::c_int as uint8_t;
    while (index as libc::c_int) < 2 as libc::c_int {
        if serialPortUsageList[index as usize].identifier as libc::c_int ==
               identifier as libc::c_int {
            serialPortUsageList[index as usize].identifier = SERIAL_PORT_NONE;
            serialPortCount = serialPortCount.wrapping_sub(1)
        }
        index = index.wrapping_add(1)
    };
}
#[no_mangle]
pub unsafe extern "C" fn serialGetAvailablePortCount() -> uint8_t {
    return serialPortCount;
}
#[no_mangle]
pub unsafe extern "C" fn serialIsPortAvailable(mut identifier:
                                                   serialPortIdentifier_e)
 -> bool {
    let mut index: uint8_t = 0 as libc::c_int as uint8_t;
    while (index as libc::c_int) < 2 as libc::c_int {
        if serialPortUsageList[index as usize].identifier as libc::c_int ==
               identifier as libc::c_int {
            return 1 as libc::c_int != 0
        }
        index = index.wrapping_add(1)
    }
    return 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn waitForSerialPortToFinishTransmitting(mut serialPort:
                                                                   *mut serialPort_t) {
    while !isSerialTransmitBufferEmpty(serialPort) {
        delay(10 as libc::c_int as timeMs_t);
    };
}
// Default data consumer for serialPassThrough.
unsafe extern "C" fn nopConsumer(mut data: uint8_t) { }
//
// msp/cli/bootloader
//
/*
 A high-level serial passthrough implementation. Used by cli to start an
 arbitrary serial passthrough "proxy". Optional callbacks can be given to allow
 for specialized data processing.
 */
#[no_mangle]
pub unsafe extern "C" fn serialPassthrough(mut left: *mut serialPort_t,
                                           mut right: *mut serialPort_t,
                                           mut leftC: Option<serialConsumer>,
                                           mut rightC:
                                               Option<serialConsumer>) {
    waitForSerialPortToFinishTransmitting(left);
    waitForSerialPortToFinishTransmitting(right);
    if leftC.is_none() {
        leftC = Some(nopConsumer as unsafe extern "C" fn(_: uint8_t) -> ())
    }
    if rightC.is_none() {
        rightC = Some(nopConsumer as unsafe extern "C" fn(_: uint8_t) -> ())
    }
    ledSet(0 as libc::c_int, 0 as libc::c_int != 0);
    ledSet(1 as libc::c_int, 0 as libc::c_int != 0);
    loop 
         // Either port might be open in a mode other than MODE_RXTX. We rely on
    // serialRxBytesWaiting() to do the right thing for a TX only port. No
    // special handling is necessary OR performed.
         // TODO: maintain a timestamp of last data received. Use this to
        // implement a guard interval and check for `+++` as an escape sequence
        // to return to CLI command mode.
        // https://en.wikipedia.org/wiki/Escape_sequence#Modem_control
         {
        if serialRxBytesWaiting(left) != 0 {
            ledSet(0 as libc::c_int, 1 as libc::c_int != 0);
            let mut c: uint8_t = serialRead(left);
            // Make sure there is space in the tx buffer
            while serialTxBytesFree(right) == 0 { }
            serialWrite(right, c);
            leftC.expect("non-null function pointer")(c);
            ledSet(0 as libc::c_int, 0 as libc::c_int != 0);
        }
        if serialRxBytesWaiting(right) != 0 {
            ledSet(0 as libc::c_int, 1 as libc::c_int != 0);
            let mut c_0: uint8_t = serialRead(right);
            // Make sure there is space in the tx buffer
            while serialTxBytesFree(left) == 0 { }
            serialWrite(left, c_0);
            rightC.expect("non-null function pointer")(c_0);
            ledSet(0 as libc::c_int, 0 as libc::c_int != 0);
        }
    };
}
