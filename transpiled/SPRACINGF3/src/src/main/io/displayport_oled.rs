use ::libc;
extern "C" {
    #[no_mangle]
    fn displayInit(instance: *mut displayPort_t,
                   vTable: *const displayPortVTable_t);
    #[no_mangle]
    fn i2c_OLED_set_xy(bus: *mut busDevice_t, col: uint8_t, row: uint8_t);
    #[no_mangle]
    fn i2c_OLED_send_char(bus: *mut busDevice_t, ascii: libc::c_uchar);
    #[no_mangle]
    fn i2c_OLED_send_string(bus: *mut busDevice_t,
                            string: *const libc::c_char);
    #[no_mangle]
    fn i2c_OLED_clear_display_quick(bus: *mut busDevice_t);
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SPI_TypeDef {
    pub CR1: uint16_t,
    pub RESERVED0: uint16_t,
    pub CR2: uint16_t,
    pub RESERVED1: uint16_t,
    pub SR: uint16_t,
    pub RESERVED2: uint16_t,
    pub DR: uint16_t,
    pub RESERVED3: uint16_t,
    pub CRCPR: uint16_t,
    pub RESERVED4: uint16_t,
    pub RXCRCR: uint16_t,
    pub RESERVED5: uint16_t,
    pub TXCRCR: uint16_t,
    pub RESERVED6: uint16_t,
    pub I2SCFGR: uint16_t,
    pub RESERVED7: uint16_t,
    pub I2SPR: uint16_t,
    pub RESERVED8: uint16_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct displayPortVTable_s {
    pub grab: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                         -> libc::c_int>,
    pub release: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                            -> libc::c_int>,
    pub clearScreen: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                                -> libc::c_int>,
    pub drawScreen: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                               -> libc::c_int>,
    pub screenSize: Option<unsafe extern "C" fn(_: *const displayPort_t)
                               -> libc::c_int>,
    pub writeString: Option<unsafe extern "C" fn(_: *mut displayPort_t,
                                                 _: uint8_t, _: uint8_t,
                                                 _: *const libc::c_char)
                                -> libc::c_int>,
    pub writeChar: Option<unsafe extern "C" fn(_: *mut displayPort_t,
                                               _: uint8_t, _: uint8_t,
                                               _: uint8_t) -> libc::c_int>,
    pub isTransferInProgress: Option<unsafe extern "C" fn(_:
                                                              *const displayPort_t)
                                         -> bool>,
    pub heartbeat: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                              -> libc::c_int>,
    pub resync: Option<unsafe extern "C" fn(_: *mut displayPort_t) -> ()>,
    pub isSynced: Option<unsafe extern "C" fn(_: *const displayPort_t)
                             -> bool>,
    pub txBytesFree: Option<unsafe extern "C" fn(_: *const displayPort_t)
                                -> uint32_t>,
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
pub type displayPort_t = displayPort_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct displayPort_s {
    pub vTable: *const displayPortVTable_s,
    pub device: *mut libc::c_void,
    pub rows: uint8_t,
    pub cols: uint8_t,
    pub posX: uint8_t,
    pub posY: uint8_t,
    pub cleared: bool,
    pub cursorRow: int8_t,
    pub grabCount: int8_t,
}
// CMS state
// displayPort_t is used as a parameter group in 'displayport_msp.h' and 'displayport_max7456`.h'. Treat accordingly!
pub type displayPortVTable_t = displayPortVTable_s;
pub type IO_t = *mut libc::c_void;
pub type I2CDevice = libc::c_int;
pub const I2CDEV_4: I2CDevice = 3;
pub const I2CDEV_3: I2CDevice = 2;
pub const I2CDEV_2: I2CDevice = 1;
pub const I2CDEV_1: I2CDevice = 0;
pub const I2CINVALID: I2CDevice = -1;
pub type busType_e = libc::c_uint;
pub const BUSTYPE_MPU_SLAVE: busType_e = 3;
pub const BUSTYPE_SPI: busType_e = 2;
pub const BUSTYPE_I2C: busType_e = 1;
pub const BUSTYPE_NONE: busType_e = 0;
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
// Slave I2C on SPI master
#[derive(Copy, Clone)]
#[repr(C)]
pub struct busDevice_s {
    pub bustype: busType_e,
    pub busdev_u: C2RustUnnamed,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed {
    pub spi: deviceSpi_s,
    pub i2c: deviceI2C_s,
    pub mpuSlave: deviceMpuSlave_s,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceMpuSlave_s {
    pub master: *const busDevice_s,
    pub address: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceI2C_s {
    pub device: I2CDevice,
    pub address: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceSpi_s {
    pub instance: *mut SPI_TypeDef,
    pub csnPin: IO_t,
}
pub type busDevice_t = busDevice_s;
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
static mut oledDisplayPort: displayPort_t =
    displayPort_t{vTable: 0 as *const displayPortVTable_s,
                  device: 0 as *const libc::c_void as *mut libc::c_void,
                  rows: 0,
                  cols: 0,
                  posX: 0,
                  posY: 0,
                  cleared: false,
                  cursorRow: 0,
                  grabCount: 0,};
unsafe extern "C" fn oledGrab(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    return 0 as libc::c_int;
}
unsafe extern "C" fn oledRelease(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    return 0 as libc::c_int;
}
unsafe extern "C" fn oledClearScreen(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    i2c_OLED_clear_display_quick((*displayPort).device as *mut busDevice_t);
    return 0 as libc::c_int;
}
unsafe extern "C" fn oledDrawScreen(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    return 0 as libc::c_int;
}
unsafe extern "C" fn oledScreenSize(mut displayPort: *const displayPort_t)
 -> libc::c_int {
    return (*displayPort).rows as libc::c_int *
               (*displayPort).cols as libc::c_int;
}
unsafe extern "C" fn oledWriteString(mut displayPort: *mut displayPort_t,
                                     mut x: uint8_t, mut y: uint8_t,
                                     mut s: *const libc::c_char)
 -> libc::c_int {
    i2c_OLED_set_xy((*displayPort).device as *mut busDevice_t, x, y);
    i2c_OLED_send_string((*displayPort).device as *mut busDevice_t, s);
    return 0 as libc::c_int;
}
unsafe extern "C" fn oledWriteChar(mut displayPort: *mut displayPort_t,
                                   mut x: uint8_t, mut y: uint8_t,
                                   mut c: uint8_t) -> libc::c_int {
    i2c_OLED_set_xy((*displayPort).device as *mut busDevice_t, x, y);
    i2c_OLED_send_char((*displayPort).device as *mut busDevice_t, c);
    return 0 as libc::c_int;
}
unsafe extern "C" fn oledIsTransferInProgress(mut displayPort:
                                                  *const displayPort_t)
 -> bool {
    return 0 as libc::c_int != 0;
}
unsafe extern "C" fn oledIsSynced(mut displayPort: *const displayPort_t)
 -> bool {
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn oledHeartbeat(mut displayPort: *mut displayPort_t)
 -> libc::c_int {
    return 0 as libc::c_int;
}
unsafe extern "C" fn oledResync(mut displayPort: *mut displayPort_t) { }
unsafe extern "C" fn oledTxBytesFree(mut displayPort: *const displayPort_t)
 -> uint32_t {
    return 4294967295 as libc::c_uint;
}
static mut oledVTable: displayPortVTable_t =
    unsafe {
        {
            let mut init =
                displayPortVTable_s{grab:
                                        Some(oledGrab as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> libc::c_int),
                                    release:
                                        Some(oledRelease as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> libc::c_int),
                                    clearScreen:
                                        Some(oledClearScreen as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> libc::c_int),
                                    drawScreen:
                                        Some(oledDrawScreen as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> libc::c_int),
                                    screenSize:
                                        Some(oledScreenSize as
                                                 unsafe extern "C" fn(_:
                                                                          *const displayPort_t)
                                                     -> libc::c_int),
                                    writeString:
                                        Some(oledWriteString as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t,
                                                                      _:
                                                                          uint8_t,
                                                                      _:
                                                                          uint8_t,
                                                                      _:
                                                                          *const libc::c_char)
                                                     -> libc::c_int),
                                    writeChar:
                                        Some(oledWriteChar as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t,
                                                                      _:
                                                                          uint8_t,
                                                                      _:
                                                                          uint8_t,
                                                                      _:
                                                                          uint8_t)
                                                     -> libc::c_int),
                                    isTransferInProgress:
                                        Some(oledIsTransferInProgress as
                                                 unsafe extern "C" fn(_:
                                                                          *const displayPort_t)
                                                     -> bool),
                                    heartbeat:
                                        Some(oledHeartbeat as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> libc::c_int),
                                    resync:
                                        Some(oledResync as
                                                 unsafe extern "C" fn(_:
                                                                          *mut displayPort_t)
                                                     -> ()),
                                    isSynced:
                                        Some(oledIsSynced as
                                                 unsafe extern "C" fn(_:
                                                                          *const displayPort_t)
                                                     -> bool),
                                    txBytesFree:
                                        Some(oledTxBytesFree as
                                                 unsafe extern "C" fn(_:
                                                                          *const displayPort_t)
                                                     -> uint32_t),};
            init
        }
    };
#[no_mangle]
pub unsafe extern "C" fn displayPortOledInit(mut device: *mut libc::c_void)
 -> *mut displayPort_t {
    oledDisplayPort.device = device;
    displayInit(&mut oledDisplayPort, &oledVTable);
    oledDisplayPort.rows =
        (64 as libc::c_int / (7 as libc::c_int + 1 as libc::c_int)) as
            uint8_t;
    oledDisplayPort.cols =
        (128 as libc::c_int / (5 as libc::c_int + 1 as libc::c_int)) as
            uint8_t;
    return &mut oledDisplayPort;
}
