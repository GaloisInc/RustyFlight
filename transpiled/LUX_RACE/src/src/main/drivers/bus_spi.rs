use ::libc;
extern "C" {
    #[no_mangle]
    fn spiTransferByte(instance: *mut SPI_TypeDef, data: uint8_t) -> uint8_t;
    #[no_mangle]
    fn spiTransfer(instance: *mut SPI_TypeDef, txData: *const uint8_t,
                   rxData: *mut uint8_t, len: libc::c_int) -> bool;
    #[no_mangle]
    fn spiInitDevice(device: SPIDevice);
    #[no_mangle]
    fn IOHi(io: IO_t);
    #[no_mangle]
    fn IOLo(io: IO_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* *
  * @brief Serial Peripheral Interface
  */
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
pub type SPIDevice = libc::c_int;
pub const SPIDEV_4: SPIDevice = 3;
pub const SPIDEV_3: SPIDevice = 2;
pub const SPIDEV_2: SPIDevice = 1;
pub const SPIDEV_1: SPIDevice = 0;
pub const SPIINVALID: SPIDevice = -1;
pub type spiDevice_t = SPIDevice_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SPIDevice_s {
    pub dev: *mut SPI_TypeDef,
    pub sck: ioTag_t,
    pub miso: ioTag_t,
    pub mosi: ioTag_t,
    pub af: uint8_t,
    pub rcc: rccPeriphTag_t,
    pub errorCount: uint16_t,
    pub leadingEdge: bool,
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
#[no_mangle]
pub static mut spiDevice: [spiDevice_t; 3] =
    [spiDevice_t{dev: 0 as *const SPI_TypeDef as *mut SPI_TypeDef,
                 sck: 0,
                 miso: 0,
                 mosi: 0,
                 af: 0,
                 rcc: 0,
                 errorCount: 0,
                 leadingEdge: false,}; 3];
#[no_mangle]
pub unsafe extern "C" fn spiDeviceByInstance(mut instance: *mut SPI_TypeDef)
 -> SPIDevice {
    if instance ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x10000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x3000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut SPI_TypeDef {
        return SPIDEV_1
    } // read transaction
    return SPIINVALID; // read transaction
}
#[no_mangle]
pub unsafe extern "C" fn spiInstanceByDevice(mut device: SPIDevice)
 -> *mut SPI_TypeDef {
    if device as libc::c_int >= 3 as libc::c_int {
        return 0 as *mut SPI_TypeDef
    }
    return spiDevice[device as usize].dev;
}
#[no_mangle]
pub unsafe extern "C" fn spiInit(mut device: SPIDevice) -> bool {
    match device as libc::c_int {
        -1 => { return 0 as libc::c_int != 0 }
        0 => { spiInitDevice(device); return 1 as libc::c_int != 0 }
        1 | 2 | 3 | _ => { }
    }
    return 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn spiTimeoutUserCallback(mut instance:
                                                    *mut SPI_TypeDef)
 -> uint32_t {
    let mut device: SPIDevice = spiDeviceByInstance(instance);
    if device as libc::c_int == SPIINVALID as libc::c_int {
        return -(1 as libc::c_int) as uint32_t
    }
    ::core::ptr::write_volatile(&mut spiDevice[device as usize].errorCount as
                                    *mut uint16_t,
                                ::core::ptr::read_volatile::<uint16_t>(&spiDevice[device
                                                                                      as
                                                                                      usize].errorCount
                                                                           as
                                                                           *const uint16_t).wrapping_add(1));
    return spiDevice[device as usize].errorCount as uint32_t;
}
#[no_mangle]
pub unsafe extern "C" fn spiBusTransfer(mut bus: *const busDevice_t,
                                        mut txData: *const uint8_t,
                                        mut rxData: *mut uint8_t,
                                        mut length: libc::c_int) -> bool {
    IOLo((*bus).busdev_u.spi.csnPin);
    spiTransfer((*bus).busdev_u.spi.instance, txData, rxData, length);
    IOHi((*bus).busdev_u.spi.csnPin);
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn spiGetErrorCounter(mut instance: *mut SPI_TypeDef)
 -> uint16_t {
    let mut device: SPIDevice = spiDeviceByInstance(instance);
    if device as libc::c_int == SPIINVALID as libc::c_int {
        return 0 as libc::c_int as uint16_t
    }
    return spiDevice[device as usize].errorCount;
}
#[no_mangle]
pub unsafe extern "C" fn spiResetErrorCounter(mut instance:
                                                  *mut SPI_TypeDef) {
    let mut device: SPIDevice = spiDeviceByInstance(instance);
    if device as libc::c_int != SPIINVALID as libc::c_int {
        ::core::ptr::write_volatile(&mut spiDevice[device as usize].errorCount
                                        as *mut uint16_t,
                                    0 as libc::c_int as uint16_t)
    };
}
#[no_mangle]
pub unsafe extern "C" fn spiBusWriteRegister(mut bus: *const busDevice_t,
                                             mut reg: uint8_t,
                                             mut data: uint8_t) -> bool {
    IOLo((*bus).busdev_u.spi.csnPin);
    spiTransferByte((*bus).busdev_u.spi.instance, reg);
    spiTransferByte((*bus).busdev_u.spi.instance, data);
    IOHi((*bus).busdev_u.spi.csnPin);
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn spiBusReadRegisterBuffer(mut bus: *const busDevice_t,
                                                  mut reg: uint8_t,
                                                  mut data: *mut uint8_t,
                                                  mut length: uint8_t)
 -> bool {
    IOLo((*bus).busdev_u.spi.csnPin);
    spiTransferByte((*bus).busdev_u.spi.instance,
                    (reg as libc::c_int | 0x80 as libc::c_int) as uint8_t);
    spiTransfer((*bus).busdev_u.spi.instance, 0 as *const uint8_t, data,
                length as libc::c_int);
    IOHi((*bus).busdev_u.spi.csnPin);
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn spiBusReadRegister(mut bus: *const busDevice_t,
                                            mut reg: uint8_t) -> uint8_t {
    let mut data: uint8_t = 0;
    IOLo((*bus).busdev_u.spi.csnPin);
    spiTransferByte((*bus).busdev_u.spi.instance,
                    (reg as libc::c_int | 0x80 as libc::c_int) as uint8_t);
    spiTransfer((*bus).busdev_u.spi.instance, 0 as *const uint8_t, &mut data,
                1 as libc::c_int);
    IOHi((*bus).busdev_u.spi.csnPin);
    return data;
}
#[no_mangle]
pub unsafe extern "C" fn spiBusSetInstance(mut bus: *mut busDevice_t,
                                           mut instance: *mut SPI_TypeDef) {
    (*bus).bustype = BUSTYPE_SPI;
    (*bus).busdev_u.spi.instance = instance;
}
