use core;
use libc;
extern "C" {
    #[no_mangle]
    fn spiTransferByte(instance: *mut SPI_TypeDef, data: uint8_t) -> uint8_t;
    #[no_mangle]
    fn spiTransfer(instance: *mut SPI_TypeDef, txData: *const uint8_t,
                   rxData: *mut uint8_t, len: libc::c_int) -> bool;
    #[no_mangle]
    fn IOHi(io: IO_t);
    #[no_mangle]
    fn IOLo(io: IO_t);
    #[no_mangle]
    fn millis() -> timeMs_t;
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
#[derive ( Copy, Clone )]
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
    /* !< Reserved, 0x22                                                            */
}
pub type IO_t = *mut libc::c_void;
pub type I2CDevice = libc::c_int;
pub const I2CDEV_4: I2CDevice = 3;
pub const I2CDEV_3: I2CDevice = 2;
pub const I2CDEV_2: I2CDevice = 1;
pub const I2CDEV_1: I2CDevice = 0;
pub const I2CINVALID: I2CDevice = -1;
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
pub type busType_e = libc::c_uint;
pub const BUSTYPE_MPU_SLAVE: busType_e = 3;
pub const BUSTYPE_SPI: busType_e = 2;
pub const BUSTYPE_I2C: busType_e = 1;
pub const BUSTYPE_NONE: busType_e = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct busDevice_s {
    pub bustype: busType_e,
    pub busdev_u: C2RustUnnamed,
}
#[derive ( Copy, Clone )]
#[repr ( C )]
pub union C2RustUnnamed {
    pub spi: deviceSpi_s,
    pub i2c: deviceI2C_s,
    pub mpuSlave: deviceMpuSlave_s,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct deviceMpuSlave_s {
    pub master: *const busDevice_s,
    pub address: uint8_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct deviceI2C_s {
    pub device: I2CDevice,
    pub address: uint8_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct deviceSpi_s {
    pub instance: *mut SPI_TypeDef,
    pub csnPin: IO_t,
}
pub type busDevice_t = busDevice_s;
pub type flashType_e = libc::c_uint;
pub const FLASH_TYPE_NAND: flashType_e = 1;
pub const FLASH_TYPE_NOR: flashType_e = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct flashGeometry_s {
    pub sectors: uint16_t,
    pub pageSize: uint16_t,
    pub sectorSize: uint32_t,
    pub totalSize: uint32_t,
    pub pagesPerSector: uint16_t,
    pub flashType: flashType_e,
}
pub type flashGeometry_t = flashGeometry_s;
// Slave I2C on SPI master
// Count of the number of erasable blocks on the device
// In bytes
// This is just pagesPerSector * pageSize
// This is just sectorSize * sectors
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
 * Author: jflyper
 */
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct flashVTable_s {
    pub isReady: Option<unsafe extern "C" fn(_: *mut flashDevice_t) -> bool>,
    pub waitForReady: Option<unsafe extern "C" fn(_: *mut flashDevice_t,
                                                  _: uint32_t) -> bool>,
    pub eraseSector: Option<unsafe extern "C" fn(_: *mut flashDevice_t,
                                                 _: uint32_t) -> ()>,
    pub eraseCompletely: Option<unsafe extern "C" fn(_: *mut flashDevice_t)
                                    -> ()>,
    pub pageProgramBegin: Option<unsafe extern "C" fn(_: *mut flashDevice_t,
                                                      _: uint32_t) -> ()>,
    pub pageProgramContinue: Option<unsafe extern "C" fn(_:
                                                             *mut flashDevice_t,
                                                         _: *const uint8_t,
                                                         _: libc::c_int)
                                        -> ()>,
    pub pageProgramFinish: Option<unsafe extern "C" fn(_: *mut flashDevice_t)
                                      -> ()>,
    pub pageProgram: Option<unsafe extern "C" fn(_: *mut flashDevice_t,
                                                 _: uint32_t,
                                                 _: *const uint8_t,
                                                 _: libc::c_int) -> ()>,
    pub flush: Option<unsafe extern "C" fn(_: *mut flashDevice_t) -> ()>,
    pub readBytes: Option<unsafe extern "C" fn(_: *mut flashDevice_t,
                                               _: uint32_t, _: *mut uint8_t,
                                               _: libc::c_int)
                              -> libc::c_int>,
    pub getGeometry: Option<unsafe extern "C" fn(_: *mut flashDevice_t)
                                -> *const flashGeometry_t>,
}
pub type flashDevice_t = flashDevice_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct flashDevice_s {
    pub busdev: *mut busDevice_t,
    pub vTable: *const flashVTable_s,
    pub geometry: flashGeometry_t,
    pub currentWriteAddress: uint32_t,
    pub isLargeFlash: bool,
    pub couldBeBusy: bool,
}
pub type flashVTable_t = flashVTable_s;
pub type timeMs_t = uint32_t;
unsafe extern "C" fn m25p16_disable(mut bus: *mut busDevice_t) {
    IOHi((*bus).busdev_u.spi.csnPin);
    asm!("nop" : : : : "volatile");
}
unsafe extern "C" fn m25p16_enable(mut bus: *mut busDevice_t) {
    asm!("nop" : : : : "volatile");
    IOLo((*bus).busdev_u.spi.csnPin);
}
unsafe extern "C" fn m25p16_transfer(mut bus: *mut busDevice_t,
                                     mut txData: *const uint8_t,
                                     mut rxData: *mut uint8_t,
                                     mut len: libc::c_int) {
    m25p16_enable(bus);
    spiTransfer((*bus).busdev_u.spi.instance, txData, rxData, len);
    m25p16_disable(bus);
}
/* *
 * Send the given command byte to the device.
 */
unsafe extern "C" fn m25p16_performOneByteCommand(mut bus: *mut busDevice_t,
                                                  mut command: uint8_t) {
    m25p16_enable(bus);
    spiTransferByte((*bus).busdev_u.spi.instance, command);
    m25p16_disable(bus);
}
/* *
 * The flash requires this write enable command to be sent before commands that would cause
 * a write like program and erase.
 */
unsafe extern "C" fn m25p16_writeEnable(mut fdevice: *mut flashDevice_t) {
    m25p16_performOneByteCommand((*fdevice).busdev, 0x6i32 as uint8_t);
    // Assume that we're about to do some writing, so the device is just about to become busy
    (*fdevice).couldBeBusy = 1i32 != 0;
}
unsafe extern "C" fn m25p16_readStatus(mut bus: *mut busDevice_t) -> uint8_t {
    let command: [uint8_t; 2] = [0x5i32 as uint8_t, 0i32 as uint8_t];
    let mut in_0: [uint8_t; 2] = [0; 2];
    m25p16_transfer(bus, command.as_ptr(), in_0.as_mut_ptr(),
                    ::core::mem::size_of::<[uint8_t; 2]>() as libc::c_ulong as
                        libc::c_int);
    return in_0[1];
}
unsafe extern "C" fn m25p16_isReady(mut fdevice: *mut flashDevice_t) -> bool {
    // If couldBeBusy is false, don't bother to poll the flash chip for its status
    (*fdevice).couldBeBusy =
        (*fdevice).couldBeBusy as libc::c_int != 0 &&
            m25p16_readStatus((*fdevice).busdev) as libc::c_int & 0x1i32 !=
                0i32;
    return !(*fdevice).couldBeBusy;
}
unsafe extern "C" fn m25p16_waitForReady(mut fdevice: *mut flashDevice_t,
                                         mut timeoutMillis: uint32_t)
 -> bool {
    let mut time: uint32_t = millis();
    while !m25p16_isReady(fdevice) {
        if millis().wrapping_sub(time) > timeoutMillis { return 0i32 != 0 }
    }
    return 1i32 != 0;
}
/* *
 * Read chip identification and geometry information (into global `geometry`).
 *
 * Returns true if we get valid ident, false if something bad happened like there is no M25P16.
 */
#[no_mangle]
pub unsafe extern "C" fn m25p16_detect(mut fdevice: *mut flashDevice_t,
                                       mut chipID: uint32_t) -> bool {
    match chipID {
        15679509 | 2105365 => {
            (*fdevice).geometry.sectors = 32i32 as uint16_t;
            (*fdevice).geometry.pagesPerSector = 256i32 as uint16_t
        }
        15679510 | 12722198 => {
            (*fdevice).geometry.sectors = 64i32 as uint16_t;
            (*fdevice).geometry.pagesPerSector = 256i32 as uint16_t
        }
        2144791 | 15679511 | 12722199 => {
            (*fdevice).geometry.sectors = 128i32 as uint16_t;
            (*fdevice).geometry.pagesPerSector = 256i32 as uint16_t
        }
        2144792 | 15679512 | 90136 => {
            (*fdevice).geometry.sectors = 256i32 as uint16_t;
            (*fdevice).geometry.pagesPerSector = 256i32 as uint16_t
        }
        15679513 | 12722201 => {
            (*fdevice).geometry.sectors = 512i32 as uint16_t;
            (*fdevice).geometry.pagesPerSector = 256i32 as uint16_t
        }
        _ => {
            // Unsupported chip or not an SPI NOR flash
            (*fdevice).geometry.sectors =
                0i32 as
                    uint16_t; // Just for luck we'll assume the chip could be busy even though it isn't specced to be
            (*fdevice).geometry.pagesPerSector = 0i32 as uint16_t;
            (*fdevice).geometry.sectorSize = 0i32 as uint32_t;
            (*fdevice).geometry.totalSize = 0i32 as uint32_t;
            return 0i32 != 0
        }
    }
    (*fdevice).geometry.flashType = FLASH_TYPE_NOR;
    (*fdevice).geometry.pageSize = 256i32 as uint16_t;
    (*fdevice).geometry.sectorSize =
        ((*fdevice).geometry.pagesPerSector as libc::c_int *
             (*fdevice).geometry.pageSize as libc::c_int) as uint32_t;
    (*fdevice).geometry.totalSize =
        (*fdevice).geometry.sectorSize.wrapping_mul((*fdevice).geometry.sectors
                                                        as libc::c_uint);
    if (*fdevice).geometry.totalSize >
           (16i32 * 1024i32 * 1024i32) as libc::c_uint {
        (*fdevice).isLargeFlash = 1i32 != 0;
        m25p16_performOneByteCommand((*fdevice).busdev, 0xb7i32 as uint8_t);
    }
    (*fdevice).couldBeBusy = 1i32 != 0;
    (*fdevice).vTable = &m25p16_vTable;
    return 1i32 != 0;
}
unsafe extern "C" fn m25p16_setCommandAddress(mut buf: *mut uint8_t,
                                              mut address: uint32_t,
                                              mut useLongAddress: bool) {
    if useLongAddress {
        let fresh0 = buf;
        buf = buf.offset(1);
        *fresh0 = (address >> 24i32 & 0xffi32 as libc::c_uint) as uint8_t
    }
    let fresh1 = buf;
    buf = buf.offset(1);
    *fresh1 = (address >> 16i32 & 0xffi32 as libc::c_uint) as uint8_t;
    let fresh2 = buf;
    buf = buf.offset(1);
    *fresh2 = (address >> 8i32 & 0xffi32 as libc::c_uint) as uint8_t;
    *buf = (address & 0xffi32 as libc::c_uint) as uint8_t;
}
/* *
 * Erase a sector full of bytes to all 1's at the given byte offset in the flash chip.
 */
unsafe extern "C" fn m25p16_eraseSector(mut fdevice: *mut flashDevice_t,
                                        mut address: uint32_t) {
    let mut out: [uint8_t; 5] = [0xd8i32 as uint8_t, 0, 0, 0, 0];
    m25p16_setCommandAddress(&mut *out.as_mut_ptr().offset(1), address,
                             (*fdevice).isLargeFlash);
    m25p16_waitForReady(fdevice, 5000i32 as uint32_t);
    m25p16_writeEnable(fdevice);
    m25p16_transfer((*fdevice).busdev, out.as_mut_ptr(), 0 as *mut uint8_t,
                    ::core::mem::size_of::<[uint8_t; 5]>() as libc::c_ulong as
                        libc::c_int);
}
unsafe extern "C" fn m25p16_eraseCompletely(mut fdevice: *mut flashDevice_t) {
    m25p16_waitForReady(fdevice, 21000i32 as uint32_t);
    m25p16_writeEnable(fdevice);
    m25p16_performOneByteCommand((*fdevice).busdev, 0xc7i32 as uint8_t);
}
unsafe extern "C" fn m25p16_pageProgramBegin(mut fdevice: *mut flashDevice_t,
                                             mut address: uint32_t) {
    (*fdevice).currentWriteAddress = address;
}
unsafe extern "C" fn m25p16_pageProgramContinue(mut fdevice:
                                                    *mut flashDevice_t,
                                                mut data: *const uint8_t,
                                                mut length: libc::c_int) {
    let mut command: [uint8_t; 5] = [0x2i32 as uint8_t, 0, 0, 0, 0];
    m25p16_setCommandAddress(&mut *command.as_mut_ptr().offset(1),
                             (*fdevice).currentWriteAddress,
                             (*fdevice).isLargeFlash);
    m25p16_waitForReady(fdevice, 6i32 as uint32_t);
    m25p16_writeEnable(fdevice);
    m25p16_enable((*fdevice).busdev);
    spiTransfer((*(*fdevice).busdev).busdev_u.spi.instance,
                command.as_mut_ptr(), 0 as *mut uint8_t,
                if (*fdevice).isLargeFlash as libc::c_int != 0 {
                    5i32
                } else { 4i32 });
    spiTransfer((*(*fdevice).busdev).busdev_u.spi.instance, data,
                0 as *mut uint8_t, length);
    m25p16_disable((*fdevice).busdev);
    (*fdevice).currentWriteAddress =
        ((*fdevice).currentWriteAddress as
             libc::c_uint).wrapping_add(length as libc::c_uint) as uint32_t as
            uint32_t;
}
unsafe extern "C" fn m25p16_pageProgramFinish(mut fdevice:
                                                  *mut flashDevice_t) {
}
/* *
 * Write bytes to a flash page. Address must not cross a page boundary.
 *
 * Bits can only be set to zero, not from zero back to one again. In order to set bits to 1, use the erase command.
 *
 * Length must be smaller than the page size.
 *
 * This will wait for the flash to become ready before writing begins.
 *
 * Datasheet indicates typical programming time is 0.8ms for 256 bytes, 0.2ms for 64 bytes, 0.05ms for 16 bytes.
 * (Although the maximum possible write time is noted as 5ms).
 *
 * If you want to write multiple buffers (whose sum of sizes is still not more than the page size) then you can
 * break this operation up into one beginProgram call, one or more continueProgram calls, and one finishProgram call.
 */
unsafe extern "C" fn m25p16_pageProgram(mut fdevice: *mut flashDevice_t,
                                        mut address: uint32_t,
                                        mut data: *const uint8_t,
                                        mut length: libc::c_int) {
    m25p16_pageProgramBegin(fdevice, address);
    m25p16_pageProgramContinue(fdevice, data, length);
    m25p16_pageProgramFinish(fdevice);
}
/* *
 * Read `length` bytes into the provided `buffer` from the flash starting from the given `address` (which need not lie
 * on a page boundary).
 *
 * Waits up to DEFAULT_TIMEOUT_MILLIS milliseconds for the flash to become ready before reading.
 *
 * The number of bytes actually read is returned, which can be zero if an error or timeout occurred.
 */
unsafe extern "C" fn m25p16_readBytes(mut fdevice: *mut flashDevice_t,
                                      mut address: uint32_t,
                                      mut buffer: *mut uint8_t,
                                      mut length: libc::c_int)
 -> libc::c_int {
    let mut command: [uint8_t; 5] = [0x3i32 as uint8_t, 0, 0, 0, 0];
    m25p16_setCommandAddress(&mut *command.as_mut_ptr().offset(1), address,
                             (*fdevice).isLargeFlash);
    if !m25p16_waitForReady(fdevice, 6i32 as uint32_t) { return 0i32 }
    m25p16_enable((*fdevice).busdev);
    spiTransfer((*(*fdevice).busdev).busdev_u.spi.instance,
                command.as_mut_ptr(), 0 as *mut uint8_t,
                if (*fdevice).isLargeFlash as libc::c_int != 0 {
                    5i32
                } else { 4i32 });
    spiTransfer((*(*fdevice).busdev).busdev_u.spi.instance,
                0 as *const uint8_t, buffer, length);
    m25p16_disable((*fdevice).busdev);
    return length;
}
/* *
 * Fetch information about the detected flash chip layout.
 *
 * Can be called before calling m25p16_init() (the result would have totalSize = 0).
 */
unsafe extern "C" fn m25p16_getGeometry(mut fdevice: *mut flashDevice_t)
 -> *const flashGeometry_t {
    return &mut (*fdevice).geometry;
}
#[no_mangle]
pub static mut m25p16_vTable: flashVTable_t =
    unsafe {
        {
            let mut init =
                flashVTable_s{isReady:
                                  Some(m25p16_isReady as
                                           unsafe extern "C" fn(_:
                                                                    *mut flashDevice_t)
                                               -> bool),
                              waitForReady:
                                  Some(m25p16_waitForReady as
                                           unsafe extern "C" fn(_:
                                                                    *mut flashDevice_t,
                                                                _: uint32_t)
                                               -> bool),
                              eraseSector:
                                  Some(m25p16_eraseSector as
                                           unsafe extern "C" fn(_:
                                                                    *mut flashDevice_t,
                                                                _: uint32_t)
                                               -> ()),
                              eraseCompletely:
                                  Some(m25p16_eraseCompletely as
                                           unsafe extern "C" fn(_:
                                                                    *mut flashDevice_t)
                                               -> ()),
                              pageProgramBegin:
                                  Some(m25p16_pageProgramBegin as
                                           unsafe extern "C" fn(_:
                                                                    *mut flashDevice_t,
                                                                _: uint32_t)
                                               -> ()),
                              pageProgramContinue:
                                  Some(m25p16_pageProgramContinue as
                                           unsafe extern "C" fn(_:
                                                                    *mut flashDevice_t,
                                                                _:
                                                                    *const uint8_t,
                                                                _:
                                                                    libc::c_int)
                                               -> ()),
                              pageProgramFinish:
                                  Some(m25p16_pageProgramFinish as
                                           unsafe extern "C" fn(_:
                                                                    *mut flashDevice_t)
                                               -> ()),
                              pageProgram:
                                  Some(m25p16_pageProgram as
                                           unsafe extern "C" fn(_:
                                                                    *mut flashDevice_t,
                                                                _: uint32_t,
                                                                _:
                                                                    *const uint8_t,
                                                                _:
                                                                    libc::c_int)
                                               -> ()),
                              flush: None,
                              readBytes:
                                  Some(m25p16_readBytes as
                                           unsafe extern "C" fn(_:
                                                                    *mut flashDevice_t,
                                                                _: uint32_t,
                                                                _:
                                                                    *mut uint8_t,
                                                                _:
                                                                    libc::c_int)
                                               -> libc::c_int),
                              getGeometry:
                                  Some(m25p16_getGeometry as
                                           unsafe extern "C" fn(_:
                                                                    *mut flashDevice_t)
                                               -> *const flashGeometry_t),};
            init
        }
    };
