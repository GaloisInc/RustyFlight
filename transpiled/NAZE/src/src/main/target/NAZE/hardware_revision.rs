use ::libc;
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
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn delay(ms: timeMs_t);
    // current crystal frequency - 8 or 12MHz
    #[no_mangle]
    static mut hse_value: uint32_t;
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
pub type ioTag_t = uint8_t;
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
// packet tag to specify IO pin
pub type IO_t = *mut libc::c_void;
// millisecond time
pub type timeMs_t = uint32_t;
pub type nazeHardwareRevision_t = libc::c_uint;
pub const NAZE32_SP: nazeHardwareRevision_t = 3;
pub const NAZE32_REV5: nazeHardwareRevision_t = 2;
pub const NAZE32: nazeHardwareRevision_t = 1;
pub const UNKNOWN: nazeHardwareRevision_t = 0;
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
pub static mut hardwareRevision: uint8_t = UNKNOWN as libc::c_int as uint8_t;
#[no_mangle]
pub unsafe extern "C" fn detectHardwareRevision() {
    if hse_value == 8000000 as libc::c_int as libc::c_uint {
        hardwareRevision = NAZE32 as libc::c_int as uint8_t
    } else if hse_value == 12000000 as libc::c_int as libc::c_uint {
        hardwareRevision = NAZE32_REV5 as libc::c_int as uint8_t
    };
}
static mut nazeSpiCsPin: IO_t = 0 as *const libc::c_void as IO_t;
#[no_mangle]
pub unsafe extern "C" fn detectSpiDevice() -> uint8_t {
    nazeSpiCsPin =
        IOGetByTag(((1 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int
                        | 12 as libc::c_int) as ioTag_t);
    let out: [uint8_t; 4] =
        [0x9f as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t];
    let mut in_0: [uint8_t; 4] = [0; 4];
    let mut flash_id: uint32_t = 0;
    // try autodetect flash chip
    delay(50 as libc::c_int as
              timeMs_t); // short delay required after initialisation of SPI device instance.
    IOLo(nazeSpiCsPin);
    spiTransfer((0x40000000 as libc::c_int as
                     uint32_t).wrapping_add(0x3800 as libc::c_int as
                                                libc::c_uint) as
                    *mut SPI_TypeDef, out.as_ptr(), in_0.as_mut_ptr(),
                ::core::mem::size_of::<[uint8_t; 4]>() as libc::c_ulong as
                    libc::c_int);
    IOHi(nazeSpiCsPin);
    flash_id =
        ((in_0[1 as libc::c_int as usize] as libc::c_int) << 16 as libc::c_int
             |
             (in_0[2 as libc::c_int as usize] as libc::c_int) <<
                 8 as libc::c_int |
             in_0[3 as libc::c_int as usize] as libc::c_int) as uint32_t;
    if flash_id == 0x202015 as libc::c_int as libc::c_uint {
        return 1 as libc::c_int as uint8_t
    }
    // try autodetect MPU
    delay(50 as libc::c_int as timeMs_t);
    IOLo(nazeSpiCsPin);
    spiTransferByte((0x40000000 as libc::c_int as
                         uint32_t).wrapping_add(0x3800 as libc::c_int as
                                                    libc::c_uint) as
                        *mut SPI_TypeDef,
                    (0x75 as libc::c_int | 0x80 as libc::c_int) as uint8_t);
    in_0[0 as libc::c_int as usize] =
        spiTransferByte((0x40000000 as libc::c_int as
                             uint32_t).wrapping_add(0x3800 as libc::c_int as
                                                        libc::c_uint) as
                            *mut SPI_TypeDef, 0xff as libc::c_int as uint8_t);
    IOHi(nazeSpiCsPin);
    if in_0[0 as libc::c_int as usize] as libc::c_int == 0x70 as libc::c_int {
        return 2 as libc::c_int as uint8_t
    }
    return 0 as libc::c_int as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn updateHardwareRevision() {
    let mut detectedSpiDevice: uint8_t = detectSpiDevice();
    if detectedSpiDevice as libc::c_int == 2 as libc::c_int &&
           hardwareRevision as libc::c_int == NAZE32_REV5 as libc::c_int {
        hardwareRevision = NAZE32_SP as libc::c_int as uint8_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn selectMPUIntExtiConfigByHardwareRevision()
 -> ioTag_t {
    if (hardwareRevision as libc::c_int) < NAZE32_REV5 as libc::c_int {
        // MPU_INT output on rev4 PB13
        return ((1 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int |
                    13 as libc::c_int) as ioTag_t
    } else {
        // MPU_INT output on rev5 PC13
        return ((2 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int |
                    13 as libc::c_int) as ioTag_t
    };
}
