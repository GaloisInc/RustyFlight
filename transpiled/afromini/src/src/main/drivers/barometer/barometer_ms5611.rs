use core;
use libc;
extern "C" {
    #[no_mangle]
    fn busWriteRegister(bus: *const busDevice_t, reg: uint8_t, data: uint8_t)
     -> bool;
    #[no_mangle]
    fn busReadRegisterBuffer(bus: *const busDevice_t, reg: uint8_t,
                             data: *mut uint8_t, length: uint8_t) -> bool;
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
    fn delayMicroseconds(us: timeUs_t);
    #[no_mangle]
    fn delay(ms: timeMs_t);
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type __int64_t = libc::c_long;
pub type __uint64_t = libc::c_ulong;
pub type int8_t = __int8_t;
pub type int32_t = __int32_t;
pub type int64_t = __int64_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct baroDev_s {
    pub busdev: busDevice_t,
    pub ut_delay: uint16_t,
    pub up_delay: uint16_t,
    pub start_ut: baroOpFuncPtr,
    pub get_ut: baroOpFuncPtr,
    pub start_up: baroOpFuncPtr,
    pub get_up: baroOpFuncPtr,
    pub calculate: baroCalculateFuncPtr,
}
pub type baroCalculateFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut int32_t, _: *mut int32_t) -> ()>;
pub type baroOpFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut baroDev_s) -> ()>;
pub type baroDev_t = baroDev_s;
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
// time difference, 32 bits always sufficient
// millisecond time
// microsecond time
pub type timeUs_t = uint32_t;
pub type timeMs_t = uint32_t;
static mut ms5611_ut: uint32_t = 0;
// static result of temperature measurement
static mut ms5611_up: uint32_t = 0;
// static result of pressure measurement
static mut ms5611_c: [uint16_t; 8] = [0; 8];
// on-chip ROM
static mut ms5611_osr: uint8_t = 0x8i32 as uint8_t;
#[no_mangle]
pub unsafe extern "C" fn ms5611BusInit(mut busdev: *mut busDevice_t) { }
#[no_mangle]
pub unsafe extern "C" fn ms5611BusDeinit(mut busdev: *mut busDevice_t) { }
#[no_mangle]
pub unsafe extern "C" fn ms5611Detect(mut baro: *mut baroDev_t) -> bool {
    let mut sig: uint8_t =
        0; // No idea how long the chip takes to power-up, but let's make it 10ms
    let mut i: libc::c_int = 0;
    let mut defaultAddressApplied: bool = 0i32 != 0;
    delay(10i32 as timeMs_t);
    let mut busdev: *mut busDevice_t = &mut (*baro).busdev;
    ms5611BusInit(busdev);
    if (*busdev).bustype as libc::c_uint ==
           BUSTYPE_I2C as libc::c_int as libc::c_uint &&
           (*busdev).busdev_u.i2c.address as libc::c_int == 0i32 {
        // Default address for MS5611
        (*busdev).busdev_u.i2c.address = 0x77i32 as uint8_t;
        defaultAddressApplied = 1i32 != 0
    }
    if !(!busReadRegisterBuffer(busdev, 0xa0i32 as uint8_t, &mut sig,
                                1i32 as uint8_t) ||
             sig as libc::c_int == 0xffi32) {
        ms5611_reset(busdev);
        // read all coefficients
        i = 0i32;
        while i < 8i32 {
            ms5611_c[i as usize] = ms5611_prom(busdev, i as int8_t);
            i += 1
        }
        // check crc, bail out if wrong - we are probably talking to BMP085 w/o XCLR line!
        if !(ms5611_crc(ms5611_c.as_mut_ptr()) as libc::c_int != 0i32) {
            // TODO prom + CRC
            (*baro).ut_delay = 10000i32 as uint16_t; // send PROM READ command
            (*baro).up_delay = 10000i32 as uint16_t; // read ADC
            (*baro).start_ut =
                Some(ms5611_start_ut as
                         unsafe extern "C" fn(_: *mut baroDev_t) -> ());
            (*baro).get_ut =
                Some(ms5611_get_ut as
                         unsafe extern "C" fn(_: *mut baroDev_t) -> ());
            (*baro).start_up =
                Some(ms5611_start_up as
                         unsafe extern "C" fn(_: *mut baroDev_t) -> ());
            (*baro).get_up =
                Some(ms5611_get_up as
                         unsafe extern "C" fn(_: *mut baroDev_t) -> ());
            (*baro).calculate =
                Some(ms5611_calculate as
                         unsafe extern "C" fn(_: *mut int32_t,
                                              _: *mut int32_t) -> ());
            return 1i32 != 0
        }
    }
    ms5611BusDeinit(busdev);
    if defaultAddressApplied {
        (*busdev).busdev_u.i2c.address = 0i32 as uint8_t
    }
    return 0i32 != 0;
}
unsafe extern "C" fn ms5611_reset(mut busdev: *mut busDevice_t) {
    busWriteRegister(busdev, 0x1ei32 as uint8_t, 1i32 as uint8_t);
    delayMicroseconds(2800i32 as timeUs_t);
}
unsafe extern "C" fn ms5611_prom(mut busdev: *mut busDevice_t,
                                 mut coef_num: int8_t) -> uint16_t {
    let mut rxbuf: [uint8_t; 2] = [0i32 as uint8_t, 0i32 as uint8_t];
    busReadRegisterBuffer(busdev,
                          (0xa0i32 + coef_num as libc::c_int * 2i32) as
                              uint8_t, rxbuf.as_mut_ptr(), 2i32 as uint8_t);
    return ((rxbuf[0] as libc::c_int) << 8i32 | rxbuf[1] as libc::c_int) as
               uint16_t;
}
unsafe extern "C" fn ms5611_crc(mut prom: *mut uint16_t) -> int8_t {
    let mut i: int32_t = 0;
    let mut j: int32_t = 0;
    let mut res: uint32_t = 0i32 as uint32_t;
    let mut crc: uint8_t =
        (*prom.offset(7) as libc::c_int & 0xfi32) as uint8_t;
    let ref mut fresh0 = *prom.offset(7);
    *fresh0 = (*fresh0 as libc::c_int & 0xff00i32) as uint16_t;
    let mut blankEeprom: bool = 1i32 != 0;
    i = 0i32;
    while i < 16i32 {
        if *prom.offset((i >> 1i32) as isize) != 0 { blankEeprom = 0i32 != 0 }
        if i & 1i32 != 0 {
            res ^=
                (*prom.offset((i >> 1i32) as isize) as libc::c_int & 0xffi32)
                    as libc::c_uint
        } else {
            res ^=
                (*prom.offset((i >> 1i32) as isize) as libc::c_int >> 8i32) as
                    libc::c_uint
        }
        j = 8i32;
        while j > 0i32 {
            if res & 0x8000i32 as libc::c_uint != 0 {
                res ^= 0x1800i32 as libc::c_uint
            }
            res <<= 1i32;
            j -= 1
        }
        i += 1
    }
    let ref mut fresh1 = *prom.offset(7);
    *fresh1 = (*fresh1 as libc::c_int | crc as libc::c_int) as uint16_t;
    if !blankEeprom &&
           crc as libc::c_uint == res >> 12i32 & 0xfi32 as libc::c_uint {
        return 0i32 as int8_t
    }
    return -1i32 as int8_t;
}
unsafe extern "C" fn ms5611_read_adc(mut busdev: *mut busDevice_t)
 -> uint32_t {
    let mut rxbuf: [uint8_t; 3] = [0; 3];
    busReadRegisterBuffer(busdev, 0i32 as uint8_t, rxbuf.as_mut_ptr(),
                          3i32 as uint8_t);
    return ((rxbuf[0] as libc::c_int) << 16i32 |
                (rxbuf[1] as libc::c_int) << 8i32 | rxbuf[2] as libc::c_int)
               as uint32_t;
}
unsafe extern "C" fn ms5611_start_ut(mut baro: *mut baroDev_t) {
    busWriteRegister(&mut (*baro).busdev,
                     (0x40i32 + 0x10i32 + ms5611_osr as libc::c_int) as
                         uint8_t, 1i32 as uint8_t);
    // D2 (temperature) conversion start!
}
unsafe extern "C" fn ms5611_get_ut(mut baro: *mut baroDev_t) {
    ms5611_ut = ms5611_read_adc(&mut (*baro).busdev);
}
unsafe extern "C" fn ms5611_start_up(mut baro: *mut baroDev_t) {
    busWriteRegister(&mut (*baro).busdev,
                     (0x40i32 + 0i32 + ms5611_osr as libc::c_int) as uint8_t,
                     1i32 as uint8_t);
    // D1 (pressure) conversion start!
}
unsafe extern "C" fn ms5611_get_up(mut baro: *mut baroDev_t) {
    ms5611_up = ms5611_read_adc(&mut (*baro).busdev);
}
unsafe extern "C" fn ms5611_calculate(mut pressure: *mut int32_t,
                                      mut temperature: *mut int32_t) {
    let mut press: uint32_t = 0;
    let mut temp: int64_t = 0;
    let mut delt: int64_t = 0;
    let mut dT: int64_t =
        (ms5611_ut as int64_t as
             libc::c_ulong).wrapping_sub((ms5611_c[5] as
                                              uint64_t).wrapping_mul(256i32 as
                                                                         libc::c_ulong))
            as int64_t;
    let mut off: int64_t =
        ((ms5611_c[2] as int64_t) << 16i32) +
            (ms5611_c[4] as int64_t * dT >> 7i32);
    let mut sens: int64_t =
        ((ms5611_c[1] as int64_t) << 15i32) +
            (ms5611_c[3] as int64_t * dT >> 8i32);
    temp = 2000i32 as libc::c_long + (dT * ms5611_c[6] as int64_t >> 23i32);
    if temp < 2000i32 as libc::c_long {
        // temperature lower than 20degC
        delt = temp - 2000i32 as libc::c_long;
        delt = 5i32 as libc::c_long * delt * delt;
        off -= delt >> 1i32;
        sens -= delt >> 2i32;
        if temp < -1500i32 as libc::c_long {
            // temperature lower than -15degC
            delt = temp + 1500i32 as libc::c_long;
            delt = delt * delt;
            off -= 7i32 as libc::c_long * delt;
            sens -= 11i32 as libc::c_long * delt >> 1i32
        }
        temp -= dT * dT >> 31i32
    }
    press =
        ((ms5611_up as int64_t * sens >> 21i32) - off >> 15i32) as uint32_t;
    if !pressure.is_null() { *pressure = press as int32_t }
    if !temperature.is_null() { *temperature = temp as int32_t };
}
