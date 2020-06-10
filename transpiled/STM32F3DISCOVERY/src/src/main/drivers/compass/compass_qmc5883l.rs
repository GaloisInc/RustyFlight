use ::libc;
extern "C" {
    #[no_mangle]
    fn busWriteRegister(bus: *const busDevice_t, reg: uint8_t, data: uint8_t)
     -> bool;
    #[no_mangle]
    fn busReadRegisterBuffer(bus: *const busDevice_t, reg: uint8_t,
                             data: *mut uint8_t, length: uint8_t) -> bool;
    #[no_mangle]
    fn delay(ms: timeMs_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int16_t = __int16_t;
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
pub type C2RustUnnamed = libc::c_uint;
pub const Z: C2RustUnnamed = 2;
pub const Y: C2RustUnnamed = 1;
pub const X: C2RustUnnamed = 0;
pub type ioTag_t = uint8_t;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct busDevice_s {
    pub bustype: busType_e,
    pub busdev_u: C2RustUnnamed_0,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_0 {
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
pub type sensor_align_e = libc::c_uint;
pub const CW270_DEG_FLIP: sensor_align_e = 8;
pub const CW180_DEG_FLIP: sensor_align_e = 7;
pub const CW90_DEG_FLIP: sensor_align_e = 6;
pub const CW0_DEG_FLIP: sensor_align_e = 5;
pub const CW270_DEG: sensor_align_e = 4;
pub const CW180_DEG: sensor_align_e = 3;
pub const CW90_DEG: sensor_align_e = 2;
pub const CW0_DEG: sensor_align_e = 1;
pub const ALIGN_DEFAULT: sensor_align_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct magDev_s {
    pub init: sensorMagInitFuncPtr,
    pub read: sensorMagReadFuncPtr,
    pub exti: extiCallbackRec_t,
    pub busdev: busDevice_t,
    pub magAlign: sensor_align_e,
    pub magIntExtiTag: ioTag_t,
    pub magGain: [int16_t; 3],
}
pub type extiCallbackRec_t = extiCallbackRec_s;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct extiCallbackRec_s {
    pub fn_0: Option<extiHandlerCallback>,
}
pub type extiHandlerCallback
    =
    unsafe extern "C" fn(_: *mut extiCallbackRec_t) -> ();
pub type sensorMagReadFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut magDev_s, _: *mut int16_t) -> bool>;
pub type sensorMagInitFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut magDev_s) -> bool>;
// millisecond time
pub type timeMs_t = uint32_t;
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
pub type magDev_t = magDev_s;
unsafe extern "C" fn qmc5883lInit(mut magDev: *mut magDev_t) -> bool {
    let mut ack: bool = 1 as libc::c_int != 0;
    let mut busdev: *mut busDevice_t = &mut (*magDev).busdev;
    ack =
        ack as libc::c_int != 0 &&
            busWriteRegister(busdev, 0xb as libc::c_int as uint8_t,
                             0x1 as libc::c_int as uint8_t) as libc::c_int !=
                0;
    ack =
        ack as libc::c_int != 0 &&
            busWriteRegister(busdev, 0x9 as libc::c_int as uint8_t,
                             (0x1 as libc::c_int |
                                  (0x3 as libc::c_int) << 2 as libc::c_int |
                                  (0 as libc::c_int) << 6 as libc::c_int |
                                  (0x1 as libc::c_int) << 4 as libc::c_int) as
                                 uint8_t) as libc::c_int != 0;
    if !ack { return 0 as libc::c_int != 0 }
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn qmc5883lRead(mut magDev: *mut magDev_t,
                                  mut magData: *mut int16_t) -> bool {
    let mut status: uint8_t = 0;
    let mut buf: [uint8_t; 6] = [0; 6];
    // initialize function
    // read 3 axis data function
    // set magData to zero for case of failed read
    *magData.offset(X as libc::c_int as isize) = 0 as libc::c_int as int16_t;
    *magData.offset(Y as libc::c_int as isize) = 0 as libc::c_int as int16_t;
    *magData.offset(Z as libc::c_int as isize) = 0 as libc::c_int as int16_t;
    let mut busdev: *mut busDevice_t = &mut (*magDev).busdev;
    let mut ack: bool =
        busReadRegisterBuffer(busdev, 0x6 as libc::c_int as uint8_t,
                              &mut status, 1 as libc::c_int as uint8_t);
    if !ack || status as libc::c_int & 0x4 as libc::c_int == 0 as libc::c_int
       {
        return 0 as libc::c_int != 0
    }
    ack =
        busReadRegisterBuffer(busdev, 0 as libc::c_int as uint8_t,
                              buf.as_mut_ptr(), 6 as libc::c_int as uint8_t);
    if !ack { return 0 as libc::c_int != 0 }
    *magData.offset(X as libc::c_int as isize) =
        ((buf[1 as libc::c_int as usize] as libc::c_int) << 8 as libc::c_int |
             buf[0 as libc::c_int as usize] as libc::c_int) as int16_t;
    *magData.offset(Y as libc::c_int as isize) =
        ((buf[3 as libc::c_int as usize] as libc::c_int) << 8 as libc::c_int |
             buf[2 as libc::c_int as usize] as libc::c_int) as int16_t;
    *magData.offset(Z as libc::c_int as isize) =
        ((buf[5 as libc::c_int as usize] as libc::c_int) << 8 as libc::c_int |
             buf[4 as libc::c_int as usize] as libc::c_int) as int16_t;
    return 1 as libc::c_int != 0;
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
pub unsafe extern "C" fn qmc5883lDetect(mut magDev: *mut magDev_t) -> bool {
    let mut busdev: *mut busDevice_t = &mut (*magDev).busdev;
    if (*busdev).bustype as libc::c_uint ==
           BUSTYPE_I2C as libc::c_int as libc::c_uint &&
           (*busdev).busdev_u.i2c.address as libc::c_int == 0 as libc::c_int {
        (*busdev).busdev_u.i2c.address = 0xd as libc::c_int as uint8_t
    }
    // Must write reset first  - don't care about the result
    busWriteRegister(busdev, 0xa as libc::c_int as uint8_t,
                     0x80 as libc::c_int as uint8_t);
    delay(20 as libc::c_int as timeMs_t);
    let mut sig: uint8_t = 0 as libc::c_int as uint8_t;
    let mut ack: bool =
        busReadRegisterBuffer(busdev, 0xd as libc::c_int as uint8_t, &mut sig,
                              1 as libc::c_int as uint8_t);
    if ack as libc::c_int != 0 && sig as libc::c_int == 0xff as libc::c_int {
        // Should be in standby mode after soft reset and sensor is really present
        // Reading ChipID of 0xFF alone is not sufficient to be sure the QMC is present
        ack =
            busReadRegisterBuffer(busdev, 0x9 as libc::c_int as uint8_t,
                                  &mut sig, 1 as libc::c_int as uint8_t);
        if ack as libc::c_int != 0 && sig as libc::c_int != 0 as libc::c_int {
            return 0 as libc::c_int != 0
        }
        (*magDev).init =
            Some(qmc5883lInit as
                     unsafe extern "C" fn(_: *mut magDev_t) -> bool);
        (*magDev).read =
            Some(qmc5883lRead as
                     unsafe extern "C" fn(_: *mut magDev_t, _: *mut int16_t)
                         -> bool);
        return 1 as libc::c_int != 0
    }
    return 0 as libc::c_int != 0;
}
