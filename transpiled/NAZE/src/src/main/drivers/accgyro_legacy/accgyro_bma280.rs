use ::libc;
extern "C" {
    #[no_mangle]
    fn i2cWrite(device: I2CDevice, addr_: uint8_t, reg: uint8_t,
                data: uint8_t) -> bool;
    #[no_mangle]
    fn i2cRead(device: I2CDevice, addr_: uint8_t, reg: uint8_t, len: uint8_t,
               buf: *mut uint8_t) -> bool;
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type int16_t = __int16_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
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
// packet tag to specify IO pin
pub type IO_t = *mut libc::c_void;
pub type I2CDevice = libc::c_int;
pub const I2CDEV_4: I2CDevice = 3;
pub const I2CDEV_3: I2CDevice = 2;
pub const I2CDEV_2: I2CDevice = 1;
pub const I2CDEV_1: I2CDevice = 0;
pub const I2CINVALID: I2CDevice = -1;
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
pub struct accDev_s {
    pub initFn: sensorAccInitFuncPtr,
    pub readFn: sensorAccReadFuncPtr,
    pub bus: busDevice_t,
    pub acc_1G: uint16_t,
    pub ADCRaw: [int16_t; 3],
    pub mpuDetectionResult: mpuDetectionResult_t,
    pub accAlign: sensor_align_e,
    pub dataReady: bool,
    pub acc_high_fsr: bool,
    pub revisionCode: libc::c_char,
    pub filler: [uint8_t; 2],
}
pub type mpuDetectionResult_t = mpuDetectionResult_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mpuDetectionResult_s {
    pub sensor: mpuSensor_e,
    pub resolution: mpu6050Resolution_e,
}
pub type mpu6050Resolution_e = libc::c_uint;
pub const MPU_FULL_RESOLUTION: mpu6050Resolution_e = 1;
pub const MPU_HALF_RESOLUTION: mpu6050Resolution_e = 0;
pub type mpuSensor_e = libc::c_uint;
pub const BMI_160_SPI: mpuSensor_e = 12;
pub const ICM_20689_SPI: mpuSensor_e = 11;
pub const ICM_20649_SPI: mpuSensor_e = 10;
pub const ICM_20608_SPI: mpuSensor_e = 9;
pub const ICM_20602_SPI: mpuSensor_e = 8;
pub const ICM_20601_SPI: mpuSensor_e = 7;
pub const MPU_9250_SPI: mpuSensor_e = 6;
pub const MPU_65xx_SPI: mpuSensor_e = 5;
pub const MPU_65xx_I2C: mpuSensor_e = 4;
pub const MPU_60x0_SPI: mpuSensor_e = 3;
pub const MPU_60x0: mpuSensor_e = 2;
pub const MPU_3050: mpuSensor_e = 1;
pub const MPU_NONE: mpuSensor_e = 0;
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
pub type busDevice_t = busDevice_s;
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
pub type busType_e = libc::c_uint;
pub const BUSTYPE_MPU_SLAVE: busType_e = 3;
pub const BUSTYPE_SPI: busType_e = 2;
pub const BUSTYPE_I2C: busType_e = 1;
pub const BUSTYPE_NONE: busType_e = 0;
pub type sensorAccReadFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut accDev_s) -> bool>;
pub type sensorAccInitFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut accDev_s) -> ()>;
pub type accDev_t = accDev_s;
unsafe extern "C" fn bma280Init(mut acc: *mut accDev_t) {
    // initialize function
    // read 3 axis data function
    // a revision code for the sensor, if known
    i2cWrite(I2CDEV_2, 0x18 as libc::c_int as uint8_t,
             0xf as libc::c_int as uint8_t,
             0x8 as libc::c_int as uint8_t); // +-8g range
    i2cWrite(I2CDEV_2, 0x18 as libc::c_int as uint8_t,
             0x10 as libc::c_int as uint8_t,
             0xe as libc::c_int as uint8_t); // 500Hz BW
    (*acc).acc_1G = (512 as libc::c_int * 8 as libc::c_int) as uint16_t;
}
unsafe extern "C" fn bma280Read(mut acc: *mut accDev_t) -> bool {
    let mut buf: [uint8_t; 6] = [0; 6];
    if !i2cRead(I2CDEV_2, 0x18 as libc::c_int as uint8_t,
                0x2 as libc::c_int as uint8_t, 6 as libc::c_int as uint8_t,
                buf.as_mut_ptr()) {
        return 0 as libc::c_int != 0
    }
    // Data format is lsb<5:0><crap><new_data_bit> | msb<13:6>
    (*acc).ADCRaw[0 as libc::c_int as usize] =
        ((buf[0 as libc::c_int as usize] as libc::c_int >> 2 as libc::c_int) +
             ((buf[1 as libc::c_int as usize] as libc::c_int) <<
                  8 as libc::c_int)) as int16_t;
    (*acc).ADCRaw[1 as libc::c_int as usize] =
        ((buf[2 as libc::c_int as usize] as libc::c_int >> 2 as libc::c_int) +
             ((buf[3 as libc::c_int as usize] as libc::c_int) <<
                  8 as libc::c_int)) as int16_t;
    (*acc).ADCRaw[2 as libc::c_int as usize] =
        ((buf[4 as libc::c_int as usize] as libc::c_int >> 2 as libc::c_int) +
             ((buf[5 as libc::c_int as usize] as libc::c_int) <<
                  8 as libc::c_int)) as int16_t;
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
pub unsafe extern "C" fn bma280Detect(mut acc: *mut accDev_t) -> bool {
    let mut sig: uint8_t = 0 as libc::c_int as uint8_t;
    let mut ack: bool =
        i2cRead(I2CDEV_2, 0x18 as libc::c_int as uint8_t,
                0 as libc::c_int as uint8_t, 1 as libc::c_int as uint8_t,
                &mut sig);
    if !ack || sig as libc::c_int != 0xfb as libc::c_int {
        return 0 as libc::c_int != 0
    }
    (*acc).initFn =
        Some(bma280Init as unsafe extern "C" fn(_: *mut accDev_t) -> ());
    (*acc).readFn =
        Some(bma280Read as unsafe extern "C" fn(_: *mut accDev_t) -> bool);
    return 1 as libc::c_int != 0;
}
