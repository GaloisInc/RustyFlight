use ::libc;
extern "C" {
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
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
pub type C2RustUnnamed = libc::c_uint;
pub const GPIO_Speed_50MHz: C2RustUnnamed = 3;
pub const GPIO_Speed_2MHz: C2RustUnnamed = 2;
pub const GPIO_Speed_10MHz: C2RustUnnamed = 1;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const GPIO_Mode_AF_PP: C2RustUnnamed_0 = 24;
pub const GPIO_Mode_AF_OD: C2RustUnnamed_0 = 28;
pub const GPIO_Mode_Out_PP: C2RustUnnamed_0 = 16;
pub const GPIO_Mode_Out_OD: C2RustUnnamed_0 = 20;
pub const GPIO_Mode_IPU: C2RustUnnamed_0 = 72;
pub const GPIO_Mode_IPD: C2RustUnnamed_0 = 40;
pub const GPIO_Mode_IN_FLOATING: C2RustUnnamed_0 = 4;
pub const GPIO_Mode_AIN: C2RustUnnamed_0 = 0;
pub type resourceOwner_e = libc::c_uint;
pub const OWNER_TOTAL_COUNT: resourceOwner_e = 55;
pub const OWNER_SPI_PREINIT_OPU: resourceOwner_e = 54;
pub const OWNER_SPI_PREINIT_IPU: resourceOwner_e = 53;
pub const OWNER_USB_MSC_PIN: resourceOwner_e = 52;
pub const OWNER_PINIO: resourceOwner_e = 51;
pub const OWNER_RX_SPI: resourceOwner_e = 50;
pub const OWNER_RANGEFINDER: resourceOwner_e = 49;
pub const OWNER_TIMUP: resourceOwner_e = 48;
pub const OWNER_CAMERA_CONTROL: resourceOwner_e = 47;
pub const OWNER_ESCSERIAL: resourceOwner_e = 46;
pub const OWNER_RX_BIND_PLUG: resourceOwner_e = 45;
pub const OWNER_COMPASS_CS: resourceOwner_e = 44;
pub const OWNER_VTX: resourceOwner_e = 43;
pub const OWNER_TRANSPONDER: resourceOwner_e = 42;
pub const OWNER_LED_STRIP: resourceOwner_e = 41;
pub const OWNER_INVERTER: resourceOwner_e = 40;
pub const OWNER_RX_BIND: resourceOwner_e = 39;
pub const OWNER_OSD: resourceOwner_e = 38;
pub const OWNER_BEEPER: resourceOwner_e = 37;
pub const OWNER_USB_DETECT: resourceOwner_e = 36;
pub const OWNER_USB: resourceOwner_e = 35;
pub const OWNER_COMPASS_EXTI: resourceOwner_e = 34;
pub const OWNER_BARO_EXTI: resourceOwner_e = 33;
pub const OWNER_MPU_EXTI: resourceOwner_e = 32;
pub const OWNER_SPI_CS: resourceOwner_e = 31;
pub const OWNER_RX_SPI_CS: resourceOwner_e = 30;
pub const OWNER_OSD_CS: resourceOwner_e = 29;
pub const OWNER_MPU_CS: resourceOwner_e = 28;
pub const OWNER_BARO_CS: resourceOwner_e = 27;
pub const OWNER_FLASH_CS: resourceOwner_e = 26;
pub const OWNER_SDCARD_DETECT: resourceOwner_e = 25;
pub const OWNER_SDCARD_CS: resourceOwner_e = 24;
pub const OWNER_SDCARD: resourceOwner_e = 23;
pub const OWNER_I2C_SDA: resourceOwner_e = 22;
pub const OWNER_I2C_SCL: resourceOwner_e = 21;
pub const OWNER_SPI_MOSI: resourceOwner_e = 20;
pub const OWNER_SPI_MISO: resourceOwner_e = 19;
pub const OWNER_SPI_SCK: resourceOwner_e = 18;
pub const OWNER_SYSTEM: resourceOwner_e = 17;
pub const OWNER_SONAR_ECHO: resourceOwner_e = 16;
pub const OWNER_SONAR_TRIGGER: resourceOwner_e = 15;
pub const OWNER_TIMER: resourceOwner_e = 14;
pub const OWNER_PINDEBUG: resourceOwner_e = 13;
pub const OWNER_SERIAL_RX: resourceOwner_e = 12;
pub const OWNER_SERIAL_TX: resourceOwner_e = 11;
pub const OWNER_ADC_RSSI: resourceOwner_e = 10;
pub const OWNER_ADC_EXT: resourceOwner_e = 9;
pub const OWNER_ADC_CURR: resourceOwner_e = 8;
pub const OWNER_ADC_BATT: resourceOwner_e = 7;
pub const OWNER_ADC: resourceOwner_e = 6;
pub const OWNER_LED: resourceOwner_e = 5;
pub const OWNER_SERVO: resourceOwner_e = 4;
pub const OWNER_MOTOR: resourceOwner_e = 3;
pub const OWNER_PPMINPUT: resourceOwner_e = 2;
pub const OWNER_PWMINPUT: resourceOwner_e = 1;
pub const OWNER_FREE: resourceOwner_e = 0;
pub type ioTag_t = uint8_t;
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
// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
// packet tag to specify IO pin
// type specifying IO pin. Currently ioRec_t pointer, but this may change
// NONE initializer for ioTag_t variables
// NONE initializer for IO_t variable
// both ioTag_t and IO_t are guarantied to be zero if pinid is NONE (no pin)
// this simplifies initialization (globals are zeroed on start) and allows
//  omitting unused fields in structure initializers.
// it is also possible to use IO_t and ioTag_t as boolean value
//   TODO - this may conflict with requirement to generate warning/error on IO_t - ioTag_t assignment
//   IO_t being pointer is only possibility I know of ..
// pin config handling
// pin config is packed into ioConfig_t to decrease memory requirements
// IOCFG_x macros are defined for common combinations for all CPUs; this
//  helps masking CPU differences
pub type ioConfig_t = uint8_t;
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
pub type busDevice_t = busDevice_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct busDevice_s {
    pub bustype: busType_e,
    pub busdev_u: C2RustUnnamed_1,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_1 {
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
static mut device_id: uint8_t = 0;
#[inline]
unsafe extern "C" fn mma8451ConfigureInterrupt() {
    IOInit(IOGetByTag(((0 as libc::c_int + 1 as libc::c_int) <<
                           4 as libc::c_int | 5 as libc::c_int) as ioTag_t),
           OWNER_MPU_EXTI, 0 as libc::c_int as uint8_t);
    // TODO - maybe pullup / pulldown ?
    IOConfigGPIO(IOGetByTag(((0 as libc::c_int + 1 as libc::c_int) <<
                                 4 as libc::c_int | 5 as libc::c_int) as
                                ioTag_t),
                 (GPIO_Mode_IN_FLOATING as libc::c_int |
                      GPIO_Speed_2MHz as libc::c_int) as
                     ioConfig_t); // Interrupt polarity (active HIGH)
    i2cWrite(I2CDEV_2, 0x1c as libc::c_int as uint8_t,
             0x2c as libc::c_int as uint8_t,
             0x2 as libc::c_int as
                 uint8_t); // Enable DRDY interrupt (unused by this driver)
    i2cWrite(I2CDEV_2, 0x1c as libc::c_int as uint8_t,
             0x2d as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t);
    i2cWrite(I2CDEV_2, 0x1c as libc::c_int as uint8_t,
             0x2e as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t);
    // DRDY routed to INT2
}
unsafe extern "C" fn mma8452Init(mut acc: *mut accDev_t) {
    i2cWrite(I2CDEV_2, 0x1c as libc::c_int as uint8_t,
             0x2a as libc::c_int as uint8_t,
             0 as libc::c_int as
                 uint8_t); // Put device in standby to configure stuff
    i2cWrite(I2CDEV_2, 0x1c as libc::c_int as uint8_t,
             0xe as libc::c_int as uint8_t,
             0x2 as libc::c_int as
                 uint8_t); // High resolution measurement in both sleep and active modes
    i2cWrite(I2CDEV_2, 0x1c as libc::c_int as uint8_t,
             0xf as libc::c_int as uint8_t,
             0x3 as libc::c_int as
                 uint8_t); // Turn on measurements, low noise at max scale mode, Data Rate 800Hz. LNoise mode makes range +-4G.
    i2cWrite(I2CDEV_2, 0x1c as libc::c_int as uint8_t,
             0x2b as libc::c_int as uint8_t,
             (0x2 as libc::c_int | (0x2 as libc::c_int) << 3 as libc::c_int)
                 as uint8_t);
    mma8451ConfigureInterrupt();
    i2cWrite(I2CDEV_2, 0x1c as libc::c_int as uint8_t,
             0x2a as libc::c_int as uint8_t,
             (0x4 as libc::c_int | 0x1 as libc::c_int) as uint8_t);
    (*acc).acc_1G = 256 as libc::c_int as uint16_t;
}
unsafe extern "C" fn mma8452Read(mut acc: *mut accDev_t) -> bool {
    let mut buf: [uint8_t; 6] = [0; 6];
    if !i2cRead(I2CDEV_2, 0x1c as libc::c_int as uint8_t,
                0x1 as libc::c_int as uint8_t, 6 as libc::c_int as uint8_t,
                buf.as_mut_ptr()) {
        return 0 as libc::c_int != 0
    }
    (*acc).ADCRaw[0 as libc::c_int as usize] =
        ((((buf[0 as libc::c_int as usize] as libc::c_int) << 8 as libc::c_int
               | buf[1 as libc::c_int as usize] as libc::c_int) as int16_t as
              libc::c_int >> 2 as libc::c_int) / 4 as libc::c_int) as int16_t;
    (*acc).ADCRaw[1 as libc::c_int as usize] =
        ((((buf[2 as libc::c_int as usize] as libc::c_int) << 8 as libc::c_int
               | buf[3 as libc::c_int as usize] as libc::c_int) as int16_t as
              libc::c_int >> 2 as libc::c_int) / 4 as libc::c_int) as int16_t;
    (*acc).ADCRaw[2 as libc::c_int as usize] =
        ((((buf[4 as libc::c_int as usize] as libc::c_int) << 8 as libc::c_int
               | buf[5 as libc::c_int as usize] as libc::c_int) as int16_t as
              libc::c_int >> 2 as libc::c_int) / 4 as libc::c_int) as int16_t;
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
pub unsafe extern "C" fn mma8452Detect(mut acc: *mut accDev_t) -> bool {
    let mut sig: uint8_t = 0 as libc::c_int as uint8_t;
    let mut ack: bool =
        i2cRead(I2CDEV_2, 0x1c as libc::c_int as uint8_t,
                0xd as libc::c_int as uint8_t, 1 as libc::c_int as uint8_t,
                &mut sig);
    if !ack ||
           sig as libc::c_int != 0x2a as libc::c_int &&
               sig as libc::c_int != 0x1a as libc::c_int {
        return 0 as libc::c_int != 0
    }
    (*acc).initFn =
        Some(mma8452Init as unsafe extern "C" fn(_: *mut accDev_t) -> ());
    (*acc).readFn =
        Some(mma8452Read as unsafe extern "C" fn(_: *mut accDev_t) -> bool);
    device_id = sig;
    return 1 as libc::c_int != 0;
}
