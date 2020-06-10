use ::libc;
extern "C" {
    #[no_mangle]
    fn busWriteRegister(bus: *const busDevice_t, reg: uint8_t, data: uint8_t)
     -> bool;
    #[no_mangle]
    fn busReadRegisterBuffer(bus: *const busDevice_t, reg: uint8_t,
                             data: *mut uint8_t, length: uint8_t) -> bool;
    #[no_mangle]
    fn spiBusWriteRegister(bus: *const busDevice_t, reg: uint8_t,
                           data: uint8_t) -> bool;
    #[no_mangle]
    fn spiBusReadRegisterBuffer(bus: *const busDevice_t, reg: uint8_t,
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
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    fn rescheduleTask(taskId: cfTaskId_e, newPeriodMicros: uint32_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
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
pub type extiCallbackRec_t = extiCallbackRec_s;
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
// microsecond time
pub type timeUs_t = uint32_t;
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
pub type ak8963ReadState_e = libc::c_uint;
pub const WAITING_FOR_DATA: ak8963ReadState_e = 2;
pub const WAITING_FOR_STATUS: ak8963ReadState_e = 1;
pub const CHECK_STATUS: ak8963ReadState_e = 0;
pub type queuedReadState_t = queuedReadState_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct queuedReadState_s {
    pub waiting: bool,
    pub len: uint8_t,
    pub readStartedAt: uint32_t,
}
pub type cfTaskId_e = libc::c_uint;
pub const TASK_SELF: cfTaskId_e = 25;
pub const TASK_NONE: cfTaskId_e = 24;
pub const TASK_COUNT: cfTaskId_e = 24;
pub const TASK_PINIOBOX: cfTaskId_e = 23;
pub const TASK_RCDEVICE: cfTaskId_e = 22;
pub const TASK_CAMCTRL: cfTaskId_e = 21;
pub const TASK_VTXCTRL: cfTaskId_e = 20;
pub const TASK_CMS: cfTaskId_e = 19;
pub const TASK_ESC_SENSOR: cfTaskId_e = 18;
pub const TASK_LEDSTRIP: cfTaskId_e = 17;
pub const TASK_TELEMETRY: cfTaskId_e = 16;
pub const TASK_ALTITUDE: cfTaskId_e = 15;
pub const TASK_RANGEFINDER: cfTaskId_e = 14;
pub const TASK_BARO: cfTaskId_e = 13;
pub const TASK_COMPASS: cfTaskId_e = 12;
pub const TASK_BEEPER: cfTaskId_e = 11;
pub const TASK_BATTERY_ALERTS: cfTaskId_e = 10;
pub const TASK_BATTERY_CURRENT: cfTaskId_e = 9;
pub const TASK_BATTERY_VOLTAGE: cfTaskId_e = 8;
pub const TASK_DISPATCH: cfTaskId_e = 7;
pub const TASK_SERIAL: cfTaskId_e = 6;
pub const TASK_RX: cfTaskId_e = 5;
pub const TASK_ATTITUDE: cfTaskId_e = 4;
pub const TASK_ACCEL: cfTaskId_e = 3;
pub const TASK_GYROPID: cfTaskId_e = 2;
pub const TASK_MAIN: cfTaskId_e = 1;
pub const TASK_SYSTEM: cfTaskId_e = 0;
#[inline(always)]
unsafe extern "C" fn __enable_irq() {
    asm!("cpsie i" : : : "memory" : "volatile");
}
#[inline(always)]
unsafe extern "C" fn __disable_irq() {
    asm!("cpsid i" : : : "memory" : "volatile");
}
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
unsafe extern "C" fn ak8963SpiWriteRegisterDelay(mut bus: *const busDevice_t,
                                                 mut reg: uint8_t,
                                                 mut data: uint8_t) -> bool {
    spiBusWriteRegister(bus, reg, data);
    delayMicroseconds(10 as libc::c_int as timeUs_t);
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn ak8963SlaveReadRegisterBuffer(mut slavedev:
                                                       *const busDevice_t,
                                                   mut reg: uint8_t,
                                                   mut buf: *mut uint8_t,
                                                   mut len: uint8_t) -> bool {
    let mut bus: *const busDevice_t = (*slavedev).busdev_u.mpuSlave.master;
    // initialize function
    // read 3 axis data function
    // time read was queued in micros.
    ak8963SpiWriteRegisterDelay(bus, 0x25 as libc::c_int as uint8_t,
                                ((*slavedev).busdev_u.mpuSlave.address as
                                     libc::c_int | 0x80 as libc::c_int) as
                                    uint8_t); // set I2C slave address for read
    ak8963SpiWriteRegisterDelay(bus, 0x26 as libc::c_int as uint8_t,
                                reg); // set I2C slave register
    ak8963SpiWriteRegisterDelay(bus, 0x27 as libc::c_int as uint8_t,
                                (len as libc::c_int & 0xf as libc::c_int |
                                     0x80 as libc::c_int) as
                                    uint8_t); // read number of bytes
    delay(4 as libc::c_int as timeMs_t); // read I2C
    __disable_irq(); // set I2C slave address for write
    let mut ack: bool =
        spiBusReadRegisterBuffer(bus, 0x49 as libc::c_int as uint8_t, buf,
                                 len); // set I2C slave register
    __enable_irq(); // set I2C sLave value
    return ack; // write 1 byte
}
unsafe extern "C" fn ak8963SlaveWriteRegister(mut slavedev:
                                                  *const busDevice_t,
                                              mut reg: uint8_t,
                                              mut data: uint8_t) -> bool {
    let mut bus: *const busDevice_t =
        (*slavedev).busdev_u.mpuSlave.master; // set I2C slave address for read
    ak8963SpiWriteRegisterDelay(bus, 0x25 as libc::c_int as uint8_t,
                                (*slavedev).busdev_u.mpuSlave.address); // set I2C slave register
    ak8963SpiWriteRegisterDelay(bus, 0x26 as libc::c_int as uint8_t,
                                reg); // read number of bytes
    ak8963SpiWriteRegisterDelay(bus, 0x63 as libc::c_int as uint8_t,
                                data); // read I2C buffer
    ak8963SpiWriteRegisterDelay(bus, 0x27 as libc::c_int as uint8_t,
                                (1 as libc::c_int & 0xf as libc::c_int |
                                     0x80 as libc::c_int) as uint8_t);
    return 1 as libc::c_int != 0;
}
static mut queuedRead: queuedReadState_t =
    {
        let mut init =
            queuedReadState_s{waiting: 0 as libc::c_int != 0,
                              len: 0 as libc::c_int as uint8_t,
                              readStartedAt: 0 as libc::c_int as uint32_t,};
        init
    };
unsafe extern "C" fn ak8963SlaveStartRead(mut slavedev: *const busDevice_t,
                                          mut reg: uint8_t, mut len: uint8_t)
 -> bool {
    if queuedRead.waiting { return 0 as libc::c_int != 0 }
    let mut bus: *const busDevice_t = (*slavedev).busdev_u.mpuSlave.master;
    queuedRead.len = len;
    ak8963SpiWriteRegisterDelay(bus, 0x25 as libc::c_int as uint8_t,
                                ((*slavedev).busdev_u.mpuSlave.address as
                                     libc::c_int | 0x80 as libc::c_int) as
                                    uint8_t);
    ak8963SpiWriteRegisterDelay(bus, 0x26 as libc::c_int as uint8_t, reg);
    ak8963SpiWriteRegisterDelay(bus, 0x27 as libc::c_int as uint8_t,
                                (len as libc::c_int & 0xf as libc::c_int |
                                     0x80 as libc::c_int) as uint8_t);
    queuedRead.readStartedAt = micros();
    queuedRead.waiting = 1 as libc::c_int != 0;
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn ak8963SlaveQueuedReadTimeRemaining() -> uint32_t {
    if !queuedRead.waiting { return 0 as libc::c_int as uint32_t }
    let mut timeSinceStarted: int32_t =
        micros().wrapping_sub(queuedRead.readStartedAt) as int32_t;
    let mut timeRemaining: int32_t = 8000 as libc::c_int - timeSinceStarted;
    if timeRemaining < 0 as libc::c_int {
        return 0 as libc::c_int as uint32_t
    }
    return timeRemaining as uint32_t;
}
unsafe extern "C" fn ak8963SlaveCompleteRead(mut slavedev: *const busDevice_t,
                                             mut buf: *mut uint8_t) -> bool {
    let mut timeRemaining: uint32_t = ak8963SlaveQueuedReadTimeRemaining();
    let mut bus: *const busDevice_t = (*slavedev).busdev_u.mpuSlave.master;
    if timeRemaining > 0 as libc::c_int as libc::c_uint {
        delayMicroseconds(timeRemaining);
    }
    queuedRead.waiting = 0 as libc::c_int != 0;
    spiBusReadRegisterBuffer(bus, 0x49 as libc::c_int as uint8_t, buf,
                             queuedRead.len);
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn ak8963SlaveReadData(mut busdev: *const busDevice_t,
                                         mut buf: *mut uint8_t) -> bool {
    static mut state: ak8963ReadState_e = CHECK_STATUS;
    let mut ack: bool = 0 as libc::c_int != 0;
    // we currently need a different approach for the MPU9250 connected via SPI.
    // we cannot use the ak8963SlaveReadRegisterBuffer() method for SPI, it is to slow and blocks for far too long.
    let mut retry: bool = 1 as libc::c_int != 0;
    loop  {
        match state as libc::c_uint {
            0 => {
                ak8963SlaveStartRead(busdev, 0x2 as libc::c_int as uint8_t,
                                     1 as libc::c_int as uint8_t);
                state = WAITING_FOR_STATUS;
                return 0 as libc::c_int != 0
            }
            1 => {
                let mut timeRemaining: uint32_t =
                    ak8963SlaveQueuedReadTimeRemaining();
                if timeRemaining != 0 { return 0 as libc::c_int != 0 }
                ack =
                    ak8963SlaveCompleteRead(busdev,
                                            &mut *buf.offset(0 as libc::c_int
                                                                 as isize));
                let mut status: uint8_t =
                    *buf.offset(0 as libc::c_int as isize);
                if !ack ||
                       status as libc::c_int & 0x1 as libc::c_int ==
                           0 as libc::c_int {
                    // too early. queue the status read again
                    state = CHECK_STATUS;
                    if retry {
                        retry = 0 as libc::c_int != 0
                    } else { return 0 as libc::c_int != 0 }
                } else {
                    // read the 6 bytes of data and the status2 register
                    ak8963SlaveStartRead(busdev,
                                         0x3 as libc::c_int as uint8_t,
                                         7 as libc::c_int as
                                             uint8_t); // start reading again    uint8_t status2 = buf[6];
                    state =
                        WAITING_FOR_DATA; // power down before entering fuse mode
                    return 0 as libc::c_int != 0
                }
            }
            2 => {
                let mut timeRemaining_0: uint32_t =
                    ak8963SlaveQueuedReadTimeRemaining(); // Enter Fuse ROM access mode
                if timeRemaining_0 != 0 {
                    return 0 as libc::c_int != 0
                } // Read the x-, y-, and z-axis calibration values
                ack =
                    ak8963SlaveCompleteRead(busdev,
                                            &mut *buf.offset(0 as libc::c_int
                                                                 as
                                                                 isize)); // power down after reading.
                state = CHECK_STATUS;
                break ;
            }
            _ => { break ; }
        }
    }
    return ack;
}
unsafe extern "C" fn ak8963ReadRegisterBuffer(mut busdev: *const busDevice_t,
                                              mut reg: uint8_t,
                                              mut buf: *mut uint8_t,
                                              mut len: uint8_t) -> bool {
    if (*busdev).bustype as libc::c_uint ==
           BUSTYPE_MPU_SLAVE as libc::c_int as libc::c_uint {
        return ak8963SlaveReadRegisterBuffer(busdev, reg, buf, len)
    }
    return busReadRegisterBuffer(busdev, reg, buf, len);
}
unsafe extern "C" fn ak8963WriteRegister(mut busdev: *const busDevice_t,
                                         mut reg: uint8_t, mut data: uint8_t)
 -> bool {
    if (*busdev).bustype as libc::c_uint ==
           BUSTYPE_MPU_SLAVE as libc::c_int as libc::c_uint {
        return ak8963SlaveWriteRegister(busdev, reg, data)
    }
    return busWriteRegister(busdev, reg, data);
}
unsafe extern "C" fn ak8963DirectReadData(mut busdev: *const busDevice_t,
                                          mut buf: *mut uint8_t) -> bool {
    let mut status: uint8_t = 0;
    let mut ack: bool =
        ak8963ReadRegisterBuffer(busdev, 0x2 as libc::c_int as uint8_t,
                                 &mut status, 1 as libc::c_int as uint8_t);
    if !ack || status as libc::c_int & 0x1 as libc::c_int == 0 as libc::c_int
       {
        return 0 as libc::c_int != 0
    }
    return ak8963ReadRegisterBuffer(busdev, 0x3 as libc::c_int as uint8_t,
                                    buf, 7 as libc::c_int as uint8_t);
}
unsafe extern "C" fn parseMag(mut raw: *mut uint8_t, mut gain: int16_t)
 -> int16_t {
    let mut ret: libc::c_int =
        ((*raw.offset(1 as libc::c_int as isize) as libc::c_int) <<
             8 as libc::c_int |
             *raw.offset(0 as libc::c_int as isize) as libc::c_int) as int16_t
            as libc::c_int * gain as libc::c_int / 256 as libc::c_int;
    return constrain(ret, -(32767 as libc::c_int) - 1 as libc::c_int,
                     32767 as libc::c_int) as int16_t;
}
unsafe extern "C" fn ak8963Read(mut mag: *mut magDev_t,
                                mut magData: *mut int16_t) -> bool {
    let mut ack: bool = 0 as libc::c_int != 0;
    let mut buf: [uint8_t; 7] = [0; 7];
    let mut busdev: *const busDevice_t = &mut (*mag).busdev;
    match (*busdev).bustype as libc::c_uint {
        1 | 2 => { ack = ak8963DirectReadData(busdev, buf.as_mut_ptr()) }
        3 => { ack = ak8963SlaveReadData(busdev, buf.as_mut_ptr()) }
        _ => { }
    }
    let mut status2: uint8_t = buf[6 as libc::c_int as usize];
    if !ack { return 0 as libc::c_int != 0 }
    ak8963WriteRegister(busdev, 0xa as libc::c_int as uint8_t,
                        (0x10 as libc::c_int | 0x1 as libc::c_int) as
                            uint8_t);
    if status2 as libc::c_int & 0x8 as libc::c_int != 0 {
        return 0 as libc::c_int != 0
    }
    *magData.offset(X as libc::c_int as isize) =
        parseMag(buf.as_mut_ptr().offset(0 as libc::c_int as isize),
                 (*mag).magGain[X as libc::c_int as usize]);
    *magData.offset(Y as libc::c_int as isize) =
        parseMag(buf.as_mut_ptr().offset(2 as libc::c_int as isize),
                 (*mag).magGain[Y as libc::c_int as usize]);
    *magData.offset(Z as libc::c_int as isize) =
        parseMag(buf.as_mut_ptr().offset(4 as libc::c_int as isize),
                 (*mag).magGain[Z as libc::c_int as usize]);
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn ak8963Init(mut mag: *mut magDev_t) -> bool {
    let mut asa: [uint8_t; 3] = [0; 3];
    let mut status: uint8_t = 0;
    let mut busdev: *const busDevice_t = &mut (*mag).busdev;
    ak8963WriteRegister(busdev, 0xa as libc::c_int as uint8_t,
                        0 as libc::c_int as uint8_t);
    ak8963WriteRegister(busdev, 0xa as libc::c_int as uint8_t,
                        0xf as libc::c_int as uint8_t);
    ak8963ReadRegisterBuffer(busdev, 0x10 as libc::c_int as uint8_t,
                             asa.as_mut_ptr(),
                             ::core::mem::size_of::<[uint8_t; 3]>() as
                                 libc::c_ulong as uint8_t);
    (*mag).magGain[X as libc::c_int as usize] =
        (asa[X as libc::c_int as usize] as libc::c_int + 128 as libc::c_int)
            as int16_t;
    (*mag).magGain[Y as libc::c_int as usize] =
        (asa[Y as libc::c_int as usize] as libc::c_int + 128 as libc::c_int)
            as int16_t;
    (*mag).magGain[Z as libc::c_int as usize] =
        (asa[Z as libc::c_int as usize] as libc::c_int + 128 as libc::c_int)
            as int16_t;
    ak8963WriteRegister(busdev, 0xa as libc::c_int as uint8_t,
                        0 as libc::c_int as uint8_t);
    // Clear status registers
    ak8963ReadRegisterBuffer(busdev, 0x2 as libc::c_int as uint8_t,
                             &mut status, 1 as libc::c_int as uint8_t);
    ak8963ReadRegisterBuffer(busdev, 0x9 as libc::c_int as uint8_t,
                             &mut status, 1 as libc::c_int as uint8_t);
    // Trigger first measurement
    ak8963WriteRegister(busdev, 0xa as libc::c_int as uint8_t,
                        (0x10 as libc::c_int | 0x1 as libc::c_int) as
                            uint8_t);
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn ak8963BusInit(mut busdev: *const busDevice_t) {
    match (*busdev).bustype as libc::c_uint {
        3 => {
            rescheduleTask(TASK_COMPASS,
                           (1000000 as libc::c_int / 40 as libc::c_int) as
                               uint32_t);
            // initialze I2C master via SPI bus
            ak8963SpiWriteRegisterDelay((*busdev).busdev_u.mpuSlave.master,
                                        0x37 as libc::c_int as uint8_t,
                                        ((1 as libc::c_int) <<
                                             4 as libc::c_int |
                                             (1 as libc::c_int) <<
                                                 1 as libc::c_int) as
                                            uint8_t); // I2C multi-master / 400kHz
            ak8963SpiWriteRegisterDelay((*busdev).busdev_u.mpuSlave.master,
                                        0x24 as libc::c_int as uint8_t,
                                        0xd as libc::c_int as
                                            uint8_t); // I2C master mode, SPI mode only
            ak8963SpiWriteRegisterDelay((*busdev).busdev_u.mpuSlave.master,
                                        0x6a as libc::c_int as uint8_t,
                                        0x30 as libc::c_int as uint8_t);
        }
        1 | _ => { }
    };
}
#[no_mangle]
pub unsafe extern "C" fn ak8963BusDeInit(mut busdev: *const busDevice_t) {
    match (*busdev).bustype as libc::c_uint {
        3 => {
            ak8963SpiWriteRegisterDelay((*busdev).busdev_u.mpuSlave.master,
                                        0x37 as libc::c_int as uint8_t,
                                        ((1 as libc::c_int) <<
                                             4 as libc::c_int) as uint8_t);
        }
        1 | _ => { }
    };
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
pub unsafe extern "C" fn ak8963Detect(mut mag: *mut magDev_t) -> bool {
    let mut sig: uint8_t = 0 as libc::c_int as uint8_t; // reset MAG
    let mut busdev: *mut busDevice_t = &mut (*mag).busdev; // check for AK8963
    if ((*busdev).bustype as libc::c_uint ==
            BUSTYPE_I2C as libc::c_int as libc::c_uint ||
            (*busdev).bustype as libc::c_uint ==
                BUSTYPE_MPU_SLAVE as libc::c_int as libc::c_uint) &&
           (*busdev).busdev_u.mpuSlave.address as libc::c_int ==
               0 as libc::c_int {
        (*busdev).busdev_u.mpuSlave.address = 0xc as libc::c_int as uint8_t
    }
    ak8963BusInit(busdev);
    ak8963WriteRegister(busdev, 0xb as libc::c_int as uint8_t,
                        0x1 as libc::c_int as uint8_t);
    delay(4 as libc::c_int as timeMs_t);
    let mut ack: bool =
        ak8963ReadRegisterBuffer(busdev, 0 as libc::c_int as uint8_t,
                                 &mut sig, 1 as libc::c_int as uint8_t);
    if ack as libc::c_int != 0 && sig as libc::c_int == 0x48 as libc::c_int {
        // 0x48 / 01001000 / 'H'
        (*mag).init =
            Some(ak8963Init as
                     unsafe extern "C" fn(_: *mut magDev_t) -> bool);
        (*mag).read =
            Some(ak8963Read as
                     unsafe extern "C" fn(_: *mut magDev_t, _: *mut int16_t)
                         -> bool);
        return 1 as libc::c_int != 0
    }
    ak8963BusDeInit(busdev);
    return 0 as libc::c_int != 0;
}
