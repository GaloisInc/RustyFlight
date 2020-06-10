use ::libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    // declare available IO pins. Available pins are specified per target
    #[no_mangle]
    fn IORead(io: IO_t) -> bool;
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn dmaGetRefByIdentifier(identifier: dmaIdentifier_e)
     -> *mut DMA_Stream_TypeDef;
    #[no_mangle]
    fn millis() -> timeMs_t;
    /* Prototype(s) -----------------------------------------------------------------------------------------------------*/
    #[no_mangle]
    static mut SD_CardInfo: SD_CardInfo_t;
    #[no_mangle]
    static mut SD_CardType: SD_CardType_t;
    #[no_mangle]
    fn SD_Initialize_LL(dma: *mut DMA_Stream_TypeDef);
    #[no_mangle]
    fn SD_Init() -> bool;
    #[no_mangle]
    fn SD_IsDetected() -> bool;
    #[no_mangle]
    fn SD_GetState() -> bool;
    #[no_mangle]
    fn SD_GetCardInfo() -> SD_Error_t;
    #[no_mangle]
    fn SD_ReadBlocks_DMA(ReadAddress: uint64_t, buffer: *mut uint32_t,
                         BlockSize: uint32_t, NumberOfBlocks: uint32_t)
     -> SD_Error_t;
    #[no_mangle]
    fn SD_CheckRead() -> SD_Error_t;
    #[no_mangle]
    fn SD_WriteBlocks_DMA(WriteAddress: uint64_t, buffer: *mut uint32_t,
                          BlockSize: uint32_t, NumberOfBlocks: uint32_t)
     -> SD_Error_t;
    #[no_mangle]
    fn SD_CheckWrite() -> SD_Error_t;
    #[no_mangle]
    static mut sdioConfig_System: sdioConfig_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type __uint64_t = libc::c_ulong;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
/* *
  * @brief DMA Controller
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_Stream_TypeDef {
    pub CR: uint32_t,
    pub NDTR: uint32_t,
    pub PAR: uint32_t,
    pub M0AR: uint32_t,
    pub M1AR: uint32_t,
    pub FCR: uint32_t,
}
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
pub type dmaIdentifier_e = libc::c_uint;
pub const DMA_LAST_HANDLER: dmaIdentifier_e = 16;
pub const DMA2_ST7_HANDLER: dmaIdentifier_e = 16;
pub const DMA2_ST6_HANDLER: dmaIdentifier_e = 15;
pub const DMA2_ST5_HANDLER: dmaIdentifier_e = 14;
pub const DMA2_ST4_HANDLER: dmaIdentifier_e = 13;
pub const DMA2_ST3_HANDLER: dmaIdentifier_e = 12;
pub const DMA2_ST2_HANDLER: dmaIdentifier_e = 11;
pub const DMA2_ST1_HANDLER: dmaIdentifier_e = 10;
pub const DMA2_ST0_HANDLER: dmaIdentifier_e = 9;
pub const DMA1_ST7_HANDLER: dmaIdentifier_e = 8;
pub const DMA1_ST6_HANDLER: dmaIdentifier_e = 7;
pub const DMA1_ST5_HANDLER: dmaIdentifier_e = 6;
pub const DMA1_ST4_HANDLER: dmaIdentifier_e = 5;
pub const DMA1_ST3_HANDLER: dmaIdentifier_e = 4;
pub const DMA1_ST2_HANDLER: dmaIdentifier_e = 3;
pub const DMA1_ST1_HANDLER: dmaIdentifier_e = 2;
pub const DMA1_ST0_HANDLER: dmaIdentifier_e = 1;
pub const DMA_NONE: dmaIdentifier_e = 0;
pub type timeMs_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct sdcardConfig_s {
    pub useDma: uint8_t,
    pub enabled: uint8_t,
    pub device: uint8_t,
    pub cardDetectTag: ioTag_t,
    pub chipSelectTag: ioTag_t,
    pub cardDetectInverted: uint8_t,
    pub dmaIdentifier: uint8_t,
    pub dmaChannel: uint8_t,
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
pub type sdcardConfig_t = sdcardConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct sdcardMetadata_s {
    pub numBlocks: uint32_t,
    pub oemID: uint16_t,
    pub manufacturerID: uint8_t,
    pub productName: [libc::c_char; 5],
    pub productSerial: uint32_t,
    pub productRevisionMajor: uint8_t,
    pub productRevisionMinor: uint8_t,
    pub productionYear: uint16_t,
    pub productionMonth: uint8_t,
}
pub type sdcardMetadata_t = sdcardMetadata_s;
pub type sdcardBlockOperation_e = libc::c_uint;
pub const SDCARD_BLOCK_OPERATION_ERASE: sdcardBlockOperation_e = 2;
pub const SDCARD_BLOCK_OPERATION_WRITE: sdcardBlockOperation_e = 1;
pub const SDCARD_BLOCK_OPERATION_READ: sdcardBlockOperation_e = 0;
pub type sdcardOperationStatus_e = libc::c_uint;
pub const SDCARD_OPERATION_FAILURE: sdcardOperationStatus_e = 3;
pub const SDCARD_OPERATION_SUCCESS: sdcardOperationStatus_e = 2;
pub const SDCARD_OPERATION_BUSY: sdcardOperationStatus_e = 1;
pub const SDCARD_OPERATION_IN_PROGRESS: sdcardOperationStatus_e = 0;
pub type sdcard_operationCompleteCallback_c
    =
    Option<unsafe extern "C" fn(_: sdcardBlockOperation_e, _: uint32_t,
                                _: *mut uint8_t, _: uint32_t) -> ()>;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct sdcard_t {
    pub pendingOperation: C2RustUnnamed,
    pub operationStartTime: uint32_t,
    pub failureCount: uint8_t,
    pub version: uint8_t,
    pub highCapacity: bool,
    pub multiWriteNextBlock: uint32_t,
    pub multiWriteBlocksRemain: uint32_t,
    pub state: sdcardState_e,
    pub metadata: sdcardMetadata_t,
    pub csd: sdcardCSD_t,
    pub enabled: bool,
    pub cardDetectPin: IO_t,
    pub dma: dmaIdentifier_e,
    pub dmaChannel: uint8_t,
    pub useCache: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct sdcardCSD_t {
    pub data: [uint8_t; 16],
}
pub type sdcardState_e = libc::c_uint;
pub const SDCARD_STATE_STOPPING_MULTIPLE_BLOCK_WRITE: sdcardState_e = 9;
pub const SDCARD_STATE_WRITING_MULTIPLE_BLOCKS: sdcardState_e = 8;
pub const SDCARD_STATE_WAITING_FOR_WRITE: sdcardState_e = 7;
pub const SDCARD_STATE_SENDING_WRITE: sdcardState_e = 6;
pub const SDCARD_STATE_READING: sdcardState_e = 5;
pub const SDCARD_STATE_READY: sdcardState_e = 4;
pub const SDCARD_STATE_INITIALIZATION_RECEIVE_CID: sdcardState_e = 3;
pub const SDCARD_STATE_CARD_INIT_IN_PROGRESS: sdcardState_e = 2;
pub const SDCARD_STATE_RESET: sdcardState_e = 1;
pub const SDCARD_STATE_NOT_PRESENT: sdcardState_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed {
    pub buffer: *mut uint8_t,
    pub blockIndex: uint32_t,
    pub chunkIndex: uint8_t,
    pub callback: sdcard_operationCompleteCallback_c,
    pub callbackData: uint32_t,
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
pub type sdioConfig_t = sdioConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct sdioConfig_s {
    pub clockBypass: uint8_t,
    pub useCache: uint8_t,
}
pub const SD_OK: SD_Error_t = 0;
pub type SD_Error_t = libc::c_uint;
pub const SD_BUSY: SD_Error_t = 43;
pub const SD_ERROR: SD_Error_t = 42;
pub const SD_UNSUPPORTED_HW: SD_Error_t = 41;
pub const SD_UNSUPPORTED_FEATURE: SD_Error_t = 40;
pub const SD_INVALID_PARAMETER: SD_Error_t = 39;
pub const SD_REQUEST_NOT_APPLICABLE: SD_Error_t = 38;
pub const SD_REQUEST_PENDING: SD_Error_t = 37;
pub const SD_NOT_CONFIGURED: SD_Error_t = 36;
pub const SD_INTERNAL_ERROR: SD_Error_t = 35;
pub const SD_OUT_OF_BOUND: SD_Error_t = 34;
pub const SD_SDMMC_UNKNOWN_FUNCTION: SD_Error_t = 33;
pub const SD_SDMMC_FUNCTION_FAILED: SD_Error_t = 32;
pub const SD_SDMMC_FUNCTION_BUSY: SD_Error_t = 31;
pub const SD_SDMMC_DISABLED: SD_Error_t = 30;
pub const SD_SWITCH_ERROR: SD_Error_t = 29;
pub const SD_ADDR_OUT_OF_RANGE: SD_Error_t = 28;
pub const SD_INVALID_VOLTRANGE: SD_Error_t = 27;
pub const SD_AKE_SEQ_ERROR: SD_Error_t = 26;
pub const SD_ERASE_RESET: SD_Error_t = 25;
pub const SD_CARD_ECC_DISABLED: SD_Error_t = 24;
pub const SD_WP_ERASE_SKIP: SD_Error_t = 23;
pub const SD_CID_CSD_OVERWRITE: SD_Error_t = 22;
pub const SD_STREAM_WRITE_OVERRUN: SD_Error_t = 21;
pub const SD_STREAM_READ_UNDERRUN: SD_Error_t = 20;
pub const SD_GENERAL_UNKNOWN_ERROR: SD_Error_t = 19;
pub const SD_CC_ERROR: SD_Error_t = 18;
pub const SD_CARD_ECC_FAILED: SD_Error_t = 17;
pub const SD_ILLEGAL_CMD: SD_Error_t = 16;
pub const SD_COM_CRC_FAILED: SD_Error_t = 15;
pub const SD_LOCK_UNLOCK_FAILED: SD_Error_t = 14;
pub const SD_WRITE_PROT_VIOLATION: SD_Error_t = 13;
pub const SD_BAD_ERASE_PARAM: SD_Error_t = 12;
pub const SD_ERASE_SEQ_ERR: SD_Error_t = 11;
pub const SD_BLOCK_LEN_ERR: SD_Error_t = 10;
pub const SD_ADDR_MISALIGNED: SD_Error_t = 9;
pub const SD_CMD_OUT_OF_RANGE: SD_Error_t = 8;
pub const SD_START_BIT_ERR: SD_Error_t = 7;
pub const SD_RX_OVERRUN: SD_Error_t = 6;
pub const SD_TX_UNDERRUN: SD_Error_t = 5;
pub const SD_DATA_TIMEOUT: SD_Error_t = 4;
pub const SD_CMD_RSP_TIMEOUT: SD_Error_t = 3;
pub const SD_DATA_CRC_FAIL: SD_Error_t = 2;
pub const SD_CMD_CRC_FAIL: SD_Error_t = 1;
pub const SDCARD_RECEIVE_ERROR: sdcardReceiveBlockStatus_e = 2;
pub const SDCARD_RECEIVE_BLOCK_IN_PROGRESS: sdcardReceiveBlockStatus_e = 1;
pub const SDCARD_RECEIVE_SUCCESS: sdcardReceiveBlockStatus_e = 0;
pub type sdcardReceiveBlockStatus_e = libc::c_uint;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SD_CID_t {
    pub ManufacturerID: uint8_t,
    pub OEM_AppliID: uint16_t,
    pub ProdName1: uint32_t,
    pub ProdName2: uint8_t,
    pub ProdRev: uint8_t,
    pub ProdSN: uint32_t,
    pub Reserved1: uint8_t,
    pub ManufactDate: uint16_t,
    pub CID_CRC: uint8_t,
    pub Reserved2: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SD_CardInfo_t {
    pub SD_csd: SD_CSD_t,
    pub SD_cid: SD_CID_t,
    pub CardCapacity: uint64_t,
    pub CardBlockSize: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SD_CSD_t {
    pub CSDStruct: uint8_t,
    pub SysSpecVersion: uint8_t,
    pub Reserved1: uint8_t,
    pub TAAC: uint8_t,
    pub NSAC: uint8_t,
    pub MaxBusClkFrec: uint8_t,
    pub CardComdClasses: uint16_t,
    pub RdBlockLen: uint8_t,
    pub PartBlockRead: uint8_t,
    pub WrBlockMisalign: uint8_t,
    pub RdBlockMisalign: uint8_t,
    pub DSRImpl: uint8_t,
    pub Reserved2: uint8_t,
    pub DeviceSize: uint32_t,
    pub MaxRdCurrentVDDMin: uint8_t,
    pub MaxRdCurrentVDDMax: uint8_t,
    pub MaxWrCurrentVDDMin: uint8_t,
    pub MaxWrCurrentVDDMax: uint8_t,
    pub DeviceSizeMul: uint8_t,
    pub EraseGrSize: uint8_t,
    pub EraseGrMul: uint8_t,
    pub WrProtectGrSize: uint8_t,
    pub WrProtectGrEnable: uint8_t,
    pub ManDeflECC: uint8_t,
    pub WrSpeedFact: uint8_t,
    pub MaxWrBlockLen: uint8_t,
    pub WriteBlockPaPartial: uint8_t,
    pub Reserved3: uint8_t,
    pub ContentProtectAppli: uint8_t,
    pub FileFormatGrouop: uint8_t,
    pub CopyFlag: uint8_t,
    pub PermWrProtect: uint8_t,
    pub TempWrProtect: uint8_t,
    pub FileFormat: uint8_t,
    pub ECC: uint8_t,
    pub CSD_CRC: uint8_t,
    pub Reserved4: uint8_t,
}
pub type SD_CardType_t = libc::c_uint;
pub const SD_HIGH_CAPACITY_MMC: SD_CardType_t = 7;
pub const SD_SECURE_DIGITAL_IO_COMBO: SD_CardType_t = 6;
pub const SD_HIGH_SPEED_MULTIMEDIA: SD_CardType_t = 5;
pub const SD_SECURE_DIGITAL_IO: SD_CardType_t = 4;
pub const SD_MULTIMEDIA: SD_CardType_t = 3;
pub const SD_HIGH_CAPACITY: SD_CardType_t = 2;
pub const SD_STD_CAPACITY_V2_0: SD_CardType_t = 1;
pub const SD_STD_CAPACITY_V1_1: SD_CardType_t = 0;
#[inline]
unsafe extern "C" fn sdioConfig() -> *const sdioConfig_t {
    return &mut sdioConfig_System;
}
#[no_mangle]
pub static mut writeCache: [uint8_t; 8192] = [0; 8192];
#[no_mangle]
pub static mut cacheCount: uint32_t = 0 as libc::c_int as uint32_t;
#[no_mangle]
pub unsafe extern "C" fn cache_write(mut buffer: *mut uint8_t) {
    if cacheCount as libc::c_ulong ==
           ::core::mem::size_of::<[uint8_t; 8192]>() as libc::c_ulong {
        // Prevents overflow
        return
    }
    memcpy(&mut *writeCache.as_mut_ptr().offset(cacheCount as isize) as
               *mut uint8_t as *mut libc::c_void,
           buffer as *const libc::c_void,
           512 as libc::c_int as libc::c_ulong);
    cacheCount =
        (cacheCount as
             libc::c_uint).wrapping_add(512 as libc::c_int as libc::c_uint) as
            uint32_t as uint32_t;
}
#[no_mangle]
pub unsafe extern "C" fn cache_getCount() -> uint16_t {
    return cacheCount.wrapping_div(512 as libc::c_int as libc::c_uint) as
               uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn cache_reset() {
    cacheCount = 0 as libc::c_int as uint32_t;
}
static mut sdcard: sdcard_t =
    sdcard_t{pendingOperation:
                 C2RustUnnamed{buffer: 0 as *const uint8_t as *mut uint8_t,
                               blockIndex: 0,
                               chunkIndex: 0,
                               callback: None,
                               callbackData: 0,},
             operationStartTime: 0,
             failureCount: 0,
             version: 0,
             highCapacity: false,
             multiWriteNextBlock: 0,
             multiWriteBlocksRemain: 0,
             state: SDCARD_STATE_NOT_PRESENT,
             metadata:
                 sdcardMetadata_t{numBlocks: 0,
                                  oemID: 0,
                                  manufacturerID: 0,
                                  productName: [0; 5],
                                  productSerial: 0,
                                  productRevisionMajor: 0,
                                  productRevisionMinor: 0,
                                  productionYear: 0,
                                  productionMonth: 0,},
             csd: sdcardCSD_t{data: [0; 16],},
             enabled: false,
             cardDetectPin: 0 as *const libc::c_void as *mut libc::c_void,
             dma: DMA_NONE,
             dmaChannel: 0,
             useCache: 0,};
#[no_mangle]
pub unsafe extern "C" fn sdcardInsertionDetectDeinit() {
    if !sdcard.cardDetectPin.is_null() {
        IOInit(sdcard.cardDetectPin, OWNER_FREE, 0 as libc::c_int as uint8_t);
        IOConfigGPIO(sdcard.cardDetectPin,
                     (0 as libc::c_uint |
                          (0 as libc::c_uint) << 2 as libc::c_int |
                          (0 as libc::c_uint) << 5 as libc::c_int) as
                         ioConfig_t);
    };
}
#[no_mangle]
pub unsafe extern "C" fn sdcardInsertionDetectInit() {
    if !sdcard.cardDetectPin.is_null() {
        IOInit(sdcard.cardDetectPin, OWNER_SDCARD_DETECT,
               0 as libc::c_int as uint8_t);
        IOConfigGPIO(sdcard.cardDetectPin,
                     (0 as libc::c_uint |
                          (0 as libc::c_uint) << 2 as libc::c_int |
                          (0x1 as libc::c_uint) << 5 as libc::c_int) as
                         ioConfig_t);
    };
}
/* *
 * Detect if a SD card is physically present in the memory slot.
 */
#[no_mangle]
pub unsafe extern "C" fn sdcard_isInserted() -> bool {
    let mut ret: bool = 1 as libc::c_int != 0;
    if !sdcard.cardDetectPin.is_null() {
        ret = IORead(sdcard.cardDetectPin) as libc::c_int == 0 as libc::c_int
    }
    return ret;
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
/* Card capacity in 512-byte blocks*/
/* *
 * Returns true if the card has already been, or is currently, initializing and hasn't encountered enough errors to
 * trip our error threshold and be disabled (i.e. our card is in and working!)
 */
#[no_mangle]
pub unsafe extern "C" fn sdcard_isFunctional() -> bool {
    return sdcard.state as libc::c_uint !=
               SDCARD_STATE_NOT_PRESENT as libc::c_int as libc::c_uint;
}
/* *
 * Handle a failure of an SD card operation by resetting the card back to its initialization phase.
 *
 * Increments the failure counter, and when the failure threshold is reached, disables the card until
 * the next call to sdcard_init().
 */
unsafe extern "C" fn sdcard_reset() {
    if SD_Init() as libc::c_int != 0 as libc::c_int {
        sdcard.failureCount = sdcard.failureCount.wrapping_add(1);
        if sdcard.failureCount as libc::c_int >= 8 as libc::c_int ||
               sdcard_isInserted() as libc::c_int ==
                   0 as libc::c_int as uint8_t as libc::c_int {
            sdcard.state = SDCARD_STATE_NOT_PRESENT
        } else {
            sdcard.operationStartTime = millis();
            sdcard.state = SDCARD_STATE_RESET
        }
    };
}
/* *
 * Attempt to receive a data block from the SD card.
 *
 * Return true on success, otherwise the card has not responded yet and you should retry later.
 */
unsafe extern "C" fn sdcard_receiveDataBlock(mut buffer: *mut uint8_t,
                                             mut count: libc::c_int)
 -> sdcardReceiveBlockStatus_e {
    let mut ret: SD_Error_t = SD_CheckRead();
    if ret as libc::c_uint == SD_BUSY as libc::c_int as libc::c_uint {
        return SDCARD_RECEIVE_BLOCK_IN_PROGRESS
    }
    if SD_GetState() as libc::c_int != 1 as libc::c_int {
        return SDCARD_RECEIVE_ERROR
    }
    return SDCARD_RECEIVE_SUCCESS;
}
unsafe extern "C" fn sdcard_receiveCID() -> bool {
    let mut sdinfo: *mut SD_CardInfo_t = &mut SD_CardInfo;
    let mut error: SD_Error_t = SD_GetCardInfo();
    if error as u64 != 0 { return 0 as libc::c_int != 0 }
    sdcard.metadata.manufacturerID = (*sdinfo).SD_cid.ManufacturerID;
    sdcard.metadata.oemID = (*sdinfo).SD_cid.OEM_AppliID;
    sdcard.metadata.productName[0 as libc::c_int as usize] =
        (((*sdinfo).SD_cid.ProdName1 & 0xff000000 as libc::c_uint) >>
             24 as libc::c_int) as libc::c_char;
    sdcard.metadata.productName[1 as libc::c_int as usize] =
        (((*sdinfo).SD_cid.ProdName1 &
              0xff0000 as libc::c_int as libc::c_uint) >> 16 as libc::c_int)
            as libc::c_char;
    sdcard.metadata.productName[2 as libc::c_int as usize] =
        (((*sdinfo).SD_cid.ProdName1 & 0xff00 as libc::c_int as libc::c_uint)
             >> 8 as libc::c_int) as libc::c_char;
    sdcard.metadata.productName[3 as libc::c_int as usize] =
        (((*sdinfo).SD_cid.ProdName1 & 0xff as libc::c_int as libc::c_uint) >>
             0 as libc::c_int) as libc::c_char;
    sdcard.metadata.productName[4 as libc::c_int as usize] =
        (*sdinfo).SD_cid.ProdName2 as libc::c_char;
    sdcard.metadata.productRevisionMajor =
        ((*sdinfo).SD_cid.ProdRev as libc::c_int >> 4 as libc::c_int) as
            uint8_t;
    sdcard.metadata.productRevisionMinor =
        ((*sdinfo).SD_cid.ProdRev as libc::c_int & 0xf as libc::c_int) as
            uint8_t;
    sdcard.metadata.productSerial = (*sdinfo).SD_cid.ProdSN;
    sdcard.metadata.productionYear =
        ((((*sdinfo).SD_cid.ManufactDate as libc::c_int &
               0xf00 as libc::c_int) >> 8 as libc::c_int |
              ((*sdinfo).SD_cid.ManufactDate as libc::c_int &
                   0xff as libc::c_int) >> 4 as libc::c_int) +
             2000 as libc::c_int) as uint16_t;
    sdcard.metadata.productionMonth =
        ((*sdinfo).SD_cid.ManufactDate as libc::c_int & 0xf as libc::c_int) as
            uint8_t;
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn sdcard_fetchCSD() -> bool {
    /* The CSD command's data block should always arrive within 8 idle clock cycles (SD card spec). This is because
     * the information about card latency is stored in the CSD register itself, so we can't use that yet!
     */
    let mut sdinfo: *mut SD_CardInfo_t = &mut SD_CardInfo;
    let mut error: SD_Error_t = SD_OK;
    error = SD_GetCardInfo();
    if error as u64 != 0 { return 0 as libc::c_int != 0 }
    sdcard.metadata.numBlocks = (*sdinfo).CardCapacity as uint32_t;
    return 1 as libc::c_int != 0;
}
/* *
 * Check if the SD Card has completed its startup sequence. Must be called with sdcard.state == SDCARD_STATE_INITIALIZATION.
 *
 * Returns true if the card has finished its init process.
 */
unsafe extern "C" fn sdcard_checkInitDone() -> bool {
    if SD_GetState() {
        let mut sdtype: *mut SD_CardType_t = &mut SD_CardType;
        SD_GetCardInfo();
        sdcard.version =
            if *sdtype as libc::c_uint != 0 {
                2 as libc::c_int
            } else { 1 as libc::c_int } as uint8_t;
        sdcard.highCapacity =
            if *sdtype as libc::c_uint == 2 as libc::c_int as libc::c_uint {
                1 as libc::c_int
            } else { 0 as libc::c_int } != 0;
        return 1 as libc::c_int != 0
    }
    // When card init is complete, the idle bit in the response becomes zero.
    return 0 as libc::c_int != 0;
}
/* *
 * Begin the initialization process for the SD card. This must be called first before any other sdcard_ routine.
 */
#[no_mangle]
pub unsafe extern "C" fn sdcard_init(mut config: *const sdcardConfig_t) {
    sdcard.enabled = (*config).enabled != 0;
    if !sdcard.enabled { sdcard.state = SDCARD_STATE_NOT_PRESENT; return }
    sdcard.dma = (*config).dmaIdentifier as dmaIdentifier_e;
    if sdcard.dma as libc::c_uint == 0 as libc::c_int as libc::c_uint {
        sdcard.state = SDCARD_STATE_NOT_PRESENT;
        return
    }
    if (*config).cardDetectTag != 0 {
        sdcard.cardDetectPin = IOGetByTag((*config).cardDetectTag)
    } else { sdcard.cardDetectPin = 0 as IO_t }
    if (*sdioConfig()).useCache != 0 {
        sdcard.useCache = 1 as libc::c_int as uint8_t
    } else { sdcard.useCache = 0 as libc::c_int as uint8_t }
    SD_Initialize_LL(dmaGetRefByIdentifier(sdcard.dma));
    if SD_IsDetected() {
        if SD_Init() as libc::c_int != 0 as libc::c_int {
            sdcard.state = SDCARD_STATE_NOT_PRESENT;
            sdcard.failureCount = sdcard.failureCount.wrapping_add(1);
            return
        }
    } else {
        sdcard.state = SDCARD_STATE_NOT_PRESENT;
        sdcard.failureCount = sdcard.failureCount.wrapping_add(1);
        return
    }
    sdcard.operationStartTime = millis();
    sdcard.state = SDCARD_STATE_RESET;
    sdcard.failureCount = 0 as libc::c_int as uint8_t;
}
/*
 * Returns true if the card is ready to accept read/write commands.
 */
unsafe extern "C" fn sdcard_isReady() -> bool {
    return sdcard.state as libc::c_uint ==
               SDCARD_STATE_READY as libc::c_int as libc::c_uint ||
               sdcard.state as libc::c_uint ==
                   SDCARD_STATE_WRITING_MULTIPLE_BLOCKS as libc::c_int as
                       libc::c_uint;
}
/* *
 * Send the stop-transmission token to complete a multi-block write.
 *
 * Returns:
 *     SDCARD_OPERATION_IN_PROGRESS - We're now waiting for that stop to complete, the card will enter
 *                                    the SDCARD_STATE_STOPPING_MULTIPLE_BLOCK_WRITE state.
 *     SDCARD_OPERATION_SUCCESS     - The multi-block write finished immediately, the card will enter
 *                                    the SDCARD_READY state.
 *
 */
unsafe extern "C" fn sdcard_endWriteBlocks() -> sdcardOperationStatus_e {
    sdcard.multiWriteBlocksRemain = 0 as libc::c_int as uint32_t;
    if sdcard.useCache != 0 { cache_reset(); }
    // 8 dummy clocks to guarantee N_WR clocks between the last card response and this token
    // Card may choose to raise a busy (non-0xFF) signal after at most N_BR (1 byte) delay
    if SD_GetState() {
        sdcard.state = SDCARD_STATE_READY;
        return SDCARD_OPERATION_SUCCESS
    } else {
        sdcard.state = SDCARD_STATE_STOPPING_MULTIPLE_BLOCK_WRITE;
        sdcard.operationStartTime = millis();
        return SDCARD_OPERATION_IN_PROGRESS
    };
}
/* *
 * Call periodically for the SD card to perform in-progress transfers.
 *
 * Returns true if the card is ready to accept commands.
 */
#[no_mangle]
pub unsafe extern "C" fn sdcard_poll() -> bool {
    if !sdcard.enabled {
        sdcard.state = SDCARD_STATE_NOT_PRESENT;
        return 0 as libc::c_int != 0
    }
    loop 
         // Timeout has expired, so fall through to convert to a fatal error
         {
        match sdcard.state as libc::c_uint {
            1 => {
                //HAL Takes care of voltage crap.
                sdcard.state = SDCARD_STATE_CARD_INIT_IN_PROGRESS
            }
            2 => {
                if !sdcard_checkInitDone() { break ; }
                // Now fetch the CSD and CID registers
                if sdcard_fetchCSD() {
                    sdcard.state = SDCARD_STATE_INITIALIZATION_RECEIVE_CID
                } else {
                    sdcard_reset(); // else keep waiting for the CID to arrive
                }
            }
            3 => {
                if !sdcard_receiveCID() { break ; }
                /* The spec is a little iffy on what the default block size is for Standard Size cards (it can be changed on
                 * standard size cards) so let's just set it to 512 explicitly so we don't have a problem.
                 */
//                if (!sdcard.highCapacity && SDMMC_CmdBlockLength(_HSD.Instance, SDCARD_BLOCK_SIZE)) {
//                    sdcard_reset();
//                    goto doMore;
//                }
                sdcard.multiWriteBlocksRemain = 0 as libc::c_int as uint32_t;
                sdcard.state = SDCARD_STATE_READY
            }
            6 => {
                // Have we finished sending the write yet?
                if SD_CheckWrite() as libc::c_uint ==
                       SD_OK as libc::c_int as libc::c_uint {
                    // The SD card is now busy committing that write to the card
                    sdcard.state = SDCARD_STATE_WAITING_FOR_WRITE;
                    sdcard.operationStartTime = millis();
                    // Since we've transmitted the buffer we can go ahead and tell the caller their operation is complete
                    if sdcard.pendingOperation.callback.is_some() {
                        sdcard.pendingOperation.callback.expect("non-null function pointer")(SDCARD_BLOCK_OPERATION_WRITE,
                                                                                             sdcard.pendingOperation.blockIndex,
                                                                                             sdcard.pendingOperation.buffer,
                                                                                             sdcard.pendingOperation.callbackData); // Assume the card is good if it can complete a write
                    }
                }
                break ;
            }
            7 => {
                if SD_GetState() {
                    sdcard.failureCount = 0 as libc::c_int as uint8_t;
                    // Still more blocks left to write in a multi-block chain?
                    if sdcard.multiWriteBlocksRemain >
                           1 as libc::c_int as libc::c_uint {
                        sdcard.multiWriteBlocksRemain =
                            sdcard.multiWriteBlocksRemain.wrapping_sub(1);
                        sdcard.multiWriteNextBlock =
                            sdcard.multiWriteNextBlock.wrapping_add(1);
                        if sdcard.useCache != 0 { cache_reset(); }
                        sdcard.state = SDCARD_STATE_WRITING_MULTIPLE_BLOCKS
                    } else if sdcard.multiWriteBlocksRemain ==
                                  1 as libc::c_int as libc::c_uint {
                        // This function changes the sd card state for us whether immediately succesful or delayed:
                        sdcard_endWriteBlocks();
                    } else { sdcard.state = SDCARD_STATE_READY }
                    break ;
                } else {
                    if !(millis() >
                             sdcard.operationStartTime.wrapping_add(250 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint))
                       {
                        break ;
                    }
                    /*
                 * The caller has already been told that their write has completed, so they will have discarded
                 * their buffer and have no hope of retrying the operation. But this should be very rare and it allows
                 * them to reuse their buffer milliseconds faster than they otherwise would.
                 */
                    sdcard_reset(); // Assume the card is good if it can complete a read
                }
            }
            5 => {
                match sdcard_receiveDataBlock(sdcard.pendingOperation.buffer,
                                              512 as libc::c_int) as
                          libc::c_uint {
                    0 => {
                        sdcard.state = SDCARD_STATE_READY;
                        sdcard.failureCount = 0 as libc::c_int as uint8_t;
                        if sdcard.pendingOperation.callback.is_some() {
                            sdcard.pendingOperation.callback.expect("non-null function pointer")(SDCARD_BLOCK_OPERATION_READ,
                                                                                                 sdcard.pendingOperation.blockIndex,
                                                                                                 sdcard.pendingOperation.buffer,
                                                                                                 sdcard.pendingOperation.callbackData);
                        }
                        break ;
                    }
                    1 => {
                        if millis() <=
                               sdcard.operationStartTime.wrapping_add(100 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
                           {
                            break ;
                        }
                    }
                    2 => { }
                    _ => { break ; }
                }
            }
            9 => {
                if SD_GetState() {
                    sdcard.state = SDCARD_STATE_READY;
                    break ;
                } else {
                    if !(millis() >
                             sdcard.operationStartTime.wrapping_add(250 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint))
                       {
                        break ;
                    }
                    sdcard_reset();
                }
            }
            0 | _ => { break ; }
        }
    }
    // Timeout not reached yet so keep waiting
    if sdcard.state as libc::c_uint >=
           SDCARD_STATE_RESET as libc::c_int as libc::c_uint &&
           (sdcard.state as libc::c_uint) <
               SDCARD_STATE_READY as libc::c_int as libc::c_uint &&
           millis().wrapping_sub(sdcard.operationStartTime) >
               200 as libc::c_int as libc::c_uint {
        sdcard_reset();
    }
    return sdcard_isReady();
}
// Is the card's initialization taking too long?
/* *
 * Write the 512-byte block from the given buffer into the block with the given index.
 *
 * If the write does not complete immediately, your callback will be called later. If the write was successful, the
 * buffer pointer will be the same buffer you originally passed in, otherwise the buffer will be set to NULL.
 *
 * Returns:
 *     SDCARD_OPERATION_IN_PROGRESS - Your buffer is currently being transmitted to the card and your callback will be
 *                                    called later to report the completion. The buffer pointer must remain valid until
 *                                    that time.
 *     SDCARD_OPERATION_SUCCESS     - Your buffer has been transmitted to the card now.
 *     SDCARD_OPERATION_BUSY        - The card is already busy and cannot accept your write
 *     SDCARD_OPERATION_FAILURE     - Your write was rejected by the card, card will be reset
 */
#[no_mangle]
pub unsafe extern "C" fn sdcard_writeBlock(mut blockIndex: uint32_t,
                                           mut buffer: *mut uint8_t,
                                           mut callback:
                                               sdcard_operationCompleteCallback_c,
                                           mut callbackData: uint32_t)
 -> sdcardOperationStatus_e {
    loop 
         // Now we've entered the ready state, we can try again
         {
        match sdcard.state as libc::c_uint {
            8 => {
                // Do we need to cancel the previous multi-block write?
                if !(blockIndex != sdcard.multiWriteNextBlock) { break ; }
                if sdcard_endWriteBlocks() as libc::c_uint ==
                       SDCARD_OPERATION_SUCCESS as libc::c_int as libc::c_uint
                   {
                    continue ;
                }
                return SDCARD_OPERATION_BUSY
            }
            4 => { break ; }
            _ => { return SDCARD_OPERATION_BUSY }
        }
    }
    sdcard.pendingOperation.buffer = buffer;
    sdcard.pendingOperation.blockIndex = blockIndex;
    let mut block_count: uint16_t = 1 as libc::c_int as uint16_t;
    if (cache_getCount() as libc::c_int) < 16 as libc::c_int &&
           sdcard.multiWriteBlocksRemain != 0 as libc::c_int as libc::c_uint
           && sdcard.useCache as libc::c_int != 0 {
        cache_write(buffer);
        if cache_getCount() as libc::c_int == 16 as libc::c_int ||
               sdcard.multiWriteBlocksRemain ==
                   1 as libc::c_int as libc::c_uint {
            //Relocate buffer
            buffer = writeCache.as_mut_ptr();
            //Recalculate block index
            blockIndex =
                (blockIndex as
                     libc::c_uint).wrapping_sub((cache_getCount() as
                                                     libc::c_int -
                                                     1 as libc::c_int) as
                                                    libc::c_uint) as uint32_t
                    as
                    uint32_t; // (for non-DMA transfers) we've sent chunk #0 already
            block_count = cache_getCount()
        } else {
            sdcard.multiWriteBlocksRemain =
                sdcard.multiWriteBlocksRemain.wrapping_sub(1);
            sdcard.multiWriteNextBlock =
                sdcard.multiWriteNextBlock.wrapping_add(1);
            sdcard.state = SDCARD_STATE_READY;
            return SDCARD_OPERATION_SUCCESS
        }
    }
    sdcard.pendingOperation.callback = callback;
    sdcard.pendingOperation.callbackData = callbackData;
    sdcard.pendingOperation.chunkIndex = 1 as libc::c_int as uint8_t;
    sdcard.state = SDCARD_STATE_SENDING_WRITE;
    if SD_WriteBlocks_DMA(blockIndex as uint64_t, buffer as *mut uint32_t,
                          512 as libc::c_int as uint32_t,
                          block_count as uint32_t) as libc::c_uint !=
           SD_OK as libc::c_int as libc::c_uint {
        /* Our write was rejected! This could be due to a bad address but we hope not to attempt that, so assume
         * the card is broken and needs reset.
         */
        sdcard_reset();
        // Announce write failure:
        if sdcard.pendingOperation.callback.is_some() {
            sdcard.pendingOperation.callback.expect("non-null function pointer")(SDCARD_BLOCK_OPERATION_WRITE,
                                                                                 sdcard.pendingOperation.blockIndex,
                                                                                 0
                                                                                     as
                                                                                     *mut uint8_t,
                                                                                 sdcard.pendingOperation.callbackData);
        }
        return SDCARD_OPERATION_FAILURE
    }
    return SDCARD_OPERATION_IN_PROGRESS;
}
/* *
 * Begin writing a series of consecutive blocks beginning at the given block index. This will allow (but not require)
 * the SD card to pre-erase the number of blocks you specifiy, which can allow the writes to complete faster.
 *
 * Afterwards, just call sdcard_writeBlock() as normal to write those blocks consecutively.
 *
 * It's okay to abort the multi-block write at any time by writing to a non-consecutive address, or by performing a read.
 *
 * Returns:
 *     SDCARD_OPERATION_SUCCESS     - Multi-block write has been queued
 *     SDCARD_OPERATION_BUSY        - The card is already busy and cannot accept your write
 *     SDCARD_OPERATION_FAILURE     - A fatal error occured, card will be reset
 */
#[no_mangle]
pub unsafe extern "C" fn sdcard_beginWriteBlocks(mut blockIndex: uint32_t,
                                                 mut blockCount: uint32_t)
 -> sdcardOperationStatus_e {
    if sdcard.state as libc::c_uint !=
           SDCARD_STATE_READY as libc::c_int as libc::c_uint {
        if sdcard.state as libc::c_uint ==
               SDCARD_STATE_WRITING_MULTIPLE_BLOCKS as libc::c_int as
                   libc::c_uint {
            if blockIndex == sdcard.multiWriteNextBlock {
                // Assume that the caller wants to continue the multi-block write they already have in progress!
                return SDCARD_OPERATION_SUCCESS
            } else {
                if sdcard_endWriteBlocks() as libc::c_uint !=
                       SDCARD_OPERATION_SUCCESS as libc::c_int as libc::c_uint
                   {
                    return SDCARD_OPERATION_BUSY
                }
            }
            // Else we've completed the previous multi-block write and can fall through to start the new one
        } else { return SDCARD_OPERATION_BUSY }
    }
    sdcard.state = SDCARD_STATE_WRITING_MULTIPLE_BLOCKS;
    sdcard.multiWriteBlocksRemain = blockCount;
    sdcard.multiWriteNextBlock = blockIndex;
    return SDCARD_OPERATION_SUCCESS;
}
/* *
 * Read the 512-byte block with the given index into the given 512-byte buffer.
 *
 * When the read completes, your callback will be called. If the read was successful, the buffer pointer will be the
 * same buffer you originally passed in, otherwise the buffer will be set to NULL.
 *
 * You must keep the pointer to the buffer valid until the operation completes!
 *
 * Returns:
 *     true - The operation was successfully queued for later completion, your callback will be called later
 *     false - The operation could not be started due to the card being busy (try again later).
 */
#[no_mangle]
pub unsafe extern "C" fn sdcard_readBlock(mut blockIndex: uint32_t,
                                          mut buffer: *mut uint8_t,
                                          mut callback:
                                              sdcard_operationCompleteCallback_c,
                                          mut callbackData: uint32_t)
 -> bool {
    if sdcard.state as libc::c_uint !=
           SDCARD_STATE_READY as libc::c_int as libc::c_uint {
        if sdcard.state as libc::c_uint ==
               SDCARD_STATE_WRITING_MULTIPLE_BLOCKS as libc::c_int as
                   libc::c_uint {
            if sdcard_endWriteBlocks() as libc::c_uint !=
                   SDCARD_OPERATION_SUCCESS as libc::c_int as libc::c_uint {
                return 0 as libc::c_int != 0
            }
        } else { return 0 as libc::c_int != 0 }
    }
    // Standard size cards use byte addressing, high capacity cards use block addressing
    let mut status: uint8_t =
        SD_ReadBlocks_DMA(blockIndex as uint64_t, buffer as *mut uint32_t,
                          512 as libc::c_int as uint32_t,
                          1 as libc::c_int as uint32_t) as uint8_t;
    if status as libc::c_int == SD_OK as libc::c_int {
        sdcard.pendingOperation.buffer = buffer;
        sdcard.pendingOperation.blockIndex = blockIndex;
        sdcard.pendingOperation.callback = callback;
        sdcard.pendingOperation.callbackData = callbackData;
        sdcard.state = SDCARD_STATE_READING;
        sdcard.operationStartTime = millis();
        return 1 as libc::c_int != 0
    } else {
        sdcard_reset();
        if sdcard.pendingOperation.callback.is_some() {
            sdcard.pendingOperation.callback.expect("non-null function pointer")(SDCARD_BLOCK_OPERATION_READ,
                                                                                 sdcard.pendingOperation.blockIndex,
                                                                                 0
                                                                                     as
                                                                                     *mut uint8_t,
                                                                                 sdcard.pendingOperation.callbackData);
        }
        return 0 as libc::c_int != 0
    };
}
/* *
 * Returns true if the SD card has successfully completed its startup procedures.
 */
#[no_mangle]
pub unsafe extern "C" fn sdcard_isInitialized() -> bool {
    return sdcard.state as libc::c_uint >=
               SDCARD_STATE_READY as libc::c_int as libc::c_uint;
}
#[no_mangle]
pub unsafe extern "C" fn sdcard_getMetadata() -> *const sdcardMetadata_t {
    return &mut sdcard.metadata;
}
