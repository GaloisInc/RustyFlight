use ::libc;
extern "C" {
    /* Prototype(s) -----------------------------------------------------------------------------------------------------*/
    #[no_mangle]
    static mut SD_CardInfo: SD_CardInfo_t;
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
    fn ledSet(led: libc::c_int, state: bool);
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type __uint64_t = libc::c_ulong;
pub type int8_t = __int8_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
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
pub type SD_Error_t = libc::c_uint;
pub const SD_OK: SD_Error_t = 0;
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
pub struct _USBD_STORAGE {
    pub Init: Option<unsafe extern "C" fn(_: uint8_t) -> int8_t>,
    pub GetCapacity: Option<unsafe extern "C" fn(_: uint8_t, _: *mut uint32_t,
                                                 _: *mut uint16_t) -> int8_t>,
    pub IsReady: Option<unsafe extern "C" fn(_: uint8_t) -> int8_t>,
    pub IsWriteProtected: Option<unsafe extern "C" fn(_: uint8_t) -> int8_t>,
    pub Read: Option<unsafe extern "C" fn(_: uint8_t, _: *mut uint8_t,
                                          _: uint32_t, _: uint16_t)
                         -> int8_t>,
    pub Write: Option<unsafe extern "C" fn(_: uint8_t, _: *mut uint8_t,
                                           _: uint32_t, _: uint16_t)
                          -> int8_t>,
    pub GetMaxLun: Option<unsafe extern "C" fn() -> int8_t>,
    pub pInquiry: *mut int8_t,
}
pub type USBD_StorageTypeDef = _USBD_STORAGE;
/* USB Mass storage Standard Inquiry Data */
static mut STORAGE_Inquirydata: [uint8_t; 36] =
    [0 as libc::c_int as uint8_t, 0x80 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     (0x24 as libc::c_int - 5 as libc::c_int) as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 'S' as i32 as uint8_t,
     'T' as i32 as uint8_t, 'M' as i32 as uint8_t, ' ' as i32 as uint8_t,
     ' ' as i32 as uint8_t, ' ' as i32 as uint8_t, ' ' as i32 as uint8_t,
     ' ' as i32 as uint8_t, 'P' as i32 as uint8_t, 'r' as i32 as uint8_t,
     'o' as i32 as uint8_t, 'd' as i32 as uint8_t, 'u' as i32 as uint8_t,
     't' as i32 as uint8_t, ' ' as i32 as uint8_t, ' ' as i32 as uint8_t,
     ' ' as i32 as uint8_t, ' ' as i32 as uint8_t, ' ' as i32 as uint8_t,
     ' ' as i32 as uint8_t, ' ' as i32 as uint8_t, ' ' as i32 as uint8_t,
     ' ' as i32 as uint8_t, ' ' as i32 as uint8_t, '0' as i32 as uint8_t,
     '.' as i32 as uint8_t, '0' as i32 as uint8_t, '1' as i32 as uint8_t];
#[no_mangle]
pub static mut USBD_MSC_MICRO_SDIO_fops: USBD_StorageTypeDef =
    unsafe {
        {
            let mut init =
                _USBD_STORAGE{Init:
                                  Some(STORAGE_Init as
                                           unsafe extern "C" fn(_: uint8_t)
                                               -> int8_t),
                              GetCapacity:
                                  Some(STORAGE_GetCapacity as
                                           unsafe extern "C" fn(_: uint8_t,
                                                                _:
                                                                    *mut uint32_t,
                                                                _:
                                                                    *mut uint16_t)
                                               -> int8_t),
                              IsReady:
                                  Some(STORAGE_IsReady as
                                           unsafe extern "C" fn(_: uint8_t)
                                               -> int8_t),
                              IsWriteProtected:
                                  Some(STORAGE_IsWriteProtected as
                                           unsafe extern "C" fn(_: uint8_t)
                                               -> int8_t),
                              Read:
                                  Some(STORAGE_Read as
                                           unsafe extern "C" fn(_: uint8_t,
                                                                _:
                                                                    *mut uint8_t,
                                                                _: uint32_t,
                                                                _: uint16_t)
                                               -> int8_t),
                              Write:
                                  Some(STORAGE_Write as
                                           unsafe extern "C" fn(_: uint8_t,
                                                                _:
                                                                    *mut uint8_t,
                                                                _: uint32_t,
                                                                _: uint16_t)
                                               -> int8_t),
                              GetMaxLun:
                                  Some(STORAGE_GetMaxLun as
                                           unsafe extern "C" fn() -> int8_t),
                              pInquiry:
                                  STORAGE_Inquirydata.as_ptr() as *mut _ as
                                      *mut int8_t,};
            init
        }
    };
/* ******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the microSD card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
unsafe extern "C" fn STORAGE_Init(mut lun: uint8_t) -> int8_t {
    //Initialize SD_DET
    ledSet(0 as libc::c_int, 0 as libc::c_int != 0);
    SD_Initialize_LL((0x40000000 as
                          libc::c_uint).wrapping_add(0x20000 as
                                                         libc::c_uint).wrapping_add(0x6400
                                                                                        as
                                                                                        libc::c_uint).wrapping_add(0x58
                                                                                                                       as
                                                                                                                       libc::c_uint)
                         as *mut DMA_Stream_TypeDef);
    if SD_Init() as libc::c_int != 0 as libc::c_int {
        return 1 as libc::c_int as int8_t
    }
    ledSet(0 as libc::c_int, 1 as libc::c_int != 0);
    return 0 as libc::c_int as int8_t;
}
/* ******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
unsafe extern "C" fn STORAGE_GetCapacity(mut lun: uint8_t,
                                         mut block_num: *mut uint32_t,
                                         mut block_size: *mut uint16_t)
 -> int8_t {
    if SD_IsDetected() as libc::c_int == 0 as libc::c_int {
        return -(1 as libc::c_int) as int8_t
    }
    SD_GetCardInfo();
    *block_num = SD_CardInfo.CardCapacity as uint32_t;
    *block_size = 512 as libc::c_int as uint16_t;
    return 0 as libc::c_int as int8_t;
}
/* ******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
unsafe extern "C" fn STORAGE_IsReady(mut lun: uint8_t) -> int8_t {
    let mut ret: int8_t = -(1 as libc::c_int) as int8_t;
    if SD_GetState() as libc::c_int == 1 as libc::c_int &&
           SD_IsDetected() as libc::c_int ==
               0x1 as libc::c_int as uint8_t as libc::c_int {
        ret = 0 as libc::c_int as int8_t
    }
    return ret;
}
/* ******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
unsafe extern "C" fn STORAGE_IsWriteProtected(mut lun: uint8_t) -> int8_t {
    return 0 as libc::c_int as int8_t;
}
/* ******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
unsafe extern "C" fn STORAGE_Read(mut lun: uint8_t, mut buf: *mut uint8_t,
                                  mut blk_addr: uint32_t,
                                  mut blk_len: uint16_t) -> int8_t {
    if SD_IsDetected() as libc::c_int == 0 as libc::c_int {
        return -(1 as libc::c_int) as int8_t
    }
    ledSet(1 as libc::c_int, 1 as libc::c_int != 0);
    //buf should be 32bit aligned, but usually is so we don't do byte alignment
    if SD_ReadBlocks_DMA(blk_addr as uint64_t, buf as *mut uint32_t,
                         512 as libc::c_int as uint32_t, blk_len as uint32_t)
           as libc::c_uint == 0 as libc::c_int as libc::c_uint {
        while SD_CheckRead() as u64 != 0 { }
        while SD_GetState() as libc::c_int == 0 as libc::c_int { }
        ledSet(1 as libc::c_int, 0 as libc::c_int != 0);
        return 0 as libc::c_int as int8_t
    }
    ledSet(1 as libc::c_int, 0 as libc::c_int != 0);
    return -(1 as libc::c_int) as int8_t;
}
/* ******************************************************************************
* Function Name  : Write_Memory
* Description    : Handle the Write operation to the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
unsafe extern "C" fn STORAGE_Write(mut lun: uint8_t, mut buf: *mut uint8_t,
                                   mut blk_addr: uint32_t,
                                   mut blk_len: uint16_t) -> int8_t {
    if SD_IsDetected() as libc::c_int == 0 as libc::c_int {
        return -(1 as libc::c_int) as int8_t
    }
    ledSet(1 as libc::c_int, 1 as libc::c_int != 0);
    //buf should be 32bit aligned, but usually is so we don't do byte alignment
    if SD_WriteBlocks_DMA(blk_addr as uint64_t, buf as *mut uint32_t,
                          512 as libc::c_int as uint32_t, blk_len as uint32_t)
           as libc::c_uint == 0 as libc::c_int as libc::c_uint {
        while SD_CheckWrite() as u64 != 0 { }
        while SD_GetState() as libc::c_int == 0 as libc::c_int { }
        ledSet(1 as libc::c_int, 0 as libc::c_int != 0);
        return 0 as libc::c_int as int8_t
    }
    ledSet(1 as libc::c_int, 0 as libc::c_int != 0);
    return -(1 as libc::c_int) as int8_t;
}
/* ******************************************************************************
* Function Name  : Write_Memory
* Description    : Handle the Write operation to the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
unsafe extern "C" fn STORAGE_GetMaxLun() -> int8_t {
    return (1 as libc::c_int - 1 as libc::c_int) as int8_t;
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
