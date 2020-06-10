use ::libc;
extern "C" {
    #[no_mangle]
    fn USBD_GetString(desc: *mut uint8_t, unicode: *mut uint8_t,
                      len: *mut uint16_t);
    #[no_mangle]
    static mut usbDevConfig_System: usbDev_t;
    #[no_mangle]
    fn mscCheckBoot() -> bool;
    /* USB_DeviceDescriptor */
    #[no_mangle]
    static mut USBD_HID_CDC_DeviceDescriptor: [uint8_t; 18];
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USBD_DescriptorsTypeDef {
    pub GetDeviceDescriptor: Option<unsafe extern "C" fn(_: USBD_SpeedTypeDef,
                                                         _: *mut uint16_t)
                                        -> *mut uint8_t>,
    pub GetLangIDStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                USBD_SpeedTypeDef,
                                                            _: *mut uint16_t)
                                           -> *mut uint8_t>,
    pub GetManufacturerStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                      USBD_SpeedTypeDef,
                                                                  _:
                                                                      *mut uint16_t)
                                                 -> *mut uint8_t>,
    pub GetProductStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                 USBD_SpeedTypeDef,
                                                             _: *mut uint16_t)
                                            -> *mut uint8_t>,
    pub GetSerialStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                USBD_SpeedTypeDef,
                                                            _: *mut uint16_t)
                                           -> *mut uint8_t>,
    pub GetConfigurationStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                       USBD_SpeedTypeDef,
                                                                   _:
                                                                       *mut uint16_t)
                                                  -> *mut uint8_t>,
    pub GetInterfaceStrDescriptor: Option<unsafe extern "C" fn(_:
                                                                   USBD_SpeedTypeDef,
                                                               _:
                                                                   *mut uint16_t)
                                              -> *mut uint8_t>,
}
pub type USBD_SpeedTypeDef = libc::c_uint;
pub const USBD_SPEED_LOW: USBD_SpeedTypeDef = 2;
pub const USBD_SPEED_FULL: USBD_SpeedTypeDef = 1;
pub const USBD_SPEED_HIGH: USBD_SpeedTypeDef = 0;
pub const COMPOSITE: USB_DEV = 1;
pub type usbDev_t = usbDev_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct usbDev_s {
    pub type_0: uint8_t,
    pub mscButtonPin: ioTag_t,
    pub mscButtonUsePullup: uint8_t,
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
pub type USB_DEV = libc::c_uint;
pub const DEFAULT: USB_DEV = 0;
#[inline]
unsafe extern "C" fn usbDevConfig() -> *const usbDev_t {
    return &mut usbDevConfig_System;
}
/* USB_SUPPORT_USER_STRING_DESC */
/* Private variables ---------------------------------------------------------*/
#[no_mangle]
pub static mut VCP_Desc: USBD_DescriptorsTypeDef =
    unsafe {
        {
            let mut init =
                USBD_DescriptorsTypeDef{GetDeviceDescriptor:
                                            Some(USBD_VCP_DeviceDescriptor as
                                                     unsafe extern "C" fn(_:
                                                                              USBD_SpeedTypeDef,
                                                                          _:
                                                                              *mut uint16_t)
                                                         -> *mut uint8_t),
                                        GetLangIDStrDescriptor:
                                            Some(USBD_VCP_LangIDStrDescriptor
                                                     as
                                                     unsafe extern "C" fn(_:
                                                                              USBD_SpeedTypeDef,
                                                                          _:
                                                                              *mut uint16_t)
                                                         -> *mut uint8_t),
                                        GetManufacturerStrDescriptor:
                                            Some(USBD_VCP_ManufacturerStrDescriptor
                                                     as
                                                     unsafe extern "C" fn(_:
                                                                              USBD_SpeedTypeDef,
                                                                          _:
                                                                              *mut uint16_t)
                                                         -> *mut uint8_t),
                                        GetProductStrDescriptor:
                                            Some(USBD_VCP_ProductStrDescriptor
                                                     as
                                                     unsafe extern "C" fn(_:
                                                                              USBD_SpeedTypeDef,
                                                                          _:
                                                                              *mut uint16_t)
                                                         -> *mut uint8_t),
                                        GetSerialStrDescriptor:
                                            Some(USBD_VCP_SerialStrDescriptor
                                                     as
                                                     unsafe extern "C" fn(_:
                                                                              USBD_SpeedTypeDef,
                                                                          _:
                                                                              *mut uint16_t)
                                                         -> *mut uint8_t),
                                        GetConfigurationStrDescriptor:
                                            Some(USBD_VCP_ConfigStrDescriptor
                                                     as
                                                     unsafe extern "C" fn(_:
                                                                              USBD_SpeedTypeDef,
                                                                          _:
                                                                              *mut uint16_t)
                                                         -> *mut uint8_t),
                                        GetInterfaceStrDescriptor:
                                            Some(USBD_VCP_InterfaceStrDescriptor
                                                     as
                                                     unsafe extern "C" fn(_:
                                                                              USBD_SpeedTypeDef,
                                                                          _:
                                                                              *mut uint16_t)
                                                         -> *mut uint8_t),};
            init
        }
    };
/* USB Standard Device Descriptor */
/* !< IAR Compiler */
#[no_mangle]
pub static mut USBD_DeviceDesc: [uint8_t; 18] =
    [0x12 as libc::c_int as uint8_t, 1 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x2 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 64 as libc::c_int as uint8_t,
     (0x483 as libc::c_int & 0xff as libc::c_int) as uint8_t,
     ((0x483 as libc::c_int & 0xff00 as libc::c_int) >> 8 as libc::c_int) as
         uint8_t, (0x5740 as libc::c_int & 0xff as libc::c_int) as uint8_t,
     ((0x5740 as libc::c_int & 0xff00 as libc::c_int) >> 8 as libc::c_int) as
         uint8_t, 0 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t, 1 as libc::c_int as uint8_t];
#[no_mangle]
pub static mut USBD_MSC_DeviceDesc: [uint8_t; 18] =
    [0x12 as libc::c_int as uint8_t, 1 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 64 as libc::c_int as uint8_t,
     (0x483 as libc::c_int & 0xff as libc::c_int) as uint8_t,
     ((0x483 as libc::c_int & 0xff00 as libc::c_int) >> 8 as libc::c_int) as
         uint8_t, (22314 as libc::c_int & 0xff as libc::c_int) as uint8_t,
     ((22314 as libc::c_int & 0xff00 as libc::c_int) >> 8 as libc::c_int) as
         uint8_t, 0 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x1 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
     0x3 as libc::c_int as uint8_t, 1 as libc::c_int as uint8_t];
/* USB Standard Device Descriptor */
/* !< IAR Compiler */
#[no_mangle]
pub static mut USBD_LangIDDesc: [uint8_t; 4] =
    [0x4 as libc::c_int as uint8_t, 3 as libc::c_int as uint8_t,
     (0x409 as libc::c_int & 0xff as libc::c_int) as uint8_t,
     ((0x409 as libc::c_int & 0xff00 as libc::c_int) >> 8 as libc::c_int) as
         uint8_t];
#[no_mangle]
pub static mut USBD_StringSerial: [uint8_t; 26] =
    [0x1a as libc::c_int as uint8_t, 3 as libc::c_int as uint8_t, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
/* !< IAR Compiler */
#[no_mangle]
pub static mut USBD_StrDesc: [uint8_t; 256] = [0; 256];
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* *
  * @brief  Returns the device descriptor.
  * @param  speed: Current device speed
  * @param  length: Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_VCP_DeviceDescriptor(mut speed:
                                                       USBD_SpeedTypeDef,
                                                   mut length: *mut uint16_t)
 -> *mut uint8_t {
    if mscCheckBoot() {
        *length =
            ::core::mem::size_of::<[uint8_t; 18]>() as libc::c_ulong as
                uint16_t;
        return USBD_MSC_DeviceDesc.as_mut_ptr()
    }
    if (*usbDevConfig()).type_0 as libc::c_int == COMPOSITE as libc::c_int {
        *length =
            ::core::mem::size_of::<[uint8_t; 18]>() as libc::c_ulong as
                uint16_t;
        return USBD_HID_CDC_DeviceDescriptor.as_mut_ptr()
    }
    *length =
        ::core::mem::size_of::<[uint8_t; 18]>() as libc::c_ulong as uint16_t;
    return USBD_DeviceDesc.as_mut_ptr();
}
/* *
  * @brief  Returns the LangID string descriptor.
  * @param  speed: Current device speed
  * @param  length: Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_VCP_LangIDStrDescriptor(mut speed:
                                                          USBD_SpeedTypeDef,
                                                      mut length:
                                                          *mut uint16_t)
 -> *mut uint8_t {
    *length =
        ::core::mem::size_of::<[uint8_t; 4]>() as libc::c_ulong as uint16_t;
    return USBD_LangIDDesc.as_mut_ptr();
}
/* *
  * @brief  Returns the product string descriptor.
  * @param  speed: Current device speed
  * @param  length: Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_VCP_ProductStrDescriptor(mut speed:
                                                           USBD_SpeedTypeDef,
                                                       mut length:
                                                           *mut uint16_t)
 -> *mut uint8_t {
    if speed as libc::c_uint == USBD_SPEED_HIGH as libc::c_int as libc::c_uint
       {
        USBD_GetString(b"STM32 Virtual ComPort in HS Mode\x00" as *const u8 as
                           *const libc::c_char as *mut uint8_t,
                       USBD_StrDesc.as_mut_ptr(), length);
    } else {
        USBD_GetString(b"STM32 Virtual ComPort in FS Mode\x00" as *const u8 as
                           *const libc::c_char as *mut uint8_t,
                       USBD_StrDesc.as_mut_ptr(), length);
    }
    return USBD_StrDesc.as_mut_ptr();
}
/* *
  * @brief  Returns the manufacturer string descriptor.
  * @param  speed: Current device speed
  * @param  length: Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_VCP_ManufacturerStrDescriptor(mut speed:
                                                                USBD_SpeedTypeDef,
                                                            mut length:
                                                                *mut uint16_t)
 -> *mut uint8_t {
    USBD_GetString(b"Cleanflight\x00" as *const u8 as *const libc::c_char as
                       *mut uint8_t, USBD_StrDesc.as_mut_ptr(), length);
    return USBD_StrDesc.as_mut_ptr();
}
/* *
  * @brief  Returns the serial number string descriptor.
  * @param  speed: Current device speed
  * @param  length: Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_VCP_SerialStrDescriptor(mut speed:
                                                          USBD_SpeedTypeDef,
                                                      mut length:
                                                          *mut uint16_t)
 -> *mut uint8_t {
    *length = 0x1a as libc::c_int as uint16_t;
    /* Update the serial number string descriptor with the data from the unique ID*/
    Get_SerialNum();
    return USBD_StringSerial.as_mut_ptr();
}
/* *
  * @brief  Returns the configuration string descriptor.
  * @param  speed: Current device speed
  * @param  length: Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_VCP_ConfigStrDescriptor(mut speed:
                                                          USBD_SpeedTypeDef,
                                                      mut length:
                                                          *mut uint16_t)
 -> *mut uint8_t {
    if speed as libc::c_uint == USBD_SPEED_HIGH as libc::c_int as libc::c_uint
       {
        USBD_GetString(b"VCP Config\x00" as *const u8 as *const libc::c_char
                           as *mut uint8_t, USBD_StrDesc.as_mut_ptr(),
                       length);
    } else {
        USBD_GetString(b"VCP Config\x00" as *const u8 as *const libc::c_char
                           as *mut uint8_t, USBD_StrDesc.as_mut_ptr(),
                       length);
    }
    return USBD_StrDesc.as_mut_ptr();
}
/* *
  * @brief  Returns the interface string descriptor.
  * @param  speed: Current device speed
  * @param  length: Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
#[no_mangle]
pub unsafe extern "C" fn USBD_VCP_InterfaceStrDescriptor(mut speed:
                                                             USBD_SpeedTypeDef,
                                                         mut length:
                                                             *mut uint16_t)
 -> *mut uint8_t {
    if speed as libc::c_uint == USBD_SPEED_HIGH as libc::c_int as libc::c_uint
       {
        USBD_GetString(b"VCP Interface\x00" as *const u8 as
                           *const libc::c_char as *mut uint8_t,
                       USBD_StrDesc.as_mut_ptr(), length);
    } else {
        USBD_GetString(b"VCP Interface\x00" as *const u8 as
                           *const libc::c_char as *mut uint8_t,
                       USBD_StrDesc.as_mut_ptr(), length);
    }
    return USBD_StrDesc.as_mut_ptr();
}
/* *
  * @brief  Create the serial number string descriptor
  * @param  None
  * @retval None
  */
unsafe extern "C" fn Get_SerialNum() {
    let mut deviceserial0: uint32_t = 0;
    let mut deviceserial1: uint32_t = 0;
    let mut deviceserial2: uint32_t = 0;
    deviceserial0 = *(0x1ff0f420 as libc::c_int as *mut uint32_t);
    deviceserial1 = *(0x1ff0f424 as libc::c_int as *mut uint32_t);
    deviceserial2 = *(0x1ff0f428 as libc::c_int as *mut uint32_t);
    deviceserial0 =
        (deviceserial0 as libc::c_uint).wrapping_add(deviceserial2) as
            uint32_t as uint32_t;
    if deviceserial0 != 0 as libc::c_int as libc::c_uint {
        IntToUnicode(deviceserial0,
                     &mut *USBD_StringSerial.as_mut_ptr().offset(2 as
                                                                     libc::c_int
                                                                     as
                                                                     isize),
                     8 as libc::c_int as uint8_t);
        IntToUnicode(deviceserial1,
                     &mut *USBD_StringSerial.as_mut_ptr().offset(18 as
                                                                     libc::c_int
                                                                     as
                                                                     isize),
                     4 as libc::c_int as uint8_t);
    };
}
/* Private functions ---------------------------------------------------------*/
/* *
  * @brief  Convert Hex 32Bits value into char
  * @param  value: value to convert
  * @param  pbuf: pointer to the buffer
  * @param  len: buffer length
  * @retval None
  */
unsafe extern "C" fn IntToUnicode(mut value: uint32_t, mut pbuf: *mut uint8_t,
                                  mut len: uint8_t) {
    let mut idx: uint8_t = 0 as libc::c_int as uint8_t;
    idx = 0 as libc::c_int as uint8_t;
    while (idx as libc::c_int) < len as libc::c_int {
        if (value >> 28 as libc::c_int) < 0xa as libc::c_int as libc::c_uint {
            *pbuf.offset((2 as libc::c_int * idx as libc::c_int) as isize) =
                (value >>
                     28 as
                         libc::c_int).wrapping_add('0' as i32 as libc::c_uint)
                    as uint8_t
        } else {
            *pbuf.offset((2 as libc::c_int * idx as libc::c_int) as isize) =
                (value >>
                     28 as
                         libc::c_int).wrapping_add('A' as i32 as
                                                       libc::c_uint).wrapping_sub(10
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                    as uint8_t
        }
        value = value << 4 as libc::c_int;
        *pbuf.offset((2 as libc::c_int * idx as libc::c_int +
                          1 as libc::c_int) as isize) =
            0 as libc::c_int as uint8_t;
        idx = idx.wrapping_add(1)
    };
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
