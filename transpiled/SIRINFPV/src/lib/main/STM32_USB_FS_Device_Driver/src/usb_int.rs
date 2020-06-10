use ::libc;
extern "C" {
    #[no_mangle]
    static mut wIstr: uint16_t;
    /* Exported constants --------------------------------------------------------*/
    /* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
    #[no_mangle]
    fn Setup0_Process() -> uint8_t;
    #[no_mangle]
    fn Out0_Process() -> uint8_t;
    #[no_mangle]
    fn In0_Process() -> uint8_t;
    /* External variables --------------------------------------------------------*/
/*  The number of current endpoint, it will be used to specify an endpoint */
    #[no_mangle]
    static mut EPindex: uint8_t;
    /* Extern variables ----------------------------------------------------------*/
    #[no_mangle]
    static mut pEpInt_IN: [Option<unsafe extern "C" fn() -> ()>; 7];
    /*  Handles IN  interrupts   */
    #[no_mangle]
    static mut pEpInt_OUT: [Option<unsafe extern "C" fn() -> ()>; 7];
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* *
  ******************************************************************************
  * @file    usb_int.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Endpoint CTR (Low and High) interrupt's service routines
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#[no_mangle]
pub static mut SaveRState: uint16_t = 0;
#[no_mangle]
pub static mut SaveTState: uint16_t = 0;
/*  Handles OUT interrupts   */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* ******************************************************************************
* Function Name  : CTR_LP.
* Description    : Low priority Endpoint Correct Transfer interrupt's service
*                  routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn CTR_LP() {
    let mut wEPVal: uint16_t = 0 as libc::c_int as uint16_t;
    loop 
         /* stay in loop while pending interrupts */
         {
        ::core::ptr::write_volatile(&mut wIstr as *mut uint16_t,
                                    *((0x40005c00 as libc::c_long +
                                           0x44 as libc::c_int as
                                               libc::c_long) as
                                          *mut libc::c_uint) as uint16_t);
        if !(::core::ptr::read_volatile::<uint16_t>(&wIstr as *const uint16_t)
                 as libc::c_int & 0x8000 as libc::c_int != 0 as libc::c_int) {
            break ;
        }
        /* extract highest priority endpoint number */
        EPindex = (wIstr as libc::c_int & 0xf as libc::c_int) as uint8_t;
        if EPindex as libc::c_int == 0 as libc::c_int {
            /* if(EPindex == 0) else */
            ::core::ptr::write_volatile(&mut SaveRState as *mut uint16_t,
                                        *(0x40005c00 as libc::c_long as
                                              *mut libc::c_uint).offset(0 as
                                                                            libc::c_int
                                                                            as
                                                                            uint8_t
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            isize)
                                            as
                                            uint16_t); /* if(EPindex == 0) */
            ::core::ptr::write_volatile(&mut SaveTState as *mut uint16_t,
                                        (SaveRState as libc::c_int &
                                             0x30 as libc::c_int) as
                                            uint16_t);
            ::core::ptr::write_volatile(&mut SaveRState as *mut uint16_t,
                                        (::core::ptr::read_volatile::<uint16_t>(&SaveRState
                                                                                    as
                                                                                    *const uint16_t)
                                             as libc::c_int &
                                             0x3000 as libc::c_int) as
                                            uint16_t as uint16_t);
            let mut _wRegVal: uint32_t = 0;
            _wRegVal =
                (*(0x40005c00 as libc::c_long as
                       *mut libc::c_uint).offset(0 as libc::c_int as uint8_t
                                                     as libc::c_int as isize)
                     as uint16_t as libc::c_int &
                     (0x3000 as libc::c_int |
                          (0x8000 as libc::c_int | 0x800 as libc::c_int |
                               0x600 as libc::c_int | 0x100 as libc::c_int |
                               0x80 as libc::c_int | 0xf as libc::c_int) |
                          0x30 as libc::c_int)) as uint32_t;
            if 0x1000 as libc::c_int & 0x2000 as libc::c_int !=
                   0 as libc::c_int {
                _wRegVal ^= 0x1000 as libc::c_int as libc::c_uint
            }
            if 0x2000 as libc::c_int & 0x2000 as libc::c_int !=
                   0 as libc::c_int {
                _wRegVal ^= 0x2000 as libc::c_int as libc::c_uint
            }
            if 0x10 as libc::c_int & 0x20 as libc::c_int != 0 as libc::c_int {
                _wRegVal ^= 0x10 as libc::c_int as libc::c_uint
            }
            if 0x20 as libc::c_int & 0x20 as libc::c_int != 0 as libc::c_int {
                _wRegVal ^= 0x20 as libc::c_int as libc::c_uint
            }
            ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                             *mut libc::c_uint).offset(0 as
                                                                           libc::c_int
                                                                           as
                                                                           uint8_t
                                                                           as
                                                                           libc::c_int
                                                                           as
                                                                           isize),
                                        (_wRegVal as uint16_t as libc::c_int |
                                             0x8000 as libc::c_int |
                                             0x80 as libc::c_int) as
                                            libc::c_uint);
            /* Decode and service control endpoint interrupt */
      /* calling related service routine */
      /* (Setup0_Process, In0_Process, Out0_Process) */
            /* save RX & TX status */
      /* and set both to NAK */
            /* DIR bit = origin of the interrupt */
            if wIstr as libc::c_int & 0x10 as libc::c_int == 0 as libc::c_int
               {
                /* DIR = 0 */
                /* DIR = 0      => IN  int */
        /* DIR = 0 implies that (EP_CTR_TX = 1) always  */
                ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                                 *mut libc::c_uint).offset(0
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               uint8_t
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               isize),
                                            (*(0x40005c00 as libc::c_long as
                                                   *mut libc::c_uint).offset(0
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 uint8_t
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 isize)
                                                 as uint16_t as libc::c_int &
                                                 0xff7f as libc::c_int &
                                                 (0x8000 as libc::c_int |
                                                      0x800 as libc::c_int |
                                                      0x600 as libc::c_int |
                                                      0x100 as libc::c_int |
                                                      0x80 as libc::c_int |
                                                      0xf as libc::c_int)) as
                                                libc::c_uint);
                In0_Process();
                /* before terminate set Tx & Rx status */
                let mut _wRegVal_0: uint32_t = 0;
                _wRegVal_0 =
                    (*(0x40005c00 as libc::c_long as
                           *mut libc::c_uint).offset(0 as libc::c_int as
                                                         uint8_t as
                                                         libc::c_int as isize)
                         as uint16_t as libc::c_int &
                         (0x3000 as libc::c_int |
                              (0x8000 as libc::c_int | 0x800 as libc::c_int |
                                   0x600 as libc::c_int | 0x100 as libc::c_int
                                   | 0x80 as libc::c_int | 0xf as libc::c_int)
                              | 0x30 as libc::c_int)) as uint32_t;
                if 0x1000 as libc::c_int & SaveRState as libc::c_int !=
                       0 as libc::c_int {
                    _wRegVal_0 ^= 0x1000 as libc::c_int as libc::c_uint
                }
                if 0x2000 as libc::c_int & SaveRState as libc::c_int !=
                       0 as libc::c_int {
                    _wRegVal_0 ^= 0x2000 as libc::c_int as libc::c_uint
                }
                if 0x10 as libc::c_int & SaveTState as libc::c_int !=
                       0 as libc::c_int {
                    _wRegVal_0 ^= 0x10 as libc::c_int as libc::c_uint
                }
                if 0x20 as libc::c_int & SaveTState as libc::c_int !=
                       0 as libc::c_int {
                    _wRegVal_0 ^= 0x20 as libc::c_int as libc::c_uint
                }
                ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                                 *mut libc::c_uint).offset(0
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               uint8_t
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               isize),
                                            (_wRegVal_0 as uint16_t as
                                                 libc::c_int |
                                                 0x8000 as libc::c_int |
                                                 0x80 as libc::c_int) as
                                                libc::c_uint);
                return
            } else {
                /* DIR = 1 */
                /* DIR = 1 & CTR_RX       => SETUP or OUT int */
        /* DIR = 1 & (CTR_TX | CTR_RX) => 2 int pending */
                ::core::ptr::write_volatile(&mut wEPVal as *mut uint16_t,
                                            *(0x40005c00 as libc::c_long as
                                                  *mut libc::c_uint).offset(0
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                uint8_t
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                isize)
                                                as
                                                uint16_t); /* SETUP bit kept frozen while CTR_RX = 1 */
                if wEPVal as libc::c_int & 0x800 as libc::c_int !=
                       0 as libc::c_int {
                    ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                                     *mut libc::c_uint).offset(0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   uint8_t
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   isize),
                                                (*(0x40005c00 as libc::c_long
                                                       as
                                                       *mut libc::c_uint).offset(0
                                                                                     as
                                                                                     libc::c_int
                                                                                     as
                                                                                     uint8_t
                                                                                     as
                                                                                     libc::c_int
                                                                                     as
                                                                                     isize)
                                                     as uint16_t as
                                                     libc::c_int &
                                                     0x7fff as libc::c_int &
                                                     (0x8000 as libc::c_int |
                                                          0x800 as libc::c_int
                                                          |
                                                          0x600 as libc::c_int
                                                          |
                                                          0x100 as libc::c_int
                                                          |
                                                          0x80 as libc::c_int
                                                          |
                                                          0xf as libc::c_int))
                                                    as libc::c_uint);
                    Setup0_Process();
                    /* before terminate set Tx & Rx status */
                    let mut _wRegVal_1: uint32_t = 0;
                    _wRegVal_1 =
                        (*(0x40005c00 as libc::c_long as
                               *mut libc::c_uint).offset(0 as libc::c_int as
                                                             uint8_t as
                                                             libc::c_int as
                                                             isize) as
                             uint16_t as libc::c_int &
                             (0x3000 as libc::c_int |
                                  (0x8000 as libc::c_int |
                                       0x800 as libc::c_int |
                                       0x600 as libc::c_int |
                                       0x100 as libc::c_int |
                                       0x80 as libc::c_int |
                                       0xf as libc::c_int) |
                                  0x30 as libc::c_int)) as uint32_t;
                    if 0x1000 as libc::c_int & SaveRState as libc::c_int !=
                           0 as libc::c_int {
                        _wRegVal_1 ^= 0x1000 as libc::c_int as libc::c_uint
                    }
                    if 0x2000 as libc::c_int & SaveRState as libc::c_int !=
                           0 as libc::c_int {
                        _wRegVal_1 ^= 0x2000 as libc::c_int as libc::c_uint
                    }
                    if 0x10 as libc::c_int & SaveTState as libc::c_int !=
                           0 as libc::c_int {
                        _wRegVal_1 ^= 0x10 as libc::c_int as libc::c_uint
                    }
                    if 0x20 as libc::c_int & SaveTState as libc::c_int !=
                           0 as libc::c_int {
                        _wRegVal_1 ^= 0x20 as libc::c_int as libc::c_uint
                    }
                    ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                                     *mut libc::c_uint).offset(0
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   uint8_t
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   isize),
                                                (_wRegVal_1 as uint16_t as
                                                     libc::c_int |
                                                     0x8000 as libc::c_int |
                                                     0x80 as libc::c_int) as
                                                    libc::c_uint);
                    return
                } else {
                    if wEPVal as libc::c_int & 0x8000 as libc::c_int !=
                           0 as libc::c_int {
                        ::core::ptr::write_volatile((0x40005c00 as
                                                         libc::c_long as
                                                         *mut libc::c_uint).offset(0
                                                                                       as
                                                                                       libc::c_int
                                                                                       as
                                                                                       uint8_t
                                                                                       as
                                                                                       libc::c_int
                                                                                       as
                                                                                       isize),
                                                    (*(0x40005c00 as
                                                           libc::c_long as
                                                           *mut libc::c_uint).offset(0
                                                                                         as
                                                                                         libc::c_int
                                                                                         as
                                                                                         uint8_t
                                                                                         as
                                                                                         libc::c_int
                                                                                         as
                                                                                         isize)
                                                         as uint16_t as
                                                         libc::c_int &
                                                         0x7fff as libc::c_int
                                                         &
                                                         (0x8000 as
                                                              libc::c_int |
                                                              0x800 as
                                                                  libc::c_int
                                                              |
                                                              0x600 as
                                                                  libc::c_int
                                                              |
                                                              0x100 as
                                                                  libc::c_int
                                                              |
                                                              0x80 as
                                                                  libc::c_int
                                                              |
                                                              0xf as
                                                                  libc::c_int))
                                                        as libc::c_uint);
                        Out0_Process();
                        /* before terminate set Tx & Rx status */
                        let mut _wRegVal_2: uint32_t = 0;
                        _wRegVal_2 =
                            (*(0x40005c00 as libc::c_long as
                                   *mut libc::c_uint).offset(0 as libc::c_int
                                                                 as uint8_t as
                                                                 libc::c_int
                                                                 as isize) as
                                 uint16_t as libc::c_int &
                                 (0x3000 as libc::c_int |
                                      (0x8000 as libc::c_int |
                                           0x800 as libc::c_int |
                                           0x600 as libc::c_int |
                                           0x100 as libc::c_int |
                                           0x80 as libc::c_int |
                                           0xf as libc::c_int) |
                                      0x30 as libc::c_int)) as uint32_t;
                        if 0x1000 as libc::c_int & SaveRState as libc::c_int
                               != 0 as libc::c_int {
                            _wRegVal_2 ^=
                                0x1000 as libc::c_int as libc::c_uint
                        }
                        if 0x2000 as libc::c_int & SaveRState as libc::c_int
                               != 0 as libc::c_int {
                            _wRegVal_2 ^=
                                0x2000 as libc::c_int as libc::c_uint
                        }
                        if 0x10 as libc::c_int & SaveTState as libc::c_int !=
                               0 as libc::c_int {
                            _wRegVal_2 ^= 0x10 as libc::c_int as libc::c_uint
                        }
                        if 0x20 as libc::c_int & SaveTState as libc::c_int !=
                               0 as libc::c_int {
                            _wRegVal_2 ^= 0x20 as libc::c_int as libc::c_uint
                        }
                        ::core::ptr::write_volatile((0x40005c00 as
                                                         libc::c_long as
                                                         *mut libc::c_uint).offset(0
                                                                                       as
                                                                                       libc::c_int
                                                                                       as
                                                                                       uint8_t
                                                                                       as
                                                                                       libc::c_int
                                                                                       as
                                                                                       isize),
                                                    (_wRegVal_2 as uint16_t as
                                                         libc::c_int |
                                                         0x8000 as libc::c_int
                                                         |
                                                         0x80 as libc::c_int)
                                                        as libc::c_uint);
                        return
                    }
                }
            }
        } else {
            /* Decode and service non control endpoints interrupt  */
            /* process related endpoint register */
            ::core::ptr::write_volatile(&mut wEPVal as *mut uint16_t,
                                        *(0x40005c00 as libc::c_long as
                                              *mut libc::c_uint).offset(EPindex
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            isize)
                                            as uint16_t);
            /* if((wEPVal & EP_CTR_TX) != 0) */
            if wEPVal as libc::c_int & 0x8000 as libc::c_int !=
                   0 as libc::c_int { /* if((wEPVal & EP_CTR_RX) */
                /* clear int flag */
                ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                                 *mut libc::c_uint).offset(EPindex
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               isize),
                                            (*(0x40005c00 as libc::c_long as
                                                   *mut libc::c_uint).offset(EPindex
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 isize)
                                                 as uint16_t as libc::c_int &
                                                 0x7fff as libc::c_int &
                                                 (0x8000 as libc::c_int |
                                                      0x800 as libc::c_int |
                                                      0x600 as libc::c_int |
                                                      0x100 as libc::c_int |
                                                      0x80 as libc::c_int |
                                                      0xf as libc::c_int)) as
                                                libc::c_uint);
                /* call OUT service function */
                Some((*pEpInt_OUT.as_mut_ptr().offset((EPindex as libc::c_int
                                                           - 1 as libc::c_int)
                                                          as
                                                          isize)).expect("non-null function pointer")).expect("non-null function pointer")();
            }
            if wEPVal as libc::c_int & 0x80 as libc::c_int != 0 as libc::c_int
               {
                /* clear int flag */
                ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                                 *mut libc::c_uint).offset(EPindex
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               isize),
                                            (*(0x40005c00 as libc::c_long as
                                                   *mut libc::c_uint).offset(EPindex
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 isize)
                                                 as uint16_t as libc::c_int &
                                                 0xff7f as libc::c_int &
                                                 (0x8000 as libc::c_int |
                                                      0x800 as libc::c_int |
                                                      0x600 as libc::c_int |
                                                      0x100 as libc::c_int |
                                                      0x80 as libc::c_int |
                                                      0xf as libc::c_int)) as
                                                libc::c_uint);
                /* call IN service function */
                Some((*pEpInt_IN.as_mut_ptr().offset((EPindex as libc::c_int -
                                                          1 as libc::c_int) as
                                                         isize)).expect("non-null function pointer")).expect("non-null function pointer")();
            }
        }
    };
    /* while(...) */
}
/* *
  ******************************************************************************
  * @file    usb_int.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Endpoint CTR (Low and High) interrupt's service routines prototypes
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* ******************************************************************************
* Function Name  : CTR_HP.
* Description    : High Priority Endpoint Correct Transfer interrupt's service 
*                  routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn CTR_HP() {
    let mut wEPVal: uint32_t =
        0 as libc::c_int as uint32_t; /* clear CTR flag */
    loop  {
        ::core::ptr::write_volatile(&mut wIstr as *mut uint16_t,
                                    *((0x40005c00 as libc::c_long +
                                           0x44 as libc::c_int as
                                               libc::c_long) as
                                          *mut libc::c_uint) as uint16_t);
        if !(::core::ptr::read_volatile::<uint16_t>(&wIstr as *const uint16_t)
                 as libc::c_int & 0x8000 as libc::c_int != 0 as libc::c_int) {
            break ;
        }
        ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                         0x44 as libc::c_int as libc::c_long)
                                        as *mut libc::c_uint,
                                    !(0x8000 as libc::c_int) as uint16_t as
                                        libc::c_uint);
        /* if((wEPVal & EP_CTR_TX) != 0) */
        EPindex = (wIstr as libc::c_int & 0xf as libc::c_int) as uint8_t;
        wEPVal =
            *(0x40005c00 as libc::c_long as
                  *mut libc::c_uint).offset(EPindex as libc::c_int as isize)
                as uint16_t as uint32_t;
        if wEPVal & 0x8000 as libc::c_int as libc::c_uint !=
               0 as libc::c_int as libc::c_uint {
            /* extract highest priority endpoint number */
            /* process related endpoint register */
            ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                             *mut libc::c_uint).offset(EPindex
                                                                           as
                                                                           libc::c_int
                                                                           as
                                                                           isize),
                                        (*(0x40005c00 as libc::c_long as
                                               *mut libc::c_uint).offset(EPindex
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             isize)
                                             as uint16_t as libc::c_int &
                                             0x7fff as libc::c_int &
                                             (0x8000 as libc::c_int |
                                                  0x800 as libc::c_int |
                                                  0x600 as libc::c_int |
                                                  0x100 as libc::c_int |
                                                  0x80 as libc::c_int |
                                                  0xf as libc::c_int)) as
                                            libc::c_uint); /* if((wEPVal & EP_CTR_RX) */
            /* clear int flag */
            /* call OUT service function */
            Some((*pEpInt_OUT.as_mut_ptr().offset((EPindex as libc::c_int -
                                                       1 as libc::c_int) as
                                                      isize)).expect("non-null function pointer")).expect("non-null function pointer")();
        } else if wEPVal & 0x80 as libc::c_int as libc::c_uint !=
                      0 as libc::c_int as libc::c_uint {
            /* clear int flag */
            ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                             *mut libc::c_uint).offset(EPindex
                                                                           as
                                                                           libc::c_int
                                                                           as
                                                                           isize),
                                        (*(0x40005c00 as libc::c_long as
                                               *mut libc::c_uint).offset(EPindex
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             isize)
                                             as uint16_t as libc::c_int &
                                             0xff7f as libc::c_int &
                                             (0x8000 as libc::c_int |
                                                  0x800 as libc::c_int |
                                                  0x600 as libc::c_int |
                                                  0x100 as libc::c_int |
                                                  0x80 as libc::c_int |
                                                  0xf as libc::c_int)) as
                                            libc::c_uint);
            /* call IN service function */
            Some((*pEpInt_IN.as_mut_ptr().offset((EPindex as libc::c_int -
                                                      1 as libc::c_int) as
                                                     isize)).expect("non-null function pointer")).expect("non-null function pointer")();
        }
    };
    /* while(...) */
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
