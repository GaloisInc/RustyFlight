use ::libc;
extern "C" {
    /* *
  ******************************************************************************
  * @file    usb_init.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Initialization routines & global variables
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
    /* External variables --------------------------------------------------------*/
/*  The number of current endpoint, it will be used to specify an endpoint */
    /*  The number of current device, it is an index to the Device_Table */
/*extern uint8_t	Device_no; */
/*  Points to the DEVICE_INFO structure of current device */
/*  The purpose of this register is to speed up the execution */
    /*  Points to the DEVICE_PROP structure of current device */
/*  The purpose of this register is to speed up the execution */
    /*  Temporary save the state of Rx & Tx status. */
/*  Whenever the Rx or Tx state is changed, its value is saved */
/*  in this variable first and will be set to the EPRB or EPRA */
/*  at the end of interrupt process */
    #[no_mangle]
    static mut wInterrupt_Mask: uint16_t;
    #[no_mangle]
    fn Leave_LowPowerMode();
    #[no_mangle]
    fn USB_Cable_Config(NewState: FunctionalState);
}
/* *
  ******************************************************************************
  * @file    usb_type.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Type definitions used by the USB Library
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
pub type boolean = libc::c_uint;
pub const TRUE: boolean = 1;
pub const FALSE: boolean = 0;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SCB_Type {
    pub CPUID: uint32_t,
    pub ICSR: uint32_t,
    pub VTOR: uint32_t,
    pub AIRCR: uint32_t,
    pub SCR: uint32_t,
    pub CCR: uint32_t,
    pub SHP: [uint8_t; 12],
    pub SHCSR: uint32_t,
    pub CFSR: uint32_t,
    pub HFSR: uint32_t,
    pub DFSR: uint32_t,
    pub MMFAR: uint32_t,
    pub BFAR: uint32_t,
    pub AFSR: uint32_t,
    pub PFR: [uint32_t; 2],
    pub DFR: uint32_t,
    pub ADR: uint32_t,
    pub MMFR: [uint32_t; 4],
    pub ISAR: [uint32_t; 5],
    pub RESERVED0: [uint32_t; 5],
    pub CPACR: uint32_t,
}
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PWR_TypeDef {
    pub CR: uint32_t,
    pub CSR: uint32_t,
}
pub type _RESULT = libc::c_uint;
pub const USB_NOT_READY: _RESULT = 3;
pub const USB_UNSUPPORT: _RESULT = 2;
pub const USB_ERROR: _RESULT = 1;
pub const USB_SUCCESS: _RESULT = 0;
pub type RESULT = _RESULT;
pub type _RESUME_STATE = libc::c_uint;
pub const RESUME_ESOF: _RESUME_STATE = 7;
pub const RESUME_OFF: _RESUME_STATE = 6;
pub const RESUME_ON: _RESUME_STATE = 5;
pub const RESUME_START: _RESUME_STATE = 4;
pub const RESUME_WAIT: _RESUME_STATE = 3;
pub const RESUME_LATER: _RESUME_STATE = 2;
pub const RESUME_INTERNAL: _RESUME_STATE = 1;
pub const RESUME_EXTERNAL: _RESUME_STATE = 0;
/* *
 ******************************************************************************
 * @file    usb_pwr.h
 * @author  MCD Application Team
 * @version V4.0.0
 * @date    21-January-2013
 * @brief   Connection/disconnection & power management header
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
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
pub type RESUME_STATE = _RESUME_STATE;
pub type _DEVICE_STATE = libc::c_uint;
pub const CONFIGURED: _DEVICE_STATE = 5;
pub const ADDRESSED: _DEVICE_STATE = 4;
pub const SUSPENDED: _DEVICE_STATE = 3;
pub const POWERED: _DEVICE_STATE = 2;
pub const ATTACHED: _DEVICE_STATE = 1;
pub const UNCONNECTED: _DEVICE_STATE = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed {
    pub eState: RESUME_STATE,
    pub bESOFcnt: uint8_t,
}
/* *
 ******************************************************************************
 * @file    usb_pwr.c
 * @author  MCD Application Team
 * @version V4.0.0
 * @date    21-January-2013
 * @brief   Connection/disconnection & power management
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
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
pub static mut bDeviceState: uint32_t =
    UNCONNECTED as libc::c_int as uint32_t;
/* USB device status */
#[no_mangle]
pub static mut fSuspendEnabled: boolean = TRUE;
/* true when suspend is possible */  // HJI
#[no_mangle]
pub static mut EP: [uint32_t; 8] = [0; 8];
#[no_mangle]
pub static mut ResumeS: C2RustUnnamed =
    C2RustUnnamed{eState: RESUME_EXTERNAL, bESOFcnt: 0,};
#[no_mangle]
pub static mut remotewakeupon: uint32_t = 0 as libc::c_int as uint32_t;
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function prototypes ------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* ******************************************************************************
 * Function Name  : PowerOn
 * Description    :
 * Input          : None.
 * Output         : None.
 * Return         : USB_SUCCESS.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn PowerOn() -> RESULT {
    let mut wRegVal: uint16_t = 0;
    /* ** cable plugged-in ? ***/
    USB_Cable_Config(ENABLE);
    /* ** CNTR_PWDN = 0 ***/
    wRegVal = 0x1 as libc::c_int as uint16_t;
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                     0x40 as libc::c_int as libc::c_long) as
                                    *mut libc::c_uint,
                                wRegVal as libc::c_uint);
    /* ** CNTR_FRES = 0 ***/
    wInterrupt_Mask = 0 as libc::c_int as uint16_t;
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                     0x40 as libc::c_int as libc::c_long) as
                                    *mut libc::c_uint,
                                wInterrupt_Mask as libc::c_uint);
    /* ** Clear pending interrupts ***/
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                     0x44 as libc::c_int as libc::c_long) as
                                    *mut libc::c_uint,
                                0 as libc::c_int as uint16_t as libc::c_uint);
    /* ** Set interrupt mask ***/
    wInterrupt_Mask =
        (0x400 as libc::c_int | 0x800 as libc::c_int | 0x1000 as libc::c_int)
            as uint16_t;
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                     0x40 as libc::c_int as libc::c_long) as
                                    *mut libc::c_uint,
                                wInterrupt_Mask as libc::c_uint);
    return USB_SUCCESS;
}
/* ******************************************************************************
 * Function Name  : PowerOff
 * Description    : handles switch-off conditions
 * Input          : None.
 * Output         : None.
 * Return         : USB_SUCCESS.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn PowerOff() -> RESULT {
    /* disable all interrupts and force USB reset */
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                     0x40 as libc::c_int as libc::c_long) as
                                    *mut libc::c_uint,
                                0x1 as libc::c_int as uint16_t as
                                    libc::c_uint);
    /* clear interrupt status register */
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                     0x44 as libc::c_int as libc::c_long) as
                                    *mut libc::c_uint,
                                0 as libc::c_int as uint16_t as libc::c_uint);
    /* Disable the Pull-Up*/
    USB_Cable_Config(DISABLE);
    /* switch-off device */
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                     0x40 as libc::c_int as libc::c_long) as
                                    *mut libc::c_uint,
                                (0x1 as libc::c_int as uint16_t as libc::c_int
                                     + 0x2 as libc::c_int) as libc::c_uint);
    /* sw variables reset */
    /* ... */
    return USB_SUCCESS;
}
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* ******************************************************************************
 * Function Name  : Suspend
 * Description    : sets suspend mode operating conditions
 * Input          : None.
 * Output         : None.
 * Return         : USB_SUCCESS.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Suspend() {
    let mut i: uint32_t = 0 as libc::c_int as uint32_t;
    let mut wCNTR: uint16_t = 0;
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    let mut savePWR_CR: uint32_t = 0 as libc::c_int as uint32_t;
    /* suspend preparation */
    /* ... */
    /*Store CNTR value */
    wCNTR =
        *((0x40005c00 as libc::c_long + 0x40 as libc::c_int as libc::c_long)
              as *mut libc::c_uint) as uint16_t;
    /* This a sequence to apply a force RESET to handle a robustness case */
    /*Store endpoints registers status */
    i = 0 as libc::c_int as uint32_t;
    while i < 8 as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut EP[i as usize] as *mut uint32_t,
                                    *(0x40005c00 as libc::c_long as
                                          *mut libc::c_uint).offset(i as
                                                                        isize)
                                        as uint16_t as uint32_t);
        i = i.wrapping_add(1)
    }
    /* unmask RESET flag */
    wCNTR = (wCNTR as libc::c_int | 0x400 as libc::c_int) as uint16_t;
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                     0x40 as libc::c_int as libc::c_long) as
                                    *mut libc::c_uint, wCNTR as libc::c_uint);
    /*apply FRES */
    wCNTR = (wCNTR as libc::c_int | 0x1 as libc::c_int) as uint16_t;
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                     0x40 as libc::c_int as libc::c_long) as
                                    *mut libc::c_uint, wCNTR as libc::c_uint);
    /*clear FRES*/
    wCNTR = (wCNTR as libc::c_int & !(0x1 as libc::c_int)) as uint16_t;
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                     0x40 as libc::c_int as libc::c_long) as
                                    *mut libc::c_uint, wCNTR as libc::c_uint);
    /*poll for RESET flag in ISTR*/
    while *((0x40005c00 as libc::c_long + 0x44 as libc::c_int as libc::c_long)
                as *mut libc::c_uint) as uint16_t as libc::c_int &
              0x400 as libc::c_int == 0 as libc::c_int {
    }
    /* clear RESET flag in ISTR */
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                     0x44 as libc::c_int as libc::c_long) as
                                    *mut libc::c_uint,
                                !(0x400 as libc::c_int) as uint16_t as
                                    libc::c_uint);
    /*restore Enpoints*/
    i = 0 as libc::c_int as uint32_t;
    while i < 8 as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile((0x40005c00 as libc::c_long as
                                         *mut libc::c_uint).offset(i as
                                                                       isize),
                                    EP[i as usize] as uint16_t as
                                        libc::c_uint);
        i = i.wrapping_add(1)
    }
    /* Now it is safe to enter macrocell in suspend mode */
    wCNTR = (wCNTR as libc::c_int | 0x8 as libc::c_int) as uint16_t;
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                     0x40 as libc::c_int as libc::c_long) as
                                    *mut libc::c_uint, wCNTR as libc::c_uint);
    /* force low-power mode in the macrocell */
    wCNTR =
        *((0x40005c00 as libc::c_long + 0x40 as libc::c_int as libc::c_long)
              as *mut libc::c_uint) as uint16_t;
    wCNTR = (wCNTR as libc::c_int | 0x4 as libc::c_int) as uint16_t;
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                     0x40 as libc::c_int as libc::c_long) as
                                    *mut libc::c_uint, wCNTR as libc::c_uint);
    /*prepare entry in low power mode (STOP mode)*/
    /* Select the regulator state in STOP mode*/
    ::core::ptr::write_volatile(&mut savePWR_CR as *mut uint32_t,
                                (*((0x40000000 as libc::c_int as
                                        uint32_t).wrapping_add(0x7000 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_uint)
                                       as *mut PWR_TypeDef)).CR);
    tmpreg =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x7000 as libc::c_int as libc::c_uint)
               as *mut PWR_TypeDef)).CR;
    /* Clear PDDS and LPDS bits */
    tmpreg &= 0xfffffffc as libc::c_uint;
    /* Set LPDS bit according to PWR_Regulator value */
    tmpreg |= 0x1 as libc::c_int as uint32_t;
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x7000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut PWR_TypeDef)).CR as
                                    *mut uint32_t, tmpreg);
    /* Set SLEEPDEEP bit of Cortex System Control Register */
    let ref mut fresh0 =
        (*((0xe000e000 as libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong)
               as *mut SCB_Type)).SCR;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x4 as libc::c_int as uint8_t as
                                         libc::c_uint) as uint32_t as
                                    uint32_t);
    /* enter system in STOP mode, only when wakeup flag in not set */
    if *((0x40005c00 as libc::c_long + 0x44 as libc::c_int as libc::c_long) as
             *mut libc::c_uint) as uint16_t as libc::c_int &
           0x1000 as libc::c_int == 0 as libc::c_int {
        asm!("wfi" : : : : "volatile");
        /* Reset SLEEPDEEP bit of Cortex System Control Register */
        let ref mut fresh1 =
            (*((0xe000e000 as
                    libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong) as
                   *mut SCB_Type)).SCR;
        ::core::ptr::write_volatile(fresh1,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x4 as libc::c_int as uint8_t as
                                               uint32_t)) as uint32_t as
                                        uint32_t)
    } else {
        /* Clear Wakeup flag */
        ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                         0x44 as libc::c_int as libc::c_long)
                                        as *mut libc::c_uint,
                                    !(0x1000 as libc::c_int) as uint16_t as
                                        libc::c_uint);
        /* clear FSUSP to abort entry in suspend mode  */
        wCNTR =
            *((0x40005c00 as libc::c_long +
                   0x40 as libc::c_int as libc::c_long) as *mut libc::c_uint)
                as uint16_t;
        wCNTR = (wCNTR as libc::c_int & !(0x8 as libc::c_int)) as uint16_t;
        ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                         0x40 as libc::c_int as libc::c_long)
                                        as *mut libc::c_uint,
                                    wCNTR as libc::c_uint);
        /*restore sleep mode configuration */
        /* restore Power regulator config in sleep mode*/
        ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x7000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint)
                                                as *mut PWR_TypeDef)).CR as
                                        *mut uint32_t, savePWR_CR);
        /* Reset SLEEPDEEP bit of Cortex System Control Register */
        let ref mut fresh2 =
            (*((0xe000e000 as
                    libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong) as
                   *mut SCB_Type)).SCR;
        ::core::ptr::write_volatile(fresh2,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x4 as libc::c_int as uint8_t as
                                               uint32_t)) as uint32_t as
                                        uint32_t)
    };
}
/* ******************************************************************************
 * Function Name  : Resume_Init
 * Description    : Handles wake-up restoring normal operations
 * Input          : None.
 * Output         : None.
 * Return         : USB_SUCCESS.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Resume_Init() {
    let mut wCNTR: uint16_t = 0;
    /* ------------------ ONLY WITH BUS-POWERED DEVICES ---------------------- */
    /* restart the clocks */
    /* ...  */
    /* CNTR_LPMODE = 0 */
    wCNTR =
        *((0x40005c00 as libc::c_long + 0x40 as libc::c_int as libc::c_long)
              as *mut libc::c_uint) as uint16_t;
    wCNTR = (wCNTR as libc::c_int & !(0x4 as libc::c_int)) as uint16_t;
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                     0x40 as libc::c_int as libc::c_long) as
                                    *mut libc::c_uint, wCNTR as libc::c_uint);
    /* restore full power */
    /* ... on connected devices */
    Leave_LowPowerMode();
    /* reset FSUSP bit */
    ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                     0x40 as libc::c_int as libc::c_long) as
                                    *mut libc::c_uint,
                                (0x8000 as libc::c_int | 0x1000 as libc::c_int
                                     | 0x200 as libc::c_int |
                                     0x400 as libc::c_int) as uint16_t as
                                    libc::c_uint);
    /* reverse suspend preparation */
    /* ... */
}
/* ******************************************************************************
 * Function Name  : Resume
 * Description    : This is the state machine handling resume operations and
 *                 timing sequence. The control is based on the Resume structure
 *                 variables and on the ESOF interrupt calling this subroutine
 *                 without changing machine state.
 * Input          : a state machine value (RESUME_STATE)
 *                  RESUME_ESOF doesn't change ResumeS.eState allowing
 *                  decrementing of the ESOF counter in different states.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn Resume(mut eResumeSetVal: RESUME_STATE) {
    let mut wCNTR: uint16_t =
        0; /* RESUME detected during the RemoteWAkeup signalling => keep RemoteWakeup handling*/
    if eResumeSetVal as libc::c_uint !=
           RESUME_ESOF as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut ResumeS.eState as *mut RESUME_STATE,
                                    eResumeSetVal)
    }
    match ResumeS.eState as libc::c_uint {
        0 => {
            if remotewakeupon == 0 as libc::c_int as libc::c_uint {
                Resume_Init();
                ::core::ptr::write_volatile(&mut ResumeS.eState as
                                                *mut RESUME_STATE, RESUME_OFF)
            } else {
                ::core::ptr::write_volatile(&mut ResumeS.eState as
                                                *mut RESUME_STATE, RESUME_ON)
            }
        }
        1 => {
            Resume_Init();
            ::core::ptr::write_volatile(&mut ResumeS.eState as
                                            *mut RESUME_STATE, RESUME_START);
            ::core::ptr::write_volatile(&mut remotewakeupon as *mut uint32_t,
                                        1 as libc::c_int as uint32_t)
        }
        2 => {
            ::core::ptr::write_volatile(&mut ResumeS.bESOFcnt as *mut uint8_t,
                                        2 as libc::c_int as uint8_t);
            ::core::ptr::write_volatile(&mut ResumeS.eState as
                                            *mut RESUME_STATE, RESUME_WAIT)
        }
        3 => {
            ::core::ptr::write_volatile(&mut ResumeS.bESOFcnt as *mut uint8_t,
                                        ::core::ptr::read_volatile::<uint8_t>(&ResumeS.bESOFcnt
                                                                                  as
                                                                                  *const uint8_t).wrapping_sub(1));
            if ResumeS.bESOFcnt as libc::c_int == 0 as libc::c_int {
                ::core::ptr::write_volatile(&mut ResumeS.eState as
                                                *mut RESUME_STATE,
                                            RESUME_START)
            }
        }
        4 => {
            wCNTR =
                *((0x40005c00 as libc::c_long +
                       0x40 as libc::c_int as libc::c_long) as
                      *mut libc::c_uint) as uint16_t;
            wCNTR = (wCNTR as libc::c_int | 0x10 as libc::c_int) as uint16_t;
            ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                             0x40 as libc::c_int as
                                                 libc::c_long) as
                                            *mut libc::c_uint,
                                        wCNTR as libc::c_uint);
            ::core::ptr::write_volatile(&mut ResumeS.eState as
                                            *mut RESUME_STATE, RESUME_ON);
            ::core::ptr::write_volatile(&mut ResumeS.bESOFcnt as *mut uint8_t,
                                        10 as libc::c_int as uint8_t)
        }
        5 => {
            ::core::ptr::write_volatile(&mut ResumeS.bESOFcnt as *mut uint8_t,
                                        ::core::ptr::read_volatile::<uint8_t>(&ResumeS.bESOFcnt
                                                                                  as
                                                                                  *const uint8_t).wrapping_sub(1));
            if ResumeS.bESOFcnt as libc::c_int == 0 as libc::c_int {
                wCNTR =
                    *((0x40005c00 as libc::c_long +
                           0x40 as libc::c_int as libc::c_long) as
                          *mut libc::c_uint) as uint16_t;
                wCNTR =
                    (wCNTR as libc::c_int & !(0x10 as libc::c_int)) as
                        uint16_t;
                ::core::ptr::write_volatile((0x40005c00 as libc::c_long +
                                                 0x40 as libc::c_int as
                                                     libc::c_long) as
                                                *mut libc::c_uint,
                                            wCNTR as libc::c_uint);
                ::core::ptr::write_volatile(&mut ResumeS.eState as
                                                *mut RESUME_STATE,
                                            RESUME_OFF);
                ::core::ptr::write_volatile(&mut remotewakeupon as
                                                *mut uint32_t,
                                            0 as libc::c_int as uint32_t)
            }
        }
        6 | 7 | _ => {
            ::core::ptr::write_volatile(&mut ResumeS.eState as
                                            *mut RESUME_STATE, RESUME_OFF)
        }
    };
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
